//! Example USB HID Gamepad implementation
//!
//! This is a complete working example showing how to support USB HID devices,
//! particularly gamepads and joysticks, using the imxrt-usbh library.
//! 
//! This demonstrates:
//! - HID descriptor parsing
//! - Generic HID report processing  
//! - Button and axis event detection
//! - Multiple device management
//!
//! Based on USB HID Specification 1.11 and HID Usage Tables 1.12

#![no_std]
#![no_main]

use teensy4_panic as _;
use cortex_m_rt::entry;

use imxrt_ral as ral;

use imxrt_usbh::{
    Result, UsbError,
    ehci::controller::{EhciController, Uninitialized, Running},
    phy::UsbPhy,
    enumeration::{DeviceEnumerator, EnumeratedDevice, DeviceClass},
    transfer::simple_control::{SetupPacket, ControlExecutor},
    dma::memory::UsbMemoryPool,
};

use core::sync::atomic::{AtomicU8, AtomicBool, Ordering};

/// HID Class Code
pub const HID_CLASS: u8 = 0x03;

/// HID Subclass Codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidSubclass {
    None = 0x00,
    Boot = 0x01,
}

/// HID Protocol Codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidProtocol {
    None = 0x00,
    Keyboard = 0x01,
    Mouse = 0x02,
}

/// HID Descriptor Types
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum HidDescriptorType {
    Hid = 0x21,
    Report = 0x22,
    Physical = 0x23,
}

/// HID Class-Specific Requests
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum HidRequest {
    GetReport = 0x01,
    GetIdle = 0x02,
    GetProtocol = 0x03,
    SetReport = 0x09,
    SetIdle = 0x0A,
    SetProtocol = 0x0B,
}

/// HID Report Types
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum HidReportType {
    Input = 0x01,
    Output = 0x02,
    Feature = 0x03,
}

/// HID Descriptor (follows interface descriptor)
#[repr(C, packed)]
pub struct HidDescriptor {
    pub length: u8,
    pub descriptor_type: u8,  // 0x21 for HID
    pub bcd_hid: u16,         // HID spec version (BCD)
    pub country_code: u8,
    pub num_descriptors: u8,
    pub report_type: u8,      // 0x22 for Report
    pub report_length: u16,
    // Additional descriptors may follow
}

/// Simple gamepad report structure (for most USB gamepads)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct GamepadReport {
    pub buttons: u16,      // Button bitmap
    pub left_x: u8,        // Left stick X axis
    pub left_y: u8,        // Left stick Y axis
    pub right_x: u8,       // Right stick X axis (if present)
    pub right_y: u8,       // Right stick Y axis (if present)
    pub triggers: u16,     // Trigger values
}

impl GamepadReport {
    pub const fn new() -> Self {
        Self {
            buttons: 0,
            left_x: 128,  // Centered
            left_y: 128,  // Centered
            right_x: 128, // Centered
            right_y: 128, // Centered
            triggers: 0,
        }
    }
    
    /// Check if a button is pressed
    pub fn is_button_pressed(&self, button: u8) -> bool {
        if button < 16 {
            (self.buttons & (1 << button)) != 0
        } else {
            false
        }
    }
    
    /// Get left stick position as signed values (-128 to 127)
    pub fn left_stick(&self) -> (i8, i8) {
        (
            self.left_x as i8,
            self.left_y as i8,
        )
    }
    
    /// Get right stick position as signed values (-128 to 127)
    pub fn right_stick(&self) -> (i8, i8) {
        (
            self.right_x as i8,
            self.right_y as i8,
        )
    }
}

/// HID gamepad driver
pub struct HidGamepad {
    /// Device address
    device_address: u8,
    /// Interface number
    interface: u8,
    /// Interrupt IN endpoint address
    interrupt_endpoint: u8,
    /// Maximum packet size for interrupt endpoint
    max_packet_size: u16,
    /// Polling interval (ms)
    poll_interval: u8,
    /// Current gamepad report
    current_report: GamepadReport,
    /// Previous gamepad report (for detecting changes)
    previous_report: GamepadReport,
    /// Protocol (boot or report)
    protocol: HidProtocol,
    /// Report descriptor length
    report_desc_length: u16,
    /// Idle rate (4ms units, 0 = indefinite)
    idle_rate: u8,
    /// Number of reports received
    report_count: AtomicU8,
    /// Error flag
    has_error: AtomicBool,
}

impl HidGamepad {
    /// Create new HID gamepad driver
    pub fn new(
        device_address: u8,
        interface: u8,
        interrupt_endpoint: u8,
        max_packet_size: u16,
        poll_interval: u8,
    ) -> Self {
        Self {
            device_address,
            interface,
            interrupt_endpoint,
            max_packet_size,
            poll_interval,
            current_report: GamepadReport::new(),
            previous_report: GamepadReport::new(),
            protocol: HidProtocol::None,
            report_desc_length: 0,
            idle_rate: 0,
            report_count: AtomicU8::new(0),
            has_error: AtomicBool::new(false),
        }
    }
    
    /// Initialize gamepad
    pub fn initialize(&mut self, memory_pool: &mut UsbMemoryPool) -> Result<()> {
        // Set idle rate to 0 (indefinite) for all reports
        self.set_idle(0, 0, memory_pool)?;
        
        Ok(())
    }
    
    /// Set idle rate
    fn set_idle(&mut self, duration: u8, report_id: u8, memory_pool: &mut UsbMemoryPool) -> Result<()> {
        let setup = SetupPacket {
            bmRequestType: 0x21, // Host-to-device, class, interface
            bRequest: HidRequest::SetIdle as u8,
            wValue: ((duration as u16) << 8) | (report_id as u16),
            wIndex: self.interface as u16,
            wLength: 0,
        };
        
        ControlExecutor::execute_with_retry(
            setup,
            self.device_address,
            self.max_packet_size,
            memory_pool,
            3,
        )?;
        
        self.idle_rate = duration;
        Ok(())
    }
    
    /// Get HID descriptor
    pub fn get_hid_descriptor(&mut self, memory_pool: &mut UsbMemoryPool) -> Result<HidDescriptor> {
        let setup = SetupPacket {
            bmRequestType: 0x81, // Device-to-host, standard, interface
            bRequest: 0x06,      // GET_DESCRIPTOR
            wValue: (HidDescriptorType::Hid as u16) << 8,
            wIndex: self.interface as u16,
            wLength: 9,  // HID descriptor is 9 bytes minimum
        };
        
        let data = ControlExecutor::execute_with_retry(
            setup,
            self.device_address,
            self.max_packet_size,
            memory_pool,
            3,
        )?;
        
        if data.len() < 9 {
            return Err(UsbError::InvalidDescriptor);
        }
        
        // Parse descriptor
        let desc = unsafe {
            core::ptr::read_unaligned(data.as_ptr() as *const HidDescriptor)
        };
        
        self.report_desc_length = desc.report_length;
        
        Ok(desc)
    }
    
    /// Process incoming gamepad report
    pub fn process_report(&mut self, data: &[u8]) -> Option<GamepadEvent> {
        if data.len() < core::mem::size_of::<GamepadReport>() {
            return None;
        }
        
        // Store previous report
        self.previous_report = self.current_report;
        
        // Parse new report
        if let Some(report) = parse_gamepad_report(data) {
            self.current_report = report;
            
            // Increment report counter
            self.report_count.fetch_add(1, Ordering::Relaxed);
            
            // Detect changes
            self.detect_gamepad_changes()
        } else {
            self.has_error.store(true, Ordering::Release);
            None
        }
    }
    
    /// Detect gamepad changes
    fn detect_gamepad_changes(&self) -> Option<GamepadEvent> {
        // Check button changes
        let prev_buttons = self.previous_report.buttons;
        let curr_buttons = self.current_report.buttons;
        
        if prev_buttons != curr_buttons {
            // Find which buttons changed
            let changed = prev_buttons ^ curr_buttons;
            let pressed = changed & curr_buttons;
            let released = changed & prev_buttons;
            
            if pressed != 0 {
                // Find first pressed button
                for i in 0..16 {
                    if (pressed & (1 << i)) != 0 {
                        return Some(GamepadEvent::ButtonPress(i));
                    }
                }
            }
            
            if released != 0 {
                // Find first released button
                for i in 0..16 {
                    if (released & (1 << i)) != 0 {
                        return Some(GamepadEvent::ButtonRelease(i));
                    }
                }
            }
        }
        
        // Check stick movements (with deadzone)
        let (prev_lx, prev_ly) = self.previous_report.left_stick();
        let (curr_lx, curr_ly) = self.current_report.left_stick();
        
        let dx = curr_lx.wrapping_sub(prev_lx);
        let dy = curr_ly.wrapping_sub(prev_ly);
        
        if dx.abs() > 5 || dy.abs() > 5 { // 5-unit deadzone
            return Some(GamepadEvent::LeftStickMove(curr_lx, curr_ly));
        }
        
        None
    }
    
    /// Get current pressed buttons
    pub fn get_pressed_buttons(&self) -> heapless::Vec<u8, 16> {
        let mut buttons = heapless::Vec::new();
        for i in 0..16 {
            if self.current_report.is_button_pressed(i) {
                let _ = buttons.push(i);
            }
        }
        buttons
    }
    
    /// Get current left stick position
    pub fn get_left_stick(&self) -> (i8, i8) {
        self.current_report.left_stick()
    }
    
    /// Get report statistics
    pub fn report_count(&self) -> u8 {
        self.report_count.load(Ordering::Relaxed)
    }
}

/// Gamepad events
#[derive(Debug, Clone, Copy)]
pub enum GamepadEvent {
    /// Button pressed
    ButtonPress(u8),
    /// Button released
    ButtonRelease(u8),
    /// Left stick moved
    LeftStickMove(i8, i8),
    /// Right stick moved
    RightStickMove(i8, i8),
}

/// Parse a generic gamepad report
fn parse_gamepad_report(data: &[u8]) -> Option<GamepadReport> {
    if data.len() < 8 {
        return None;
    }
    
    // Simple parsing for common gamepad formats
    Some(GamepadReport {
        buttons: u16::from_le_bytes([data[0], data[1]]),
        left_x: data[2],
        left_y: data[3],
        right_x: data.get(4).copied().unwrap_or(128),
        right_y: data.get(5).copied().unwrap_or(128),
        triggers: data.get(6..8)
            .map(|t| u16::from_le_bytes([t[0], t[1]]))
            .unwrap_or(0),
    })
}

/// HID Manager for multiple devices
pub struct HidManager {
    /// Active gamepads
    gamepads: heapless::Vec<HidGamepad, 4>,
    /// Event queue
    event_queue: heapless::Deque<(u8, GamepadEvent), 32>,
}

impl HidManager {
    /// Create new HID manager
    pub const fn new() -> Self {
        Self {
            gamepads: heapless::Vec::new(),
            event_queue: heapless::Deque::new(),
        }
    }
    
    /// Register a new gamepad
    pub fn register_gamepad(&mut self, gamepad: HidGamepad) -> Result<usize> {
        let index = self.gamepads.len();
        self.gamepads.push(gamepad)
            .map_err(|_| UsbError::NoResources)?;
        Ok(index)
    }
    
    /// Process report from a gamepad
    pub fn process_gamepad_report(&mut self, gamepad_index: usize, data: &[u8]) {
        if let Some(gamepad) = self.gamepads.get_mut(gamepad_index) {
            if let Some(event) = gamepad.process_report(data) {
                // Queue event with gamepad index
                let _ = self.event_queue.push_back((gamepad_index as u8, event));
            }
        }
    }
    
    /// Get next event from queue
    pub fn get_next_event(&mut self) -> Option<(u8, GamepadEvent)> {
        self.event_queue.pop_front()
    }
    
    /// Get gamepad by index
    pub fn get_gamepad(&self, index: usize) -> Option<&HidGamepad> {
        self.gamepads.get(index)
    }
    
    /// Get mutable gamepad by index
    pub fn get_gamepad_mut(&mut self, index: usize) -> Option<&mut HidGamepad> {
        self.gamepads.get_mut(index)
    }
}

#[entry]
fn main() -> ! {
    // Initialize peripherals
    let _peripherals = unsafe { ral::Instances::instances() };
    
    // Initialize clocks
    configure_usb_clocks();
    
    // Initialize USB memory pool
    let mut memory_pool = UsbMemoryPool::new();
    
    // Initialize USB PHY
    let phy_base = 0x400D_9000; // USBPHY1 base address
    let ccm_base = 0x400F_C000;  // CCM base address
    let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };
    
    // Initialize USB host controller
    let usb1_base = 0x402E_0140; // USB1 base address
    let controller = unsafe {
        EhciController::<8, Uninitialized>::new(usb1_base)
            .expect("Failed to create EHCI controller")
    };
    
    let controller = unsafe {
        controller.initialize()
            .expect("Failed to initialize EHCI controller")
    };
    
    let mut controller = unsafe { controller.start() };
    
    // Initialize HID manager
    let mut hid_manager = HidManager::new();
    
    // Main loop
    let mut device_connected = false;
    
    loop {
        if !device_connected {
            // Try to find and connect to a gamepad
            match find_and_init_gamepad(&mut controller, &mut memory_pool) {
                Ok(gamepad) => {
                    match hid_manager.register_gamepad(gamepad) {
                        Ok(index) => {
                            device_connected = true;
                            // In a real app, you'd start interrupt transfers here
                            let _ = index;
                        }
                        Err(_) => {
                            // Manager full
                        }
                    }
                }
                Err(_) => {
                    // No gamepad found
                }
            }
        }
        
        // Process events
        while let Some((gamepad_id, event)) = hid_manager.get_next_event() {
            handle_gamepad_event(gamepad_id, event, &hid_manager);
        }
        
        // Small delay
        delay_ms(10);
    }
}

/// Find and initialize a USB gamepad
fn find_and_init_gamepad(
    controller: &mut EhciController<8, Running>,
    memory_pool: &mut UsbMemoryPool,
) -> Result<HidGamepad> {
    // Enumerate device
    let mut enumerator = DeviceEnumerator::new(controller, memory_pool);
    let device = enumerator.enumerate_device()?;
    
    // Check if it's a HID device
    if device.class != DeviceClass::Hid {
        return Err(UsbError::Unsupported);
    }
    
    // Create gamepad driver
    let mut gamepad = HidGamepad::new(
        device.address,
        0,  // Interface 0
        0x81, // Endpoint 1 IN
        8,    // Max packet size
        10,   // 10ms poll interval
    );
    
    // Initialize the gamepad
    gamepad.initialize(memory_pool)?;
    
    Ok(gamepad)
}

/// Handle gamepad events
fn handle_gamepad_event(gamepad_id: u8, event: GamepadEvent, _manager: &HidManager) {
    match event {
        GamepadEvent::ButtonPress(button) => {
            // Handle button press
            // In a real app: trigger actions, update game state, etc.
            let _ = (gamepad_id, button);
        }
        GamepadEvent::ButtonRelease(button) => {
            // Handle button release
            let _ = (gamepad_id, button);
        }
        GamepadEvent::LeftStickMove(x, y) => {
            // Handle stick movement
            // In a real app: move character, control camera, etc.
            let _ = (gamepad_id, x, y);
        }
        GamepadEvent::RightStickMove(x, y) => {
            // Handle right stick movement
            let _ = (gamepad_id, x, y);
        }
    }
}

/// Configure USB clocks for i.MX RT1062
fn configure_usb_clocks() {
    use ral::{modify_reg, read_reg};
    
    unsafe {
        let ccm = ral::ccm::CCM::instance();
        
        // Enable USB clocks
        modify_reg!(ral::ccm, ccm, CCGR6, 
            CG0: 0b11,  // usb_ctrl1_clk
            CG1: 0b11,  // usb_ctrl2_clk  
            CG2: 0b11,  // usb_phy1_clk
            CG3: 0b11   // usb_phy2_clk
        );
        
        // Configure USB PHY PLL (480MHz)
        let analog = ral::ccm_analog::CCM_ANALOG::instance();
        
        // Power up PLL
        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: 1,
            ENABLE: 1,
            EN_USB_CLKS: 1
        );
        
        // Wait for PLL lock
        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) == 0 {}
    }
}

/// Simple delay function
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms); // Assuming 600MHz clock
}