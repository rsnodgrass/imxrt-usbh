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

use bsp::board;
use log::info;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_usbh::{
    dma::UsbMemoryPool,
    ehci::{EhciController, Running, TransferExecutor, Uninitialized},
    enumeration::{DeviceClass, DeviceEnumerator},
    phy::UsbPhy,
    transfer::simple_control::{ControlExecutor, SetupPacket},
    transfer::{Direction, InterruptTransferManager},
    Result, UsbError,
};

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

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
    pub descriptor_type: u8, // 0x21 for HID
    pub bcd_hid: u16,        // HID spec version (BCD)
    pub country_code: u8,
    pub num_descriptors: u8,
    pub report_type: u8, // 0x22 for Report
    pub report_length: u16,
    // Additional descriptors may follow
}

/// Simple gamepad report structure (for most USB gamepads)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct GamepadReport {
    pub buttons: u16,  // Button bitmap
    pub left_x: u8,    // Left stick X axis
    pub left_y: u8,    // Left stick Y axis
    pub right_x: u8,   // Right stick X axis (if present)
    pub right_y: u8,   // Right stick Y axis (if present)
    pub triggers: u16, // Trigger values
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
        (self.left_x as i8, self.left_y as i8)
    }

    /// Get right stick position as signed values (-128 to 127)
    pub fn right_stick(&self) -> (i8, i8) {
        (self.right_x as i8, self.right_y as i8)
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
    pub fn initialize(
        &mut self,
        transfer_executor: &mut TransferExecutor,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<()> {
        // Set idle rate to 0 (indefinite) for all reports
        self.set_idle(0, 0, transfer_executor, memory_pool)?;

        Ok(())
    }

    /// Set idle rate
    fn set_idle(
        &mut self,
        duration: u8,
        report_id: u8,
        transfer_executor: &mut TransferExecutor,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<()> {
        let setup = SetupPacket {
            bmRequestType: 0x21, // Host-to-device, class, interface
            bRequest: HidRequest::SetIdle as u8,
            wValue: ((duration as u16) << 8) | (report_id as u16),
            wIndex: self.interface as u16,
            wLength: 0,
        };

        let mut executor = ControlExecutor::new(transfer_executor, memory_pool);
        executor.execute_with_retry(setup, self.device_address, self.max_packet_size, 3)?;

        self.idle_rate = duration;
        Ok(())
    }

    /// Get HID descriptor
    pub fn get_hid_descriptor(
        &mut self,
        transfer_executor: &mut TransferExecutor,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<HidDescriptor> {
        let setup = SetupPacket {
            bmRequestType: 0x81, // Device-to-host, standard, interface
            bRequest: 0x06,      // GET_DESCRIPTOR
            wValue: (HidDescriptorType::Hid as u16) << 8,
            wIndex: self.interface as u16,
            wLength: 9, // HID descriptor is 9 bytes minimum
        };

        let mut executor = ControlExecutor::new(transfer_executor, memory_pool);
        let data =
            executor.execute_with_retry(setup, self.device_address, self.max_packet_size, 3)?;

        if data.len() < 9 {
            return Err(UsbError::InvalidDescriptor);
        }

        // Parse descriptor
        let desc = unsafe { core::ptr::read_unaligned(data.as_ptr() as *const HidDescriptor) };

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

        if dx.abs() > 5 || dy.abs() > 5 {
            // 5-unit deadzone
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
        triggers: data
            .get(6..8)
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
        self.gamepads
            .push(gamepad)
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

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        ..
    } = board::t40(board::instances());

    let led = board::led(&mut gpio2, pins.p13);

    // USB CDC logging
    let mut poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

    info!("\r\n=== USB HID Gamepad Example ===");
    poller.poll();

    // Initialize USB PHY for host mode
    info!("Initializing USB PHY...");
    poller.poll();

    let mut phy = unsafe { UsbPhy::new(0x400DA000, 0x400F_C000) };

    if phy.init_host_mode().is_err() {
        info!("✗ PHY init failed!");
        poller.poll();
        loop {
            led.toggle();
            cortex_m::asm::delay(60_000_000);
        }
    }

    info!("✓ USB PHY initialized");
    poller.poll();

    // Initialize EHCI controller (USB2 for host)
    let controller = unsafe {
        match EhciController::<8, Uninitialized>::new(0x402E_0200) {
            Ok(c) => c,
            Err(_) => {
                info!("✗ EHCI init failed!");
                poller.poll();
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000);
                }
            }
        }
    };

    let controller = unsafe { controller.initialize().unwrap() };
    let mut controller = unsafe { controller.start() };

    info!("✓ USB host controller running");
    poller.poll();

    // Initialize memory pool and transfer executor
    let mut memory_pool = UsbMemoryPool::new();
    let mut transfer_executor = unsafe { TransferExecutor::new(0x402E_0200) };

    info!("\r\nWaiting for gamepad...");
    poller.poll();

    let mut gamepad_detected = false;
    let mut counter = 0u32;

    loop {
        poller.poll();
        counter += 1;

        if !gamepad_detected && counter % 1000 == 0 {
            // Try to enumerate gamepad
            let mut enumerator =
                DeviceEnumerator::new(&mut controller, &mut memory_pool, &mut transfer_executor);

            if let Ok(device) = enumerator.enumerate_device() {
                if device.class == DeviceClass::Hid {
                    info!("\r\n✓ HID Gamepad detected!");
                    info!("  Address: {}", device.address);
                    poller.poll();

                    // Create gamepad driver
                    let mut gamepad = HidGamepad::new(
                        device.address,
                        0,    // Interface 0
                        0x81, // Endpoint 1 IN
                        8,    // Max packet size
                        10,   // 10ms poll interval
                    );

                    // Initialize gamepad (set idle rate)
                    if gamepad
                        .initialize(&mut transfer_executor, &mut memory_pool)
                        .is_ok()
                    {
                        info!("✓ Gamepad initialized");
                        poller.poll();
                        gamepad_detected = true;

                        // Set up interrupt transfer for gamepad input
                        let mut interrupt_mgr = InterruptTransferManager::<4>::new();

                        if let Some(buffer) = memory_pool.alloc_buffer(8) {
                            match interrupt_mgr.submit(
                                Direction::In,
                                device.address,
                                0x81,
                                8,
                                buffer,
                                10,
                                true,
                            ) {
                                Ok(transfer_id) => {
                                    info!("✓ Interrupt transfer started");
                                    info!("\r\nGamepad ready! Press buttons...\r\n");
                                    poller.poll();

                                    // Monitor gamepad input
                                    let mut last_bytes = 0u32;
                                    loop {
                                        poller.poll();

                                        if let Some(transfer) =
                                            interrupt_mgr.get_transfer(transfer_id)
                                        {
                                            let current_bytes = transfer.bytes_transferred();

                                            if transfer.is_complete() && current_bytes != last_bytes
                                            {
                                                info!("Gamepad input: {} bytes", current_bytes);
                                                last_bytes = current_bytes;
                                                led.toggle();
                                            }
                                        }

                                        cortex_m::asm::delay(10000);
                                    }
                                }
                                Err(_) => {
                                    info!("✗ Failed to start interrupt transfer");
                                    poller.poll();
                                }
                            }
                        }
                    }
                }
            }
        }

        // Blink LED
        if counter % 300_000 == 0 {
            led.toggle();
        }

        cortex_m::asm::delay(1000);
    }
}
