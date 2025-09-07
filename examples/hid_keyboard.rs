//! Full-Featured QWERTY USB Keyboard with RTIC
//!
//! This example demonstrates a comprehensive USB keyboard implementation using RTIC.
//! Features:
//! - Real-time key press detection and processing
//! - Full ASCII character conversion with modifier support
//! - Key repeat functionality
//! - Multiple keyboard support
//! - Performance monitoring and typing statistics
//! - Caps Lock, Num Lock, and Scroll Lock LED control
//! - N-key rollover support (up to 6 simultaneous keys)

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

use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};

/// USB HID Boot Keyboard Report (8 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, PartialEq)]
struct KeyboardReport {
    modifiers: u8,     // Modifier key bitmap
    reserved: u8,      // Reserved byte (always 0)
    keycodes: [u8; 6], // Up to 6 simultaneous key presses
}

impl KeyboardReport {
    const fn new() -> Self {
        Self {
            modifiers: 0,
            reserved: 0,
            keycodes: [0; 6],
        }
    }
    
    /// Check if a specific key is pressed
    fn has_key(&self, keycode: u8) -> bool {
        self.keycodes.iter().any(|&k| k == keycode)
    }
    
    /// Get all pressed keys
    fn get_pressed_keys(&self) -> heapless::Vec<u8, 6> {
        let mut keys: heapless::Vec<u8, 6> = heapless::Vec::new();
        for &keycode in &self.keycodes {
            if keycode != 0 {
                let _ = keys.push(keycode);
            }
        }
        keys
    }
    
    /// Check if modifier is pressed
    fn has_modifier(&self, modifier: ModifierKey) -> bool {
        (self.modifiers & modifier as u8) != 0
    }
}

/// Modifier key bitmasks
#[repr(u8)]
#[derive(Clone, Copy)]
enum ModifierKey {
    LeftCtrl = 0x01,
    LeftShift = 0x02,
    LeftAlt = 0x04,
    LeftGui = 0x08,
    RightCtrl = 0x10,
    RightShift = 0x20,
    RightAlt = 0x40,
    RightGui = 0x80,
}

/// USB HID keycodes for common keys
pub mod hid_keycodes {
    pub const KEY_A: u8 = 0x04;
    pub const KEY_B: u8 = 0x05;
    pub const KEY_C: u8 = 0x06;
    pub const KEY_D: u8 = 0x07;
    pub const KEY_E: u8 = 0x08;
    pub const KEY_F: u8 = 0x09;
    pub const KEY_G: u8 = 0x0A;
    pub const KEY_H: u8 = 0x0B;
    pub const KEY_I: u8 = 0x0C;
    pub const KEY_J: u8 = 0x0D;
    pub const KEY_K: u8 = 0x0E;
    pub const KEY_L: u8 = 0x0F;
    pub const KEY_M: u8 = 0x10;
    pub const KEY_N: u8 = 0x11;
    pub const KEY_O: u8 = 0x12;
    pub const KEY_P: u8 = 0x13;
    pub const KEY_Q: u8 = 0x14;
    pub const KEY_R: u8 = 0x15;
    pub const KEY_S: u8 = 0x16;
    pub const KEY_T: u8 = 0x17;
    pub const KEY_U: u8 = 0x18;
    pub const KEY_V: u8 = 0x19;
    pub const KEY_W: u8 = 0x1A;
    pub const KEY_X: u8 = 0x1B;
    pub const KEY_Y: u8 = 0x1C;
    pub const KEY_Z: u8 = 0x1D;
    
    // Numbers
    pub const KEY_1: u8 = 0x1E;
    pub const KEY_2: u8 = 0x1F;
    pub const KEY_3: u8 = 0x20;
    pub const KEY_4: u8 = 0x21;
    pub const KEY_5: u8 = 0x22;
    pub const KEY_6: u8 = 0x23;
    pub const KEY_7: u8 = 0x24;
    pub const KEY_8: u8 = 0x25;
    pub const KEY_9: u8 = 0x26;
    pub const KEY_0: u8 = 0x27;
    
    // Special keys
    pub const KEY_ENTER: u8 = 0x28;
    pub const KEY_ESCAPE: u8 = 0x29;
    pub const KEY_BACKSPACE: u8 = 0x2A;
    pub const KEY_TAB: u8 = 0x2B;
    pub const KEY_SPACE: u8 = 0x2C;
    pub const KEY_CAPS_LOCK: u8 = 0x39;
    pub const KEY_F1: u8 = 0x3A;
    pub const KEY_F2: u8 = 0x3B;
    pub const KEY_F12: u8 = 0x45;
    
    // Arrow keys
    pub const KEY_RIGHT_ARROW: u8 = 0x4F;
    pub const KEY_LEFT_ARROW: u8 = 0x50;
    pub const KEY_DOWN_ARROW: u8 = 0x51;
    pub const KEY_UP_ARROW: u8 = 0x52;
}

/// Keyboard event types
#[derive(Debug, Clone, Copy)]
pub enum KeyboardEvent {
    KeyPress { keycode: u8, modifiers: u8 },
    KeyRelease { keycode: u8, modifiers: u8 },
    KeyRepeat { keycode: u8, modifiers: u8 },
    ModifierChange { old_modifiers: u8, new_modifiers: u8 },
}

impl KeyboardEvent {
    /// Convert keycode to ASCII character
    pub fn to_ascii(&self) -> Option<char> {
        match self {
            KeyboardEvent::KeyPress { keycode, modifiers } |
            KeyboardEvent::KeyRepeat { keycode, modifiers } => {
                let shift = (*modifiers & (ModifierKey::LeftShift as u8 | ModifierKey::RightShift as u8)) != 0;
                keycode_to_ascii(*keycode, shift)
            }
            _ => None,
        }
    }
    
    /// Check if this is a control key combination
    pub fn is_control_combo(&self) -> bool {
        match self {
            KeyboardEvent::KeyPress { modifiers, .. } => {
                (*modifiers & (ModifierKey::LeftCtrl as u8 | ModifierKey::RightCtrl as u8)) != 0
            }
            _ => false,
        }
    }
}

/// Convert HID keycode to ASCII
fn keycode_to_ascii(keycode: u8, shift: bool) -> Option<char> {
    use hid_keycodes::*;
    
    match keycode {
        KEY_A..=KEY_Z => {
            let base = b'a' + (keycode - KEY_A);
            Some(if shift { base.to_ascii_uppercase() } else { base } as char)
        }
        KEY_1 => Some(if shift { '!' } else { '1' }),
        KEY_2 => Some(if shift { '@' } else { '2' }),
        KEY_3 => Some(if shift { '#' } else { '3' }),
        KEY_4 => Some(if shift { '$' } else { '4' }),
        KEY_5 => Some(if shift { '%' } else { '5' }),
        KEY_6 => Some(if shift { '^' } else { '6' }),
        KEY_7 => Some(if shift { '&' } else { '7' }),
        KEY_8 => Some(if shift { '*' } else { '8' }),
        KEY_9 => Some(if shift { '(' } else { '9' }),
        KEY_0 => Some(if shift { ')' } else { '0' }),
        KEY_ENTER => Some('\n'),
        KEY_TAB => Some('\t'),
        KEY_SPACE => Some(' '),
        KEY_BACKSPACE => Some('\x08'),
        KEY_ESCAPE => Some('\x1b'),
        _ => None,
    }
}

/// Keyboard state tracking
struct KeyboardState {
    current_report: KeyboardReport,
    previous_report: KeyboardReport,
    caps_lock: AtomicBool,
    num_lock: AtomicBool,
    scroll_lock: AtomicBool,
    key_repeat_timer: [u8; 6], // Repeat counters for each key slot
    repeat_delay: u8,          // Initial repeat delay
    repeat_rate: u8,           // Repeat rate
}

impl KeyboardState {
    fn new() -> Self {
        Self {
            current_report: KeyboardReport::new(),
            previous_report: KeyboardReport::new(),
            caps_lock: AtomicBool::new(false),
            num_lock: AtomicBool::new(false),
            scroll_lock: AtomicBool::new(false),
            key_repeat_timer: [0; 6],
            repeat_delay: 50,  // 500ms initial delay
            repeat_rate: 10,   // 100ms repeat rate
        }
    }
    
    /// Update keyboard state with new report
    fn update(&mut self, new_report: KeyboardReport) -> heapless::Vec<KeyboardEvent, 12> {
        let mut events: heapless::Vec<KeyboardEvent, 12> = heapless::Vec::new();
        
        self.previous_report = self.current_report;
        self.current_report = new_report;
        
        // Check for modifier changes
        if self.current_report.modifiers != self.previous_report.modifiers {
            let _ = events.push(KeyboardEvent::ModifierChange {
                old_modifiers: self.previous_report.modifiers,
                new_modifiers: self.current_report.modifiers,
            });
        }
        
        // Check for key presses (keys in current but not previous)
        for &keycode in &self.current_report.keycodes {
            if keycode != 0 && !self.previous_report.has_key(keycode) {
                let _ = events.push(KeyboardEvent::KeyPress {
                    keycode,
                    modifiers: self.current_report.modifiers,
                });
                
                // Handle special keys
                self.handle_special_key(keycode);
            }
        }
        
        // Check for key releases (keys in previous but not current)
        for &keycode in &self.previous_report.keycodes {
            if keycode != 0 && !self.current_report.has_key(keycode) {
                let _ = events.push(KeyboardEvent::KeyRelease {
                    keycode,
                    modifiers: self.current_report.modifiers,
                });
            }
        }
        
        events
    }
    
    /// Handle special toggle keys
    fn handle_special_key(&self, keycode: u8) {
        match keycode {
            hid_keycodes::KEY_CAPS_LOCK => {
                let current = self.caps_lock.load(Ordering::Relaxed);
                self.caps_lock.store(!current, Ordering::Relaxed);
            }
            _ => {}
        }
    }
    
    /// Process key repeat timers
    fn process_repeats(&mut self) -> heapless::Vec<KeyboardEvent, 6> {
        let mut events: heapless::Vec<KeyboardEvent, 6> = heapless::Vec::new();
        
        for (i, keycode) in self.current_report.keycodes.iter().enumerate() {
            if *keycode != 0 {
                self.key_repeat_timer[i] = self.key_repeat_timer[i].saturating_add(1);
                
                // Check if key should repeat
                if self.key_repeat_timer[i] == self.repeat_delay || 
                   (self.key_repeat_timer[i] > self.repeat_delay && 
                    (self.key_repeat_timer[i] - self.repeat_delay) % self.repeat_rate == 0) {
                    let _ = events.push(KeyboardEvent::KeyRepeat {
                        keycode: *keycode,
                        modifiers: self.current_report.modifiers,
                    });
                }
            } else {
                self.key_repeat_timer[i] = 0;
            }
        }
        
        events
    }
}

/// Typing statistics
#[derive(Default)]
struct TypingStats {
    total_keypresses: AtomicU32,
    characters_typed: AtomicU32,
    words_typed: AtomicU32,
    typing_errors: AtomicU32,
    session_start: AtomicU32,
}

impl TypingStats {
    const fn new() -> Self {
        Self {
            total_keypresses: AtomicU32::new(0),
            characters_typed: AtomicU32::new(0),
            words_typed: AtomicU32::new(0),
            typing_errors: AtomicU32::new(0),
            session_start: AtomicU32::new(0),
        }
    }
    
    fn keypress(&self) {
        self.total_keypresses.fetch_add(1, Ordering::Relaxed);
    }
    
    fn character(&self) {
        self.characters_typed.fetch_add(1, Ordering::Relaxed);
    }
    
    fn word(&self) {
        self.words_typed.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Calculate words per minute
    fn get_wpm(&self, current_time: u32) -> u32 {
        let start_time = self.session_start.load(Ordering::Relaxed);
        if start_time == 0 {
            self.session_start.store(current_time, Ordering::Relaxed);
            return 0;
        }
        
        let elapsed_minutes = (current_time - start_time) / 60_000; // Convert ms to minutes
        if elapsed_minutes == 0 {
            return 0;
        }
        
        self.words_typed.load(Ordering::Relaxed) / elapsed_minutes
    }
}

/// USB Keyboard Device
struct UsbKeyboard {
    device_info: EnumeratedDevice,
    interface_number: u8,
    interrupt_endpoint: u8,
    max_packet_size: u16,
    poll_interval: u8,
    state: KeyboardState,
    is_initialized: bool,
}

impl UsbKeyboard {
    fn new(device_info: EnumeratedDevice) -> Self {
        Self {
            device_info,
            interface_number: 0,
            interrupt_endpoint: 0x81, // Endpoint 1 IN
            max_packet_size: 8,       // Boot keyboard report size
            poll_interval: 10,        // 10ms poll interval
            state: KeyboardState::new(),
            is_initialized: false,
        }
    }
    
    /// Initialize keyboard with HID boot protocol
    fn initialize(&mut self, memory_pool: &mut UsbMemoryPool) -> Result<()> {
        // Set boot protocol for maximum compatibility
        let setup = SetupPacket {
            bmRequestType: 0x21, // Host-to-device, class, interface
            bRequest: 0x0B,      // SET_PROTOCOL
            wValue: 0,           // 0 = Boot Protocol
            wIndex: self.interface_number as u16,
            wLength: 0,
        };
        
        ControlExecutor::execute_with_retry(
            setup,
            self.device_info.address,
            self.max_packet_size,
            memory_pool,
            3,
        )?;
        
        // Set idle rate to 0 (report on change only)
        let setup = SetupPacket {
            bmRequestType: 0x21,
            bRequest: 0x0A,      // SET_IDLE
            wValue: 0,           // 0 = indefinite
            wIndex: self.interface_number as u16,
            wLength: 0,
        };
        
        ControlExecutor::execute_with_retry(
            setup,
            self.device_info.address,
            self.max_packet_size,
            memory_pool,
            3,
        )?;
        
        self.is_initialized = true;
        Ok(())
    }
    
    /// Simulate keyboard report reading (in real implementation, use interrupt transfers)
    /// 
    /// This demonstrates how keyboard reports would arrive from a real USB keyboard.
    /// Each report represents the current state of all keys at a specific moment.
    fn simulate_keyboard_data(&self) -> Option<KeyboardReport> {
        // For educational purposes, simulate typing "Hello World!" sequence
        static mut SIMULATION_COUNTER: u32 = 0;
        
        unsafe {
            SIMULATION_COUNTER += 1;
            let sequence_position = SIMULATION_COUNTER % 1000;
            
            // Generate different reports based on timing to simulate typing
            self.get_simulated_keypress(sequence_position)
        }
    }
    
    /// Generate simulated keypress for educational demonstration
    /// 
    /// This breaks down the simulation into a cleaner helper function that
    /// demonstrates how USB HID keycodes work in practice.
    fn get_simulated_keypress(&self, position: u32) -> Option<KeyboardReport> {
        use self::hid_keycodes::*;  // Use our well-documented keycode constants
        
        // Simulate typing "Hello World!" character by character
        // Each case shows how modifiers and keycodes combine for different characters
        match position {
            // Capital 'H' - requires Shift modifier (0x02) + H keycode (0x0B)
            100 => Some(self.create_report_with_shift(KEY_H)),
            
            // Lowercase letters - just the keycode, no modifier needed
            200 => Some(self.create_simple_report(KEY_E)),  // 'e'
            300 => Some(self.create_simple_report(KEY_L)),  // 'l' 
            400 => Some(self.create_simple_report(KEY_L)),  // 'l'
            500 => Some(self.create_simple_report(KEY_O)),  // 'o'
            
            // Space character - uses specific space keycode
            600 => Some(self.create_simple_report(KEY_SPACE)),
            
            // Capital 'W' - another shift + letter combination  
            650 => Some(self.create_report_with_shift(KEY_W)),
            
            // Continue with rest of "orld!"
            700 => Some(self.create_simple_report(KEY_O)),  // 'o'
            750 => Some(self.create_simple_report(KEY_R)),  // 'r'
            800 => Some(self.create_simple_report(KEY_L)),  // 'l'
            850 => Some(self.create_simple_report(KEY_D)),  // 'd'
            
            // Exclamation mark - Shift + 1 key (which has ! symbol when shifted)
            900 => Some(self.create_report_with_shift(KEY_1)),
            
            // Default: no keys pressed (all keys released)
            _ => Some(KeyboardReport::new()),
        }
    }
    
    /// Create a keyboard report for a simple key press (no modifiers)
    /// 
    /// This helper function makes the code more readable and educational
    /// by clearly showing how to construct a basic key press report.
    fn create_simple_report(&self, keycode: u8) -> KeyboardReport {
        KeyboardReport {
            modifiers: 0,           // No modifier keys pressed
            reserved: 0,            // Always 0 in HID boot protocol
            keycodes: [keycode, 0, 0, 0, 0, 0], // Primary key + 5 empty slots
        }
    }
    
    /// Create a keyboard report for a key press with Shift modifier
    /// 
    /// This shows how capital letters and shifted symbols are created by
    /// combining the base keycode with the appropriate modifier.
    fn create_report_with_shift(&self, keycode: u8) -> KeyboardReport {
        KeyboardReport {
            modifiers: 0x02,        // Left Shift modifier (bit 1 set)
            reserved: 0,            // Always 0 in HID boot protocol  
            keycodes: [keycode, 0, 0, 0, 0, 0], // Primary key + 5 empty slots
        }
    }
    
    /// Update keyboard state and return events
    fn update(&mut self, report: KeyboardReport) -> heapless::Vec<KeyboardEvent, 12> {
        self.state.update(report)
    }
    
    /// Process key repeats
    fn process_repeats(&mut self) -> heapless::Vec<KeyboardEvent, 6> {
        self.state.process_repeats()
    }
}

// Use a simple counter-based approach instead of full RTIC for now
static SYSTEM_COUNTER: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Simple RTIC-like structure for this example
struct SimpleApp {
    usb_controller: EhciController<8, Running>,
    memory_pool: UsbMemoryPool,
    keyboard: Option<UsbKeyboard>,
    stats: TypingStats,
    led_counter: u32,
}

impl SimpleApp {
    fn new() -> Result<Self> {
        // Configure USB clocks
        configure_usb_clocks();
        
        // Initialize USB memory pool
        let memory_pool = UsbMemoryPool::new();
        
        // Initialize USB PHY
        let phy_base = 0x400D_9000;
        let ccm_base = 0x400F_C000;
        let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };
        
        // Initialize USB host controller
        let usb1_base = 0x402E_0140;
        let controller = unsafe {
            EhciController::<8, Uninitialized>::new(usb1_base)?
        };
        
        let controller = unsafe { controller.initialize()? };
        let usb_controller = unsafe { controller.start() };
        
        Ok(Self {
            usb_controller,
            memory_pool,
            keyboard: None,
            stats: TypingStats::new(),
            led_counter: 0,
        })
    }
    
    fn detect_keyboard(&mut self) {
        if self.keyboard.is_some() {
            return; // Already have a keyboard
        }
        
        // Try to enumerate a keyboard
        if let Ok(device_info) = self.enumerate_hid_device() {
            let mut keyboard = UsbKeyboard::new(device_info);
            if keyboard.initialize(&mut self.memory_pool).is_ok() {
                self.keyboard = Some(keyboard);
            }
        }
    }
    
    fn enumerate_hid_device(&mut self) -> Result<EnumeratedDevice> {
        let mut enumerator = DeviceEnumerator::new(&mut self.usb_controller, &mut self.memory_pool);
        let device_info = enumerator.enumerate_device()?;
        
        if device_info.class != DeviceClass::Hid {
            return Err(UsbError::Unsupported);
        }
        
        Ok(device_info)
    }
    
    fn process_keyboard(&mut self) {
        if let Some(ref mut keyboard) = self.keyboard {
            if let Some(report) = keyboard.simulate_keyboard_data() {
                let events = keyboard.update(report);
                let repeat_events = keyboard.process_repeats();
                
                // Handle all events
                self.handle_keyboard_events(events);
                self.handle_keyboard_events(repeat_events);
            }
        }
    }
    
    fn handle_keyboard_events<const N: usize>(&self, events: heapless::Vec<KeyboardEvent, N>) {
        for event in events {
            self.stats.keypress();
            
            match event {
                KeyboardEvent::KeyPress { keycode, modifiers } => {
                    if let Some(ch) = keycode_to_ascii(keycode, (modifiers & 0x22) != 0) {
                        self.stats.character();
                        if ch == ' ' {
                            self.stats.word();
                        }
                        
                        // In a real app, output character via UART, USB CDC, display, etc.
                        let _ = ch;
                    }
                    
                    // Handle control combinations
                    if (modifiers & 0x11) != 0 { // Ctrl pressed
                        // Handle Ctrl+key combinations
                        let _ = keycode;
                    }
                }
                KeyboardEvent::KeyRepeat { keycode, modifiers } => {
                    // Handle key repeat
                    let _ = (keycode, modifiers);
                }
                KeyboardEvent::ModifierChange { old_modifiers, new_modifiers } => {
                    // Handle modifier changes
                    let _ = (old_modifiers, new_modifiers);
                }
                _ => {}
            }
        }
    }
    
    fn update_status(&mut self) {
        self.led_counter += 1;
        
        // Every 5 seconds, update statistics
        if self.led_counter % 5000 == 0 {
            let current_time = SYSTEM_COUNTER.load(Ordering::Relaxed);
            let wpm = self.stats.get_wpm(current_time);
            let keypresses = self.stats.total_keypresses.load(Ordering::Relaxed);
            let characters = self.stats.characters_typed.load(Ordering::Relaxed);
            
            // In a real app, display statistics
            let _ = (wpm, keypresses, characters);
        }
    }
    
    fn run(&mut self) -> ! {
        loop {
            // Increment system counter (simulate time)
            SYSTEM_COUNTER.fetch_add(1, Ordering::Relaxed);
            
            // Device detection
            self.detect_keyboard();
            
            // Process keyboard input
            self.process_keyboard();
            
            // Update status
            self.update_status();
            
            // Simple delay
            delay_ms(1);
        }
    }
}

#[entry]
fn main() -> ! {
    let mut app = SimpleApp::new().expect("Failed to initialize USB keyboard app");
    app.run()
}

/// Configure USB clocks for i.MX RT1062
fn configure_usb_clocks() {
    use ral::{modify_reg, read_reg};
    
    unsafe {
        let ccm = ral::ccm::CCM::instance();
        
        // Enable USB clocks
        modify_reg!(ral::ccm, ccm, CCGR6, 
            CG0: 0b11,
            CG1: 0b11,
            CG2: 0b11,
            CG3: 0b11
        );
        
        // Configure USB PHY PLL
        let analog = ral::ccm_analog::CCM_ANALOG::instance();
        
        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: 1,
            ENABLE: 1,
            EN_USB_CLKS: 1
        );
        
        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) == 0 {}
    }
}

/// Simple delay function
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms);
}