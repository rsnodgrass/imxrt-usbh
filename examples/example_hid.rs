//! Example USB HID (Human Interface Device) implementation
//!
//! This is an example implementation showing how to support USB HID devices,
//! particularly keyboards, using the imxrt-usbh library.
//! 
//! This code demonstrates:
//! - HID descriptor parsing
//! - Keyboard boot protocol support  
//! - HID report processing
//! - Key event detection
//!
//! Based on USB HID Specification 1.11 and HID Usage Tables 1.12
//!
//! Note: This is example code that would typically be in a separate
//! crate like `imxrt-usbh-hid` for production use.

#![allow(dead_code)]

// These imports would come from the main imxrt-usbh crate in a real example
// use imxrt_usbh::error::{Result, UsbError};
// use imxrt_usbh::transfer::{Direction, SetupPacket, InterruptTransfer, InterruptTransferManager};
// use imxrt_usbh::dma::DmaBuffer;

// For now, create placeholder types to make the example compile standalone
use core::sync::atomic::{AtomicU8, AtomicBool, Ordering};

type Result<T> = core::result::Result<T, UsbError>;

#[derive(Debug)]
enum UsbError {
    InvalidParameter,
    NotReady,
    NoResources,
    NotFound,
    BufferTooSmall,
}

#[derive(Debug)]
enum Direction {
    In,
    Out,
}

struct SetupPacket;
struct InterruptTransfer;
struct InterruptTransferManager;
struct DmaBuffer;

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

/// Keyboard modifier keys (byte 0 of boot keyboard report)
#[derive(Debug, Clone, Copy)]
pub struct KeyboardModifiers {
    pub left_ctrl: bool,
    pub left_shift: bool,
    pub left_alt: bool,
    pub left_gui: bool,
    pub right_ctrl: bool,
    pub right_shift: bool,
    pub right_alt: bool,
    pub right_gui: bool,
}

impl KeyboardModifiers {
    /// Parse modifiers from report byte
    pub fn from_byte(byte: u8) -> Self {
        Self {
            left_ctrl: (byte & 0x01) != 0,
            left_shift: (byte & 0x02) != 0,
            left_alt: (byte & 0x04) != 0,
            left_gui: (byte & 0x08) != 0,
            right_ctrl: (byte & 0x10) != 0,
            right_shift: (byte & 0x20) != 0,
            right_alt: (byte & 0x40) != 0,
            right_gui: (byte & 0x80) != 0,
        }
    }
    
    /// Convert to byte representation
    pub fn to_byte(&self) -> u8 {
        let mut byte = 0;
        if self.left_ctrl { byte |= 0x01; }
        if self.left_shift { byte |= 0x02; }
        if self.left_alt { byte |= 0x04; }
        if self.left_gui { byte |= 0x08; }
        if self.right_ctrl { byte |= 0x10; }
        if self.right_shift { byte |= 0x20; }
        if self.right_alt { byte |= 0x40; }
        if self.right_gui { byte |= 0x80; }
        byte
    }
}

/// Boot keyboard input report (8 bytes)
#[repr(C, packed)]
pub struct BootKeyboardReport {
    pub modifiers: u8,        // Modifier keys
    pub reserved: u8,         // Reserved byte
    pub keycodes: [u8; 6],    // Up to 6 simultaneous keys
}

impl BootKeyboardReport {
    /// Create empty report
    pub const fn new() -> Self {
        Self {
            modifiers: 0,
            reserved: 0,
            keycodes: [0; 6],
        }
    }
    
    /// Parse report from raw bytes
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }
        
        Some(Self {
            modifiers: data[0],
            reserved: data[1],
            keycodes: [
                data[2], data[3], data[4],
                data[5], data[6], data[7],
            ],
        })
    }
    
    /// Get modifier keys
    pub fn get_modifiers(&self) -> KeyboardModifiers {
        KeyboardModifiers::from_byte(self.modifiers)
    }
    
    /// Check if a keycode is pressed
    pub fn is_key_pressed(&self, keycode: u8) -> bool {
        self.keycodes.iter().any(|&k| k == keycode)
    }
    
    /// Get all pressed keycodes (excluding zeros)
    pub fn pressed_keys(&self) -> impl Iterator<Item = u8> + '_ {
        self.keycodes.iter().copied().filter(|&k| k != 0)
    }
    
    /// Check for phantom state (all keys pressed - error condition)
    pub fn is_phantom_state(&self) -> bool {
        self.keycodes.iter().all(|&k| k == 0x01)
    }
}

/// HID keyboard driver
pub struct HidKeyboard {
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
    /// Current keyboard report
    current_report: BootKeyboardReport,
    /// Previous keyboard report (for detecting changes)
    previous_report: BootKeyboardReport,
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

impl HidKeyboard {
    /// Create new HID keyboard driver
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
            current_report: BootKeyboardReport::new(),
            previous_report: BootKeyboardReport::new(),
            protocol: HidProtocol::Keyboard,
            report_desc_length: 0,
            idle_rate: 0,
            report_count: AtomicU8::new(0),
            has_error: AtomicBool::new(false),
        }
    }
    
    /// Initialize keyboard (set boot protocol, etc.)
    pub async fn initialize(&mut self) -> Result<()> {
        // Set boot protocol for simplicity
        self.set_protocol(HidProtocol::Keyboard).await?;
        
        // Set idle rate to 0 (indefinite) for all reports
        self.set_idle(0, 0).await?;
        
        #[cfg(feature = "defmt")]
        defmt::info!("HID keyboard initialized: addr={}, ep={:02x}", 
                    self.device_address, self.interrupt_endpoint);
        
        Ok(())
    }
    
    /// Set protocol (boot or report)
    async fn set_protocol(&mut self, protocol: HidProtocol) -> Result<()> {
        let setup = SetupPacket {
            request_type: 0x21, // Host-to-device, class, interface
            request: HidRequest::SetProtocol as u8,
            value: protocol as u16,
            index: self.interface as u16,
            length: 0,
        };
        
        // Execute control transfer
        // This would use the control transfer implementation
        
        self.protocol = protocol;
        Ok(())
    }
    
    /// Get protocol
    pub async fn get_protocol(&self) -> Result<HidProtocol> {
        let setup = SetupPacket {
            request_type: 0xA1, // Device-to-host, class, interface
            request: HidRequest::GetProtocol as u8,
            value: 0,
            index: self.interface as u16,
            length: 1,
        };
        
        let mut protocol_byte = [0u8; 1];
        // Execute control transfer
        // This would use the control transfer implementation
        
        Ok(if protocol_byte[0] == 0 {
            HidProtocol::None
        } else if protocol_byte[0] == 1 {
            HidProtocol::Keyboard
        } else {
            HidProtocol::Mouse
        })
    }
    
    /// Set idle rate
    async fn set_idle(&mut self, duration: u8, report_id: u8) -> Result<()> {
        let setup = SetupPacket {
            request_type: 0x21, // Host-to-device, class, interface
            request: HidRequest::SetIdle as u8,
            value: ((duration as u16) << 8) | (report_id as u16),
            index: self.interface as u16,
            length: 0,
        };
        
        // Execute control transfer
        // This would use the control transfer implementation
        
        self.idle_rate = duration;
        Ok(())
    }
    
    /// Get HID descriptor
    pub async fn get_hid_descriptor(&mut self) -> Result<HidDescriptor> {
        let setup = SetupPacket {
            request_type: 0x81, // Device-to-host, standard, interface
            request: 0x06,      // GET_DESCRIPTOR
            value: (HidDescriptorType::Hid as u16) << 8,
            index: self.interface as u16,
            length: 9,  // HID descriptor is 9 bytes minimum
        };
        
        let mut desc_data = [0u8; 9];
        // Execute control transfer
        // This would use the control transfer implementation
        
        // Parse descriptor
        let desc = unsafe {
            core::ptr::read_unaligned(desc_data.as_ptr() as *const HidDescriptor)
        };
        
        self.report_desc_length = desc.report_length;
        
        Ok(desc)
    }
    
    /// Get report descriptor
    pub async fn get_report_descriptor(&self, buffer: &mut [u8]) -> Result<usize> {
        let length = buffer.len().min(self.report_desc_length as usize);
        
        let setup = SetupPacket {
            request_type: 0x81, // Device-to-host, standard, interface
            request: 0x06,      // GET_DESCRIPTOR
            value: (HidDescriptorType::Report as u16) << 8,
            index: self.interface as u16,
            length: length as u16,
        };
        
        // Execute control transfer
        // This would use the control transfer implementation
        
        Ok(length)
    }
    
    /// Process incoming keyboard report
    pub fn process_report(&mut self, data: &[u8]) -> Option<KeyboardEvent> {
        if let Some(report) = BootKeyboardReport::from_bytes(data) {
            // Check for phantom state
            if report.is_phantom_state() {
                #[cfg(feature = "defmt")]
                defmt::warn!("Keyboard phantom state detected");
                return None;
            }
            
            // Store previous report
            self.previous_report = self.current_report;
            self.current_report = report;
            
            // Increment report counter
            self.report_count.fetch_add(1, Ordering::Relaxed);
            
            // Detect changes
            self.detect_key_changes()
        } else {
            self.has_error.store(true, Ordering::Release);
            None
        }
    }
    
    /// Detect key press/release events
    fn detect_key_changes(&self) -> Option<KeyboardEvent> {
        // Check modifier changes
        let prev_mods = self.previous_report.modifiers;
        let curr_mods = self.current_report.modifiers;
        
        if prev_mods != curr_mods {
            return Some(KeyboardEvent::ModifierChange {
                previous: KeyboardModifiers::from_byte(prev_mods),
                current: KeyboardModifiers::from_byte(curr_mods),
            });
        }
        
        // Check for key presses (keys in current but not previous)
        for &key in &self.current_report.keycodes {
            if key != 0 && !self.previous_report.is_key_pressed(key) {
                return Some(KeyboardEvent::KeyPress(key));
            }
        }
        
        // Check for key releases (keys in previous but not current)
        for &key in &self.previous_report.keycodes {
            if key != 0 && !self.current_report.is_key_pressed(key) {
                return Some(KeyboardEvent::KeyRelease(key));
            }
        }
        
        None
    }
    
    /// Start receiving reports via interrupt transfer
    pub fn start_polling(
        &self,
        interrupt_manager: &mut InterruptTransferManager,
        buffer: DmaBuffer,
    ) -> Result<usize> {
        // Submit interrupt IN transfer for keyboard reports
        interrupt_manager.submit(
            Direction::In,
            self.device_address,
            self.interrupt_endpoint,
            self.max_packet_size,
            buffer,
            self.poll_interval,
            true, // Periodic transfer
        )
    }
    
    /// Get current pressed keys
    pub fn get_pressed_keys(&self) -> impl Iterator<Item = u8> + '_ {
        self.current_report.pressed_keys()
    }
    
    /// Get current modifier state
    pub fn get_modifiers(&self) -> KeyboardModifiers {
        self.current_report.get_modifiers()
    }
    
    /// Check if specific key is pressed
    pub fn is_key_pressed(&self, keycode: u8) -> bool {
        self.current_report.is_key_pressed(keycode)
    }
    
    /// Get report statistics
    pub fn report_count(&self) -> u8 {
        self.report_count.load(Ordering::Relaxed)
    }
}

/// Keyboard events
#[derive(Debug, Clone, Copy)]
pub enum KeyboardEvent {
    /// Key pressed
    KeyPress(u8),
    /// Key released
    KeyRelease(u8),
    /// Modifier keys changed
    ModifierChange {
        previous: KeyboardModifiers,
        current: KeyboardModifiers,
    },
}

/// Common USB HID keycodes (subset)
pub mod keycodes {
    // Letters
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
    pub const KEY_ESC: u8 = 0x29;
    pub const KEY_BACKSPACE: u8 = 0x2A;
    pub const KEY_TAB: u8 = 0x2B;
    pub const KEY_SPACE: u8 = 0x2C;
    pub const KEY_MINUS: u8 = 0x2D;
    pub const KEY_EQUAL: u8 = 0x2E;
    pub const KEY_LEFT_BRACKET: u8 = 0x2F;
    pub const KEY_RIGHT_BRACKET: u8 = 0x30;
    pub const KEY_BACKSLASH: u8 = 0x31;
    pub const KEY_SEMICOLON: u8 = 0x33;
    pub const KEY_QUOTE: u8 = 0x34;
    pub const KEY_GRAVE: u8 = 0x35;
    pub const KEY_COMMA: u8 = 0x36;
    pub const KEY_DOT: u8 = 0x37;
    pub const KEY_SLASH: u8 = 0x38;
    pub const KEY_CAPS_LOCK: u8 = 0x39;
    
    // Function keys
    pub const KEY_F1: u8 = 0x3A;
    pub const KEY_F2: u8 = 0x3B;
    pub const KEY_F3: u8 = 0x3C;
    pub const KEY_F4: u8 = 0x3D;
    pub const KEY_F5: u8 = 0x3E;
    pub const KEY_F6: u8 = 0x3F;
    pub const KEY_F7: u8 = 0x40;
    pub const KEY_F8: u8 = 0x41;
    pub const KEY_F9: u8 = 0x42;
    pub const KEY_F10: u8 = 0x43;
    pub const KEY_F11: u8 = 0x44;
    pub const KEY_F12: u8 = 0x45;
    
    // Navigation
    pub const KEY_INSERT: u8 = 0x49;
    pub const KEY_HOME: u8 = 0x4A;
    pub const KEY_PAGE_UP: u8 = 0x4B;
    pub const KEY_DELETE: u8 = 0x4C;
    pub const KEY_END: u8 = 0x4D;
    pub const KEY_PAGE_DOWN: u8 = 0x4E;
    pub const KEY_RIGHT: u8 = 0x4F;
    pub const KEY_LEFT: u8 = 0x50;
    pub const KEY_DOWN: u8 = 0x51;
    pub const KEY_UP: u8 = 0x52;
    
    /// Convert keycode to ASCII character (basic mapping)
    pub fn to_ascii(keycode: u8, shift: bool) -> Option<char> {
        match keycode {
            KEY_A..=KEY_Z => {
                let base = b'a' + (keycode - KEY_A);
                Some(if shift {
                    base.to_ascii_uppercase() as char
                } else {
                    base as char
                })
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
            KEY_SPACE => Some(' '),
            KEY_MINUS => Some(if shift { '_' } else { '-' }),
            KEY_EQUAL => Some(if shift { '+' } else { '=' }),
            KEY_LEFT_BRACKET => Some(if shift { '{' } else { '[' }),
            KEY_RIGHT_BRACKET => Some(if shift { '}' } else { ']' }),
            KEY_BACKSLASH => Some(if shift { '|' } else { '\\' }),
            KEY_SEMICOLON => Some(if shift { ':' } else { ';' }),
            KEY_QUOTE => Some(if shift { '"' } else { '\'' }),
            KEY_GRAVE => Some(if shift { '~' } else { '`' }),
            KEY_COMMA => Some(if shift { '<' } else { ',' }),
            KEY_DOT => Some(if shift { '>' } else { '.' }),
            KEY_SLASH => Some(if shift { '?' } else { '/' }),
            KEY_ENTER => Some('\n'),
            KEY_TAB => Some('\t'),
            _ => None,
        }
    }
}

/// HID device manager for multiple devices
pub struct HidManager {
    /// Active keyboards
    keyboards: heapless::Vec<HidKeyboard, 4>,
    /// Event queue
    event_queue: heapless::Queue<(u8, KeyboardEvent), 32>,
}

impl HidManager {
    /// Create new HID manager
    pub const fn new() -> Self {
        Self {
            keyboards: heapless::Vec::new(),
            event_queue: heapless::Queue::new(),
        }
    }
    
    /// Register a new keyboard
    pub fn register_keyboard(&mut self, keyboard: HidKeyboard) -> Result<usize> {
        let index = self.keyboards.len();
        self.keyboards.push(keyboard)
            .map_err(|_| UsbError::NoResources)?;
        Ok(index)
    }
    
    /// Process report from a keyboard
    pub fn process_keyboard_report(&mut self, keyboard_index: usize, data: &[u8]) {
        if let Some(keyboard) = self.keyboards.get_mut(keyboard_index) {
            if let Some(event) = keyboard.process_report(data) {
                // Queue event with keyboard index
                let _ = self.event_queue.enqueue((keyboard_index as u8, event));
            }
        }
    }
    
    /// Get next event from queue
    pub fn get_next_event(&mut self) -> Option<(u8, KeyboardEvent)> {
        self.event_queue.dequeue()
    }
    
    /// Get keyboard by index
    pub fn get_keyboard(&self, index: usize) -> Option<&HidKeyboard> {
        self.keyboards.get(index)
    }
    
    /// Get mutable keyboard by index
    pub fn get_keyboard_mut(&mut self, index: usize) -> Option<&mut HidKeyboard> {
        self.keyboards.get_mut(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_keyboard_modifiers() {
        let mods = KeyboardModifiers {
            left_ctrl: true,
            left_shift: false,
            left_alt: true,
            left_gui: false,
            right_ctrl: false,
            right_shift: true,
            right_alt: false,
            right_gui: true,
        };
        
        let byte = mods.to_byte();
        assert_eq!(byte, 0x85); // 10000101
        
        let parsed = KeyboardModifiers::from_byte(byte);
        assert_eq!(parsed.left_ctrl, mods.left_ctrl);
        assert_eq!(parsed.right_gui, mods.right_gui);
    }
    
    #[test]
    fn test_keyboard_report() {
        let data = [
            0x02, // Left shift
            0x00, // Reserved
            0x04, // KEY_A
            0x05, // KEY_B
            0x00, 0x00, 0x00, 0x00,
        ];
        
        let report = BootKeyboardReport::from_bytes(&data).unwrap();
        assert!(report.get_modifiers().left_shift);
        assert!(report.is_key_pressed(keycodes::KEY_A));
        assert!(report.is_key_pressed(keycodes::KEY_B));
        assert!(!report.is_key_pressed(keycodes::KEY_C));
    }
    
    #[test]
    fn test_keycode_to_ascii() {
        assert_eq!(keycodes::to_ascii(keycodes::KEY_A, false), Some('a'));
        assert_eq!(keycodes::to_ascii(keycodes::KEY_A, true), Some('A'));
        assert_eq!(keycodes::to_ascii(keycodes::KEY_1, false), Some('1'));
        assert_eq!(keycodes::to_ascii(keycodes::KEY_1, true), Some('!'));
        assert_eq!(keycodes::to_ascii(keycodes::KEY_SPACE, false), Some(' '));
    }
}