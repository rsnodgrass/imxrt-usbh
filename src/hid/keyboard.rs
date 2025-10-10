//! HID keyboard support
//!
//! Implements HID Boot Protocol keyboard support (USB HID Spec 1.11, Appendix B.1)

use bitflags::bitflags;

/// HID keyboard report (boot protocol)
///
/// Standard 8-byte keyboard report format:
/// - Byte 0: Modifier keys (Ctrl, Alt, Shift, GUI)
/// - Byte 1: Reserved (OEM use)
/// - Bytes 2-7: Up to 6 simultaneous key presses
///
/// See USB HID Specification 1.11, Appendix B.1
#[derive(Debug, Clone, Copy)]
pub struct KeyboardReport {
    /// Modifier key states
    pub modifiers: KeyModifiers,
    /// Reserved byte
    _reserved: u8,
    /// Active keycodes (up to 6)
    keycodes: [u8; 6],
}

impl KeyboardReport {
    /// Parse report from raw 8-byte data
    ///
    /// # Example
    ///
    /// ```no_run
    /// use imxrt_usbh::hid::KeyboardReport;
    ///
    /// let data: [u8; 8] = [0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00];
    /// let report = KeyboardReport::parse(&data);
    /// // Key 'A' is pressed
    /// ```
    pub fn parse(data: &[u8]) -> Self {
        assert!(data.len() >= 8, "Keyboard report must be at least 8 bytes");

        Self {
            modifiers: KeyModifiers::from_bits_truncate(data[0]),
            _reserved: data[1],
            keycodes: [data[2], data[3], data[4], data[5], data[6], data[7]],
        }
    }

    /// Get iterator over pressed keys
    ///
    /// Returns keycodes for all currently pressed keys (excluding modifiers).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::hid::KeyboardReport;
    /// # let report = KeyboardReport::parse(&[0; 8]);
    /// for key in report.keys_pressed() {
    ///     if let Some(ch) = key.to_ascii() {
    ///         print!("{}", ch);
    ///     }
    /// }
    /// ```
    pub fn keys_pressed(&self) -> impl Iterator<Item = KeyCode> + '_ {
        self.keycodes
            .iter()
            .filter(|&&code| code != 0) // 0x00 = no key
            .map(|&code| KeyCode(code))
    }

    /// Check if specific key is pressed
    pub fn is_key_pressed(&self, key: KeyCode) -> bool {
        self.keycodes.contains(&key.0)
    }

    /// Check if any key is pressed
    pub fn has_keys(&self) -> bool {
        self.keycodes.iter().any(|&code| code != 0)
    }
}

bitflags! {
    /// Keyboard modifier keys
    ///
    /// These are separate from regular keys and can be combined.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct KeyModifiers: u8 {
        /// Left Control
        const LEFT_CTRL  = 0b00000001;
        /// Left Shift
        const LEFT_SHIFT = 0b00000010;
        /// Left Alt
        const LEFT_ALT   = 0b00000100;
        /// Left GUI (Windows/Command key)
        const LEFT_GUI   = 0b00001000;
        /// Right Control
        const RIGHT_CTRL  = 0b00010000;
        /// Right Shift
        const RIGHT_SHIFT = 0b00100000;
        /// Right Alt
        const RIGHT_ALT   = 0b01000000;
        /// Right GUI (Windows/Command key)
        const RIGHT_GUI   = 0b10000000;
    }
}

impl KeyModifiers {
    /// Check if any Ctrl key is pressed
    pub fn ctrl(&self) -> bool {
        self.intersects(Self::LEFT_CTRL | Self::RIGHT_CTRL)
    }

    /// Check if any Shift key is pressed
    pub fn shift(&self) -> bool {
        self.intersects(Self::LEFT_SHIFT | Self::RIGHT_SHIFT)
    }

    /// Check if any Alt key is pressed
    pub fn alt(&self) -> bool {
        self.intersects(Self::LEFT_ALT | Self::RIGHT_ALT)
    }

    /// Check if any GUI key is pressed
    pub fn gui(&self) -> bool {
        self.intersects(Self::LEFT_GUI | Self::RIGHT_GUI)
    }
}

/// HID keyboard keycode
///
/// Standard USB HID keyboard scancodes (Usage Page 0x07).
/// See HID Usage Tables 1.12, Section 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyCode(pub u8);

impl KeyCode {
    // Letter keys (0x04-0x1D)
    pub const A: Self = Self(0x04);
    pub const B: Self = Self(0x05);
    pub const C: Self = Self(0x06);
    pub const D: Self = Self(0x07);
    pub const E: Self = Self(0x08);
    pub const F: Self = Self(0x09);
    pub const G: Self = Self(0x0A);
    pub const H: Self = Self(0x0B);
    pub const I: Self = Self(0x0C);
    pub const J: Self = Self(0x0D);
    pub const K: Self = Self(0x0E);
    pub const L: Self = Self(0x0F);
    pub const M: Self = Self(0x10);
    pub const N: Self = Self(0x11);
    pub const O: Self = Self(0x12);
    pub const P: Self = Self(0x13);
    pub const Q: Self = Self(0x14);
    pub const R: Self = Self(0x15);
    pub const S: Self = Self(0x16);
    pub const T: Self = Self(0x17);
    pub const U: Self = Self(0x18);
    pub const V: Self = Self(0x19);
    pub const W: Self = Self(0x1A);
    pub const X: Self = Self(0x1B);
    pub const Y: Self = Self(0x1C);
    pub const Z: Self = Self(0x1D);

    // Number keys (0x1E-0x27)
    pub const KEY_1: Self = Self(0x1E);
    pub const KEY_2: Self = Self(0x1F);
    pub const KEY_3: Self = Self(0x20);
    pub const KEY_4: Self = Self(0x21);
    pub const KEY_5: Self = Self(0x22);
    pub const KEY_6: Self = Self(0x23);
    pub const KEY_7: Self = Self(0x24);
    pub const KEY_8: Self = Self(0x25);
    pub const KEY_9: Self = Self(0x26);
    pub const KEY_0: Self = Self(0x27);

    // Special keys
    pub const ENTER: Self = Self(0x28);
    pub const ESCAPE: Self = Self(0x29);
    pub const BACKSPACE: Self = Self(0x2A);
    pub const TAB: Self = Self(0x2B);
    pub const SPACE: Self = Self(0x2C);
    pub const MINUS: Self = Self(0x2D);
    pub const EQUALS: Self = Self(0x2E);
    pub const LEFT_BRACKET: Self = Self(0x2F);
    pub const RIGHT_BRACKET: Self = Self(0x30);
    pub const BACKSLASH: Self = Self(0x31);
    pub const SEMICOLON: Self = Self(0x33);
    pub const APOSTROPHE: Self = Self(0x34);
    pub const GRAVE: Self = Self(0x35);
    pub const COMMA: Self = Self(0x36);
    pub const PERIOD: Self = Self(0x37);
    pub const SLASH: Self = Self(0x38);
    pub const CAPS_LOCK: Self = Self(0x39);

    // Function keys
    pub const F1: Self = Self(0x3A);
    pub const F2: Self = Self(0x3B);
    pub const F3: Self = Self(0x3C);
    pub const F4: Self = Self(0x3D);
    pub const F5: Self = Self(0x3E);
    pub const F6: Self = Self(0x3F);
    pub const F7: Self = Self(0x40);
    pub const F8: Self = Self(0x41);
    pub const F9: Self = Self(0x42);
    pub const F10: Self = Self(0x43);
    pub const F11: Self = Self(0x44);
    pub const F12: Self = Self(0x45);

    // Navigation keys
    pub const INSERT: Self = Self(0x49);
    pub const HOME: Self = Self(0x4A);
    pub const PAGE_UP: Self = Self(0x4B);
    pub const DELETE: Self = Self(0x4C);
    pub const END: Self = Self(0x4D);
    pub const PAGE_DOWN: Self = Self(0x4E);
    pub const RIGHT_ARROW: Self = Self(0x4F);
    pub const LEFT_ARROW: Self = Self(0x50);
    pub const DOWN_ARROW: Self = Self(0x51);
    pub const UP_ARROW: Self = Self(0x52);

    /// Convert keycode to ASCII character (lowercase)
    ///
    /// Returns `None` for keys that don't have ASCII representation.
    ///
    /// # Example
    ///
    /// ```
    /// use imxrt_usbh::hid::KeyCode;
    ///
    /// assert_eq!(KeyCode::A.to_ascii(), Some('a'));
    /// assert_eq!(KeyCode::KEY_1.to_ascii(), Some('1'));
    /// assert_eq!(KeyCode::SPACE.to_ascii(), Some(' '));
    /// assert_eq!(KeyCode::F1.to_ascii(), None);
    /// ```
    pub fn to_ascii(&self) -> Option<char> {
        match self.0 {
            0x04..=0x1D => Some((b'a' + (self.0 - 0x04)) as char), // a-z
            0x1E..=0x26 => Some((b'1' + (self.0 - 0x1E)) as char), // 1-9
            0x27 => Some('0'),
            0x28 => Some('\n'),  // Enter
            0x2C => Some(' '),   // Space
            0x2D => Some('-'),   // Minus
            0x2E => Some('='),   // Equals
            0x2F => Some('['),   // Left bracket
            0x30 => Some(']'),   // Right bracket
            0x31 => Some('\\'),  // Backslash
            0x33 => Some(';'),   // Semicolon
            0x34 => Some('\''),  // Apostrophe
            0x35 => Some('`'),   // Grave
            0x36 => Some(','),   // Comma
            0x37 => Some('.'),   // Period
            0x38 => Some('/'),   // Slash
            _ => None,
        }
    }

    /// Convert keycode to ASCII character with Shift modifier
    ///
    /// # Example
    ///
    /// ```
    /// use imxrt_usbh::hid::KeyCode;
    ///
    /// assert_eq!(KeyCode::A.to_ascii_shifted(), Some('A'));
    /// assert_eq!(KeyCode::KEY_1.to_ascii_shifted(), Some('!'));
    /// assert_eq!(KeyCode::COMMA.to_ascii_shifted(), Some('<'));
    /// ```
    pub fn to_ascii_shifted(&self) -> Option<char> {
        match self.0 {
            0x04..=0x1D => Some((b'A' + (self.0 - 0x04)) as char), // A-Z
            0x1E => Some('!'),
            0x1F => Some('@'),
            0x20 => Some('#'),
            0x21 => Some('$'),
            0x22 => Some('%'),
            0x23 => Some('^'),
            0x24 => Some('&'),
            0x25 => Some('*'),
            0x26 => Some('('),
            0x27 => Some(')'),
            0x2D => Some('_'),   // Minus -> Underscore
            0x2E => Some('+'),   // Equals -> Plus
            0x2F => Some('{'),   // [ -> {
            0x30 => Some('}'),   // ] -> }
            0x31 => Some('|'),   // \ -> |
            0x33 => Some(':'),   // ; -> :
            0x34 => Some('"'),   // ' -> "
            0x35 => Some('~'),   // ` -> ~
            0x36 => Some('<'),   // , -> <
            0x37 => Some('>'),   // . -> >
            0x38 => Some('?'),   // / -> ?
            _ => self.to_ascii(), // Fall back to unshifted
        }
    }

    /// Get raw keycode value
    pub fn raw(&self) -> u8 {
        self.0
    }
}
