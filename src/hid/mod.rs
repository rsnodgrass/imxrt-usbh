//! HID (Human Interface Device) support
//!
//! This module provides high-level abstractions for working with HID devices
//! such as keyboards, mice, and gamepads.
//!
//! # Supported Devices
//!
//! - **Keyboards**: Boot protocol support with full keycode mappings
//! - **Mice**: Boot protocol with button and movement tracking
//! - **Gamepads**: Generic HID report parsing
//!
//! # Boot Protocol vs Report Protocol
//!
//! ## Boot Protocol (Simplified)
//! - Fixed report format (8 bytes for keyboard, 3-4 bytes for mouse)
//! - Works with any boot-compatible HID device
//! - No report descriptor parsing needed
//! - **Recommended for most applications**
//!
//! ## Report Protocol (Advanced)
//! - Custom report formats defined by device
//! - Requires parsing report descriptors
//! - Supports custom HID devices
//! - Use low-level API for this
//!
//! # Quick Start
//!
//! ## Keyboard Example
//!
//! ```no_run
//! use imxrt_usbh::simple::SimpleUsbHost;
//! use imxrt_usbh::hid::{HidDevice, KeyboardReport};
//!
//! // Initialize and wait for keyboard
//! let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
//! let device = usb.wait_for_device()?;
//! let mut kbd = HidDevice::from_device(device)?;
//! kbd.enable_boot_protocol(&mut usb)?;
//!
//! // Read keys
//! loop {
//!     if let Some(data) = kbd.poll_report(&mut usb)? {
//!         let report = KeyboardReport::parse(data);
//!         for key in report.keys_pressed() {
//!             if let Some(ch) = key.to_ascii() {
//!                 print!("{}", ch);
//!             }
//!         }
//!     }
//! }
//! # Ok::<(), imxrt_usbh::UsbError>(())
//! ```
//!
//! ## Mouse Example
//!
//! ```no_run
//! use imxrt_usbh::simple::SimpleUsbHost;
//! use imxrt_usbh::hid::{HidDevice, MouseReport};
//!
//! let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
//! let device = usb.wait_for_device()?;
//! let mut mouse = HidDevice::from_device(device)?;
//! mouse.enable_boot_protocol(&mut usb)?;
//!
//! loop {
//!     if let Some(data) = mouse.poll_report(&mut usb)? {
//!         let report = MouseReport::parse(data);
//!         if report.has_movement() {
//!             println!("X: {} Y: {} Wheel: {}", report.x, report.y, report.wheel);
//!         }
//!         if report.buttons.left() {
//!             println!("Left button pressed!");
//!         }
//!     }
//! }
//! # Ok::<(), imxrt_usbh::UsbError>(())
//! ```
//!
//! # Reference
//!
//! - USB HID Specification 1.11: <https://www.usb.org/document-library/device-class-definition-hid-111>
//! - HID Usage Tables 1.12: <https://usb.org/document-library/hid-usage-tables-15>

pub mod constants;
pub mod device;
pub mod keyboard;
pub mod mouse;

pub use constants::*;
pub use device::HidDevice;
pub use keyboard::{KeyCode, KeyModifiers, KeyboardReport};
pub use mouse::{MouseButtons, MouseReport};
