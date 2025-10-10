#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::HidDevice;
use imxrt_log as logging;
use imxrt_ral as ral;
use imxrt_hal as hal;

// Hardware addresses for Teensy 4.1
const USB2_BASE: usize = 0x402E_0200; // USB2 (host mode on pins 30/31)
const USBPHY2_BASE: usize = 0x400DA000;
const CCM_BASE: usize = 0x400F_C000;

/// Generic gamepad report structure
///
/// Most USB gamepads follow this general layout:
/// - Buttons (bitmap)
/// - Left stick X/Y
/// - Right stick X/Y (optional)
/// - Triggers (optional)
///
/// Note: Actual report format varies by manufacturer.
/// For production use, parse the HID report descriptor.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GamepadReport {
    pub buttons: u16,  // button bitmap
    pub left_x: u8,    // left stick X (0-255, 128=center)
    pub left_y: u8,    // left stick Y (0-255, 128=center)
    pub right_x: u8,   // right stick X (0-255, 128=center)
    pub right_y: u8,   // right stick Y (0-255, 128=center)
    pub triggers: u16, // trigger values
}

impl GamepadReport {
    /// Parse gamepad report from raw HID data
    ///
    /// This is a simplified parser for common gamepad formats.
    /// Real gamepads may have different report structures.
    pub fn parse(data: &[u8]) -> Self {
        if data.len() >= 8 {
            Self {
                buttons: u16::from_le_bytes([data[0], data[1]]),
                left_x: data[2],
                left_y: data[3],
                right_x: if data.len() > 4 { data[4] } else { 128 },
                right_y: if data.len() > 5 { data[5] } else { 128 },
                triggers: if data.len() > 7 {
                    u16::from_le_bytes([data[6], data[7]])
                } else {
                    0
                },
            }
        } else {
            // Fallback for short reports
            Self {
                buttons: 0,
                left_x: 128,
                left_y: 128,
                right_x: 128,
                right_y: 128,
                triggers: 0,
            }
        }
    }

    /// Check if button is pressed
    pub fn is_button_pressed(&self, button: u8) -> bool {
        if button < 16 {
            (self.buttons & (1 << button)) != 0
        } else {
            false
        }
    }

    /// Get left stick position as centered values (-128 to 127)
    pub fn left_stick(&self) -> (i8, i8) {
        (
            (self.left_x as i16 - 128) as i8,
            (self.left_y as i16 - 128) as i8,
        )
    }

    /// Get right stick position as centered values (-128 to 127)
    pub fn right_stick(&self) -> (i8, i8) {
        (
            (self.right_x as i16 - 128) as i8,
            (self.right_y as i16 - 128) as i8,
        )
    }
}

#[bsp::rt::entry]
fn main() -> ! {
    let _board = board::t41(board::instances());

    // Initialize USB CDC logging (USB1 - micro USB port)
    let usb_instances = hal::usbd::Instances {
        usb: unsafe { ral::usb::USB1::instance() },
        usbnc: unsafe { ral::usbnc::USBNC1::instance() },
        usbphy: unsafe { ral::usbphy::USBPHY1::instance() },
    };

    let mut poller = logging::log::usbd(usb_instances, logging::Interrupts::Disabled)
        .expect("failed to initialize USB logging");

    // Small delay for USB CDC to enumerate
    cortex_m::asm::delay(60_000_000);

    log::info!("");
    log::info!("=== USB Gamepad Example ===");
    log::info!("Waiting for gamepad on USB2 (pins 30/31)...");
    poller.poll();

    // Initialize USB host (USB2 - pins 30/31)
    let mut usb = SimpleUsbHost::new(USB2_BASE, USBPHY2_BASE, CCM_BASE)
        .expect("failed to initialize USB host");

    log::info!("USB host initialized");
    poller.poll();

    // Wait for device to connect
    let device = usb.wait_for_device()
        .expect("failed to enumerate device");

    log::info!("Device connected: VID={:04X} PID={:04X}",
               device.vendor_id(), device.product_id());
    poller.poll();

    // Create HID device wrapper
    let mut gamepad = HidDevice::from_device(device)
        .expect("not a HID device");

    // Enable boot protocol (some gamepads support this)
    // Note: Most gamepads use report protocol, so this may not apply
    if let Err(_e) = gamepad.enable_boot_protocol(&mut usb) {
        log::warn!("Boot protocol not supported (expected for gamepads)");
    } else {
        log::info!("Boot protocol enabled");
    }
    poller.poll();

    // Set up interrupt transfer for reading gamepad reports
    let (mut interrupt_mgr, transfer_id) = gamepad.create_polling_manager::<4>(&mut usb)
        .expect("failed to create polling manager");

    log::info!("Gamepad ready! Press buttons or move sticks...");
    log::info!("");
    poller.poll();

    let mut last_buttons: u16 = 0;
    let mut last_left_x: i8 = 0;
    let mut last_left_y: i8 = 0;

    // Main loop: read and display gamepad input
    loop {
        // Check for new data
        if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
            // Parse gamepad report
            let report = GamepadReport::parse(data);

            // Display button changes
            if report.buttons != last_buttons {
                for i in 0..16 {
                    if report.is_button_pressed(i) && (last_buttons & (1 << i)) == 0 {
                        log::info!("Button {} pressed", i);
                    }
                }
                last_buttons = report.buttons;
                poller.poll();
            }

            // Display stick movements (with deadzone to reduce spam)
            let (left_x, left_y) = report.left_stick();
            if (left_x - last_left_x).abs() > 10 || (left_y - last_left_y).abs() > 10 {
                log::info!("Left stick: X={}, Y={}", left_x, left_y);
                last_left_x = left_x;
                last_left_y = left_y;
                poller.poll();
            }
        }

        // Poll USB CDC to drive serial output
        poller.poll();

        // Small delay to avoid busy-waiting
        cortex_m::asm::delay(60_000); // ~100μs
    }
}
