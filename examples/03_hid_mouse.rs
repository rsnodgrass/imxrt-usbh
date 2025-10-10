#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;


use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::{HidDevice, MouseReport};

// Hardware addresses for Teensy 4.1
const USB2_BASE: usize = 0x402E_0200; // USB2 (host mode on pins 30/31)
const USBPHY2_BASE: usize = 0x400DA000;
const CCM_BASE: usize = 0x400F_C000;

#[bsp::rt::entry]
fn main() -> ! {
    let _board = board::t41(board::instances());
    // Initialize USB host
    let mut usb = SimpleUsbHost::new(USB2_BASE, USBPHY2_BASE, CCM_BASE)
        .expect("failed to initialize USB host");


    // Wait for mouse to connect
    let device = usb.wait_for_device()
        .expect("failed to enumerate device");


    // Create HID device wrapper
    let mut mouse = HidDevice::from_device(device)
        .expect("not a HID device");

    // Enable boot protocol
    mouse.enable_boot_protocol(&mut usb)
        .expect("failed to enable boot protocol");


    // Set up interrupt transfer for reading mouse reports
    let (mut interrupt_mgr, transfer_id) = mouse.create_polling_manager::<4>(&mut usb)
        .expect("failed to create polling manager");

    // Track cursor position (for demonstration)
    let mut cursor_x: i32 = 0;
    let mut cursor_y: i32 = 0;

    // Main loop: read and display mouse data
    loop {
        // Check for new data
        if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
            // Parse mouse report
            let report = MouseReport::parse(data);

            // Update cursor position
            if report.has_movement() {

            }

            // Display button presses
            if report.buttons.left() {
            }
            if report.buttons.right() {
            }
            if report.buttons.middle() {
            }
        }

        // Small delay to avoid busy-waiting
        cortex_m::asm::delay(60_000); // ~100μs
    }
}
