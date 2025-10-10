#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::{HidDevice, KeyboardReport};

// Hardware addresses for Teensy 4.1
const USB2_BASE: usize = 0x402E_0200; // USB2 (host mode on pins 30/31)
const USBPHY2_BASE: usize = 0x400DA000;
const CCM_BASE: usize = 0x400F_C000;

#[bsp::rt::entry]
fn main() -> ! {
    let _board = board::t41(board::instances());
    // Initialize USB host (handles PHY, DMA, controller setup)
    let mut usb = SimpleUsbHost::new(USB2_BASE, USBPHY2_BASE, CCM_BASE)
        .expect("failed to initialize USB host");


    // Wait for keyboard to connect
    let device = usb.wait_for_device()
        .expect("failed to enumerate device");


    // Create HID device wrapper
    let mut keyboard = HidDevice::from_device(device)
        .expect("not a HID device");

    // Enable boot protocol (required for reading keyboard data)
    keyboard.enable_boot_protocol(&mut usb)
        .expect("failed to enable boot protocol");


    // Set up interrupt transfer for reading keyboard reports
    let (mut interrupt_mgr, transfer_id) = keyboard.create_polling_manager::<4>(&mut usb)
        .expect("failed to create polling manager");

    // Main loop: read and print key presses
    loop {
        // Check for new data
        if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
            // Parse keyboard report
            let report = KeyboardReport::parse(data);

            // Print pressed keys
            for key in report.keys_pressed() {
                if let Some(ch) = key.to_ascii() {
                }
            }
        }

        // Small delay to avoid busy-waiting
        cortex_m::asm::delay(60_000); // ~100μs
    }
}
