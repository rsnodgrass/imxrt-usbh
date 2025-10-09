//! Example 01: Basic USB Host Initialization
//!
//! This is the simplest possible USB host example. It shows how to:
//! - Initialize the USB PHY hardware
//! - Handle basic hardware initialization
//! - Use the core library components
//!
//! This example does NOT connect to devices - it just initializes the hardware.
//!
//! **Learning Objectives:**
//! - Understand USB host hardware initialization
//! - Learn about the USB PHY (Physical Layer)
//! - See basic error handling patterns
//!
//! **Hardware Requirements:**
//! - **Teensy 4.1** board (USB2 host port required)
//! - USB cable connected to micro USB port (for USB CDC logging)
//! - USB Host device connected to USB2 port (5-pin header)
//!
//! **To run:**
//! 1. Flash to your Teensy 4.1
//! 2. Open serial monitor on the micro USB port (USB CDC)
//! 3. LED blinks slowly if successful, rapidly if failed
//!
//! **Note:** USB2 is used for USB Host, USB1 (micro USB) is used for CDC logging.
//!
//! **Next Step:** Try `02_device_enumeration.rs` to add device detection.

#![no_main]
#![no_std]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_usbh::phy::UsbPhy;
use log::info;

/// defmt timestamp function (required by library's defmt usage)
#[no_mangle]
fn _defmt_timestamp() -> u64 {
    0 // Simple timestamp - could use DWT cycle counter if needed
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        mut dma,
        ..
    } = board::t40(board::instances());

    let led = board::led(&mut gpio2, pins.p13);

    // Set up USB CDC logging on USB1 (micro USB port)
    let mut poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

    info!("\r\n=== USB Host Example 01: PHY Initialization ===");
    info!("Initializing USB PHY...");
    poller.poll();

    // STEP 1: Initialize USB PHY
    // The USB PHY handles low-level signaling and must be set up first
    // Using USB2 PHY for USB Host functionality (Teensy 4.1 USB Host port)
    let mut phy = unsafe {
        // These addresses are specific to i.MX RT1062:
        // - 0x400DA000: USBPHY2 register base (USB Host port on Teensy 4.1)
        // - 0x400F_C000: CCM (Clock Control Module) base
        // Note: USB1 (0x400D9000) is used for device mode (USB CDC logging)
        UsbPhy::new(0x400DA000, 0x400F_C000)
    };

    // STEP 2: Initialize PHY for host mode
    match phy.init_host_mode() {
        Ok(()) => {
            led.set();
            info!("✓ USB PHY initialized successfully!");
            poller.poll();
            info!("");
            info!("What happened:");
            info!("  • USB PLL configured for 480 MHz");
            info!("  • PHY calibration completed");
            info!("  • Host mode enabled");
            info!("  • Ready for EHCI controller setup");
            info!("");
            info!("Success! USB PHY ready. LED blinking slowly.");
            poller.poll();

            let mut counter = 0u32;
            loop {
                cortex_m::asm::delay(300_000_000); // Slow blink = success
                led.toggle();
                poller.poll();

                counter += 1;
                if counter % 10 == 0 {
                    info!("USB Host PHY running... ({}s)", counter / 2);
                    poller.poll();
                }
            }
        }
        Err(_) => {
            info!("✗ USB PHY initialization FAILED!");
            info!("LED blinking rapidly to indicate error.");
            poller.poll();

            loop {
                led.toggle();
                cortex_m::asm::delay(60_000_000); // Fast blink = error
                poller.poll();
            }
        }
    }
}
