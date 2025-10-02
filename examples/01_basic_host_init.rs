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
//! - Teensy 4.0 or 4.1 board
//! - USB host connections on pins 30/31
//! - 5V power for VBUS (through switching circuit)
//!
//! **To run:** Flash to your Teensy. Open serial monitor at 115200 baud on pins 0/1.
//! LED blinks slowly if successful, rapidly if failed.
//!
//! **Next Step:** Try `02_device_enumeration.rs` to add device detection.

#![no_main]
#![no_std]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::phy::UsbPhy;
use cortex_m::prelude::_embedded_hal_serial_Write as _;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        lpuart6,
        ..
    } = board::t40(board::instances());

    let led = board::led(&mut gpio2, pins.p13);

    // Set up serial output on pins 0/1
    let mut serial = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

    // Helper to print strings
    let mut print = |s: &str| {
        for byte in s.bytes() {
            nb::block!(serial.write(byte)).ok();
        }
    };

    print("\r\n=== USB Host Example 01: PHY Initialization ===\r\n");
    print("Initializing USB PHY...\r\n");

    // STEP 1: Initialize USB PHY
    // The USB PHY handles low-level signaling and must be set up first
    let mut phy = unsafe {
        // These addresses are specific to i.MX RT1062:
        // - 0x400D_9000: USBPHY1 register base
        // - 0x400F_C000: CCM (Clock Control Module) base
        UsbPhy::new(0x400D_9000, 0x400F_C000)
    };

    // STEP 2: Initialize PHY for host mode
    match phy.init_host_mode() {
        Ok(()) => {
            print("✓ USB PHY initialized successfully!\r\n");
            print("\r\nWhat happened:\r\n");
            print("  • USB PLL configured for 480 MHz\r\n");
            print("  • PHY calibration completed\r\n");
            print("  • Host mode enabled\r\n");
            print("  • Ready for EHCI controller setup\r\n");
            print("\r\nSuccess! USB PHY ready. LED blinking slowly.\r\n");

            led.set();
            loop {
                cortex_m::asm::delay(300_000_000); // Slow blink = success
                led.toggle();
            }
        },
        Err(_) => {
            print("✗ USB PHY initialization FAILED!\r\n");
            print("LED blinking rapidly to indicate error.\r\n");

            loop {
                led.toggle();
                cortex_m::asm::delay(60_000_000); // Fast blink = error
            }
        }
    }
}
