//! Example 02: Basic EHCI Controller Setup
//!
//! This example shows how to:
//! - Initialize the USB PHY (from Example 01)
//! - Create an EHCI controller instance
//! - Understand the basic USB host components
//!
//! This is educational - it shows the core components but doesn't
//! implement full device enumeration (that's complex and shown in
//! the working examples like hid_keyboard.rs).
//!
//! **Learning Objectives:**
//! - Understand EHCI controller creation
//! - See the relationship between PHY and controller
//! - Learn about the basic USB host architecture
//!
//! **Hardware Requirements:**
//! - Teensy 4.0 or 4.1
//! - USB host connections
//!
//! **To run:** Flash to your Teensy. Open serial monitor at 115200 baud on pins 0/1.
//! LED blinks slowly if successful, rapidly if failed.
//!
//! **Next Step:** Try the working examples (hid_keyboard.rs, etc.)

#![no_main]
#![no_std]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::ehci::{EhciController, Uninitialized};
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

    print("\r\n=== USB Host Example 02: EHCI Controller Setup ===\r\n");

    // STEP 1: Initialize USB PHY (same as example 01)
    print("Step 1: Initializing USB PHY...\r\n");
    let mut phy = unsafe { UsbPhy::new(0x400D_9000, 0x400F_C000) };

    match phy.init_host_mode() {
        Ok(()) => {
            print("✓ USB PHY initialized\r\n");
        },
        Err(_) => {
            print("✗ USB PHY initialization FAILED!\r\n");
            loop {
                led.toggle();
                cortex_m::asm::delay(60_000_000); // Fast blink = error
            }
        }
    }

    // STEP 2: Create EHCI Controller
    print("\r\nStep 2: Creating EHCI Controller...\r\n");
    print("EHCI = Enhanced Host Controller Interface\r\n");

    let _controller = unsafe {
        match EhciController::<8, Uninitialized>::new(0x402E_0140) {
            Ok(controller) => {
                print("✓ EHCI controller created successfully!\r\n");
                print("\r\nWhat we have now:\r\n");
                print("  • USB PHY - Physical layer for USB signaling\r\n");
                print("  • EHCI Controller - Manages USB protocol and transfers\r\n");
                print("\r\nWhat's still missing for full USB host:\r\n");
                print("  • Controller initialization and configuration\r\n");
                print("  • DMA memory management setup\r\n");
                print("  • Device enumeration state machine\r\n");
                print("  • Transfer queue management\r\n");
                print("  • Interrupt handling\r\n");
                print("\r\nSee full working examples: hid_keyboard.rs, midi_keyboard.rs\r\n");
                controller
            },
            Err(_) => {
                print("✗ EHCI controller creation FAILED!\r\n");
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000); // Fast blink = error
                }
            }
        }
    };

    // Success! Blink LED slowly
    print("\r\nSuccess! USB host components ready. LED blinking slowly.\r\n");

    led.set();
    loop {
        cortex_m::asm::delay(600_000_000); // Slow blink = success
        led.toggle();
    }
}
