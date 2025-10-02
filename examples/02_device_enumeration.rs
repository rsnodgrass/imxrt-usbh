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
//! **To run:** Flash to your Teensy. LED blinks slowly if successful, rapidly if failed.
//!
//! **Next Step:** Try the working examples (hid_keyboard.rs, etc.)

#![no_main]
#![no_std]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::ehci::{EhciController, Uninitialized};

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        ..
    } = board::t40(board::instances());

    let led = board::led(&mut gpio2, pins.p13);

    // STEP 1: Initialize USB PHY (same as example 01)
    let mut phy = unsafe { UsbPhy::new(0x400D_9000, 0x400F_C000) };

    match phy.init_host_mode() {
        Ok(()) => {
            // PHY initialized successfully
        },
        Err(_) => {
            // Failed - blink LED rapidly
            loop {
                led.toggle();
                cortex_m::asm::delay(60_000_000); // Fast blink = error
            }
        }
    }

    // STEP 2: Create EHCI Controller
    // EHCI = Enhanced Host Controller Interface
    // This manages all USB transfers and device communication
    let _controller = unsafe {
        match EhciController::<8, Uninitialized>::new(0x402E_0140) {
            Ok(controller) => {
                // Success! We now have:
                //   • USB PHY - Physical layer for USB signaling
                //   • EHCI Controller - Manages USB protocol and transfers
                //
                // What's still missing for full USB host:
                //   • Controller initialization and configuration
                //   • DMA memory management setup
                //   • Device enumeration state machine
                //   • Transfer queue management
                //   • Interrupt handling
                //
                // These complex topics are shown in working examples like:
                //   • hid_keyboard.rs - Complete HID keyboard support
                //   • mass_storage.rs - USB flash drive communication
                //   • midi_keyboard.rs - MIDI device handling
                controller
            },
            Err(_) => {
                // Failed - blink LED rapidly
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000); // Fast blink = error
                }
            }
        }
    };

    // Success! Blink LED slowly
    led.set();
    loop {
        cortex_m::asm::delay(600_000_000); // Slow blink = success
        led.toggle();
    }
}
