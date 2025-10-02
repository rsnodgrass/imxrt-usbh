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
//! **Next Step:** Try the working examples (hid_keyboard.rs, etc.)

#![no_main]
#![no_std]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::ehci::{EhciController, Uninitialized};
use log::info;

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
    let mut poller = imxrt_log::log::usbd(
        usb,
        imxrt_log::Interrupts::Enabled,
    ).unwrap();

    info!("\r\n=== USB Host Example 02: EHCI Controller Setup ===");
    poller.poll();

    // STEP 1: Initialize USB PHY (same as example 01)
    info!("Step 1: Initializing USB PHY...");
    poller.poll();

    // Using USB2 PHY for USB Host functionality (Teensy 4.1 USB Host port)
    let mut phy = unsafe {
        // 0x400DA000: USBPHY2 register base (USB Host port on Teensy 4.1)
        // 0x400F_C000: CCM (Clock Control Module) base
        UsbPhy::new(0x400DA000, 0x400F_C000)
    };

    match phy.init_host_mode() {
        Ok(()) => {
            info!("✓ USB PHY initialized");
            poller.poll();
        },
        Err(_) => {
            info!("✗ USB PHY initialization FAILED!");
            poller.poll();
            loop {
                led.toggle();
                cortex_m::asm::delay(60_000_000); // Fast blink = error
                poller.poll();
            }
        }
    }

    // STEP 2: Create EHCI Controller
    info!("");
    info!("Step 2: Creating EHCI Controller...");
    info!("EHCI = Enhanced Host Controller Interface");
    poller.poll();

    let _controller = unsafe {
        // Using USB2 EHCI controller (USB Host port on Teensy 4.1)
        // 0x402E_0400: USB2 EHCI register base
        match EhciController::<8, Uninitialized>::new(0x402E_0400) {
            Ok(controller) => {
                info!("✓ EHCI controller created successfully!");
                poller.poll();
                info!("");
                info!("What we have now:");
                info!("  • USB PHY - Physical layer for USB signaling");
                info!("  • EHCI Controller - Manages USB protocol and transfers");
                poller.poll();
                info!("");
                info!("What's still missing for full USB host:");
                info!("  • Controller initialization and configuration");
                info!("  • DMA memory management setup");
                info!("  • Device enumeration state machine");
                info!("  • Transfer queue management");
                info!("  • Interrupt handling");
                poller.poll();
                info!("");
                info!("See full working examples: hid_keyboard.rs, midi_keyboard.rs");
                poller.poll();
                controller
            },
            Err(_) => {
                info!("✗ EHCI controller creation FAILED!");
                poller.poll();
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000); // Fast blink = error
                    poller.poll();
                }
            }
        }
    };

    // Success! Blink LED slowly
    info!("");
    info!("Success! USB host components ready. LED blinking slowly.");
    poller.poll();

    led.set();
    let mut counter = 0u32;
    loop {
        cortex_m::asm::delay(600_000_000); // Slow blink = success
        led.toggle();
        poller.poll();

        counter += 1;
        if counter % 2 == 0 {
            info!("USB host ready... ({}s)", counter);
            poller.poll();
        }
    }
}
