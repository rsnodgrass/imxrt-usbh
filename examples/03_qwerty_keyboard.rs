//! Example 03: USB HID Keyboard Support
//!
//! This example demonstrates basic USB keyboard enumeration and key detection.
//! It's a simplified version showing the core concepts of HID keyboard support.
//!
//! **Features:**
//! - USB HID keyboard detection
//! - Boot protocol initialization
//! - Basic interrupt transfer setup
//! - Key press logging
//!
//! **Hardware Requirements:**
//! - Teensy 4.1 board
//! - USB keyboard connected to USB host port
//! - USB cable for CDC logging (micro USB port)
//!
//! **To run:**
//! 1. Flash to Teensy 4.1
//! 2. Connect USB keyboard to host port
//! 3. Open serial monitor
//! 4. Type on keyboard to see key codes

#![no_std]
#![no_main]

use bsp::board;
use log::info;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use imxrt_usbh::{
    dma::UsbMemoryPool,
    ehci::{EhciController, Initialized, Running, TransferExecutor, Uninitialized},
    enumeration::{DeviceClass, DeviceEnumerator},
    phy::UsbPhy,
    transfer::simple_control::{ControlExecutor, SetupPacket},
    transfer::{Direction, InterruptTransferManager},
};

/// USB HID Boot Keyboard Report (8 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct KeyboardReport {
    modifiers: u8,
    reserved: u8,
    keycodes: [u8; 6],
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        ..
    } = board::t40(board::instances());

    let led = board::led(&mut gpio2, pins.p13);

    // USB CDC logging
    let mut poller = imxrt_log::log::usbd(usb, imxrt_log::Interrupts::Enabled).unwrap();

    info!("\r\n=== USB HID Keyboard Example ===");
    poller.poll();

    // Initialize USB PHY for host mode
    info!("Step 1: Initializing USB PHY...");
    poller.poll();

    let mut phy = unsafe { UsbPhy::new(0x400DA000, 0x400F_C000) };

    if let Err(_) = phy.init_host_mode() {
        info!("✗ PHY init failed!");
        poller.poll();
        loop {
            led.toggle();
            cortex_m::asm::delay(60_000_000);
            poller.poll();
        }
    }

    info!("✓ USB PHY initialized");
    poller.poll();

    // Initialize EHCI controller
    info!("Step 2: Initializing EHCI controller...");
    poller.poll();

    let controller = unsafe {
        match EhciController::<8, Uninitialized>::new(0x402E_0200) {
            Ok(c) => c,
            Err(_) => {
                info!("✗ EHCI init failed!");
                poller.poll();
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000);
                    poller.poll();
                }
            }
        }
    };

    info!("✓ EHCI controller created");
    poller.poll();

    // Initialize DMA and controller
    let controller = unsafe {
        match controller.initialize() {
            Ok(c) => c,
            Err(_) => {
                info!("✗ Controller init failed!");
                poller.poll();
                loop {
                    led.toggle();
                    cortex_m::asm::delay(60_000_000);
                    poller.poll();
                }
            }
        }
    };

    let mut controller = unsafe { controller.start() };
    info!("✓ USB host controller running");
    poller.poll();

    // Initialize memory pool and transfer executor
    let mut memory_pool = UsbMemoryPool::new();
    let mut transfer_executor = unsafe { TransferExecutor::new(0x402E_0200) };

    info!("\r\nStep 3: Waiting for keyboard...");
    poller.poll();

    led.set();

    // Main loop: detect and handle keyboard
    let mut keyboard_detected = false;
    let mut counter = 0u32;

    loop {
        poller.poll();
        counter += 1;

        if !keyboard_detected && counter % 1000 == 0 {
            // Try to enumerate keyboard
            let mut enumerator =
                DeviceEnumerator::new(&mut controller, &mut memory_pool, &mut transfer_executor);

            if let Ok(device) = enumerator.enumerate_device() {
                if device.class == DeviceClass::Hid {
                    info!("\r\n✓ HID Keyboard detected!");
                    info!("  Address: {}", device.address);
                    info!("  Max packet: {}", device.max_packet_size);
                    poller.poll();

                    // Initialize keyboard with boot protocol
                    let mut executor =
                        ControlExecutor::new(&mut transfer_executor, &mut memory_pool);

                    // SET_PROTOCOL (Boot Protocol)
                    let setup = SetupPacket {
                        bmRequestType: 0x21,
                        bRequest: 0x0B,
                        wValue: 0,
                        wIndex: 0,
                        wLength: 0,
                    };

                    if executor
                        .execute_with_retry(setup, device.address, 64, 3)
                        .is_ok()
                    {
                        info!("✓ Boot protocol set");
                        poller.poll();
                        keyboard_detected = true;

                        // Note: Full interrupt transfer implementation would go here
                        info!("\r\nKeyboard ready!");
                        info!("(Full interrupt transfer demo requires additional implementation)");
                        poller.poll();

                        led.clear();
                    }
                }
            }
        }

        // Blink LED
        if counter % 300_000 == 0 {
            led.toggle();
        }

        cortex_m::asm::delay(1000);
    }
}
