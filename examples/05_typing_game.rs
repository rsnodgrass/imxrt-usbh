//! Interactive USB Keyboard Typing Speed Test
//!
//! # Overview
//!
//! This example demonstrates a complete end-to-end USB workflow by implementing
//! an engaging typing speed test. It showcases the simple API in one cohesive,
//! interactive experience.
//!
//! # Game Description
//!
//! A 30-second typing speed test that measures your words-per-minute (WPM) and
//! accuracy. Type the displayed phrase as quickly and accurately as possible.
//!
//! **Test Flow:**
//! 1. Connect USB keyboard to Teensy host port (pins 30/31)
//! 2. Keyboard auto-detected and enumerated
//! 3. Test phrase displayed (standard pangram)
//! 4. Press SPACE to start countdown
//! 5. Type as fast and accurately as you can for 30 seconds
//! 6. Real-time visual feedback (dots for correct, X for errors)
//! 7. Final results with WPM, accuracy, and performance rating
//!
//! **Performance Ratings:**
//! - *** Excellent: 60+ WPM at 95%+ accuracy
//! - ** Great: 40+ WPM at 90%+ accuracy
//! - * Good: 20+ WPM at 80%+ accuracy
//! - Keep Practicing: Below thresholds
//!
//! # Hardware Requirements
//!
//! - Teensy 4.1 board
//! - USB host cable (pins 30/31)
//! - USB keyboard
//! - USB cable for serial output (micro USB port)
//!
//! # Usage
//!
//! 1. Flash to Teensy 4.1
//! 2. Connect USB keyboard to host port (pins 30/31)
//! 3. Open serial monitor (115200 baud)
//! 4. Follow on-screen instructions
//! 5. Press SPACE to start test
//! 6. Type the displayed text as quickly and accurately as possible
//! 7. View your results after 30 seconds
//! 8. Reset Teensy to try again
//!
//! # What This Demonstrates
//!
//! - Simple API usage (SimpleUsbHost, HidDevice)
//! - Automatic device enumeration
//! - HID boot protocol configuration
//! - Interrupt transfer setup and management
//! - Real-time keyboard input processing
//! - ASCII keycode conversion
//! - Timing and statistics tracking
//! - User-friendly error handling

#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;

use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::{HidDevice, KeyboardReport};
use imxrt_log as logging;
use imxrt_ral as ral;
use imxrt_hal as hal;

use heapless::String;

// Hardware addresses for Teensy 4.1
const USB2_BASE: usize = 0x402E_0200; // USB2 (host mode on pins 30/31)
const USBPHY2_BASE: usize = 0x400DA000;
const CCM_BASE: usize = 0x400F_C000;

/// Test phrase for typing speed measurement
const TEST_PHRASE: &str = "the quick brown fox jumps over the lazy dog";

/// Test duration in microseconds (30 seconds)
const TEST_DURATION_US: u32 = 30_000_000;

/// CPU frequency for timing calculations
const CPU_FREQ_MHZ: u32 = 600;

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
    log::info!("========================================================");
    log::info!("    USB Keyboard Typing Speed Test - 30 Seconds      ");
    log::info!("========================================================");
    log::info!("");
    log::info!("Waiting for keyboard on USB2 (pins 30/31)...");
    poller.poll();

    // Initialize USB host (USB2 - pins 30/31)
    let mut usb = SimpleUsbHost::new(USB2_BASE, USBPHY2_BASE, CCM_BASE)
        .expect("failed to initialize USB host");

    // Wait for keyboard to connect
    let device = usb.wait_for_device()
        .expect("failed to enumerate device");

    log::info!("Keyboard connected!");
    poller.poll();

    // Create HID device wrapper
    let mut keyboard = HidDevice::from_device(device)
        .expect("not a HID device");

    // Enable boot protocol
    keyboard.enable_boot_protocol(&mut usb)
        .expect("failed to enable boot protocol");

    // Set up interrupt transfer for reading keyboard reports
    let (mut interrupt_mgr, transfer_id) = keyboard.create_polling_manager::<4>(&mut usb)
        .expect("failed to create polling manager");

    log::info!("");
    log::info!("Type this phrase as fast and accurately as you can:");
    log::info!("  \"{}\"", TEST_PHRASE);
    log::info!("");
    log::info!("Press SPACEBAR to start the 30-second timer...");
    poller.poll();

    // Wait for spacebar to start
    loop {
        if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
            let report = KeyboardReport::parse(data);

            let mut found_space = false;
            for key in report.keys_pressed() {
                if key.0 == 0x2C { // SPACE
                    found_space = true;
                    break;
                }
            }

            if found_space {
                log::info!("");
                log::info!("Starting in 3...");
                poller.poll();
                cortex_m::asm::delay(600_000_000);
                log::info!("2...");
                poller.poll();
                cortex_m::asm::delay(600_000_000);
                log::info!("1...");
                poller.poll();
                cortex_m::asm::delay(600_000_000);
                log::info!("GO!");
                log::info!("");
                poller.poll();
                break;
            }
        }

        poller.poll();
        cortex_m::asm::delay(60_000); // ~100μs
    }

    // Test variables
    let mut typed_text: String<128> = String::new();
    let mut total_keypresses = 0u32;
    let mut correct_keys = 0u32;
    let start_time = cortex_m::peripheral::DWT::cycle_count();
    let mut last_keycode = 0u8;

    // Main test loop
    loop {
        let elapsed_cycles = cortex_m::peripheral::DWT::cycle_count()
            .wrapping_sub(start_time);
        let elapsed_us = elapsed_cycles / CPU_FREQ_MHZ;

        // Check if time is up
        if elapsed_us >= TEST_DURATION_US {
            break;
        }

        // Check for new key presses
        if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
            let report = KeyboardReport::parse(data);

            for key in report.keys_pressed() {
                // Debounce: only count key if different from last
                if key.0 != last_keycode && key.0 != 0 {
                    last_keycode = key.0;

                    if let Some(ch) = key.to_ascii() {
                        total_keypresses += 1;

                        // Check if typed character is correct
                        let position = typed_text.len();
                        if position < TEST_PHRASE.len() {
                            let expected = TEST_PHRASE.as_bytes()[position] as char;
                            if ch == expected {
                                correct_keys += 1;
                                let _ = typed_text.push(ch);
                            }
                        }
                    }
                }
            }

            // Reset last_keycode if no keys pressed (for debounce)
            if report.keys_pressed().count() == 0 {
                last_keycode = 0;
            }
        }

        poller.poll();
        cortex_m::asm::delay(60_000); // ~100μs
    }

    // Calculate results
    let test_duration_seconds = TEST_DURATION_US / 1_000_000;
    let words_typed = typed_text.len() / 5; // Standard: 5 chars = 1 word
    let wpm = (words_typed * 60) / test_duration_seconds as usize;
    let accuracy = if total_keypresses > 0 {
        (correct_keys * 100) / total_keypresses
    } else {
        0
    };

    // Display results
    log::info!("");
    log::info!("");
    log::info!("========================================================");
    log::info!("                    Test Complete!                   ");
    log::info!("========================================================");
    log::info!("");
    log::info!("Results:");
    log::info!("  Words per minute: {} WPM", wpm);
    log::info!("  Accuracy: {}%", accuracy);
    log::info!("  Characters typed: {}/{}", typed_text.len(), TEST_PHRASE.len());
    log::info!("  Correct keypresses: {}/{}", correct_keys, total_keypresses);
    log::info!("");

    // Performance rating
    let rating = if wpm >= 60 && accuracy >= 95 {
        "*** EXCELLENT! You're a typing master!"
    } else if wpm >= 40 && accuracy >= 90 {
        "** GREAT JOB! Very impressive speed and accuracy."
    } else if wpm >= 20 && accuracy >= 80 {
        "* GOOD WORK! Keep practicing to improve further."
    } else {
        "KEEP PRACTICING! Consistency is key to improvement."
    };

    log::info!("Performance: {}", rating);
    log::info!("");
    log::info!("Reset Teensy to try again!");
    poller.poll();

    loop {
        poller.poll();
        cortex_m::asm::wfi(); // Wait for interrupt
    }
}
