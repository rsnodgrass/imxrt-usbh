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
//! **Next Step:** Try the working examples (hid_keyboard.rs, etc.)

#![no_main]
#![no_std]

use panic_halt as _;
use cortex_m_rt::entry;

// Core library imports - just use what actually exists
use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::ehci::{EhciController, Uninitialized};


#[entry]
fn main() -> ! {
    #[cfg(feature = "defmt")]
    {
        defmt::println!("USB Host Learning Example 02: EHCI Controller Setup");
        defmt::println!("====================================================");
        defmt::println!("");
        defmt::println!("This example builds on Example 01 by adding EHCI controller.");
        defmt::println!("");
    }
    
    // STEP 1: Initialize USB PHY (same as example 01)
    #[cfg(feature = "defmt")]
    defmt::println!("Step 1: USB PHY initialization...");
    
    let mut phy = unsafe { UsbPhy::new(0x400D_9000, 0x400F_C000) };
    
    match phy.init_host_mode() {
        Ok(()) => {
            #[cfg(feature = "defmt")]
            defmt::println!("✓ USB PHY initialized and ready");
        },
        Err(e) => {
            #[cfg(feature = "defmt")]
            defmt::println!("✗ USB PHY failed: {}", e.description_with_help());
            panic!("PHY required for USB host");
        }
    }
    
    // STEP 2: Create EHCI Controller
    #[cfg(feature = "defmt")]
    {
        defmt::println!("");
        defmt::println!("Step 2: EHCI Controller creation...");
        defmt::println!("EHCI = Enhanced Host Controller Interface");
        defmt::println!("This manages all USB transfers and device communication.");
    }
    
    let _controller = unsafe { 
        match EhciController::<8, Uninitialized>::new(0x402E_0140) {
            Ok(controller) => {
                #[cfg(feature = "defmt")]
                {
                    defmt::println!("✓ EHCI controller created successfully!");
                    defmt::println!("  • Controller supports up to 8 ports");
                    defmt::println!("  • Base address: 0x402E_0140 (USB1 controller)");
                    defmt::println!("  • Currently in Uninitialized state");
                }
                controller
            },
            Err(e) => {
                #[cfg(feature = "defmt")]
                {
                    defmt::println!("✗ EHCI controller creation failed!");
                    defmt::println!("Error: {}", e.description_with_help());
                }
                panic!("EHCI controller required");
            }
        }
    };
    
    // STEP 3: Explain what we've accomplished
    #[cfg(feature = "defmt")]
    {
        defmt::println!("");
        defmt::println!("Success! Basic USB host components are ready.");
        defmt::println!("");
        defmt::println!("What we have now:");
        defmt::println!("  • USB PHY - Physical layer for USB signaling");
        defmt::println!("  • EHCI Controller - Manages USB protocol and transfers");
        defmt::println!("");
        defmt::println!("What's missing for full USB host:");
        defmt::println!("  • Controller initialization and configuration");
        defmt::println!("  • DMA memory management setup");
        defmt::println!("  • Device enumeration state machine");
        defmt::println!("  • Transfer queue management");
        defmt::println!("  • Interrupt handling");
        defmt::println!("");
        defmt::println!("These complex topics are shown in working examples like:");
        defmt::println!("  • hid_keyboard.rs - Complete HID keyboard support");
        defmt::println!("  • mass_storage.rs - USB flash drive communication");
        defmt::println!("  • midi_keyboard.rs - MIDI device handling");
    }
    
    // Keep system running
    let mut counter = 0u32;
    loop {
        cortex_m::asm::delay(600_000_000); // ~1 second
        counter = counter.saturating_add(1);
        
        #[cfg(feature = "defmt")]
        if counter % 60 == 0 { // Every minute
            defmt::println!("USB host components still ready... ({}m)", counter / 60);
        }
        
        // In real applications, the initialized components would be used to:
        // - Configure EHCI registers and enable controller
        // - Set up periodic and asynchronous schedules  
        // - Monitor port status for device connections
        // - Handle USB interrupts and transfer completion
        // - Enumerate connected devices
        // - Manage ongoing USB transfers
    }
}