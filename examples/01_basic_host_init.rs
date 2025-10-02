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
//! **Next Step:** Try `02_device_enumeration.rs` to add device detection.

#![no_main]
#![no_std]

use panic_halt as _;
use cortex_m_rt::entry;
use imxrt_usbh::phy::UsbPhy;


#[entry]
fn main() -> ! {
    // Educational: Use defmt if available, otherwise just continue
    #[cfg(feature = "defmt")]
    {
        defmt::println!("USB Host Learning Example 01: Basic Initialization");
        defmt::println!("==================================================");
        defmt::println!("");
        defmt::println!("This example teaches USB host hardware initialization.");
        defmt::println!("We'll initialize the USB PHY step by step.");
        defmt::println!("");
    }
    
    // STEP 1: Initialize USB PHY
    // The USB PHY handles low-level signaling and must be set up first
    #[cfg(feature = "defmt")]
    defmt::println!("Step 1: Creating USB PHY instance...");
    
    let mut phy = unsafe { 
        // These addresses are specific to i.MX RT1062:
        // - 0x400D_9000: USBPHY1 register base
        // - 0x400F_C000: CCM (Clock Control Module) base
        UsbPhy::new(0x400D_9000, 0x400F_C000)
    };
    
    #[cfg(feature = "defmt")]
    defmt::println!("✓ USB PHY instance created");
    
    // STEP 2: Initialize PHY for host mode
    #[cfg(feature = "defmt")]
    defmt::println!("Step 2: Initializing PHY for host mode...");
    
    match phy.init_host_mode() {
        Ok(()) => {
            #[cfg(feature = "defmt")]
            {
                defmt::println!("✓ USB PHY initialized successfully!");
                defmt::println!("");
                defmt::println!("What just happened:");
                defmt::println!("  • USB PLL configured for 480 MHz");
                defmt::println!("  • PHY calibration completed");
                defmt::println!("  • Host mode enabled");
                defmt::println!("  • Ready for EHCI controller setup");
            }
        },
        Err(e) => {
            #[cfg(feature = "defmt")]
            {
                defmt::println!("✗ USB PHY initialization failed!");
                defmt::println!("Error: {}", e.description_with_help());
            }
            // In a real application, you might try recovery here
            panic!("USB PHY init failed");
        }
    }
    
    #[cfg(feature = "defmt")]
    {
        defmt::println!("");
        defmt::println!("Success! USB PHY is ready for host operations.");
        defmt::println!("");
        defmt::println!("Next steps (shown in other examples):");
        defmt::println!("  1. Initialize EHCI controller");
        defmt::println!("  2. Set up DMA memory management");
        defmt::println!("  3. Start device enumeration");
        defmt::println!("");
    }
    
    // Keep the PHY running
    let mut counter = 0u32;
    loop {
        // Simple delay
        cortex_m::asm::delay(600_000_000); // ~1 second at 600MHz
        counter = counter.saturating_add(1);
        
        #[cfg(feature = "defmt")]
        if counter % 30 == 0 { // Every 30 seconds
            defmt::println!("USB PHY still running... ({}s)", counter);
        }
        
        // In a real application, you would:
        // - Initialize the EHCI controller here
        // - Set up interrupt handling
        // - Start monitoring for device connections
    }
}