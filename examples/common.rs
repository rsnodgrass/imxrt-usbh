//! Common utilities for educational examples
//! 
//! This module contains helper code shared across examples to reduce duplication
//! and provide consistent educational experiences.

use imxrt_usbh::error::{Result, UsbError};
use imxrt_usbh::ehci::{EhciController, Uninitialized};
use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::dma::UsbMemoryPool;

/// Simple wrapper for educational examples
/// 
/// This provides an easier-to-understand interface for learning,
/// without being part of the core library.
pub struct ExampleHost {
    // Keep this simple - just hold what we need for basic examples
    _phy: UsbPhy,
    // We'll add controller and other components as needed
}

impl ExampleHost {
    /// Initialize USB host for educational examples
    pub fn new() -> Result<Self> {
        // Initialize USB PHY
        let phy = UsbPhy::new(0x400D_9000, 0x400F_C000)?;
        
        // For basic examples, we don't need the full controller yet
        Ok(Self { _phy: phy })
    }
    
    /// Check if initialization was successful
    pub fn is_ready(&self) -> bool {
        true // Simple check for educational purposes
    }
}

/// Educational error display helper
pub fn display_error_help(error: &UsbError) {
    println!("Error Details:");
    println!("  Description: {}", error.description_with_help());
    println!("  Category: {:?}", error.category());
    println!();
    println!("Suggested Actions:");
    for (i, action) in error.suggested_actions().iter().enumerate() {
        println!("  {}. {}", i + 1, action);
    }
}

/// Simple delay helper for examples
pub fn delay_ms(ms: u32) {
    // Simple delay implementation for educational purposes
    // In real code, you'd use proper timer peripherals
    let cycles = ms * 600_000; // Approximate for 600MHz Cortex-M7
    cortex_m::asm::delay(cycles);
}

/// Print helper that works with or without defmt
#[macro_export]
macro_rules! example_println {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        defmt::println!($($arg)*);
        
        #[cfg(not(feature = "defmt"))]
        {
            // In real examples, you might use RTT or UART here
            let _ = ($($arg)*,); // Consume to avoid warnings
        }
    };
}