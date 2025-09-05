//! VBUS power control for USB host
//! 
//! Provides hardware abstraction for VBUS power switching and over-current detection
//! as required for USB host operation. Teensy 4.1 requires external VBUS control.

use crate::error::{Result, UsbError};
use embedded_hal::digital::{InputPin, OutputPin};

/// VBUS power control trait for USB host
/// 
/// Implementations handle platform-specific VBUS power switching and monitoring.
pub trait VbusPowerControl {
    /// Enable VBUS power (5V @ 500mA minimum)
    /// 
    /// Returns after VBUS has stabilized or error if over-current detected.
    fn enable_vbus(&mut self) -> Result<()>;
    
    /// Disable VBUS power
    fn disable_vbus(&mut self);
    
    /// Check if over-current condition is detected
    fn is_overcurrent(&self) -> bool;
    
    /// Get current VBUS state
    fn is_enabled(&self) -> bool;
    
    /// Reset after over-current condition
    fn reset_overcurrent(&mut self);
}

/// Teensy 4.1 VBUS control implementation
/// 
/// Uses GPIO pins for VBUS enable and over-current detection.
/// Requires external VBUS switch circuit (e.g., TPS2051B or similar).
pub struct Teensy41VbusControl<EN, OC> 
where
    EN: OutputPin,
    OC: InputPin,
{
    enable_pin: EN,
    oc_pin: OC,
    enabled: bool,
    oc_count: u32,
}

impl<EN, OC> Teensy41VbusControl<EN, OC>
where
    EN: OutputPin,
    OC: InputPin,
{
    /// Create new VBUS control instance
    /// 
    /// # Arguments
    /// * `enable_pin` - GPIO output to VBUS switch enable (active high)
    /// * `oc_pin` - GPIO input from over-current detect (active low)
    pub fn new(mut enable_pin: EN, oc_pin: OC) -> Result<Self> {
        // Start with VBUS disabled
        let _ = enable_pin.set_low();
        
        Ok(Self {
            enable_pin,
            oc_pin,
            enabled: false,
            oc_count: 0,
        })
    }
    
    /// Check and debounce over-current signal
    fn check_overcurrent_debounced(&mut self) -> bool {
        // Over-current signal is active low
        let oc_detected = match self.oc_pin.is_low() {
            Ok(is_low) => is_low,
            Err(_) => true, // Assume over-current on error for safety
        };
        
        if oc_detected {
            self.oc_count += 1;
            // Require 3 consecutive OC readings for debouncing
            if self.oc_count >= 3 {
                return true;
            }
        } else {
            self.oc_count = 0;
        }
        
        false
    }
}

impl<EN, OC> VbusPowerControl for Teensy41VbusControl<EN, OC>
where
    EN: OutputPin,
    OC: InputPin,
{
    fn enable_vbus(&mut self) -> Result<()> {
        if self.enabled {
            return Ok(());
        }
        
        // Check for existing over-current condition
        if self.check_overcurrent_debounced() {
            return Err(UsbError::VbusOverCurrent);
        }
        
        // Enable VBUS
        let _ = self.enable_pin.set_high();
        
        // Wait 100ms for VBUS rise time (USB 2.0 spec 7.1.2.1)
        // In RTIC context, this would be replaced with a delayed task
        cortex_m::asm::delay(600_000_000 / 10); // Assuming 600MHz clock
        
        // Check for over-current after power-on
        if self.check_overcurrent_debounced() {
            let _ = self.enable_pin.set_low();
            self.enabled = false;
            return Err(UsbError::VbusOverCurrent);
        }
        
        self.enabled = true;
        Ok(())
    }
    
    fn disable_vbus(&mut self) {
        let _ = self.enable_pin.set_low();
        self.enabled = false;
        self.oc_count = 0;
    }
    
    fn is_overcurrent(&self) -> bool {
        // Simple non-debounced over-current check
        // Over-current signal is active low, but we can't access pin from &self
        // Return false for now - debounced check happens in enable_vbus
        false
    }
    
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    
    fn reset_overcurrent(&mut self) {
        self.oc_count = 0;
        self.disable_vbus();
    }
}

/// Dummy VBUS control for boards without VBUS switching
/// 
/// Use this for boards where VBUS is always on or controlled externally.
pub struct NoVbusControl;

impl VbusPowerControl for NoVbusControl {
    fn enable_vbus(&mut self) -> Result<()> {
        Ok(())
    }
    
    fn disable_vbus(&mut self) {
        // No-op
    }
    
    fn is_overcurrent(&self) -> bool {
        false
    }
    
    fn is_enabled(&self) -> bool {
        true
    }
    
    fn reset_overcurrent(&mut self) {
        // No-op
    }
}

/// VBUS control with RTIC integration
#[cfg(feature = "rtic-support")]
pub mod rtic {
    use super::*;
    use rtic_sync::channel::{Sender, Receiver};
    
    /// VBUS events for RTIC message passing
    #[derive(Debug, Clone, Copy)]
    pub enum VbusEvent {
        Enable,
        Disable,
        OverCurrentDetected,
        PowerGood,
    }
    
    /// RTIC-aware VBUS controller
    pub struct RticVbusControl<V: VbusPowerControl> {
        inner: V,
        event_sender: Sender<'static, VbusEvent, 8>,
    }
    
    impl<V: VbusPowerControl> RticVbusControl<V> {
        pub fn new(inner: V, event_sender: Sender<'static, VbusEvent, 8>) -> Self {
            Self {
                inner,
                event_sender,
            }
        }
        
        pub fn enable(&mut self) -> Result<()> {
            match self.inner.enable_vbus() {
                Ok(()) => {
                    let _ = self.event_sender.try_send(VbusEvent::PowerGood);
                    Ok(())
                }
                Err(UsbError::VbusOverCurrent) => {
                    let _ = self.event_sender.try_send(VbusEvent::OverCurrentDetected);
                    Err(UsbError::VbusOverCurrent)
                }
                Err(e) => Err(e),
            }
        }
        
        pub fn disable(&mut self) {
            self.inner.disable_vbus();
            let _ = self.event_sender.try_send(VbusEvent::Disable);
        }
        
        pub fn check_status(&mut self) {
            if self.inner.is_overcurrent() && self.inner.is_enabled() {
                self.inner.disable_vbus();
                let _ = self.event_sender.try_send(VbusEvent::OverCurrentDetected);
            }
        }
    }
}