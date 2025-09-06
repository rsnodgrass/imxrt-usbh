//! VBUS power control for USB host with embedded systems enhancements
//! 
//! Provides hardware abstraction for VBUS power switching, over-current detection,
//! and power management as required for robust USB host operation.
//! Enhanced with fault recovery, power profiling, and real-time monitoring.

use crate::error::{Result, UsbError};
use embedded_hal::digital::{InputPin, OutputPin};
use core::sync::atomic::{AtomicU8, AtomicU16, AtomicBool, Ordering};

/// Hardware timing constants for VBUS operations
pub mod timing {
    /// VBUS rise time per USB 2.0 spec 7.1.2.1 (100ms max)
    pub const VBUS_RISE_TIME_US: u32 = 100_000;
    /// VBUS fall time requirement (1ms typical)
    pub const VBUS_FALL_TIME_US: u32 = 1_000;
    /// Over-current detection debounce time (10ms)
    pub const OVERCURRENT_DEBOUNCE_US: u32 = 10_000;
    /// Power good validation delay (10ms after rise)
    pub const POWER_GOOD_DELAY_US: u32 = 10_000;
    /// Minimum time between overcurrent recovery attempts (1 second)
    pub const RECOVERY_INTERVAL_US: u32 = 1_000_000;
}

/// VBUS power states for state machine management
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VbusState {
    /// Power off, ready to enable
    PowerOff,
    /// Powering up, waiting for stabilization
    PoweringUp,
    /// Power good, normal operation
    PowerGood,
    /// Over-current fault detected
    OverCurrentFault,
    /// Fault recovery in progress
    FaultRecovery,
    /// Permanent fault, manual intervention required
    PermanentFault,
}

/// VBUS power control trait for USB host with enhanced monitoring
pub trait VbusPowerControl {
    /// Enable VBUS power (5V @ 500mA minimum)
    fn enable_vbus(&mut self) -> Result<()>;
    
    /// Disable VBUS power immediately
    fn disable_vbus(&mut self);
    
    /// Check if over-current condition is detected
    fn is_overcurrent(&mut self) -> bool;
    
    /// Get current VBUS state
    fn vbus_state(&self) -> VbusState;
    
    /// Reset after over-current condition
    fn reset_overcurrent(&mut self) -> Result<()>;
    
    /// Get fault statistics
    fn fault_statistics(&self) -> VbusFaultStats;
    
    /// Perform health check and maintenance
    fn health_check(&mut self) -> Result<()>;
}

/// VBUS fault statistics for monitoring and diagnostics
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VbusFaultStats {
    /// Total overcurrent events
    pub overcurrent_events: u16,
    /// Successful recovery attempts
    pub recovery_successes: u16,
    /// Failed recovery attempts
    pub recovery_failures: u16,
    /// Current fault state
    pub current_state: VbusState,
    /// Time since last fault (in system ticks)
    pub time_since_fault: u32,
}

/// Enhanced VBUS control implementation with fault recovery
pub struct EnhancedVbusControl<EN, OC> 
where
    EN: OutputPin,
    OC: InputPin,
{
    /// GPIO pins
    enable_pin: EN,
    oc_pin: OC,
    
    /// State management
    state: VbusState,
    state_entry_time: u32,
    
    /// Fault tracking (atomic for interrupt safety)
    overcurrent_events: AtomicU16,
    recovery_successes: AtomicU16,
    recovery_failures: AtomicU16,
    debounce_counter: AtomicU8,
    last_fault_time: AtomicU16,
    
    /// Safety limits
    max_recovery_attempts: u8,
    enabled: AtomicBool,
}

impl<EN, OC> EnhancedVbusControl<EN, OC>
where
    EN: OutputPin,
    OC: InputPin,
{
    /// Create new enhanced VBUS control instance
    pub fn new(mut enable_pin: EN, oc_pin: OC, max_recovery_attempts: u8) -> Result<Self> {
        // Start with VBUS disabled for safety
        let _ = enable_pin.set_low();
        
        Ok(Self {
            enable_pin,
            oc_pin,
            state: VbusState::PowerOff,
            state_entry_time: Self::get_system_time_us(),
            overcurrent_events: AtomicU16::new(0),
            recovery_successes: AtomicU16::new(0),
            recovery_failures: AtomicU16::new(0),
            debounce_counter: AtomicU8::new(0),
            last_fault_time: AtomicU16::new(0),
            max_recovery_attempts,
            enabled: AtomicBool::new(false),
        })
    }
    
    /// Check over-current with proper debouncing
    fn check_overcurrent_debounced(&mut self) -> bool {
        // Over-current signal is typically active low
        let oc_detected = match self.oc_pin.is_low() {
            Ok(is_low) => is_low,
            Err(_) => {
                // Assume over-current on pin error for safety
                true
            }
        };
        
        let current_count = self.debounce_counter.load(Ordering::Relaxed);
        
        if oc_detected {
            let new_count = current_count.saturating_add(1);
            self.debounce_counter.store(new_count, Ordering::Relaxed);
            
            // Require multiple consecutive readings for confirmation
            if new_count >= 5 {
                self.debounce_counter.store(0, Ordering::Relaxed);
                return true;
            }
        } else {
            // Clear debounce counter on good reading
            self.debounce_counter.store(0, Ordering::Relaxed);
        }
        
        false
    }
    
    /// State machine update - call regularly from main loop or timer
    pub fn update_state_machine(&mut self) -> Result<()> {
        let current_time = Self::get_system_time_us();
        let state_duration = current_time.wrapping_sub(self.state_entry_time);
        
        match self.state {
            VbusState::PowerOff => {
                // Ready to power up when requested
            }
            
            VbusState::PoweringUp => {
                // Check for overcurrent during power-up
                if self.check_overcurrent_debounced() {
                    self.handle_overcurrent_fault()?;
                    return Ok(());
                }
                
                // Wait for VBUS to stabilize
                if state_duration >= timing::VBUS_RISE_TIME_US {
                    // Validate power good
                    self.delay_us(timing::POWER_GOOD_DELAY_US);
                    
                    if !self.check_overcurrent_debounced() {
                        self.transition_to_state(VbusState::PowerGood);
                        self.enabled.store(true, Ordering::Release);
                    } else {
                        self.handle_overcurrent_fault()?;
                    }
                }
            }
            
            VbusState::PowerGood => {
                // Monitor for overcurrent during normal operation
                if self.check_overcurrent_debounced() {
                    self.handle_overcurrent_fault()?;
                }
            }
            
            VbusState::OverCurrentFault => {
                // Ensure power is off during fault
                let _ = self.enable_pin.set_low();
                self.enabled.store(false, Ordering::Release);
                
                // Wait minimum time before attempting recovery
                if state_duration >= timing::RECOVERY_INTERVAL_US {
                    self.attempt_fault_recovery()?;
                }
            }
            
            VbusState::FaultRecovery => {
                // Check if recovery was successful
                if state_duration >= timing::POWER_GOOD_DELAY_US {
                    if !self.check_overcurrent_debounced() {
                        // Recovery successful
                        self.transition_to_state(VbusState::PowerGood);
                        self.recovery_successes.fetch_add(1, Ordering::Relaxed);
                        self.enabled.store(true, Ordering::Release);
                    } else {
                        // Recovery failed
                        self.recovery_failures.fetch_add(1, Ordering::Relaxed);
                        self.handle_recovery_failure();
                    }
                }
            }
            
            VbusState::PermanentFault => {
                // Permanent fault - power remains off until manual reset
                let _ = self.enable_pin.set_low();
                self.enabled.store(false, Ordering::Release);
            }
        }
        
        Ok(())
    }
    
    /// Handle overcurrent fault detection
    fn handle_overcurrent_fault(&mut self) -> Result<()> {
        // Immediately disable power
        let _ = self.enable_pin.set_low();
        self.enabled.store(false, Ordering::Release);
        
        // Record fault
        self.overcurrent_events.fetch_add(1, Ordering::Relaxed);
        self.last_fault_time.store((Self::get_system_time_us() / 1000) as u16, Ordering::Relaxed);
        
        // Transition to fault state
        self.transition_to_state(VbusState::OverCurrentFault);
        
        Ok(())
    }
    
    /// Attempt automated fault recovery
    fn attempt_fault_recovery(&mut self) -> Result<()> {
        let failure_count = self.recovery_failures.load(Ordering::Relaxed);
        
        if failure_count >= self.max_recovery_attempts as u16 {
            // Exceeded maximum recovery attempts
            self.transition_to_state(VbusState::PermanentFault);
            return Err(UsbError::PortError);
        }
        
        // Attempt to re-enable power
        let _ = self.enable_pin.set_high();
        self.transition_to_state(VbusState::FaultRecovery);
        
        Ok(())
    }
    
    /// Handle recovery failure
    fn handle_recovery_failure(&mut self) {
        let _ = self.enable_pin.set_low();
        self.enabled.store(false, Ordering::Release);
        
        let failure_count = self.recovery_failures.load(Ordering::Relaxed);
        
        if failure_count >= self.max_recovery_attempts as u16 {
            self.transition_to_state(VbusState::PermanentFault);
        } else {
            // Return to fault state for another attempt
            self.transition_to_state(VbusState::OverCurrentFault);
        }
    }
    
    /// Transition to new state with timing
    fn transition_to_state(&mut self, new_state: VbusState) {
        self.state = new_state;
        self.state_entry_time = Self::get_system_time_us();
    }
    
    /// Hardware-specific microsecond delay
    #[inline(always)]
    fn delay_us(&self, us: u32) {
        // Use DWT cycle counter for precise timing (assuming 600MHz CPU)
        let start_cycles = cortex_m::peripheral::DWT::cycle_count();
        let target_cycles = us * 600; // 600 cycles per μs at 600MHz
        
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start_cycles) < target_cycles {
            cortex_m::asm::nop();
        }
    }
    
    /// Get system time in microseconds (placeholder implementation)
    fn get_system_time_us() -> u32 {
        // In real implementation, this would use a system timer
        // For now, use DWT cycle counter
        cortex_m::peripheral::DWT::cycle_count() / 600
    }
    
    /// Force manual recovery from permanent fault
    pub fn manual_recovery(&mut self) -> Result<()> {
        match self.state {
            VbusState::PermanentFault => {
                // Reset fault counters
                self.recovery_failures.store(0, Ordering::Relaxed);
                self.debounce_counter.store(0, Ordering::Relaxed);
                
                // Return to power-off state
                self.transition_to_state(VbusState::PowerOff);
                Ok(())
            }
            _ => Err(UsbError::InvalidState),
        }
    }
}

impl<EN, OC> VbusPowerControl for EnhancedVbusControl<EN, OC>
where
    EN: OutputPin,
    OC: InputPin,
{
    fn enable_vbus(&mut self) -> Result<()> {
        match self.state {
            VbusState::PowerOff => {
                // Check for existing fault condition
                if self.check_overcurrent_debounced() {
                    return Err(UsbError::PortError);
                }
                
                // Begin power-up sequence
                let _ = self.enable_pin.set_high();
                self.transition_to_state(VbusState::PoweringUp);
                Ok(())
            }
            VbusState::PowerGood => {
                // Already enabled
                Ok(())
            }
            VbusState::PoweringUp | VbusState::FaultRecovery => {
                // Power-up in progress
                Ok(())
            }
            VbusState::OverCurrentFault => {
                Err(UsbError::PortError)
            }
            VbusState::PermanentFault => {
                Err(UsbError::PortError)
            }
        }
    }
    
    fn disable_vbus(&mut self) {
        let _ = self.enable_pin.set_low();
        self.enabled.store(false, Ordering::Release);
        self.transition_to_state(VbusState::PowerOff);
        
        // Clear debounce counter
        self.debounce_counter.store(0, Ordering::Relaxed);
    }
    
    fn is_overcurrent(&mut self) -> bool {
        // Non-blocking overcurrent check
        match self.oc_pin.is_low() {
            Ok(is_low) => is_low,
            Err(_) => true, // Assume overcurrent on error for safety
        }
    }
    
    fn vbus_state(&self) -> VbusState {
        self.state
    }
    
    fn reset_overcurrent(&mut self) -> Result<()> {
        match self.state {
            VbusState::OverCurrentFault => {
                // Reset debounce and attempt recovery
                self.debounce_counter.store(0, Ordering::Relaxed);
                self.attempt_fault_recovery()
            }
            VbusState::PermanentFault => {
                self.manual_recovery()
            }
            _ => Ok(()),
        }
    }
    
    fn fault_statistics(&self) -> VbusFaultStats {
        VbusFaultStats {
            overcurrent_events: self.overcurrent_events.load(Ordering::Relaxed),
            recovery_successes: self.recovery_successes.load(Ordering::Relaxed),
            recovery_failures: self.recovery_failures.load(Ordering::Relaxed),
            current_state: self.state,
            time_since_fault: Self::get_system_time_us()
                .wrapping_sub(self.last_fault_time.load(Ordering::Relaxed) as u32 * 1000),
        }
    }
    
    fn health_check(&mut self) -> Result<()> {
        // Update state machine
        self.update_state_machine()?;
        
        // Check if we're in a healthy state
        match self.state {
            VbusState::PowerOff | VbusState::PowerGood => Ok(()),
            VbusState::PoweringUp | VbusState::FaultRecovery => {
                // Transitional states are acceptable
                Ok(())
            }
            VbusState::OverCurrentFault => Err(UsbError::PortError),
            VbusState::PermanentFault => Err(UsbError::PortError),
        }
    }
}

/// Dummy VBUS control for boards without VBUS switching
pub struct NoVbusControl {
    state: VbusState,
}

impl Default for NoVbusControl {
    fn default() -> Self {
        Self {
            state: VbusState::PowerGood, // Always assume power good
        }
    }
}

impl VbusPowerControl for NoVbusControl {
    fn enable_vbus(&mut self) -> Result<()> {
        self.state = VbusState::PowerGood;
        Ok(())
    }
    
    fn disable_vbus(&mut self) {
        self.state = VbusState::PowerOff;
    }
    
    fn is_overcurrent(&mut self) -> bool {
        false
    }
    
    fn vbus_state(&self) -> VbusState {
        self.state
    }
    
    fn reset_overcurrent(&mut self) -> Result<()> {
        Ok(())
    }
    
    fn fault_statistics(&self) -> VbusFaultStats {
        VbusFaultStats {
            overcurrent_events: 0,
            recovery_successes: 0,
            recovery_failures: 0,
            current_state: self.state,
            time_since_fault: 0,
        }
    }
    
    fn health_check(&mut self) -> Result<()> {
        Ok(())
    }
}

/// VBUS control with RTIC integration for real-time systems
#[cfg(feature = "rtic-support")]
pub mod rtic {
    use super::*;
    
    /// VBUS events for RTIC message passing
    #[derive(Debug, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum VbusEvent {
        /// Power enable requested
        EnableRequested,
        /// Power disable requested
        DisableRequested,
        /// Over-current detected
        OverCurrentDetected,
        /// Power good achieved
        PowerGood,
        /// Fault recovery started
        RecoveryStarted,
        /// Recovery successful
        RecoverySuccessful,
        /// Recovery failed
        RecoveryFailed,
        /// Permanent fault - manual intervention required
        PermanentFault,
    }
    
    /// RTIC-aware VBUS controller with event reporting
    pub struct RticVbusControl<V: VbusPowerControl> {
        inner: V,
        last_state: VbusState,
    }
    
    impl<V: VbusPowerControl> RticVbusControl<V> {
        pub fn new(inner: V) -> Self {
            let last_state = inner.vbus_state();
            Self {
                inner,
                last_state,
            }
        }
        
        /// Enable VBUS with event notification
        pub fn enable(&mut self) -> Result<VbusEvent> {
            match self.inner.enable_vbus() {
                Ok(()) => {
                    Ok(VbusEvent::EnableRequested)
                }
                Err(UsbError::PortError) => {
                    Ok(VbusEvent::OverCurrentDetected)
                }
                Err(e) => Err(e),
            }
        }
        
        /// Disable VBUS with event notification
        pub fn disable(&mut self) -> VbusEvent {
            self.inner.disable_vbus();
            VbusEvent::DisableRequested
        }
        
        /// Update and check for state changes
        pub fn update(&mut self) -> Result<Option<VbusEvent>> {
            // Perform health check (updates state machine)
            self.inner.health_check()?;
            
            let current_state = self.inner.vbus_state();
            
            // Check for state transitions
            let event = if current_state != self.last_state {
                match (self.last_state, current_state) {
                    (_, VbusState::PowerGood) => Some(VbusEvent::PowerGood),
                    (_, VbusState::OverCurrentFault) => Some(VbusEvent::OverCurrentDetected),
                    (VbusState::OverCurrentFault, VbusState::FaultRecovery) => Some(VbusEvent::RecoveryStarted),
                    (VbusState::FaultRecovery, VbusState::PowerGood) => Some(VbusEvent::RecoverySuccessful),
                    (VbusState::FaultRecovery, VbusState::OverCurrentFault) => Some(VbusEvent::RecoveryFailed),
                    (_, VbusState::PermanentFault) => Some(VbusEvent::PermanentFault),
                    _ => None,
                }
            } else {
                None
            };
            
            self.last_state = current_state;
            Ok(event)
        }
        
        /// Get current statistics
        pub fn statistics(&self) -> VbusFaultStats {
            self.inner.fault_statistics()
        }
        
        /// Get current state
        pub fn state(&self) -> VbusState {
            self.inner.vbus_state()
        }
        
        /// Manual recovery from permanent fault
        pub fn manual_recovery(&mut self) -> Result<VbusEvent> {
            self.inner.reset_overcurrent()?;
            Ok(VbusEvent::RecoveryStarted)
        }
    }
}

/// Utility functions for VBUS monitoring
pub mod utils {
    use super::*;
    
    /// Check if VBUS state indicates healthy operation
    pub fn is_healthy_state(state: VbusState) -> bool {
        matches!(state, VbusState::PowerOff | VbusState::PowerGood)
    }
    
    /// Check if recovery is needed
    pub fn needs_recovery(state: VbusState) -> bool {
        matches!(state, VbusState::OverCurrentFault)
    }
    
    /// Check if manual intervention is required
    pub fn needs_manual_intervention(state: VbusState) -> bool {
        matches!(state, VbusState::PermanentFault)
    }
    
    /// Calculate fault rate from statistics
    pub fn calculate_fault_rate(stats: &VbusFaultStats, uptime_seconds: u32) -> f32 {
        if uptime_seconds == 0 {
            return 0.0;
        }
        stats.overcurrent_events as f32 / uptime_seconds as f32
    }
}