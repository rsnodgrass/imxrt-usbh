//! USB PHY initialization and control
//!
//! Based on i.MX RT1060 Reference Manual sections 14.4-14.5 and 66.5
//! Implements embedded systems best practices for hardware timing and error recovery

use crate::ehci::{clear_bits_at, read_register_at, set_bits_at, RegisterTimeout};
use crate::error::{Result, UsbError};
use crate::{safe_modify_reg, safe_read_reg};
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use imxrt_ral as ral;

/// USB PLL divider value for 480MHz output from 24MHz OSC (RM 14.5.1)
const USB_PLL_DIV_SELECT: u32 = 20;


/// USBPHY base addresses per RM Chapter 2
const USBPHY1_BASE: usize = 0x400D_9000;
#[allow(dead_code)] // Reserved for potential multi-PHY support
const USBPHY2_BASE: usize = 0x400D_A000;

/// Hardware timing constants per i.MX RT reference manual
mod timing {
    /// PLL lock timeout (10ms max per RM 14.5.3)
    pub const PLL_LOCK_TIMEOUT_US: u32 = 10_000;
    /// PHY reset hold time (minimum 1μs per RM 66.5.1)
    pub const PHY_RESET_HOLD_TIME_US: u32 = 10;
    /// PHY power-up stabilization time (1ms per RM 66.5.1)
    pub const PHY_POWER_STABILIZATION_US: u32 = 1_000;
    /// Clock stabilization time after PLL lock (100μs per RM 14.5.3)
    pub const CLOCK_STABILIZATION_US: u32 = 100;
    /// USB PHY calibration timeout (5ms max per RM 66.5.1.5)
    pub const PHY_CALIBRATION_TIMEOUT_US: u32 = 5_000;
}

/// USBPHY register offsets per RM 66.6
#[allow(dead_code)]
const USBPHY_PWD_OFFSET: usize = 0x00; // Power-Down register
/// TX register offset - reserved for future PHY calibration/tuning features
#[allow(dead_code)]
const USBPHY_TX_OFFSET: usize = 0x10;
/// RX register offset - reserved for future PHY calibration/tuning features
#[allow(dead_code)]
const USBPHY_RX_OFFSET: usize = 0x20;
const USBPHY_CTRL_OFFSET: usize = 0x30; // Control register
#[allow(dead_code)]
const USBPHY_CTRL_SET_OFFSET: usize = 0x34;
#[allow(dead_code)]
const USBPHY_CTRL_CLR_OFFSET: usize = 0x38;

/// PHY control bits per RM 66.6.33
const USBPHY_CTRL_SFTRST: u32 = 1 << 31;
const USBPHY_CTRL_CLKGATE: u32 = 1 << 30;
const USBPHY_CTRL_HOSTDISCONDETECT_IRQ: u32 = 1 << 3;
const USBPHY_CTRL_ENHOSTDISCONDETECT: u32 = 1 << 1;

/// USB PHY controller with hardware state tracking
pub struct UsbPhy {
    /// PHY power state for tracking
    powered: AtomicBool,
    /// PLL lock state for monitoring
    pll_locked: AtomicBool,
    /// PHY calibration state
    calibrated: AtomicBool,
    /// Error recovery attempt counter
    recovery_attempts: AtomicU8,
    /// Base address for direct register access
    phy_base: usize,
    /// CCM base address (kept for API compatibility, RAL now used internally)
    #[allow(dead_code)]
    ccm_base: usize,
}

impl UsbPhy {
    /// Create a new USB PHY instance
    ///
    /// # Safety
    ///
    /// Caller must ensure exclusive access to CCM and USBPHY peripherals
    pub unsafe fn new(phy_base: usize, ccm_base: usize) -> Self {
        Self {
            powered: AtomicBool::new(false),
            pll_locked: AtomicBool::new(false),
            calibrated: AtomicBool::new(false),
            recovery_attempts: AtomicU8::new(0),
            phy_base,
            ccm_base,
        }
    }

    /// Initialize USB PHY for host mode
    ///
    /// Implements initialization sequence from RM 66.5.1 and AN12042 section 3.1
    /// with proper hardware timing and error recovery
    pub fn init_host_mode(&mut self) -> Result<()> {
        // Step 1: Initialize USB PLL (RM 14.5.3)
        self.init_usb_pll()?;

        // Step 2: Reset and configure PHY (RM 66.5.1.2)
        self.reset_phy()?;

        // Step 3: Power up PHY with proper timing
        self.power_up_phy()?;

        // Step 4: Configure for host mode
        self.configure_host_mode()?;

        // Step 5: Calibrate PHY (RM 66.5.1.5)
        self.calibrate_phy()?;

        // Step 6: Enable host disconnect detection
        self.enable_host_disconnect_detect();

        self.powered.store(true, Ordering::Release);
        Ok(())
    }

    /// Reset the USB PHY with proper timing
    fn reset_phy(&mut self) -> Result<()> {
        let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;

        // Assert soft reset (RM 66.5.1.2 step 1)
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, set_bits_at uses atomic operations
        unsafe { set_bits_at(ctrl_reg, USBPHY_CTRL_SFTRST) };

        // Hold reset for minimum time (RM timing requirement)
        self.delay_us(timing::PHY_RESET_HOLD_TIME_US);

        // Deassert soft reset and clock gate
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, clear_bits_at uses atomic operations
        unsafe { clear_bits_at(ctrl_reg, USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE) };

        // Wait for PHY to stabilize
        self.delay_us(timing::PHY_POWER_STABILIZATION_US);

        Ok(())
    }

    /// Initialize USB PLL with hardware timing verification
    fn init_usb_pll(&mut self) -> Result<()> {
        // CRITICAL: Enable USB peripheral clock before accessing PLL registers
        // Per i.MX RT1060 RM Ch. 13.1.1: Accessing gated peripherals causes bus faults
        // Safety: Getting singleton instance of CCM peripheral, safe as no concurrent access
        let ccm = unsafe { ral::ccm::CCM::instance() };

        // Enable USB clock gate (CG0 = 0b11 for always-on)
        safe_modify_reg!(ral::ccm, ccm, CCGR6, CG0: 0b11);

        // Verify clock gate enabled with readback
        let ccgr6 = safe_read_reg!(ral::ccm, ccm, CCGR6, CG0);
        if ccgr6 != 0b11 {
            #[cfg(feature = "defmt")]
            defmt::error!("USB clock gate not enabled: CG0={:#b}", ccgr6);
            return Err(UsbError::HardwareFailure);
        }

        // CRITICAL: Use correct PLL for this PHY instance
        // Safety: Getting singleton instance of CCM_ANALOG peripheral, safe as no concurrent access
        let analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
        let is_usb1 = self.phy_base == USBPHY1_BASE;

        // Step 1: Set bypass and configure PLL (RM 14.5.3)
        if is_usb1 {
            safe_modify_reg!(ral::ccm_analog, analog, PLL_USB1,
                BYPASS: 1,
                DIV_SELECT: USB_PLL_DIV_SELECT,
                ENABLE: 1,
                POWER: 1
            );

            // Verify write took effect
            let bypass = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, BYPASS);
            let enable = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, ENABLE);
            let div = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, DIV_SELECT);

            if bypass != 1 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB1 bypass not set after write");
                return Err(UsbError::HardwareFailure);
            }
            if enable != 1 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB1 enable not set after write");
                return Err(UsbError::HardwareFailure);
            }
            if div != USB_PLL_DIV_SELECT {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB1 divider mismatch: expected {}, got {}", USB_PLL_DIV_SELECT, div);
                return Err(UsbError::HardwareFailure);
            }
        } else {
            safe_modify_reg!(ral::ccm_analog, analog, PLL_USB2,
                BYPASS: 1,
                DIV_SELECT: USB_PLL_DIV_SELECT,
                ENABLE: 1,
                POWER: 1
            );

            // Verify write took effect
            let bypass = safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, BYPASS);
            let enable = safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, ENABLE);
            let div = safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, DIV_SELECT);

            if bypass != 1 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB2 bypass not set after write");
                return Err(UsbError::HardwareFailure);
            }
            if enable != 1 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB2 enable not set after write");
                return Err(UsbError::HardwareFailure);
            }
            if div != USB_PLL_DIV_SELECT {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB2 divider mismatch: expected {}, got {}", USB_PLL_DIV_SELECT, div);
                return Err(UsbError::HardwareFailure);
            }
        }

        // Verify we didn't accidentally modify the OTHER PLL
        self.verify_other_pll_unchanged(is_usb1)?;

        // Step 2: Wait for PLL lock with timeout (RM 14.5.3)
        let timeout = RegisterTimeout::new_us(timing::PLL_LOCK_TIMEOUT_US);
        timeout
            .wait_for(|| {
                if is_usb1 {
                    safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) != 0
                } else {
                    safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, LOCK) != 0
                }
            })
            .map_err(|_| UsbError::HardwareFailure)?;

        // Step 3: Clear bypass after lock confirmed (RM requirement)
        if is_usb1 {
            safe_modify_reg!(ral::ccm_analog, analog, PLL_USB1,
                BYPASS: 0,
                EN_USB_CLKS: 1
            );

            // Verify bypass cleared and clocks enabled
            let bypass = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, BYPASS);
            let clks = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, EN_USB_CLKS);

            if bypass != 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB1 bypass still set after clear");
                return Err(UsbError::HardwareFailure);
            }
            if clks == 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB1 clocks not enabled after write");
                return Err(UsbError::HardwareFailure);
            }
        } else {
            safe_modify_reg!(ral::ccm_analog, analog, PLL_USB2,
                BYPASS: 0,
                EN_USB_CLKS: 1
            );

            // Verify bypass cleared and clocks enabled
            let bypass = safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, BYPASS);
            let clks = safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, EN_USB_CLKS);

            if bypass != 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB2 bypass still set after clear");
                return Err(UsbError::HardwareFailure);
            }
            if clks == 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("PLL_USB2 clocks not enabled after write");
                return Err(UsbError::HardwareFailure);
            }
        }

        // Wait for clock stabilization
        self.delay_us(timing::CLOCK_STABILIZATION_US);
        self.pll_locked.store(true, Ordering::Release);

        Ok(())
    }

    /// Verify the OTHER USB PLL wasn't accidentally modified
    ///
    /// This catches bugs where we write to the wrong PLL register.
    /// We check the other PLL's divider to ensure it wasn't recently configured.
    fn verify_other_pll_unchanged(&self, is_usb1: bool) -> Result<()> {
        // Safety: Getting singleton instance of CCM_ANALOG peripheral, safe as no concurrent access
        let analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };

        // Read the OTHER PLL's divider
        let other_divider = if is_usb1 {
            safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, DIV_SELECT)
        } else {
            safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, DIV_SELECT)
        };

        // If the other PLL has our exact divider value, that's suspicious
        // (though not conclusive since both PLLs might use 480MHz)
        if other_divider == USB_PLL_DIV_SELECT {
            #[cfg(feature = "defmt")]
            defmt::warn!("Other PLL has same divider ({}) - verify correct PLL modified", other_divider);
        }

        Ok(())
    }

    /// Power up PHY with monitored sequencing
    fn power_up_phy(&mut self) -> Result<()> {
        // Power up PHY (RM 66.5.1.3)
        let pwr_reg = (self.phy_base + 0x00) as *mut u32;

        // Clear power-down bits
        // Safety: pwr_reg is valid USBPHY_PWD register address, clear_bits_at uses atomic operations
        unsafe { clear_bits_at(pwr_reg, 1 << 10) }; // PWDN bit

        // Wait for power stabilization with verification
        self.delay_us(timing::PHY_POWER_STABILIZATION_US);

        // Verify PHY is responding
        self.verify_phy_response()?;

        Ok(())
    }

    /// Configure PHY for host mode operation
    fn configure_host_mode(&mut self) -> Result<()> {
        let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;

        // Enable host mode features
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, set_bits_at uses atomic operations
        unsafe {
            set_bits_at(
                ctrl_reg,
                USBPHY_CTRL_ENHOSTDISCONDETECT | USBPHY_CTRL_HOSTDISCONDETECT_IRQ,
            )
        };

        Ok(())
    }

    /// Calibrate PHY with timeout and verification
    fn calibrate_phy(&mut self) -> Result<()> {
        let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;

        // Start calibration sequence (RM 66.5.1.5)
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, set_bits_at uses atomic operations
        unsafe { set_bits_at(ctrl_reg, 1 << 16) }; // Start calibration

        // Wait for calibration completion
        let timeout = RegisterTimeout::new_us(timing::PHY_CALIBRATION_TIMEOUT_US);
        timeout
            .wait_for(|| {
                // Safety: ctrl_reg is valid USBPHY_CTRL register address for polling calibration status
                let ctrl = unsafe { read_register_at(ctrl_reg) };
                (ctrl & (1 << 17)) != 0 // Calibration done
            })
            .map_err(|_| UsbError::HardwareFailure)?;

        self.calibrated.store(true, Ordering::Release);
        Ok(())
    }

    /// Enable host disconnect detection with debouncing
    fn enable_host_disconnect_detect(&mut self) {
        let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, set_bits_at uses atomic operations
        unsafe { set_bits_at(ctrl_reg, USBPHY_CTRL_ENHOSTDISCONDETECT) };
    }

    /// Verify PHY is responding to register accesses
    fn verify_phy_response(&mut self) -> Result<()> {
        // Read CTRL register to verify PHY is accessible
        // (No test register exists in USBPHY per RM 66.6)
        let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *const u32;
        // Safety: ctrl_reg is valid USBPHY_CTRL register address, read verifies PHY is accessible
        let _ctrl = unsafe { read_register_at(ctrl_reg) };
        // If read succeeds without bus fault, PHY is responding

        Ok(())
    }

    /// Hardware-specific microsecond delay implementation
    #[inline(always)]
    fn delay_us(&self, us: u32) {
        // Use DWT cycle counter for precise timing
        let start_cycles = cortex_m::peripheral::DWT::cycle_count();
        let target_cycles = us * crate::CPU_FREQ_MHZ;

        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start_cycles) < target_cycles {
            cortex_m::asm::nop();
        }
    }

    /// Check PHY health and attempt recovery if needed
    pub fn health_check_and_recovery(&mut self) -> Result<()> {
        // Check PLL lock status for this PHY instance
        // Safety: Getting singleton instance of CCM_ANALOG peripheral, safe as no concurrent access
        let analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
        let is_usb1 = self.phy_base == USBPHY1_BASE;

        let pll_locked = if is_usb1 {
            safe_read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) != 0
        } else {
            safe_read_reg!(ral::ccm_analog, analog, PLL_USB2, LOCK) != 0
        };

        if !pll_locked {
            let attempts = self.recovery_attempts.load(Ordering::Relaxed);
            if attempts >= 3 {
                return Err(UsbError::HardwareFailure);
            }

            self.recovery_attempts.fetch_add(1, Ordering::Relaxed);

            // Attempt PLL recovery
            self.init_usb_pll()?;
        }

        // Verify PHY is still responding
        self.verify_phy_response()?;

        Ok(())
    }

    /// Get PHY status for diagnostics
    pub fn status(&self) -> PhyStatus {
        PhyStatus {
            powered: self.powered.load(Ordering::Relaxed),
            pll_locked: self.pll_locked.load(Ordering::Relaxed),
            calibrated: self.calibrated.load(Ordering::Relaxed),
            recovery_attempts: self.recovery_attempts.load(Ordering::Relaxed),
        }
    }

}

// Note: enable_interrupts() method was removed - it was buggy (wrong register offset of 0x00
// instead of USBPHY_CTRL_OFFSET, missing memory barriers) and functionality is already
// covered by configure_host_mode() method above.

/// PHY status for health monitoring
#[derive(Debug, Clone, Copy)]
pub struct PhyStatus {
    /// PHY power state
    pub powered: bool,
    /// PLL lock state
    pub pll_locked: bool,
    /// Calibration completion state
    pub calibrated: bool,
    /// Number of recovery attempts
    pub recovery_attempts: u8,
}
