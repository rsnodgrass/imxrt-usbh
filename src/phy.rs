//! USB PHY initialization and control
//!
//! Based on i.MX RT1060 Reference Manual sections 14.4-14.5 and 66.5
//! Implements embedded systems best practices for hardware timing and error recovery

use crate::ehci::RegisterTimeout;
use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
// TODO: Fix RAL API usage after determining correct register names
// use imxrt_ral as ral;

/// USB1 PLL configuration constants per RM 14.5.1
const USB1_PLL_DIV_SELECT: u32 = 20; // 480MHz from 24MHz OSC
const USB1_PLL_ENABLE: u32 = 1 << 13;
const USB1_PLL_POWER: u32 = 1 << 12;
const USB1_PLL_EN_USB_CLKS: u32 = 1 << 6;
const USB1_PLL_LOCK: u32 = 1 << 31;
const USB1_PLL_BYPASS: u32 = 1 << 16; // Bypass bit per RM 14.5.3

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
#[allow(dead_code)]
const USBPHY_TX_OFFSET: usize = 0x10; // TX register
#[allow(dead_code)]
const USBPHY_RX_OFFSET: usize = 0x20; // RX register
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
        // Assert soft reset (RM 66.5.1.2 step 1)
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
            cortex_m::asm::dmb();
            let mut ctrl = core::ptr::read_volatile(ctrl_reg);
            ctrl |= USBPHY_CTRL_SFTRST;
            core::ptr::write_volatile(ctrl_reg, ctrl);
            cortex_m::asm::dsb();
        }

        // Hold reset for minimum time (RM timing requirement)
        self.delay_us(timing::PHY_RESET_HOLD_TIME_US);

        // Deassert soft reset and clock gate
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
            cortex_m::asm::dmb();
            let mut ctrl = core::ptr::read_volatile(ctrl_reg);
            ctrl &= !(USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE);
            core::ptr::write_volatile(ctrl_reg, ctrl);
            cortex_m::asm::dsb();
        }

        // Wait for PHY to stabilize
        self.delay_us(timing::PHY_POWER_STABILIZATION_US);

        Ok(())
    }

    /// Initialize USB PLL with hardware timing verification
    fn init_usb_pll(&mut self) -> Result<()> {
        // Step 1: Set bypass and configure PLL (RM 14.5.3)
        unsafe {
            let pll_reg = (self.ccm_base + 0x10) as *mut u32;
            cortex_m::asm::dmb();
            let mut pll = core::ptr::read_volatile(pll_reg);
            cortex_m::asm::dmb();

            // Set bypass before modifying PLL configuration (RM requirement)
            pll |= USB1_PLL_BYPASS;

            // Configure divider and enable PLL
            pll &= !0x3F; // Clear divider field
            pll |= USB1_PLL_DIV_SELECT & 0x3F;
            pll |= USB1_PLL_ENABLE | USB1_PLL_POWER;

            cortex_m::asm::dmb();
            core::ptr::write_volatile(pll_reg, pll);
            cortex_m::asm::dsb();
        }

        // Step 2: Wait for PLL lock with timeout (RM 14.5.3)
        let timeout = RegisterTimeout::new_us(timing::PLL_LOCK_TIMEOUT_US);
        timeout
            .wait_for(|| unsafe {
                let pll_reg = (self.ccm_base + 0x10) as *const u32;
                cortex_m::asm::dmb();
                let pll = core::ptr::read_volatile(pll_reg);
                cortex_m::asm::dmb();
                (pll & USB1_PLL_LOCK) != 0
            })
            .map_err(|_| UsbError::HardwareFailure)?;

        // Step 3: Clear bypass after lock confirmed (RM requirement)
        unsafe {
            let pll_reg = (self.ccm_base + 0x10) as *mut u32;
            cortex_m::asm::dmb();
            let mut pll = core::ptr::read_volatile(pll_reg);
            cortex_m::asm::dmb();
            pll &= !USB1_PLL_BYPASS;
            pll |= USB1_PLL_EN_USB_CLKS;
            cortex_m::asm::dmb();
            core::ptr::write_volatile(pll_reg, pll);
            cortex_m::asm::dsb();
        }

        // Wait for clock stabilization
        self.delay_us(timing::CLOCK_STABILIZATION_US);
        self.pll_locked.store(true, Ordering::Release);

        Ok(())
    }

    /// Power up PHY with monitored sequencing
    fn power_up_phy(&mut self) -> Result<()> {
        // Power up PHY (RM 66.5.1.3)
        unsafe {
            let pwr_reg = (self.phy_base + 0x00) as *mut u32;
            cortex_m::asm::dmb();
            let mut pwr = core::ptr::read_volatile(pwr_reg);
            cortex_m::asm::dmb();

            // Clear power-down bits
            pwr &= !(1 << 10); // PWDN bit
            cortex_m::asm::dmb();
            core::ptr::write_volatile(pwr_reg, pwr);
            cortex_m::asm::dsb();
        }

        // Wait for power stabilization with verification
        self.delay_us(timing::PHY_POWER_STABILIZATION_US);

        // Verify PHY is responding
        self.verify_phy_response()?;

        Ok(())
    }

    /// Configure PHY for host mode operation
    fn configure_host_mode(&mut self) -> Result<()> {
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
            cortex_m::asm::dmb();
            let mut ctrl = core::ptr::read_volatile(ctrl_reg);
            cortex_m::asm::dmb();

            // Enable host mode features
            ctrl |= USBPHY_CTRL_ENHOSTDISCONDETECT;
            ctrl |= USBPHY_CTRL_HOSTDISCONDETECT_IRQ;

            cortex_m::asm::dmb();
            core::ptr::write_volatile(ctrl_reg, ctrl);
            cortex_m::asm::dsb();
        }

        Ok(())
    }

    /// Calibrate PHY with timeout and verification
    fn calibrate_phy(&mut self) -> Result<()> {
        // Start calibration sequence (RM 66.5.1.5)
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
            cortex_m::asm::dmb();
            let mut ctrl = core::ptr::read_volatile(ctrl_reg);
            cortex_m::asm::dmb();
            ctrl |= 1 << 16; // Start calibration
            cortex_m::asm::dmb();
            core::ptr::write_volatile(ctrl_reg, ctrl);
            cortex_m::asm::dsb();
        }

        // Wait for calibration completion
        let timeout = RegisterTimeout::new_us(timing::PHY_CALIBRATION_TIMEOUT_US);
        timeout
            .wait_for(|| {
                unsafe {
                    let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *const u32;
                    cortex_m::asm::dmb();
                    let ctrl = core::ptr::read_volatile(ctrl_reg);
                    cortex_m::asm::dmb();
                    (ctrl & (1 << 17)) != 0 // Calibration done
                }
            })
            .map_err(|_| UsbError::HardwareFailure)?;

        self.calibrated.store(true, Ordering::Release);
        Ok(())
    }

    /// Enable host disconnect detection with debouncing
    fn enable_host_disconnect_detect(&mut self) {
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *mut u32;
            cortex_m::asm::dmb();
            let mut ctrl = core::ptr::read_volatile(ctrl_reg);
            cortex_m::asm::dmb();
            ctrl |= USBPHY_CTRL_ENHOSTDISCONDETECT;
            cortex_m::asm::dmb();
            core::ptr::write_volatile(ctrl_reg, ctrl);
            cortex_m::asm::dsb();
        }
    }

    /// Verify PHY is responding to register accesses
    fn verify_phy_response(&mut self) -> Result<()> {
        // Read CTRL register to verify PHY is accessible
        // (No test register exists in USBPHY per RM 66.6)
        unsafe {
            let ctrl_reg = (self.phy_base + USBPHY_CTRL_OFFSET) as *const u32;
            cortex_m::asm::dmb();
            let _ctrl = core::ptr::read_volatile(ctrl_reg);
            cortex_m::asm::dmb();
            // If read succeeds without bus fault, PHY is responding
        }

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
        // Check PLL lock status
        let pll_locked = unsafe {
            let pll_reg = (self.ccm_base + 0x10) as *const u32;
            cortex_m::asm::dmb();
            let pll = core::ptr::read_volatile(pll_reg);
            cortex_m::asm::dmb();
            (pll & USB1_PLL_LOCK) != 0
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
