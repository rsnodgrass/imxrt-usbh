//! EHCI Controller with const generics for compile-time configuration
//!
//! Implements the improvements from the Rust expert review

use super::{PortSc, RegisterTimeout, UsbSts};
use crate::error::{Result, UsbError};
use core::marker::PhantomData;

/// Controller lifecycle states for compile-time state tracking
pub struct Uninitialized;
pub struct Initialized;
pub struct Running;

/// EHCI Controller with compile-time port count configuration
pub struct EhciController<const N_PORTS: usize = 8, State = Uninitialized>
where
    [(); N_PORTS]: Sized,
{
    /// Base address of the controller
    base_addr: usize,

    /// Cached capability values for performance
    port_count: u8,
    has_64bit_addr: bool,
    has_port_power_control: bool,

    /// Pre-allocated port state cache for zero-allocation operation
    port_states: [core::sync::atomic::AtomicU32; N_PORTS],

    /// Phantom state marker
    _state: PhantomData<State>,
}

impl<const N_PORTS: usize> EhciController<N_PORTS, Uninitialized>
where
    [(); N_PORTS]: Sized,
{
    /// Create a new EHCI controller instance
    ///
    /// # Safety
    ///
    /// The caller must ensure exclusive access to the EHCI controller at base_addr
    pub unsafe fn new(base_addr: usize) -> Result<Self> {
        // Read capability registers to validate hardware configuration
        let _cap_base = unsafe { core::ptr::read_volatile((base_addr + 0x00) as *const u32) };
        let hcsparams = unsafe { core::ptr::read_volatile((base_addr + 0x04) as *const u32) };
        let hccparams = unsafe { core::ptr::read_volatile((base_addr + 0x08) as *const u32) };

        let hw_port_count = (hcsparams & 0xF) as usize;

        // Validate that hardware port count matches generic parameter
        if hw_port_count != N_PORTS {
            return Err(UsbError::InvalidParameter);
        }

        Ok(Self {
            base_addr,
            port_count: hw_port_count as u8,
            has_64bit_addr: (hccparams & 0x01) != 0,
            has_port_power_control: (hcsparams & 0x10) != 0,
            port_states: [const { core::sync::atomic::AtomicU32::new(0) }; N_PORTS],
            _state: PhantomData,
        })
    }

    /// Initialize the controller with proper reset sequence
    pub unsafe fn initialize(self) -> Result<EhciController<N_PORTS, Initialized>> {
        let op_base = self.operational_base();

        // Reset controller (RM 66.6.10 USBCMD[1])
        let usbcmd_ptr = (op_base + 0x00) as *mut u32;
        let mut usbcmd = unsafe { core::ptr::read_volatile(usbcmd_ptr) };
        usbcmd |= 0x02; // HC Reset
        unsafe { core::ptr::write_volatile(usbcmd_ptr, usbcmd) };

        // Wait for reset completion with timeout
        let timeout = RegisterTimeout::new_us(10_000);
        timeout.wait_for(|| {
            let cmd = unsafe { core::ptr::read_volatile(usbcmd_ptr) };
            (cmd & 0x02) == 0 // Reset bit clears when complete
        })?;

        Ok(EhciController {
            base_addr: self.base_addr,
            port_count: self.port_count,
            has_64bit_addr: self.has_64bit_addr,
            has_port_power_control: self.has_port_power_control,
            port_states: self.port_states,
            _state: PhantomData,
        })
    }
}

impl<const N_PORTS: usize> EhciController<N_PORTS, Initialized>
where
    [(); N_PORTS]: Sized,
{
    /// Start the controller
    pub unsafe fn start(self) -> EhciController<N_PORTS, Running> {
        let op_base = self.operational_base();
        let usbcmd_ptr = (op_base + 0x00) as *mut u32;

        let mut usbcmd = unsafe { core::ptr::read_volatile(usbcmd_ptr) };
        usbcmd |= 0x01; // Run/Stop bit
        unsafe { core::ptr::write_volatile(usbcmd_ptr, usbcmd) };

        EhciController {
            base_addr: self.base_addr,
            port_count: self.port_count,
            has_64bit_addr: self.has_64bit_addr,
            has_port_power_control: self.has_port_power_control,
            port_states: self.port_states,
            _state: PhantomData,
        }
    }
}

impl<const N_PORTS: usize, State> EhciController<N_PORTS, State>
where
    [(); N_PORTS]: Sized,
{
    /// Get port count (cached for performance)
    #[inline(always)]
    pub const fn port_count(&self) -> u8 {
        self.port_count
    }

    /// Check if controller supports 64-bit addressing
    #[inline(always)]
    pub const fn has_64bit_addressing(&self) -> bool {
        self.has_64bit_addr
    }

    /// Check if controller has port power control
    #[inline(always)]
    pub const fn has_port_power_control(&self) -> bool {
        self.has_port_power_control
    }

    /// Type-safe port operations with bounds checking
    pub fn port_status(&self, port: usize) -> Result<PortSc> {
        if port >= N_PORTS {
            return Err(UsbError::InvalidParameter);
        }
        let op_base = self.operational_base();
        let portsc_ptr = (op_base + 0x44 + (port * 4)) as *const u32;
        let portsc_val = unsafe { core::ptr::read_volatile(portsc_ptr) };
        Ok(PortSc::from_bits_truncate(portsc_val))
    }

    /// Get operational registers base address
    fn operational_base(&self) -> usize {
        let cap_base = unsafe { core::ptr::read_volatile(self.base_addr as *const u32) };
        let cap_length = (cap_base & 0xFF) as usize;
        self.base_addr + cap_length
    }
}

impl<const N_PORTS: usize> EhciController<N_PORTS, Running>
where
    [(); N_PORTS]: Sized,
{
    /// Process USB transfers (only available when running)
    pub fn process_transfers(&mut self) -> Result<()> {
        // Transfer processing logic will be implemented later
        Ok(())
    }

    /// Check controller status
    pub fn status(&self) -> UsbSts {
        let op_base = self.operational_base();
        let usbsts_ptr = (op_base + 0x04) as *const u32;
        let status_val = unsafe { core::ptr::read_volatile(usbsts_ptr) };
        UsbSts::from_bits_truncate(status_val)
    }
}

/// Builder pattern for controller configuration
pub struct EhciControllerBuilder<const N_PORTS: usize = 8>
where
    [(); N_PORTS]: Sized,
{
    base_addr: usize,
    interrupt_threshold: u8,
    async_park_mode: bool,
}

impl<const N_PORTS: usize> EhciControllerBuilder<N_PORTS>
where
    [(); N_PORTS]: Sized,
{
    /// Create new controller builder
    pub const fn new(base_addr: usize) -> Self {
        Self {
            base_addr,
            interrupt_threshold: 1,
            async_park_mode: true,
        }
    }

    /// Set interrupt threshold (microframes)
    pub const fn interrupt_threshold(mut self, threshold: u8) -> Self {
        self.interrupt_threshold = threshold;
        self
    }

    /// Enable/disable async park mode
    pub const fn async_park_mode(mut self, enabled: bool) -> Self {
        self.async_park_mode = enabled;
        self
    }

    /// Build the controller
    pub unsafe fn build(self) -> Result<EhciController<N_PORTS, Uninitialized>> {
        unsafe { EhciController::new(self.base_addr) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_const_generic_bounds() {
        // This should compile - valid port index
        fn test_valid_port<const N_PORTS: usize>(_controller: &EhciController<N_PORTS>)
        where
            [(); N_PORTS]: Sized,
            Assert<{ 0 < N_PORTS }>: IsTrue,
        {
            // Would call controller.port_status::<0>() here
        }

        // This demonstrates compile-time port validation
        let _test = test_valid_port::<8>;
    }
}
