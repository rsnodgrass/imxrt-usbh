//! EHCI (Enhanced Host Controller Interface) implementation for i.MX RT1062
//!
//! This module provides memory-mapped register structures and bit definitions
//! for the USB EHCI controller on the i.MX RT1062 microcontroller.
//!
//! # Safety
//!
//! All register access must be performed through proper synchronization mechanisms.
//! The EHCI controller registers are documented in:
//! - i.MX RT1060 Reference Manual, Chapter 66.6 (USB Host Controller Registers)
//! - EHCI Specification Section 2 (Host Controller Interface)
//!
//! # Register Memory Layout
//!
//! The EHCI controller registers are divided into:
//! - Capability Registers (read-only, offset 0x000-0x00F)
//! - Operational Registers (read-write, offset varies by CAPLENGTH)

pub mod register;
pub mod qtd;
pub mod qh;
pub mod controller;

pub use qtd::QueueTD;
pub use qh::QueueHead;
pub use register::{Register, RegisterTimeout};
pub use controller::{EhciController, EhciControllerBuilder, Uninitialized, Initialized, Running};

// Re-export important types for easier use
pub use qtd::token;
pub use qh::{endpoint, capabilities};

use crate::error::{Result, UsbError};
use bitflags::bitflags;

/// Type-safe port identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PortId(u8);

impl PortId {
    /// Create a new port ID, validating range
    pub const fn new(port: u8) -> Result<Self> {
        if port >= 8 {
            Err(UsbError::InvalidParameter)
        } else {
            Ok(Self(port))
        }
    }
    
    /// Get the port index as usize for array access
    #[inline(always)]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
    
    /// Get the raw port number
    #[inline(always)]
    pub const fn value(self) -> u8 {
        self.0
    }
}

impl From<u8> for PortId {
    fn from(port: u8) -> Self {
        Self(port.min(7)) // Clamp to valid range
    }
}

impl TryFrom<usize> for PortId {
    type Error = UsbError;
    
    fn try_from(port: usize) -> Result<Self> {
        if port >= 8 {
            Err(UsbError::InvalidParameter)
        } else {
            Ok(Self(port as u8))
        }
    }
}

/// Compile-time assertion helper for const generic bounds checking
pub struct Assert<const COND: bool>;

/// Trait to enable compile-time assertions
pub trait IsTrue {}

impl IsTrue for Assert<true> {}

/// Base address for USB1 EHCI controller on i.MX RT1062
pub const USB1_BASE: usize = 0x402E_0000;

/// Base address for USB2 EHCI controller on i.MX RT1062
pub const USB2_BASE: usize = 0x402E_0400;


/// Port speed detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortSpeed {
    FullSpeed,   // 12 Mbps
    LowSpeed,    // 1.5 Mbps
    HighSpeed,   // 480 Mbps
    Unknown,
}

/// USB line state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineState {
    SE0,      // Single-ended zero
    JState,   // J-state
    KState,   // K-state
    SE1,      // Single-ended one (invalid)
}

/// EHCI Capability Registers
#[repr(C)]
pub struct EhciCapabilityRegisters {
    /// Capability Parameters Register (HCCAPBASE)
    pub hccapbase: Register<u32>,
    /// Host Controller Structural Parameters (HCSPARAMS)
    pub hcsparams: Register<u32>,
    /// Host Controller Capability Parameters (HCCPARAMS)
    pub hccparams: Register<u32>,
    /// Host Controller Companion Port Route Description
    pub hcsp_portroute: Register<u32>,
}

/// EHCI Operational Registers
#[repr(C)]
pub struct EhciOperationalRegisters {
    /// USB Command Register (USBCMD)
    pub usbcmd: Register<u32>,
    /// USB Status Register (USBSTS)
    pub usbsts: Register<u32>,
    /// USB Interrupt Enable Register (USBINTR)
    pub usbintr: Register<u32>,
    /// USB Frame Index Register (FRINDEX)
    pub frindex: Register<u32>,
    /// Control Data Structure Segment Register (CTRLDSSEGMENT)
    pub ctrldssegment: Register<u32>,
    /// Periodic Frame List Base Address Register (PERIODICLISTBASE)
    pub periodiclistbase: Register<u32>,
    /// Asynchronous List Address Register (ASYNCLISTADDR)
    pub asynclistaddr: Register<u32>,
    /// Reserved space
    _reserved0: [u32; 9],
    /// Configured Flag Register (CONFIGFLAG)
    pub configflag: Register<u32>,
    /// Port Status and Control Register (PORTSC)
    pub portsc: [Register<u32>; 8], // i.MX RT1062 supports up to 8 ports per controller
}

bitflags! {
    /// USB Command Register (USBCMD) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct UsbCmd: u32 {
        /// Run/Stop (RS) - Bit 0
        const RUN_STOP = 1 << 0;
        /// Host Controller Reset (HCRESET) - Bit 1
        const HC_RESET = 1 << 1;
        /// Frame List Size - Bits [3:2]
        const FRAME_LIST_SIZE_MASK = 0b11 << 2;
        const FRAME_LIST_SIZE_1024 = 0b00 << 2;
        const FRAME_LIST_SIZE_512 = 0b01 << 2;
        const FRAME_LIST_SIZE_256 = 0b10 << 2;
        /// Periodic Schedule Enable (PSE) - Bit 4
        const PERIODIC_SCHEDULE_ENABLE = 1 << 4;
        /// Asynchronous Schedule Enable (ASE) - Bit 5
        const ASYNC_SCHEDULE_ENABLE = 1 << 5;
        /// Interrupt on Async Advance Doorbell (IAAD) - Bit 6
        const INTERRUPT_ON_ASYNC_ADVANCE = 1 << 6;
        /// Light Host Controller Reset (LHCRESET) - Bit 7
        const LIGHT_HC_RESET = 1 << 7;
        /// Asynchronous Schedule Park Mode Count - Bits [9:8]
        const ASYNC_PARK_MODE_COUNT_MASK = 0b11 << 8;
        /// Asynchronous Schedule Park Mode Enable (ASPME) - Bit 11
        const ASYNC_PARK_MODE_ENABLE = 1 << 11;
        /// Interrupt Threshold Control - Bits [23:16]
        const INTERRUPT_THRESHOLD_MASK = 0xFF << 16;
    }
}

bitflags! {
    /// USB Status Register (USBSTS) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct UsbSts: u32 {
        /// USB Interrupt (USBINT) - Bit 0
        const USB_INTERRUPT = 1 << 0;
        /// USB Error Interrupt (USBERRINT) - Bit 1
        const USB_ERROR_INTERRUPT = 1 << 1;
        /// Port Change Detect (PCD) - Bit 2
        const PORT_CHANGE_DETECT = 1 << 2;
        /// Frame List Rollover (FLR) - Bit 3
        const FRAME_LIST_ROLLOVER = 1 << 3;
        /// Host System Error (HSE) - Bit 4
        const HOST_SYSTEM_ERROR = 1 << 4;
        /// Interrupt on Async Advance (IAA) - Bit 5
        const INTERRUPT_ON_ASYNC_ADVANCE = 1 << 5;
        /// Host Controller Halted (HCHalted) - Bit 12
        const HC_HALTED = 1 << 12;
        /// Reclamation (Reclamation) - Bit 13
        const RECLAMATION = 1 << 13;
        /// Periodic Schedule Status (PSS) - Bit 14
        const PERIODIC_SCHEDULE_STATUS = 1 << 14;
        /// Asynchronous Schedule Status (ASS) - Bit 15
        const ASYNC_SCHEDULE_STATUS = 1 << 15;
    }
}

bitflags! {
    /// USB Interrupt Enable Register (USBINTR) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct UsbIntr: u32 {
        /// USB Interrupt Enable - Bit 0
        const USB_INTERRUPT_ENABLE = 1 << 0;
        /// USB Error Interrupt Enable - Bit 1
        const USB_ERROR_INTERRUPT_ENABLE = 1 << 1;
        /// Port Change Interrupt Enable - Bit 2
        const PORT_CHANGE_INTERRUPT_ENABLE = 1 << 2;
        /// Frame List Rollover Enable - Bit 3
        const FRAME_LIST_ROLLOVER_ENABLE = 1 << 3;
        /// Host System Error Enable - Bit 4
        const HOST_SYSTEM_ERROR_ENABLE = 1 << 4;
        /// Interrupt on Async Advance Enable - Bit 5
        const INTERRUPT_ON_ASYNC_ADVANCE_ENABLE = 1 << 5;
    }
}

bitflags! {
    /// Port Status and Control Register (PORTSC) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PortSc: u32 {
        /// Current Connect Status (CCS) - Bit 0
        const CURRENT_CONNECT_STATUS = 1 << 0;
        /// Connect Status Change (CSC) - Bit 1
        const CONNECT_STATUS_CHANGE = 1 << 1;
        /// Port Enabled/Disabled (PED) - Bit 2
        const PORT_ENABLED = 1 << 2;
        /// Port Enable/Disable Change (PEDC) - Bit 3
        const PORT_ENABLE_CHANGE = 1 << 3;
        /// Over-current Active (OCA) - Bit 4
        const OVER_CURRENT_ACTIVE = 1 << 4;
        /// Over-current Change (OCC) - Bit 5
        const OVER_CURRENT_CHANGE = 1 << 5;
        /// Force Port Resume (FPR) - Bit 6
        const FORCE_PORT_RESUME = 1 << 6;
        /// Suspend (SUSP) - Bit 7
        const SUSPEND = 1 << 7;
        /// Port Reset (PR) - Bit 8
        const PORT_RESET = 1 << 8;
        /// Line Status - Bits [11:10]
        const LINE_STATUS_MASK = 0b11 << 10;
        const LINE_STATUS_SE0 = 0b00 << 10;
        const LINE_STATUS_K_STATE = 0b01 << 10;
        const LINE_STATUS_J_STATE = 0b10 << 10;
        /// Port Power (PP) - Bit 12
        const PORT_POWER = 1 << 12;
        /// Port Owner (PO) - Bit 13
        const PORT_OWNER = 1 << 13;
        /// Port Indicator Control - Bits [15:14]
        const PORT_INDICATOR_MASK = 0b11 << 14;
        const PORT_INDICATOR_OFF = 0b00 << 14;
        const PORT_INDICATOR_AMBER = 0b01 << 14;
        const PORT_INDICATOR_GREEN = 0b10 << 14;
        /// Port Test Control - Bits [19:16]
        const PORT_TEST_CONTROL_MASK = 0xF << 16;
        const PORT_TEST_J_STATE = 0x1 << 16;
        const PORT_TEST_K_STATE = 0x2 << 16;
        const PORT_TEST_SE0_NAK = 0x3 << 16;
        const PORT_TEST_PACKET = 0x4 << 16;
        const PORT_TEST_FORCE_ENABLE = 0x5 << 16;
        /// Wake on Connect Enable (WKCNNT_E) - Bit 20
        const WAKE_ON_CONNECT_ENABLE = 1 << 20;
        /// Wake on Disconnect Enable (WKDSCNNT_E) - Bit 21
        const WAKE_ON_DISCONNECT_ENABLE = 1 << 21;
        /// Wake on Over-current Enable (WKOC_E) - Bit 22
        const WAKE_ON_OVER_CURRENT_ENABLE = 1 << 22;
        /// Port Speed - Bits [27:26] (i.MX RT specific extension)
        const PORT_SPEED_MASK = 0b11 << 26;
        const PORT_SPEED_FULL = 0b00 << 26;
        const PORT_SPEED_LOW = 0b01 << 26;
        const PORT_SPEED_HIGH = 0b10 << 26;
    }
}

bitflags! {
    /// Host Controller Capability Parameters (HCCPARAMS) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct HcCparams: u32 {
        /// 64-bit Addressing Capability (ADC) - Bit 0
        const ADDRESSING_64BIT = 1 << 0;
        /// Programmable Frame List Flag (PFL) - Bit 1
        const PROGRAMMABLE_FRAME_LIST = 1 << 1;
        /// Asynchronous Schedule Park Capability (ASP) - Bit 2
        const ASYNC_SCHEDULE_PARK = 1 << 2;
        /// Isochronous Scheduling Threshold (IST) - Bits [7:4]
        const ISOC_SCHEDULING_THRESHOLD_MASK = 0xF << 4;
        /// EHCI Extended Capabilities Pointer (EECP) - Bits [15:8]
        const EXTENDED_CAPABILITIES_POINTER_MASK = 0xFF << 8;
    }
}

bitflags! {
    /// Host Controller Structural Parameters (HCSPARAMS) bit definitions
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct HcSparams: u32 {
        /// Number of Ports (N_PORTS) - Bits [3:0]
        const N_PORTS_MASK = 0xF;
        /// Port Power Control (PPC) - Bit 4
        const PORT_POWER_CONTROL = 1 << 4;
        /// Number of Ports per Companion Controller (N_PCC) - Bits [11:8]
        const N_PCC_MASK = 0xF << 8;
        /// Number of Companion Controllers (N_CC) - Bits [15:12]
        const N_CC_MASK = 0xF << 12;
        /// Port Indicators (P_INDICATOR) - Bit 16
        const PORT_INDICATORS = 1 << 16;
        /// Debug Port Number (DEBUG_PORT_NUMBER) - Bits [23:20]
        const DEBUG_PORT_NUMBER_MASK = 0xF << 20;
    }
}

/// Timeout constants per USB 2.0 specification (in microseconds)
pub mod timeouts {
    /// Host controller reset timeout (250ms per EHCI spec)
    pub const HC_RESET_TIMEOUT_US: u32 = 250_000;
    /// Port reset timeout (50ms per USB 2.0 spec)
    pub const PORT_RESET_TIMEOUT_US: u32 = 50_000;
    /// Port reset assertion time (20ms minimum per USB 2.0 spec)
    pub const PORT_RESET_ASSERT_TIME_US: u32 = 20_000;
    /// Schedule enable/disable timeout (2ms)
    pub const SCHEDULE_TIMEOUT_US: u32 = 2_000;
    /// Controller halt timeout (16 microframes = ~2ms)
    pub const HALT_TIMEOUT_US: u32 = 2_000;
}

// Ensure proper alignment and size of register structures
const _: () = {
    assert!(core::mem::size_of::<EhciCapabilityRegisters>() == 16);
    assert!(core::mem::align_of::<EhciCapabilityRegisters>() == 4);
    assert!(core::mem::size_of::<EhciOperationalRegisters>() >= 68);
    assert!(core::mem::align_of::<EhciOperationalRegisters>() == 4);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bitflag_definitions() {
        assert_eq!(UsbCmd::RUN_STOP.bits(), 1);
        assert_eq!(UsbCmd::HC_RESET.bits(), 2);
        assert_eq!(UsbSts::USB_INTERRUPT.bits(), 1);
        assert_eq!(PortSc::CURRENT_CONNECT_STATUS.bits(), 1);
        assert_eq!(PortSc::PORT_RESET.bits(), 1 << 8);
    }

    #[test]
    fn test_register_structure_sizes() {
        assert_eq!(core::mem::size_of::<EhciCapabilityRegisters>(), 16);
        assert!(core::mem::size_of::<EhciOperationalRegisters>() >= 68);
    }
}