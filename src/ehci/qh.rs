//! Queue Head (qH) implementation for EHCI
//!
//! Based on EHCI Specification Section 3.6

use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicU32, Ordering};

/// Endpoint characteristics field bits
#[allow(missing_docs)]
pub mod endpoint {
    pub const DEVICE_ADDRESS_SHIFT: u32 = 0;
    pub const DEVICE_ADDRESS_MASK: u32 = 0x7F;

    pub const INACTIVE_ON_NEXT: u32 = 1 << 7;

    pub const ENDPOINT_NUMBER_SHIFT: u32 = 8;
    pub const ENDPOINT_NUMBER_MASK: u32 = 0xF;

    pub const ENDPOINT_SPEED_SHIFT: u32 = 12;
    pub const ENDPOINT_SPEED_MASK: u32 = 0x3;
    pub const SPEED_FULL: u32 = 0;
    pub const SPEED_LOW: u32 = 1;
    pub const SPEED_HIGH: u32 = 2;

    pub const DATA_TOGGLE_CONTROL: u32 = 1 << 14;
    pub const HEAD_OF_LIST: u32 = 1 << 15;

    pub const MAX_PACKET_LENGTH_SHIFT: u32 = 16;
    pub const MAX_PACKET_LENGTH_MASK: u32 = 0x7FF;

    pub const CONTROL_ENDPOINT: u32 = 1 << 27;

    pub const NAK_COUNT_RELOAD_SHIFT: u32 = 28;
    pub const NAK_COUNT_RELOAD_MASK: u32 = 0xF;
}

/// Endpoint capabilities field bits
pub mod capabilities {
    pub const INTERRUPT_SCHEDULE_MASK_SHIFT: u32 = 0;
    pub const INTERRUPT_SCHEDULE_MASK_MASK: u32 = 0xFF;

    pub const SPLIT_COMPLETION_MASK_SHIFT: u32 = 8;
    pub const SPLIT_COMPLETION_MASK_MASK: u32 = 0xFF;

    pub const HUB_ADDRESS_SHIFT: u32 = 16;
    pub const HUB_ADDRESS_MASK: u32 = 0x7F;

    pub const PORT_NUMBER_SHIFT: u32 = 23;
    pub const PORT_NUMBER_MASK: u32 = 0x7F;

    pub const MULT_SHIFT: u32 = 30;
    pub const MULT_MASK: u32 = 0x3;
}

/// Queue Head (qH)
///
/// EHCI Specification Section 3.6
/// Must be 32-byte aligned for DMA
#[repr(C, align(32))]
pub struct QueueHead {
    /// Horizontal link pointer to next qH (bit 0 = terminate, bits 2:1 = type)
    pub horizontal_link: AtomicU32,

    /// Endpoint characteristics
    pub endpoint_chars: AtomicU32,

    /// Endpoint capabilities (split transaction, multiplier)
    pub endpoint_caps: AtomicU32,

    /// Current qTD pointer (overlay area begins here)
    pub current_qtd: AtomicU32,

    // Queue Head Overlay Area (mirrors qTD structure)
    /// Next qTD pointer
    pub next_qtd: AtomicU32,

    /// Alternate next qTD
    pub alt_next_qtd: AtomicU32,

    /// Token (status and control)
    pub token: AtomicU32,

    /// Buffer pointers
    pub buffer_pointers: [AtomicU32; 5],

    /// Extended buffer pointers (unused on RT1062)
    pub ext_buffer_pointers: [AtomicU32; 5],

    // Software-only fields
    /// Device address for this endpoint
    pub device_id: AtomicU32,

    /// Reserved for alignment
    _reserved: [u32; 3],
}

impl QueueHead {
    /// Type field values for horizontal link
    #[allow(missing_docs)]
    pub const TYPE_ITD: u32 = 0 << 1;
    #[allow(missing_docs)]
    pub const TYPE_QH: u32 = 1 << 1;
    #[allow(missing_docs)]
    pub const TYPE_SITD: u32 = 2 << 1;
    #[allow(missing_docs)]
    pub const TYPE_FSTN: u32 = 3 << 1;

    /// Terminator bit
    pub const TERMINATE: u32 = 1;

    /// Create new inactive Queue Head
    pub const fn new() -> Self {
        Self {
            horizontal_link: AtomicU32::new(Self::TERMINATE),
            endpoint_chars: AtomicU32::new(0),
            endpoint_caps: AtomicU32::new(0),
            current_qtd: AtomicU32::new(Self::TERMINATE),
            next_qtd: AtomicU32::new(Self::TERMINATE),
            alt_next_qtd: AtomicU32::new(Self::TERMINATE),
            token: AtomicU32::new(0),
            buffer_pointers: [
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
            ],
            ext_buffer_pointers: [
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
            ],
            device_id: AtomicU32::new(0),
            _reserved: [0; 3],
        }
    }

    /// Initialize Queue Head for an endpoint
    pub fn init_endpoint(
        &self,
        device_addr: u8,
        endpoint_num: u8,
        max_packet_size: u16,
        speed: u32,
        is_control: bool,
    ) -> Result<()> {
        if device_addr > 127 {
            return Err(UsbError::InvalidParameter);
        }
        if endpoint_num > 15 {
            return Err(UsbError::InvalidParameter);
        }
        if max_packet_size > 1024 {
            return Err(UsbError::InvalidParameter);
        }

        let mut chars = 0u32;
        chars |= (device_addr as u32) << endpoint::DEVICE_ADDRESS_SHIFT;
        chars |= (endpoint_num as u32) << endpoint::ENDPOINT_NUMBER_SHIFT;
        chars |= speed << endpoint::ENDPOINT_SPEED_SHIFT;
        chars |= (max_packet_size as u32) << endpoint::MAX_PACKET_LENGTH_SHIFT;

        if is_control {
            chars |= endpoint::CONTROL_ENDPOINT;
            chars |= endpoint::DATA_TOGGLE_CONTROL; // Use qTD data toggle
        }

        // Set NAK count reload for bulk/control endpoints
        if speed == endpoint::SPEED_HIGH {
            chars |= 0 << endpoint::NAK_COUNT_RELOAD_SHIFT; // No NAK count
        } else {
            chars |= 4 << endpoint::NAK_COUNT_RELOAD_SHIFT; // Reload 4
        }

        self.endpoint_chars.store(chars, Ordering::Release);

        // Set multiplier for high-bandwidth endpoints
        let mut caps = 0u32;
        caps |= 1 << capabilities::MULT_SHIFT; // Mult = 1 (1 transaction per microframe)

        self.endpoint_caps.store(caps, Ordering::Release);

        // Store device ID for software use
        self.device_id.store(device_addr as u32, Ordering::Release);

        Ok(())
    }

    /// Configure for split transaction (full/low-speed device behind hub)
    pub fn configure_split(
        &self,
        hub_addr: u8,
        hub_port: u8,
        start_mask: u8,
        complete_mask: u8,
    ) -> Result<()> {
        if hub_addr > 127 || hub_port > 127 {
            return Err(UsbError::InvalidParameter);
        }

        let mut caps = self.endpoint_caps.load(Ordering::Acquire);

        caps &= !(capabilities::HUB_ADDRESS_MASK << capabilities::HUB_ADDRESS_SHIFT);
        caps &= !(capabilities::PORT_NUMBER_MASK << capabilities::PORT_NUMBER_SHIFT);
        caps &= !(capabilities::SPLIT_COMPLETION_MASK_MASK
            << capabilities::SPLIT_COMPLETION_MASK_SHIFT);
        caps &= !(capabilities::INTERRUPT_SCHEDULE_MASK_MASK
            << capabilities::INTERRUPT_SCHEDULE_MASK_SHIFT);

        caps |= (hub_addr as u32) << capabilities::HUB_ADDRESS_SHIFT;
        caps |= (hub_port as u32) << capabilities::PORT_NUMBER_SHIFT;
        caps |= (complete_mask as u32) << capabilities::SPLIT_COMPLETION_MASK_SHIFT;
        caps |= (start_mask as u32) << capabilities::INTERRUPT_SCHEDULE_MASK_SHIFT;

        self.endpoint_caps.store(caps, Ordering::Release);

        Ok(())
    }

    /// Link a qTD to this Queue Head
    ///
    /// # Safety
    ///
    /// The qTD must remain valid and properly aligned for DMA access.
    pub unsafe fn link_qtd(&self, qtd_addr: u32) -> Result<()> {
        if qtd_addr & 0x1F != 0 {
            return Err(UsbError::InvalidParameter);
        }

        // Set next qTD in overlay area
        self.next_qtd.store(qtd_addr, Ordering::Release);

        // Clear current qTD to trigger fetch of next qTD
        self.current_qtd.store(0, Ordering::Release);

        Ok(())
    }

    /// Check if Queue Head is active
    pub fn is_active(&self) -> bool {
        let token = self.token.load(Ordering::Acquire);
        (token & super::qtd::token::STATUS_ACTIVE) != 0
    }

    /// Reset Queue Head overlay area
    pub fn reset_overlay(&self) {
        self.current_qtd.store(Self::TERMINATE, Ordering::Release);
        self.next_qtd.store(Self::TERMINATE, Ordering::Release);
        self.alt_next_qtd.store(Self::TERMINATE, Ordering::Release);
        self.token.store(0, Ordering::Release);

        for i in 0..5 {
            self.buffer_pointers[i].store(0, Ordering::Release);
        }
    }

    /// Mark as head of async schedule list
    pub fn set_head_of_list(&self) {
        self.endpoint_chars
            .fetch_or(endpoint::HEAD_OF_LIST, Ordering::SeqCst);
    }
}

// Ensure size is correct per EHCI spec
const _: () = assert!(core::mem::size_of::<QueueHead>() == 96);
const _: () = assert!(core::mem::align_of::<QueueHead>() >= 32);

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering;

    #[test]
    fn test_qh_creation() {
        let qh = QueueHead::new();

        // Verify initial state
        assert_eq!(
            qh.horizontal_link.load(Ordering::Relaxed),
            QueueHead::TERMINATE
        );
        assert_eq!(qh.current_qtd.load(Ordering::Relaxed), QueueHead::TERMINATE);
        assert_eq!(qh.next_qtd.load(Ordering::Relaxed), QueueHead::TERMINATE);
        assert_eq!(qh.endpoint_chars.load(Ordering::Relaxed), 0);
        assert_eq!(qh.endpoint_caps.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_qh_endpoint_init() {
        let qh = QueueHead::new();

        // Test valid initialization
        let result = qh.init_endpoint(5, 1, 512, endpoint::SPEED_HIGH, false);
        assert!(result.is_ok());

        let chars = qh.endpoint_chars.load(Ordering::Relaxed);

        // Verify device address
        assert_eq!(
            (chars >> endpoint::DEVICE_ADDRESS_SHIFT) & endpoint::DEVICE_ADDRESS_MASK,
            5
        );

        // Verify endpoint number
        assert_eq!(
            (chars >> endpoint::ENDPOINT_NUMBER_SHIFT) & endpoint::ENDPOINT_NUMBER_MASK,
            1
        );

        // Verify max packet size
        assert_eq!(
            (chars >> endpoint::MAX_PACKET_LENGTH_SHIFT) & endpoint::MAX_PACKET_LENGTH_MASK,
            512
        );

        // Verify speed
        assert_eq!(
            (chars >> endpoint::ENDPOINT_SPEED_SHIFT) & endpoint::ENDPOINT_SPEED_MASK,
            endpoint::SPEED_HIGH
        );

        // Should not be control endpoint
        assert_eq!(chars & endpoint::CONTROL_ENDPOINT, 0);
    }

    #[test]
    fn test_qh_control_endpoint_init() {
        let qh = QueueHead::new();

        let result = qh.init_endpoint(1, 0, 64, endpoint::SPEED_FULL, true);
        assert!(result.is_ok());

        let chars = qh.endpoint_chars.load(Ordering::Relaxed);

        // Should be marked as control endpoint
        assert_ne!(chars & endpoint::CONTROL_ENDPOINT, 0);
        assert_ne!(chars & endpoint::DATA_TOGGLE_CONTROL, 0);

        // Verify NAK count reload for non-high-speed
        let nak_count =
            (chars >> endpoint::NAK_COUNT_RELOAD_SHIFT) & endpoint::NAK_COUNT_RELOAD_MASK;
        assert_eq!(nak_count, 4);
    }

    #[test]
    fn test_qh_invalid_parameters() {
        let qh = QueueHead::new();

        // Test invalid device address (>127)
        let result = qh.init_endpoint(128, 0, 64, endpoint::SPEED_HIGH, true);
        assert!(matches!(result, Err(UsbError::InvalidParameter)));

        // Test invalid endpoint number (>15)
        let result = qh.init_endpoint(1, 16, 64, endpoint::SPEED_HIGH, true);
        assert!(matches!(result, Err(UsbError::InvalidParameter)));

        // Test invalid max packet size (>1024)
        let result = qh.init_endpoint(1, 0, 1025, endpoint::SPEED_HIGH, true);
        assert!(matches!(result, Err(UsbError::InvalidParameter)));
    }

    #[test]
    fn test_qh_split_transaction_config() {
        let qh = QueueHead::new();

        // First initialize endpoint
        qh.init_endpoint(1, 1, 64, endpoint::SPEED_FULL, false)
            .unwrap();

        // Configure for split transaction
        let result = qh.configure_split(3, 2, 0x01, 0x02);
        assert!(result.is_ok());

        let caps = qh.endpoint_caps.load(Ordering::Relaxed);

        // Verify hub address
        assert_eq!(
            (caps >> capabilities::HUB_ADDRESS_SHIFT) & capabilities::HUB_ADDRESS_MASK,
            3
        );

        // Verify hub port
        assert_eq!(
            (caps >> capabilities::PORT_NUMBER_SHIFT) & capabilities::PORT_NUMBER_MASK,
            2
        );

        // Verify split completion mask
        assert_eq!(
            (caps >> capabilities::SPLIT_COMPLETION_MASK_SHIFT)
                & capabilities::SPLIT_COMPLETION_MASK_MASK,
            0x02
        );

        // Verify interrupt schedule mask
        assert_eq!(
            (caps >> capabilities::INTERRUPT_SCHEDULE_MASK_SHIFT)
                & capabilities::INTERRUPT_SCHEDULE_MASK_MASK,
            0x01
        );
    }

    #[test]
    fn test_qh_split_invalid_params() {
        let qh = QueueHead::new();

        // Test invalid hub address (>127)
        let result = qh.configure_split(128, 1, 0x01, 0x02);
        assert!(matches!(result, Err(UsbError::InvalidParameter)));

        // Test invalid hub port (>127)
        let result = qh.configure_split(1, 128, 0x01, 0x02);
        assert!(matches!(result, Err(UsbError::InvalidParameter)));
    }

    #[test]
    fn test_qh_overlay_reset() {
        let qh = QueueHead::new();

        // Set some values in overlay
        qh.current_qtd.store(0x1000, Ordering::Relaxed);
        qh.next_qtd.store(0x2000, Ordering::Relaxed);
        qh.token.store(0x3000, Ordering::Relaxed);
        qh.buffer_pointers[0].store(0x4000, Ordering::Relaxed);

        // Reset overlay
        qh.reset_overlay();

        // Verify reset
        assert_eq!(qh.current_qtd.load(Ordering::Relaxed), QueueHead::TERMINATE);
        assert_eq!(qh.next_qtd.load(Ordering::Relaxed), QueueHead::TERMINATE);
        assert_eq!(
            qh.alt_next_qtd.load(Ordering::Relaxed),
            QueueHead::TERMINATE
        );
        assert_eq!(qh.token.load(Ordering::Relaxed), 0);
        assert_eq!(qh.buffer_pointers[0].load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_qh_head_of_list() {
        let qh = QueueHead::new();

        // Initially not head of list
        let chars = qh.endpoint_chars.load(Ordering::Relaxed);
        assert_eq!(chars & endpoint::HEAD_OF_LIST, 0);

        // Set as head of list
        qh.set_head_of_list();

        let chars = qh.endpoint_chars.load(Ordering::Relaxed);
        assert_ne!(chars & endpoint::HEAD_OF_LIST, 0);
    }

    #[test]
    fn test_qh_constants() {
        // Verify type constants
        assert_eq!(QueueHead::TYPE_ITD, 0 << 1);
        assert_eq!(QueueHead::TYPE_QH, 1 << 1);
        assert_eq!(QueueHead::TYPE_SITD, 2 << 1);
        assert_eq!(QueueHead::TYPE_FSTN, 3 << 1);
        assert_eq!(QueueHead::TERMINATE, 1);
    }

    #[test]
    fn test_qh_size_alignment() {
        // Verify structure size and alignment requirements
        assert_eq!(core::mem::size_of::<QueueHead>(), 96);
        assert!(core::mem::align_of::<QueueHead>() >= 32);

        // Verify proper alignment when allocated
        let qh = QueueHead::new();
        let addr = &qh as *const _ as usize;
        assert_eq!(addr & 0x1F, 0, "QueueHead not 32-byte aligned");
    }
}
