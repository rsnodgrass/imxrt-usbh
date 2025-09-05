//! Queue Head (qH) implementation for EHCI
//! 
//! Based on EHCI Specification Section 3.6

use core::sync::atomic::{AtomicU32, Ordering};
use crate::error::{Result, UsbError};

/// Endpoint characteristics field bits
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
    pub const TYPE_ITD: u32 = 0 << 1;
    pub const TYPE_QH: u32 = 1 << 1;
    pub const TYPE_SITD: u32 = 2 << 1;
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
        caps &= !(capabilities::SPLIT_COMPLETION_MASK_MASK << capabilities::SPLIT_COMPLETION_MASK_SHIFT);
        caps &= !(capabilities::INTERRUPT_SCHEDULE_MASK_MASK << capabilities::INTERRUPT_SCHEDULE_MASK_SHIFT);
        
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
    pub unsafe fn link_qtd(&self, qtd_addr: u32) {
        if qtd_addr & 0x1F != 0 {
            panic!("qTD address not 32-byte aligned");
        }
        
        // Set next qTD in overlay area
        self.next_qtd.store(qtd_addr, Ordering::Release);
        
        // Clear current qTD to trigger fetch of next qTD
        self.current_qtd.store(0, Ordering::Release);
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
        self.endpoint_chars.fetch_or(endpoint::HEAD_OF_LIST, Ordering::SeqCst);
    }
}

// Ensure size is correct per EHCI spec
const _: () = assert!(core::mem::size_of::<QueueHead>() == 96);
const _: () = assert!(core::mem::align_of::<QueueHead>() >= 32);