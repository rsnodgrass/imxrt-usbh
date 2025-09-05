//! Queue Transfer Descriptor (qTD) implementation for EHCI
//! 
//! Based on EHCI Specification Section 3.5

use core::sync::atomic::{AtomicU32, Ordering};
use crate::error::{Result, UsbError};
use crate::dma::is_dma_aligned;

/// qTD token field bit definitions
pub mod token {
    pub const STATUS_ACTIVE: u32 = 1 << 7;
    pub const STATUS_HALTED: u32 = 1 << 6;
    pub const STATUS_DATA_BUFFER_ERROR: u32 = 1 << 5;
    pub const STATUS_BABBLE: u32 = 1 << 4;
    pub const STATUS_TRANSACTION_ERROR: u32 = 1 << 3;
    pub const STATUS_MISSED_MICROFRAME: u32 = 1 << 2;
    pub const STATUS_SPLIT_STATE: u32 = 1 << 1;
    pub const STATUS_PING_STATE: u32 = 1 << 0;
    
    pub const ERROR_COUNTER_SHIFT: u32 = 10;
    pub const ERROR_COUNTER_MASK: u32 = 0x3;
    
    pub const CURRENT_PAGE_SHIFT: u32 = 12;
    pub const CURRENT_PAGE_MASK: u32 = 0x7;
    
    pub const INTERRUPT_ON_COMPLETE: u32 = 1 << 15;
    
    pub const TOTAL_BYTES_SHIFT: u32 = 16;
    pub const TOTAL_BYTES_MASK: u32 = 0x7FFF;
    
    pub const DATA_TOGGLE: u32 = 1 << 31;
    
    pub const PID_OUT: u32 = 0x0 << 8;
    pub const PID_IN: u32 = 0x1 << 8;
    pub const PID_SETUP: u32 = 0x2 << 8;
}

/// Queue Transfer Descriptor (qTD)
/// 
/// EHCI Specification Section 3.5
/// Must be 32-byte aligned for DMA
#[repr(C, align(32))]
pub struct QueueTD {
    /// Next qTD pointer (bits 31:5 valid, bit 0 = terminate)
    pub next_qtd: AtomicU32,
    
    /// Alternate next qTD pointer (for short packets)
    pub alt_next_qtd: AtomicU32,
    
    /// Token field containing status, PID, and transfer length
    pub token: AtomicU32,
    
    /// Buffer pointer pages (up to 5 pages, 4KB each = 20KB max)
    pub buffer_pointers: [AtomicU32; 5],
    
    /// Extended buffer pointers for 64-bit addressing (unused on RT1062)
    pub ext_buffer_pointers: [AtomicU32; 5],
    
    // Software-only fields (not accessed by hardware)
    /// Link to owning transfer for completion handling
    pub transfer_id: AtomicU32,
    
    /// Reserved for alignment
    _reserved: [u32; 3],
}

impl QueueTD {
    /// qTD terminator bit
    pub const TERMINATE: u32 = 1;
    
    /// Maximum transfer size per qTD (20KB)
    pub const MAX_TRANSFER_SIZE: usize = 20 * 1024;
    
    /// Create a new inactive qTD
    pub const fn new() -> Self {
        Self {
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
            transfer_id: AtomicU32::new(0),
            _reserved: [0; 3],
        }
    }
    
    /// Initialize qTD for a transfer
    /// 
    /// # Safety
    /// 
    /// Buffer must remain valid and unchanged for the duration of the transfer.
    /// Buffer address must be DMA-accessible.
    pub unsafe fn init_transfer(
        &self,
        pid: u32,
        data_toggle: bool,
        buffer: Option<(*const u8, usize)>,
        interrupt_on_complete: bool,
    ) -> Result<()> {
        // Check buffer alignment if provided
        if let Some((addr, len)) = buffer {
            if !is_dma_aligned(addr as usize) {
                return Err(UsbError::InvalidParameter);
            }
            if len > Self::MAX_TRANSFER_SIZE {
                return Err(UsbError::BufferOverflow);
            }
            
            // Set up buffer pointers (one pointer per 4KB page)
            let mut remaining = len;
            let mut current_addr = addr as u32;
            
            for i in 0..5 {
                if remaining == 0 {
                    self.buffer_pointers[i].store(0, Ordering::Release);
                } else {
                    self.buffer_pointers[i].store(current_addr, Ordering::Release);
                    
                    // Calculate bytes in this page (up to 4KB)
                    let page_offset = current_addr & 0xFFF;
                    let bytes_in_page = core::cmp::min(0x1000 - page_offset, remaining as u32);
                    
                    current_addr += bytes_in_page;
                    remaining = remaining.saturating_sub(bytes_in_page as usize);
                }
            }
        }
        
        // Build token field
        let mut token = token::STATUS_ACTIVE;
        token |= pid;
        token |= 3 << token::ERROR_COUNTER_SHIFT; // 3 retries
        
        if let Some((_, len)) = buffer {
            token |= (len as u32) << token::TOTAL_BYTES_SHIFT;
        }
        
        if data_toggle {
            token |= token::DATA_TOGGLE;
        }
        
        if interrupt_on_complete {
            token |= token::INTERRUPT_ON_COMPLETE;
        }
        
        self.token.store(token, Ordering::Release);
        
        Ok(())
    }
    
    /// Check if qTD is still active
    pub fn is_active(&self) -> bool {
        let token = self.token.load(Ordering::Acquire);
        (token & token::STATUS_ACTIVE) != 0
    }
    
    /// Check if qTD encountered an error
    pub fn has_error(&self) -> Option<UsbError> {
        let token = self.token.load(Ordering::Acquire);
        
        if (token & token::STATUS_HALTED) != 0 {
            if (token & token::STATUS_BABBLE) != 0 {
                return Some(UsbError::Babble);
            }
            if (token & token::STATUS_DATA_BUFFER_ERROR) != 0 {
                return Some(UsbError::BufferOverflow);
            }
            if (token & token::STATUS_TRANSACTION_ERROR) != 0 {
                return Some(UsbError::TransactionError);
            }
            if (token & token::STATUS_MISSED_MICROFRAME) != 0 {
                return Some(UsbError::MissedMicroframe);
            }
            return Some(UsbError::Stall);
        }
        
        None
    }
    
    /// Get bytes transferred
    pub fn bytes_transferred(&self) -> usize {
        let token = self.token.load(Ordering::Acquire);
        let total_bytes = (token >> token::TOTAL_BYTES_SHIFT) & token::TOTAL_BYTES_MASK;
        total_bytes as usize
    }
    
    /// Reset qTD for reuse
    pub fn reset(&self) {
        self.next_qtd.store(Self::TERMINATE, Ordering::Release);
        self.alt_next_qtd.store(Self::TERMINATE, Ordering::Release);
        self.token.store(0, Ordering::Release);
        
        for i in 0..5 {
            self.buffer_pointers[i].store(0, Ordering::Release);
        }
    }
}

// Ensure size is correct per EHCI spec
// QueueTD structure size: 12 AtomicU32 fields (12*4=48) + 4 u32 fields (4*4=16) = 64 bytes
// With 32-byte alignment, the actual size may be larger
const _: () = assert!(core::mem::size_of::<QueueTD>() >= 64);
const _: () = assert!(core::mem::align_of::<QueueTD>() == 32);
const _: () = assert!(core::mem::align_of::<QueueTD>() >= 32);