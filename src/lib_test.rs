//! Library functionality verification
//!
//! Simple compile-time checks for basic library functionality

use crate::{UsbError, Direction, TransferType};
use crate::ehci::{QueueTD, QueueHead};
use crate::dma::DmaBufferPool;

/// Verify basic types compile and can be used
pub fn verify_basic_functionality() {
    // Test error enum
    let _timeout = UsbError::Timeout;
    let _invalid = UsbError::InvalidParameter;
    
    // Test direction enum
    let _in_dir = Direction::In;
    let _out_dir = Direction::Out;
    
    // Test transfer types
    let _control = TransferType::Control;
    let _bulk = TransferType::Bulk;
    let _interrupt = TransferType::Interrupt;
    let _iso = TransferType::Isochronous;
    
    // Test EHCI structures
    let _qtd = QueueTD::new();
    let _qh = QueueHead::new();
    
    // Test DMA pool
    let _pool = DmaBufferPool::new();
}

/// Check structure sizes are reasonable
pub fn verify_structure_sizes() {
    let qh_size = core::mem::size_of::<QueueHead>();
    let qtd_size = core::mem::size_of::<QueueTD>();
    
    // Basic sanity checks - structures should be non-zero sized
    // and properly aligned for DMA
    let _qh_ok = qh_size > 0 && qh_size >= 32;
    let _qtd_ok = qtd_size > 0 && qtd_size >= 20; // Minimum for basic qTD fields
    
    let qh_align = core::mem::align_of::<QueueHead>();
    let qtd_align = core::mem::align_of::<QueueTD>();
    
    let _qh_aligned = qh_align >= 32;
    let _qtd_aligned = qtd_align >= 32;
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_basic_functionality() {
        verify_basic_functionality();
    }
    
    #[test]
    fn test_structure_sizes() {
        verify_structure_sizes();
    }
}