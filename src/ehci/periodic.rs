//! Periodic schedule for interrupt and isochronous transfers
//!
//! EHCI periodic schedule uses a 1024-entry frame list for time-sensitive transfers.
//! Each entry can point to Queue Heads (for interrupt) or iTD/siTD (for isochronous).

use crate::ehci::{read_register_at, set_bits_at, clear_bits_at, write_register_at};
use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicU32, Ordering};

/// Frame list size (1024 entries per EHCI spec)
pub const FRAME_LIST_SIZE: usize = 1024;

/// Frame list entry type bits (bits 2:1)
#[allow(dead_code)]
const TYPE_ITD: u32 = 0 << 1;
const TYPE_QH: u32 = 1 << 1;
#[allow(dead_code)]
const TYPE_SITD: u32 = 2 << 1;
#[allow(dead_code)]
const TYPE_FSTN: u32 = 3 << 1;

/// Terminate bit
const TERMINATE: u32 = 1;

/// Periodic frame list for interrupt and isochronous transfers
///
/// Must be 4096-byte aligned per EHCI specification
#[repr(C, align(4096))]
pub struct PeriodicFrameList {
    /// 1024 frame list entries (one per microframe)
    entries: [AtomicU32; FRAME_LIST_SIZE],
}

impl PeriodicFrameList {
    /// Create new frame list with all entries terminated
    pub const fn new() -> Self {
        const TERMINATED: AtomicU32 = AtomicU32::new(TERMINATE);
        Self {
            entries: [TERMINATED; FRAME_LIST_SIZE],
        }
    }

    /// Get physical address of frame list (for PERIODICLISTBASE register)
    pub fn base_address(&self) -> u32 {
        self.entries.as_ptr() as u32
    }

    /// Link a QH to a specific frame
    ///
    /// # Safety
    ///
    /// QH must remain valid and properly aligned while linked
    pub unsafe fn link_qh(&self, frame_index: usize, qh_addr: u32) -> Result<()> {
        if frame_index >= FRAME_LIST_SIZE {
            return Err(UsbError::InvalidParameter);
        }

        if qh_addr & 0x1F != 0 {
            return Err(UsbError::InvalidParameter);
        }

        // Read current entry
        let current = self.entries[frame_index].load(Ordering::Acquire);

        // Link QH, preserving existing chain if needed
        let new_entry = if current & TERMINATE != 0 {
            // Empty slot, directly link QH
            qh_addr | TYPE_QH
        } else {
            // Existing entry - QH should point to it via horizontal_link
            // For simplicity, just replace (production should chain)
            qh_addr | TYPE_QH
        };

        self.entries[frame_index].store(new_entry, Ordering::Release);

        Ok(())
    }

    /// Unlink a QH from a specific frame
    pub fn unlink_qh(&self, frame_index: usize) -> Result<()> {
        if frame_index >= FRAME_LIST_SIZE {
            return Err(UsbError::InvalidParameter);
        }

        // Simplified: just terminate the entry
        // Production should traverse the chain and remove specific QH
        self.entries[frame_index].store(TERMINATE, Ordering::Release);

        Ok(())
    }

    /// Link QH to multiple frames (for periodic interrupt transfers)
    ///
    /// # Arguments
    ///
    /// * `interval` - Polling interval in frames (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024)
    /// * `qh_addr` - Physical address of Queue Head
    ///
    /// # Safety
    ///
    /// QH must remain valid while linked
    pub unsafe fn link_qh_periodic(&self, interval: u32, qh_addr: u32) -> Result<()> {
        if !interval.is_power_of_two() || interval > 1024 {
            return Err(UsbError::InvalidParameter);
        }

        // Link QH to every Nth frame based on interval
        let mut frame = 0;
        while frame < FRAME_LIST_SIZE {
            unsafe {
                self.link_qh(frame, qh_addr)?;
            }
            frame += interval as usize;
        }

        Ok(())
    }

    /// Get current frame index from FRINDEX register
    ///
    /// # Arguments
    ///
    /// * `op_base` - EHCI operational register base address
    pub fn current_frame_index(op_base: usize) -> u32 {
        let frindex_ptr = (op_base + 0x0C) as *const u32;
        let frindex = unsafe { read_register_at(frindex_ptr) };
        (frindex >> 3) & 0x3FF // Bits 12:3, mask to 1024 frames
    }
}

/// Interrupt transfer scheduler
///
/// Manages QH placement in periodic schedule for optimal bandwidth
pub struct InterruptScheduler {
    /// Tracks QH count per frame for load balancing
    frame_load: [AtomicU32; FRAME_LIST_SIZE],
}

impl InterruptScheduler {
    /// Create new scheduler
    pub const fn new() -> Self {
        const ZERO: AtomicU32 = AtomicU32::new(0);
        Self {
            frame_load: [ZERO; FRAME_LIST_SIZE],
        }
    }

    /// Find optimal frame offset for given interval
    ///
    /// Balances load across frames to avoid bandwidth bottlenecks
    pub fn find_best_offset(&self, interval: u32) -> usize {
        if !interval.is_power_of_two() || interval > 1024 {
            return 0;
        }

        let interval_usize = interval as usize;
        let mut min_load = u32::MAX;
        let mut best_offset = 0;

        // Try each possible offset within the interval
        for offset in 0..interval_usize {
            let mut total_load = 0;
            let mut frame = offset;

            // Sum load for all frames this QH would occupy
            while frame < FRAME_LIST_SIZE {
                total_load += self.frame_load[frame].load(Ordering::Relaxed);
                frame += interval_usize;
            }

            if total_load < min_load {
                min_load = total_load;
                best_offset = offset;
            }
        }

        best_offset
    }

    /// Record QH scheduling in load tracker
    pub fn schedule_qh(&self, interval: u32, offset: usize, max_packet_size: u16) {
        let interval_usize = interval as usize;
        let mut frame = offset;

        // Estimate bandwidth usage (very simplified)
        let bandwidth = (max_packet_size as u32 + 100) / 100; // Rough estimate

        while frame < FRAME_LIST_SIZE {
            self.frame_load[frame].fetch_add(bandwidth, Ordering::Relaxed);
            frame += interval_usize;
        }
    }

    /// Remove QH from load tracker
    pub fn unschedule_qh(&self, interval: u32, offset: usize, max_packet_size: u16) {
        let interval_usize = interval as usize;
        let mut frame = offset;

        let bandwidth = (max_packet_size as u32 + 100) / 100;

        while frame < FRAME_LIST_SIZE {
            self.frame_load[frame].fetch_sub(bandwidth, Ordering::Relaxed);
            frame += interval_usize;
        }
    }

    /// Get total bandwidth used in a frame (rough percentage)
    pub fn frame_utilization(&self, frame_index: usize) -> u32 {
        if frame_index >= FRAME_LIST_SIZE {
            return 0;
        }

        // EHCI has ~80% usable bandwidth per frame
        let load = self.frame_load[frame_index].load(Ordering::Relaxed);
        (load * 100) / 80 // Return percentage
    }
}

/// Static periodic frame list
static mut PERIODIC_FRAME_LIST: PeriodicFrameList = PeriodicFrameList::new();

/// Static interrupt scheduler
static INTERRUPT_SCHEDULER: InterruptScheduler = InterruptScheduler::new();

/// Initialize periodic schedule
///
/// # Safety
///
/// Must be called once during USB host initialization
pub unsafe fn init_periodic_schedule(op_base: usize) -> Result<()> {
    let frame_list = unsafe { &*core::ptr::addr_of!(PERIODIC_FRAME_LIST) };

    // Set PERIODICLISTBASE register (offset 0x14)
    let base_addr = frame_list.base_address();

    // Ensure 4096-byte alignment
    if base_addr & 0xFFF != 0 {
        return Err(UsbError::InvalidParameter);
    }

    unsafe {
        write_register_at((op_base + 0x14) as *mut u32, base_addr);
    }

    Ok(())
}

/// Enable periodic schedule
///
/// # Safety
///
/// Periodic schedule must be initialized first
pub unsafe fn enable_periodic_schedule(op_base: usize) -> Result<()> {
    let usbcmd_ptr = op_base as *mut u32;

    unsafe {
        set_bits_at(usbcmd_ptr, 1 << 4); // Periodic Schedule Enable bit
    }

    // Wait for periodic schedule to start
    let usbsts_ptr = (op_base + 0x04) as *const u32;
    let timeout = super::RegisterTimeout::new_ms(10);

    timeout.wait_for(|| {
        let usbsts = unsafe { read_register_at(usbsts_ptr) };
        (usbsts & (1 << 14)) != 0 // Periodic Schedule Status bit
    })?;

    Ok(())
}

/// Disable periodic schedule
pub fn disable_periodic_schedule(op_base: usize) -> Result<()> {
    let usbcmd_ptr = op_base as *mut u32;

    unsafe {
        clear_bits_at(usbcmd_ptr, 1 << 4); // Clear Periodic Schedule Enable
    }

    // Wait for schedule to stop
    let usbsts_ptr = (op_base + 0x04) as *const u32;
    let timeout = super::RegisterTimeout::new_ms(10);

    timeout.wait_for(|| {
        let usbsts = unsafe { read_register_at(usbsts_ptr) };
        (usbsts & (1 << 14)) == 0 // Periodic Schedule Status cleared
    })?;

    Ok(())
}

/// Get access to global periodic frame list
///
/// # Safety
///
/// Caller must ensure exclusive access if modifying
pub unsafe fn get_frame_list() -> &'static PeriodicFrameList {
    unsafe { &*core::ptr::addr_of!(PERIODIC_FRAME_LIST) }
}

/// Get access to global interrupt scheduler
pub fn get_scheduler() -> &'static InterruptScheduler {
    &INTERRUPT_SCHEDULER
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_list_creation() {
        let frame_list = PeriodicFrameList::new();

        // All entries should be terminated
        for i in 0..FRAME_LIST_SIZE {
            assert_eq!(frame_list.entries[i].load(Ordering::Relaxed), TERMINATE);
        }
    }

    #[test]
    fn test_frame_list_alignment() {
        let frame_list = PeriodicFrameList::new();
        let addr = &frame_list as *const _ as usize;

        // Must be 4096-byte aligned
        assert_eq!(addr & 0xFFF, 0, "Frame list not 4096-byte aligned");
    }

    #[test]
    fn test_scheduler_offset_selection() {
        let scheduler = InterruptScheduler::new();

        // Test various intervals
        let offset_1ms = scheduler.find_best_offset(1);
        assert!(offset_1ms < 1);

        let offset_8ms = scheduler.find_best_offset(8);
        assert!(offset_8ms < 8);

        let offset_16ms = scheduler.find_best_offset(16);
        assert!(offset_16ms < 16);
    }

    #[test]
    fn test_scheduler_load_tracking() {
        let scheduler = InterruptScheduler::new();

        // Schedule a QH with 8ms interval
        scheduler.schedule_qh(8, 0, 64);

        // Check that frames have non-zero load
        assert!(scheduler.frame_utilization(0) > 0);
        assert!(scheduler.frame_utilization(8) > 0);
        assert!(scheduler.frame_utilization(16) > 0);

        // Unschedule
        scheduler.unschedule_qh(8, 0, 64);

        // Load should return to zero
        assert_eq!(scheduler.frame_utilization(0), 0);
    }
}
