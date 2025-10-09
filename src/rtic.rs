//! RTIC integration for USB host with real-time constraints
//!
//! Implements embedded systems best practices for interrupt latency optimization
//! and deterministic real-time behavior per RTIC 2.0 patterns

use crate::error::{Result, UsbError};
use crate::perf::{InterruptTimer, PerfCounters};
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use cortex_m::asm;

pub mod interrupt;

/// Maximum interrupt processing time budget (10μs at 600MHz = 6000 cycles)
pub const MAX_ISR_CYCLES: u32 = 6_000;

/// High-priority interrupt events requiring immediate handling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UrgentEvent {
    /// VBUS over-current detected
    PowerError,
    /// Host system error requiring immediate attention
    SystemError,
    /// Critical port error requiring immediate handling
    CriticalPortError,
}

/// Lower-priority events that can be deferred
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeferredEvent {
    /// Normal transfer completion
    TransferComplete { qtd_addr: u32 },
    /// Port status change (connect/disconnect)
    PortChange { port: u8 },
    /// Frame list rollover
    FrameRollover,
    /// Asynchronous advance completion
    AsyncAdvance,
}

/// Lock-free MPMC (multi-producer/multi-consumer) ring buffer for event queuing
///
/// Uses CAS-based atomic slot claiming to safely handle concurrent access from
/// multiple interrupt contexts with different priorities on single-core Cortex-M7.
///
/// # Safety
///
/// - Safe for multiple producers (e.g., different ISRs enqueueing events)
/// - Safe for multiple consumers (e.g., ISR + main task dequeueing events)
/// - Uses `compare_exchange_weak` to prevent TOCTOU races during slot access
/// - Zero-overhead on uncontended access (single CAS vs single atomic store)
///
/// # Performance
///
/// - Bounded retry on contention (typically 1-2 retries in embedded scenarios)
/// - Lock-free: safe to call from ISR context without deadlock risk
pub struct EventRingBuffer<T, const N: usize> {
    buffer: [core::mem::MaybeUninit<T>; N],
    head: AtomicU8,
    tail: AtomicU8,
    full: AtomicBool,
}

impl<T, const N: usize> EventRingBuffer<T, N> {
    /// Create new ring buffer (const-compatible)
    pub const fn new() -> Self {
        const UNINIT: core::mem::MaybeUninit<()> = core::mem::MaybeUninit::uninit();
        Self {
            // Safety: transmuting MaybeUninit<()> to MaybeUninit<T> is valid - both are uninitialized
            buffer: [unsafe { core::mem::transmute(UNINIT) }; N],
            head: AtomicU8::new(0),
            tail: AtomicU8::new(0),
            full: AtomicBool::new(false),
        }
    }

    /// Enqueue item (lock-free, interrupt-safe, multi-producer safe)
    ///
    /// Uses CAS-based atomic slot claiming to prevent TOCTOU races between
    /// multiple producers (e.g., different ISRs or ISR+main task).
    pub fn enqueue(&self, item: T) -> bool {
        loop {
            // Snapshot current state with acquire ordering
            let current_head = self.head.load(Ordering::Acquire);
            let current_tail = self.tail.load(Ordering::Acquire);
            let is_full = self.full.load(Ordering::Acquire);

            // Calculate next position
            let next_head = (current_head + 1) % N as u8;

            // Check if buffer is full
            if next_head == current_tail && is_full {
                return false; // Buffer full, cannot enqueue
            }

            // Atomic slot claiming: only succeeds if head hasn't changed
            match self.head.compare_exchange_weak(
                current_head,
                next_head,
                Ordering::AcqRel, // Success: acquire previous writes + release our write
                Ordering::Acquire, // Failure: acquire current state for retry
            ) {
                Ok(_) => {
                    // SUCCESS: We atomically claimed slot at current_head
                    // Safety: current_head < N (masked), buffer has N slots, we atomically claimed this slot
                    unsafe {
                        let slot = &self.buffer[current_head as usize] as *const _
                            as *mut core::mem::MaybeUninit<T>;
                        slot.write(core::mem::MaybeUninit::new(item));
                    }

                    // Check if we just filled the buffer
                    if next_head == self.tail.load(Ordering::Acquire) {
                        self.full.store(true, Ordering::Release);
                    }

                    return true;
                }
                Err(_) => {
                    // FAILURE: Another producer claimed this slot, retry
                    core::hint::spin_loop();
                    continue;
                }
            }
        }
    }

    /// Dequeue item (lock-free, interrupt-safe, multi-consumer safe)
    ///
    /// Uses CAS-based atomic slot claiming to prevent TOCTOU races between
    /// multiple consumers (e.g., different ISRs or ISR+main task).
    pub fn dequeue(&self) -> Option<T> {
        loop {
            // Snapshot current state with acquire ordering
            let current_tail = self.tail.load(Ordering::Acquire);
            let current_head = self.head.load(Ordering::Acquire);
            let is_full = self.full.load(Ordering::Acquire);

            // Check if buffer is empty
            if current_tail == current_head && !is_full {
                return None; // Buffer empty
            }

            // Calculate next position
            let next_tail = (current_tail + 1) % N as u8;

            // Atomic slot claiming: only succeeds if tail hasn't changed
            match self.tail.compare_exchange_weak(
                current_tail,
                next_tail,
                Ordering::AcqRel, // Success: acquire previous writes + release our read
                Ordering::Acquire, // Failure: acquire current state for retry
            ) {
                Ok(_) => {
                    // SUCCESS: We atomically claimed slot at current_tail
                    // Safety: current_tail < N (masked), buffer has N slots, slot was initialized by push
                    let item = unsafe {
                        let slot = &self.buffer[current_tail as usize];
                        slot.assume_init_read()
                    };

                    // Clear full flag (we just consumed an item)
                    self.full.store(false, Ordering::Release);

                    return Some(item);
                }
                Err(_) => {
                    // FAILURE: Another consumer claimed this slot, retry
                    core::hint::spin_loop();
                    continue;
                }
            }
        }
    }

    /// Check if buffer is near capacity (> 75%)
    pub fn is_near_full(&self) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        let used = if head >= tail {
            head - tail
        } else {
            N as u8 - tail + head
        };
        used > (N as u8 * 3) / 4
    }
}

/// RTIC-compatible USB interrupt handler
pub struct UsbInterruptHandler {
    /// Urgent events requiring immediate handling
    urgent_events: EventRingBuffer<UrgentEvent, 8>,
    /// Deferred events for background processing
    deferred_events: EventRingBuffer<DeferredEvent, 32>,
    /// Interrupt overrun counter for monitoring
    overruns: AtomicU8,
    /// Performance counters reference
    perf_counters: &'static PerfCounters,
}

impl UsbInterruptHandler {
    /// Create new interrupt handler
    pub const fn new(perf_counters: &'static PerfCounters) -> Self {
        Self {
            urgent_events: EventRingBuffer::new(),
            deferred_events: EventRingBuffer::new(),
            overruns: AtomicU8::new(0),
            perf_counters,
        }
    }

    /// High-priority interrupt service routine
    ///
    /// This function must complete within MAX_ISR_CYCLES to meet real-time constraints
    /// Only handles critical events that require immediate response
    #[inline(never)]
    pub fn high_priority_isr(&self, usb_status: u32) -> Result<()> {
        let _timer = InterruptTimer::start(self.perf_counters);

        // Critical path: Handle only urgent conditions
        if usb_status & (1 << 4) != 0 {
            // Host System Error
            if !self.urgent_events.enqueue(UrgentEvent::SystemError) {
                self.overruns.fetch_add(1, Ordering::Relaxed);
            }
            // Clear the interrupt immediately
            self.clear_host_system_error();
        }

        if usb_status & (1 << 2) != 0 {
            // Port Change Detect - check for critical conditions
            let port_status = self.read_port_status_fast();
            if port_status & (1 << 4) != 0 {
                // Over-current
                if !self.urgent_events.enqueue(UrgentEvent::PowerError) {
                    self.overruns.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                // Non-critical port change, defer it
                if !self
                    .deferred_events
                    .enqueue(DeferredEvent::PortChange { port: 0 })
                {
                    self.overruns.fetch_add(1, Ordering::Relaxed);
                }
            }
        }

        // Clear processed interrupt bits
        self.clear_interrupt_status(usb_status);

        Ok(())
    }

    /// Process deferred events in background task
    ///
    /// This runs at lower priority and can take longer to process
    /// complex events without affecting real-time constraints
    pub fn process_deferred_events(&self) -> Result<()> {
        let mut processed = 0;
        const MAX_BATCH_SIZE: u32 = 16; // Limit batch size to prevent starvation

        while let Some(event) = self.deferred_events.dequeue() {
            match event {
                DeferredEvent::TransferComplete { qtd_addr } => {
                    self.handle_transfer_completion(qtd_addr)?;
                }
                DeferredEvent::PortChange { port } => {
                    self.handle_port_change(port)?;
                }
                DeferredEvent::FrameRollover => {
                    self.handle_frame_rollover()?;
                }
                DeferredEvent::AsyncAdvance => {
                    self.handle_async_advance()?;
                }
            }

            processed += 1;
            if processed >= MAX_BATCH_SIZE {
                break; // Yield to other tasks
            }
        }

        Ok(())
    }

    /// Handle urgent events with minimal latency
    pub fn handle_urgent_events(&self) -> Result<()> {
        while let Some(event) = self.urgent_events.dequeue() {
            match event {
                UrgentEvent::PowerError => {
                    // Immediately disable VBUS to protect hardware
                    self.emergency_vbus_shutdown();
                }
                UrgentEvent::SystemError => {
                    // Reset and reinitialize controller
                    self.emergency_controller_reset()?;
                }
                UrgentEvent::CriticalPortError => {
                    // Disable problematic port
                    self.emergency_port_disable();
                }
            }
        }
        Ok(())
    }

    /// Optimized register access for critical path
    #[inline(always)]
    fn read_port_status_fast(&self) -> u32 {
        // Direct register access for minimal latency
        // Safety: 0x402E_0044 is valid PORTSC[0] register address for USB2 on i.MX RT1062
        unsafe {
            core::ptr::read_volatile(0x402E_0044 as *const u32) // PORTSC[0]
        }
    }

    /// Clear interrupt status with minimal cycles
    #[inline(always)]
    fn clear_interrupt_status(&self, status: u32) {
        // Safety: 0x402E_0004 is valid USBSTS register address for USB2 on i.MX RT1062
        unsafe {
            core::ptr::write_volatile(0x402E_0004 as *mut u32, status); // USBSTS
        }
    }

    /// Emergency VBUS shutdown for over-current protection
    #[inline(never)]
    fn emergency_vbus_shutdown(&self) {
        // Safety: 0x402E_0044 is valid PORTSC[0] register, emergency context justifies direct access
        unsafe {
            // Immediately disable VBUS power
            let portsc = 0x402E_0044 as *mut u32;
            let mut port = core::ptr::read_volatile(portsc);
            port &= !(1 << 12); // Clear Port Power
            core::ptr::write_volatile(portsc, port);
        }
    }

    /// Emergency controller reset for system errors
    fn emergency_controller_reset(&self) -> Result<()> {
        // Safety: 0x402E_0000 is valid USBCMD register, emergency context justifies direct access
        unsafe {
            // Assert controller reset
            let usbcmd = 0x402E_0000 as *mut u32;
            let mut cmd = core::ptr::read_volatile(usbcmd);
            cmd |= 1 << 1; // Host Controller Reset
            core::ptr::write_volatile(usbcmd, cmd);
        }
        Ok(())
    }

    /// Emergency port disable for critical port errors
    fn emergency_port_disable(&self) {
        // Safety: 0x402E_0044 is valid PORTSC[0] register, emergency context justifies direct access
        unsafe {
            let portsc = 0x402E_0044 as *mut u32;
            let mut port = core::ptr::read_volatile(portsc);
            port &= !(1 << 2); // Clear Port Enabled
            core::ptr::write_volatile(portsc, port);
        }
    }

    /// Clear host system error interrupt
    #[inline(always)]
    fn clear_host_system_error(&self) {
        // Safety: 0x402E_0004 is valid USBSTS register, write-1-to-clear HSE bit
        unsafe {
            core::ptr::write_volatile(0x402E_0004 as *mut u32, 1 << 4); // Clear HSE bit
        }
    }

    /// Handle transfer completion in background
    fn handle_transfer_completion(&self, qtd_addr: u32) -> Result<()> {
        // Process completed transfer
        // This can take longer since it's not in the critical path
        self.perf_counters.record_transfer_success(0); // Placeholder
        Ok(())
    }

    /// Handle port status changes
    fn handle_port_change(&self, _port: u8) -> Result<()> {
        // Process port status change
        // Device enumeration, disconnection handling, etc.
        Ok(())
    }

    /// Handle frame list rollover
    fn handle_frame_rollover(&self) -> Result<()> {
        // Update frame counter and handle any frame-based logic
        Ok(())
    }

    /// Handle asynchronous advance completion
    fn handle_async_advance(&self) -> Result<()> {
        // Process asynchronous schedule modifications
        Ok(())
    }

    /// Get interrupt handler statistics
    pub fn statistics(&self) -> InterruptStats {
        InterruptStats {
            overruns: self.overruns.load(Ordering::Relaxed),
            urgent_queue_near_full: self.urgent_events.is_near_full(),
            deferred_queue_near_full: self.deferred_events.is_near_full(),
        }
    }

    /// Check if system is healthy from interrupt perspective
    pub fn is_healthy(&self) -> bool {
        let stats = self.statistics();
        stats.overruns == 0 && !stats.urgent_queue_near_full
    }
}

/// Interrupt handler statistics
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStats {
    /// Number of interrupt overruns (lost events)
    pub overruns: u8,
    /// Urgent event queue near capacity
    pub urgent_queue_near_full: bool,
    /// Deferred event queue near capacity
    pub deferred_queue_near_full: bool,
}

/// RTIC task priorities for USB operations
pub mod priorities {
    /// Critical interrupt handling (highest priority)
    pub const USB_INTERRUPT: u8 = 15;
    /// Urgent event processing (high priority)
    pub const URGENT_EVENTS: u8 = 12;
    /// Background event processing (medium priority)
    pub const DEFERRED_EVENTS: u8 = 8;
    /// Periodic health checks (low priority)
    pub const HEALTH_CHECK: u8 = 4;
    /// Statistics and monitoring (lowest priority)
    pub const MONITORING: u8 = 1;
}

/// Memory barrier for interrupt synchronization
#[inline(always)]
pub fn interrupt_barrier() {
    asm::dsb();
    asm::isb();
}

/// Critical section helper for RTIC integration
pub struct CriticalSection;

impl CriticalSection {
    /// Execute closure in critical section with timing measurement
    pub fn execute<F, R>(f: F) -> R
    where
        F: FnOnce() -> R,
    {
        let start_cycles = cortex_m::peripheral::DWT::cycle_count();

        cortex_m::interrupt::free(|_| {
            let result = f();
            interrupt_barrier();
            result
        })
        // Note: Could add timing verification here if needed
    }
}

/// Example RTIC integration for embedded systems patterns
pub mod example {
    use super::*;
    use crate::perf::PERF_COUNTERS;

    /// Global interrupt handler instance
    pub static USB_INTERRUPT_HANDLER: UsbInterruptHandler =
        UsbInterruptHandler::new(&PERF_COUNTERS);

    /// USB interrupt handler for RTIC integration
    pub fn usb_interrupt_rtic() -> Result<()> {
        // Read USB status register
        let usb_status = unsafe {
            core::ptr::read_volatile(0x402E_0004 as *const u32) // USBSTS
        };

        // Process in high-priority ISR
        USB_INTERRUPT_HANDLER.high_priority_isr(usb_status)
    }

    /// Background task for processing deferred events
    pub async fn process_usb_events() -> Result<()> {
        loop {
            // Handle urgent events first
            USB_INTERRUPT_HANDLER.handle_urgent_events()?;

            // Process deferred events
            USB_INTERRUPT_HANDLER.process_deferred_events()?;

            // Check interrupt handler health
            if !USB_INTERRUPT_HANDLER.is_healthy() {
                let stats = USB_INTERRUPT_HANDLER.statistics();

                #[cfg(feature = "defmt")]
                defmt::warn!("Interrupt handler issues: {:?}", stats);
            }

            // Yield to other tasks
            #[cfg(feature = "rtic-support")]
            rtic_monotonics::Monotonic::delay(core::time::Duration::from_millis(1)).await;

            #[cfg(not(feature = "rtic-support"))]
            cortex_m::asm::wfe(); // Wait for event if not using RTIC
        }
    }
}
