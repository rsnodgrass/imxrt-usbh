//! RTIC interrupt handler implementation for USB host
//! 
//! Provides real-time USB interrupt handling with RTIC integration

use crate::error::{Result, UsbError};
use crate::ehci::{UsbSts, PortSc};
use crate::perf::{PerfCounters, InterruptTimer};
use crate::rtic::{UsbInterruptHandler, DeferredEvent, UrgentEvent};
use crate::transfer::control::ControlTransferManager;
use crate::dma::DescriptorAllocator;
use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};

/// USB interrupt context for RTIC
pub struct UsbInterruptContext {
    /// Interrupt handler
    handler: &'static UsbInterruptHandler,
    /// Performance counters
    perf: &'static PerfCounters,
    /// Base address of USB controller
    usb_base: usize,
    /// Interrupt pending flag
    interrupt_pending: AtomicBool,
    /// Last interrupt status
    last_status: AtomicU32,
}

impl UsbInterruptContext {
    /// Create new interrupt context
    pub const fn new(
        handler: &'static UsbInterruptHandler,
        perf: &'static PerfCounters,
        usb_base: usize,
    ) -> Self {
        Self {
            handler,
            perf,
            usb_base,
            interrupt_pending: AtomicBool::new(false),
            last_status: AtomicU32::new(0),
        }
    }
    
    /// High-priority USB interrupt handler (RTIC task)
    /// 
    /// This runs at highest priority for minimal latency
    #[inline(never)]
    pub fn usb_interrupt(&self) -> Result<()> {
        // Start timing for latency measurement
        let _timer = InterruptTimer::start(self.perf);
        
        // Create deadline monitor (10μs max)
        let deadline = crate::safety::DeadlineMonitor::new(10);
        
        // Check stack safety
        if !crate::safety::check_safety() {
            return Err(UsbError::InvalidState);
        }
        
        // Read and clear interrupt status atomically
        let status = self.read_and_clear_status();
        
        // Store status for deferred processing
        self.last_status.store(status, Ordering::Release);
        
        // Process critical interrupts immediately
        self.handler.high_priority_isr(status)?;
        
        // Signal that deferred processing is needed
        if self.has_deferred_events(status) {
            self.interrupt_pending.store(true, Ordering::Release);
        }
        
        // Check deadline was met
        if !deadline.check_deadline() {
            self.perf.record_transfer_error();
        }
        
        Ok(())
    }
    
    /// Read and clear USB interrupt status
    #[inline(always)]
    fn read_and_clear_status(&self) -> u32 {
        unsafe {
            let usbsts_addr = (self.usb_base + 0x144) as *mut u32; // USBSTS offset
            let status = core::ptr::read_volatile(usbsts_addr);
            
            // Clear the interrupt bits by writing them back
            core::ptr::write_volatile(usbsts_addr, status);
            
            // Memory barrier to ensure completion
            cortex_m::asm::dsb();
            
            status
        }
    }
    
    /// Check if status has deferred events
    #[inline(always)]
    fn has_deferred_events(&self, status: u32) -> bool {
        // These can be processed in background:
        // - USB INT (bit 0) - Transfer complete
        // - USB Error INT (bit 1) - Non-critical errors
        // - Frame List Rollover (bit 3)
        // - Async Advance (bit 5)
        status & 0x2B != 0
    }
    
    /// Process deferred events (RTIC task at lower priority)
    pub fn process_deferred(&self) -> Result<()> {
        // Clear pending flag
        self.interrupt_pending.store(false, Ordering::Release);
        
        // Get saved status
        let status = self.last_status.load(Ordering::Acquire);
        
        // Process urgent events first
        self.handler.handle_urgent_events()?;
        
        // Convert status bits to deferred events
        if status & (1 << 0) != 0 {
            // USB INT - transfer complete
            self.queue_transfer_complete()?;
        }
        
        if status & (1 << 1) != 0 {
            // USB Error INT
            self.handle_usb_error(status)?;
        }
        
        if status & (1 << 2) != 0 {
            // Port Change Detect
            self.queue_port_change()?;
        }
        
        if status & (1 << 3) != 0 {
            // Frame List Rollover
            self.handler.deferred_events.enqueue(DeferredEvent::FrameRollover);
        }
        
        if status & (1 << 5) != 0 {
            // Async Advance
            self.handler.deferred_events.enqueue(DeferredEvent::AsyncAdvance);
        }
        
        // Process all queued deferred events
        self.handler.process_deferred_events()?;
        
        Ok(())
    }
    
    /// Queue transfer complete event
    fn queue_transfer_complete(&self) -> Result<()> {
        // In real implementation, would scan async/periodic lists
        // For now, queue a placeholder event
        self.handler.deferred_events.enqueue(
            DeferredEvent::TransferComplete { qtd_addr: 0 }
        );
        Ok(())
    }
    
    /// Handle USB error interrupt
    fn handle_usb_error(&self, status: u32) -> Result<()> {
        // Determine error type from status
        if status & (1 << 4) != 0 {
            // Host System Error - this is urgent
            self.handler.urgent_events.enqueue(UrgentEvent::HostSystemError);
        }
        
        // Log error for debugging
        self.perf.record_transfer_error();
        
        Ok(())
    }
    
    /// Queue port change event
    fn queue_port_change(&self) -> Result<()> {
        // Check each port for changes
        for port in 0..8 {
            if self.port_has_change(port) {
                self.handler.deferred_events.enqueue(
                    DeferredEvent::PortChange { port }
                );
            }
        }
        Ok(())
    }
    
    /// Check if port has status change
    fn port_has_change(&self, port: u8) -> bool {
        unsafe {
            let portsc_addr = (self.usb_base + 0x184 + (port as usize * 4)) as *const u32;
            let portsc = core::ptr::read_volatile(portsc_addr);
            
            // Check for any change bits (Connect, Enable, Over-current)
            (portsc & 0x0E) != 0
        }
    }
    
    /// Check if interrupt is pending
    pub fn is_pending(&self) -> bool {
        self.interrupt_pending.load(Ordering::Acquire)
    }
}

/// RTIC app module for USB host
pub mod app {
    use super::*;
    use rtic::app;
    
    /// RTIC app configuration
    #[app(device = crate::pac, peripherals = true, dispatchers = [UART1, UART2])]
    pub mod usb_host_app {
        use super::*;
        use crate::perf::PERF_COUNTERS;
        use crate::rtic::example::USB_INTERRUPT_HANDLER;
        
        /// Shared resources
        #[shared]
        struct Shared {
            /// USB interrupt context
            usb_context: UsbInterruptContext,
            /// Control transfer manager
            transfer_manager: ControlTransferManager<16>,
            /// Descriptor allocator
            descriptor_allocator: DescriptorAllocator<32, 128>,
        }
        
        /// Local resources
        #[local]
        struct Local {
            /// Performance timer
            perf_timer: u32,
        }
        
        /// RTIC init
        #[init]
        fn init(ctx: init::Context) -> (Shared, Local) {
            // Initialize hardware
            // This would normally set up clocks, PHY, etc.
            
            // Create USB interrupt context
            let usb_context = UsbInterruptContext::new(
                &USB_INTERRUPT_HANDLER,
                &PERF_COUNTERS,
                0x402E_0000, // USB1 base address
            );
            
            // Create transfer manager
            let transfer_manager = ControlTransferManager::new();
            
            // Create descriptor allocator (would use actual memory in real impl)
            let descriptor_allocator = unsafe {
                // Placeholder - would use DMA memory region
                let qh_mem = &mut [][..];
                let qtd_mem = &mut [][..];
                DescriptorAllocator::new(qh_mem, qtd_mem)
            };
            
            // Enable USB interrupt
            // rtic::pend(Interrupt::USB1);
            
            (
                Shared {
                    usb_context,
                    transfer_manager,
                    descriptor_allocator,
                },
                Local {
                    perf_timer: 0,
                },
            )
        }
        
        /// USB interrupt handler (highest priority)
        #[task(binds = USB1, priority = 15, shared = [usb_context])]
        fn usb_interrupt(mut ctx: usb_interrupt::Context) {
            ctx.shared.usb_context.lock(|context| {
                if let Err(e) = context.usb_interrupt() {
                    // Log error
                    #[cfg(feature = "defmt")]
                    defmt::error!("USB interrupt error: {:?}", e);
                }
                
                // Spawn deferred processing if needed
                if context.is_pending() {
                    let _ = process_usb_deferred::spawn();
                }
            });
        }
        
        /// Deferred USB processing (medium priority)
        #[task(priority = 8, shared = [usb_context, transfer_manager, descriptor_allocator])]
        async fn process_usb_deferred(ctx: process_usb_deferred::Context) {
            let mut usb_context = ctx.shared.usb_context;
            let mut transfer_manager = ctx.shared.transfer_manager;
            let mut descriptor_allocator = ctx.shared.descriptor_allocator;
            
            usb_context.lock(|context| {
                if let Err(e) = context.process_deferred() {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("Deferred processing error: {:?}", e);
                }
            });
            
            // Process completed transfers
            // This would scan the async/periodic lists and update transfer states
            
            // Spawn enumeration if new device detected
            // if new_device_detected {
            //     let _ = enumerate_device::spawn(port);
            // }
        }
        
        /// Device enumeration task (low priority)
        #[task(priority = 4, shared = [transfer_manager, descriptor_allocator])]
        async fn enumerate_device(
            ctx: enumerate_device::Context,
            port: u8,
        ) {
            let mut transfer_manager = ctx.shared.transfer_manager;
            let mut descriptor_allocator = ctx.shared.descriptor_allocator;
            
            // Enumeration sequence:
            // 1. Reset port
            // 2. Get device descriptor (8 bytes)
            // 3. Set address
            // 4. Get full device descriptor
            // 5. Get configuration descriptor
            // 6. Set configuration
            
            #[cfg(feature = "defmt")]
            defmt::info!("Starting enumeration on port {}", port);
            
            // Implementation would go here
        }
        
        /// Periodic health check (lowest priority)
        #[task(priority = 1, shared = [usb_context], local = [perf_timer])]
        async fn health_check(ctx: health_check::Context) {
            let usb_context = ctx.shared.usb_context;
            let perf_timer = ctx.local.perf_timer;
            
            // Check interrupt handler health
            usb_context.lock(|context| {
                if !context.handler.is_healthy() {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("USB interrupt handler unhealthy!");
                }
            });
            
            // Log performance stats periodically
            *perf_timer += 1;
            if *perf_timer >= 1000 {
                *perf_timer = 0;
                
                #[cfg(feature = "defmt")]
                defmt::info!("USB performance: {:?}", PERF_COUNTERS.snapshot());
            }
            
            // Re-spawn after delay
            // let _ = health_check::spawn_after(1.secs());
        }
    }
}

/// Integration with existing RTIC module
impl UsbInterruptHandler {
    /// Process transfer completion event
    pub fn handle_transfer_complete(&self, qtd_addr: u32) -> Result<()> {
        // Find the transfer associated with this qTD
        // Update transfer state machine
        // Signal completion to waiting task
        
        self.perf_counters.record_transfer_success(1);
        Ok(())
    }
    
    /// Process port change event
    pub fn handle_port_change_event(&self, port: u8) -> Result<()> {
        unsafe {
            let portsc_addr = (0x402E_0184 + (port as usize * 4)) as *mut u32;
            let portsc = core::ptr::read_volatile(portsc_addr);
            
            // Clear change bits by writing 1s
            let clear_bits = portsc & 0x0E;
            core::ptr::write_volatile(portsc_addr, clear_bits);
            
            // Check what changed
            if portsc & (1 << 1) != 0 {
                // Connect status changed
                if portsc & (1 << 0) != 0 {
                    // Device connected
                    #[cfg(feature = "defmt")]
                    defmt::info!("Device connected on port {}", port);
                    
                    // Queue enumeration
                } else {
                    // Device disconnected
                    #[cfg(feature = "defmt")]
                    defmt::info!("Device disconnected from port {}", port);
                    
                    // Clean up resources
                }
            }
            
            if portsc & (1 << 3) != 0 {
                // Over-current change
                if portsc & (1 << 4) != 0 {
                    // Over-current active
                    self.urgent_events.enqueue(UrgentEvent::PowerError);
                }
            }
        }
        
        Ok(())
    }
}