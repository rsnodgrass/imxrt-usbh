//! RTIC support for USB host driver
//! 
//! Provides interrupt-driven USB host operations using RTIC framework

use crate::error::{Result, UsbError};
use crate::ehci::EhciController;
use heapless::spsc::{Queue, Producer, Consumer};
use rtic_sync::channel::{Channel, Sender, Receiver};

/// USB event types for RTIC processing
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UsbEvent {
    /// USB transfer completed
    TransferComplete,
    /// Port status changed (connect/disconnect)
    PortChange { port: u8 },
    /// USB error occurred
    Error(UsbError),
    /// Host system error (requires reset)
    HostSystemError,
    /// Frame list rollover
    FrameListRollover,
}

/// USB host resources for RTIC
pub struct UsbHostResources {
    /// Event queue for bottom-half processing
    pub event_queue: Queue<UsbEvent, 32>,
    /// Transfer completion notifications
    pub transfer_complete: Channel<(), 16>,
}

impl UsbHostResources {
    /// Create new USB host resources
    pub const fn new() -> Self {
        Self {
            event_queue: Queue::new(),
            transfer_complete: Channel::new(),
        }
    }
}

/// USB interrupt handler for RTIC
/// 
/// Minimal ISR that acknowledges interrupts and queues events for processing.
/// Designed to execute in <10μs to meet USB timing requirements.
pub fn usb_interrupt_handler(
    controller: &EhciController,
    event_producer: &mut Producer<'_, UsbEvent, 32>,
) -> Result<()> {
    // Read and acknowledge interrupts immediately (RM 66.6.16)
    let status = controller.interrupt_status();
    controller.clear_interrupt_status(status);
    
    // Queue events for bottom-half processing (non-blocking)
    if status.contains(crate::ehci::UsbSts::USB_INTERRUPT) {
        let _ = event_producer.enqueue(UsbEvent::TransferComplete);
    }
    
    if status.contains(crate::ehci::UsbSts::PORT_CHANGE_DETECT) {
        // Check which port(s) changed
        for port in 0..controller.port_count() {
            if let Ok(port_status) = controller.port_status(port.into()) {
                if port_status.contains(crate::ehci::PortSc::CONNECT_STATUS_CHANGE) {
                    let _ = event_producer.enqueue(UsbEvent::PortChange { port });
                }
            }
        }
    }
    
    if status.contains(crate::ehci::UsbSts::USB_ERROR_INTERRUPT) {
        let _ = event_producer.enqueue(UsbEvent::Error(UsbError::TransactionError));
    }
    
    if status.contains(crate::ehci::UsbSts::HOST_SYSTEM_ERROR) {
        let _ = event_producer.enqueue(UsbEvent::HostSystemError);
    }
    
    if status.contains(crate::ehci::UsbSts::FRAME_LIST_ROLLOVER) {
        let _ = event_producer.enqueue(UsbEvent::FrameListRollover);
    }
    
    Ok(())
}

/// USB event processor for RTIC bottom-half task
/// 
/// Processes USB events from the interrupt handler in a lower-priority task.
/// This allows for longer processing times without blocking interrupts.
pub struct UsbEventProcessor {
    controller: &'static EhciController,
    transfer_sender: Sender<'static, (), 16>,
}

impl UsbEventProcessor {
    /// Create new event processor
    pub fn new(
        controller: &'static EhciController,
        transfer_sender: Sender<'static, (), 16>,
    ) -> Self {
        Self {
            controller,
            transfer_sender,
        }
    }
    
    /// Process a USB event
    pub fn process_event(&mut self, event: UsbEvent) -> Result<()> {
        match event {
            UsbEvent::TransferComplete => {
                self.handle_transfer_complete()?;
            }
            UsbEvent::PortChange { port } => {
                self.handle_port_change(port)?;
            }
            UsbEvent::Error(error) => {
                self.handle_error(error)?;
            }
            UsbEvent::HostSystemError => {
                self.handle_host_system_error()?;
            }
            UsbEvent::FrameListRollover => {
                // Frame list rollover is informational
                #[cfg(feature = "defmt")]
                defmt::trace!("Frame list rollover");
            }
        }
        Ok(())
    }
    
    fn handle_transfer_complete(&mut self) -> Result<()> {
        // Scan async schedule for completed transfers
        // This will be implemented when qTD/qH structures are ready
        
        // Notify waiting tasks
        let _ = self.transfer_sender.try_send(());
        
        Ok(())
    }
    
    fn handle_port_change(&mut self, port: u8) -> Result<()> {
        let port_status = self.controller.port_status(port.into())?;
        
        if port_status.contains(crate::ehci::PortSc::CURRENT_CONNECT_STATUS) {
            #[cfg(feature = "defmt")]
            defmt::info!("Device connected on port {}", port);
            
            // Begin enumeration process
            // This will be implemented with the enumeration state machine
        } else {
            #[cfg(feature = "defmt")]
            defmt::info!("Device disconnected from port {}", port);
            
            // Clean up device resources
        }
        
        // Clear port change bit
        self.controller.clear_port_change(port.into())?;
        
        Ok(())
    }
    
    fn handle_error(&mut self, error: UsbError) -> Result<()> {
        #[cfg(feature = "defmt")]
        defmt::error!("USB error: {:?}", error);
        
        // Implement error recovery based on error type
        match error {
            UsbError::TransactionError => {
                // Retry transaction or mark endpoint as halted
            }
            UsbError::Timeout => {
                // Cancel timed-out transfer
            }
            _ => {
                // Log error for debugging
            }
        }
        
        Ok(())
    }
    
    fn handle_host_system_error(&mut self) -> Result<()> {
        #[cfg(feature = "defmt")]
        defmt::error!("Host system error - resetting controller");
        
        // Stop controller
        unsafe {
            self.controller.stop();
            
            // Wait for halt
            let timeout = crate::ehci::register::RegisterTimeout::new_us(1000);
            timeout.wait_for(|| self.controller.is_halted())?;
            
            // Reset controller
            self.controller.reset();
            
            // Wait for reset completion
            let timeout = crate::ehci::register::RegisterTimeout::new_us(10000);
            timeout.wait_for(|| self.controller.is_reset_complete())?;
            
            // Restart controller
            self.controller.start();
        }
        
        Ok(())
    }
}

/// Example RTIC application structure
/// 
/// This demonstrates how to integrate the USB host driver with RTIC.
/// Users should adapt this to their specific application needs.
#[cfg(feature = "defmt")]
pub mod example {
    use super::*;
    
    /// RTIC app module structure
    #[rtic::app(device = imxrt_ral, dispatchers = [LPSPI1])]
    pub mod app {
        use super::super::*;
        use crate::ehci::{EhciController, USB1_BASE};
        
        /// Shared resources
        #[shared]
        struct Shared {
            usb_controller: EhciController,
        }
        
        /// Local resources
        #[local]
        struct Local {
            event_producer: Producer<'static, UsbEvent, 32>,
            event_consumer: Consumer<'static, UsbEvent, 32>,
            event_processor: UsbEventProcessor,
        }
        
        /// Initialize USB host
        #[init(local = [resources: UsbHostResources = UsbHostResources::new()])]
        fn init(ctx: init::Context) -> (Shared, Local) {
            // Initialize USB controller
            let controller = unsafe {
                EhciController::new(USB1_BASE).expect("Failed to initialize USB controller")
            };
            
            // Split event queue
            let (producer, consumer) = ctx.local.resources.event_queue.split();
            
            // Create event processor
            let (sender, _receiver) = ctx.local.resources.transfer_complete.split();
            let event_processor = UsbEventProcessor::new(
                unsafe { core::mem::transmute(&controller) },
                sender,
            );
            
            // Enable USB interrupts
            controller.enable_interrupts(
                crate::ehci::UsbIntr::USB_INTERRUPT_ENABLE |
                crate::ehci::UsbIntr::USB_ERROR_INTERRUPT_ENABLE |
                crate::ehci::UsbIntr::PORT_CHANGE_INTERRUPT_ENABLE |
                crate::ehci::UsbIntr::HOST_SYSTEM_ERROR_ENABLE
            );
            
            (
                Shared {
                    usb_controller: controller,
                },
                Local {
                    event_producer: producer,
                    event_consumer: consumer,
                    event_processor,
                },
            )
        }
        
        /// USB interrupt handler (high priority)
        #[task(binds = USB_OTG1, priority = 3, shared = [usb_controller], local = [event_producer])]
        fn usb_interrupt(ctx: usb_interrupt::Context) {
            let controller = ctx.shared.usb_controller;
            let producer = ctx.local.event_producer;
            
            // Handle interrupt in minimal time
            let _ = usb_interrupt_handler(controller, producer);
        }
        
        /// USB event processor (low priority)
        #[task(priority = 1, local = [event_consumer, event_processor])]
        async fn process_usb_events(ctx: process_usb_events::Context) {
            loop {
                if let Some(event) = ctx.local.event_consumer.dequeue() {
                    let _ = ctx.local.event_processor.process_event(event);
                }
                
                // Yield to other tasks
                rtic_monotonics::systick::Systick::delay(1.millis()).await;
            }
        }
    }
}