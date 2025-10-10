//! Simple USB Host API for common use cases
//!
//! This module provides a high-level, batteries-included API for USB host operations.
//! It's designed for the 90% use case where you just want to read from a keyboard,
//! mouse, or gamepad without dealing with low-level EHCI details.
//!
//! # When to Use This API
//!
//! Use the simple API when you:
//! - Want to get started quickly
//! - Are building typical USB host applications (HID devices, simple enumeration)
//! - Don't need precise control over transfer scheduling
//! - Prefer synchronous, blocking operations
//!
//! # When to Use the Low-Level API
//!
//! Use the low-level [`crate::ehci`] API when you:
//! - Need isochronous transfers (audio/video streaming)
//! - Require precise timing control
//! - Want to implement custom transfer scheduling
//! - Need direct EHCI register access
//! - Are implementing USB hub support
//!
//! # Example
//!
//! ```no_run
//! use imxrt_usbh::simple::SimpleUsbHost;
//! use imxrt_usbh::hid::{HidDevice, KeyboardReport};
//!
//! // Initialize USB host (handles PHY, DMA, controller setup)
//! let mut usb = SimpleUsbHost::new(
//!     0x402E_0200,  // USB2 base
//!     0x400DA000,   // PHY2 base
//!     0x400F_C000,  // CCM base
//! )?;
//!
//! // Wait for keyboard to connect
//! let device = usb.wait_for_device()?;
//! let mut keyboard = HidDevice::from_device(device)?;
//! keyboard.enable_boot_protocol(&mut usb)?;
//!
//! // Read keys
//! loop {
//!     if let Some(report) = keyboard.poll_report(&mut usb)? {
//!         let kbd_report = KeyboardReport::parse(report);
//!         for key in kbd_report.keys_pressed() {
//!             if let Some(ch) = key.to_ascii() {
//!                 print!("{}", ch);
//!             }
//!         }
//!     }
//! }
//! # Ok::<(), imxrt_usbh::UsbError>(())
//! ```

use crate::dma::UsbMemoryPool;
use crate::ehci::{EhciController, Running, Uninitialized, TransferExecutor};
use crate::error::Result;
use crate::phy::UsbPhy;
use crate::enumeration::DeviceEnumerator;

pub mod device;
pub mod events;

pub use device::{UsbDevice, TransferType};
pub use events::{DeviceFilter, HostEvent};

/// Simple USB host controller for common use cases
///
/// This bundles together the components needed for typical USB host operations:
/// - EHCI controller
/// - USB PHY
/// - DMA memory pool
/// - Transfer executor
///
/// It provides a simplified interface that handles initialization, enumeration,
/// and data transfer with minimal boilerplate.
pub struct SimpleUsbHost {
    controller: EhciController<8, Running>,
    memory_pool: UsbMemoryPool,
    transfer_executor: TransferExecutor,
    usb_base: usize,
}

impl SimpleUsbHost {
    /// Initialize USB host controller
    ///
    /// This performs all necessary initialization steps:
    /// 1. Initialize USB PHY
    /// 2. Initialize DMA region (cache coherency)
    /// 3. Create and start EHCI controller
    /// 4. Initialize memory pool
    ///
    /// # Arguments
    ///
    /// - `usb_base` - USB controller base address (0x402E_0200 for USB2 on Teensy 4.x)
    /// - `phy_base` - USB PHY base address (0x400DA000 for USBPHY2)
    /// - `ccm_base` - Clock Control Module base address (0x400F_C000)
    ///
    /// # Safety
    ///
    /// This function accesses hardware registers and modifies USB controller state.
    /// Caller must ensure:
    /// - Called only once per USB controller instance
    /// - No other code is accessing these USB registers
    /// - Addresses are valid for the target hardware
    ///
    /// # Example
    ///
    /// ```no_run
    /// use imxrt_usbh::simple::SimpleUsbHost;
    ///
    /// let mut usb = SimpleUsbHost::new(
    ///     0x402E_0200,  // USB2
    ///     0x400DA000,   // USBPHY2
    ///     0x400F_C000,  // CCM
    /// )?;
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn new(usb_base: usize, phy_base: usize, ccm_base: usize) -> Result<Self> {
        // Initialize USB PHY
        let mut phy = unsafe { UsbPhy::new(phy_base, ccm_base) };
        phy.init_host_mode()?;

        // Initialize DMA region (critical for cache coherency)
        unsafe {
            crate::dma::init_dma_region()?;
        }

        // Create and initialize EHCI controller
        let controller = unsafe { EhciController::<8, Uninitialized>::new(usb_base)? };
        let controller = unsafe { controller.initialize()? };
        let controller = unsafe { controller.start() };

        // Initialize memory pool and transfer executor
        let memory_pool = UsbMemoryPool::new();
        let transfer_executor = unsafe { TransferExecutor::new(usb_base) };

        Ok(Self {
            controller,
            memory_pool,
            transfer_executor,
            usb_base,
        })
    }

    /// Wait for a USB device to connect and enumerate it
    ///
    /// This is a blocking call that:
    /// 1. Polls for device connection
    /// 2. Resets the port
    /// 3. Enumerates the device (GET_DESCRIPTOR, SET_ADDRESS, etc.)
    /// 4. Returns a [`UsbDevice`] representing the connected device
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::SimpleUsbHost;
    /// # let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// let device = usb.wait_for_device()?;
    /// println!("Device connected: VID={:04X} PID={:04X}",
    ///     device.vendor_id(), device.product_id());
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn wait_for_device(&mut self) -> Result<UsbDevice> {
        // Poll until device enumeration succeeds
        loop {
            let mut enumerator = DeviceEnumerator::new(
                &mut self.controller,
                &mut self.memory_pool,
                &mut self.transfer_executor,
            );

            if let Ok(enum_device) = enumerator.enumerate_device() {
                // Convert EnumeratedDevice to UsbDevice
                return UsbDevice::from_enumerated(enum_device, &mut self.memory_pool, &mut self.transfer_executor);
            }

            // Wait before retry
            cortex_m::asm::delay(60_000_000); // ~100ms
        }
    }

    /// Poll for device connection (non-blocking)
    ///
    /// Returns `Some(device)` if a device is connected and enumerated,
    /// `None` if no device is present or enumeration failed.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::SimpleUsbHost;
    /// # let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// loop {
    ///     if let Some(device) = usb.poll_device()? {
    ///         println!("Device found!");
    ///         break;
    ///     }
    ///     // Do other work...
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn poll_device(&mut self) -> Result<Option<UsbDevice>> {
        let mut enumerator = DeviceEnumerator::new(
            &mut self.controller,
            &mut self.memory_pool,
            &mut self.transfer_executor,
        );

        match enumerator.enumerate_device() {
            Ok(enum_device) => {
                let device = UsbDevice::from_enumerated(
                    enum_device,
                    &mut self.memory_pool,
                    &mut self.transfer_executor,
                )?;
                Ok(Some(device))
            }
            Err(_) => Ok(None),
        }
    }

    /// Get mutable reference to memory pool
    ///
    /// Advanced users may need direct access to the memory pool for
    /// custom buffer management.
    pub fn memory_pool(&mut self) -> &mut UsbMemoryPool {
        &mut self.memory_pool
    }

    /// Get mutable reference to transfer executor
    ///
    /// Advanced users may need direct access to the transfer executor
    /// for custom control transfers.
    pub fn transfer_executor(&mut self) -> &mut TransferExecutor {
        &mut self.transfer_executor
    }

    /// Get mutable references to both transfer executor and memory pool
    ///
    /// This is needed for creating ControlExecutor instances, which require
    /// mutable access to both components simultaneously.
    pub fn executor_and_pool(&mut self) -> (&mut TransferExecutor, &mut UsbMemoryPool) {
        (&mut self.transfer_executor, &mut self.memory_pool)
    }

    /// Get mutable reference to EHCI controller
    ///
    /// Advanced users may need direct access to the controller for
    /// low-level operations.
    pub fn controller(&mut self) -> &mut EhciController<8, Running> {
        &mut self.controller
    }
}
