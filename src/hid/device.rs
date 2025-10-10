//! HID device wrapper

use crate::error::{Result, UsbError};
use crate::simple::{SimpleUsbHost, UsbDevice};
use crate::transfer::simple_control::SetupPacket;
use crate::transfer::{Direction, InterruptTransferManager};
use super::constants::{HidProtocolMode, HidRequest};

/// HID device wrapper
///
/// Provides high-level operations for HID devices, handling the common
/// setup and data transfer patterns.
pub struct HidDevice {
    device: UsbDevice,
    interface_num: u8,
    interrupt_endpoint: u8,
    max_packet_size: u16,
    interval: u8,
}

impl HidDevice {
    /// Create HID device from enumerated USB device
    ///
    /// Validates that the device is HID class and finds the interrupt endpoint.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::SimpleUsbHost;
    /// # use imxrt_usbh::hid::HidDevice;
    /// # let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// let device = usb.wait_for_device()?;
    ///
    /// if device.is_hid() {
    ///     let mut hid = HidDevice::from_device(device)?;
    ///     // Use HID device...
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn from_device(device: UsbDevice) -> Result<Self> {
        // Validate device class
        if !device.is_hid() {
            return Err(UsbError::InvalidParameter);
        }

        // Find interrupt IN endpoint and extract info before moving device
        let endpoint_info = device
            .find_endpoint(crate::transfer::Direction::In, crate::simple::device::TransferType::Interrupt)
            .ok_or(UsbError::InvalidDescriptor)?;

        let interface_num = endpoint_info.interface_num;
        let interrupt_endpoint = endpoint_info.address;
        let max_packet_size = endpoint_info.max_packet_size;
        let interval = endpoint_info.interval;

        Ok(Self {
            device,
            interface_num,
            interrupt_endpoint,
            max_packet_size,
            interval,
        })
    }

    /// Enable boot protocol
    ///
    /// Configures the device to use simplified boot protocol reports.
    /// This is required for keyboards and mice before reading data.
    ///
    /// Internally performs:
    /// - SET_PROTOCOL (boot mode)
    /// - SET_IDLE (disable auto-repeat)
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::SimpleUsbHost;
    /// # use imxrt_usbh::hid::HidDevice;
    /// # let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// # let device = usb.wait_for_device()?;
    /// let mut hid = HidDevice::from_device(device)?;
    /// hid.enable_boot_protocol(&mut usb)?;
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn enable_boot_protocol(&mut self, host: &mut SimpleUsbHost) -> Result<()> {
        let device_addr = self.device.address();
        let interface = self.interface_num;
        let max_packet_size = self.device.max_packet_ep0() as u16;

        // Get both executor and pool references at once
        let (transfer_executor, memory_pool) = host.executor_and_pool();
        let mut executor = crate::transfer::simple_control::ControlExecutor::new(
            transfer_executor,
            memory_pool,
        );

        // SET_PROTOCOL to boot mode (0x00)
        let setup = SetupPacket {
            bmRequestType: 0x21, // Class, Interface, Host-to-Device
            bRequest: HidRequest::SetProtocol as u8,
            wValue: HidProtocolMode::Boot as u16,
            wIndex: interface as u16,
            wLength: 0,
        };

        executor.execute_with_retry(setup, device_addr, max_packet_size, 3)?;

        // SET_IDLE to 0 (disable auto-repeat, we poll manually)
        let setup = SetupPacket {
            bmRequestType: 0x21,
            bRequest: HidRequest::SetIdle as u8,
            wValue: 0, // 0 = infinite duration (no auto-repeat)
            wIndex: interface as u16,
            wLength: 0,
        };

        executor.execute_with_retry(setup, device_addr, max_packet_size, 3)?;

        Ok(())
    }

    /// Get information needed to set up interrupt transfers
    ///
    /// For reading HID reports, you need to use [`InterruptTransferManager`](crate::InterruptTransferManager).
    /// This method returns the parameters needed to set up the transfer.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use imxrt_usbh::simple::SimpleUsbHost;
    /// use imxrt_usbh::hid::{HidDevice, KeyboardReport};
    /// use imxrt_usbh::{InterruptTransferManager, Direction};
    ///
    /// let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// let device = usb.wait_for_device()?;
    /// let mut hid = HidDevice::from_device(device)?;
    /// hid.enable_boot_protocol(&mut usb)?;
    ///
    /// // Set up interrupt transfer manager
    /// let mut interrupt_mgr = InterruptTransferManager::<4>::new();
    /// let buffer = usb.memory_pool().alloc_buffer(8)?;
    ///
    /// let transfer_id = interrupt_mgr.submit(
    ///     Direction::In,
    ///     hid.address(),
    ///     hid.endpoint(),
    ///     hid.max_packet_size(),
    ///     buffer,
    ///     hid.interval(),
    ///     true, // auto-restart
    /// )?;
    ///
    /// // Poll for data
    /// loop {
    ///     if let Some(transfer) = interrupt_mgr.get_transfer(transfer_id) {
    ///         if transfer.is_complete() {
    ///             // Process report...
    ///         }
    ///     }
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn endpoint_info(&self) -> (u8, u8, u16, u8) {
        (
            self.device.address(),
            self.interrupt_endpoint,
            self.max_packet_size,
            self.interval,
        )
    }

    /// Get device address
    pub fn address(&self) -> u8 {
        self.device.address()
    }

    /// Get interrupt endpoint address
    pub fn endpoint(&self) -> u8 {
        self.interrupt_endpoint
    }

    /// Get maximum packet size
    pub fn max_packet_size(&self) -> u16 {
        self.max_packet_size
    }

    /// Get polling interval (milliseconds)
    pub fn interval(&self) -> u8 {
        self.interval
    }

    /// Get reference to underlying USB device
    pub fn device(&self) -> &UsbDevice {
        &self.device
    }

    /// Create a pre-configured interrupt transfer manager for HID polling
    ///
    /// This factory method creates and configures an `InterruptTransferManager`
    /// with a transfer already set up for reading HID reports. Returns the
    /// manager and transfer ID ready to use.
    ///
    /// # Type Parameters
    ///
    /// - `N` - Maximum number of concurrent interrupt transfers (typically 4 for simple cases)
    ///
    /// # Returns
    ///
    /// Returns `(InterruptTransferManager<N>, transfer_id)` where the transfer is already
    /// submitted and started. Call `process_completed()` in your poll loop.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::SimpleUsbHost;
    /// # use imxrt_usbh::hid::{HidDevice, KeyboardReport};
    /// # let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;
    /// # let device = usb.wait_for_device()?;
    /// let mut hid = HidDevice::from_device(device)?;
    /// hid.enable_boot_protocol(&mut usb)?;
    ///
    /// // Create polling manager - just one line!
    /// let (mut mgr, id) = hid.create_polling_manager::<4>(&mut usb)?;
    ///
    /// // Poll for reports
    /// loop {
    ///     mgr.process_completed(usb.controller());
    ///     if let Some(transfer) = mgr.get_transfer(id) {
    ///         if transfer.is_complete() {
    ///             let report = KeyboardReport::parse(transfer.buffer_data().unwrap());
    ///             // Process report...
    ///         }
    ///     }
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub fn create_polling_manager<const N: usize>(
        &self,
        host: &mut SimpleUsbHost,
    ) -> Result<(InterruptTransferManager<N>, usize)> {
        let mut mgr = InterruptTransferManager::new();

        // Allocate buffer for HID reports
        let buffer = host.memory_pool().alloc_buffer(self.max_packet_size as usize)?;

        // Submit interrupt transfer
        let transfer_id = mgr.submit(
            Direction::In,
            self.device.address(),
            self.interrupt_endpoint & 0x7F, // Remove direction bit
            self.max_packet_size,
            buffer,
            self.interval,
            true, // periodic/auto-restart
        )?;

        Ok((mgr, transfer_id))
    }
}
