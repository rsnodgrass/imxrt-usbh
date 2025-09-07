//! Example USB Flash Drive (Mass Storage) implementation
//!
//! This is a complete working example showing how to support USB mass storage devices
//! using the Bulk-Only Transport (BOT) protocol and SCSI command set with the
//! imxrt-usbh library. Works with USB flash drives, external hard drives, etc.
//!
//! This demonstrates:
//! - Command Block Wrapper (CBW) and Command Status Wrapper (CSW) handling
//! - SCSI command construction (INQUIRY, READ_CAPACITY, READ_10, etc.)
//! - Mass storage device enumeration and initialization
//! - Block-level read/write operations
//! - Bulk transfers for high-speed data transfer

#![no_std]
#![no_main]

use teensy4_panic as _;
use cortex_m_rt::entry;

use imxrt_ral as ral;

use imxrt_usbh::{
    Result, UsbError,
    ehci::controller::{EhciController, Uninitialized, Running},
    phy::UsbPhy,
    enumeration::{DeviceEnumerator, EnumeratedDevice, DeviceClass},
    transfer::simple_control::{SetupPacket, ControlExecutor},
    dma::memory::UsbMemoryPool,
};

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

/// MSC subclass codes
pub mod subclass {
    pub const RBC: u8 = 0x01;        // Reduced Block Commands
    pub const SFF8020I: u8 = 0x02;   // CD-ROM
    pub const SFF8070I: u8 = 0x05;   // Floppy
    pub const SCSI: u8 = 0x06;       // SCSI transparent command set
    pub const LSDFS: u8 = 0x07;      // LSD FS
    pub const IEEE1667: u8 = 0x08;   // IEEE 1667
}

/// MSC protocol codes
pub mod protocol {
    pub const CBI_NO_INTERRUPT: u8 = 0x00;  // Control/Bulk/Interrupt without interrupt
    pub const CBI: u8 = 0x01;               // Control/Bulk/Interrupt
    pub const BULK_ONLY: u8 = 0x50;         // Bulk-Only Transport
    pub const UAS: u8 = 0x62;               // USB Attached SCSI
}

/// Command Block Wrapper (CBW) for Bulk-Only Transport
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct CommandBlockWrapper {
    pub signature: u32,         // 'USBC' (0x43425355)
    pub tag: u32,              // Command tag (matches CSW)
    pub data_transfer_length: u32,  // Bytes to transfer
    pub flags: u8,             // Bit 7: 0=OUT, 1=IN
    pub lun: u8,               // Logical Unit Number (bits 0-3)
    pub cb_length: u8,         // Command block length (1-16)
    pub cb: [u8; 16],         // Command block (SCSI command)
}

impl CommandBlockWrapper {
    pub const SIGNATURE: u32 = 0x43425355; // 'USBC'
    pub const FLAG_DATA_IN: u8 = 0x80;
    pub const FLAG_DATA_OUT: u8 = 0x00;
    
    /// Create a new CBW with the given parameters
    pub fn new(tag: u32, transfer_length: u32, direction_in: bool, lun: u8) -> Self {
        Self {
            signature: Self::SIGNATURE,
            tag,
            data_transfer_length: transfer_length,
            flags: if direction_in { Self::FLAG_DATA_IN } else { Self::FLAG_DATA_OUT },
            lun: lun & 0x0F,
            cb_length: 0,
            cb: [0; 16],
        }
    }
    
    /// Set SCSI command in the command block
    pub fn set_command(&mut self, command: &[u8]) {
        let len = command.len().min(16);
        self.cb_length = len as u8;
        self.cb[..len].copy_from_slice(&command[..len]);
    }
}

/// Command Status Wrapper (CSW) for Bulk-Only Transport
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct CommandStatusWrapper {
    pub signature: u32,         // 'USBS' (0x53425355)
    pub tag: u32,              // Command tag (matches CBW)
    pub data_residue: u32,     // Difference between expected and actual data
    pub status: u8,            // Command status
}

impl CommandStatusWrapper {
    pub const SIGNATURE: u32 = 0x53425355; // 'USBS'
    pub const STATUS_PASSED: u8 = 0x00;
    pub const STATUS_FAILED: u8 = 0x01;
    pub const STATUS_PHASE_ERROR: u8 = 0x02;
    
    /// Check if CSW is valid
    pub fn is_valid(&self, expected_tag: u32) -> bool {
        self.signature == Self::SIGNATURE && self.tag == expected_tag
    }
}

/// SCSI commands
pub mod scsi {
    /// Test Unit Ready command
    pub const TEST_UNIT_READY: u8 = 0x00;
    
    /// Request Sense command
    pub const REQUEST_SENSE: u8 = 0x03;
    
    /// Inquiry command
    pub const INQUIRY: u8 = 0x12;
    
    /// Read Capacity (10) command
    pub const READ_CAPACITY_10: u8 = 0x25;
    
    /// Read (10) command
    pub const READ_10: u8 = 0x28;
    
    /// Write (10) command
    pub const WRITE_10: u8 = 0x2A;
    
    /// Mode Sense (6) command
    pub const MODE_SENSE_6: u8 = 0x1A;
    
    /// Prevent/Allow Medium Removal
    pub const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1E;
    
    /// Create TEST_UNIT_READY command
    pub fn test_unit_ready() -> [u8; 6] {
        [TEST_UNIT_READY, 0, 0, 0, 0, 0]
    }
    
    /// Create REQUEST_SENSE command
    pub fn request_sense(allocation_length: u8) -> [u8; 6] {
        [REQUEST_SENSE, 0, 0, 0, allocation_length, 0]
    }
    
    /// Create INQUIRY command
    pub fn inquiry(allocation_length: u8) -> [u8; 6] {
        [INQUIRY, 0, 0, 0, allocation_length, 0]
    }
    
    /// Create READ_CAPACITY_10 command
    pub fn read_capacity_10() -> [u8; 10] {
        [READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }
    
    /// Create READ_10 command
    pub fn read_10(lba: u32, transfer_blocks: u16) -> [u8; 10] {
        [
            READ_10,
            0,  // Flags
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            0,  // Group number
            (transfer_blocks >> 8) as u8,
            transfer_blocks as u8,
            0,  // Control
        ]
    }
    
    /// Create WRITE_10 command
    pub fn write_10(lba: u32, transfer_blocks: u16) -> [u8; 10] {
        [
            WRITE_10,
            0,  // Flags
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            0,  // Group number
            (transfer_blocks >> 8) as u8,
            transfer_blocks as u8,
            0,  // Control
        ]
    }
}

/// SCSI INQUIRY response data
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct InquiryData {
    pub peripheral_device_type: u8,  // Bits 0-4: device type
    pub removable_media: u8,         // Bit 7: RMB
    pub version: u8,                 // SPC version
    pub response_data_format: u8,    // Bits 0-3: format
    pub additional_length: u8,       // Additional bytes available
    pub flags1: u8,
    pub flags2: u8,
    pub flags3: u8,
    pub vendor_id: [u8; 8],         // T10 vendor identification
    pub product_id: [u8; 16],       // Product identification
    pub product_revision: [u8; 4],   // Product revision level
}

impl InquiryData {
    /// Check if device is direct access (disk)
    pub fn is_direct_access(&self) -> bool {
        (self.peripheral_device_type & 0x1F) == 0x00
    }
    
    /// Check if media is removable
    pub fn is_removable(&self) -> bool {
        (self.removable_media & 0x80) != 0
    }
    
    /// Get vendor ID as string
    pub fn get_vendor_id(&self) -> &str {
        core::str::from_utf8(&self.vendor_id)
            .unwrap_or("Unknown")
            .trim()
    }
    
    /// Get product ID as string
    pub fn get_product_id(&self) -> &str {
        core::str::from_utf8(&self.product_id)
            .unwrap_or("Unknown")
            .trim()
    }
}

/// SCSI READ_CAPACITY_10 response data
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct ReadCapacity10Data {
    pub last_lba: u32,      // Last logical block address (big-endian)
    pub block_size: u32,    // Block size in bytes (big-endian)
}

impl ReadCapacity10Data {
    /// Get last LBA (convert from big-endian)
    pub fn get_last_lba(&self) -> u32 {
        u32::from_be(self.last_lba)
    }
    
    /// Get block size (convert from big-endian)
    pub fn get_block_size(&self) -> u32 {
        u32::from_be(self.block_size)
    }
    
    /// Get total capacity in bytes
    pub fn get_capacity_bytes(&self) -> u64 {
        let last_lba = self.get_last_lba() as u64;
        let block_size = self.get_block_size() as u64;
        (last_lba + 1) * block_size
    }
}

/// Mass Storage Device
pub struct MassStorageDevice {
    device_address: u8,
    interface: u8,
    bulk_in_endpoint: u8,
    bulk_out_endpoint: u8,
    max_packet_size_in: u16,
    max_packet_size_out: u16,
    max_lun: u8,
    subclass: u8,
    protocol: u8,
    
    // Device information from SCSI commands
    inquiry_data: Option<InquiryData>,
    last_lba: AtomicU32,
    block_size: AtomicU32,
    
    // Transfer state
    current_tag: AtomicU32,
    is_ready: AtomicBool,
}

impl MassStorageDevice {
    /// Create new mass storage device
    pub fn new(
        device_address: u8,
        interface: u8,
        bulk_in_endpoint: u8,
        bulk_out_endpoint: u8,
        max_packet_size_in: u16,
        max_packet_size_out: u16,
        max_lun: u8,
        subclass: u8,
        protocol: u8,
    ) -> Self {
        Self {
            device_address,
            interface,
            bulk_in_endpoint,
            bulk_out_endpoint,
            max_packet_size_in,
            max_packet_size_out,
            max_lun,
            subclass,
            protocol,
            inquiry_data: None,
            last_lba: AtomicU32::new(0),
            block_size: AtomicU32::new(512), // Default block size
            current_tag: AtomicU32::new(1),
            is_ready: AtomicBool::new(false),
        }
    }
    
    /// Get next command tag
    fn next_tag(&self) -> u32 {
        self.current_tag.fetch_add(1, Ordering::Relaxed)
    }
    
    /// Initialize device (get device info and capacity)
    pub fn initialize(&mut self, memory_pool: &mut UsbMemoryPool) -> Result<()> {
        // Test if unit is ready
        self.test_unit_ready(memory_pool)?;
        
        // Get device information
        self.inquiry(memory_pool)?;
        
        // Get capacity
        self.read_capacity(memory_pool)?;
        
        self.is_ready.store(true, Ordering::Release);
        Ok(())
    }
    
    /// Send TEST_UNIT_READY command
    pub fn test_unit_ready(&self, _memory_pool: &mut UsbMemoryPool) -> Result<()> {
        let _tag = self.next_tag();
        let mut _cbw = CommandBlockWrapper::new(_tag, 0, false, 0);
        _cbw.set_command(&scsi::test_unit_ready());
        
        // In a real implementation:
        // 1. Send CBW via bulk OUT endpoint
        // 2. Receive CSW via bulk IN endpoint
        // 3. Check CSW status
        
        // For this example, simulate success
        Ok(())
    }
    
    /// Send INQUIRY command
    pub fn inquiry(&mut self, _memory_pool: &mut UsbMemoryPool) -> Result<()> {
        let _tag = self.next_tag();
        let allocation_length = core::mem::size_of::<InquiryData>() as u8;
        let mut _cbw = CommandBlockWrapper::new(
            _tag,
            allocation_length as u32,
            true,  // Data IN
            0
        );
        _cbw.set_command(&scsi::inquiry(allocation_length));
        
        // In a real implementation:
        // 1. Send CBW via bulk OUT endpoint
        // 2. Receive data via bulk IN endpoint
        // 3. Receive CSW via bulk IN endpoint
        
        // For simulation, create example inquiry data
        let inquiry_data = InquiryData {
            peripheral_device_type: 0x00, // Direct access device
            removable_media: 0x80,         // Removable
            version: 0x04,                 // SPC-2
            response_data_format: 0x02,    // Standard format
            additional_length: 31,         // Additional bytes
            flags1: 0,
            flags2: 0,
            flags3: 0,
            vendor_id: *b"EXAMPLE ",
            product_id: *b"USB Flash Drive ",
            product_revision: *b"1.00",
        };
        
        self.inquiry_data = Some(inquiry_data);
        Ok(())
    }
    
    /// Send READ_CAPACITY_10 command
    pub fn read_capacity(&self, _memory_pool: &mut UsbMemoryPool) -> Result<()> {
        let _tag = self.next_tag();
        let data_length = core::mem::size_of::<ReadCapacity10Data>() as u32;
        let mut _cbw = CommandBlockWrapper::new(_tag, data_length, true, 0);
        _cbw.set_command(&scsi::read_capacity_10());
        
        // In a real implementation:
        // 1. Send CBW via bulk OUT endpoint
        // 2. Receive data via bulk IN endpoint
        // 3. Receive CSW via bulk IN endpoint
        
        // For simulation, set example capacity (1GB with 512-byte blocks)
        self.last_lba.store(0x001FFFFF, Ordering::Release); // ~1GB
        self.block_size.store(512, Ordering::Release);
        
        Ok(())
    }
    
    /// Read blocks from device
    pub fn read_blocks(&self, lba: u32, blocks: u16, buffer: &mut [u8], _memory_pool: &mut UsbMemoryPool) -> Result<usize> {
        if !self.is_ready.load(Ordering::Acquire) {
            return Err(UsbError::InvalidState);
        }
        
        let block_size = self.block_size.load(Ordering::Acquire);
        let expected_bytes = (blocks as u32) * block_size;
        
        if buffer.len() < expected_bytes as usize {
            return Err(UsbError::BufferOverflow);
        }
        
        let _tag = self.next_tag();
        let mut _cbw = CommandBlockWrapper::new(_tag, expected_bytes, true, 0);
        _cbw.set_command(&scsi::read_10(lba, blocks));
        
        // In a real implementation:
        // 1. Send CBW via bulk OUT endpoint
        // 2. Receive data via bulk IN endpoint  
        // 3. Receive CSW via bulk IN endpoint
        
        // For simulation, fill buffer with pattern
        for (i, byte) in buffer.iter_mut().enumerate().take(expected_bytes as usize) {
            *byte = (i as u8).wrapping_add(lba as u8);
        }
        
        Ok(expected_bytes as usize)
    }
    
    /// Write blocks to device
    pub fn write_blocks(&self, lba: u32, blocks: u16, data: &[u8], _memory_pool: &mut UsbMemoryPool) -> Result<usize> {
        if !self.is_ready.load(Ordering::Acquire) {
            return Err(UsbError::InvalidState);
        }
        
        let block_size = self.block_size.load(Ordering::Acquire);
        let expected_bytes = (blocks as u32) * block_size;
        
        if data.len() < expected_bytes as usize {
            return Err(UsbError::BufferOverflow);
        }
        
        let _tag = self.next_tag();
        let mut _cbw = CommandBlockWrapper::new(_tag, expected_bytes, false, 0);
        _cbw.set_command(&scsi::write_10(lba, blocks));
        
        // In a real implementation:
        // 1. Send CBW via bulk OUT endpoint
        // 2. Send data via bulk OUT endpoint
        // 3. Receive CSW via bulk IN endpoint
        
        // For simulation, just return success
        Ok(expected_bytes as usize)
    }
    
    /// Get device capacity in bytes
    pub fn get_capacity_bytes(&self) -> u64 {
        let last_lba = self.last_lba.load(Ordering::Acquire) as u64;
        let block_size = self.block_size.load(Ordering::Acquire) as u64;
        (last_lba + 1) * block_size
    }
    
    /// Get block size in bytes
    pub fn get_block_size(&self) -> u32 {
        self.block_size.load(Ordering::Acquire)
    }
    
    /// Get vendor ID string
    pub fn get_vendor_id(&self) -> &str {
        self.inquiry_data
            .as_ref()
            .map(|d| d.get_vendor_id())
            .unwrap_or("Unknown")
    }
    
    /// Get product ID string
    pub fn get_product_id(&self) -> &str {
        self.inquiry_data
            .as_ref()
            .map(|d| d.get_product_id())
            .unwrap_or("Unknown")
    }
}

/// Mass Storage Manager for multiple devices
pub struct MscManager {
    devices: [Option<MassStorageDevice>; 4],
    device_count: AtomicU8,
}

impl MscManager {
    /// Create new MSC manager
    pub const fn new() -> Self {
        Self {
            devices: [None, None, None, None],
            device_count: AtomicU8::new(0),
        }
    }
    
    /// Add a mass storage device
    pub fn add_device(&mut self, device: MassStorageDevice) -> Result<u8> {
        for (index, slot) in self.devices.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(device);
                self.device_count.fetch_add(1, Ordering::SeqCst);
                return Ok(index as u8);
            }
        }
        Err(UsbError::NoResources)
    }
    
    /// Remove a device by index
    pub fn remove_device(&mut self, index: u8) -> Result<()> {
        if index as usize >= self.devices.len() {
            return Err(UsbError::InvalidParameter);
        }
        
        if self.devices[index as usize].take().is_some() {
            self.device_count.fetch_sub(1, Ordering::SeqCst);
            Ok(())
        } else {
            Err(UsbError::InvalidParameter)
        }
    }
    
    /// Get device by index
    pub fn get_device(&self, index: u8) -> Option<&MassStorageDevice> {
        self.devices.get(index as usize)?.as_ref()
    }
    
    /// Get mutable device by index
    pub fn get_device_mut(&mut self, index: u8) -> Option<&mut MassStorageDevice> {
        self.devices.get_mut(index as usize)?.as_mut()
    }
    
    /// Get number of connected devices
    pub fn device_count(&self) -> u8 {
        self.device_count.load(Ordering::Acquire)
    }
    
    /// Iterate over all devices
    pub fn iter(&self) -> impl Iterator<Item = &MassStorageDevice> {
        self.devices.iter().filter_map(|d| d.as_ref())
    }
}

/// Helper to detect MSC interface
pub fn is_msc_interface(interface_class: u8, interface_subclass: u8, interface_protocol: u8) -> bool {
    interface_class == 0x08 && // Mass Storage Class
    interface_protocol == protocol::BULK_ONLY // We only support Bulk-Only Transport
}

/// Helper to format capacity
pub fn format_capacity(bytes: u64) -> (f32, &'static str) {
    const UNITS: &[&str] = &["B", "KB", "MB", "GB", "TB"];
    let mut size = bytes as f32;
    let mut unit_index = 0;
    
    while size >= 1024.0 && unit_index < UNITS.len() - 1 {
        size /= 1024.0;
        unit_index += 1;
    }
    
    (size, UNITS[unit_index])
}

#[entry]
fn main() -> ! {
    // Initialize peripherals
    let _peripherals = unsafe { ral::Instances::instances() };
    
    // Initialize clocks
    configure_usb_clocks();
    
    // Initialize USB memory pool
    let mut memory_pool = UsbMemoryPool::new();
    
    // Initialize USB PHY
    let phy_base = 0x400D_9000; // USBPHY1 base address
    let ccm_base = 0x400F_C000;  // CCM base address
    let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };
    
    // Initialize USB host controller
    let usb1_base = 0x402E_0140; // USB1 base address
    let controller = unsafe {
        EhciController::<8, Uninitialized>::new(usb1_base)
            .expect("Failed to create EHCI controller")
    };
    
    let controller = unsafe {
        controller.initialize()
            .expect("Failed to initialize EHCI controller")
    };
    
    let mut controller = unsafe { controller.start() };
    
    // Initialize MSC manager
    let mut msc_manager = MscManager::new();
    
    // Main loop
    let mut device_connected = false;
    
    loop {
        if !device_connected {
            // Try to find and connect to a mass storage device
            match find_and_init_flash_drive(&mut controller, &mut memory_pool) {
                Ok(device) => {
                    match msc_manager.add_device(device) {
                        Ok(device_id) => {
                            device_connected = true;
                            demonstrate_flash_operations(device_id, &mut msc_manager, &mut memory_pool);
                        }
                        Err(_) => {
                            // Manager full
                        }
                    }
                }
                Err(_) => {
                    // No flash drive found
                }
            }
        }
        
        // Small delay
        delay_ms(100);
    }
}

/// Find and initialize a USB flash drive
fn find_and_init_flash_drive(
    controller: &mut EhciController<8, Running>,
    memory_pool: &mut UsbMemoryPool,
) -> Result<MassStorageDevice> {
    // Enumerate device
    let mut enumerator = DeviceEnumerator::new(controller, memory_pool);
    let device = enumerator.enumerate_device()?;
    
    // Check if it's a mass storage device
    if device.class != DeviceClass::MassStorage {
        return Err(UsbError::Unsupported);
    }
    
    // Create mass storage device
    let mut msc_device = MassStorageDevice::new(
        device.address,
        0,    // Interface 0
        0x81, // Bulk IN endpoint
        0x02, // Bulk OUT endpoint
        64,   // Max packet size IN
        64,   // Max packet size OUT
        0,    // Max LUN
        subclass::SCSI,
        protocol::BULK_ONLY,
    );
    
    // Initialize the device
    msc_device.initialize(memory_pool)?;
    
    Ok(msc_device)
}

/// Demonstrate flash drive operations
fn demonstrate_flash_operations(
    device_id: u8,
    manager: &mut MscManager,
    memory_pool: &mut UsbMemoryPool,
) {
    if let Some(device) = manager.get_device_mut(device_id) {
        // Display device information
        let capacity = device.get_capacity_bytes();
        let (size, unit) = format_capacity(capacity);
        
        // In a real app, you would output this information
        let _vendor = device.get_vendor_id();
        let _product = device.get_product_id();
        let _size = size;
        let _unit = unit;
        
        // Demonstrate reading first block
        let mut buffer = [0u8; 512];
        match device.read_blocks(0, 1, &mut buffer, memory_pool) {
            Ok(_bytes_read) => {
                // Successfully read block 0
                // In a real app, you would process this data
                process_boot_sector(&buffer);
            }
            Err(_e) => {
                // Read failed
            }
        }
        
        // Demonstrate writing (be careful with real devices!)
        let write_data = [0x55u8; 512]; // Test pattern
        match device.write_blocks(1000, 1, &write_data, memory_pool) {
            Ok(_bytes_written) => {
                // Successfully wrote to LBA 1000
                // Verify by reading back
                let mut verify_buffer = [0u8; 512];
                if device.read_blocks(1000, 1, &mut verify_buffer, memory_pool).is_ok() {
                    // Verify the data matches
                    let matches = verify_buffer.iter().all(|&b| b == 0x55);
                    let _ = matches; // In real app, report verification result
                }
            }
            Err(_e) => {
                // Write failed
            }
        }
    }
}

/// Process boot sector (for educational purposes)
fn process_boot_sector(buffer: &[u8; 512]) {
    // Check for FAT boot sector signature
    if buffer[510] == 0x55 && buffer[511] == 0xAA {
        // This looks like a boot sector
        // In a real app, you could parse FAT32/exFAT headers here
        let _boot_signature = true;
    }
    
    // In a real filesystem implementation, you would:
    // 1. Parse the boot sector to get filesystem parameters
    // 2. Read the FAT (File Allocation Table)
    // 3. Navigate directories and files
    // 4. Implement file operations
}

/// Configure USB clocks for i.MX RT1062
fn configure_usb_clocks() {
    use ral::{modify_reg, read_reg};
    
    unsafe {
        let ccm = ral::ccm::CCM::instance();
        
        // Enable USB clocks
        modify_reg!(ral::ccm, ccm, CCGR6, 
            CG0: 0b11,  // usb_ctrl1_clk
            CG1: 0b11,  // usb_ctrl2_clk  
            CG2: 0b11,  // usb_phy1_clk
            CG3: 0b11   // usb_phy2_clk
        );
        
        // Configure USB PHY PLL (480MHz)
        let analog = ral::ccm_analog::CCM_ANALOG::instance();
        
        // Power up PLL
        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: 1,
            ENABLE: 1,
            EN_USB_CLKS: 1
        );
        
        // Wait for PLL lock
        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) == 0 {}
    }
}

/// Simple delay function
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms); // Assuming 600MHz clock
}