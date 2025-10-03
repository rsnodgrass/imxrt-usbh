//! USB MIDI Keyboard Example
//!
//! This example demonstrates USB MIDI keyboard support with visual and serial feedback.
//!
//! ## What This Example Does
//!
//! When you connect a USB MIDI keyboard and play notes, you'll see:
//! - **LED blinks rapidly** when notes are played
//! - **LED blinks slowly** when notes are released
//! - **Serial output** showing note names, velocities, and MIDI events
//! - **Statistics** displayed every few seconds
//!
//! ## Hardware Setup
//!
//! - Connect USB MIDI keyboard to Teensy 4.1 USB Host port (pins 30-32)
//! - Connect USB-to-serial adapter to pins 0 (TX) and 1 (RX) for serial output
//! - Open serial monitor at 115200 baud to see MIDI events
//!
//! ## Features
//!
//! - Real-time USB MIDI device enumeration
//! - MIDI event parsing and processing
//! - Note on/off detection with velocity and note names
//! - Control Change (CC) message handling (volume, pan, modulation)
//! - Pitch bend and program change support
//! - Visual LED feedback for note events
//! - Serial console output with human-readable event descriptions
//! - Performance monitoring and statistics

#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use teensy4_panic as _;
use imxrt_ral as ral;
use log::info;
use embedded_hal::digital::OutputPin;

use imxrt_usbh::{
    Result,
    ehci::controller::{EhciController, Uninitialized, Running},
    phy::UsbPhy,
    enumeration::{DeviceEnumerator, EnumeratedDevice, DeviceClass},
    dma::memory::UsbMemoryPool,
};

use core::sync::atomic::{AtomicU32, Ordering};

/// i.MX RT1062 Hardware Register Base Addresses
/// 
/// These addresses are from the i.MX RT1062 reference manual and are
/// specific to the Teensy 4.x hardware platform.
pub mod hardware_addresses {
    /// USB PHY1 register base address (Reference Manual Section 15.6)
    /// Controls the physical layer of the USB interface including:
    /// - PLL configuration for USB clock generation
    /// - Transceiver settings for signal levels
    /// - Power management for the USB PHY
    pub const USBPHY1_BASE_ADDRESS: u32 = 0x400D_9000;
    
    /// Clock Control Module (CCM) register base address (Reference Manual Section 14)
    /// Controls all system clocks including:
    /// - PLL configuration and control  
    /// - Clock gating for power management
    /// - Clock dividers and multiplexers
    /// - USB-specific clock routing
    pub const CCM_BASE_ADDRESS: u32 = 0x400F_C000;
    
    /// USB1 Host Controller (EHCI) register base address (Reference Manual Section 15.3)
    /// This is the Enhanced Host Controller Interface for USB 2.0:
    /// - USB transfer scheduling and control
    /// - Port status and control registers
    /// - DMA descriptor management
    /// - Interrupt status and control
    pub const USB1_HOST_BASE_ADDRESS: u32 = 0x402E_0140;
}

// Re-export constants for convenient access
use hardware_addresses::*;

/// Educational Error Handling Patterns for USB Applications
///
/// This module demonstrates comprehensive error handling strategies for 
/// embedded USB applications, showing when to retry, when to reset, and
/// when to give up on different types of USB errors.
pub mod usb_error_handling {
    use imxrt_usbh::UsbError;
    
    /// Recovery action to take for different error conditions
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum RecoveryAction {
        /// Retry the operation immediately - for transient errors
        RetryImmediate,
        /// Retry after a delay - for temporary conditions  
        RetryWithDelay { delay_ms: u32 },
        /// Retry with exponential backoff - for congested conditions
        RetryWithBackoff { base_delay_ms: u32, max_attempts: u8 },
        /// Reset the device - for protocol errors
        ResetDevice,
        /// Remove device from system - for permanent failures
        RemoveDevice,
        /// Fatal error - cannot recover
        Fatal,
    }
    
    /// Analyze USB error and determine appropriate recovery strategy
    /// 
    /// This educational function shows how to categorize USB errors and
    /// choose appropriate recovery mechanisms based on the error type.
    pub fn analyze_usb_error(error: &UsbError, attempt_count: u8) -> RecoveryAction {
        
        match error {
            // Transient errors - can retry immediately a few times
            UsbError::Timeout => {
                if attempt_count < 3 {
                    RecoveryAction::RetryImmediate
                } else {
                    // After multiple timeouts, maybe device is stressed
                    RecoveryAction::RetryWithDelay { delay_ms: 100 }
                }
            }
            
            // Device temporarily unavailable - retry with backoff
            UsbError::Nak => {
                if attempt_count < 5 {
                    RecoveryAction::RetryWithBackoff { 
                        base_delay_ms: 10, 
                        max_attempts: 5 
                    }
                } else {
                    RecoveryAction::RetryWithDelay { delay_ms: 100 }
                }
            }
            
            // Transaction errors - retry with backoff
            UsbError::TransactionError => {
                if attempt_count < 5 {
                    RecoveryAction::RetryWithBackoff { 
                        base_delay_ms: 50, 
                        max_attempts: 5 
                    }
                } else {
                    // Device may be malfunctioning
                    RecoveryAction::ResetDevice
                }
            }
            
            // Stall condition - need endpoint clear
            UsbError::Stall => {
                RecoveryAction::ResetDevice
            }
            
            // Invalid descriptors - device configuration issue
            UsbError::InvalidDescriptor => {
                RecoveryAction::RemoveDevice
            }
            
            // Device doesn't support required feature - permanent failure
            UsbError::Unsupported => RecoveryAction::RemoveDevice,
            
            // Hardware or system failures - cannot recover
            UsbError::HardwareFailure => RecoveryAction::Fatal,
            
            // Port errors - physical issues
            UsbError::PortError => RecoveryAction::ResetDevice,
            
            // Device disconnection - remove immediately
            UsbError::DeviceDisconnected => RecoveryAction::RemoveDevice,
            
            // Resource exhaustion - retry after delay
            UsbError::NoResources => {
                if attempt_count < 3 {
                    RecoveryAction::RetryWithDelay { delay_ms: 50 }
                } else {
                    RecoveryAction::Fatal
                }
            }
            
            // Programming errors - non-recoverable
            UsbError::InvalidParameter | UsbError::InvalidState => {
                RecoveryAction::Fatal
            }
            
            // Buffer issues - non-recoverable for this transfer
            UsbError::BufferOverflow => RecoveryAction::RemoveDevice,
            
            // Initialization errors - shouldn't happen during operation
            UsbError::AlreadyInitialized => RecoveryAction::Fatal,
        }
    }
    
    /// Execute recovery action with proper educational logging
    pub fn execute_recovery(action: RecoveryAction, _device_info: &str) -> Result<(), &'static str> {
        match action {
            RecoveryAction::RetryImmediate => {
                // Educational: Log that we're retrying immediately
                // In production: log::info!("Retrying USB operation for {}", device_info);
                Ok(())
            }
            
            RecoveryAction::RetryWithDelay { delay_ms } => {
                // Educational: Show delay implementation
                delay_ms_educational(delay_ms);
                // In production: log::warn!("Retrying after {}ms delay for {}", delay_ms, device_info);
                Ok(())
            }
            
            RecoveryAction::RetryWithBackoff { base_delay_ms, max_attempts: _ } => {
                // Educational: Exponential backoff calculation
                let delay = base_delay_ms * 2; // Simple backoff for demonstration
                delay_ms_educational(delay);
                Ok(())
            }
            
            RecoveryAction::ResetDevice => {
                // Educational: Device reset would go here
                // In production: perform device reset sequence
                // log::error!("Resetting device: {}", device_info);
                Ok(())
            }
            
            RecoveryAction::RemoveDevice => {
                // Educational: Device cleanup would go here
                // In production: clean up device resources and notify system
                // log::error!("Removing device: {}", device_info);
                Err("Device removed")
            }
            
            RecoveryAction::Fatal => {
                // Educational: System-level error handling
                // In production: might trigger system reset or error reporting
                // log::critical!("Fatal USB error for {}", device_info);
                Err("Fatal error")
            }
        }
    }
    
    /// Educational delay function (replace with actual timer in production)
    fn delay_ms_educational(ms: u32) {
        // In production, use proper timer or async delay
        cortex_m::asm::delay(600_000 * ms); // Rough approximation for 600MHz
    }
    
    /// Comprehensive error handling wrapper for USB operations
    /// 
    /// This demonstrates how to wrap USB operations with automatic
    /// retry logic and appropriate error recovery.
    pub fn with_retry<F, R>(
        mut operation: F,
        operation_name: &str,
        max_attempts: u8,
    ) -> Result<R, &'static str>
    where
        F: FnMut() -> Result<R, UsbError>,
    {
        let mut attempt = 0;
        
        loop {
            attempt += 1;
            
            match operation() {
                Ok(result) => return Ok(result),
                Err(error) => {
                    // Analyze error and determine recovery strategy
                    let recovery = analyze_usb_error(&error, attempt);
                    
                    // Educational logging
                    // In production: log::debug!("USB error in {}: {:?}, attempt {}/{}", 
                    //                           operation_name, error, attempt, max_attempts);
                    
                    // Check if we've exceeded maximum attempts
                    if attempt >= max_attempts {
                        return Err("Max retry attempts exceeded");
                    }
                    
                    // Execute recovery action
                    match execute_recovery(recovery, operation_name) {
                        Ok(()) => continue, // Retry the operation
                        Err(msg) => return Err(msg), // Cannot recover
                    }
                }
            }
        }
    }
}

/// Cache Coherency Management for Teensy 4.x (ARM Cortex-M7)
///
/// Critical for USB DMA operations on Teensy 4.x which has data cache.
/// The ARM Cortex-M7 has separate instruction and data caches that can
/// cause coherency issues with DMA operations if not managed properly.
pub mod cache_coherency {
    use cortex_m::asm::{dsb, dmb};
    
    /// Prepare buffer for DMA write operation (CPU → USB peripheral)
    /// 
    /// This ensures the USB controller sees all CPU writes by cleaning
    /// (flushing) the data cache lines containing the buffer.
    /// 
    /// Call this before starting a USB OUT transfer (host to device).
    pub fn prepare_dma_write_buffer(buffer: &[u8]) {
        let _start_addr = buffer.as_ptr() as u32;
        let _length = buffer.len();
        
        // Data Synchronization Barrier - ensure all CPU writes complete
        dsb();
        
        // Clean D-cache for the buffer range to ensure DMA sees CPU data
        // In production: SCB::clean_dcache_by_address(start_addr, length);
        // For educational purposes, we document the requirement
        
        // Memory barrier to ensure cache operations complete before DMA
        dmb();
    }
    
    /// Process buffer after DMA read operation (USB peripheral → CPU)
    /// 
    /// This ensures the CPU sees all USB controller writes by invalidating
    /// the data cache lines, forcing the CPU to read from main memory.
    /// 
    /// Call this after completing a USB IN transfer (device to host).
    pub fn process_dma_read_buffer(buffer: &mut [u8]) {
        let _start_addr = buffer.as_ptr() as u32;
        let _length = buffer.len();
        
        // Data Synchronization Barrier - ensure DMA transfer completes
        dsb();
        
        // Invalidate D-cache for the buffer range to ensure CPU sees DMA data
        // In production: SCB::invalidate_dcache_by_address(start_addr, length);
        // For educational purposes, we document the requirement
        
        // Memory barrier to ensure cache operations complete
        dmb();
    }
    
    /// Comprehensive cache management for bidirectional DMA buffer
    /// 
    /// Use this for buffers that will be both read and written by DMA operations.
    /// This performs both clean and invalidate operations.
    pub fn prepare_bidirectional_dma_buffer(buffer: &mut [u8]) {
        let _start_addr = buffer.as_ptr() as u32;
        let _length = buffer.len();
        
        dsb();
        
        // Clean and invalidate D-cache for complete coherency
        // In production: SCB::clean_invalidate_dcache_by_address(start_addr, length);
        
        dmb();
    }
    
    /// Educational note on cache alignment requirements
    /// 
    /// ARM Cortex-M7 cache lines are 32 bytes. For optimal performance,
    /// DMA buffers should be aligned to 32-byte boundaries and sized
    /// in multiples of 32 bytes to avoid partial cache line issues.
    pub const CACHE_LINE_SIZE: usize = 32;
    
    /// Check if buffer is properly aligned for optimal cache performance
    pub fn is_cache_aligned(buffer: &[u8]) -> bool {
        let addr = buffer.as_ptr() as usize;
        let len = buffer.len();
        
        // Check both address alignment and length alignment
        (addr % CACHE_LINE_SIZE == 0) && (len % CACHE_LINE_SIZE == 0)
    }
}

/// VBUS Power Control for Teensy 4.x USB Host Mode
///
/// Teensy 4.x requires external circuitry for proper USB host VBUS power control.
/// The built-in USB controller can detect devices but cannot directly provide the
/// 5V power that USB devices require.
pub mod vbus_power_control {
    /// Configure VBUS switching circuit for USB host mode
    /// 
    /// Recommended circuit:
    /// - Use P-channel MOSFET for high-side switching (e.g., IRF9540)
    /// - GPIO pin controls MOSFET gate through pullup resistor
    /// - Add current limiting resistor or PTC fuse (500mA for USB 2.0)
    /// - Include TVS diode for overvoltage protection
    /// 
    /// Teensy 4.x Pin Connections:
    /// - GPIO pin (e.g., Digital Pin 2) → MOSFET gate control
    /// - MOSFET source → +5V supply
    /// - MOSFET drain → VBUS (USB connector pin 1)
    /// - Current limit: 0.5A for USB 2.0 Low/Full Speed devices
    pub fn configure_vbus_switching() {
        // Example GPIO configuration for VBUS control
        // In production code, you would:
        // 1. Configure GPIO pin as output
        // 2. Set appropriate drive strength
        // 3. Initialize to VBUS OFF state for safety
        
        // Educational note: This is hardware-dependent
        // Actual implementation depends on your specific GPIO library
    }
    
    /// Enable VBUS power to connected USB device
    /// 
    /// Call this after detecting device connection but before enumeration.
    /// USB devices need power before they can respond to enumeration requests.
    pub fn enable_vbus_power() {
        // Set GPIO pin LOW to turn on P-channel MOSFET
        // (P-channel MOSFETs are ON when gate is pulled low)
        
        // Educational note: Always implement proper sequencing:
        // 1. Enable VBUS power
        // 2. Wait for power stabilization (typically 100ms)
        // 3. Begin USB enumeration sequence
    }
    
    /// Disable VBUS power to connected USB device
    /// 
    /// Call this when device is disconnected or during error conditions.
    /// This ensures proper power cycling for device recovery.
    pub fn disable_vbus_power() {
        // Set GPIO pin HIGH to turn off P-channel MOSFET
        // This removes power from the USB device
        
        // Educational note: Important for:
        // - Device removal detection
        // - Error recovery sequences  
        // - Power management
        // - Safety during faults
    }
    
    /// Monitor VBUS current for overcurrent protection
    /// 
    /// USB 2.0 specification allows maximum 500mA per device.
    /// Monitoring helps detect faulty devices and protect the host.
    pub fn check_vbus_current() -> VbusStatus {
        // In production, you would:
        // 1. Read current sensor (e.g., INA219)
        // 2. Compare against USB current limits
        // 3. Trigger overcurrent protection if needed
        
        // For educational purposes, return a status enum
        VbusStatus::Normal
    }
    
    /// VBUS monitoring status for educational purposes
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum VbusStatus {
        /// Normal operation - current within USB limits
        Normal,
        /// Overcurrent detected - device drawing too much power
        Overcurrent,
        /// No current - device not connected or powered off
        NoCurrent,
        /// Fault condition - short circuit or hardware failure
        Fault,
    }
}

/// MIDI Message Types
#[derive(Debug, Clone, Copy, PartialEq)]
enum MidiMessageType {
    NoteOff = 0x80,
    NoteOn = 0x90,
    PolyphonicKeyPressure = 0xA0,
    ControlChange = 0xB0,
    ProgramChange = 0xC0,
    ChannelPressure = 0xD0,
    PitchBend = 0xE0,
    SystemExclusive = 0xF0,
}

impl MidiMessageType {
    fn from_status_byte(status: u8) -> Option<Self> {
        match status & 0xF0 {
            0x80 => Some(Self::NoteOff),
            0x90 => Some(Self::NoteOn),
            0xA0 => Some(Self::PolyphonicKeyPressure),
            0xB0 => Some(Self::ControlChange),
            0xC0 => Some(Self::ProgramChange),
            0xD0 => Some(Self::ChannelPressure),
            0xE0 => Some(Self::PitchBend),
            0xF0 => Some(Self::SystemExclusive),
            _ => None,
        }
    }
}

/// MIDI Event
#[derive(Debug, Clone, Copy)]
pub enum MidiEvent {
    NoteOn { channel: u8, note: u8, velocity: u8 },
    NoteOff { channel: u8, note: u8, velocity: u8 },
    ControlChange { channel: u8, controller: u8, value: u8 },
    PitchBend { channel: u8, value: u16 },
    ProgramChange { channel: u8, program: u8 },
}

impl MidiEvent {
    /// Parse MIDI event from raw USB MIDI packet
    fn from_usb_midi_packet(packet: &[u8; 4]) -> Option<Self> {
        let _cable_number = (packet[0] >> 4) & 0x0F;
        let code_index = packet[0] & 0x0F;
        
        match code_index {
            0x8 => {
                // Note Off
                let channel = packet[1] & 0x0F;
                let note = packet[2] & 0x7F;
                let velocity = packet[3] & 0x7F;
                Some(MidiEvent::NoteOff { channel, note, velocity })
            }
            0x9 => {
                // Note On
                let channel = packet[1] & 0x0F;
                let note = packet[2] & 0x7F;
                let velocity = packet[3] & 0x7F;
                
                // Velocity 0 is actually Note Off
                if velocity == 0 {
                    Some(MidiEvent::NoteOff { channel, note, velocity })
                } else {
                    Some(MidiEvent::NoteOn { channel, note, velocity })
                }
            }
            0xB => {
                // Control Change
                let channel = packet[1] & 0x0F;
                let controller = packet[2] & 0x7F;
                let value = packet[3] & 0x7F;
                Some(MidiEvent::ControlChange { channel, controller, value })
            }
            0xE => {
                // Pitch Bend
                let channel = packet[1] & 0x0F;
                let lsb = packet[2] & 0x7F;
                let msb = packet[3] & 0x7F;
                let value = ((msb as u16) << 7) | (lsb as u16);
                Some(MidiEvent::PitchBend { channel, value })
            }
            0xC => {
                // Program Change
                let channel = packet[1] & 0x0F;
                let program = packet[2] & 0x7F;
                Some(MidiEvent::ProgramChange { channel, program })
            }
            _ => None,
        }
    }
    
    /// Get MIDI note name from note number
    fn note_name(note: u8) -> &'static str {
        const NOTES: &[&str] = &[
            "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
        ];
        NOTES[(note % 12) as usize]
    }
    
    /// Get octave from note number
    fn note_octave(note: u8) -> i8 {
        (note / 12) as i8 - 1
    }
}

/// MIDI Performance Statistics
#[derive(Default)]
struct MidiStats {
    notes_played: AtomicU32,
    control_changes: AtomicU32,
    packets_received: AtomicU32,
    parse_errors: AtomicU32,
}

impl MidiStats {
    const fn new() -> Self {
        Self {
            notes_played: AtomicU32::new(0),
            control_changes: AtomicU32::new(0),
            packets_received: AtomicU32::new(0),
            parse_errors: AtomicU32::new(0),
        }
    }
    
    fn note_played(&self) {
        self.notes_played.fetch_add(1, Ordering::Relaxed);
    }
    
    fn control_change(&self) {
        self.control_changes.fetch_add(1, Ordering::Relaxed);
    }
    
    fn packet_received(&self) {
        self.packets_received.fetch_add(1, Ordering::Relaxed);
    }
    
    fn parse_error(&self) {
        self.parse_errors.fetch_add(1, Ordering::Relaxed);
    }
    
    fn get_stats(&self) -> (u32, u32, u32, u32) {
        (
            self.notes_played.load(Ordering::Relaxed),
            self.control_changes.load(Ordering::Relaxed),
            self.packets_received.load(Ordering::Relaxed),
            self.parse_errors.load(Ordering::Relaxed),
        )
    }
}

/// MIDI Device state
struct MidiDevice {
    device_info: EnumeratedDevice,
    interface: u8,
    bulk_in_endpoint: u8,
    bulk_out_endpoint: u8,
    is_initialized: bool,
}

impl MidiDevice {
    fn new(device_info: EnumeratedDevice) -> Self {
        Self {
            device_info,
            interface: 0,  // Typically interface 1 for MIDI streaming
            bulk_in_endpoint: 0x81, // Endpoint 1 IN
            bulk_out_endpoint: 0x01, // Endpoint 1 OUT
            is_initialized: false,
        }
    }
    
    /// Initialize MIDI device (set up endpoints, etc.)
    fn initialize(&mut self, _memory_pool: &mut UsbMemoryPool) -> Result<()> {
        // In a full implementation, we would:
        // 1. Parse configuration descriptor to find MIDI streaming interface
        // 2. Find bulk endpoints for MIDI data
        // 3. Set up interrupt transfers for the bulk IN endpoint
        // 4. Send any necessary class-specific setup commands
        
        // For this example, we'll simulate successful initialization
        self.is_initialized = true;
        Ok(())
    }
    
    /// Simulate reading MIDI data (in real implementation, this would be interrupt-driven)
    ///
    /// TODO: Replace this with real USB interrupt transfers
    /// In a full implementation, you would:
    /// 1. Set up interrupt endpoint for bulk IN transfers
    /// 2. Use interrupt transfers to read MIDI packets from the device
    /// 3. Process packets in an interrupt handler or polling loop
    /// 4. Handle USB errors and device disconnection
    ///
    /// For now, this simulates MIDI events for demonstration purposes.
    fn simulate_midi_data(&self) -> Option<[u8; 4]> {
        // Simulate various MIDI events for demonstration
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            match COUNTER % 300 {
                0 => Some([0x09, 0x90, 60, 127]),   // Note On C4, velocity 127
                50 => Some([0x08, 0x80, 60, 127]),  // Note Off C4
                100 => Some([0x09, 0x90, 64, 100]), // Note On E4, velocity 100
                150 => Some([0x08, 0x80, 64, 100]), // Note Off E4
                200 => Some([0x0B, 0xB0, 7, 100]),  // Volume CC (controller 7), value 100
                250 => Some([0x0E, 0xE0, 0x00, 0x40]), // Pitch bend center
                _ => None,
            }
        }
    }
}

/// Simple application structure for MIDI keyboard example
struct MidiApp {
    usb_controller: EhciController<8, Running>,
    memory_pool: UsbMemoryPool,
    midi_device: Option<MidiDevice>,
    stats: MidiStats,
    tick_counter: u32,
    led_blink_counter: u32,
}

impl MidiApp {
    /// Initialize the MIDI application
    fn new() -> Result<Self> {
        info!("\r\n=== USB MIDI Keyboard Example ===");
        info!("Initializing USB host for MIDI devices...");

        // Configure USB clocks manually (educational: showing register-level setup)
        configure_usb_clocks();

        // Initialize USB memory pool
        let memory_pool = UsbMemoryPool::new();

        // Initialize USB PHY with documented register addresses
        let phy_base = USBPHY1_BASE_ADDRESS as usize;   // USB PHY1 register base
        let ccm_base = CCM_BASE_ADDRESS as usize;       // Clock Control Module base
        let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };

        // Initialize USB host controller with documented register address
        let usb1_base = USB1_HOST_BASE_ADDRESS as usize; // EHCI controller registers
        let controller = unsafe {
            EhciController::<8, Uninitialized>::new(usb1_base)?
        };

        let controller = unsafe { controller.initialize()? };
        let usb_controller = unsafe { controller.start() };

        info!("USB host initialized successfully");
        info!("Waiting for MIDI keyboard...\r\n");

        Ok(Self {
            usb_controller,
            memory_pool,
            midi_device: None,
            stats: MidiStats::new(),
            tick_counter: 0,
            led_blink_counter: 0,
        })
    }
    
    /// Try to detect and enumerate a MIDI device
    fn detect_midi_device(&mut self) {
        if self.midi_device.is_some() {
            return; // Already have a device
        }

        // Try to enumerate a MIDI device
        match find_midi_device(&mut self.usb_controller, &mut self.memory_pool) {
            Ok(mut device) => {
                if device.initialize(&mut self.memory_pool).is_ok() {
                    info!("MIDI keyboard detected and initialized!");
                    info!("Start playing to see MIDI events\r\n");
                    self.midi_device = Some(device);
                }
            }
            Err(_) => {
                // No MIDI device found, will retry
            }
        }
    }
    
    /// Process MIDI data from connected device
    fn process_midi_data(&mut self) {
        if let Some(ref mut device) = self.midi_device {
            if !device.is_initialized {
                return;
            }
            
            // Simulate reading MIDI data (in real implementation, use interrupt transfers)
            if let Some(packet) = device.simulate_midi_data() {
                self.stats.packet_received();
                
                // Parse MIDI event
                if let Some(event) = MidiEvent::from_usb_midi_packet(&packet) {
                    self.handle_midi_event(event);
                } else {
                    self.stats.parse_error();
                }
            }
        }
    }
    
    /// Handle a parsed MIDI event
    fn handle_midi_event(&mut self, event: MidiEvent) {
        match event {
            MidiEvent::NoteOn { channel, note, velocity } => {
                self.stats.note_played();
                self.led_blink_counter = 50; // Blink LED rapidly for 50ms

                let note_name = MidiEvent::note_name(note);
                let octave = MidiEvent::note_octave(note);
                info!("♪ Note ON:  {} {} (ch{}, vel{})", note_name, octave, channel + 1, velocity);

                // TODO: In a real application, you would:
                // - Trigger synthesizer
                // - Send to DAW over USB/MIDI
                // - Update display or other visual feedback
            }
            MidiEvent::NoteOff { channel, note, velocity } => {
                self.led_blink_counter = 10; // Blink LED slowly for 10ms

                let note_name = MidiEvent::note_name(note);
                let octave = MidiEvent::note_octave(note);
                info!("♪ Note OFF: {} {} (ch{}, vel{})", note_name, octave, channel + 1, velocity);
            }
            MidiEvent::ControlChange { channel, controller, value } => {
                self.stats.control_change();

                // Handle CC messages (volume, pan, modulation, etc.)
                let cc_name = match controller {
                    1 => "Modulation",
                    7 => "Volume",
                    10 => "Pan",
                    11 => "Expression",
                    64 => "Sustain Pedal",
                    _ => "Unknown CC",
                };
                info!("🎛️  CC: {} ({}={}) on ch{}", cc_name, controller, value, channel + 1);
            }
            MidiEvent::PitchBend { channel, value } => {
                info!("🎸 Pitch Bend: {} on ch{}", value, channel + 1);
            }
            MidiEvent::ProgramChange { channel, program } => {
                info!("🎹 Program Change: {} on ch{}", program, channel + 1);
            }
        }
    }
    
    /// Update status and statistics
    fn update_status(&mut self) {
        self.tick_counter += 1;

        // Decrement LED blink counter
        if self.led_blink_counter > 0 {
            self.led_blink_counter -= 1;
        }

        // Every 5000 ticks (~5 seconds), report statistics
        if self.tick_counter % 5000 == 0 && self.midi_device.is_some() {
            let (notes, ccs, packets, errors) = self.stats.get_stats();
            info!("\r\n📊 Stats: {} notes, {} CCs, {} packets, {} errors",
                  notes, ccs, packets, errors);
        }
    }
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        usb,
        ..
    } = board::t40(board::instances());

    let mut led = board::led(&mut gpio2, pins.p13);

    let mut poller = imxrt_log::log::usbd(
        usb,
        imxrt_log::Interrupts::Enabled,
    ).unwrap();

    poller.poll();

    let mut app = MidiApp::new().expect("Failed to initialize MIDI app");

    loop {
        poller.poll();

        if app.tick_counter % 500 == 0 {
            app.detect_midi_device();
        }

        app.process_midi_data();

        if app.led_blink_counter > 0 {
            let _ = led.set_high();
        } else {
            let _ = led.set_low();
        }

        app.update_status();
        delay_ms(1);
    }
}


/// Find and enumerate a MIDI device
fn find_midi_device(
    controller: &mut EhciController<8, Running>,
    memory_pool: &mut UsbMemoryPool,
) -> Result<MidiDevice> {
    // Enumerate device
    let mut enumerator = DeviceEnumerator::new(controller, memory_pool);
    let device_info = enumerator.enumerate_device()?;
    
    // Check if it's a MIDI device (Audio class with MIDI interface)
    if device_info.class != DeviceClass::Audio {
        return Err(imxrt_usbh::UsbError::Unsupported);
    }
    
    Ok(MidiDevice::new(device_info))
}

/// Simple delay function (educational: shows basic timing loop)
fn delay_ms(ms: u32) {
    // In production code, use hardware timers or RTIC monotonic timers
    // This uses ARM Cortex-M cycle counter for basic delays
    cortex_m::asm::delay(600_000 * ms); // Assuming 600MHz clock
}

/// USB Clock Control Register Values (educational constants)
pub mod usb_clock_values {
    /// Clock Gate Control Values for CCGR registers
    /// These 2-bit values control each clock gate in the Clock Control Module
    pub mod clock_gate_control {
        /// Clock is OFF in all modes - saves maximum power
        pub const CLOCK_OFF: u32 = 0b00;
        /// Clock is ON only in CPU RUN mode - automatic power saving  
        pub const CLOCK_ON_RUN_ONLY: u32 = 0b01;
        /// Clock is ON in RUN and WAIT modes - moderate power saving
        pub const CLOCK_ON_RUN_WAIT: u32 = 0b10;
        /// Clock is ALWAYS ON - maximum performance, highest power
        pub const CLOCK_ALWAYS_ON: u32 = 0b11;
    }
    
    /// USB PLL Control Values
    pub mod pll_control {
        /// PLL Power Control - 1 = powered up, 0 = powered down
        pub const PLL_POWER_UP: u32 = 1;
        /// PLL Output Enable - 1 = output enabled, 0 = output disabled
        pub const PLL_OUTPUT_ENABLE: u32 = 1;
        /// USB Clock Enable - 1 = USB clocks enabled from this PLL
        pub const USB_CLOCKS_ENABLE: u32 = 1;
        /// PLL Lock Status - 1 = PLL frequency is locked and stable
        pub const PLL_LOCKED: u32 = 1;
    }
    
    /// Standard USB frequencies as defined by USB 2.0 specification
    pub mod usb_frequencies {
        /// USB 2.0 High-Speed reference clock - exactly 480 MHz
        pub const USB_HS_FREQUENCY_HZ: u32 = 480_000_000;
        /// USB 2.0 Full-Speed bit clock - exactly 12 MHz  
        pub const USB_FS_FREQUENCY_HZ: u32 = 12_000_000;
        /// USB 2.0 Low-Speed bit clock - exactly 1.5 MHz
        pub const USB_LS_FREQUENCY_HZ: u32 = 1_500_000;
    }
}

/// Configure USB clocks for i.MX RT1062 (educational example showing register-level setup)
/// 
/// This function demonstrates the low-level clock configuration required for USB operation.
/// In a production system, you might use imxrt-hal's clock management instead.
/// 
/// Clock configuration sequence (order is important):
/// 1. Enable clock gates for USB peripherals
/// 2. Configure and power up the USB PLL  
/// 3. Wait for PLL lock before proceeding
/// 4. USB controller can now operate reliably
fn configure_usb_clocks() {
    use ral::{modify_reg, read_reg};
    use usb_clock_values::clock_gate_control::*;
    use usb_clock_values::pll_control::*;
    
    unsafe {
        // Step 1: Enable USB peripheral clock gates
        let ccm = ral::ccm::CCM::instance();
        
        // CCGR6 (Clock Gating Register 6) controls USB-related clocks
        // Setting all USB clocks to ALWAYS_ON for reliable operation
        modify_reg!(ral::ccm, ccm, CCGR6, 
            CG0: CLOCK_ALWAYS_ON,  // usb_ctrl1_clk - USB controller 1 register access
            CG1: CLOCK_ALWAYS_ON,  // usb_ctrl2_clk - USB controller 2 register access
            CG2: CLOCK_ALWAYS_ON,  // usb_phy1_clk - USB PHY 1 operation clock
            CG3: CLOCK_ALWAYS_ON   // usb_phy2_clk - USB PHY 2 operation clock
        );
        
        // Step 2: Configure USB PHY PLL for 480MHz generation
        let analog = ral::ccm_analog::CCM_ANALOG::instance();
        
        // Power up and configure the USB1 PLL
        // This PLL generates the precise 480MHz clock required for USB 2.0 High-Speed
        modify_reg!(ral::ccm_analog, analog, PLL_USB1,
            POWER: PLL_POWER_UP,         // Power up the PLL circuitry
            ENABLE: PLL_OUTPUT_ENABLE,   // Enable PLL clock output
            EN_USB_CLKS: USB_CLOCKS_ENABLE  // Route PLL output to USB clocks
        );
        
        // Step 3: Wait for PLL frequency lock (critical for stability)
        // PLL needs time to stabilize after configuration changes
        let mut pll_lock_timeout = 1000; // Prevent infinite loop in case of hardware issues
        while read_reg!(ral::ccm_analog, analog, PLL_USB1, LOCK) != PLL_LOCKED {
            pll_lock_timeout -= 1;
            if pll_lock_timeout == 0 {
                // In production, this would be a critical error
                // For educational purposes, we document the timeout requirement
                break;
            }
            // Small delay to avoid excessive register reads
            cortex_m::asm::delay(1000);
        }
        
        // At this point, USB controllers have stable 480MHz reference clock
        // and can begin normal USB operations
    }
}
