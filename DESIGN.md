# imxrt-usbh Design Document
## Rust `no_std` USB Host Driver for i.MX RT1062 (Teensy 4.0/4.1)

### 1. Clock/USB PLL/PHY Initialization

The i.MX RT1062 USB host controller requires precise clock configuration per RM sections 14.4-14.5:

```rust
// USB1 PLL (480 MHz) configuration - RM 14.5.1
const USB1_PLL_DIV_SELECT: u32 = 20;  // 480MHz from 24MHz OSC
const USB1_PLL_ENABLE: u32 = 1 << 13;
const USB1_PLL_POWER: u32 = 1 << 12;
const USB1_PLL_EN_USB_CLKS: u32 = 1 << 6;

// PHY initialization sequence - RM 66.5.1
fn init_usb_phy(ccm: &CCM, usb_phy: &USBPHY1) {
    // 1. Enable USB clocks via CCM_CCGR6[CG0] - RM 14.7.21
    ccm.ccgr6.modify(|_, w| w.cg0().bits(0b11));
    
    // 2. Configure USB1 PLL for 480MHz
    ccm.analog_pll_usb1.modify(|_, w| {
        w.div_select().bits(USB1_PLL_DIV_SELECT)
         .enable().set_bit()
         .power().set_bit()
         .en_usb_clks().set_bit()
    });
    
    // 3. Wait for PLL lock (typ. 50us) - RM 14.5.1.3
    while !ccm.analog_pll_usb1.read().lock().bit() {}
    
    // 4. PHY power-on reset per AN12042 section 3.1
    usb_phy.ctrl.modify(|_, w| w.sftrst().set_bit());
    cortex_m::asm::delay(10_000); // 10us minimum
    usb_phy.ctrl.modify(|_, w| w.sftrst().clear_bit());
    
    // 5. Configure host mode - RM 66.6.33
    usb_phy.ctrl.modify(|_, w| {
        w.endevplugindet().clear_bit()  // Disable device plug-in
         .hostdiscondetect_irq().set_bit() // Enable disconnect detect
    });
}
```

### 2. EHCI Schedules and qH/qTD Lifetime Management

EHCI uses Queue Heads (qH) and Queue Transfer Descriptors (qTD) per EHCI spec 3.5-3.6:

```rust
#[repr(C, align(32))]
struct QueueHead {
    horizontal_link: AtomicU32,    // Next qH in async list
    endpoint_chars: u32,           // Speed, addr, endpoint, max packet
    endpoint_caps: u32,             // Mult, port, hub addr for split
    current_qtd: AtomicU32,        // Currently executing qTD
    next_qtd: AtomicU32,           // Next qTD to execute
    alt_next_qtd: AtomicU32,       // Alternate on short packet
    token: AtomicU32,              // Status and control
    buffer_pointers: [AtomicU32; 5], // Data buffer addresses
    // ... overlay area continues
}

#[repr(C, align(32))]
struct QueueTD {
    next_qtd: u32,
    alt_next_qtd: u32,
    token: u32,                    // PID, status, length
    buffer_pointers: [u32; 5],     // Up to 20KB data
    // Extended fields for 64-bit addressing (unused on RT1062)
}

// Lifetime management using indices instead of pointers
struct QhPool {
    qheads: [QueueHead; 64],       // Static pool in DMA memory
    free_list: heapless::Vec<u8, 64>, // Free indices
}

impl QhPool {
    fn alloc(&mut self) -> Option<QhIndex> {
        self.free_list.pop().map(QhIndex)
    }
    
    fn free(&mut self, idx: QhIndex) {
        // Clear qH state before returning to pool
        self.qheads[idx.0 as usize] = QueueHead::default();
        self.free_list.push(idx.0).ok();
    }
}
```

### 3. Cache/MPU Policy

The Cortex-M7 D-cache requires careful management for DMA coherency (RM 3.3.3, ARM TRM):

```rust
// Option A: MPU-based non-cacheable region (preferred)
#[link_section = ".dma_ram"]  // Linker places in DTCM or OCRAM
#[repr(C, align(4096))]
struct DmaRegion {
    qh_pool: [QueueHead; 64],
    qtd_pool: [QueueTD; 256],
    data_buffers: [[u8; 512]; 32],
}

fn configure_mpu_for_dma(region_addr: usize) {
    const MPU_RASR_TEX_DEVICE: u32 = 0b010 << 19; // Device memory
    const MPU_RASR_SHARED: u32 = 1 << 18;
    const MPU_RASR_BUFFERABLE: u32 = 1 << 16;
    
    unsafe {
        // Region 7: DMA descriptors as Device memory
        cortex_m::peripheral::MPU::ptr().write(|mpu| {
            mpu.rnr.write(7);
            mpu.rbar.write(region_addr as u32);
            mpu.rasr.write(
                MPU_RASR_ENABLE |
                MPU_RASR_SIZE_16KB |
                MPU_RASR_TEX_DEVICE |
                MPU_RASR_SHARED |
                MPU_RASR_BUFFERABLE
            );
        });
    }
}

// Option B: Explicit cache maintenance (fallback)
fn dma_write_back(addr: usize, size: usize) {
    cortex_m::asm::dsb();
    // Clean D-cache by address - ARM Cortex-M7 TRM 2.5.7
    let dcache_line_size = 32;
    let start = addr & !(dcache_line_size - 1);
    let end = (addr + size + dcache_line_size - 1) & !(dcache_line_size - 1);
    
    for line_addr in (start..end).step_by(dcache_line_size) {
        unsafe {
            cortex_m::asm::dccmvac(line_addr as *const u32);
        }
    }
    cortex_m::asm::dsb();
}
```

### 4. VBUS Power and Over-Current Handling

Teensy 4.1 requires external VBUS control (5V @ 500mA) - hardware-specific:

```rust
trait VbusPowerControl {
    async fn enable_vbus(&mut self) -> Result<(), VbusError>;
    async fn disable_vbus(&mut self);
    fn is_overcurrent(&self) -> bool;
}

struct Teensy41VbusControl<P1: OutputPin, P2: InputPin> {
    enable_pin: P1,      // GPIO to VBUS switch enable
    oc_pin: P2,         // Over-current detect input
}

impl<P1: OutputPin, P2: InputPin> VbusPowerControl for Teensy41VbusControl<P1, P2> {
    async fn enable_vbus(&mut self) -> Result<(), VbusError> {
        self.enable_pin.set_high().ok();
        
        // Wait 100ms for VBUS rise time (USB 2.0 spec 7.1.2.1)
        embassy_time::Timer::after_millis(100).await;
        
        if self.oc_pin.is_low().unwrap_or(false) {
            self.enable_pin.set_low().ok();
            return Err(VbusError::OverCurrent);
        }
        Ok(())
    }
}
```

### 5. Control Transfer State Machine

SETUP/DATA/STATUS stages with automatic retry per USB 2.0 spec 8.5.3:

```rust
enum ControlState {
    Setup { attempt: u8 },
    DataIn { remaining: usize, attempt: u8 },
    DataOut { remaining: usize, attempt: u8 },
    Status { attempt: u8 },
    Complete,
    Failed(UsbError),
}

async fn execute_control_transfer(
    qh: &mut QueueHead,
    setup: SetupPacket,
    data: Option<&mut [u8]>,
) -> Result<(), UsbError> {
    const MAX_RETRIES: u8 = 3;
    let mut state = ControlState::Setup { attempt: 0 };
    
    loop {
        match state {
            ControlState::Setup { attempt } => {
                let qtd = build_setup_qtd(&setup);
                qh.queue_qtd(qtd);
                
                match wait_qtd_complete(&qtd).await {
                    Ok(_) => {
                        state = if setup.length > 0 {
                            if setup.is_in() {
                                ControlState::DataIn { remaining: setup.length as usize, attempt: 0 }
                            } else {
                                ControlState::DataOut { remaining: setup.length as usize, attempt: 0 }
                            }
                        } else {
                            ControlState::Status { attempt: 0 }
                        };
                    }
                    Err(e) if attempt < MAX_RETRIES => {
                        state = ControlState::Setup { attempt: attempt + 1 };
                        embassy_time::Timer::after_millis(10).await;
                    }
                    Err(e) => state = ControlState::Failed(e),
                }
            }
            // DATA and STATUS stages follow similar pattern
            ControlState::Complete => return Ok(()),
            ControlState::Failed(e) => return Err(e),
            _ => { /* ... */ }
        }
    }
}
```

### 6. Error Handling and Recovery

EHCI error conditions from RM 66.6.16 (USBSTS) and recovery procedures:

```rust
#[derive(Debug)]
enum EhciError {
    Stall,          // Device returned STALL
    Babble,         // Device sent too much data  
    XactError,      // CRC, timeout, bad PID
    MissedMicroframe,
    HostSystemError, // DMA error
}

async fn handle_qtd_error(qtd: &QueueTD, qh: &mut QueueHead) -> Result<(), EhciError> {
    let token = qtd.token;
    
    if token & QTD_TOKEN_HALTED != 0 {
        // Check specific error bits
        if token & QTD_TOKEN_BABBLE != 0 {
            // Babble: reset endpoint per EHCI 4.10.2
            reset_endpoint(qh).await?;
            return Err(EhciError::Babble);
        }
        if token & QTD_TOKEN_XACT_ERROR != 0 {
            // Transaction error: retry up to 3 times
            if (token & QTD_TOKEN_CERR_MASK) >> QTD_TOKEN_CERR_SHIFT > 0 {
                // Hardware will retry automatically
                return Ok(());
            }
            return Err(EhciError::XactError);
        }
    }
    Ok(())
}

async fn reset_endpoint(qh: &mut QueueHead) -> Result<(), EhciError> {
    // 1. Remove qH from schedule
    unlink_qh(qh);
    
    // 2. Clear halt condition
    qh.token.fetch_and(!QH_TOKEN_HALTED, Ordering::Release);
    
    // 3. Restart with fresh qTD
    qh.current_qtd.store(QTD_TERMINATE, Ordering::Release);
    
    // 4. Re-insert into schedule
    link_qh(qh);
    
    Ok(())
}
```

### 7. ISR vs Bottom-Half Split

Minimal ISR with async task processing per RM 66.6.16:

```rust
static USB_EVENT_CHANNEL: Channel<ThreadModeRawMutex, UsbEvent, 32> = Channel::new();

#[interrupt]
fn USB_OTG1() {
    let usbsts = unsafe { &*USB1::ptr() }.usbsts.read();
    
    // Acknowledge interrupts immediately
    unsafe { &*USB1::ptr() }.usbsts.write(|w| w.bits(usbsts.bits())); 
    
    // Send events to async task - non-blocking try_send
    if usbsts.ui().bit() {  // USB interrupt (transaction complete)
        let _ = USB_EVENT_CHANNEL.try_send(UsbEvent::TransferComplete);
    }
    if usbsts.pci().bit() { // Port change detect
        let _ = USB_EVENT_CHANNEL.try_send(UsbEvent::PortChange);
    }
    if usbsts.uei().bit() { // USB error
        let _ = USB_EVENT_CHANNEL.try_send(UsbEvent::Error);
    }
}

#[embassy_executor::task]
async fn usb_bottom_half() {
    loop {
        let event = USB_EVENT_CHANNEL.receive().await;
        match event {
            UsbEvent::TransferComplete => {
                // Scan async schedule for completed qTDs
                scan_async_schedule().await;
            }
            UsbEvent::PortChange => {
                // Handle connect/disconnect
                handle_port_change().await;
            }
            UsbEvent::Error => {
                // Error recovery
                handle_usb_error().await;
            }
        }
    }
}
```

### 8. Full-Speed Device Support via Transaction Translator

FS/LS devices behind HS hub require split transactions (EHCI spec 4.12):

```rust
struct SplitTransaction {
    hub_addr: u8,
    hub_port: u8,
    start_mask: u8,     // Microframes for start-split
    complete_mask: u8,  // Microframes for complete-split
}

fn configure_split_qh(qh: &mut QueueHead, split: &SplitTransaction) {
    // Set split transaction bits in endpoint_caps - EHCI 3.6.2
    qh.endpoint_caps = 
        (split.hub_addr as u32) << 16 |  // Hub address
        (split.hub_port as u32) << 23 |  // Port number
        (split.complete_mask as u32) << 8 | // Complete-split mask
        (split.start_mask as u32);        // Start-split mask
    
    // Mark as FS device behind TT
    qh.endpoint_chars |= QH_EPS_FULL_SPEED | QH_CONTROL_ENDPOINT;
}

async fn schedule_fs_interrupt(
    device: &FsDevice,
    endpoint: u8,
    data: &mut [u8],
) -> Result<(), UsbError> {
    let split = SplitTransaction {
        hub_addr: device.hub_address,
        hub_port: device.hub_port,
        start_mask: 0x01,      // Microframe 0 for start
        complete_mask: 0x1C,   // Microframes 2,3,4 for complete
    };
    
    let mut qh = allocate_periodic_qh()?;
    configure_split_qh(&mut qh, &split);
    
    // Execute split transaction
    execute_split_transfer(&mut qh, data).await
}
```

### 9. Crate Features and Examples

```toml
[features]
default = ["defmt"]
embassy = ["embassy-executor", "embassy-time", "embassy-usb-driver"]
alloc = []
class-hid = []
class-msc = []
hub = []
defmt = ["dep:defmt"]
tracing = ["dep:tracing"]

[examples]
[[example]]
name = "hid_keyboard_teensy41"
required-features = ["embassy", "class-hid"]

[[example]]
name = "msc_read_teensy41"  
required-features = ["embassy", "class-msc"]

[[example]]
name = "hub_fs_keyboard"
required-features = ["embassy", "hub", "class-hid"]
```

CI matrix configuration:
```yaml
jobs:
  test:
    strategy:
      matrix:
        features: 
          - ""  # no_std, no_alloc
          - "alloc"
          - "embassy,class-hid"
          - "embassy,class-msc,hub"
    steps:
      - run: cargo build --features ${{ matrix.features }}
      - run: cargo test --features ${{ matrix.features }}
      - run: cargo miri test --features ${{ matrix.features }}
```

### 10. Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| **Cache coherency bugs** | Data corruption, enumeration failures | MPU configuration for non-cacheable DMA region; test suite with cache disabled to expose bugs |
| **qTD pool exhaustion** | Transfer failures under load | Static sizing based on worst-case; runtime metrics; graceful degradation |
| **FS device timing** | Split transaction failures | Follow EHCI spec 11.18 precisely; test with USB analyzer |
| **VBUS overcurrent** | Hardware damage | Hardware current limiting; software debounce; immediate shutdown on OC detect |
| **Interrupt latency** | Missed microframes | Minimal ISR (< 10μs); async task for processing; NVIC priority configuration |
| **Memory safety in unsafe blocks** | UB, crashes | `#![deny(unsafe_op_in_unsafe_fn)]`; miri testing; documented invariants with RM citations |
| **Hub cascading** | Bandwidth calculation errors | Limit hub depth to 2; pre-calculate bandwidth allocation |

### References
- i.MX RT1060 Reference Manual Rev. 3 (IMXRT1060RM)
- AN12042: "Using the i.MX RT L1 Cache"
- EHCI Specification 1.0
- USB 2.0 Specification
- ARM Cortex-M7 Technical Reference Manual
- cotton-usb-host architecture (adaptation reference)