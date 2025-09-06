# Project Card: Teensy 4.x USB Host Driver (imxrt-usbh)

## Scope & Platform
- **Target boards:** Teensy 4.0 / 4.1 (i.MX RT1062, Cortex-M7 @ 600 MHz)
- **USB IP:** On-chip HS USB host (EHCI-compatible), with FS/LS device handling via HS hub + TT
- **Board power:** Teensy 4.1 host port requires VBUS power switching and over-current handling abstraction
- **Rust env:** `no_std`, `no_alloc` default; `alloc` feature optional. Build within the `imxrt-rs` ecosystem using Rust edition 2024
- **Executor:** Support RTIC v2 async (preferred) + blocking shim fallback

---

## Architecture & Safety
- **Layers:**
  1. HAL glue (clocks, USB PLL, pins, VBUS, IRQs)
  2. EHCI core (qTD/qH, async & periodic schedules, resets/recovery)
  3. Host-stack traits (aligned with `usbh` crate)
  4. Class drivers (HID keyboard, MSC read-only)

- **DMA/cache:** Unified `DmaBuffer` with MPU config or explicit clean/invalidate ops. Test by disabling cache maintenance to show data corruption risk.
- **Unsafe:** All `unsafe` must cite RM/app note sections + document invariants. Add `#![deny(unsafe_op_in_unsafe_fn)]`.
- **Interrupts:** ISRs acknowledge only; real work runs in bottom-half task.
- **Zero-copy:** Provide class drivers safe views into DMA descriptors/buffers.

---

## Process
- **Performance first:** All transfers must use DMA; CPU only for control plane.
- **2025 standards:** Use RTIC v2 async where possible; follow `imxrt-rs` project style and safety guidelines using Rust edition 2024.
- **Reviewers:** Require @code-reviewer, @teensy-expert, @embedded-systems, and @unsafe-auditor on all PRs.
- **Design doc required before code PRs** (covers clocks, PHY, schedules, cache policy, VBUS, error paths).
- **Each PR includes test plan + HIL checklist.**

---

## Milestones
1. Bring-up: clocks + EHCI reset + PortStatus read + SOF heartbeat
2. Control transfers: enumerate single HS device
3. HID keyboard demo (interrupt IN)
4. MSC read-only demo (SCSI INQUIRY/READ10)
5. Hub + FS device via TT
6. Power mgmt: suspend/resume + over-current
7. Perf pass: throughput benchmarks + CPU load validation

---

## Todo
- Implement the library per `imxrt-usbh` README, staged via milestones above
- Provide Embassy driver traits shim (or equivalent)
- Add VBUS power and over-current handling abstractions
- Ship HID + MSC (read-only) example class drivers
- Comparative notes doc on adapting `cotton-usb-host` patterns

---

## Guidance
- **EHCI core:** Implement schedules, qTD/qH pools, doorbell handling, port reset timing, error paths, with RM citations in comments
- **DMA:** Use allocator-free descriptor & buffer pool in non-cacheable memory (or guarded by cache ops)
- **Cotton ref:** Borrow architecture/testing from `cotton-usb-host` but document differences (EHCI vs RP2040 host)

---

## Testing & Validation
- **HIL test scripts:** Enumerate HS flash drive, FS keyboard via hub, CDC-ACM dongle; unplug during transfer; STALL/NAK handling; throughput benchmarks
- **Acceptance:** Control transfer retries, error recovery, MSC reads across 4 KB boundaries
- **Conformance:** Descriptor fuzzing with `proptest`
- **CI matrix:** `no-std/no-alloc`, `alloc`, `embassy-async`; run miri on host logic; nightly HIL smoke tests

---

## Developer UX
- **Examples:** `hid_keyboard_teensy41`, `msc_read_teensy41`, `hub_fs_keyboard`
- **Cargo features:** `embassy`, `alloc`, `class-hid`, `class-msc`, `hub`, `tracing-defmt`
- **Docs:** Pinout/power notes, VBUS requirements, cache/MPU policy, example wiring diagrams
- **Style:** `rustfmt` + `clippy pedantic`, crate root `#![forbid(unsafe_code)]` except scoped `allow`s

---

## Kickoff Prompt for Claude
> Produce a 2–4 page design doc for a Rust `no_std` i.MX RT1062 USB **host** driver targeting Teensy 4.0/4.1 and following the `cotton-usb-host` implementation. Cover: (1) clocks/USB PLL/PHY init; (2) EHCI schedules and qH/qTD lifetimes; (3) cache/MPU policy with code sketches; (4) VBUS power/OC handling; (5) control transfer state machine + retries; (6) error handling/recovery; (7) ISR vs bottom-half split; (8) FS device strategy via HS hub/TT; (9) crate features, examples, CI; (10) risks and mitigations. Cite relevant i.MX RT1060 RM + AN12042 directly.
