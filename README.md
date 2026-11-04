# sensor-hub-firmware

Firmware core for an IMU sensor hub, written in C++17 with no heap
allocation, plus a host-side simulator that runs and tests it on a laptop.

The firmware reads a 6-axis IMU at 1 kHz, fuses it into roll and pitch with a
fixed-point (Q15) complementary filter, and streams telemetry over a framed,
CRC-checked, retransmitting serial protocol. A watchdog and a fault state
machine handle hung code, stuck sensors and brownouts. Firmware updates use
A/B slots with CRC verification, trial boots and rollback, and survive power
loss at any flash operation.

**Status: host simulator only.** Everything runs against a simulated HAL.
There is no port to a real microcontroller yet. See [Limitations](#limitations-and-unfinished-work).

## Architecture

```
 +----------------------------- host simulator (sim/) -----------------------------+
 |  SimHal: virtual or real-time clock | UART byte stream | 1 kHz IMU motion model  |
 |          NOR flash with power-cut injection | watchdog | supply voltage | retained RAM
 +-------+---------------+-----------------+----------------+-------------+---------+
         | 1 kHz tick    | UART RX ISR     | imu_read()     | flash ops   | wdt kick,
         | ISR           | (one byte/call) |                |             | supply_mv
 ========+===============+=================+================+=============+========== HAL
         |               v                 |                |             |  include/shf/hal.hpp
         |      SpscRing<uint8_t,256>      |                |             |
         |      (lock-free, ISR -> loop)   |                |             |
 +-------+---------------+-----------------+----------------+-------------+---------+
 | Firmware core (src/, no heap, no exceptions)                                      |
 |                                                                                   |
 |  Scheduler: fixed priority, cooperative, release-jitter histogram                 |
 |   p0 imu       1 kHz  -> FaultManager (frozen / no data) -> ComplementaryFilter Q15
 |   p1 comms     1 kHz  -> Deframer (COBS, 2x CRC-16) -> LinkEndpoint (seq/ACK/NAK)  |
 |                          -> message handler -> OtaManager (A/B slots, CRC-32)     |
 |   p2 telemetry 50 Hz  -> TX queue -> LinkEndpoint -> UART                         |
 |   p3 health    100 Hz -> FaultManager (brownout, safe mode) -> WatchdogMonitor    |
 |                                                                                   |
 |  Bootloader (bootloader_run): ping-pong boot control block, CRC check of the      |
 |  chosen slot, trial-boot countdown, rollback, fallback to the other slot          |
 +-----------------------------------------------------------------------------------+
```

Fault state machine:

```
             sensor frozen / no data              5 IMU resets failed
   NORMAL ---------------------------> DEGRADED -----------------------> SAFE_MODE
    ^  ^       healthy data for 50 ms     |                                ^    |
    |  +----------------------------------+                                |    | CLEAR_FAULT
    |                                                  3 watchdog resets   |    | command
    |  supply < 3.0 V (3 readings)                     in a row, at boot --+    v
    +---- BROWNOUT <---- NORMAL / DEGRADED                                    NORMAL
     supply >= 3.1 V for 100 ms          (flash writes blocked while in brownout)
```

Flash layout (4 KiB sectors):

```
 0x00000  boot control block, record 0   } ping-pong, generation counter + CRC-32
 0x01000  boot control block, record 1   }
 0x02000  slot A: [32 B header][image body, up to 64 KiB - 32 B]
 0x12000  slot B: [32 B header][image body, up to 64 KiB - 32 B]
```

## Design

| Area | Choice | Why |
|---|---|---|
| Memory | All state in one statically allocated `Firmware` object; fixed-size buffers; `make test` fails if the core objects reference `new`/`malloc`/`free`, exceptions or static-init guards | Bounded RAM, no fragmentation, footprint known at build time |
| HAL | Abstract `Hal` class: timer + tick ISR, UART RX ISR + write, IMU read/reset, NOR flash, supply voltage, watchdog, retained RAM, reset | One seam between firmware and hardware; the simulator plugs in behind it |
| Scheduler | Fixed-priority, cooperative, run-to-completion periodic tasks. Releases are anchored to the schedule (no drift); missed releases are skipped and counted | Simple and deterministic. Jitter is measured as start time minus release time, into a 1 us histogram |
| ISR to main loop | Lock-free SPSC ring with free-running 32-bit indices and acquire/release ordering; the producer drops and counts on overflow | Safe between one interrupt and the main loop without disabling interrupts |
| Framing | COBS with a 0x00 delimiter on both sides of each frame | Any byte loss or corruption resynchronizes at the next delimiter |
| Integrity | Two CRC-16s over header and payload: CRC-16/CCITT-FALSE (0x1021) and CRC-16/T10-DIF (0x8BB7) | One CRC-16 lets about 1 in 65536 heavily damaged frames through. At a 1% bit-error rate the bench measured 22 such frames across 1M packets. The pair acts like a 32-bit check |
| Reliability | Stop-and-wait ARQ per direction, 8-bit sequence numbers, ACK on accept or duplicate, rate-limited NAK on damage, retransmit timer started after the frame has left the UART, SYN flag to resynchronize after a reset or a give-up | Small and easy to verify. Throughput is the cost (see bench) |
| Fusion | Complementary filter, alpha = 0.98 in Q15. CORDIC atan2 and integer sqrt for the accelerometer angles. State kept as a 32-bit binary angle, output as a Q15 angle (32768 = 180 deg) | A Q15-only state would round away slow rotations (1 deg/s is 0.001 deg per sample, below the 0.0055 deg Q15 step). Binary angles wrap at +/-180 deg for free |
| Watchdog | Hardware watchdog (100 ms) kicked by the health task only when every required task has checked in since the last kick | Catches a hung loop and a single stalled task |
| Faults | Frozen sensor (50 identical samples) or no data (20 ms): IMU reset every 100 ms, up to 5 tries, then SAFE_MODE. Brownout with debounce and hysteresis. 3 consecutive watchdog resets: boot into SAFE_MODE | SAFE_MODE keeps the link and the watchdog alive so the device can still be inspected and updated |
| OTA | A/B slots. The running slot is never written. Body first, header last. CRC-32 checked after download and on every boot. The boot control block is written to the older of two records. New images get 2 trial boots and must confirm after 2 s of healthy running, otherwise the bootloader rolls back | Any single torn erase or write leaves a bootable image |

## Commands

Requires clang with C++17 and GNU make. Tested with clang 21 on macOS arm64.

```sh
make test          # unit + fuzz + integration tests (ASan + UBSan), ring tests under TSan, no-heap check
make bench         # host-simulator bench (about 20 s) + footprint estimate
make footprint     # footprint estimate only
make check-noheap  # fails if the firmware core references heap or exception symbols
make clean

./build/asan/shf_tests ota   # run only tests whose name contains "ota"
```

## Tests

`make test` builds three variants: the tests with AddressSanitizer and
UndefinedBehaviorSanitizer, the ring buffer tests with ThreadSanitizer, and
the firmware core alone at `-Os -fno-exceptions -fno-rtti` for the no-heap
check. 28 tests plus 3 under TSan, about 8 s from a clean build.

| Area | What is checked |
|---|---|
| Ring buffer | FIFO order and full/empty behavior; 2M items through a 64-slot ring between two threads with no loss, reordering or torn reads; an ISR-style producer that drops on overflow (received + dropped = produced) |
| Protocol | CRC check values; COBS round trips; every 1-bit and 2-bit error and 200k random 3-bit errors on a frame are rejected; 2M random bytes produce no accepted frame; full-duplex fuzzing with bit errors up to 3%, frame drops and duplicates up to 30% and garbage bursts. Every payload carries its index, so the checker separates corruption, duplication, reordering and loss: no corrupted, duplicated or reordered payload is ever delivered; at the lower noise levels every payload arrives; resynchronization after the sender gives up |
| Fusion | CORDIC atan2 error < 2e-6 rad; integer sqrt exact; fixed-point filter within 0.01 deg of a double-precision reference over 60 s in four motion scenarios, including continuous rotation through +/-180 deg |
| Watchdog and faults | Hung main loop and a single stalled task both cause a watchdog reset and a clean reboot; 3 resets in a row boot into SAFE_MODE, which stays up and clears on command; the reset counter clears after 10 s of normal running; frozen and dead IMU; brownout entry, hysteresis and hold time; brownout aborts an OTA in progress |
| OTA | Clean A to B and B to A updates; bad CRC and bad arguments rejected; fallback when both boot records or the active slot are corrupt; a power cut at **every** flash operation of an update (177 runs); 200 chained updates with random repeated cuts; 100 images that fail their self-test all roll back. After every cut the bootloader must find an image whose CRC verifies |
| Integration | The host sends a 10 KB image over the simulated UART with bit errors, the device reboots into it on trial, confirms it, and reports the new version in telemetry. Then a build that hangs on boot is rolled back after 2 watchdog resets |

## Bench results (host-simulator sanity runs)

These numbers come from the simulator on a laptop (arm64, macOS), not from
a microcontroller. The jitter section uses the real host clock with a host
thread acting as the UART interrupt, so it reflects macOS scheduling and a
1 us timer, not MCU interrupt latency. The protocol and OTA sections run in
virtual time and are deterministic. The footprint is a static estimate from
host arm64 object files.

Exact output of `make bench` (build lines removed):

```
sensor-hub-firmware bench: HOST-SIMULATOR SANITY RUN (simulated HAL on a laptop, not an MCU)

== Scheduler loop jitter (real-time host clock)
run: 10.0 s, 10000 releases of the 1 kHz imu/fusion task (probe)
imu task release jitter: p50 0 us, p99 0 us, max 51 us (mean 0.04 us, 1 us timer resolution)
  task imu       runs   10301  max jitter    51 us  max exec   22 us  overruns 0
  task comms     runs   10301  max jitter    58 us  max exec    6 us  overruns 0
  task telemetry runs     516  max jitter    14 us  max exec    4 us  overruns 0
  task health    runs    1031  max jitter    15 us  max exec    3 us  overruns 0
uart traffic: host->device 515 commands, device->host 1030 messages, rx ring overflows 0
state NORMAL, watchdog resets 0

== Protocol goodput (virtual time, 115200 baud, stop-and-wait ARQ, 3 ms ACK timeout)
BER 0e+00, 100000 packets x 16 B payload:
  delivered 100000, corrupted accepted 0, duplicated/reordered 0, abandoned 0
  goodput 5109 B/s over 313 s simulated = 44.4% of the 11494 B/s line rate
  data frames sent 100000, retransmissions 0 = 0.00 per packet, 0.0% of data frames (0 on NAK, 0 on timeout)
  frames damaged in transit 0 (data 0, ack/nak 0)
  damaged frames a CRC-16/CCITT-only check would have accepted: 0 (dual CRC-16 accepted 0)
BER 1e-03, 100000 packets x 16 B payload:
  delivered 100000, corrupted accepted 0, duplicated/reordered 0, abandoned 0
  goodput 3583 B/s over 447 s simulated = 31.2% of the 11494 B/s line rate
  data frames sent 134174, retransmissions 34174 = 0.34 per packet, 25.5% of data frames (21439 on NAK, 12735 on timeout)
  frames damaged in transit 35523 (data 25235, ack/nak 10288)
  damaged frames a CRC-16/CCITT-only check would have accepted: 0 (dual CRC-16 accepted 0)
BER 1e-02, 1000000 packets x 16 B payload:
  delivered 1000000, corrupted accepted 0, duplicated/reordered 0, abandoned 0
  goodput 184 B/s over 86754 s simulated = 1.6% of the 11494 B/s line rate
  data frames sent 18084315, retransmissions 17084315 = 17.08 per packet, 94.5% of data frames (2955153 on NAK, 14129162 on timeout)
  frames damaged in transit 25225688 (data 15850607, ack/nak 9375081)
  damaged frames a CRC-16/CCITT-only check would have accepted: 22 (dual CRC-16 accepted 0)
  (wall time 7.7 s)

== OTA power-loss recovery (500 interrupted updates)
updates: 400 good images, 100 images that fail their self-test; sizes 2-60 KB
power cuts injected: 666 (one at a uniformly random flash op per update, plus 25% chance of another on each retry)
first cut landed in: slot erase 5, image body write 482, image header write 3, boot-control erase 3, boot-control write 3
boot right after the first cut: previous image 495, new image (trial) 5
boots checked: 1366, boots without a valid image: 0
good images running and confirmed after recovery: 400/400
bad images rolled back to the previous version: 100/100
unexpected OTA errors: 0, NOR write violations: 0, mean flash ops per clean update: 549

== Static RAM footprint (sizeof, host build)
Firmware instance (everything below + glue): 4352 B
  SpscRing<uint8_t,256> UART RX:  384 B (64-byte aligned counters on the host; build with -DSHF_RING_ALIGN=4 on an MCU)
  LinkEndpoint:                   400 B
  Scheduler (8 tasks + histogram): 2608 B (jitter histogram 2064 B)
  ComplementaryFilter:            32 B
  FaultManager:                   144 B
  OtaManager:                     56 B

== Firmware core footprint (static estimate from build/fw/src/*.o, host arm64 -Os)
  object             code   rodata     data      bss
  cobs.o              340        0        0        0
  crc.o               152     2048        0        0
  fault.o            1148       77        0        0
  firmware.o         3160       91        0        0
  fusion.o            624       96        0        0
  ota.o              2800      117        0        0
  protocol.o         2232        0        0        0
  scheduler.o         800        0        0        0
  total             11256     2429        0        0
flash estimate (code + rodata + data): 13685 B
static RAM in .data/.bss: 0 B (firmware state lives in the Firmware instance, see sizeof above)
deepest single stack frames (-fstack-usage; call-chain depth not analyzed):
     208 B  _ZN3shf10OtaManager6finishEv
     192 B  _ZN3shf8Deframer6finishERNS_5FrameE
     192 B  _ZN3shf11verify_slotERNS_3HalERKNS_11FlashLayoutEhPNS_11ImageHeaderE
     144 B  _ZN3shf12encode_frameERKNS_5FrameEPhm
     128 B  _ZN3shf8Firmware10comms_taskEPvy
```

Notes on reading it:

- **Jitter.** p50 and p99 are 0 us because the 1 kHz IMU task has the
  highest priority and usually finds the loop idle at its release time. The
  maximum (tens of us) occurs when a lower-priority task is still running at
  the release, plus host OS preemption.
- **Goodput.** At BER 0, stop-and-wait reaches 44% of line rate: a 26-byte
  data frame and a 10-byte ACK cross the wire for every 16 payload bytes. At
  a 1% bit-error rate a 208-bit data frame arrives intact with probability
  0.99^208 = 12% (the bench counted 12.4%) and an 80-bit ACK with 45%, so
  each packet needs about 18 transmissions and goodput drops to 1.6% of
  line rate. All 1M packets were still delivered in order with no corrupted
  payload accepted.
- **OTA.** Cuts are placed uniformly over flash operations, so most land in
  body writes. The exhaustive per-operation sweep in `make test` covers the
  rarer header and boot-control writes.

## Layout

```
include/shf/   firmware core headers (HAL, ring buffer, CRC, COBS, protocol,
               scheduler, fusion, faults, OTA, firmware app)
src/           firmware core sources (no heap, no exceptions, no RTTI)
sim/           host simulator: SimHal, device harness with reset handling,
               noisy link model, host peer, OTA power-loss rig
tests/         test framework and tests
bench/         bench
tools/         footprint script
```

## Limitations and unfinished work

- **No real MCU target yet.** There is no startup code, linker script,
  vector table, clock setup or peripheral drivers. The HAL is implemented
  only by the simulator.
- **Simulated HAL.** The IMU is a motion model with Gaussian noise and no
  linear acceleration. The flash model tears operations as described in
  `sim/sim_hal.cpp` but does not model wear, read disturb or real erase
  timing. In virtual-time tests, tasks take zero time.
- **Host jitter is not MCU jitter.** The jitter numbers show the
  scheduler's behavior on macOS with a 1 us timer. They say nothing about
  interrupt latency or cycle counts on a Cortex-M.
- **Footprint is an estimate.** Code size comes from host arm64 `-Os`
  objects; Thumb-2 code is usually smaller. RAM is `sizeof(Firmware)`
  (4.3 KB on the host, including 64-byte alignment padding in the rings and
  a 2 KB jitter histogram). Stack usage is reported per function only;
  call-chain depth is not analyzed.
- **Ring buffer concurrency** is tested with two host threads under TSan,
  which checks the C++ memory model usage. It does not prove behavior on a
  specific MCU's interrupt and memory system.
- **Protocol.** Stop-and-wait is slow; a sliding window would raise
  throughput on long or noisy links. Sequence numbers are 8 bits. After a
  device reset the new initial sequence number is derived from a reset
  counter; if it happens to equal the host's last accepted number, the
  first message is taken as a duplicate and dropped (1 in 256 resets).
  Telemetry is best effort and is skipped while the link is busy.
- **OTA.** An interrupted download restarts from the beginning (no resume).
  Images are checked with CRC-32 only: there is no signature, so this
  protects against corruption, not against a malicious image. There is no
  anti-rollback version check. Image contents in the simulator are random
  bytes; nothing executes from the slots.
- **Fusion.** Roll and pitch only, no yaw. The filter feeds body rates
  straight into the angles, which is accurate only near level. In the "fast
  2 Hz" test scenario (+/-60 deg roll) both the fixed-point and the float
  versions are about 10 deg off the true angle. The fixed-point version
  matches the float version to within 0.0055 deg, which is what the test checks.
- **No CI** is set up.

## License

MIT. See [LICENSE](LICENSE).
