//! The FeeTech servo bus.
//!
//! One combined `sync_read` per tick covering the IMU board and all 15 servos, and one
//! `sync_write` of goal positions. The read is 15 bytes per device, from `PRESENT_POSITION_L`
//! (56) to `PRESENT_CURRENT_L` (70): position, speed, load, voltage, temperature, `moving` and
//! current in one transaction.
//!
//! That contiguous block is a real difference from the XL330 layout, where voltage and
//! temperature sat twenty registers past the end of the motion block and cost a transaction of
//! their own. Here [`RobotIo::slow_sensors`] is answered from the tick's own sample, and the
//! second transaction is gone rather than merely made cheaper.
//!
//! The IMU board is listed first, but — unlike the Dynamixel devices this bus replaced — that
//! is a *decoding* convention, not an arrival order. FeeTech nodes answer a broadcast
//! `sync_read` whenever they get around to it, so replies are collected by id and put back
//! into request order; see [`drain_replies`].
//!
//! `bus.fast_sync_read` is accepted and has **no effect** on this bus. It chose between
//! Dynamixel protocol 2.0's fast sync read (instruction `0x8A`, every device's answer appended
//! to one status packet) and its plain one; the FeeTech broadcast `sync_read` (`0x82`) is
//! already a single request that each device answers with its own status packet, so there is no
//! slower variant left to fall back to. `robotd` still reads the key and still says so when it
//! is off, so a robot whose `robotd.toml` was written for the old bus is unaffected.
//!
//! Framing, checksums and register addresses come from [`crate::feetech`], the tested module
//! that agrees byte for byte with the node's `fee.c` and the vendor `SCS.cpp`. The `imu_to_dxl`
//! node has a FeeTech personality, so it answers the same read at the same address: the first
//! 12 bytes of its telemetry map are the SFLP block, and slot 0 is decoded with [`SflpDecoder`]
//! instead of as a servo.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use crate::feetech::{
    self, FrameReader, MA_PER_CURRENT_COUNT, RAD_PER_SEC_PER_SPEED_COUNT, VOLTS_PER_COUNT,
};
use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{
    BAUD_RATE, EXPECTED_BAUD_CODE, FACTORY_ID, IMU_DXL_ID, JOINT_IDS, JOINT_NAMES, NUM_JOINTS,
};

/// Start of the contiguous block read every tick — and of the IMU node's telemetry map.
const READ_ADDR: u8 = feetech::reg::PRESENT_POSITION_L;

/// 56..70 inclusive. Fifteen bytes is the smallest read that covers position, speed, load,
/// voltage, temperature, `moving` and current: skipping the two reserved words between
/// temperature and `moving`, or between `moving` and current, would mean a second transaction
/// per tick for the fields that do not fit.
const READ_LEN: u8 = 15;

/// The same fifteen bytes, as an array length.
const SERVO_BLOCK_LEN: usize = READ_LEN as usize;

/// How long the line must stay quiet before a broadcast read is called settled.
///
/// One reply slot on this bus measures about 295 µs — that is the time a device occupies the
/// wire with its status block, and it is spent even when the addressed device is absent and
/// sends nothing. Two milliseconds is therefore roughly six reply slots of margin: comfortably
/// past the last real reply, yet short enough that one missing servo cannot overrun the 20 ms
/// control period the way the old 30 ms wait did.
///
/// It is also the serial port's own read timeout, which is what makes an empty read and "the
/// burst is over" the same event: [`FeetechIo::pump`] cannot report silence before the line has
/// been idle this long, so the burst loop needs no second, separate idle timer.
const BURST_IDLE: Duration = Duration::from_millis(2);

/// Budget for one *addressed* command and its acknowledgement.
///
/// A single-device transaction has no burst to settle and a servo may legitimately take a
/// moment to answer, so this keeps its pre-burst value even though the port's read timeout is
/// now only [`BURST_IDLE`]. [`FeetechIo::exchange`] spends it by polling.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

/// FeeTech instruction `0x08`: restart the servo, reloading RAM from EEPROM.
///
/// The vendor's own HAL defines it (`FTServo_stm32HAL-main/SCSLib/INST.h`: `INST_REBOOT 0x08`)
/// and sends it through the plain write path, no acknowledgement read. The address lives here
/// rather than in [`crate::feetech::inst`] because that module is a byte-for-byte copy of the
/// tested reference sources, which stop at `RESET` (0x0A) — and `RESET` is a factory reset that
/// would erase the id and baud rate, not the reboot this method wants.
const REBOOT_INST: u8 = 0x08;

/// RAM `Kp` and `Kd` of the position loop, from the vendor's HLS memory table
/// (`飞特通讯协议/HLS系列舵机内存表.html`, "磁编码HLS舵机-内存表解析", SRAM section).
///
/// These are the **RAM** copies (`0x32`/`0x33`); the EEPROM defaults live at 21/22 (`0x15`/`0x16`)
/// and are only loaded into RAM at power-on. Writing the RAM register takes effect immediately and
/// costs no flash, which is what a per-run gain change needs. The vendor table also documents
/// `Ki` at `0x34` (52) but calls it invalid in position-servo mode, so it is deliberately not
/// touched here. `reg` in [`crate::feetech`] stops at the registers the reference sources list and
/// is a byte-for-byte copy of the tested module, so these two addresses live here rather than
/// there.
const GAIN_KP_ADDR: u8 = 50;
const GAIN_KD_ADDR: u8 = 51;

/// The 锁标志 (address 55, `HLSCL_LOCK`): 0 lets a write to an EEPROM address survive a power
/// cycle, 1 lets it be forgotten. Ships at 1.
///
/// EEPROM writes here are wrapped in a `0 → value → 1` sequence by [`FeetechIo::write_eeprom`],
/// which is what the vendor's own `HLSCL::unLockEprom`/`LockEprom` do (and why the vendor's
/// `unLockEprom` also cuts torque first).
const EEPROM_UNLOCK: u8 = 0;
const EEPROM_LOCK: u8 = 1;

/// 舵机状态 (address 65), the servo's own error word: bit 0 voltage, bit 1 magnetic encoder,
/// bit 2 temperature, bit 3 current. Read-only, initial value 0.
///
/// This is the FeeTech counterpart of the XL330's `hardware_error_status`: the alert that holds
/// torque off until the servo is power-cycled or rebooted, and the reason
/// [`FeetechIo::adopt_replacement`] reboots a servo it has just re-addressed. It lives here for
/// the same reason as the gain addresses — the vendor's own `HLSCL.h` stops short of it.
const SERVO_STATUS_ADDR: u8 = 65;

/// Pause after each EEPROM write. The servo acknowledges before the cell is necessarily
/// committed, and the writes here happen once per motor swap, so waiting costs nothing and
/// removes the one race the vendor table leaves open.
const EEPROM_SETTLE: Duration = Duration::from_millis(20);

/// How long a servo is off the bus after a REBOOT before it answers again, before pinging it.
///
/// Measured on an HD-1910-C001 (firmware 3.46, 1 Mbps): the servo answers nothing to `0x08` and
/// is back after **823 ms** (PING polled every 20 ms), against the vendor manual's ~800 ms. The
/// constant has to clear the measured figure with margin, because pinging too early reads a
/// servo that is merely still booting as one whose re-addressing failed — and would fail a swap
/// that actually worked. The XL330 this replaced came back in "a few hundred milliseconds",
/// which is why this is not the Dynamixel path's 500 ms.
const REBOOT_SETTLE: Duration = Duration::from_millis(900);

/// How long the port is given after `open` before its first transaction.
///
/// See [`FeetechIo::open`]: a USB-serial adapter of this class drops the first transaction
/// issued immediately after a close/open cycle, and 350 ms was measured to be enough on this
/// bench (`host/bus.py`'s `OPEN_SETTLE_S`). This is that figure with a little margin — once per
/// open, against a startup path that otherwise loses a servo and waits seconds for a retry.
const OPEN_SETTLE: Duration = Duration::from_millis(400);

/// Run of consecutive stale reads at which the journal says something.
///
/// 25 reads is half a second at 50 Hz — the same span [`SflpDecoder::ready`] waits for before
/// it will call the chip's output a measurement, and far longer than any ordinary hiccup. Below
/// it the tracker stays quiet on purpose: warning on the very first repeated block is what
/// taught everyone to ignore this message. Kept in step with `ImuHealth::FROZEN_RUN`, which is
/// where the same threshold is applied to the health report — this crate is the hardware layer
/// and deliberately does not depend on the IPC vocabulary, so the number lives in both places.
const STALE_RUN_WARN: u64 = 25;

/// Detects an IMU board that answers without refreshing, by remembering the last block.
///
/// Split out from the read path so it can be tested without a serial port: the fault it
/// describes is one nothing else on the robot reports, and it would otherwise be verifiable
/// only against broken hardware.
#[derive(Debug, Default)]
struct StaleImuTracker {
    /// `None` until the first block. A fixed initial value cannot work here: it would have to
    /// be all zeros, and an all-zero block is exactly what a board whose SFLP table is still
    /// empty sends — scoring a stale read against a predecessor that never existed.
    last: Option<[u8; IMU_BLOCK_LEN]>,
    stale: ImuStale,
}

impl StaleImuTracker {
    /// Records one block and returns the length of the run it belongs to — 0 when the block is
    /// fresh, which is the overwhelmingly common answer.
    fn observe(&mut self, block: &[u8; IMU_BLOCK_LEN]) -> u64 {
        if self.last.replace(*block) == Some(*block) {
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
        } else {
            self.stale.run = 0;
        }
        self.stale.run
    }
}

/// The byte pipe under the framing, narrowed to the two operations the bus uses.
///
/// A trait rather than `Box<dyn SerialPort>` directly so a scripted fake can stand in for the
/// port in tests: the failures worth pinning here — a servo that never answers, a burst that
/// arrives out of order — otherwise need broken hardware on a bench. `Send` because the
/// control loop is moved onto its own thread by `robotd`.
///
/// **Deliberately no `flush`.** `serialport`'s `flush` is `tcdrain`, and on this board that
/// costs a fixed **~12 ms** — measured on the Radxa Zero 3W's `ttyS2` (2026-10-02, 30 reps per
/// shape) and independent of both frame length and whether anything answers: 8 bytes 12.08 ms,
/// 9 bytes 10.82 ms, 24 bytes 11.23 ms with sixteen devices replying and 12.69 ms with none,
/// 56 bytes 12.69 ms. Eight bytes is 80 µs of wire time at 1 Mbps, so the wait is the driver
/// reporting a drained transmitter, not the bus. Two transactions a tick spent ~25 ms of a
/// 20 ms period on it — the loop could not exceed ~41 Hz and reported 36.3 — while the replies
/// it was waiting for were arriving in ~1 ms and simply sat in the input buffer until it
/// looked. The read timeouts are what bound a transaction, and both are already far past the
/// moment the frame is on the wire: [`BURST_IDLE`] ends a burst, [`READ_TIMEOUT`] bounds an
/// addressed command. The reference implementation (`firmware/v1/host/bus.py`) and the
/// `rustypot`-based upstream bus both write and then read, and neither drains.
trait Transport: Send {
    fn write_all(&mut self, bytes: &[u8]) -> std::io::Result<()>;
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
}

/// The real port, wrapped so it satisfies [`Transport`] without a second `impl` block on
/// someone else's type.
struct SerialTransport(Box<dyn SerialPort>);

impl Transport for SerialTransport {
    fn write_all(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.0.write_all(bytes)
    }

    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

/// `11, 20` — how the burst summary spells a list of ids.
fn join_ids(ids: &[u8]) -> String {
    ids.iter()
        .map(u8::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

/// Drain every complete reply already buffered, slotting each by its id.
///
/// This exists rather than a `for id in ids { read_status_packet(id) }` loop because FeeTech
/// nodes answer a broadcast `sync_read` in whatever order they finish, and nothing in the
/// frame says which position in the request it belongs to. Matching on the ack's id is the
/// only correct way back to request order; trusting arrival order is the bug this function
/// makes impossible, and it is the single most important difference from the Dynamixel path.
///
/// `blocks` is one slot per requested id, in request order. `faults` collects `(id, status)`
/// for a device that answered with a nonzero status: that is *its* answer, and it must not cut
/// the burst short, or one overloaded joint would hide the state of the other fourteen. The
/// summary error is raised once, by [`FeetechIo::sync_read_blocks`], after collection ends.
///
/// A malformed or unknown-id frame is skipped, not fatal: the port carries whatever the other
/// end printed, and a missing slot at the end of the burst is what reports a genuinely silent
/// device. A reply whose payload length disagrees with the request is still fatal, because the
/// two ends no longer agree on the transaction and no later frame can repair that.
fn drain_replies(
    reader: &mut FrameReader,
    ids: &[u8],
    want_len: usize,
    blocks: &mut [Option<Vec<u8>>],
    faults: &mut Vec<(u8, u8)>,
) -> Result<()> {
    while let Some(frame) = reader.next_frame() {
        let frame = match frame {
            Ok(frame) => frame,
            // A checksum miss means resynchronise, not "the bus is down".
            Err(_) => continue,
        };
        let ack = match feetech::parse_ack(&frame) {
            Ok(ack) => ack,
            Err(_) => continue,
        };
        let Some(slot) = ids.iter().position(|want| *want == ack.id) else {
            // A late reply from a node this transaction did not address. Dropping it is
            // deliberate: writing it into a slot by arrival would reintroduce the ordering
            // assumption above.
            continue;
        };
        // The first frame for an id is its answer, data or fault alike; anything later is a
        // duplicate echo and must not overwrite the slot or double-count the fault.
        if blocks[slot].is_some() || faults.iter().any(|(fault_id, _)| *fault_id == ack.id) {
            continue;
        }
        if ack.status != 0 {
            // Recorded and skipped, never returned here: a fault is one device's news, and
            // the burst keeps going so the other replies still reach their slots.
            faults.push((ack.id, ack.status));
            continue;
        }
        if ack.data.len() != want_len {
            return Err(IoError::ShortRead {
                what: "sync_read block",
                expected: want_len,
                got: ack.data.len(),
            });
        }
        blocks[slot] = Some(ack.data);
    }
    Ok(())
}

/// The first [`IMU_BLOCK_LEN`] bytes of the IMU node's status block.
///
/// The node answers the servo status address (56) with its telemetry map, which on this board
/// is 20 bytes long (`imu_block_bytes()` in `firmware/v1/src/imu.c`): the same 12-byte
/// SFLP payload the old Dynamixel read consumed, followed by its diagnostic tail — raw
/// accelerometer, a sample counter and status flags. The tick asks for the 15 bytes the servos
/// need, so bytes 12..15 of that tail arrive along with it and are deliberately ignored.
/// Nothing but this prefix may ever be handed to [`SflpDecoder`].
fn imu_block_from_slot(slot: &[u8]) -> Result<[u8; IMU_BLOCK_LEN]> {
    if slot.len() < IMU_BLOCK_LEN {
        return Err(IoError::ShortRead {
            what: "imu block",
            expected: IMU_BLOCK_LEN,
            got: slot.len(),
        });
    }
    let mut raw = [0u8; IMU_BLOCK_LEN];
    raw.copy_from_slice(&slot[..IMU_BLOCK_LEN]);
    Ok(raw)
}

/// The fields microduck consumes from a servo's 15-byte status block.
struct ServoReading {
    position_rad: f64,
    velocity_rad_s: f64,
    current_ma: f64,
    volts: f64,
    temp_c: f64,
}

/// Decode bytes 56..70 of one servo's status block.
///
/// Reading the block contiguously is the whole point: position and speed sit at the servo's
/// own addresses, and voltage and temperature at 62/63 come along for three extra bytes rather
/// than a transaction of their own. Layout, low byte first:
///
/// ```text
/// 56,57 present_position     15-bit sign-magnitude — see feetech::sign_magnitude; a plain
///                            `as i16` reads the direction bit as a large positive angle
/// 58,59 present_speed        the same encoding, bit 15 the direction flag
/// 60,61 present_load         16-bit, direction flag at bit **10** (`HLSCL::ReadLoad`),
///                            available but unused: `Sensors` has no load field, so
///                            inventing one would put a number nobody consumes on the tick
/// 62    present_voltage      u8, 0.1 V/count
/// 63    present_temperature  u8, whole °C
/// 64..68                     registers nothing here wants (66 is `moving`)
/// 69,70 present_current      16-bit sign-magnitude, bit 15 the direction flag; the sign is
///                            dropped by the caller
/// ```
///
/// All three signed fields use the vendor's direction bit rather than two's complement, which
/// is what `HLSCL::ReadPos`/`ReadSpeed`/`ReadCurrent` do on the vendor side and what
/// `software/hls_servo_debugger/` does here. Reading speed or current as a little-endian `i16`
/// leaves a small negative reading as a number near the end of the range: current would then be
/// reported at roughly 327 times its real value, on whichever half of the joints had the bit
/// set, and every joint velocity in the observation vector would be wrong the same way.
fn decode_servo_block(block: &[u8]) -> Result<ServoReading> {
    if block.len() != SERVO_BLOCK_LEN {
        return Err(IoError::ShortRead {
            what: "motor block",
            expected: SERVO_BLOCK_LEN,
            got: block.len(),
        });
    }
    Ok(ServoReading {
        position_rad: feetech::position_rad(block[0], block[1]),
        velocity_rad_s: feetech::sign_magnitude(feetech::le_u16(block[2], block[3]), 15) as f64
            * RAD_PER_SEC_PER_SPEED_COUNT,
        volts: block[6] as f64 * VOLTS_PER_COUNT,
        temp_c: block[7] as f64,
        current_ma: feetech::sign_magnitude(feetech::le_u16(block[13], block[14]), 15) as f64
            * MA_PER_CURRENT_COUNT,
    })
}

/// Turn one tick's reply blocks into [`Sensors`], plus the raw IMU block the staleness tracker
/// counts and the voltage/temperature sample the tick now carries for free.
///
/// Slot 0 is the IMU node, slots 1.. are the servos in [`JOINT_IDS`] order. Free of the port
/// and of [`FeetechIo`] so the slot rule can be tested: feeding a servo block to the SFLP
/// decoder (or the IMU block to the servo parser) yields plausible-looking numbers, which is
/// the worst kind of bug to discover on a robot that is already moving.
fn assemble_tick(
    blocks: &[Vec<u8>],
    imu: &mut SflpDecoder,
) -> Result<(Sensors, [u8; IMU_BLOCK_LEN], Option<SlowSensors>)> {
    if blocks.len() != NUM_JOINTS + 1 {
        return Err(IoError::ShortRead {
            what: "sync_read blocks",
            expected: NUM_JOINTS + 1,
            got: blocks.len(),
        });
    }

    let raw = imu_block_from_slot(&blocks[0])?;
    let mut sensors = Sensors::default();
    sensors.imu = imu.decode(&raw);

    let mut temps_c = [0.0; NUM_JOINTS];
    let mut volts = Vec::with_capacity(NUM_JOINTS);
    for (joint, block) in blocks[1..].iter().enumerate() {
        let reading = decode_servo_block(block)?;
        sensors.positions[joint] = reading.position_rad;
        sensors.velocities[joint] = reading.velocity_rad_s;
        // Sign is dropped on purpose: direction is inferable from velocity, and every
        // consumer so far wants load, not direction — see `Sensors::currents_ma`.
        sensors.currents_ma[joint] = reading.current_ma.abs();
        temps_c[joint] = reading.temp_c;
        // A servo reporting 0 V is not a flat pack, it is a device that did not measure —
        // the IMU node does exactly this. Filtering zeros here is what keeps the mean below
        // a pack voltage instead of an average with a fake reading in it.
        if reading.volts > 0.0 {
            volts.push(reading.volts);
        }
    }

    let slow = if volts.is_empty() {
        None
    } else {
        Some(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    };
    Ok((sensors, raw, slow))
}

pub struct FeetechIo {
    port: Box<dyn Transport>,
    /// Replies are reassembled here, one stream across transactions: a frame split over two
    /// reads must survive until the rest of it arrives.
    reader: FrameReader,
    /// IMU first, then the servos in [`JOINT_IDS`] order — the order blocks are put *back*
    /// into, not the order they arrive in.
    ids: [u8; NUM_JOINTS + 1],
    imu: SflpDecoder,
    /// Blocks identical to their predecessor. The read succeeded but the board handed back
    /// the same sample, which means the policy is being fed dead orientation data — a
    /// failure that is invisible unless someone counts it. Known to happen.
    stale_imu: StaleImuTracker,
    /// The most recent tick's voltage and case temperatures. The tick's 15-byte read already
    /// carries both, so [`RobotIo::slow_sensors`] answers from here instead of paying for a
    /// transaction of its own.
    slow: Option<SlowSensors>,
}

impl FeetechIo {
    /// Assemble a bus around any byte pipe. The real [`Self::open`] passes the serial port;
    /// tests pass a scripted fake.
    fn with_transport(port: Box<dyn Transport>) -> Self {
        let mut ids = [0u8; NUM_JOINTS + 1];
        ids[0] = IMU_DXL_ID;
        ids[1..].copy_from_slice(&JOINT_IDS);
        Self {
            port,
            reader: FrameReader::new(),
            ids,
            imu: SflpDecoder::default(),
            stale_imu: StaleImuTracker::default(),
            slow: None,
        }
    }

    /// Open the bus at [`BAUD_RATE`].
    ///
    /// No read-mode flag, unlike the Dynamixel version of this function: the port is always
    /// configured with the burst's short idle timeout, and the only read the tick makes is one
    /// broadcast `sync_read`. `bus.fast_sync_read` in `robotd.toml` is therefore read by
    /// `robotd` and not passed here — see the module docs for why it has no meaning on this bus.
    ///
    /// The port is left to settle before it is used. A USB-serial adapter of this class drops
    /// the *first* transaction after a close/open cycle: measured on this bench (2026-09-30,
    /// `1a86:55d3` presenting as `ttyACM0`) four consecutive fresh starts of `robotd` each lost
    /// the ping to the first servo in the census — id 20 every time — and each then sat in the
    /// retry loop for over ten seconds. `host/bus.py` has carried the same constant
    /// (`OPEN_SETTLE_S = 0.35`) for the same reason since it was first pointed at this hardware.
    pub fn open(port: &str) -> Result<Self> {
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(BURST_IDLE)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;
        std::thread::sleep(OPEN_SETTLE);
        Ok(Self::with_transport(Box::new(SerialTransport(serial))))
    }

    /// One read burst from the port into the frame reader.
    ///
    /// `false` means the port's read timeout expired with nothing to show. That timeout is
    /// [`BURST_IDLE`] — the interval that settles a broadcast read — so an empty read is the
    /// honest end of a burst, and waiting again would spend the whole budget twice. Addressed
    /// transactions keep their own, longer budget in [`FeetechIo::exchange`].
    fn pump(&mut self) -> Result<bool> {
        let mut buf = [0u8; 256];
        match self.port.read(&mut buf) {
            Ok(0) => Ok(false),
            Ok(n) => {
                self.reader.push(&buf[..n]);
                Ok(true)
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(false),
            Err(e) => Err(IoError::Bus(format!("serial read: {e}"))),
        }
    }

    /// The next well-formed acknowledgement in the buffered stream, if a complete one is there.
    fn next_ack(&mut self) -> Option<feetech::Ack> {
        while let Some(frame) = self.reader.next_frame() {
            let frame = match frame {
                Ok(frame) => frame,
                Err(_) => continue,
            };
            if let Ok(ack) = feetech::parse_ack(&frame) {
                return Some(ack);
            }
        }
        None
    }

    /// Send one addressed instruction and wait for that node's acknowledgement.
    ///
    /// One node at a time: every per-id instruction (torque enable, register reads and
    /// writes) is addressed, so exactly one node answers and there is no order to recover.
    fn exchange(&mut self, request: &[u8], id: u8) -> Result<Vec<u8>> {
        self.port
            .write_all(request)
            .map_err(|e| IoError::Bus(format!("write to {id}: {e}")))?;

        // The port is configured with the burst's short idle timeout, but one addressed command
        // is not a burst: a single device answers it, and that device may be slow. Keep polling
        // until the pre-burst budget runs out, so tightening the burst did not quietly tighten
        // every torque enable and register check along with it.
        let deadline = Instant::now() + READ_TIMEOUT;
        loop {
            if let Some(ack) = self.next_ack() {
                if ack.id != id {
                    continue;
                }
                if ack.status != 0 {
                    return Err(IoError::Bus(format!(
                        "device {id} answered with status {:#04x}",
                        ack.status
                    )));
                }
                return Ok(ack.data);
            }
            if Instant::now() >= deadline {
                return Err(IoError::Bus(format!("no acknowledgement from {id}")));
            }
            self.pump()?;
        }
    }

    /// One broadcast `sync_read`, reassembled into request order.
    ///
    /// Everything the tick needs rides in this single transaction: the IMU node answers at the
    /// same address with its telemetry map, and every servo answers with the 15 bytes from
    /// [`READ_ADDR`]. Replies are matched by id (see [`drain_replies`]). The burst ends as soon
    /// as every requested id has been heard from — data or fault — or the line has been idle
    /// for [`BURST_IDLE`], whichever comes first; it never waits out the serial port's old
    /// 30 ms.
    ///
    /// A missing id or a nonzero status is reported as one descriptive error at the end, after
    /// every other reply has had its chance to arrive. That still fails the tick on purpose:
    /// `robotd` treats a failed read as a reason to coast on its last good sample and count
    /// `consecutive_errors`, which is the safe response. Only the stall and the mid-burst
    /// abort are gone.
    fn sync_read_blocks(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>> {
        let request = feetech::sync_read(ids, addr, len);
        // Bytes already buffered predate this request, so they cannot be a reply to it.
        // Clearing is what stops a late byte from an earlier transaction landing in whichever
        // slot happens to share its id.
        self.reader.clear();
        self.port
            .write_all(&request)
            .map_err(|e| IoError::Bus(format!("sync_read write: {e}")))?;

        let mut blocks: Vec<Option<Vec<u8>>> = vec![None; ids.len()];
        // A nonzero status is a per-device answer, not the end of the burst: the id and its
        // status are kept here while the remaining replies are still collected.
        let mut faults: Vec<(u8, u8)> = Vec::new();

        loop {
            drain_replies(&mut self.reader, ids, len as usize, &mut blocks, &mut faults)?;
            // A fault is still an answer, so an id that faulted counts as heard from: waiting
            // for the line to settle after it would only burn the idle margin for nothing.
            if ids.iter().enumerate().all(|(slot, id)| {
                blocks[slot].is_some() || faults.iter().any(|(fault_id, _)| fault_id == id)
            }) {
                break;
            }
            // Otherwise the burst ends when the line has gone idle: by the time `pump` reports
            // `false` it has already paid [`BURST_IDLE`], so there is no second wait to add.
            if !self.pump()? {
                break;
            }
        }

        // A fault or a silence still fails the tick — `robotd` coasts on its last good sample
        // and counts `consecutive_errors`, which is the safe answer. What changed is that the
        // failure is now reported once, at the end, after every other reply had its chance.
        let faulted: Vec<u8> = faults.iter().map(|(id, _)| *id).collect();
        // A device that answered with a fault is heard from, just not healthy; it must not be
        // reported as missing too, or the error would name the same id twice.
        let missing: Vec<u8> = ids
            .iter()
            .enumerate()
            .filter(|(slot, id)| blocks[*slot].is_none() && !faulted.contains(id))
            .map(|(_, id)| *id)
            .collect();

        if !faults.is_empty() || !missing.is_empty() {
            let mut parts = Vec::new();
            // Group by status, so several devices sharing one error bit produce one clause
            // instead of a stutter of near-identical ones.
            let mut statuses: Vec<(u8, Vec<u8>)> = Vec::new();
            for &(id, status) in &faults {
                match statuses.iter_mut().find(|(seen, _)| *seen == status) {
                    Some((_, group)) => group.push(id),
                    None => statuses.push((status, vec![id])),
                }
            }
            for (status, group) in statuses {
                parts.push(format!("id(s) {} faulted status {status:#04x}", join_ids(&group)));
            }
            if !missing.is_empty() {
                parts.push(format!("id(s) {} missing", join_ids(&missing)));
            }
            return Err(IoError::Bus(format!("sync_read: {}", parts.join("; "))));
        }
        Ok(blocks.into_iter().map(Option::unwrap).collect())
    }

    fn read_register(&mut self, id: u8, addr: u8) -> Result<u8> {
        let request = feetech::read(id, addr, 1);
        let data = self.exchange(&request, id)?;
        data.first().copied().ok_or(IoError::ShortRead {
            what: "register read",
            expected: 1,
            got: 0,
        })
    }

    fn write_register(&mut self, id: u8, addr: u8, value: u8) -> Result<()> {
        let request = feetech::write(id, addr, &[value]);
        self.exchange(&request, id).map(|_| ())
    }

    /// Write one **EEPROM** register, so that it survives the next power cycle.
    ///
    /// The 锁标志 (address 55) ships at 1, and the vendor memory table is explicit about what
    /// that means: "写0关闭写入锁，写入EPROM地址的值掉电保存；写1打开写入锁，写入EPROM地址的值
    /// 掉电不保存". A plain write would take effect, read back correctly, and then be silently
    /// forgotten at the next power-off — which for the id or the baud code means a servo that
    /// disappears from the bus again after every battery change, with no trace of why.
    ///
    /// So the write is wrapped the way the vendor's own library wraps it
    /// (`HLSCL::unLockEprom`/`LockEprom`: write 55 = 0, write the value, write 55 = 1). The
    /// settle is between the value and the relock, because it is the *value*'s flash cell the
    /// acknowledgement does not wait for.
    ///
    /// **The relock is addressed to the id the value implies, not the one it replaced.** When
    /// `addr` is the id register, the servo answers under its new id from that write onwards —
    /// so a relock sent to the old id would be ignored by the only device that matters, leaving
    /// the servo unlocked (its next EEPROM write unpersisted) *and* handing the caller a timeout
    /// for a change that actually worked. The id write itself is still addressed to the old id
    /// and still waits for its answer: the vendor's `SCS::Ack` reads the reply's id and fails
    /// with `ERR_SLAVE_ID` if it is not the id addressed, and the vendor's own tooling
    /// re-addresses servos exactly this way — so the reply carries the old id.
    ///
    /// Torque is deliberately not cut first, though the vendor's `unLockEprom` does. Nothing
    /// here is written while the servo is driving: the id/baud check runs at startup, before
    /// `set_torque`, and adoption runs on a servo that has never been addressed by this process.
    fn write_eeprom(&mut self, id: u8, addr: u8, value: u8) -> Result<()> {
        self.write_register(id, feetech::reg::LOCK, EEPROM_UNLOCK)?;
        self.write_register(id, addr, value)?;
        std::thread::sleep(EEPROM_SETTLE);
        let relock_to = if addr == feetech::reg::ID { value } else { id };
        self.write_register(relock_to, feetech::reg::LOCK, EEPROM_LOCK)
    }

    /// Does `id` answer a PING?
    ///
    /// Absence is `Ok(false)`, not an error: a servo that is not plugged in is the question
    /// this asks, and the startup census is built out of the answers. A write that fails is
    /// still an error — that is the port, not the servo.
    fn ping(&mut self, id: u8) -> Result<bool> {
        let request = feetech::instruction_no_addr(id, feetech::inst::PING);
        self.port
            .write_all(&request)
            .map_err(|e| IoError::Bus(format!("ping {id}: {e}")))?;

        let deadline = Instant::now() + READ_TIMEOUT;
        loop {
            if let Some(ack) = self.next_ack() {
                if ack.id == id {
                    return Ok(ack.status == 0);
                }
                continue;
            }
            if Instant::now() >= deadline {
                return Ok(false);
            }
            self.pump()?;
        }
    }

    /// The servo's own error word (address 65): bit 0 voltage, bit 1 magnetic encoder, bit 2
    /// temperature, bit 3 current. Zero when the servo reports nothing wrong.
    fn read_servo_status(&mut self, id: u8) -> Result<u8> {
        self.read_register(id, SERVO_STATUS_ADDR)
    }

    /// Assert — and correct — the EEPROM registers the control loop depends on.
    ///
    /// Returns how many needed fixing. Two registers are checked, and both are ones a
    /// factory-reset or swapped-in servo can get wrong: the id and the baud code. The XL330
    /// also needed `return_delay_time` pinned, but that reasoning does not carry over — the
    /// FeeTech bus acknowledges every instruction, so there is no per-device turnaround budget
    /// and nothing to defend. `pwm_slope` and `shutdown` have no address known for the
    /// HD-1910-C001, and writing a guessed register on a real robot is worse than not writing
    /// it at all, so they are not asserted. See the companion note's open items.
    ///
    /// A factory-reset servo is still a real hazard even with a short list: it can come back
    /// on a different baud code, which makes it invisible to every later transaction, so the
    /// check is what removes a whole class of "why is one leg dead on this robot". Both
    /// registers live in EEPROM, so both corrections go through [`Self::write_eeprom`].
    pub fn check_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &id in &JOINT_IDS {
            fixed += self.check_registers_of(id)?;
        }
        Ok(fixed)
    }

    /// [`Self::check_registers`] for one servo — also the last step of adopting a replacement,
    /// which has to leave the new servo configured exactly like the one it replaces.
    fn check_registers_of(&mut self, id: u8) -> Result<usize> {
        let mut fixed = 0;
        let got = self.read_register(id, feetech::reg::ID)?;
        if got != id {
            tracing::warn!(id, got, "correcting servo id");
            self.write_eeprom(id, feetech::reg::ID, id)?;
            fixed += 1;
        }

        let got = self.read_register(id, feetech::reg::BAUD_RATE)?;
        if got != EXPECTED_BAUD_CODE {
            tracing::warn!(
                id,
                register = "baud_rate",
                got,
                want = EXPECTED_BAUD_CODE,
                "correcting motor register"
            );
            self.write_eeprom(id, feetech::reg::BAUD_RATE, EXPECTED_BAUD_CODE)?;
            fixed += 1;
        }
        Ok(fixed)
    }

    /// The expected servo ids that do not answer a ping, in [`JOINT_IDS`] order.
    ///
    /// Fifteen pings, each bounded by [`READ_TIMEOUT`], so about half a second when the servos
    /// are unpowered and a few milliseconds when they are not. Run once at startup: this is
    /// what decides whether [`Self::adopt_replacement`] has anything to do, and it is the only
    /// bus traffic the replacement path costs a robot whose servos are all present.
    ///
    /// **A silent id is pinged twice before it counts as missing.** One lost frame is not a
    /// missing servo, and this census is the input to a *destructive* decision: a single absent
    /// id is what tells [`Self::adopt_replacement`] to go looking for a factory-fresh servo and
    /// re-address it. Measured on the bench (15 servos on a CH340-class adapter, 2026-09-30): a
    /// 15-ping sweep back to back loses one reply every few sweeps — one such miss made
    /// `robotd` sit out fourteen seconds of retries before the next sweep came back clean. The
    /// retry costs one [`READ_TIMEOUT`] per genuinely absent servo and turns a several-percent
    /// false-missing rate into its square.
    pub fn missing_servos(&mut self) -> Result<Vec<u8>> {
        let mut missing = Vec::new();
        for &id in &JOINT_IDS {
            if !self.ping(id)? && !self.ping(id)? {
                missing.push(id);
            }
        }
        Ok(missing)
    }

    /// Flash a factory-fresh servo so it takes the place of the one that is missing.
    ///
    /// A new HLS servo answers as ID 1 at 1 Mbps — the vendor memory table's shipped defaults
    /// for 主ID and 波特率 — and ID 1 is used by nothing on this bus. So when exactly one
    /// expected servo is silent, the fresh one can be found, re-addressed to the missing id,
    /// and then handed the same EEPROM check every other servo gets. That is the whole of a
    /// motor swap: nobody has to run a configuration tool first.
    ///
    /// The search is a single ping. The XL330 this bus replaced shipped at 57 600 baud and had
    /// to be looked for at that speed with a reopened port; an HLS servo already answers at the
    /// bus's own speed, so there is no second port speed to try and no `reopen` here.
    ///
    /// The servo is rebooted at the end, deliberately. `0x08` reloads RAM from EEPROM, which
    /// clears the error word at address 65 that a servo comes out of a factory flash holding —
    /// and that word is what holds torque off until a power cycle. Rebooting here means the
    /// servo that comes out of this is indistinguishable from one that was always there.
    ///
    /// Returns `Ok(false)` when nothing answers at the factory id: the servo is simply missing,
    /// or was replaced by one that is not fresh. The bus is left as it was either way, so the
    /// caller can keep waiting on it.
    pub fn adopt_replacement(&mut self, id: u8) -> Result<bool> {
        let name = JOINT_IDS
            .iter()
            .position(|&j| j == id)
            .map(|i| JOINT_NAMES[i])
            .ok_or_else(|| IoError::Bus(format!("{id} is not a joint id")))?;

        if !self.ping(FACTORY_ID)? {
            return Ok(false);
        }
        tracing::warn!(
            id,
            joint = name,
            "factory-fresh servo on the bus; flashing it as the missing joint"
        );

        // One EEPROM write, so one unlock/relock pair, and the id is the only thing that has to
        // change: the fresh servo is already at the bus's baud code.
        self.write_eeprom(FACTORY_ID, feetech::reg::ID, id)?;

        // Now an ordinary servo at the right address — but not yet one this bus will talk to
        // under its own name until it answers there, which is what the check below proves.
        let fixed = self.check_registers_of(id)?;

        RobotIo::reboot(self, id)?;
        std::thread::sleep(REBOOT_SETTLE);
        if !self.ping(id)? {
            return Err(IoError::Bus(format!(
                "servo {id} ({name}) was flashed but did not come back from its reboot"
            )));
        }
        // The reboot exists to clear this; say so if it did not, because a servo that keeps an
        // error latched holds torque off and the symptom — one limp joint — points nowhere.
        let status = self.read_servo_status(id)?;
        if status != 0 {
            tracing::error!(
                id,
                joint = name,
                status,
                "replacement servo still reports an error after its reboot"
            );
        }
        tracing::warn!(
            id,
            joint = name,
            registers_fixed = fixed,
            "replacement servo adopted"
        );
        Ok(true)
    }

    /// Present positions only — a lighter read than [`RobotIo::read`], used once at startup
    /// to adopt the pose the robot is already in.
    ///
    /// Two bytes per servo rather than the tick's fifteen: this runs once, so speed, load,
    /// voltage and current are dead weight, and a shorter reply is also a shorter burst on a
    /// wire shared with fifteen other devices.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let blocks = self.sync_read_blocks(&JOINT_IDS, feetech::reg::PRESENT_POSITION_L, 2)?;
        let mut out = [0.0; NUM_JOINTS];
        for (joint, block) in blocks.iter().enumerate() {
            out[joint] = feetech::position_rad(block[0], block[1]);
        }
        Ok(out)
    }

    /// Torque on every servo.
    ///
    /// One transaction per joint, so this is not something to call per tick — the control loop
    /// calls it once, when someone enables the policy on a limp robot. See
    /// [`RobotIo::set_torque`] for what has *not* changed: nothing touches torque because a
    /// process started.
    ///
    /// **Every servo is written, whatever the others said.** Fifteen acknowledged transactions
    /// in a row, and one dropped ack used to end the loop there — which for `on = false` on the
    /// way to a power-off meant a robot that sat down and switched off with half its legs still
    /// locked, because the tick that saw the error was the last one that could have retried.
    /// Writing the rest costs the same as it would have, and the error names every joint that
    /// did not answer so the caller can decide whether to ask again.
    ///
    /// The FeeTech bus acknowledges every instruction, so this shape is unchanged from the
    /// Dynamixel one. What is gone is the reason `return_delay_time` dominated the old tick
    /// budget: turnaround is no longer a per-device delay the host has to police, so torque
    /// bring-up is bounded by the 30 ms read timeout, not by sixteen configured delays.
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        let value = [u8::from(on)];
        let mut failed = Vec::new();
        for &id in &JOINT_IDS {
            let request = feetech::write(id, feetech::reg::TORQUE_ENABLE, &value);
            if let Err(e) = self.exchange(&request, id) {
                failed.push(format!("torque {on} on {id}: {e}"));
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(IoError::Bus(failed.join("; ")))
        }
    }

    /// Ramp every joint from where it is now to `target`, linearly.
    ///
    /// Only ever called by an explicit `init` — the control loop must never move the robot
    /// on its own, because that would make an update restart a fall risk. Blocking, and
    /// deliberately so: nothing else should be talking to the bus while this runs.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }
}

/// Which servo a factory-fresh one should become, given the ids that did not answer.
///
/// Only an unambiguous answer is one: with two servos silent there is no telling which of them
/// the new one replaces, and guessing would flash a leg joint as a neck joint. With none silent
/// there is nothing to adopt — a stray fresh servo on a complete bus is not this code's problem.
pub fn replacement_target(missing: &[u8]) -> Option<u8> {
    match missing {
        [one] => Some(*one),
        _ => None,
    }
}

impl RobotIo for FeetechIo {
    fn read(&mut self) -> Result<Sensors> {
        let ids = self.ids;
        let blocks = self.sync_read_blocks(&ids, READ_ADDR, READ_LEN)?;

        let (sensors, raw, slow) = assemble_tick(&blocks, &mut self.imu)?;
        self.slow = slow;

        // Say so, or the counters are numbers nobody ever reads — but only once the run is
        // long enough to mean something. Rate-limited past that because a board which has
        // stopped refreshing produces one of these every single tick, and 50 Hz of identical
        // warnings would evict the journal.
        let run = self.stale_imu.observe(&raw);
        if run == STALE_RUN_WARN || (run > STALE_RUN_WARN && run.is_multiple_of(500)) {
            tracing::warn!(
                consecutive = run,
                total = self.stale_imu.stale.total,
                "imu board has returned the same sample {run} reads running — orientation is frozen"
            );
        }

        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        // Sign-magnitude, never a two's-complement cast: `position_from_rad` sets the
        // direction bit the servo expects, where casting a negative count through `i16`
        // would hand a negative angle over as a large positive one.
        let payloads: Vec<[u8; 2]> = targets
            .positions
            .iter()
            .map(|rad| feetech::position_from_rad(*rad))
            .collect();
        let refs: Vec<&[u8]> = payloads.iter().map(|bytes| bytes.as_slice()).collect();
        let frame = feetech::sync_write(&JOINT_IDS, feetech::reg::GOAL_POSITION_L, &refs)
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))?;

        // Broadcast, and therefore unacknowledged by design: a SYNC_WRITE addressed to 0xFE
        // gets no status packet from anyone. There is nothing to wait for; a servo that
        // missed the write shows up on the next tick's read instead.
        self.port
            .write_all(&frame)
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))?;
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        // The inherent method, which predates the trait and is still what `robotd init` uses.
        FeetechIo::set_torque(self, on)
    }

    /// Reboot one servo — FeeTech instruction `0x08`, which reloads RAM from EEPROM.
    ///
    /// The way out of a latched hardware error — overload, overheating, electrical shock — which
    /// otherwise holds torque off until the battery is pulled. On the Dynamixel bus this was
    /// protocol 2's REBOOT; on the HD-1910-C001 it is instruction `0x08`, which the vendor HAL
    /// defines as `INST_REBOOT` and sends with no parameters. It is deliberately *not*
    /// [`feetech::inst::RESET`] (`0x0A`): that is a factory reset and would erase the id, the
    /// baud rate and the angle limits of a servo bolted into a robot.
    ///
    /// **Measured on hardware** (HD-1910-C001, firmware 3.46, 1 Mbps, 2026-09-24): `0x08` with
    /// no parameters and a unicast id exists and reboots the servo, which answers *nothing* — the
    /// request times out by design. It is alive again after **823 ms** (PING polled every 20 ms;
    /// the vendor manual says ~800 ms), with id, baud code, running mode, torque state, lock,
    /// EEPROM P/D (21/22) and OFS unchanged — but with **RAM reloaded from EEPROM**, so the RAM
    /// position gains (50/51) come back as the EEPROM values. That is the contract
    /// [`RobotIo::reboot`] documents: the servo comes back with torque off and its RAM registers
    /// at their EEPROM defaults, and the caller rewrites the gains.
    ///
    /// Torque goes off first (address 40), as the vendor manual asks, and as `robotd` already
    /// does around [`crate::safety::Safety::reboot_motors`]. That write is addressed and
    /// acknowledged, so a servo that is already gone is reported instead of silently skipped.
    ///
    /// **No acknowledgement is awaited for the reboot itself.** There is none, and waiting would
    /// burn the port timeout once per servo and stall the control loop, which calls this inside
    /// the tick. The frame simply goes on the wire and this returns; it does not sleep and it does
    /// not ping. The ~0.8 s off-bus window is the caller's to ride out — the tick's reads fail and
    /// the loop coasts, then the gain cache is already forgotten so the next `apply` rewrites the
    /// gains.
    fn reboot(&mut self, id: u8) -> Result<()> {
        self.write_register(id, feetech::reg::TORQUE_ENABLE, 0)?;

        let frame = feetech::instruction_no_addr(id, REBOOT_INST);
        // Written and done: `REBOOT` answers nothing, so there is no status packet to collect
        // and nothing to wait for on the wire either. The caller's [`REBOOT_SETTLE`] sleep is
        // what gives the servo time to come back.
        self.port
            .write_all(&frame)
            .map_err(|e| IoError::Bus(format!("reboot {id}: {e}")))?;
        Ok(())
    }

    /// Set the position P gain on every joint, from the HLS memory table.
    ///
    /// Writes RAM address 50 (`Kp`, `0x32`) once per joint with a single byte, clamped to the
    /// vendor's documented range `0..254`. The value microduck's `kp` carries is passed through
    /// unchanged — no conversion is applied here — while the register's own scale is documented
    /// as **1/8** (and `Kd`'s as 1/4), so the gain the servo actually applies is `kp / 8`. The
    /// table gives address 50 as "位置伺服模式：控制电机位置环的比例系数(1/8)".
    ///
    /// `Kd` (RAM address 51, `0x33`) is written to 0 alongside it. The factory D value is *not*
    /// zero, and leaving it in place would let the servo's own position loop damp every motion at
    /// whatever `kp` this sets — the same reason the Dynamixel path zeroed the D gain. The vendor
    /// documents `Kd` as "位置伺服模式：控制电机位置环的微分系数(1/4)".
    ///
    /// `Ki` (RAM address 52, `0x34`) is deliberately **not** written, not even as 0 for symmetry:
    /// the vendor table says it is 无效 in position-servo mode (it belongs to the constant-speed
    /// loop), so touching it would be a write with no defined meaning.
    ///
    /// The EEPROM copies at 21/22 (`0x15`/`0x16`) are not written either. Those are merely the
    /// power-on defaults loaded into RAM; the RAM value takes effect immediately, and writing the
    /// EEPROM copy on every start would burn flash for nothing. Consequently no unlock handling is
    /// needed: address 55 (`锁标志`) only governs whether EEPROM writes persist, and no EEPROM
    /// address is written here.
    ///
    /// The gain this is called with comes from `policy.gain`, which this tree defaults to **32**
    /// (the vendor's own EEPROM default) rather than the prototype's 200 — 200 is the top of the
    /// 0..254 range here and makes the position loop self-oscillate, measured on the bench. The
    /// value is still per-robot tuning: this function cannot know the right one. See the
    /// companion note's §6 and its open item 2.
    ///
    /// **Every joint is written, whatever the others said**, matching [`FeetechIo::set_torque`]:
    /// one silent servo must not leave the rest of the robot on a stale gain.
    fn set_gain(&mut self, kp: u16) -> Result<()> {
        let value = [kp.min(254) as u8];
        let kd = [0u8];
        let mut failed = Vec::new();
        for &id in &JOINT_IDS {
            let request = feetech::write(id, GAIN_KP_ADDR, &value);
            if let Err(e) = self.exchange(&request, id) {
                failed.push(format!("Kp {kp} on {id}: {e}"));
            }
            let request = feetech::write(id, GAIN_KD_ADDR, &kd);
            if let Err(e) = self.exchange(&request, id) {
                failed.push(format!("Kd 0 on {id}: {e}"));
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(IoError::Bus(failed.join("; ")))
        }
    }

    /// Supply voltage and case temperatures, from the tick's own sample.
    ///
    /// These used to live 20 registers past the end of the motion block and cost a second
    /// transaction. With the HLS layout they sit inside the block the tick already reads
    /// (voltage at 62, temperature at 63), so folding them in removes a transaction rather
    /// than making one cheaper — and the sample is at most one tick old, which for a pack
    /// voltage and a case temperature is no age at all.
    ///
    /// Voltage is averaged because all 15 servos sit on one pack: a single reading is the same
    /// measurement with more noise. Temperature is *not* averaged here — the caller gets every
    /// joint, because one loaded joint running hot is the case worth seeing and a mean over
    /// fifteen hides it. Zero-voltage readings are filtered out where the mean is built, in
    /// `assemble_tick`, so a device that did not measure cannot halve the reported pack.
    ///
    /// The previous sample is kept across a failed tick on purpose: this is all-or-nothing per
    /// tick, and the caller is expected to hold its last good number rather than treat one miss
    /// as a flat pack. Before the first successful tick there is nothing to hold, and the error
    /// says so.
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.slow.ok_or(IoError::ShortRead {
            what: "input voltage",
            expected: NUM_JOINTS,
            got: 0,
        })
    }

    fn imu_stale(&self) -> ImuStale {
        self.stale_imu.stale
    }

    fn imu_ready(&self) -> bool {
        self.imu.ready()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::feetech::RAD_PER_COUNT;

    /// One silent servo is the only case a swap can be inferred from. With two silent there is
    /// no telling which the fresh servo replaces, and guessing would flash a leg joint as a neck
    /// joint; with none silent a stray fresh servo is nobody's replacement.
    #[test]
    fn a_replacement_is_inferred_only_from_exactly_one_missing_servo() {
        assert_eq!(replacement_target(&[]), None);
        assert_eq!(replacement_target(&[23]), Some(23));
        assert_eq!(replacement_target(&[23, 31]), None);
        assert_eq!(replacement_target(&JOINT_IDS), None);
    }

    /// Speed and current carry their sign in bit 15, the vendor's way, and the tick decoder has
    /// to read them that way. A little-endian `i16` — which this decoder used to use — turns a
    /// small negative reading into one near the bottom of the range: roughly 327x the real
    /// current on whichever half of the joints had the direction bit set, and a joint velocity
    /// in the observation vector to match.
    #[test]
    fn speed_and_current_use_the_direction_bit_not_twos_complement() {
        // -100 counts of speed and -250 of current, the way the vendor encodes them: a
        // direction bit plus a magnitude, not a negative `i16`.
        let block = servo_block(0, 0x8064, 0, 82, 41, 0x80FA);
        let reading = decode_servo_block(&block).unwrap();

        assert!(
            (reading.velocity_rad_s + 100.0 * RAD_PER_SEC_PER_SPEED_COUNT).abs() < 1e-9,
            "velocity must be -100 counts, got {} rad/s",
            reading.velocity_rad_s
        );
        assert!(
            (reading.current_ma + 250.0 * MA_PER_CURRENT_COUNT).abs() < 1e-9,
            "current must be -250 counts, got {} mA",
            reading.current_ma
        );
        // And the two's-complement reading really is a different number, so the test would
        // catch a regression rather than passing by luck.
        assert_ne!(
            reading.velocity_rad_s,
            i16::from_le_bytes([0x64, 0x80]) as f64 * RAD_PER_SEC_PER_SPEED_COUNT
        );
        assert_ne!(
            reading.current_ma,
            i16::from_le_bytes([0xFA, 0x80]) as f64 * MA_PER_CURRENT_COUNT
        );
    }

    /// The block parsed per servo must cover current, velocity and position without
    /// overrunning. If `READ_LEN` and the offsets below ever disagree, joints get each
    /// other's values — which reads as a wiring fault, not a code bug.
    #[test]
    fn read_block_is_long_enough_for_every_field() {
        assert_eq!(SERVO_BLOCK_LEN, 15);
        // The deepest field the servo decoder reads is present_current at 69/70, i.e. block
        // offsets 13/14. The IMU prefix needs the first twelve bytes of the same block.
        const {
            assert!(
                SERVO_BLOCK_LEN
                    > (feetech::reg::PRESENT_CURRENT_L - feetech::reg::PRESENT_POSITION_L) as usize
            )
        };
        const { assert!(SERVO_BLOCK_LEN >= IMU_BLOCK_LEN) };
    }

    /// The position conversion must round-trip through the servo's own encoding, which is
    /// 15-bit sign-magnitude rather than a two's-complement count. A mismatch would mean the
    /// loop commands a different angle than it believes it read back — and the direction bit
    /// makes that a two-sided error, not an offset.
    #[test]
    fn position_round_trips_through_the_feetech_conversion() {
        for rad in [-3.0, -1.0, 0.0, 0.5, 3.0] {
            let [low, high] = feetech::position_from_rad(rad);
            let back = feetech::position_rad(low, high);
            assert!((back - rad).abs() < feetech::RAD_PER_COUNT, "{rad} -> {back}");
            // The bytes really are sign-magnitude on the wire, not a negative i16.
            let raw = feetech::le_u16(low, high);
            let via_counts = feetech::position_raw_to_counts(raw) as f64 * RAD_PER_COUNT;
            assert!((via_counts - back).abs() < 1e-12);
        }
    }

    /// 0.732 rpm per count, the figure the FeeTech reference sources use (60 counts is
    /// 43.92 rpm). Getting this wrong scales every joint velocity in the observation vector by
    /// a constant, which a policy tolerates just well enough to walk badly.
    #[test]
    fn velocity_scale_matches_the_datasheet_figure() {
        let one_count = RAD_PER_SEC_PER_SPEED_COUNT;
        let expected_rpm = 0.732;
        assert!((one_count * 60.0 / std::f64::consts::TAU - expected_rpm).abs() < 1e-12);
    }

    /// One healthy ack frame as a node would build it: `FF FF id LEN status data ~SUM`.
    fn ack(id: u8, data: &[u8]) -> Vec<u8> {
        ack_status(id, 0x00, data)
    }

    /// One ack frame with an explicit status byte — what a device reporting a fault sends.
    fn ack_status(id: u8, status: u8, data: &[u8]) -> Vec<u8> {
        let len = (data.len() + 2) as u8;
        let mut frame = vec![0xFF, 0xFF, id, len, status];
        frame.extend_from_slice(data);
        let sum = !frame[2..].iter().fold(0u8, |acc, b| acc.wrapping_add(*b));
        frame.push(sum);
        frame
    }

    /// One servo's 15-byte status block, field by field.
    ///
    /// The signed fields are taken as the **raw wire words** rather than as numbers, because
    /// that is the only way to write a negative one down: speed, load and current are
    /// sign-magnitude (bit 15, bit 10, bit 15), so `-100` is `0x8064` on the wire and there is
    /// no `i16` that round-trips through `to_le_bytes` to the same bytes.
    fn servo_block(
        position_raw: u16,
        speed_raw: u16,
        load_raw: u16,
        volts: u8,
        temp: u8,
        current_raw: u16,
    ) -> [u8; SERVO_BLOCK_LEN] {
        let mut block = [0u8; SERVO_BLOCK_LEN];
        block[0..2].copy_from_slice(&position_raw.to_le_bytes());
        block[2..4].copy_from_slice(&speed_raw.to_le_bytes());
        block[4..6].copy_from_slice(&load_raw.to_le_bytes());
        block[6] = volts;
        block[7] = temp;
        block[13..15].copy_from_slice(&current_raw.to_le_bytes());
        block
    }

    /// The central reason this rewrite is not a search-and-replace: FeeTech nodes answer a
    /// broadcast read in whatever order they finish. A `for id in ids` read loop — which was
    /// correct for the in-order Dynamixel devices — would hand joint 21's sample to joint 20.
    #[test]
    fn out_of_order_sync_read_replies_are_reassembled_in_request_order() {
        let ids = [20u8, 21];
        let first = servo_block(0x0100, 10, 0, 80, 30, 100);
        let second = servo_block(0x0200, 20, 0, 81, 31, 200);

        // The second servo answers first: legal on this bus, and fatal to arrival-order code.
        let mut stream = ack(21, &second);
        stream.extend_from_slice(&ack(20, &first));

        let mut reader = FrameReader::new();
        reader.push(&stream);
        let mut blocks: Vec<Option<Vec<u8>>> = vec![None; ids.len()];
        let mut faults = Vec::new();
        drain_replies(
            &mut reader,
            &ids,
            SERVO_BLOCK_LEN,
            &mut blocks,
            &mut faults,
        )
        .unwrap();
        assert!(faults.is_empty());

        assert_eq!(
            blocks[0].as_deref(),
            Some(&first[..]),
            "slot 0 must hold id 20's reply"
        );
        assert_eq!(
            blocks[1].as_deref(),
            Some(&second[..]),
            "slot 1 must hold id 21's reply"
        );
    }

    /// A byte pipe scripted with the exact bytes a burst would carry, and honest about silence:
    /// an empty read blocks for [`BURST_IDLE`] and then reports a timeout, the way the real
    /// serial port is configured. A fake that returned instantly would make the timing
    /// assertion in the missing-device test below say nothing at all.
    struct BurstTransport {
        replies: Vec<u8>,
    }

    impl BurstTransport {
        fn new(replies: Vec<u8>) -> Self {
            Self { replies }
        }
    }

    impl Transport for BurstTransport {
        fn write_all(&mut self, _bytes: &[u8]) -> std::io::Result<()> {
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            if self.replies.is_empty() {
                std::thread::sleep(BURST_IDLE);
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "the line is idle",
                ));
            }
            let n = buf.len().min(self.replies.len());
            buf[..n].copy_from_slice(&self.replies[..n]);
            self.replies.drain(..n);
            Ok(n)
        }
    }

    /// A servo that never answers must not cost the tick the old 30 ms serial wait: the burst
    /// ends after one settled interval, and the error still names the silent id. The fake sleeps
    /// [`BURST_IDLE`] on an empty read, so the elapsed time here measures the real budget.
    #[test]
    fn a_missing_servo_ends_the_burst_promptly_and_is_named() {
        let ids = [11u8, 20];
        let present = servo_block(0x0100, 0, 0, 80, 30, 0);
        // Only 11 answers; 20 is absent.
        let mut io = FeetechIo::with_transport(Box::new(BurstTransport::new(ack(11, &present))));

        let started = Instant::now();
        let err = io
            .sync_read_blocks(&ids, READ_ADDR, READ_LEN)
            .expect_err("a silent id must fail the burst");
        let elapsed = started.elapsed();

        assert!(
            elapsed < READ_TIMEOUT,
            "a missing id must not be waited out with the {READ_TIMEOUT:?} addressed budget, \
             but the burst took {elapsed:?}"
        );
        let message = err.to_string();
        assert!(
            message.contains("20"),
            "the error must name the silent id: {message}"
        );
        assert!(
            message.contains("missing"),
            "the error must say the id is missing: {message}"
        );
    }

    /// A device answering with a nonzero status is that device's answer, not the end of the
    /// burst. The faulted frame arrives *first* here, so an implementation that still bailed out
    /// on status would never collect id 20 and would then have to report it missing.
    #[test]
    fn a_faulted_servo_is_named_without_aborting_the_burst() {
        let ids = [11u8, 20];
        let present = servo_block(0x0200, 0, 0, 81, 31, 0);
        let mut stream = ack_status(11, 0x08, &[]);
        stream.extend_from_slice(&ack(20, &present));
        let mut io = FeetechIo::with_transport(Box::new(BurstTransport::new(stream)));

        let err = io
            .sync_read_blocks(&ids, READ_ADDR, READ_LEN)
            .expect_err("a faulted device must still fail the tick");

        let message = err.to_string();
        assert!(
            message.contains("11"),
            "the error must name the faulted id: {message}"
        );
        assert!(
            message.contains("faulted"),
            "the error must say the id faulted: {message}"
        );
        assert!(
            message.contains("0x08"),
            "the error must carry the status: {message}"
        );
        assert!(
            !message.contains("missing") && !message.contains("20"),
            "id 20 answered and must have been collected, not reported missing: {message}"
        );
    }

    /// The happy path through the whole burst: replies arrive out of order, and the caller still
    /// gets one block per requested id in request order. This is the `sync_read_blocks`-level
    /// counterpart of the `drain_replies` test above.
    #[test]
    fn sync_read_returns_request_order_when_replies_arrive_out_of_order() {
        let ids = [11u8, 20];
        let first = servo_block(0x0100, 10, 0, 80, 30, 100);
        let second = servo_block(0x0200, 20, 0, 81, 31, 200);
        // The second servo answers first: legal on this bus, fatal to arrival-order code.
        let mut stream = ack(20, &second);
        stream.extend_from_slice(&ack(11, &first));
        let mut io = FeetechIo::with_transport(Box::new(BurstTransport::new(stream)));

        let blocks = io
            .sync_read_blocks(&ids, READ_ADDR, READ_LEN)
            .expect("every requested id answered");

        assert_eq!(blocks[0].as_slice(), &first[..], "slot 0 must hold id 11's reply");
        assert_eq!(
            blocks[1].as_slice(),
            &second[..],
            "slot 1 must hold id 20's reply"
        );
    }

    /// The IMU node answers the servo status address with its own telemetry map, so slot 0 and
    /// slots 1.. must be decoded by different rules. The servo bytes below are full of values
    /// the SFLP decoder would happily read as a loud gyro, which is what makes a slot mix-up
    /// invisible without a test like this.
    #[test]
    fn imu_slot_is_decoded_as_sflp_and_servo_blocks_are_not() {
        let mut imu_slot = [0u8; SERVO_BLOCK_LEN];
        // A live SFLP quaternion so the decoder produces a real orientation.
        imu_slot[6..8].copy_from_slice(&0x3000u16.to_le_bytes());

        // Deliberate Dynamixel 2.0 header bytes (`FF FF FD`) in the servo payloads. On the
        // old bus that sequence needed de-stuffing; a reader that resynchronised on it would
        // tear the block apart instead of decoding it, so this pins the frame reader too.
        let servo = servo_block(0xFFFF, 0x0700, 0, 82, 41, 300);

        let mut blocks = vec![Vec::new(); NUM_JOINTS + 1];
        blocks[0] = imu_slot.to_vec();
        for slot in blocks.iter_mut().skip(1) {
            *slot = servo.to_vec();
        }

        let mut decoder = SflpDecoder::default();
        let mut decoded = None;
        for _ in 0..3 {
            decoded = Some(assemble_tick(&blocks, &mut decoder).unwrap());
        }
        let (sensors, raw, slow) = decoded.unwrap();

        assert_eq!(
            &raw[6..8],
            &0x3000u16.to_le_bytes(),
            "the SFLP prefix comes from slot 0"
        );
        assert_ne!(
            sensors.imu.quat,
            [1.0, 0.0, 0.0, 0.0],
            "slot 0 was decoded as SFLP"
        );
        // The servo payload would decode as a large gyro; the gyro must instead be slot 0's
        // three zero bytes. This is the assertion that a servo block never reaches SFLP.
        assert_eq!(sensors.imu.gyro, [0.0, 0.0, 0.0]);

        let expected_position = feetech::position_raw_to_counts(0xFFFF) as f64 * RAD_PER_COUNT;
        for (joint, _) in JOINT_IDS.iter().enumerate() {
            assert!((sensors.positions[joint] - expected_position).abs() < 1e-12);
            assert!(
                (sensors.velocities[joint] - 0x0700 as f64 * RAD_PER_SEC_PER_SPEED_COUNT).abs()
                    < 1e-9
            );
            assert!((sensors.currents_ma[joint] - 300.0 * MA_PER_CURRENT_COUNT).abs() < 1e-9);
        }

        let slow = slow.unwrap();
        assert!((slow.volts - 8.2).abs() < 1e-9);
        assert_eq!(slow.temps_c, [41.0; NUM_JOINTS]);
    }

    /// State the folded slow-sensor sample directly: the tick's read carries voltage and
    /// temperature, and a device reporting 0 V (the IMU node, and any servo that did not
    /// measure) must not drag the pack average down.
    #[test]
    fn the_tick_folds_voltage_and_temperature_in() {
        let mut blocks = vec![Vec::new(); NUM_JOINTS + 1];
        blocks[0] = vec![0u8; SERVO_BLOCK_LEN]; // IMU node: 0 V, 0 °C in the tail
        for (joint, slot) in blocks.iter_mut().skip(1).enumerate() {
            let volts = if joint == 0 { 0 } else { 82 };
            *slot = servo_block(0, 0, 0, volts, 40 + joint as u8, 0).to_vec();
        }
        let mut decoder = SflpDecoder::default();
        let (_, _, slow) = assemble_tick(&blocks, &mut decoder).unwrap();
        let slow = slow.unwrap();
        // Fourteen servos read 8.2 V; the zero and the IMU slot are filtered out.
        assert!((slow.volts - 8.2).abs() < 1e-9);
        assert_eq!(slow.temps_c[0], 40.0);
        assert_eq!(slow.temps_c[14], 54.0);
    }

    /// A byte pipe that answers the way a partly broken bus does: the ids in
    /// `ack_writes_from` acknowledge a WRITE, every other node stays silent.
    #[derive(Default)]
    struct FakeState {
        writes: Vec<u8>,
        replies: Vec<u8>,
    }

    #[derive(Clone, Default)]
    struct FakeTransport {
        state: Arc<Mutex<FakeState>>,
        ack_writes_from: Vec<u8>,
    }

    impl FakeTransport {
        fn new(ack_writes_from: Vec<u8>) -> Self {
            Self {
                state: Arc::new(Mutex::new(FakeState::default())),
                ack_writes_from,
            }
        }

        fn writes(&self) -> Vec<u8> {
            self.state.lock().unwrap().writes.clone()
        }
    }

    impl Transport for FakeTransport {
        fn write_all(&mut self, bytes: &[u8]) -> std::io::Result<()> {
            let mut state = self.state.lock().unwrap();
            state.writes.extend_from_slice(bytes);
            let is_write = bytes.len() >= 6
                && bytes[0] == 0xFF
                && bytes[1] == 0xFF
                && bytes[4] == feetech::inst::WRITE;
            if is_write && self.ack_writes_from.contains(&bytes[2]) {
                let frame = ack(bytes[2], &[]);
                state.replies.extend_from_slice(&frame);
            }
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut state = self.state.lock().unwrap();
            if state.replies.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "silent servo",
                ));
            }
            let n = buf.len().min(state.replies.len());
            buf[..n].copy_from_slice(&state.replies[..n]);
            state.replies.drain(..n);
            Ok(n)
        }
    }

    /// Count the addressed WRITE instructions in a byte log. A WRITE instruction frame parses
    /// as an ack with `status` equal to the instruction byte, which is why this can reuse the
    /// same framing the bus does.
    fn count_write_frames(bytes: &[u8]) -> usize {
        let mut reader = FrameReader::new();
        reader.push(bytes);
        let mut count = 0;
        while let Some(frame) = reader.next_frame() {
            let Ok(frame) = frame else { continue };
            if frame.get(4) == Some(&feetech::inst::WRITE) {
                count += 1;
            }
        }
        count
    }

    /// The behaviour `set_torque` is documented for: a silent servo is reported *after* every
    /// other servo has been written. Stopping at the first miss is what used to leave a robot
    /// half-locked on the way to a power-off.
    #[test]
    fn set_torque_writes_every_servo_and_reports_only_the_silent_one() {
        let silent = JOINT_IDS[2];
        let acking: Vec<u8> = JOINT_IDS.iter().copied().filter(|id| *id != silent).collect();
        let fake = FakeTransport::new(acking);
        let handle = fake.clone();
        let mut io = FeetechIo::with_transport(Box::new(fake));

        let err = io.set_torque(true).expect_err("a silent servo must be reported");
        let message = err.to_string();
        assert!(
            message.contains(&silent.to_string()),
            "the failure must name id {silent}: {message}"
        );
        assert_eq!(
            count_write_frames(&handle.writes()),
            NUM_JOINTS,
            "every servo must get its write, including the ones after the silent one"
        );
    }

    /// Decode the addressed single-byte `WRITE` instructions in a byte log as `(id, address,
    /// value)`. A WRITE instruction frame parses as an ack with `status` equal to the instruction
    /// byte, so this walks the same framing the bus does.
    fn write_frames(bytes: &[u8]) -> Vec<(u8, u8, u8)> {
        let mut reader = FrameReader::new();
        reader.push(bytes);
        let mut out = Vec::new();
        while let Some(frame) = reader.next_frame() {
            let Ok(frame) = frame else { continue };
            if frame.get(4) == Some(&feetech::inst::WRITE) {
                if let (Some(id), Some(addr), Some(value)) =
                    (frame.get(2), frame.get(5), frame.get(6))
                {
                    out.push((*id, *addr, *value));
                }
            }
        }
        out
    }

    /// Every instruction frame in a byte log, as `(id, instruction)`, in the order sent. Unlike
    /// [`write_frames`] this keeps instructions that carry no address — the reboot among them.
    fn instruction_frames(bytes: &[u8]) -> Vec<(u8, u8)> {
        let mut reader = FrameReader::new();
        reader.push(bytes);
        let mut out = Vec::new();
        while let Some(frame) = reader.next_frame() {
            let Ok(frame) = frame else { continue };
            if let (Some(id), Some(inst)) = (frame.get(2), frame.get(4)) {
                out.push((*id, *inst));
            }
        }
        out
    }

    /// `set_gain` writes the position-loop **RAM** registers the vendor memory table documents:
    /// `Kp` at address 50 with the value passed through, and `Kd` at 51 zeroed so the factory
    /// differential gain cannot damp the servo's own loop. Every joint gets both writes.
    #[test]
    fn set_gain_writes_kp_and_kd_to_every_joint() {
        let fake = FakeTransport::new(JOINT_IDS.to_vec());
        let handle = fake.clone();
        let mut io = FeetechIo::with_transport(Box::new(fake));
        io.set_gain(200).unwrap();

        let writes = write_frames(&handle.writes());
        assert_eq!(writes.len(), NUM_JOINTS * 2, "one Kp and one Kd write per joint");
        for (joint, pair) in writes.chunks(2).enumerate() {
            let id = JOINT_IDS[joint];
            assert_eq!(pair[0], (id, GAIN_KP_ADDR, 200), "Kp must go to RAM address 50");
            assert_eq!(pair[1], (id, GAIN_KD_ADDR, 0), "Kd must be zeroed at RAM address 51");
        }
        assert!(
            writes.iter().all(|(_, addr, _)| !matches!(*addr, 21 | 22 | 52)),
            "Ki (52) is invalid in position mode and the EEPROM defaults (21/22) are not \
             per-run writes"
        );
    }

    /// The vendor range for `Kp`/`Kd` is 0..254, so a larger `kp` must clamp: a `u16` sent as a
    /// single byte would otherwise wrap and command a small gain where a large one was asked for.
    #[test]
    fn set_gain_clamps_to_the_register_range() {
        let fake = FakeTransport::new(JOINT_IDS.to_vec());
        let handle = fake.clone();
        let mut io = FeetechIo::with_transport(Box::new(fake));
        io.set_gain(1000).unwrap();
        let writes = write_frames(&handle.writes());
        assert!(!writes.is_empty());
        assert!(
            writes.chunks(2).all(|pair| pair[0].2 == 254),
            "Kp must clamp to 254, the top of 0..254"
        );
    }

    /// `reboot` must cut torque (address 40 = 0) and then put exactly one REBOOT (`0x08`)
    /// instruction on the wire for the named id. It must be `0x08` and not `RESET` (`0x0A`): the
    /// latter is a factory reset that would wipe the servo's id and baud rate.
    #[test]
    fn reboot_cuts_torque_then_sends_one_reboot_instruction() {
        let id = JOINT_IDS[3];
        // Only the torque write is acknowledged; a reboot has no reply to give.
        let fake = FakeTransport::new(vec![id]);
        let handle = fake.clone();
        let mut io = FeetechIo::with_transport(Box::new(fake));

        io.reboot(id)
            .expect("a servo answering the torque write must reboot");

        let writes = handle.writes();
        assert_eq!(
            write_frames(&writes),
            vec![(id, feetech::reg::TORQUE_ENABLE, 0)],
            "torque must go off first, once, on the named servo"
        );
        assert_eq!(
            instruction_frames(&writes),
            vec![(id, feetech::inst::WRITE), (id, REBOOT_INST)],
            "exactly the torque write and one REBOOT, in that order, on the right id"
        );
        assert!(
            !instruction_frames(&writes)
                .iter()
                .any(|(_, inst)| *inst == feetech::inst::RESET),
            "RESET (0x0A) is a factory reset and must never be sent as a reboot"
        );
    }

    /// The reboot instruction is unacknowledged, so `reboot` must return as soon as the frame is
    /// on the wire. The fake acknowledges the torque write and then stays silent: an
    /// implementation that waited for a reply — as `exchange` does — would spend the whole
    /// [`READ_TIMEOUT`] and fail.
    #[test]
    fn reboot_returns_without_waiting_for_an_acknowledgement() {
        let id = JOINT_IDS[0];
        let fake = FakeTransport::new(vec![id]);
        let handle = fake.clone();
        let mut io = FeetechIo::with_transport(Box::new(fake));

        let started = Instant::now();
        io.reboot(id)
            .expect("a reboot answers nothing, and that must not be an error");
        let elapsed = started.elapsed();

        assert!(
            elapsed < READ_TIMEOUT,
            "a reboot has no reply to wait for, but it took {elapsed:?}"
        );
        assert_eq!(
            instruction_frames(&handle.writes())
                .iter()
                .filter(|(_, inst)| *inst == REBOOT_INST)
                .count(),
            1,
            "the REBOOT instruction still has to reach the wire"
        );
    }

    /// A bus with one factory-fresh servo on it.
    ///
    /// It answers PING at whatever id it currently has, acknowledges writes, and answers reads
    /// out of a register file — and a write to the id register moves it, the way the real one
    /// does. That is what makes the adoption sequence assertable at all: it is the newest and
    /// least hardware-proven code in this module, and every step of it (find at id 1, unlock,
    /// re-address, re-check, reboot, come back) is a wire sequence nobody can eyeball.
    struct FreshServo {
        state: Arc<Mutex<FreshServoState>>,
    }

    struct FreshServoState {
        id: u8,
        regs: std::collections::HashMap<u8, u8>,
        sent: Vec<u8>,
        replies: Vec<u8>,
    }

    impl FreshServo {
        fn new() -> Self {
            Self {
                state: Arc::new(Mutex::new(FreshServoState {
                    id: FACTORY_ID,
                    regs: std::collections::HashMap::new(),
                    sent: Vec::new(),
                    replies: Vec::new(),
                })),
            }
        }
    }

    impl Transport for FreshServo {
        fn write_all(&mut self, bytes: &[u8]) -> std::io::Result<()> {
            let mut s = self.state.lock().unwrap();
            s.sent.extend_from_slice(bytes);
            // Requests here are whole frames; a servo addressed by one is the only one that
            // answers, and every other id stays silent.
            if bytes.len() < 6 || bytes[0] != 0xFF || bytes[1] != 0xFF || bytes[2] != s.id {
                return Ok(());
            }
            // The id a request was addressed to is the id that answers, even for the write
            // that changes it: `SCS::Ack` in the vendor's own library reads the reply's id and
            // fails with `ERR_SLAVE_ID` if it is not the one addressed.
            let addressed = bytes[2];
            match bytes[4] {
                feetech::inst::PING => {
                    let reply = ack(addressed, &[]);
                    s.replies.extend_from_slice(&reply);
                }
                feetech::inst::WRITE => {
                    let (addr, value) = (bytes[5], bytes[6]);
                    if addr == feetech::reg::ID {
                        s.id = value;
                    }
                    s.regs.insert(addr, value);
                    let reply = ack(addressed, &[]);
                    s.replies.extend_from_slice(&reply);
                }
                feetech::inst::READ => {
                    let (addr, want) = (bytes[5], bytes[6] as usize);
                    let value = *s.regs.get(&addr).unwrap_or(&0);
                    let reply = ack(addressed, &vec![value; want]);
                    s.replies.extend_from_slice(&reply);
                }
                // REBOOT (0x08) and anything else: no reply, by design.
                _ => {}
            }
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut s = self.state.lock().unwrap();
            if s.replies.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "nothing to say",
                ));
            }
            let n = buf.len().min(s.replies.len());
            buf[..n].copy_from_slice(&s.replies[..n]);
            s.replies.drain(..n);
            Ok(n)
        }
    }

    /// The whole motor-swap sequence, as bytes: a fresh servo answering at id 1 is re-addressed
    /// to the missing joint, and every EEPROM write is wrapped in the unlock/relock pair the
    /// vendor's own library uses — without it the new id would read back correctly and then be
    /// forgotten at the next power cycle, and the joint would go missing again.
    #[test]
    fn adopting_a_replacement_readdresses_a_fresh_servo_and_relocks_its_eeprom() {
        let id = JOINT_IDS[7];
        let servo = FreshServo::new();
        let handle = servo.state.clone();
        let mut io = FeetechIo::with_transport(Box::new(servo));

        assert!(
            io.adopt_replacement(id).expect("a fresh servo must be adopted"),
            "a servo at the factory id is a replacement"
        );

        let writes = write_frames(&handle.lock().unwrap().sent);
        assert_eq!(
            writes,
            vec![
                (FACTORY_ID, feetech::reg::LOCK, EEPROM_UNLOCK),
                (FACTORY_ID, feetech::reg::ID, id),
                // The relock goes to the *new* id: from the line above onwards the old one is
                // nobody, and a relock sent there would leave the servo unlocked.
                (id, feetech::reg::LOCK, EEPROM_LOCK),
                // And then the new servo is put where a running one is: torque off, ready to
                // be rebooted into a clean state.
                (id, feetech::reg::TORQUE_ENABLE, 0),
            ],
            "unlock at the old id, re-address, relock at the new id, then torque off"
        );

        let instructions = instruction_frames(&handle.lock().unwrap().sent);
        assert_eq!(
            instructions.first(),
            Some(&(FACTORY_ID, feetech::inst::PING)),
            "the sequence starts by looking for a fresh servo at id 1: {instructions:?}"
        );
        assert_eq!(
            instructions
                .iter()
                .filter(|(_, inst)| *inst == REBOOT_INST)
                .collect::<Vec<_>>(),
            vec![&(id, REBOOT_INST)],
            "exactly one REBOOT, addressed to the servo's new id: {instructions:?}"
        );
        // The ping that proves the servo came back is the one *after* the reboot, and it is
        // addressed to the new id — which is the whole point of the exercise.
        let reboot_at = instructions
            .iter()
            .position(|(_, inst)| *inst == REBOOT_INST)
            .unwrap();
        assert!(
            instructions[reboot_at + 1..].contains(&(id, feetech::inst::PING)),
            "the servo must be pinged under its new id after the reboot: {instructions:?}"
        );
        assert_eq!(handle.lock().unwrap().id, id, "the servo kept its new id");
    }

    /// The other half of the contract: nothing at the factory id means the servo is simply
    /// missing, and the caller is told so rather than left with a broken bus. An id that is not
    /// a joint at all is a programming error and is reported as one.
    #[test]
    fn adopting_a_replacement_reports_nothing_fresh_on_the_bus() {
        let mut io = FeetechIo::with_transport(Box::new(BurstTransport::new(Vec::new())));
        assert!(
            !io.adopt_replacement(JOINT_IDS[0]).expect("silence is not an error"),
            "nothing answers, so there is nothing to adopt"
        );
        assert!(
            io.adopt_replacement(99).is_err(),
            "99 is not a joint, and that is not a missing servo"
        );
    }

    fn block(n: u8) -> [u8; IMU_BLOCK_LEN] {
        [n; IMU_BLOCK_LEN]
    }

    /// The first block has no predecessor, so it cannot be a repeat of one. This is not a
    /// hypothetical: the natural initial value is all zeros, and an all-zero block is what a
    /// board sends before SFLP has written its table — which used to score a stale read on the
    /// very first tick of every boot, and put a permanent 1 in a counter rendered as an alarm.
    #[test]
    fn the_first_block_is_never_stale() {
        let mut t = StaleImuTracker::default();
        assert_eq!(t.observe(&block(0)), 0);
        assert_eq!(t.stale.total, 0);
    }

    /// Fresh blocks must leave both counters alone. The whole point of the run is that it means
    /// "right now", so anything that does not repeat has to clear it.
    #[test]
    fn fresh_blocks_count_for_nothing() {
        let mut t = StaleImuTracker::default();
        for n in 0..10 {
            assert_eq!(t.observe(&block(n)), 0);
        }
        assert_eq!(t.stale, ImuStale { total: 0, run: 0 });
    }

    /// A hiccup: two identical blocks, then the board recovers. The total remembers it — that
    /// is what makes "9 over 40 minutes" sayable — while the run goes back to zero, because
    /// orientation is live again and nothing should be shouting.
    #[test]
    fn a_hiccup_is_remembered_in_the_total_but_not_the_run() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(1));
        assert_eq!(t.observe(&block(1)), 1, "the repeat is the first of a run");
        assert_eq!(t.observe(&block(2)), 0, "a fresh block ends the run");
        assert_eq!(t.stale, ImuStale { total: 1, run: 0 });
    }

    /// A board that has stopped refreshing repeats forever, and the run is what separates that
    /// from the hiccup above. It has to reach the threshold the journal and the health report
    /// both key off, or a genuinely dead IMU is never reported at all.
    #[test]
    fn a_dead_board_runs_past_the_warning_threshold() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(7));
        for _ in 0..STALE_RUN_WARN {
            t.observe(&block(7));
        }
        assert_eq!(t.stale.run, STALE_RUN_WARN);
        assert_eq!(t.stale.total, STALE_RUN_WARN);
    }

    /// Runs accumulate into the same total across separate episodes: the total is "how often
    /// has this ever happened", not "how bad is it now".
    #[test]
    fn separate_episodes_add_up() {
        let mut t = StaleImuTracker::default();
        for n in 0..3u8 {
            t.observe(&block(n));
            t.observe(&block(n));
            t.observe(&block(n));
        }
        assert_eq!(t.stale.total, 6, "two repeats in each of three episodes");
        assert_eq!(t.stale.run, 2, "the last episode was still going");
    }
}
