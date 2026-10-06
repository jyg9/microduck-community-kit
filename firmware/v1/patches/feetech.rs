//! FeeTech SCS/HLS serial protocol, as spoken by the HD-1910-C001 joint servo
//! and by the `imu_to_dxl` node's FeeTech personality.
//!
//! This module is deliberately dependency-free: it is pure framing and
//! conversion code with unit tests, so it can be dropped into `duck-control`
//! (or any other crate) without pulling anything in. See
//! `firmware/v1/patches/README.md` for how `bus.rs`
//! uses it, and `飞特通讯协议/飞特通讯协议说明.md` for the protocol write-up.
//!
//! Frame layout (all little endian for multi-byte fields):
//!
//! ```text
//! instruction: FF FF ID LEN INST [ADDR] [DATA...] ~SUM
//!              LEN = 1 (INST) + ADDR? + DATA + 1 (SUM)
//! ack:         FF FF ID LEN STATUS [DATA...] ~SUM
//!              LEN = 1 (STATUS) + DATA + 1 (SUM)
//! SUM          = ID + LEN + INST + ADDR + ΣDATA    (ADDR counts as 0 when absent)
//! ```
//!
//! Sources: `FTServo_Linux-main/src/SCS.cpp` (`writeBuf`, `Ack`, `Read`,
//! `syncReadPacketTx`, `syncWrite`), `INST.h` and `HLSCL.h`.

/// Sum that goes into the checksum: every byte from ID up to (but excluding)
/// the checksum itself. `ADDR` is included only when the frame carries one, and
/// the reference library adds it as 0 for the instructions without it.
fn checksum_sum(id: u8, len: u8, inst: u8, addr: u8, data: &[u8]) -> u8 {
    let mut sum = id
        .wrapping_add(len)
        .wrapping_add(inst)
        .wrapping_add(addr);
    for byte in data {
        sum = sum.wrapping_add(*byte);
    }
    sum
}

/// The byte actually sent: bitwise complement of [`checksum_sum`].
pub fn checksum(id: u8, len: u8, inst: u8, addr: u8, data: &[u8]) -> u8 {
    !checksum_sum(id, len, inst, addr, data)
}

/// Instruction set (`INST.h`).
pub mod inst {
    pub const PING: u8 = 0x01;
    pub const READ: u8 = 0x02;
    pub const WRITE: u8 = 0x03;
    pub const REG_WRITE: u8 = 0x04;
    pub const REG_ACTION: u8 = 0x05;
    pub const RECOVERY: u8 = 0x06;
    pub const SYNC_READ: u8 = 0x82;
    pub const SYNC_WRITE: u8 = 0x83;
    pub const RESET: u8 = 0x0A;
    pub const CAL: u8 = 0x0B;
}

/// HLS memory table addresses (`HLSCL.h`).
pub mod reg {
    pub const MODEL_L: u8 = 3;
    pub const MODEL_H: u8 = 4;
    pub const ID: u8 = 5;
    pub const BAUD_RATE: u8 = 6;
    pub const SECOND_ID: u8 = 7;
    pub const MIN_ANGLE_LIMIT_L: u8 = 9;
    pub const MAX_ANGLE_LIMIT_L: u8 = 11;
    pub const CW_DEAD: u8 = 26;
    pub const CCW_DEAD: u8 = 27;
    pub const OFS_L: u8 = 31;
    pub const MODE: u8 = 33;
    pub const TORQUE_ENABLE: u8 = 40;
    pub const ACC: u8 = 41;
    pub const GOAL_POSITION_L: u8 = 42;
    pub const GOAL_TORQUE_L: u8 = 44;
    pub const GOAL_SPEED_L: u8 = 46;
    pub const TORQUE_LIMIT_L: u8 = 48;
    pub const LOCK: u8 = 55;
    pub const PRESENT_POSITION_L: u8 = 56;
    pub const PRESENT_SPEED_L: u8 = 58;
    pub const PRESENT_LOAD_L: u8 = 60;
    pub const PRESENT_VOLTAGE: u8 = 62;
    pub const PRESENT_TEMPERATURE: u8 = 63;
    pub const MOVING: u8 = 66;
    pub const PRESENT_CURRENT_L: u8 = 69;
}

pub const BROADCAST_ID: u8 = 0xFE;
/// Largest frame the protocol can produce (LEN is a u8).
pub const MAX_FRAME: usize = 4 + 255;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A frame arrived but the header/length/checksum did not hold together.
    BadFrame(&'static str),
    /// The reply carried a different id than the one addressed.
    WrongId { got: u8, want: u8 },
    /// The device answered with a non-zero status byte.
    DeviceStatus(u8),
    /// Fewer bytes came back than the protocol demanded.
    ShortData { got: usize, want: usize },
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::BadFrame(why) => write!(f, "bad feetech frame: {why}"),
            Error::WrongId { got, want } => write!(f, "feetech reply from id {got}, expected {want}"),
            Error::DeviceStatus(status) => write!(f, "feetech device status {status:#04x}"),
            Error::ShortData { got, want } => write!(f, "feetech data {got} bytes, expected {want}"),
        }
    }
}

impl std::error::Error for Error {}

// ── packet construction ─────────────────────────────────────────────────────

/// Instruction with no ADDR (PING, REG_ACTION, RESET, CAL):
/// `LEN = 2`, and the checksum includes ADDR as zero.
pub fn instruction_no_addr(id: u8, instruction: u8) -> Vec<u8> {
    let len = 2u8;
    let sum = checksum(id, len, instruction, 0, &[]);
    vec![0xFF, 0xFF, id, len, instruction, sum]
}

/// Instruction with ADDR + DATA: `LEN = DATA + 3`.
pub fn instruction(id: u8, instruction: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    let len = (data.len() as u8).wrapping_add(3);
    let sum = checksum(id, len, instruction, addr, data);
    let mut frame = Vec::with_capacity(6 + data.len());
    frame.extend_from_slice(&[0xFF, 0xFF, id, len, instruction, addr]);
    frame.extend_from_slice(data);
    frame.push(sum);
    frame
}

pub fn ping(id: u8) -> Vec<u8> {
    instruction_no_addr(id, inst::PING)
}

/// READ: data is `[len]`, so `LEN = 4`.
pub fn read(id: u8, addr: u8, len: u8) -> Vec<u8> {
    instruction(id, inst::READ, addr, &[len])
}

pub fn write(id: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    instruction(id, inst::WRITE, addr, data)
}

pub fn reg_write(id: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    instruction(id, inst::REG_WRITE, addr, data)
}

pub fn reg_action(id: u8) -> Vec<u8> {
    instruction_no_addr(id, inst::REG_ACTION)
}

pub fn reset(id: u8) -> Vec<u8> {
    instruction_no_addr(id, inst::RESET)
}

/// SYNC_READ: `FF FF FE (IDN+4) 82 ADDR nLen ID... ~SUM`.
pub fn sync_read(ids: &[u8], addr: u8, len: u8) -> Vec<u8> {
    let frame_len = (ids.len() as u8).wrapping_add(4);
    let mut sum = 0xFEu8
        .wrapping_add(frame_len)
        .wrapping_add(inst::SYNC_READ)
        .wrapping_add(addr)
        .wrapping_add(len);
    let mut frame = Vec::with_capacity(7 + ids.len());
    frame.extend_from_slice(&[0xFF, 0xFF, BROADCAST_ID, frame_len, inst::SYNC_READ, addr, len]);
    for id in ids {
        frame.push(*id);
        sum = sum.wrapping_add(*id);
    }
    frame.push(!sum);
    frame
}

/// SYNC_WRITE: `FF FF FE ((nLen+1)*IDN+4) 83 ADDR nLen ID DATA... ~SUM`.
pub fn sync_write(ids: &[u8], addr: u8, payloads: &[&[u8]]) -> Result<Vec<u8>, Error> {
    if ids.is_empty() || ids.len() != payloads.len() {
        return Err(Error::BadFrame("sync_write ids/payload mismatch"));
    }
    let width = payloads[0].len();
    if payloads.iter().any(|p| p.len() != width) {
        return Err(Error::BadFrame("sync_write payload width mismatch"));
    }
    let width_u8 = u8::try_from(width).map_err(|_| Error::BadFrame("sync_write payload too wide"))?;
    let frame_len = ((width as u32 + 1) * ids.len() as u32 + 4) as u8; // wrapping is fine: protocol limit
    let mut sum = 0xFEu8
        .wrapping_add(frame_len)
        .wrapping_add(inst::SYNC_WRITE)
        .wrapping_add(addr)
        .wrapping_add(width_u8);
    let mut frame = vec![
        0xFF,
        0xFF,
        BROADCAST_ID,
        frame_len,
        inst::SYNC_WRITE,
        addr,
        width_u8,
    ];
    for (id, payload) in ids.iter().zip(payloads) {
        frame.push(*id);
        sum = sum.wrapping_add(*id);
        for byte in *payload {
            frame.push(*byte);
            sum = sum.wrapping_add(*byte);
        }
    }
    frame.push(!sum);
    Ok(frame)
}

// ── packet parsing ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ack {
    pub id: u8,
    pub status: u8,
    pub data: Vec<u8>,
}

pub fn parse_ack(frame: &[u8]) -> Result<Ack, Error> {
    if frame.len() < 6 {
        return Err(Error::BadFrame("shorter than a ping ack"));
    }
    if frame[0] != 0xFF || frame[1] != 0xFF {
        return Err(Error::BadFrame("missing header"));
    }
    let len = frame[3] as usize;
    if len < 2 {
        return Err(Error::BadFrame("LEN < 2"));
    }
    if frame.len() != len + 4 {
        return Err(Error::BadFrame("LEN does not match frame length"));
    }
    let body = &frame[2..frame.len() - 1];
    let expect = !body.iter().fold(0u8, |acc, b| acc.wrapping_add(*b));
    if expect != frame[frame.len() - 1] {
        return Err(Error::BadFrame("checksum"));
    }
    Ok(Ack {
        id: frame[2],
        status: frame[4],
        data: frame[5..frame.len() - 1].to_vec(),
    })
}

/// Total frame length if `buf` starts with a complete frame.
pub fn frame_len(buf: &[u8]) -> Option<usize> {
    if buf.len() < 4 {
        return None;
    }
    if buf[0] != 0xFF || buf[1] != 0xFF || buf[3] < 2 {
        return None;
    }
    let total = buf[3] as usize + 4;
    (buf.len() >= total).then_some(total)
}

/// Reassembles frames out of a byte stream: debug text, partial frames and
/// noise are tolerated, exactly like the C node does on the other side.
#[derive(Debug, Default)]
pub struct FrameReader {
    buf: Vec<u8>,
}

impl FrameReader {
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Bytes discarded while resynchronising (for the health counters).
    pub fn next_frame(&mut self) -> Option<Result<Vec<u8>, usize>> {
        loop {
            let Some(start) = self.buf.windows(2).position(|w| w == [0xFF, 0xFF]) else {
                let dropped = self.buf.len().saturating_sub(1);
                self.buf.drain(..dropped);
                return None;
            };
            if start > 0 {
                self.buf.drain(..start);
            }
            if self.buf.len() < 4 {
                return None;
            }
            let total = self.buf[3] as usize + 4;
            if self.buf[3] < 2 || total > MAX_FRAME {
                // not a plausible frame: step past this header
                self.buf.remove(0);
                continue;
            }
            if self.buf.len() < total {
                return None;
            }
            let frame: Vec<u8> = self.buf.drain(..total).collect();
            return Some(parse_ack(&frame).map(|_| frame).map_err(|_| total));
        }
    }

    pub fn clear(&mut self) {
        self.buf.clear();
    }

    pub fn buffered(&self) -> usize {
        self.buf.len()
    }
}

// ── conversions ─────────────────────────────────────────────────────────────

/// One 16-bit field as a signed number, in the vendor's **sign-magnitude**
/// encoding: one direction bit plus a magnitude, *not* a two's-complement `i16`.
///
/// This is `HLSCL::ReadPos`, `ReadSpeed`, `ReadLoad` and `ReadCurrent` in
/// `FTServo_Linux-main/src/HLSCL.cpp`, which are the same two lines with a
/// different bit — `if (v & (1 << bit)) v = -(v & ~(1 << bit))`. The HLS memory
/// table names the bit for each field (`BIT15 为方向位` on 当前位置/当前速度/
/// 当前电流/目标位置, `BIT10 为方向位` on 当前负载), and the servo debugger in
/// `software/hls_servo_debugger/` encodes all of them the same way.
///
/// Reading one of these as `i16::from_le_bytes` is not a rounding difference:
/// a direction bit set on a small magnitude turns into a number near the negative
/// end of the range, so `-100 counts` reads as `-32668`.
pub fn sign_magnitude(raw: u16, direction_bit: u8) -> i32 {
    let magnitude = (raw & !(1u16 << direction_bit)) as i32;
    if raw & (1u16 << direction_bit) != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Position is a 15-bit signed count: bit 15 is the direction flag and bits
/// 14..0 the magnitude (`HLSCL::ReadPos`).
pub fn position_raw_to_counts(raw: u16) -> i32 {
    sign_magnitude(raw, 15)
}

pub fn position_counts_to_raw(counts: i32) -> u16 {
    let magnitude = counts.unsigned_abs().min(0x7FFF) as u16;
    if counts < 0 {
        magnitude | 0x8000
    } else {
        magnitude
    }
}

/// Counts per revolution.
///
/// The HLS memory table ("磁编码HLS舵机-内存表解析", §2.3/§2.4) gives the unit of
/// both `目标位置` and `当前位置` as **0.087°**, i.e. 4096 counts per 360°, and
/// the 16-bit field as `-32767..32767` with bit 15 as the direction bit — so the
/// range is multi-turn (about ±8 revolutions), not one turn in 15 bits.
///
/// Getting this wrong is an 8x error on every joint angle: an earlier revision of
/// this file used 32768 counts/rev from a second-hand note, which the vendor
/// table contradicts.  Re-confirm against the HD-1910-C001 sheet before the first
/// hardware run, but 0.087°/count is what the HLS documentation says.
pub const POSITION_COUNTS_PER_REV: f64 = 4096.0;

pub const RAD_PER_COUNT: f64 = std::f64::consts::TAU / POSITION_COUNTS_PER_REV;

/// RPM per speed count (`60 * 0.732 = 43.92 rpm` in the reference example).
pub const RPM_PER_SPEED_COUNT: f64 = 0.732;

pub const RAD_PER_SEC_PER_SPEED_COUNT: f64 = RPM_PER_SPEED_COUNT * std::f64::consts::TAU / 60.0;

/// Milliamps per current count (`Torque = 300 * 6.5 = 1950 mA` in the example).
pub const MA_PER_CURRENT_COUNT: f64 = 6.5;

/// Volts per count of `PRESENT_VOLTAGE` (same scale as the Dynamixel family).
pub const VOLTS_PER_COUNT: f64 = 0.1;

/// One little-endian `u16` out of two wire bytes.
///
/// There is deliberately **no** `le_i16` companion: every signed field this
/// protocol carries is the direction bit of [`sign_magnitude`] rather than two's
/// complement, and this module used to hand speed and current out through one.
pub fn le_u16(low: u8, high: u8) -> u16 {
    u16::from_le_bytes([low, high])
}

pub fn position_rad(low: u8, high: u8) -> f64 {
    position_raw_to_counts(le_u16(low, high)) as f64 * RAD_PER_COUNT
}

pub fn position_from_rad(rad: f64) -> [u8; 2] {
    let counts = (rad / RAD_PER_COUNT).round() as i32;
    position_counts_to_raw(counts).to_le_bytes()
}

// ── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// The same vectors the firmware asserts in `host/tools/test_protocols.c`
    /// and the host tool in `host/bus.py`, so all three implementations agree
    /// byte for byte on the wire.
    #[test]
    fn ping_frame_matches_the_firmware_vector() {
        assert_eq!(ping(200), vec![0xFF, 0xFF, 0xC8, 0x02, 0x01, 0x34]);
    }

    #[test]
    fn read_frame_matches_the_firmware_vector() {
        assert_eq!(read(200, 56, 12), vec![0xFF, 0xFF, 0xC8, 0x04, 0x02, 0x38, 0x0C, 0xED]);
    }

    #[test]
    fn ack_ping_from_the_node_parses() {
        let ack = parse_ack(&[0xFF, 0xFF, 200, 0x02, 0x00, 0x35]).unwrap();
        assert_eq!(ack, Ack { id: 200, status: 0, data: vec![] });
    }

    /// The node answers a 12-byte telemetry read with `LEN = 14`; the payload
    /// must come back untouched.
    #[test]
    fn ack_with_data_parses() {
        let payload: Vec<u8> = (0..12).collect();
        let mut frame = vec![0xFF, 0xFF, 200, 14, 0x00];
        frame.extend_from_slice(&payload);
        let sum = !frame[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        frame.push(sum);
        let ack = parse_ack(&frame).unwrap();
        assert_eq!(ack.data, payload);
    }

    #[test]
    fn ack_rejects_a_corrupted_checksum() {
        let mut frame = vec![0xFF, 0xFF, 200, 0x02, 0x00, 0x35];
        *frame.last_mut().unwrap() ^= 0xFF;
        assert!(matches!(parse_ack(&frame), Err(Error::BadFrame("checksum"))));
    }

    /// The sync frames must reproduce the library's LEN formulas exactly:
    /// `IDN + 4` for sync read and `(nLen + 1) * IDN + 4` for sync write.
    #[test]
    fn sync_read_length_formula() {
        for ids in [vec![1u8], vec![1, 2], vec![200, 10, 11, 12]] {
            let frame = sync_read(&ids, 56, 12);
            assert_eq!(frame[3] as usize, ids.len() + 4);
            assert_eq!(frame.len(), frame[3] as usize + 4);
        }
    }

    #[test]
    fn sync_write_length_formula() {
        let payload = [0u8; 2];
        let frame = sync_write(&[1, 2, 3], 42, &[&payload, &payload, &payload]).unwrap();
        assert_eq!(frame[3] as usize, (2 + 1) * 3 + 4);
        assert_eq!(frame.len(), frame[3] as usize + 4);
        // rebuilding the same checksum by hand must agree
        let body = &frame[2..frame.len() - 1];
        assert_eq!(!body.iter().fold(0u8, |a, b| a.wrapping_add(*b)), *frame.last().unwrap());
    }

    #[test]
    fn sync_write_rejects_mismatched_payloads() {
        assert!(sync_write(&[1, 2], 42, &[&[0u8; 2][..]]).is_err());
        assert!(sync_write(&[], 42, &[]).is_err());
    }

    #[test]
    fn frame_reader_resynchronises_past_debug_text() {
        let mut reader = FrameReader::new();
        reader.push(b"# stats 12\r\n");
        reader.push(&ping(200));
        reader.push(b"trailing");
        let frame = reader.next_frame().unwrap().unwrap();
        assert_eq!(parse_ack(&frame).unwrap().id, 200);
        assert!(reader.next_frame().is_none());
        // resynchronising keeps only a possible partial header
        assert!(reader.buffered() <= 1);
    }

    #[test]
    fn frame_reader_waits_for_a_complete_frame() {
        let mut reader = FrameReader::new();
        let ack = [0xFF, 0xFF, 200, 0x02, 0x00, 0x35];
        reader.push(&ack[..3]);
        assert!(reader.next_frame().is_none());
        reader.push(&ack[3..]);
        assert!(reader.next_frame().unwrap().is_ok());
    }

    /// 15-bit sign-magnitude position: a bare `as i16` would read bit 15 as a
    /// large positive count instead of a direction flag.
    #[test]
    fn position_uses_the_direction_bit() {
        assert_eq!(position_raw_to_counts(0x1234), 0x1234);
        assert_eq!(position_raw_to_counts(0x9234), -0x1234);
        assert_eq!(position_raw_to_counts(0x8000), 0);
        assert_eq!(position_counts_to_raw(-0x1234), 0x9234);
        assert_eq!(position_counts_to_raw(0x1234), 0x1234);
    }

    #[test]
    fn position_radians_round_trip() {
        for rad in [-3.0, 0.0, 0.5, 3.0] {
            let [low, high] = position_from_rad(rad);
            let back = position_rad(low, high);
            assert!((back - rad).abs() < RAD_PER_COUNT, "{rad} -> {back}");
        }
    }

    #[test]
    fn documented_scales_match_the_vendor_memory_table() {
        // 60 speed counts -> 43.92 rpm, the example in the FeeTech sources
        assert!((60.0 * RPM_PER_SPEED_COUNT - 43.92).abs() < 1e-9);
        // 300 current counts -> 1950 mA
        assert!((300.0 * MA_PER_CURRENT_COUNT - 1950.0).abs() < 1e-9);
        // 0.087 degrees per count: a quarter turn is 1024 counts
        let quarter = 1024.0 * RAD_PER_COUNT;
        assert!((quarter - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{quarter}");
        // one full turn is 4096 counts
        assert!((POSITION_COUNTS_PER_REV * RAD_PER_COUNT - std::f64::consts::TAU).abs() < 1e-12);
        // and the 15-bit magnitude spans about eight turns (multi-turn field)
        let span_turns =
            (position_raw_to_counts(0xFFFF) as f64 * RAD_PER_COUNT / std::f64::consts::TAU).abs();
        assert!((7.99..=8.0).contains(&span_turns), "{span_turns} turns");
    }

    /// Every signed field the protocol carries uses a direction bit, and getting
    /// the decoder wrong is not an offset — it is a number near the far end of
    /// the range. Position, speed and current put the bit at 15; load puts it at
    /// 10, which is why the bit is a parameter rather than a constant.
    #[test]
    fn signed_fields_use_the_vendors_direction_bit() {
        // The three bit-15 fields, in the vendor's encoding.
        assert_eq!(sign_magnitude(0x0064, 15), 100);
        assert_eq!(sign_magnitude(0x8064, 15), -100);
        assert_eq!(sign_magnitude(0x8000, 15), 0);
        // The same wire bytes read two's complement would be a different number:
        // this is the bug the direction bit exists to rule out.
        assert_eq!(i16::from_le_bytes([0x64, 0x80]), -32668);
        assert_ne!(sign_magnitude(0x8064, 15), i16::from_le_bytes([0x64, 0x80]) as i32);
        // Load's direction bit is 10, not 15 (`HLSCL::ReadLoad`).
        assert_eq!(sign_magnitude(0x0400, 10), 0);
        assert_eq!(sign_magnitude(0x0064, 10), 100);
        assert_eq!(sign_magnitude(0x0464, 10), -100);
        assert_eq!(sign_magnitude(0x0464, 15), 0x0464);
    }

    #[test]
    fn voltages_and_temperatures_are_plain_counts() {
        assert!((le_u16(0x50, 0x00) as f64 * VOLTS_PER_COUNT - 8.0).abs() < 1e-9);
    }
}
