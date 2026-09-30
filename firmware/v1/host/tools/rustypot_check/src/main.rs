//! Does the *real* rustypot accept what this firmware puts on the wire?
//!
//! The Fast Sync Read (0x8A) support in src/dxl2.c is only worth anything if the
//! consumer microduck actually uses - rustypot's `parse_fast_sync_read_status` -
//! reads the bytes back the way the firmware intended: one aggregate status
//! packet, blocks in id-list order, each block ending with the CRC of everything
//! up to and including its own data.
//!
//! The C tests in host/tools/test_protocols.c check the bytes the firmware emits
//! against a transcription of that parser plus the protocol 2.0 e-manual vector.
//! This harness closes the other half: it runs rustypot 1.8.0 itself, with a fake
//! serial port standing in for the bus, so the bytes the firmware produces go
//! through the same code path robotd uses (`Xl330Controller::with_fast_sync_read`
//! -> `sync_read_raw_data`).
//!
//! It is deliberately *not* part of `./build.sh test`: it needs the Rust
//! toolchain and the crates.io cache, while everything else there runs on plain
//! gcc and python. Run it by hand after touching the 0x8A path:
//!
//! ```text
//! cd firmware/v1/host/tools/rustypot_check
//! cargo run --release
//! ```
//!
//! The block of id 200 below is pinned in two places on purpose: the C test
//! asserts the firmware emits exactly these 24 bytes, and this harness feeds
//! exactly these 24 bytes to rustypot. A change to either side breaks one of them.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

/// microduck's tick read (duck-control/src/model.rs, `READ_ADDR`/`READ_LEN`).
const IMU_ID: u8 = 200;
const JOINT_IDS: [u8; 15] = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];
const READ_ADDR: u8 = 124;
const READ_LEN: u8 = 12;

/// The 24 bytes the firmware answers this read with: the 8-byte aggregate prefix
/// (`FF FF FD 00 FE LEN_L LEN_H 0x55`, LEN = 1 + 16 * 16 = 257 = 0x0101) and the
/// block of id 200 - ERROR, ID, the 12-byte telemetry window, and the CRC over
/// everything so far.  host/tools/test_protocols.c asserts this vector against
/// the real dxl2.c with its register image set to `0, 1, 2, ...`.
const NODE_REPLY: [u8; 24] = [
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x01, 0x01, 0x55,
    0x00, 0xC8, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x8E, 0xA1,
];

/// The instruction rustypot builds for that read, pinned by the same C test, so
/// the firmware is exercised with rustypot's own bytes and not with bytes that
/// merely resemble them.
const INSTRUCTION: [u8; 30] = [
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x17, 0x00, 0x8A, 0x7C, 0x00, 0x0C, 0x00, 0xC8, 0x14, 0x15,
    0x16, 0x17, 0x18, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x5F, 0x95,
];

/// ROBOTIS `update_crc` (poly 0x8005, init 0).  rustypot keeps its own copy
/// private, and this one only *builds* the stand-in blocks for the devices behind
/// the node; every block is then validated by rustypot's own CRC check, so this
/// cannot paper over a mistake in the firmware's CRC.
fn crc16_dxl(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for byte in data {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if 0 != crc & 0x8000 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// One device's contribution to the aggregate packet: ERROR, ID, DATA, then the
/// CRC of `upto` (prefix + earlier blocks) plus this block.
fn block(id: u8, data: &[u8], error: u8, upto: &[u8]) -> Vec<u8> {
    let mut out = vec![error, id];
    out.extend_from_slice(data);
    let mut full = upto.to_vec();
    full.extend_from_slice(&out);
    out.extend_from_slice(&crc16_dxl(&full).to_le_bytes());
    out
}

/// The payload for a device that is not on the bench: `0x10 + i, 0x11 + i, ...`,
/// which is what the assertions below expect to read back.
fn dummy(i: usize) -> Vec<u8> {
    (0..READ_LEN)
        .map(|b| 0x10u8.wrapping_add(i as u8).wrapping_add(b))
        .collect()
}

/// A serial port that writes into a buffer and hands out a canned answer *after*
/// the instruction has been written - which is also what makes `flush_if_needed`
/// see an empty input buffer, exactly as on a quiet bus.
#[derive(Clone)]
struct FakePort {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    /// Bytes handed to the next `read`.
    rx: VecDeque<u8>,
    /// What `read` will return once an instruction has been written.
    pending: VecDeque<u8>,
    /// Everything written by the controller, so the instruction can be checked.
    written: Vec<u8>,
}

impl FakePort {
    fn new(answer: Vec<u8>) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner {
                rx: VecDeque::new(),
                pending: answer.into(),
                written: Vec::new(),
            })),
        }
    }

    fn written(&self) -> Vec<u8> {
        self.inner.lock().unwrap().written.clone()
    }
}

impl Read for FakePort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut inner = self.inner.lock().unwrap();
        if inner.rx.is_empty() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "no reply queued"));
        }
        let n = buf.len().min(inner.rx.len());
        for slot in buf.iter_mut().take(n) {
            *slot = inner.rx.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl Write for FakePort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut inner = self.inner.lock().unwrap();
        inner.written.extend_from_slice(buf);
        if inner.rx.is_empty() && !inner.pending.is_empty() {
            let Inner { rx, pending, .. } = &mut *inner;
            rx.append(pending);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl SerialPort for FakePort {
    fn name(&self) -> Option<String> {
        Some("fake://fast-sync-read".to_owned())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(1_000_000)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(30)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(self.inner.lock().unwrap().rx.len() as u32)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(self.clone()))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

fn aggregate(ids: &[u8]) -> Vec<u8> {
    let mut packet = NODE_REPLY.to_vec();
    for (i, id) in JOINT_IDS.iter().enumerate() {
        let blk = block(*id, &dummy(i), 0, &packet);
        packet.extend_from_slice(&blk);
    }
    assert_eq!(
        packet.len(),
        8 + ids.len() * (READ_LEN as usize + 4),
        "the aggregate is one prefix plus one block per device"
    );
    packet
}

fn read_with(answer: Vec<u8>, ids: &[u8]) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
    let port = FakePort::new(answer);
    let mut controller = Xl330Controller::new()
        .with_protocol_v2()
        .with_fast_sync_read()
        .with_serial_port(Box::new(port.clone()));
    let values = controller.sync_read_raw_data(ids, READ_ADDR, READ_LEN);
    // The instruction is checked here rather than after the call so a failing
    // read still reports it.
    let written = port.written();
    if written != INSTRUCTION {
        println!("  note: rustypot wrote {} bytes: {:02X?}", written.len(), written);
    }
    assert_eq!(written, INSTRUCTION, "the instruction rustypot generates");
    values
}

fn main() {
    let mut failures = 0;
    let mut check = |name: &str, ok: bool| {
        println!("  {} {}", if ok { "ok  " } else { "FAIL" }, name);
        if !ok {
            failures += 1;
        }
    };

    let ids: Vec<u8> = std::iter::once(IMU_ID)
        .chain(JOINT_IDS.iter().copied())
        .collect();

    println!("rustypot {} vs the imu_to_dxl 0x8A bytes", "1.8.0");

    match read_with(aggregate(&ids), &ids) {
        Ok(values) => {
            check("rustypot parses the aggregate packet", true);
            check("sixteen blocks come back", values.len() == ids.len());
            check(
                "the IMU block is block 0, byte for byte",
                values[0] == NODE_REPLY[10..22].to_vec(),
            );
            check(
                "every other block is the one its device sent",
                values[1..]
                    .iter()
                    .enumerate()
                    .all(|(i, v)| *v == dummy(i)),
            );
        }
        Err(e) => {
            check("rustypot parses the aggregate packet", false);
            println!("       error: {e}");
        }
    }

    // Negative control: rustypot has to *reject* a corrupted block, or agreeing
    // with it would say nothing about the CRC this firmware appends.
    let mut broken = aggregate(&ids);
    broken[12] ^= 0x01;
    match read_with(broken, &ids) {
        Ok(_) => check("a flipped data byte is rejected", false),
        Err(_) => check("a flipped data byte is rejected", true),
    }

    if failures != 0 {
        println!("\n{failures} FAILURE(S)");
        std::process::exit(1);
    }
    println!("\nthe firmware's 0x8A bytes are exactly what rustypot reads");
}
