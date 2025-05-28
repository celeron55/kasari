#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{AnyPin, Level, Output, OutputConfig, Pin},
    timer::timg::TimerGroup,
    uart::{Config, RxConfig, Uart, UartRx, UartTx},
    Async,
};
use esp_println::println;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use static_cell::StaticCell; // For allocator

const PACKET_SIZE: usize = 22;
const HEAD_BYTE: u8 = 0xFA;

// Assume READ_BUF_SIZE is defined elsewhere (e.g., 64)
const READ_BUF_SIZE: usize = 64;

#[embassy_executor::task]
async fn writer(mut tx: UartTx<'static, Async>, signal: &'static Signal<NoopRawMutex, usize>) {
    use core::fmt::Write;
    embedded_io_async::Write::write(&mut tx, b"Hello async serial\r\n")
        .await
        .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, Async>, signal: &'static Signal<NoopRawMutex, usize>) {
    const MAX_BUFFER_SIZE: usize = 2 * READ_BUF_SIZE + 16;
    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    let mut ring_buf: ConstGenericRingBuffer<u8, MAX_BUFFER_SIZE> = ConstGenericRingBuffer::new();
    let mut log_i: u32 = 0;

    loop {
        // Original reading method
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                // Push read data into ring buffer
                for &byte in &rbuf[..offset] {
                    ring_buf.push(byte);
                }
                //println!("Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;

                // Process complete packets
                while ring_buf.len() >= PACKET_SIZE {
                    // Check for packet header
                    if ring_buf.get(0) != Some(&HEAD_BYTE) {
                        // Skip until HEAD_BYTE
                        while let Some(byte) = ring_buf.get(0) {
                            if *byte == HEAD_BYTE {
                                break;
                            }
                            ring_buf.dequeue();
                        }
                        continue;
                    }

                    // Extract 22-byte packet
                    let mut packet = [0u8; PACKET_SIZE];
                    for i in 0..PACKET_SIZE {
                        packet[i] = ring_buf.get(i).copied().unwrap_or(0);
                    }

                    // Parse and print packet
                    if let Some(parsed) = parse_packet(&packet) {
                        log_i += 1;
                        if log_i % 20 == 1 {
                            println!(
                                "Packet {}: distances=[{}, {}, {}, {}] mm",
                                log_i,
                                parsed.distances[0],
                                parsed.distances[1],
                                parsed.distances[2],
                                parsed.distances[3]
                            );
                        }
                    } else {
                        println!("Invalid packet");
                    }

                    // Remove processed packet
                    for _ in 0..PACKET_SIZE {
                        ring_buf.dequeue();
                    }
                    signal.signal(PACKET_SIZE);
                }
            }
            Err(e) => println!("RX Error: {:?}", e),
        }
    }
}

// Parsed packet data
struct ParsedPacket {
    angle: u16,          // degrees (0–360)
    speed: u16,          // RPM
    distances: [u16; 4], // mm
}

// Parse a 22-byte packet
fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE || packet[0] != HEAD_BYTE {
        return None;
    }

    // Angle: byte 1, (ANGLE - 0xA0) * 4
    let angle = ((packet[1] as u16).wrapping_sub(0xA0)) * 4;
    if angle > 360 {
        return None;
    }

    // Speed: bytes 2 (LSB), 3 (MSB), (MSB << 8) | LSB / 64
    let speed = ((packet[3] as u16) << 8) | (packet[2] as u16);
    let speed = speed / 64;

    // Distances: bytes 4–19, 4 sets of (LSB, MSB & 0x3F)
    let mut distances = [0u16; 4];
    for (i, dist) in distances.iter_mut().enumerate() {
        let idx = 4 + i * 4;
        *dist = ((packet[idx + 1] as u16 & 0x3F) << 8) | (packet[idx] as u16);
    }

    // Checksum: bytes 20 (LSB), 21 (MSB)
    let received_sum = ((packet[21] as u16) << 8) | (packet[20] as u16);
    let computed_sum = compute_checksum(&packet[..PACKET_SIZE - 2]);
    if computed_sum != received_sum {
        return None;
    }

    Some(ParsedPacket {
        angle,
        speed,
        distances,
    })
}

// Compute checksum for packet (excluding last 2 bytes)
fn compute_checksum(data: &[u8]) -> u16 {
    let mut chk32: u32 = 0;
    for chunk in data.chunks(2) {
        let word = if chunk.len() == 2 {
            ((chunk[1] as u16) << 8) + (chunk[0] as u16)
        } else {
            chunk[0] as u16
        };
        chk32 = (chk32 << 1) + (word as u32);
    }
    let checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    (checksum & 0x7FFF) as u16
}

#[embassy_executor::task]
async fn encoder_emulation_task(encoder_emulation_output_pinmap: AnyPin) {
    let mut encoder_emulation_output_pin = Output::new(
        encoder_emulation_output_pinmap,
        Level::High,
        OutputConfig::default(),
    );

    loop {
        // 10+10ms seems to be the lowest we can go, resulting in reported
        // 198 RPM and thus about 397 reports per second, each containing 4
        // measurements, hopefully measured at a consistent 1188/s interval
        encoder_emulation_output_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        encoder_emulation_output_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // GPIO

    let encoder_emulation_output_pinmap = peripherals.GPIO13.degrade();

    // UART2

    // Default pins for Uart communication
    let (tx_pin, rx_pin) = (peripherals.GPIO17, peripherals.GPIO16);

    let config = Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(READ_BUF_SIZE as u16))
        .with_baudrate(115200);

    let uart2 = Uart::new(peripherals.UART2, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();

    let (rx, tx) = uart2.split();

    // Spawn tasks

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
    spawner
        .spawn(encoder_emulation_task(encoder_emulation_output_pinmap))
        .ok();
}
