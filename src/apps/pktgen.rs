#![feature(duration_as_u128)]
extern crate ixgbe;

use std::env::args;
use std::process;
use std::slice;
use std::time::{Duration, Instant};

const BATCH_SIZE: u32 = 64;
const PKT_SIZE: u16 = 60;
const NUM_BUFS: u32 = 1 << 11;
const ENTRY_SIZE: u32 = 1 << 11;

fn calc_ip_checksum(data: &[u8]) -> u16 {
    let len = data.len();

    if len % 1 != 0 {
        panic!("odd-sized checksums NYI"); // we don't need that
    }
    let mut cs: u32 = 0;
    for i in 0..len / 2 {
        cs = cs + (data[i + i] as u32) + (data[i + i + 1] as u32) << 8;
        if cs > 0xFFFF {
            cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
        }
    }
    !(cs as u16)
}

const PKT_DATA: [u8; 45] = [
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06, // dst MAC
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16, // src MAC
    0x08,
    0x00, // ether type: IPv4
    0x45,
    0x00,                           // Version, IHL, TOS
    ((PKT_SIZE - 14) >> 8) as u8,   // ip len excluding ethernet, high byte
    ((PKT_SIZE - 14) & 0xFF) as u8, // ip len exlucding ethernet, low byte
    0x00,
    0x00,
    0x00,
    0x00, // id, flags, fragmentation
    0x40,
    0x11,
    0x00,
    0x00, // TTL (64), protocol (UDP), checksum
    0x0A,
    0x00,
    0x00,
    0x01, // src ip (10.0.0.1)
    0x0A,
    0x00,
    0x00,
    0x02, // dst ip (10.0.0.2)
    0x00,
    0x2A,
    0x05,
    0x39,                                // src and dst ports (42 -> 1337)
    ((PKT_SIZE - 20 - 14) >> 8) as u8,   // udp len excluding ip & ethernet, high byte
    ((PKT_SIZE - 20 - 14) & 0xFF) as u8, // udp len exlucding ip & ethernet, low byte
    0x00,
    0x00, // udp checksum, optional
    'i' as u8,
    'x' as u8,
    'y' as u8, // payload
               // rest of the payload is zero-filled because mempools guarantee empty bufs
];

fn main() {
    // Prints each argument on a separate line
    if args().len() != 2 {
        println!("Usage: {} <pci bus id>\n", args().nth(0).unwrap());
        process::exit(1);
    }

    let mut mempool = ixgbe::Mempool::new(NUM_BUFS as u16, ENTRY_SIZE);
    {
        let mut bufs = Vec::new();
        for _ in 0..NUM_BUFS {
            let buf = mempool.alloc_buf().unwrap();
            unsafe {
                let data_offset = (*buf).data_offset();
                (*buf).size = PKT_SIZE as u32;
                (*buf).mempool = &mut mempool;
                std::intrinsics::copy(PKT_DATA.as_ptr(), data_offset, PKT_DATA.len());
                let chk_ptr = (data_offset as usize + 24) as *mut u16;
                *chk_ptr = calc_ip_checksum(slice::from_raw_parts(
                    (data_offset as usize + 14) as *const u8,
                    20,
                ));
            }
            bufs.push(buf);
        }
        for i in 0..NUM_BUFS {
            mempool.free_buf(bufs[i as usize]);
        }
    }

    let mut device = ixgbe::DeviceInfo::new(&args().nth(1).unwrap(), 1, 1);
    let mut last_stats_printed = Instant::now();
    let mut old_stats = ixgbe::Stats::new();
    let mut stats = ixgbe::Stats::new();
    let mut counter = 0;
    let mut seq_num = 0;

    loop {
        let bufs = mempool.alloc_buf_batch(BATCH_SIZE);
        for i in 0..BATCH_SIZE {
            unsafe {
                let seq_ptr = ((*bufs[i as usize]).data_offset() as usize + PKT_SIZE as usize - 4)
                    as *mut u32;
                *seq_ptr = seq_num;
            }
            seq_num += 1;
        }
        device.tx_batch_busy_wait(bufs);

        if (counter & 0xfff) == 0 {
            let t = Instant::now();
            if (t.duration_since(last_stats_printed)) > Duration::new(1, 0) {
                stats.update(&device);
                ixgbe::print_stats_diff(
                    stats,
                    old_stats,
                    t.duration_since(last_stats_printed).as_nanos() as u64,
                );
                old_stats = stats;
                //stats.print_since_last();
                last_stats_printed = t;
            }
        }
        counter = counter + 1;
    }
}
