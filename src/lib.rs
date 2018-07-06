#![feature(const_fn)]
#![feature(untagged_unions)]
#![feature(libc)]
#![allow(unused_assignments)]

extern crate core;
extern crate sysconf;
extern crate libc;
extern crate static_assertions;

use std::thread::sleep;
use std::time::Duration;

// macro_rules! offset_of {
//     ($ty:ty, $field:ident) => {
//         unsafe { &(*(0 as *const $ty)).$field } as *const _ as usize
//     }
// }

#[macro_use]
mod types;
mod device;
mod memory;
mod pci;

pub use self::memory::Mempool;
pub use self::device::DeviceInfo;


const SIZE_PKT_BUF_HEADROOM: u32 = 40;

#[repr(C)]
pub struct Buffer {
	// physical address to pass a buffer to a nic
	buf_addr_phys : usize,
	pub mempool: *mut Mempool,
    idx: u32,
	pub size: u32,
	head_room: [u8; SIZE_PKT_BUF_HEADROOM as usize],
}

impl Buffer {
    pub fn data_offset(&self) -> *mut u8 {
        ((self as *const _ as usize) + core::mem::size_of::<Buffer>()) as *mut u8 
    }

    pub fn free_buf(&mut self) {
        unsafe { (*self.mempool).free_buf(self); }
    }
}


#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Stats {
    rx_pkts: u32,
    tx_pkts: u32,
    rx_bytes: u64,
    tx_bytes: u64
}

fn diff_mpps(pkts_new: u32, pkts_old: u32, nanos: u64) -> f64 {
	(pkts_new as f64 - pkts_old as f64)  / 1000000.0 / (nanos as f64 / 1000000000.0)
}

fn diff_mbit(bytes_new: u64, bytes_old: u64, pkts_new: u32, pkts_old: u32, nanos: u64) -> f64 {
	// take stuff on the wire into account, i.e., the preamble, SFD and IFG (20 bytes)
	// otherwise it won't show up as 10000 mbit/s with small packets which is confusing
	((bytes_new as f64 - bytes_old as f64)  / 1000000.0 / (nanos as f64 / 1000000000.0)) * 8.0
		+ diff_mpps(pkts_new, pkts_old, nanos) * 160.0
}

pub fn print_stats_diff(stats_new: Stats, stats_old: Stats, nanos: u64) {
	println!("RX: {} Mbit/s {} Mpps\n",
		diff_mbit(stats_new.rx_bytes, stats_old.rx_bytes, stats_new.rx_pkts, stats_old.rx_pkts, nanos),
		diff_mpps(stats_new.rx_pkts, stats_old.rx_pkts, nanos)
	);
	println!("TX: {} Mbit/s {} Mpps\n",
		diff_mbit(stats_new.tx_bytes, stats_old.tx_bytes, stats_new.tx_pkts, stats_old.tx_pkts, nanos),
		diff_mpps(stats_new.tx_pkts, stats_old.tx_pkts, nanos)
	);
}


impl Stats {
    pub fn new() -> Stats {
        let stats = Stats {
            rx_pkts : 0,
            tx_pkts : 0,
            rx_bytes : 0,
            tx_bytes : 0        
        };
        stats
    }

    pub fn update(&mut self, device: &DeviceInfo) {
        device.stats(Some(self));
    }

    pub fn print_since_last(&self) {
        unimplemented!()
    }
}

#[inline]
pub (crate) fn set_reg32(base: *mut u8, reg: u32, value: u32) {
    let addr = (base as usize + reg as usize) as *mut u32;
    unsafe {
        std::ptr::write_volatile(addr, value);
    }
}

#[inline]
pub (crate) fn get_reg32(base: *const u8, reg: u32) -> u32 {
    let addr = (base as usize + reg as usize) as *mut u32;
    unsafe {
        std::ptr::read_volatile(addr)
    }
}

#[inline]
pub (crate) fn set_flags32(base: *mut u8, reg: u32, flags: u32) {
	set_reg32(base, reg, get_reg32(base, reg) | flags);
}

#[inline]
pub (crate) fn clear_flags32(base: *mut u8, reg: u32, flags: u32) {
	set_reg32(base, reg, get_reg32(base, reg) & (!flags));
}

#[inline]
pub (crate) fn wait_clear_reg32(base: *mut u8, reg: u32, mask: u32) {
    let addr = (base as usize + reg as usize) as *mut u32;
    let mut value = 0;

    while { value = unsafe {
        std::ptr::read_volatile(addr) }; (value & mask) != 0 } {
		println!("waiting for flags {:x} in register {:x} to clear, current value {:x}", mask, reg, value);
		sleep(Duration::from_millis(100));
	}
}


#[inline]
pub (crate) fn wait_set_reg32(base: *mut u8, reg: u32, mask: u32) {
	let addr = (base as usize + reg as usize) as *mut u32;
    let mut value = 0;

    while { value = unsafe {
        std::ptr::read_volatile(addr) }; (value & mask) != mask } {
		println!("waiting for flags {:x} in register {:x}, current value {:x}", mask, reg, value);
		sleep(Duration::from_millis(100));
	}
}

