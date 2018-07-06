extern crate ixgbe;

use std::env::args;
use std::process;
use std::time::{Duration, Instant};

const BATCH_SIZE : u32 = 32;

fn forward(rx_dev: &ixgbe::DeviceInfo, rx_queue: u16,
    tx_dev: &ixgbe::DeviceInfo, tx_queue: u16) {
    let bufs = rx_dev.rx_batch();
	if bufs.len() > 0 {
		tx_dev.tx_batch(bufs.as_slice());
	}
}

fn main() {
    
    // Prints each argument on a separate line
	if args().len() != 3 {
		println!("{} forwards packets between two ports.\n", args().nth(0).unwrap());
		println!("Usage: {} <pci bus id2> <pci bus id1>\n", args().nth(0).unwrap());
		process::exit(1);
	}

    let dev1 = ixgbe::DeviceInfo::new(&args().nth(1).unwrap());
    let dev2 = ixgbe::DeviceInfo::new(&args().nth(2).unwrap());
	let mempool = ixgbe::Mempool::new();
	let mut last_stats_printed = Instant::now();
	let mut counter = 0;
	let second = Duration::new(1, 0);

	loop {
		forward(&dev1, 0, &dev2, 0);
		forward(&dev2, 0, &dev1, 0);

		if (counter & 0xfff) == 0 && (Instant::now().duration_since(last_stats_printed)) > second {
			let stats1 = dev1.read_stats();
			println!("{:?}", stats1);
			let stats2 = dev2.read_stats();
			println!("{:?}", stats2);
			last_stats_printed = Instant::now();
		}
		counter = counter + 1;
	}
}