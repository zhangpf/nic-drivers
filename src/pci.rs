use core::mem;
use std::ptr;
use std::fs::OpenOptions;
use std::io::{SeekFrom, Seek, Read, Write};
use std::os::unix::io::AsRawFd;

use libc;

fn remove_driver(pci_addr: &str) {
    let res = OpenOptions::new().create(false).write(true)
                .open(format!("/sys/bus/pci/devices/{}/driver/unbind", pci_addr));
    match res {
        Err(_) => {
            println!("No driver loaded")
        },
        Ok(mut file) => {
            // let buf = pci_addr[0..pci_addr.len()]; 
            file.write_all(pci_addr.as_bytes()).unwrap();
            println!("Success on removing driver")
        }
    }

}

fn enable_dma(pci_addr: &str) {
    let mut file = OpenOptions::new().read(true).write(true)
                .open(format!("/sys/bus/pci/devices/{}/config", pci_addr))
                .expect("Open pci config error");
    file.seek(SeekFrom::Start(4)).unwrap();
    let mut buf = [0; 2];
    file.read(&mut buf).unwrap();
    unsafe {
        let dma : *mut u16 =  mem::transmute(&buf);
        *dma = *dma | (1<< 2);
    }
    file.seek(SeekFrom::Start(4)).unwrap();
    file.write(&buf).unwrap();
}

pub (crate) fn pci_map_resource(pci_addr: &str) -> *mut u8 {
    remove_driver(pci_addr);
    enable_dma(pci_addr);
    let file = OpenOptions::new().read(true).write(true)
                .open(format!("/sys/bus/pci/devices/{}/resource0", pci_addr))
                .expect("Open pci resource failed");
    unsafe {
        let st = [0; mem::size_of::<libc::stat>()];
        let ptr = (&st as *const _) as *mut libc::stat;
        libc::fstat(
            file.as_raw_fd(),
            ptr
        );
        libc::mmap(
            ptr::null_mut(),
            (*ptr).st_size as libc::size_t,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            file.as_raw_fd(),
            0
        ) as *mut u8
    }
}