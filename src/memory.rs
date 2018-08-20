use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicUsize, Ordering, ATOMIC_USIZE_INIT};
use std::{process, ptr};

use core::mem;
use libc;
use sysconf;

use super::Buffer;

const HUGE_PAGE_BITS: u32 = 21;
const HUGE_PAGE_SIZE: u32 = (1 << HUGE_PAGE_BITS);

pub struct Mempool {
    free_stack: Vec<*mut Buffer>,
    free_stack_top: u32,
}

impl Mempool {
    pub fn new(num_entries: u16, entry_size: u32) -> Mempool {
        match HUGE_PAGE_SIZE % entry_size {
            0 => {
                let (virt, _) = alloc_dma_memory((num_entries as u32) * entry_size, false);
                let mut free_stack = Vec::new();
                let mut mempool = Mempool {
                    free_stack: Vec::new(),
                    free_stack_top: num_entries as u32,
                };
                for i in 0..num_entries {
                    let buf = (virt as usize + (entry_size as usize * i as usize)) as *mut Buffer;
                    unsafe {
                        (*buf).buf_addr_phys = virt_to_phys(buf as usize as *const u8);
                        (*buf).idx = i as u32;
                        (*buf).mempool = 0 as *mut Mempool;
                        (*buf).size = 0;
                    }
                    free_stack.push(buf);
                }
                mempool.free_stack = free_stack;
                mempool
            }
            _ => panic!(
                "entry size must be a divisor of the huge page size ({})",
                HUGE_PAGE_SIZE
            ),
        }
    }

    pub fn alloc_buf_batch(&mut self, size: u32) -> Vec<*mut Buffer> {
        let mut ret = Vec::new();
        let upper_size = match self.free_stack_top < size {
            true => {
                println!(
                    "memory pool {:p} only has {} free bufs, requested {}",
                    &self, self.free_stack_top, size
                );
                self.free_stack_top
            }
            false => size,
        };
        for _ in 0..upper_size {
            self.free_stack_top -= 1;
            let entry_id = self.free_stack_top;
            ret.push(self.free_stack[entry_id as usize])
        }
        ret
    }

    pub fn alloc_buf(&mut self) -> Option<*mut Buffer> {
        Some(self.alloc_buf_batch(1)[0])
    }

    pub fn free_buf(&mut self, buf: *mut Buffer) {
        self.free_stack[self.free_stack_top as usize] = buf;
        self.free_stack_top += 1;
    }
}

static HUGE_PAGE_ID: AtomicUsize = ATOMIC_USIZE_INIT;

// translate a virtual address to a physical one via /proc/self/pagemap
fn virt_to_phys(virt: *const u8) -> usize {
    const POINTER_SZ: usize = mem::size_of::<usize>();
    let page_size = sysconf::page::pagesize();
    let mut file = OpenOptions::new()
        .read(true)
        .open("/proc/self/pagemap")
        .expect("can't open the file: /proc/self/pagemap");
    // pagemap is an array of pointers for each normal-sized page
    let pos = virt as usize / page_size * POINTER_SZ;
    file.seek(SeekFrom::Start(pos as u64))
        .expect("Seek failed!");
    let mut buf = [0u8; POINTER_SZ];
    assert!(
        POINTER_SZ == file.read(&mut buf).expect("read failed!"),
        format!(
            "failed to translate virtual address {:p} to physical address",
            virt
        )
    );

    // bits 0-54 are the page number
    unsafe {
        (mem::transmute::<_, usize>(buf) & 0x7fffffffffffffusize) * page_size
            + ((virt as usize) % page_size)
    }
}

pub fn alloc_dma_memory(size: u32, require_contiguous: bool) -> (*const u8, usize) {
    let size_needed = match size % HUGE_PAGE_SIZE {
        0 => size,
        _ => ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_BITS,
    };

    if require_contiguous && size_needed > HUGE_PAGE_SIZE {
        panic!("could not map physically contiguous memory")
    }

    let id = HUGE_PAGE_ID.fetch_add(1, Ordering::SeqCst);
    let path = format!("/mnt/huge/ixgbe-{}-{}", process::id(), id);
    let file = OpenOptions::new()
        .truncate(true)
        .write(true)
        .read(true)
        .create(true)
        .open(&path)
        .expect(&format!("can't open the file: {}", path));

    let virt = unsafe {
        let addr = libc::mmap(
            ptr::null_mut(),
            size_needed as libc::size_t,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED | libc::MAP_HUGETLB,
            file.as_raw_fd(),
            0,
        );
        libc::mlock(addr, size_needed as libc::size_t);
        libc::memset(addr, -1, size_needed as libc::size_t);
        addr
    } as *const u8;
    println!("virt: {:x}, phys: {:x}", virt as usize, virt_to_phys(virt));
    (virt, virt_to_phys(virt))
}
