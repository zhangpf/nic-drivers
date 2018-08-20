use super::memory::{alloc_dma_memory, Mempool};
use super::pci::pci_map_resource;
use super::types::*;
use super::{
    clear_flags32, get_reg32, set_flags32, set_reg32, wait_clear_reg32, wait_set_reg32, Buffer,
    Stats,
};
use std::cell::RefCell;
use std::mem;
use std::thread::sleep;
use std::time::Duration;

#[repr(C)]
#[derive(Clone, Copy)]
struct RxAddr {
    pkt_addr: usize,
    hdr_addr: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxHighRss {
    ip_id: u16,
    csum: u16,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxCsumIp {
    ip_id: u16,
    csum: u16,
}

#[repr(C)]
#[derive(Clone, Copy)]
union RxWritebackLowerLoDword {
    data: u32,
    hs_rss: RxHighRss,
}

#[repr(C)]
#[derive(Clone, Copy)]
union RxWritebackLowerHiDword {
    rss: u32,
    hs_rss: RxHighRss,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxWritebackLower {
    lo_dword: RxWritebackLowerLoDword,
    hi_dword: RxWritebackLowerHiDword,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxWritebackUpper {
    status_error: u32,
    length: u16,
    vlan: u16,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxWriteback {
    lower: RxWritebackLower,
    upper: RxWritebackUpper,
}

#[repr(C)]
#[derive(Clone, Copy)]
union AdvRxDesc {
    read: RxAddr,
    wb: RxWriteback,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct TxAddr {
    buffer_addr: u64, /* Address of descriptor's data buf */
    cmd_type_len: u32,
    olinfo_status: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct TxWriteback {
    rsvd: u64,
    nxtseq_seed: u32,
    status: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
/* Transmit Descriptor - Advanced */
union AdvTxDesc {
    read: TxAddr,
    wb: TxWriteback,
}

// allocated for each rx queue, keeps state for the receive function
struct RxQueue {
    descriptors: *const u8,
    mempool: RefCell<Mempool>,
    num_entries: u16,
    // position we are reading from
    rx_index: u16,
    // virtual addresses to map descriptors back to their mbuf for freeing
    virtual_addresses: Vec<*mut Buffer>,
}

impl RxQueue {
    pub fn get_desc(&self, idx: u16) -> Option<*mut AdvRxDesc> {
        match idx >= self.num_entries {
            true => None,
            false => {
                let ptr = self.descriptors as usize + idx as usize * mem::size_of::<AdvRxDesc>();
                Some(ptr as *mut AdvRxDesc)
            }
        }
    }
}

// allocated for each tx queue, keeps state for the transmit function
struct TxQueue {
    descriptors: *const u8,
    num_entries: u16,
    // position to clean up descriptors that where sent out by the nic
    clean_index: u16,
    // position to insert packets for transmission
    tx_index: u16,
    // virtual addresses to map descriptors back to their mbuf for freeing
    virtual_addresses: Vec<*mut Buffer>,
}

impl TxQueue {
    pub fn get_desc(&self, idx: u16) -> Option<*mut AdvTxDesc> {
        match idx >= self.num_entries {
            true => None,
            false => {
                let ptr = self.descriptors as usize + idx as usize * mem::size_of::<AdvTxDesc>();
                Some(ptr as *mut AdvTxDesc)
            }
        }
    }
}

pub struct DeviceInfo {
    num_rx_queues: u32,
    num_tx_queues: u32,
    rx_queues: Vec<RxQueue>,
    tx_queues: Vec<TxQueue>,
    addr: *mut u8,
}

const NUM_RX_QUEUE_ENTRIES: u32 = 512;
const NUM_TX_QUEUE_ENTRIES: u32 = 512;

const TX_CLEAN_BATCH: u16 = 32;

macro_rules! wrap_ring {
    ($index:expr, $ring_size:expr) => {
        ($index + 1) & ($ring_size - 1)
    };
}

impl DeviceInfo {
    pub fn new(pci_addr: &str, num_rx_queues: u32, num_tx_queues: u32) -> DeviceInfo {
        if num_rx_queues > MAX_QUEUES {
            panic!(format!(
                "cannot configur {} rx queues: limit is {}",
                num_rx_queues, MAX_QUEUES
            ))
        }
        if num_tx_queues > MAX_QUEUES {
            panic!(format!(
                "cannot configur {} tx queues: limit is {}",
                num_tx_queues, MAX_QUEUES
            ))
        }
        let addr = pci_map_resource(pci_addr);

        println!("Resetting device {}", pci_addr);
        Self::reset(addr);
        println!("Initializing device {}", pci_addr);
        let (rx_queues, tx_queues) = Self::init_device(addr, num_rx_queues, num_tx_queues);

        let mut device = DeviceInfo {
            num_rx_queues,
            num_tx_queues,
            rx_queues,
            tx_queues,
            addr,
        };

        for i in 0..device.num_rx_queues {
            device.start_rx_queue(i);
        }
        for i in 0..device.num_tx_queues {
            device.start_tx_queue(i);
        }

        // skip last step from 4.6.3 - don't want interrupts
        // finally, enable promisc mode by default, it makes testing less annoying
        device.ixgbe_set_promisc(true);

        // wait for some time for the link to come up
        device.wait_for_link();
        device
    }

    fn wait_for_link(&self) {
        println!("Waiting for link...");
        let mut max_wait: u64 = 10000;
        const POLL_INTERVAL: u64 = 100;
        let mut speed = 0;
        while {
            speed = self.get_link_speed();
            speed == 0 && max_wait > 0
        } {
            sleep(Duration::from_millis(POLL_INTERVAL));
            max_wait = max_wait - POLL_INTERVAL;
        }
        println!("Link speed is {} Mbit/s", self.get_link_speed());
    }

    fn get_link_speed(&self) -> u32 {
        let links = get_reg32(self.addr, IXGBE_LINKS);
        match links & IXGBE_LINKS_UP {
            0 => 0,
            _ => match links & IXGBE_LINKS_SPEED_82599 {
                IXGBE_LINKS_SPEED_100_82599 => 100,
                IXGBE_LINKS_SPEED_1G_82599 => 1000,
                IXGBE_LINKS_SPEED_10G_82599 => 10000,
                _ => 0,
            },
        }
    }

    fn init_link(addr: *mut u8) {
        set_reg32(
            addr,
            IXGBE_AUTOC,
            (get_reg32(addr, IXGBE_AUTOC) & !IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL,
        );
        set_reg32(
            addr,
            IXGBE_AUTOC,
            (get_reg32(addr, IXGBE_AUTOC) & !IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI,
        );
        set_flags32(addr, IXGBE_AUTOC, IXGBE_AUTOC_AN_RESTART);
    }

    fn reset(addr: *mut u8) {
        set_reg32(addr, IXGBE_EIMC, 0x7FFFFFFF);

        set_reg32(addr, IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
        wait_clear_reg32(addr, IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
        sleep(Duration::from_millis(10));
        set_reg32(addr, IXGBE_EIMC, 0x7FFFFFFF);
    }

    fn init_device(
        addr: *mut u8,
        num_rx_queues: u32,
        num_tx_queues: u32,
    ) -> (Vec<RxQueue>, Vec<TxQueue>) {
        wait_set_reg32(addr, IXGBE_EEC, IXGBE_EEC_ARD);

        // section 4.6.3 - Wait for DMA initialization done (RDRXCTL.DMAIDONE)
        wait_set_reg32(addr, IXGBE_RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);

        Self::init_link(addr);

        Self::read_stats(addr, None);

        let rx_queues = Self::init_rx(addr, num_rx_queues);
        let tx_queues = Self::init_tx(addr, num_tx_queues);

        (rx_queues, tx_queues)
    }

    fn start_tx_queue(&self, queue_id: u32) {
        println!("starting queue {}", queue_id);
        let queue = &self.tx_queues[queue_id as usize];
        if queue.num_entries & (queue.num_entries - 1) != 0 {
            panic!("number of queue entries must be a power of 2");
        }

        // tx queue starts out empty
        set_reg32(self.addr, IXGBE_TDH!(queue_id), 0);
        set_reg32(self.addr, IXGBE_TDT!(queue_id), 0);
        // enable queue and wait if necessary
        set_flags32(self.addr, IXGBE_TXDCTL!(queue_id), IXGBE_TXDCTL_ENABLE);
        wait_set_reg32(self.addr, IXGBE_TXDCTL!(queue_id), IXGBE_TXDCTL_ENABLE);
    }

    fn start_rx_queue(&mut self, queue_id: u32) {
        println!("starting rx queue {}", queue_id);
        let queue = &mut self.rx_queues[queue_id as usize];

        if queue.num_entries & (queue.num_entries - 1) != 0 {
            panic!("number of queue entries must be a power of 2");
        }

        for j in 0..queue.num_entries {
            let rxd = queue.get_desc(j).unwrap();
            let mut buffer = queue
                .mempool
                .borrow_mut()
                .alloc_buf()
                .expect("failed to allocate rx descriptor");
            unsafe {
                (*rxd).read.pkt_addr = (*buffer).buf_addr_phys + mem::size_of::<Buffer>();
                (*rxd).read.hdr_addr = 0;
            }
            // we need to return the virtual address in the rx function which the descriptor doesn't know by default
            queue.virtual_addresses[j as usize] = buffer as *mut Buffer;
        }

        // enable queue and wait if necessary
        set_flags32(self.addr, IXGBE_RXDCTL!(queue_id), IXGBE_RXDCTL_ENABLE);
        wait_set_reg32(self.addr, IXGBE_RXDCTL!(queue_id), IXGBE_RXDCTL_ENABLE);
        // rx queue starts out full
        set_reg32(self.addr, IXGBE_RDH!(queue_id), 0);
        // was set to 0 before in the init function
        set_reg32(
            self.addr,
            IXGBE_RDT!(queue_id),
            queue.num_entries as u32 - 1,
        );
    }

    fn ixgbe_set_promisc(&self, enabled: bool) {
        match enabled {
            true => {
                println!("enabling promisc mode");
                set_flags32(self.addr, IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
            }
            false => {
                println!("disabling promisc mode");
                clear_flags32(self.addr, IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
            }
        }
    }

    fn init_tx(addr: *mut u8, num_tx_queues: u32) -> Vec<TxQueue> {
        // crc offload and small packet padding
        set_flags32(
            addr,
            IXGBE_HLREG0,
            IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN,
        );
        // set default buffer size allocations
        // see also: section 4.6.11.3.4, no fancy features like DCB and VTd
        set_reg32(addr, IXGBE_TXPBSIZE!(0), IXGBE_TXPBSIZE_40KB);
        for i in 1..8 {
            set_reg32(addr, IXGBE_TXPBSIZE!(i), 0);
        }

        // required when not using DCB/VTd
        set_reg32(addr, IXGBE_DTXMXSZRQ, 0xFFFF);
        clear_flags32(addr, IXGBE_RTTDCS, IXGBE_RTTDCS_ARBDIS);

        let mut tx_queues = Vec::new();
        // per-queue config for all queues
        for i in 0..num_tx_queues {
            println!("initializing tx queue {}", i);

            // setup descriptor ring, see section 7.1.9
            let ring_size_bytes = NUM_TX_QUEUE_ENTRIES * mem::size_of::<AdvTxDesc>() as u32;
            let (virt, phys) = alloc_dma_memory(ring_size_bytes, true);
            set_reg32(addr, IXGBE_TDBAL!(i), (phys & 0xFFFFFFFF) as u32);
            set_reg32(addr, IXGBE_TDBAH!(i), (phys >> 32) as u32);
            set_reg32(addr, IXGBE_TDLEN!(i), ring_size_bytes);

            println!("tx ring {} phy addr:  {:X}", i, phys);
            println!("tx ring {} virt addr: {:X}", i, virt as usize);

            // descriptor writeback magic values, important to get good performance and low PCIe overhead
            // see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
            // we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
            let mut txdctl = get_reg32(addr, IXGBE_TXDCTL!(i));
            // there are no defines for this in ixgbe_type.h for some reason
            // pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
            txdctl &= !(0x3F | (0x3F << 8) | (0x3F << 16)); // clear bits
            txdctl |= 36 | (8 << 8) | (4 << 16); // from DPDK
            set_reg32(addr, IXGBE_TXDCTL!(i), txdctl);
            // private data for the driver, 0-initialized
            let queue = TxQueue {
                clean_index: 0,
                descriptors: virt,
                num_entries: NUM_TX_QUEUE_ENTRIES as u16,
                tx_index: 0,
                virtual_addresses: vec![0 as *mut Buffer; NUM_TX_QUEUE_ENTRIES as usize],
            };

            tx_queues.push(queue);
        }

        // final step: enable DMA
        set_reg32(addr, IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);

        tx_queues
    }

    fn init_rx(addr: *mut u8, num_rx_queues: u32) -> Vec<RxQueue> {
        // make sure that rx is disabled while re-configuring it
        // the datasheet also wants us to disable some crypto-offloading related rx paths (but we don't care about them)
        clear_flags32(addr, IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
        // no fancy dcb or vt, just a single 128kb packet buffer for us
        set_reg32(addr, IXGBE_RXPBSIZE!(0), IXGBE_RXPBSIZE_128KB);
        for i in 1..8 {
            set_reg32(addr, IXGBE_RXPBSIZE!(i), 0);
        }

        // always enable CRC offloading
        set_flags32(addr, IXGBE_HLREG0, IXGBE_HLREG0_RXCRCSTRP);
        set_flags32(addr, IXGBE_RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

        // accept broadcast packets
        set_flags32(addr, IXGBE_FCTRL, IXGBE_FCTRL_BAM);

        let mut rx_queues = Vec::new();

        // per-queue config, same for all queues
        for i in 0..num_rx_queues {
            println!("initializing rx queue {}", i);
            // enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
            set_reg32(
                addr,
                IXGBE_SRRCTL!(i),
                (get_reg32(addr, IXGBE_SRRCTL!(i)) & !IXGBE_SRRCTL_DESCTYPE_MASK)
                    | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF,
            );
            // drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
            // a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
            set_flags32(addr, IXGBE_SRRCTL!(i), IXGBE_SRRCTL_DROP_EN);
            // setup descriptor ring, see section 7.1.9
            let ring_size_bytes = NUM_RX_QUEUE_ENTRIES * mem::size_of::<AdvRxDesc>() as u32;
            let (virt, phy) = alloc_dma_memory(ring_size_bytes, true);
            // neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
            // memset(mem.virt, -1, ring_size_bytes);
            set_reg32(addr, IXGBE_RDBAL!(i), (phy & 0xFFFFFFFF) as u32);
            set_reg32(addr, IXGBE_RDBAH!(i), (phy >> 32) as u32);
            set_reg32(addr, IXGBE_RDLEN!(i), ring_size_bytes);

            println!("rx ring {} phy addr:  {:X}", i, phy);
            println!("rx ring {} virt addr: {:X}", i, virt as usize);
            // set ring to empty at start
            set_reg32(addr, IXGBE_RDH!(i), 0);
            set_reg32(addr, IXGBE_RDT!(i), 0);

            // 2048 as pktbuf size is strictly speaking incorrect:
            // we need a few headers (1 cacheline), so there's only 1984 bytes left for the device
            // but the 82599 can only handle sizes in increments of 1 kb; but this is fine since our max packet size
            // is the default MTU of 1518
            // this has to be fixed if jumbo frames are to be supported
            // mempool should be >= the number of rx and tx descriptors for a forwarding application
            let mempool_size = NUM_RX_QUEUE_ENTRIES + NUM_TX_QUEUE_ENTRIES;
            let mempool = RefCell::new(Mempool::new(
                match mempool_size < 4096 {
                    true => 4096,
                    false => mempool_size,
                } as u16,
                2048,
            ));

            // private data for the driver, 0-initialized
            let queue = RxQueue {
                descriptors: virt,
                rx_index: 0,
                num_entries: NUM_RX_QUEUE_ENTRIES as u16,
                mempool,
                virtual_addresses: vec![0 as *mut Buffer; NUM_RX_QUEUE_ENTRIES as usize],
            };

            rx_queues.push(queue);
        }

        // last step is to set some magic bits mentioned in the last sentence in 4.6.7
        set_flags32(addr, IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);
        // this flag probably refers to a broken feature: it's reserved and initialized as '1' but it must be set to '0'
        // there isn't even a constant in ixgbe_types.h for this flag
        for i in 0..num_rx_queues {
            clear_flags32(addr, IXGBE_DCA_RXCTRL!(i), 1 << 12);
        }

        // start RX
        set_flags32(addr, IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);

        rx_queues
    }

    pub fn tx_batch_busy_wait(&mut self, bufs: Vec<*mut Buffer>) {
        let slice = bufs.as_slice();
        let num_bufs = bufs.len();
        let mut num_sent = 0;
        while num_sent != num_bufs {
            num_sent += self.tx_batch(0, &slice[num_sent..]) as usize;
            // println!("{} {}", num_bufs, num_sent);
        }
    }

    pub fn read_stats(addr: *const u8, stats: Option<&mut Stats>) {
        let rx_pkts = get_reg32(addr, IXGBE_GPRC);
        let tx_pkts = get_reg32(addr, IXGBE_GPTC);
        let rx_bytes =
            get_reg32(addr, IXGBE_GORCL) as u64 + ((get_reg32(addr, IXGBE_GORCH) as u64) << 32);
        let tx_bytes =
            get_reg32(addr, IXGBE_GOTCL) as u64 + ((get_reg32(addr, IXGBE_GOTCH) as u64) << 32);
        if let Some(stats) = stats {
            stats.rx_pkts += rx_pkts;
            stats.tx_pkts += tx_pkts;
            stats.rx_bytes += rx_bytes;
            stats.tx_bytes += tx_bytes;
        }
    }

    pub fn stats(&self, stats: Option<&mut Stats>) {
        Self::read_stats(self.addr, stats);
    }

    // see datasheet section 7.1.9 for an explanation of the rx ring structure
    // tl;dr: we control the tail of the queue, the hardware the head
    pub fn rx_batch(&mut self, queue_id: u32, bufs: &mut [*mut Buffer]) -> u32 {
        let queue = &mut self.rx_queues[queue_id as usize];
        let buf_idx = 0;
        let mut last_rx_idx = 0;
        let mut rx_idx = queue.rx_index;
        for _ in 0..bufs.len() {
            // rx descriptors are explained in 7.1.5
            let desc = queue.get_desc(rx_idx).unwrap();
            let status = unsafe { (*desc).wb.upper.status_error };
            match status & IXGBE_RXDADV_STAT_DD {
                0 => break,
                _ => {
                    if 0 == (status & IXGBE_RXDADV_STAT_EOP) {
                        panic!("multi-segment packets are not supported - increase buffer size or decrease MTU");
                    }
                    // got a packet, read and copy the whole descriptor
                    let buf = queue.virtual_addresses[rx_idx as usize];
                    unsafe { (*buf).size = (*desc).wb.upper.length as u32 };
                    // this would be the place to implement RX offloading by translating the device-specific flags
                    // to an independent representation in the buf (similiar to how DPDK works)
                    // need a new mbuf for the descriptor
                    let new_buf = queue.mempool.borrow_mut().alloc_buf().
                        expect("failed to allocate new mbuf for rx, you are either leaking memory or your mempool is too small");

                    // reset the descriptor
                    unsafe {
                        let mut desc_cloned = (*desc).clone();
                        desc_cloned.read.pkt_addr =
                            (*new_buf).buf_addr_phys + mem::size_of::<Buffer>();
                        desc_cloned.read.hdr_addr = 0; // this resets the flags
                    }
                    queue.virtual_addresses[rx_idx as usize] = new_buf;
                    bufs[buf_idx] = buf;
                    // want to read the next one in the next iteration, but we still need the last/current to update RDT later
                    last_rx_idx = rx_idx;
                    rx_idx = wrap_ring!(rx_idx, queue.num_entries as u16);
                }
            }
        }
        if rx_idx != last_rx_idx {
            // tell hardware that we are done
            // this is intentionally off by one, otherwise we'd set RDT=RDH if we are receiving faster than packets are coming in
            // RDT=RDH means queue is full
            set_reg32(self.addr, IXGBE_RDT!(queue_id), last_rx_idx as u32);
            queue.rx_index = rx_idx as u16;
        }
        buf_idx as u32
    }

    // section 1.8.1 and 7.2
    // we control the tail, hardware the head
    // huge performance gains possible here by sending packets in batches - writing to TDT for every packet is not efficient
    // returns the number of packets transmitted, will not block when the queue is full
    pub fn tx_batch(&mut self, queue_id: u32, bufs: &[*mut Buffer]) -> u32 {
        let queue = &mut self.tx_queues[queue_id as usize];
        // the descriptor is explained in section 7.2.3.2.4
        // we just use a struct copy & pasted from intel, but it basically has two formats (hence a union):
        // 1. the write-back format which is written by the NIC once sending it is finished this is used in step 1
        // 2. the read format which is read by the NIC and written by us, this is used in step 2
        let mut clean_index = queue.clean_index; // next descriptor to clean up
        let mut cur_index = queue.tx_index; // next descriptor to use for tx
                                            // step 1: clean up descriptors that were sent out by the hardware and return them to the mempool
                                            // start by reading step 2 which is done first for each packet
                                            // cleaning up must be done in batches for performance reasons, so this is unfortunately somewhat complicated
        loop {
            // figure out how many descriptors can be cleaned up
            let mut cleanable = cur_index as i32 - clean_index as i32; // cur is always ahead of clean (invariant of our queue)
            if cleanable < 0 {
                // handle wrap-around
                cleanable = queue.num_entries as i32 + cleanable;
            }
            if cleanable < TX_CLEAN_BATCH as i32 {
                break;
            }

            // calculcate the index of the last transcriptor in the clean batch
            // we can't check all descriptors for performance reasons
            let mut cleanup_to = clean_index + TX_CLEAN_BATCH - 1;
            if cleanup_to >= queue.num_entries {
                cleanup_to -= queue.num_entries;
            }

            let txd = queue.get_desc(cleanup_to).unwrap();
            let status = unsafe { (*txd).wb.status };
            // hardware sets this flag as soon as it's sent out, we can give back all bufs in the batch back to the mempool
            if status & IXGBE_ADVTXD_STAT_DD != 0 {
                let mut i = clean_index;
                loop {
                    let buf = queue.virtual_addresses[i as usize];
                    unsafe {
                        (*buf).free_buf();
                    }
                    if i == cleanup_to {
                        break;
                    }
                    i = wrap_ring!(i, queue.num_entries);
                }
                // next descriptor to be cleaned up is one after the one we just cleaned
                clean_index = wrap_ring!(cleanup_to, queue.num_entries);
            } else {
                // clean the whole batch or nothing; yes, this leaves some packets in
                // the queue forever if you stop transmitting, but that's not a real concern
                break;
            }
        }

        queue.clean_index = clean_index;
        let mut sent = 0;
        // step 2: send out as many of our packets as possible
        for _ in 0..bufs.len() {
            let next_index = wrap_ring!(cur_index, queue.num_entries);
            // we are full if the next index is the one we are trying to reclaim
            if clean_index == next_index {
                break;
            }
            let buf = bufs[sent];
            // remember virtual address to clean it up later
            queue.virtual_addresses[cur_index as usize] = buf;
            queue.tx_index = wrap_ring!(queue.tx_index, queue.num_entries);
            let txd = queue.get_desc(cur_index).unwrap();
            unsafe {
                // NIC reads from here
                (*txd).read.buffer_addr = ((*buf).buf_addr_phys + mem::size_of::<Buffer>()) as u64;
                // always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
                (*txd).read.cmd_type_len = IXGBE_ADVTXD_DCMD_EOP
                    | IXGBE_ADVTXD_DCMD_RS
                    | IXGBE_ADVTXD_DCMD_IFCS
                    | IXGBE_ADVTXD_DCMD_DEXT
                    | IXGBE_ADVTXD_DTYP_DATA
                    | (*buf).size;
                // no fancy offloading stuff - only the total payload length
                // implement offloading flags here:
                // 	* ip checksum offloading is trivial: just set the offset
                // 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
                (*txd).read.olinfo_status = (*buf).size << IXGBE_ADVTXD_PAYLEN_SHIFT;
            }
            cur_index = next_index;
            sent += 1;
        }
        // send out by advancing tail, i.e., pass control of the bufs to the nic
        // this seems like a textbook case for a release memory order, but Intel's driver doesn't even use a compiler barrier here
        set_reg32(self.addr, IXGBE_TDT!(queue_id), queue.tx_index as u32);
        sent as u32
    }
}
