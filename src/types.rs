pub(crate) const MAX_QUEUES: u32 = 64;

pub(crate) const IXGBE_RXCTRL: u32 = 0x03000;
pub(crate) const IXGBE_HLREG0: u32 = 0x04240;

pub(crate) const IXGBE_EIMC: u32 = 0x00888;
pub(crate) const IXGBE_CTRL: u32 = 0x00000;
pub(crate) const IXGBE_AUTOC: u32 = 0x042A0;

pub(crate) const IXGBE_GPRC: u32 = 0x04074;
pub(crate) const IXGBE_GPTC: u32 = 0x04080;

pub(crate) const IXGBE_GORCL: u32 = 0x04088;
pub(crate) const IXGBE_GORCH: u32 = 0x0408C;
pub(crate) const IXGBE_GOTCL: u32 = 0x04090;
pub(crate) const IXGBE_GOTCH: u32 = 0x04094;

pub(crate) const IXGBE_FCTRL_MPE: u32 = 0x00000100; /* Multicast Promiscuous Ena*/
pub(crate) const IXGBE_FCTRL_UPE: u32 = 0x00000200; /* Unicast Promiscuous Ena */

/* Receive Config masks */
pub(crate) const IXGBE_RXDCTL_ENABLE: u32 = 0x02000000; /* Ena specific Rx Queue */

/* Transmit Config masks */
pub(crate) const IXGBE_TXDCTL_ENABLE: u32 = 0x02000000; /* Ena specific Tx Queue */

macro_rules! IXGBE_RXPBSIZE {
    ($_i:expr) => {
        0x03C00 + (($_i) * 4)
    };
}

/*
 * Split and Replication Receive Control Registers
 * 00-15 : 0x02100 + n*4
 * 16-64 : 0x01014 + n*0x40
 * 64-127: 0x0D014 + (n-64)*0x40
 */
macro_rules! IXGBE_SRRCTL {
    ($_i:expr) => {
        match ($_i) <= 15 {
            true => (0x02100 + (($_i) * 4)),
            false => match ($_i) < 64 {
                true => (0x01014 + (($_i) * 0x40)),
                false => (0x0D014 + ((($_i) - 64) * 0x40)),
            },
        }
    };
}

pub(crate) const IXGBE_EEC: u32 = 0x10010;

pub(crate) const IXGBE_RXCTRL_RXEN: u32 = 0x00000001; /* Enable Receiver */

pub(crate) const IXGBE_RXPBSIZE_128KB: u32 = 0x00020000; /* 128KB Packet Buffer */

pub(crate) const IXGBE_EEC_ARD: u32 = 0x00000200; /* EEPROM Auto Read Done */

pub(crate) const IXGBE_CTRL_LNK_RST: u32 = 0x00000008; /* Link Reset. Resets everything. */
pub(crate) const IXGBE_CTRL_RST: u32 = 0x04000000; /* Reset (SW) */
pub(crate) const IXGBE_CTRL_RST_MASK: u32 = (IXGBE_CTRL_LNK_RST | IXGBE_CTRL_RST);

pub(crate) const IXGBE_AUTOC_AN_RESTART: u32 = 0x00001000;
pub(crate) const IXGBE_AUTOC_LMS_SHIFT: u32 = 13;
pub(crate) const IXGBE_AUTOC_LMS_10G_SERIAL: u32 = (0x3 << IXGBE_AUTOC_LMS_SHIFT);
pub(crate) const IXGBE_AUTOC_LMS_MASK: u32 = (0x7 << IXGBE_AUTOC_LMS_SHIFT);

pub(crate) const IXGBE_AUTOC_10G_PMA_PMD_MASK: u32 = 0x00000180;
pub(crate) const IXGBE_AUTOC_10G_PMA_PMD_SHIFT: u32 = 7;
pub(crate) const IXGBE_AUTOC_10G_XAUI: u32 = (0x0 << IXGBE_AUTOC_10G_PMA_PMD_SHIFT);

pub(crate) const IXGBE_HLREG0_RXCRCSTRP: u32 = 0x00000002; /* bit  1 */
pub(crate) const IXGBE_RDRXCTL: u32 = 0x02F00;
pub(crate) const IXGBE_RDRXCTL_CRCSTRIP: u32 = 0x00000002; /* CRC Strip */

pub(crate) const IXGBE_FCTRL: u32 = 0x05080;

pub(crate) const IXGBE_FCTRL_BAM: u32 = 0x00000400; /* Broadcast Accept Mode */
pub(crate) const IXGBE_SRRCTL_DESCTYPE_MASK: u32 = 0x0E000000;

pub(crate) const IXGBE_SRRCTL_DROP_EN: u32 = 0x10000000;

pub(crate) const IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF: u32 = 0x02000000;

pub(crate) const IXGBE_LINKS_UP: u32 = 0x40000000;
pub(crate) const IXGBE_LINKS: u32 = 0x042A4;

pub(crate) const IXGBE_LINKS_SPEED_82599: u32 = 0x30000000;
pub(crate) const IXGBE_LINKS_SPEED_10G_82599: u32 = 0x30000000;
pub(crate) const IXGBE_LINKS_SPEED_1G_82599: u32 = 0x20000000;
pub(crate) const IXGBE_LINKS_SPEED_100_82599: u32 = 0x10000000;

pub(crate) const IXGBE_CTRL_EXT: u32 = 0x00018;

pub(crate) const IXGBE_CTRL_EXT_NS_DIS: u32 = 0x00010000; /* No Snoop disable */
/* HLREG0 Bit Masks */
pub(crate) const IXGBE_HLREG0_TXCRCEN: u32 = 0x00000001; /* bit  0 */
pub(crate) const IXGBE_HLREG0_TXPADEN: u32 = 0x00000400; /* bit 10 */

/* Receive DMA Registers */
macro_rules! IXGBE_RDBAL {
    ($_i:expr) => {
        match ($_i) < 64 {
            true => (0x01000 + (($_i) * 0x40)),
            false => (0x0D000 + ((($_i) - 64) * 0x40)),
        }
    };
}

macro_rules! IXGBE_RDBAH {
    ($_i:expr) => {
        match ($_i) < 64 {
            true => (0x01004 + (($_i) * 0x40)),
            false => (0x0D004 + ((($_i) - 64) * 0x40)),
        }
    };
}

macro_rules! IXGBE_RDLEN {
    ($_i:expr) => {
        match ($_i) < 64 {
            true => (0x01008 + (($_i) * 0x40)),
            false => (0x0D008 + ((($_i) - 64) * 0x40)),
        }
    };
}

macro_rules! IXGBE_RDH {
    ($_i:expr) => {
        match ($_i) < 64 {
            true => (0x01010 + (($_i) * 0x40)),
            false => (0x0D010 + ((($_i) - 64) * 0x40)),
        }
    };
}

macro_rules! IXGBE_RDT {
    ($_i:expr) => {
        match ($_i) < 64 {
            true => (0x01018 + (($_i) * 0x40)),
            false => (0x0D018 + ((($_i) - 64) * 0x40)),
        }
    };
}

/*
 * Rx DCA Control Register:
 * 00-15 : 0x02200 + n*4
 * 16-64 : 0x0100C + n*0x40
 * 64-127: 0x0D00C + (n-64)*0x40
 */

macro_rules! IXGBE_DCA_RXCTRL {
    ($_i:expr) => {
        match $_i <= 15 {
            true => (0x02200 + (($_i) * 4)),
            false => match $_i < 64 {
                true => (0x0100C + (($_i) * 0x40)),
                false => (0x0D00C + ((($_i) - 64) * 0x40)),
            },
        }
    };
}

macro_rules! IXGBE_RXDCTL {
    ($_i:expr) => {
        match $_i < 64 {
            true => (0x01028 + (($_i) * 0x40)),
            false => (0x0D028 + ((($_i) - 64) * 0x40)),
        }
    };
}

macro_rules! IXGBE_TXPBSIZE {
    ($_i:expr) => {
        0x0CC00 + (($_i) * 4)
    };
} /* 8 of these */

macro_rules! IXGBE_TDBAL {
    ($_i:expr) => {
        0x06000 + (($_i) * 0x40)
    };
}

macro_rules! IXGBE_TDBAH {
    ($_i:expr) => {
        0x06004 + (($_i) * 0x40)
    };
}

macro_rules! IXGBE_TDLEN {
    ($_i:expr) => {
        0x06008 + (($_i) * 0x40)
    };
}

macro_rules! IXGBE_TDH {
    ($_i:expr) => {
        0x06010 + (($_i) * 0x40)
    };
}

macro_rules! IXGBE_TDT {
    ($_i:expr) => {
        0x06018 + (($_i) * 0x40)
    };
}

macro_rules! IXGBE_TXDCTL {
    ($_i:expr) => {
        0x06028 + (($_i) * 0x40)
    };
}

pub(crate) const IXGBE_RXD_STAT_DD: u32 = 0x01; /* Descriptor Done */

pub(crate) const IXGBE_RXDADV_STAT_DD: u32 = IXGBE_RXD_STAT_DD; /* Done */

pub(crate) const IXGBE_RXD_STAT_EOP: u32 = 0x02; /* End of Packet */

pub(crate) const IXGBE_RXDADV_STAT_EOP: u32 = IXGBE_RXD_STAT_EOP; /* End of Packet */

pub(crate) const IXGBE_TXPBSIZE_40KB: u32 = 0x0000A000; /* 40KB Packet Buffer */

pub(crate) const IXGBE_DTXMXSZRQ: u32 = 0x08100;

pub(crate) const IXGBE_RTTDCS: u32 = 0x04900;

pub(crate) const IXGBE_RTTDCS_ARBDIS: u32 = 0x00000040; /* DCB arbiter disable */

pub(crate) const IXGBE_DMATXCTL: u32 = 0x04A80;

pub(crate) const IXGBE_DMATXCTL_TE: u32 = 0x1; /* Transmit Enable */

pub(crate) const IXGBE_TXD_STAT_DD: u32 = 0x00000001; /* Descriptor Done */
pub(crate) const IXGBE_ADVTXD_STAT_DD: u32 = IXGBE_TXD_STAT_DD; /* Descriptor Done */

pub(crate) const IXGBE_TXD_CMD_EOP: u32 = 0x01000000; /* End of Packet */
pub(crate) const IXGBE_ADVTXD_DCMD_EOP: u32 = IXGBE_TXD_CMD_EOP; /* End of Packet */

pub(crate) const IXGBE_TXD_CMD_RS: u32 = 0x08000000; /* Report Status */
pub(crate) const IXGBE_ADVTXD_DCMD_RS: u32 = IXGBE_TXD_CMD_RS; /* Report Status */

pub(crate) const IXGBE_TXD_CMD_IFCS: u32 = 0x02000000; /* Insert FCS (Ethernet CRC) */
pub(crate) const IXGBE_ADVTXD_DCMD_IFCS: u32 = IXGBE_TXD_CMD_IFCS; /* Insert FCS */

pub(crate) const IXGBE_TXD_CMD_DEXT: u32 = 0x20000000; /* Desc extension (0 = legacy) */
pub(crate) const IXGBE_ADVTXD_DCMD_DEXT: u32 = IXGBE_TXD_CMD_DEXT; /* Desc ext 1=Adv */

pub(crate) const IXGBE_ADVTXD_DTYP_DATA: u32 = 0x00300000; /* Adv Data Descriptor */

pub(crate) const IXGBE_ADVTXD_PAYLEN_SHIFT: u32 = 14; /* Adv desc PAYLEN shift */

pub(crate) const IXGBE_RDRXCTL_DMAIDONE: u32 = 0x00000008; /* DMA init cycle done */
