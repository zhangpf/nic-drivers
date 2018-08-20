# A simple driver for ixgbe
--------

## Introduce

This repository is a simple [Intel 82599 NIC](https://www.intel.cn/content/www/cn/zh/embedded/products/networking/82599-10-gbe-controller-datasheet.html) driver which is written in Rust and ported from [emmericp/ixy](https://github.com/emmericp/ixy).


## Build and run

1. Get the latest Rust in nightly channel

```bash
curl https://sh.rustup.rs -sSf | sh -s -- --default-toolchain nightly
```

2. use Cargo to build the code

```bash
cargo build --release
```

3. Since the ixgbe driver use the 2MiB huge pages, please configure it with the script:

```bash
sudo ./setup-hugetlbfs.sh
```

4. Find out the PCI address of your NIC:

```
lspci | grep 82599ES
```

For my computer, the result of the command is:

```
01:00.0 Ethernet controller: Intel Corporation 82599ES 10-Gigabit SFI/SFP+ Network Connection (rev 01)
01:00.1 Ethernet controller: Intel Corporation 82599ES 10-Gigabit SFI/SFP+ Network Connection (rev 01)
```

4. Run the test applications.
```
sudo ./target/release/pktgen 0000:01:00.0
```

## Performance

The result of `pktgen` is shown as follow:
```
virt: 7fccda600000, phys: 816400000
No driver loaded
Resetting device 0000:01:00.0
Initializing device 0000:01:00.0
initializing rx queue 0
virt: 7fccda400000, phys: 816800000
rx ring 0 phy addr:  816800000
rx ring 0 virt addr: 7FCCDA400000
virt: 7fccd9c00000, phys: 816e00000
initializing tx queue 0
virt: 7fccd9a00000, phys: 817000000
tx ring 0 phy addr:  817000000
tx ring 0 virt addr: 7FCCD9A00000
starting rx queue 0
starting queue 0
enabling promisc mode
Waiting for link...
Link speed is 10000 Mbit/s
RX: 0 Mbit/s 0 Mpps

TX: 9901.384292164801 Mbit/s 14.734186286261325 Mpps

RX: 0.0024058573361193836 Mbit/s 0.000001991603755065715 Mpps

TX: 9999.00754202517 Mbit/s 14.879477785084605 Mpps

RX: 0.0011392588353670422 Mbit/s 0.000000995855625320841 Mpps

TX: 9999.552267294279 Mbit/s 14.880286870792201 Mpps

RX: 0 Mbit/s 0 Mpps

TX: 9998.990842343424 Mbit/s 14.879450658249143 Mpps
```

The bandwidth of NIC is almost fully occupied and processed by our driver. 

## LICENSE

Please see [LICENSE](LICENSE).