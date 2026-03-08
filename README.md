# RNode Linux

RNode Linux is a userspace Linux implementation of an RNode-compatible transport for SPI-connected SX126x LoRa radios. It reads board-specific GPIO and SPI settings from a YAML file, initializes the radio, and exposes a TCP socket that carries RNode/KISS traffic for a host application such as Reticulum.

The current implementation is intended for Linux systems that provide:

- An SX126x radio connected over `spidev`
- GPIO access through `libgpiod`
- A YAML configuration file describing the SPI device, control pins, and TCP port

## What the daemon does

At startup the daemon:

1. Loads a YAML configuration file
2. Opens the configured SPI device
3. Requests the configured GPIO lines for chip select, reset, busy, DIO1, RX enable, and TX enable
4. Starts a TCP listener on the configured port
5. Translates traffic between the TCP connection and the SX126x radio using the RNode/KISS protocol

Only one TCP client is accepted at a time.

## Prerequisites

You need a Linux system with the following available:

- A C compiler toolchain such as `gcc`
- `cmake` 3.16 or newer
- Development headers for `libgpiod`
- Development headers for `libcyaml`
- Linux SPI support with an exposed device such as `/dev/spidev0.0`
- Access to the relevant GPIO chips and SPI device at runtime

Typical Debian or Ubuntu package names:

```bash
sudo apt install build-essential cmake libgpiod-dev libcyaml-dev
```

Before running the daemon, make sure SPI is enabled on the target system and that the process has permission to access both the `spidev` device and the GPIO controller.

## Build

Configure and build with CMake:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The resulting executable is:

```text
build/rnode
```

## Install

The project defines the following install targets:

- The `rnode` binary to `PREFIX/sbin`
- The example configuration file to `PREFIX/mnt/rns/rnode.yaml`

Install with the default CMake prefix:

```bash
sudo cmake --install build
```

If you specifically want the files installed as `/sbin/rnode` and `/mnt/rns/rnode.yaml`, install with `--prefix /`:

```bash
sudo cmake --install build --prefix /
```

## Run

Run the daemon by passing the path to a YAML configuration file:

```bash
./build/rnode ./rnode.yaml
```

Usage from the program itself:

```text
rnode <config_file.yaml>
```

The daemon logs to syslog using the `rnode` ident.

## Configuration

The configuration file is YAML and must contain these keys:

```yaml
spi: /dev/spidev0.0
cs: { port: 0, pin: 21 }
rst: { port: 0, pin: 18 }
busy: { port: 0, pin: 20 }
dio1: { port: 0, pin: 16 }
rx_en: { port: 0, pin: 12 }
tx_en: { port: 0, pin: 13 }
tcp_port: 7633
```

Field meanings:

- `spi`: path to the Linux SPI device used for the SX126x radio
- `cs`: GPIO chip index and line offset used as chip select
- `rst`: SX126x reset line
- `busy`: SX126x busy line
- `dio1`: SX126x DIO1 interrupt/status line
- `rx_en`: external RF switch RX enable line, if present on the board design
- `tx_en`: external RF switch TX enable line, if present on the board design
- `tcp_port`: TCP port used for the single client RNode/KISS connection

The repository includes an example configuration in [rnode.yaml](/home/sclark/dev/RNode-linux/rnode.yaml).

## Notes

- This project is currently tailored to SX126x-based SPI modules.
- The daemon binds to `0.0.0.0` on the configured TCP port.
- The code links against `libm`, `libgpiod`, and `libcyaml`.
