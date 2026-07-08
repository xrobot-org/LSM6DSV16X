# LSM6DSV16X

ST LSM6DSV16X 6-axis IMU driver module for xrobot / LibXR.

## Features

- SPI mode 0 register access with manual GPIO chip select.
- Configurable accelerometer and gyroscope ODR/range.
- Polling sample thread, no data-ready interrupt required.
- Register configuration is verified by readback during initialization.
- Publishes:
  - `lsm6dsv16x_gyro`: gyroscope data in rad/s.
  - `lsm6dsv16x_accl`: accelerometer data in g.
- RamFS command file `lsm6dsv16x`:
  - `show [time_ms] [interval_ms]`
  - `whoami`
  - `list_offset`
  - `cali`

## Hardware

Required LibXR objects:

- `ramfs`
- `database`
- SPI instance named by `spi_name`
- GPIO CS instance named by `cs_name`

The SPI object must use mode 0 (`CPOL=0`, `CPHA=0`). The module controls CS
with GPIO, so the target SPI driver should not rely on hardware CS framing for
multi-byte sensor transactions.

SPI wiring used during HPM5361EVKLite validation:

- `CS` -> `PA26`
- `SCLK` -> `PA27`
- `MISO` -> `PA28`
- `MOSI` -> `PA29`
- SPI mode 0

## Validation

Validated on HPM5361EVKLite with an LSM6DSV16X module over SPI1:

- GPIO bit-bang WHO_AM_I read: `0x70`.
- LibXR `SPI::MemRead` WHO_AM_I read: `0x70`.
- LibXR blocking `SPI::ReadAndWrite` WHO_AM_I read: `0x70`.
- LibXR DMA `SPI::ReadAndWrite` WHO_AM_I read: `0x70`.
- Register write/readback: `CTRL3` IF_INC and BDU bits verified.
- Burst sample reads produced stable gyro/accelerometer frames.
- DSLogic decode confirmed mode-0 frames such as `MOSI=[8F 00]`,
  `MISO=[00 70]` and burst reads from `0x20`.

The BSP-side test firmware and capture logs are intentionally not part of this
module repository.

## Example

```yaml
module: LSM6DSV16X
entry_header: Modules/LSM6DSV16X/LSM6DSV16X.hpp
constructor_args:
  - gyro_datarate: LSM6DSV16X::DataRate::DATA_RATE_120HZ
  - accel_datarate: LSM6DSV16X::DataRate::DATA_RATE_120HZ
  - accl_range: LSM6DSV16X::AcclRange::RANGE_8G
  - gyro_range: LSM6DSV16X::GyroRange::DPS_2000
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - poll_interval_ms: 2.0
  - task_stack_depth: 1024
  - spi_name: "spi_lsm6dsv16x"
  - cs_name: "lsm6dsv16x_cs"
  - gyro_topic_name: "lsm6dsv16x_gyro"
  - accl_topic_name: "lsm6dsv16x_accl"
template_args: []
```
