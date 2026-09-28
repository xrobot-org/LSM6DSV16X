# LSM6DSV16X

ST LSM6DSV16X 6-axis IMU driver Module for XRobot / LibXR.

- SPI mode 0 register access with a GPIO chip select driven by the Module.
- Configurable accelerometer and gyroscope output data rate and range.
- Register configuration is verified by readback during initialization; the
  constructor retries initialization every 100 ms until it succeeds.
- A polling thread (`lsm6dsv16x_thread`, realtime priority) reads one sample frame
  every `poll_interval_ms` (clamped to 1-1000 ms); no data-ready interrupt is needed.
- Publishes, with the sample timestamp:
  - `lsm6dsv16x_gyro`: gyroscope in rad/s, gyro bias removed, rotated by `rotation`.
  - `lsm6dsv16x_accl`: accelerometer in g, rotated by `rotation`.
- Stores the gyroscope bias in the database key `lsm6dsv16x_gyro_bias`.
- `OnMonitor()` logs a warning when the latest sample is not finite.
- RamFS command file `lsm6dsv16x`:
  - `whoami`: print the WHO_AM_I register.
  - `show <time_ms> <interval_ms>`: print acceleration (mg), angular rate (mrad/s)
    and temperature (0.01 degC) for `time_ms`, every `interval_ms` (2-1000).
  - `list_offset`: print the stored gyroscope bias.
  - `cali`: keep the sensor still; averages the gyroscope for about 13 s and saves the
    bias.

The SPI object must use mode 0 (`CPOL=0`, `CPHA=0`). The Module toggles the chip
select GPIO itself, so the SPI driver must not rely on hardware CS framing for
multi-byte transactions.

## Dependencies

No other Modules; LibXR only. Eigen vectors and quaternions are used unless
`LIBXR_NO_EIGEN` is defined, in which case the header uses its own small types.

## Constructor

```cpp
LSM6DSV16X(LibXR::SPI& spi,
           LibXR::GPIO& cs,
           LibXR::Database& database,
           LibXR::RamFS& ramfs,
           const Param& param = {...});
```

Dependencies:

- `spi`: SPI bus connected to the sensor (mode 0).
- `cs`: GPIO used as chip select (configured as push-pull output).
- `database`: stores the gyroscope bias.
- `ramfs`: receives the `lsm6dsv16x` command file.

Configuration (`Param`, defaults in parentheses):

- `gyro_datarate`, `accel_datarate`: `LSM6DSV16X::DataRate` (`DATA_RATE_120HZ`).
- `accl_range`: `LSM6DSV16X::AcclRange` (`RANGE_8G`).
- `gyro_range`: `LSM6DSV16X::GyroRange` (`DPS_2000`).
- `rotation`: body-frame rotation quaternion `{w, x, y, z}` (`{1, 0, 0, 0}`).
- `poll_interval_ms`: polling interval in ms (`2.0`).
- `task_stack_depth`: polling thread stack depth (`1024`).
- `gyro_topic_name`, `accl_topic_name`: topic names (`"lsm6dsv16x_gyro"`,
  `"lsm6dsv16x_accl"`).

## Use

```sh
xrobot module add xrobot-org/LSM6DSV16X
xrobot setup
xrobot instance add xrobot-org/LSM6DSV16X
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; fill the dependencies with objects the BSP
registers:

```yaml
modules:
  - module: xrobot-org/LSM6DSV16X
    id: lsm6dsv16x_0
    args:
      - spi: spi1
      - cs: lsm6dsv16x_cs
      - database: database
      - ramfs: ramfs
      - param:
          gyro_datarate: LSM6DSV16X::DataRate::DATA_RATE_120HZ
          accel_datarate: LSM6DSV16X::DataRate::DATA_RATE_120HZ
          accl_range: LSM6DSV16X::AcclRange::RANGE_8G
          gyro_range: LSM6DSV16X::GyroRange::DPS_2000
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          poll_interval_ms: 2.0f
          task_stack_depth: '1024'
          gyro_topic_name: '"lsm6dsv16x_gyro"'
          accl_topic_name: '"lsm6dsv16x_accl"'
```

BSP side:

```cpp
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(lsm6dsv16x_cs, LibXR::GPIO);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/LSM6DSV16X` in a BSP, prints the current
constructor.

## Validation

Validated on HPM5361EVKLite with an LSM6DSV16X module over SPI1 (before the
static-assembly constructor change; the register access code is unchanged):

- WHO_AM_I reads `0x70` through GPIO bit-bang, LibXR `SPI::MemRead`, blocking
  `SPI::ReadAndWrite` and DMA `SPI::ReadAndWrite`.
- Register write/readback of the `CTRL3` IF_INC and BDU bits.
- Burst reads produce stable gyroscope/accelerometer frames.
- A logic-analyzer decode confirmed mode-0 frames such as `MOSI=[8F 00]`,
  `MISO=[00 70]` and burst reads from `0x22`.
