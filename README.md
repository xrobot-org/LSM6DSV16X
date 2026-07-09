# LSM6DSV16X

ST LSM6DSV16X 六轴 IMU 的 xrobot / LibXR 驱动模块。

## 功能

- 使用 SPI mode 0 访问寄存器，并通过 GPIO 手动控制片选。
- 支持配置加速度计和陀螺仪的 ODR / 量程。
- 使用轮询采样线程，不依赖 data-ready 中断。
- 初始化时会回读校验寄存器配置。
- 发布数据：
  - `lsm6dsv16x_gyro`：单位为 rad/s 的陀螺仪数据。
  - `lsm6dsv16x_accl`：单位为 g 的加速度计数据。
- 提供 RamFS 命令文件 `lsm6dsv16x`：
  - `show [time_ms] [interval_ms]`
  - `whoami`
  - `list_offset`
  - `cali`

## 硬件连接

需要提供以下 LibXR 对象：

- `ramfs`
- `database`
- 由 `spi_name` 指定的 SPI 实例
- 由 `cs_name` 指定的 GPIO 片选实例

SPI 对象必须使用 mode 0（`CPOL=0`，`CPHA=0`）。模块内部通过 GPIO 控制
CS，因此目标 SPI 驱动不应依赖硬件 CS 对多字节传感器事务进行自动分帧。

HPM5361EVKLite 实机验证时使用的 SPI 接线：

- `CS` -> `PA26`
- `SCLK` -> `PA27`
- `MISO` -> `PA28`
- `MOSI` -> `PA29`
- SPI mode 0

## 实机验证

已在 HPM5361EVKLite 上通过 SPI1 连接 LSM6DSV16X 模块完成验证：

- GPIO 模拟 SPI 读取 WHO_AM_I：`0x70`。
- LibXR `SPI::MemRead` 读取 WHO_AM_I：`0x70`。
- LibXR 阻塞 `SPI::ReadAndWrite` 读取 WHO_AM_I：`0x70`。
- LibXR DMA `SPI::ReadAndWrite` 读取 WHO_AM_I：`0x70`。
- 寄存器写入 / 回读：已验证 `CTRL3` 的 IF_INC 和 BDU 位。
- 连续读取采样数据时，陀螺仪和加速度计帧稳定。
- DSLogic 解码确认 mode-0 帧格式正确，例如 `MOSI=[8F 00]`、
  `MISO=[00 70]`，以及从 `0x20` 开始的连续读取。

## 配置示例

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
