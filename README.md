# LSM6DSV16X

ST LSM6DSV16X 6 轴 IMU（SPI）驱动模块 / Driver Module for the ST LSM6DSV16X 6-axis IMU over SPI

## 1. 模块作用 / Purpose

LSM6DSV16X 通过 SPI 模式 0 访问芯片，片选由模块通过 GPIO 控制。加速度计与陀螺仪的输出数据率和量程可配置。初始化时寄存器配置通过读回校验；失败时构造函数每 100 ms 重试，直到成功。

采样线程 `lsm6dsv16x_thread`（REALTIME 优先级）以轮询方式每隔 `poll_interval_ms`（限制在 1 到 1000 ms）读取一帧完整数据并发布两个 Topic；SPI 读取失败时丢弃该帧，连续失败的第一次输出警告，下一周期重新读取：

- `gyro_topic_name`（默认 `lsm6dsv16x_gyro`）：陀螺仪，单位 rad/s，已去零偏并经 `rotation` 旋转。
- `accl_topic_name`（默认 `lsm6dsv16x_accl`）：加速度计，单位 g，已经 `rotation` 旋转。

陀螺仪零偏保存在 Database 的键 `lsm6dsv16x_gyro_bias` 中。`OnMonitor()` 在最新采样不是有限值时输出警告。

模块在 RamFS 中注册命令文件 `lsm6dsv16x`：

- `whoami`：打印 WHO_AM_I 寄存器。
- `show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms`（2 到 1000 ms）打印一次加速度（mg）、角速度（mrad/s）和温度（0.01 °C）。
- `list_offset`：打印已保存的陀螺仪零偏。
- `cali`：陀螺仪零偏校准，期间传感器保持静止。先等待 3 s，再对陀螺仪采集 10 s 求平均，约 13 s 后保存零偏。

The LSM6DSV16X accesses the chip over SPI mode 0, with the chip select driven by the Module through a GPIO. The output data rate and range of the accelerometer and the gyroscope are configurable. During initialization the register configuration is verified by readback; on failure the constructor retries every 100 ms until it succeeds.

The sampling thread `lsm6dsv16x_thread` (REALTIME priority) polls and reads one complete frame every `poll_interval_ms` (limited to 1 to 1000 ms) and publishes two Topics; when the SPI read fails, the frame is dropped, a warning is logged on the first failure of a run, and the next cycle reads again:

- `gyro_topic_name` (default `lsm6dsv16x_gyro`): gyroscope in rad/s, zero offset removed and rotated by `rotation`.
- `accl_topic_name` (default `lsm6dsv16x_accl`): accelerometer in g, rotated by `rotation`.

The gyroscope zero offset is stored in the Database under the key `lsm6dsv16x_gyro_bias`. `OnMonitor()` logs a warning when the latest sample is not finite.

The Module registers the command file `lsm6dsv16x` in RamFS:

- `whoami`: print the WHO_AM_I register.
- `show <time_ms> <interval_ms>`: print the acceleration (mg), angular velocity (mrad/s) and temperature (0.01 °C) every `interval_ms` (2 to 1000 ms) for `time_ms`.
- `list_offset`: print the stored gyroscope zero offset.
- `cali`: gyroscope zero-offset calibration, with the sensor held still. It waits 3 s, averages the gyroscope for 10 s and saves the zero offset after about 13 s.

## 2. 时间戳约定 / Timestamp Convention

`gyro_topic_name` 与 `accl_topic_name` 两个 Topic 使用同一帧数据读取完成时的时间戳（µs）发布，消费者读取 Topic 的 envelope timestamp。

The Topics `gyro_topic_name` and `accl_topic_name` are published with the timestamp (µs) taken when the same frame has been read; consumers read the Topic envelope timestamp.

## 3. 构造接口 / Constructor

```cpp
LSM6DSV16X(LibXR::SPI& spi,
           LibXR::GPIO& cs,
           LibXR::Database& database,
           LibXR::RamFS& ramfs,
           const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `spi`：连接传感器的 `LibXR::SPI`，使用模式 0（`CPOL=0`，`CPHA=0`），取自 BSP 的硬件注册（`XR_REGISTER`）。
- `cs`：用作片选的 `LibXR::GPIO`，模块将其配置为推挽输出；多字节事务的帧由该 GPIO 界定。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `lsm6dsv16x` 命令文件的 `LibXR::RamFS`。

配置参数（`Param`）：

- `gyro_datarate`、`accel_datarate`：陀螺仪与加速度计的输出数据率 `LSM6DSV16X::DataRate`，默认 `DATA_RATE_120HZ`。
- `accl_range`：加速度计量程 `LSM6DSV16X::AcclRange`，默认 `RANGE_8G`。
- `gyro_range`：陀螺仪量程 `LSM6DSV16X::GyroRange`，默认 `DPS_2000`。
- `rotation`：机体系旋转四元数 `{w, x, y, z}`，默认 `{1, 0, 0, 0}`。
- `poll_interval_ms`：轮询间隔，单位 ms，默认 2.0。
- `task_stack_depth`：轮询线程栈深，默认 1024。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `"lsm6dsv16x_gyro"`、`"lsm6dsv16x_accl"`。

Dependencies:

- `spi`: the `LibXR::SPI` connected to the sensor, using mode 0 (`CPOL=0`, `CPHA=0`), taken from the BSP's Registration (`XR_REGISTER`).
- `cs`: the `LibXR::GPIO` used as chip select, configured by the Module as a push-pull output; the frame of a multi-byte transaction is delimited by this GPIO.
- `database`: the `LibXR::Database` that stores the gyroscope zero offset.
- `ramfs`: the `LibXR::RamFS` that receives the `lsm6dsv16x` command file.

Configuration parameters (`Param`):

- `gyro_datarate`, `accel_datarate`: output data rate `LSM6DSV16X::DataRate` of the gyroscope and the accelerometer, default `DATA_RATE_120HZ`.
- `accl_range`: accelerometer range `LSM6DSV16X::AcclRange`, default `RANGE_8G`.
- `gyro_range`: gyroscope range `LSM6DSV16X::GyroRange`, default `DPS_2000`.
- `rotation`: body-frame rotation quaternion `{w, x, y, z}`, default `{1, 0, 0, 0}`.
- `poll_interval_ms`: polling interval in ms, default 2.0.
- `task_stack_depth`: stack depth of the polling thread, default 1024.
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, default `"lsm6dsv16x_gyro"` and `"lsm6dsv16x_accl"`.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gyro_topic_name`（默认 `lsm6dsv16x_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `lsm6dsv16x_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `gyro_topic_name` (default `lsm6dsv16x_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, zero offset removed and rotated |
| `accl_topic_name` (default `lsm6dsv16x_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/LSM6DSV16X` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/LSM6DSV16X`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

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
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          poll_interval_ms: 2.0f
          task_stack_depth: 1024
          gyro_topic_name: "lsm6dsv16x_gyro"
          accl_topic_name: "lsm6dsv16x_accl"
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。未定义 `LIBXR_NO_EIGEN` 时使用 Eigen 向量和四元数；定义时使用头文件内置的轻量类型。

硬件：一片通过 SPI 连接的 LSM6DSV16X，带一个片选 GPIO；SPI、GPIO、Database 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR. Eigen vectors and quaternions are used when `LIBXR_NO_EIGEN` is not defined; when it is defined, the header uses its own lightweight types.

Hardware: one LSM6DSV16X connected over SPI with a chip-select GPIO; the SPI, GPIO, Database and RamFS are registered by the BSP with `XR_REGISTER`.
