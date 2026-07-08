#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ST LSM6DSV16X 六轴 IMU 传感器模块 / ST LSM6DSV16X 6-axis IMU driver
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
required_hardware: ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "app_framework.hpp"
#include "database.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "spi.hpp"

#ifndef LIBXR_NO_EIGEN
#include "Eigen/Core"
#include "transform.hpp"
#endif

/**
 * @brief ST LSM6DSV16X 6-axis IMU module.
 * @brief_cn ST LSM6DSV16X 六轴 IMU 模块。
 *
 * @details The module talks to LSM6DSV16X over SPI mode 0 with a GPIO-managed
 * chip select, configures ODR/ranges, polls sensor data, and publishes gyro
 * data in rad/s plus accelerometer data in g.
 * On systems without real thread support, sampling is driven from OnMonitor()
 * instead of an auto-created worker thread.
 * @details_cn 模块使用 SPI mode 0 和 GPIO 手动片选访问 LSM6DSV16X，完成
 * ODR/量程配置，轮询读取数据，并发布 rad/s 单位陀螺仪数据与 g 单位加速度数据。
 * 在无线程支持的系统中，采样由 OnMonitor() 驱动，而不是自动创建后台线程。
 */
class LSM6DSV16X : public LibXR::Application
{
 public:
#ifdef LIBXR_NO_EIGEN
  /**
   * @brief Minimal 3D vector used when LibXR is built without Eigen.
   * @brief_cn LibXR 关闭 Eigen 时使用的轻量三维向量。
   */
  struct Vector3f
  {
    float data[3] {};

    Vector3f() = default;
    Vector3f(float x, float y, float z) : data{x, y, z} {}

    float& x() { return data[0]; }
    float& y() { return data[1]; }
    float& z() { return data[2]; }
    const float& x() const { return data[0]; }
    const float& y() const { return data[1]; }
    const float& z() const { return data[2]; }

    void setZero()
    {
      data[0] = 0.0f;
      data[1] = 0.0f;
      data[2] = 0.0f;
    }
  };

  /**
   * @brief Minimal quaternion for vector rotation when Eigen is unavailable.
   * @brief_cn 无 Eigen 时用于向量旋转的轻量四元数。
   */
  struct Quaternionf
  {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternionf() = default;
    Quaternionf(float w_in, float x_in, float y_in, float z_in)
        : w(w_in), x(x_in), y(y_in), z(z_in)
    {
    }

    Vector3f operator*(const Vector3f& v) const
    {
      const float tx = 2.0f * (y * v.z() - z * v.y());
      const float ty = 2.0f * (z * v.x() - x * v.z());
      const float tz = 2.0f * (x * v.y() - y * v.x());

      return Vector3f(v.x() + w * tx + (y * tz - z * ty),
                      v.y() + w * ty + (z * tx - x * tz),
                      v.z() + w * tz + (x * ty - y * tx));
    }
  };

  /** @brief Rotation quaternion type used by the module. */
  /** @brief_cn 模块使用的姿态旋转四元数类型。 */
  using Rotation = Quaternionf;

  /** @brief Gyroscope bias vector stored in the database, unit: rad/s. */
  /** @brief_cn 存入数据库的陀螺仪零偏向量，单位：rad/s。 */
  using BiasVector = Vector3f;

  /** @brief Raw gyroscope accumulator type used during calibration. */
  /** @brief_cn 校准时用于累加陀螺仪原始值的类型。 */
  using CaliVector = std::array<int64_t, 3>;
#else
  /** @brief 3D vector type used for sensor samples. */
  /** @brief_cn 传感器采样使用的三维向量类型。 */
  using Vector3f = Eigen::Matrix<float, 3, 1>;

  /** @brief Rotation quaternion type used by the module. */
  /** @brief_cn 模块使用的姿态旋转四元数类型。 */
  using Rotation = LibXR::Quaternion<float>;

  /** @brief Gyroscope bias vector stored in the database, unit: rad/s. */
  /** @brief_cn 存入数据库的陀螺仪零偏向量，单位：rad/s。 */
  using BiasVector = Eigen::Matrix<float, 3, 1>;

  /** @brief Raw gyroscope accumulator type used during calibration. */
  /** @brief_cn 校准时用于累加陀螺仪原始值的类型。 */
  using CaliVector = Eigen::Matrix<int64_t, 3, 1>;
#endif

  /** @brief LSM6DSV16X register addresses used by this module. */
  /** @brief_cn 本模块使用到的 LSM6DSV16X 寄存器地址。 */
  static constexpr uint8_t REG_WHO_AM_I = 0x0F;
  static constexpr uint8_t REG_CTRL1 = 0x10;
  static constexpr uint8_t REG_CTRL2 = 0x11;
  static constexpr uint8_t REG_CTRL3 = 0x12;
  static constexpr uint8_t REG_CTRL6 = 0x15;
  static constexpr uint8_t REG_CTRL8 = 0x17;
  static constexpr uint8_t REG_OUT_TEMP_L = 0x20;

  /** @brief Fixed device ID and control-bit masks. */
  /** @brief_cn 固定设备 ID 与控制位掩码。 */
  static constexpr uint8_t WHO_AM_I_VALUE = 0x70;
  static constexpr uint8_t CTRL3_SW_RESET = 0x01;
  static constexpr uint8_t CTRL3_IF_INC = 0x04;
  static constexpr uint8_t CTRL3_BDU = 0x40;
  static constexpr float DEG2RAD = 0.01745329251f;
  static constexpr size_t BURST_SIZE = 14;

  /**
   * @brief Output data rate setting for accelerometer or gyroscope.
   * @brief_cn 加速度计或陀螺仪输出数据率配置。
   */
  enum class DataRate : uint8_t
  {
    POWER_DOWN = 0x00,        ///< Power-down mode. / 掉电模式。
    DATA_RATE_1_875HZ = 0x01, ///< 1.875 Hz output data rate. / 1.875 Hz 输出数据率。
    DATA_RATE_7_5HZ = 0x02,   ///< 7.5 Hz output data rate. / 7.5 Hz 输出数据率。
    DATA_RATE_15HZ = 0x03,    ///< 15 Hz output data rate. / 15 Hz 输出数据率。
    DATA_RATE_30HZ = 0x04,    ///< 30 Hz output data rate. / 30 Hz 输出数据率。
    DATA_RATE_60HZ = 0x05,    ///< 60 Hz output data rate. / 60 Hz 输出数据率。
    DATA_RATE_120HZ = 0x06,   ///< 120 Hz output data rate. / 120 Hz 输出数据率。
    DATA_RATE_240HZ = 0x07,   ///< 240 Hz output data rate. / 240 Hz 输出数据率。
    DATA_RATE_480HZ = 0x08,   ///< 480 Hz output data rate. / 480 Hz 输出数据率。
    DATA_RATE_960HZ = 0x09,   ///< 960 Hz output data rate. / 960 Hz 输出数据率。
    DATA_RATE_1920HZ = 0x0A,  ///< 1920 Hz output data rate. / 1920 Hz 输出数据率。
    DATA_RATE_3840HZ = 0x0B,  ///< 3840 Hz output data rate. / 3840 Hz 输出数据率。
    DATA_RATE_7680HZ = 0x0C,  ///< 7680 Hz output data rate. / 7680 Hz 输出数据率。
  };

  /**
   * @brief Gyroscope full-scale range.
   * @brief_cn 陀螺仪满量程配置。
   */
  enum class GyroRange : uint8_t
  {
    DPS_125 = 0x00,  ///< ±125 dps full scale. / ±125 dps 满量程。
    DPS_250 = 0x01,  ///< ±250 dps full scale. / ±250 dps 满量程。
    DPS_500 = 0x02,  ///< ±500 dps full scale. / ±500 dps 满量程。
    DPS_1000 = 0x03, ///< ±1000 dps full scale. / ±1000 dps 满量程。
    DPS_2000 = 0x04, ///< ±2000 dps full scale. / ±2000 dps 满量程。
    DPS_4000 = 0x0C, ///< ±4000 dps full scale. / ±4000 dps 满量程。
  };

  /**
   * @brief Accelerometer full-scale range.
   * @brief_cn 加速度计满量程配置。
   */
  enum class AcclRange : uint8_t
  {
    RANGE_2G = 0x00,  ///< ±2 g full scale. / ±2 g 满量程。
    RANGE_4G = 0x01,  ///< ±4 g full scale. / ±4 g 满量程。
    RANGE_8G = 0x02,  ///< ±8 g full scale. / ±8 g 满量程。
    RANGE_16G = 0x03, ///< ±16 g full scale. / ±16 g 满量程。
  };

  /**
   * @brief Construct an LSM6DSV16X module.
   * @brief_cn 构造 LSM6DSV16X 模块。
   *
   * @param hw Hardware container. / 硬件容器。
   * @param app Application manager. / 应用管理器。
   * @param gyro_datarate Gyroscope ODR. / 陀螺仪输出数据率。
   * @param accel_datarate Accelerometer ODR. / 加速度计输出数据率。
   * @param accl_range Accelerometer full-scale range. / 加速度计满量程。
   * @param gyro_range Gyroscope full-scale range. / 陀螺仪满量程。
   * @param rotation Body-frame rotation quaternion. / 机体系旋转四元数。
   * @param poll_interval_ms Polling interval in milliseconds. / 轮询间隔，单位 ms。
   * @param task_stack_depth Sampling thread stack depth. / 采样线程栈深度。
   * @param spi_name SPI object name. / SPI 对象名称。
   * @param cs_name GPIO chip-select object name. / GPIO 片选对象名称。
   * @param gyro_topic_name Gyroscope topic name. / 陀螺仪发布话题名称。
   * @param accl_topic_name Accelerometer topic name. / 加速度计发布话题名称。
   */
  LSM6DSV16X(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             DataRate gyro_datarate, DataRate accel_datarate, AcclRange accl_range,
             GyroRange gyro_range, Rotation&& rotation,
             float poll_interval_ms, size_t task_stack_depth, const char* spi_name,
             const char* cs_name, const char* gyro_topic_name,
             const char* accl_topic_name)
      : gyro_datarate_(gyro_datarate),
        accel_datarate_(accel_datarate),
        accl_range_(accl_range),
        gyro_range_(gyro_range),
        rotation_(std::move(rotation)),
        poll_interval_ms_(poll_interval_ms),
        topic_gyro_(LibXR::Topic::CreateTopic<decltype(gyro_data_)>(gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<decltype(accl_data_)>(accl_topic_name)),
        cs_(hw.template FindOrExit<LibXR::GPIO>({cs_name})),
        spi_(hw.template FindOrExit<LibXR::SPI>({spi_name})),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("lsm6dsv16x", CommandFunc, this)),
        gyro_bias_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                       "lsm6dsv16x_gyro_bias", BiasVector(0.0f, 0.0f, 0.0f))
  {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    cs_->SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                    .pull = LibXR::GPIO::Pull::UP});
    cs_->Write(true);

    while (!Init())
    {
      XR_LOG_ERROR("LSM6DSV16X: Init failed. Try again.");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("LSM6DSV16X: Init succeeded.");

#if !defined(LIBXR_NOT_SUPPORT_MUTI_THREAD) || !(LIBXR_NOT_SUPPORT_MUTI_THREAD)
    thread_.Create(this, ThreadFunc, "lsm6dsv16x_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
#else
    UNUSED(task_stack_depth);
#endif
  }

  /**
   * @brief Monitor callback used by LibXR.
   * @brief_cn LibXR 监控回调。
   */
  void OnMonitor() override
  {
#if defined(LIBXR_NOT_SUPPORT_MUTI_THREAD) && (LIBXR_NOT_SUPPORT_MUTI_THREAD)
    const auto now = LibXR::Timebase::GetMilliseconds();
    if (uint32_t(now) - uint32_t(last_poll_ms_) >= PollIntervalMs())
    {
      last_poll_ms_ = now;
      PollOnce();
    }
#endif

    if (!std::isfinite(gyro_data_.x()) || !std::isfinite(gyro_data_.y()) ||
        !std::isfinite(gyro_data_.z()) || !std::isfinite(accl_data_.x()) ||
        !std::isfinite(accl_data_.y()) || !std::isfinite(accl_data_.z()))
    {
      XR_LOG_WARN("LSM6DSV16X: bad data");
    }
  }

 private:
  /**
   * @brief Probe, reset, configure, and verify the sensor.
   * @brief_cn 探测、复位、配置并读回校验传感器。
   *
   * @retval true Device is present and configuration readback matches. / 设备存在且配置读回匹配。
   * @retval false Device ID or configuration verification failed. / 设备 ID 或配置读回校验失败。
   */
  bool Init()
  {
    LibXR::Thread::Sleep(10);

    uint8_t value = 0;
    if (ReadSingle(REG_WHO_AM_I, value) != LibXR::ErrorCode::OK ||
        value != WHO_AM_I_VALUE)
    {
      XR_LOG_WARN("LSM6DSV16X: bad WHO_AM_I");
      return false;
    }

    if (WriteSingle(REG_CTRL3, CTRL3_SW_RESET) != LibXR::ErrorCode::OK)
    {
      XR_LOG_WARN("LSM6DSV16X: reset write failed");
      return false;
    }

    bool reset_done = false;
    for (uint8_t retry = 0; retry < 20; retry++)
    {
      if (ReadSingle(REG_CTRL3, value) == LibXR::ErrorCode::OK &&
          (value & CTRL3_SW_RESET) == 0)
      {
        reset_done = true;
        break;
      }
      LibXR::Thread::Sleep(1);
    }
    if (!reset_done)
    {
      XR_LOG_WARN("LSM6DSV16X: reset timeout");
      return false;
    }

    if (ReadSingle(REG_WHO_AM_I, value) != LibXR::ErrorCode::OK ||
        value != WHO_AM_I_VALUE)
    {
      XR_LOG_WARN("LSM6DSV16X: WHO_AM_I lost after reset");
      return false;
    }

    if (!ConfigureAndVerify())
    {
      return false;
    }

    last_sample_ts_ = LibXR::Timebase::GetMicroseconds();
    return true;
  }

  /**
   * @brief Configure CTRL registers and verify writable fields by readback.
   * @brief_cn 配置 CTRL 寄存器，并通过读回校验可写字段。
   */
  bool ConfigureAndVerify()
  {
    const uint8_t ctrl3 = CTRL3_IF_INC | CTRL3_BDU;
    const uint8_t ctrl6 = static_cast<uint8_t>(gyro_range_) & 0x0F;
    const uint8_t ctrl8 = static_cast<uint8_t>(accl_range_) & 0x03;
    const uint8_t ctrl1 = static_cast<uint8_t>(accel_datarate_) & 0x0F;
    const uint8_t ctrl2 = static_cast<uint8_t>(gyro_datarate_) & 0x0F;

    if (WriteSingle(REG_CTRL3, ctrl3) != LibXR::ErrorCode::OK ||
        WriteSingle(REG_CTRL6, ctrl6) != LibXR::ErrorCode::OK ||
        WriteSingle(REG_CTRL8, ctrl8) != LibXR::ErrorCode::OK ||
        WriteSingle(REG_CTRL1, ctrl1) != LibXR::ErrorCode::OK ||
        WriteSingle(REG_CTRL2, ctrl2) != LibXR::ErrorCode::OK)
    {
      XR_LOG_WARN("LSM6DSV16X: register configuration write failed");
      return false;
    }

    auto verify = [this](uint8_t reg, uint8_t mask, uint8_t expected,
                         const char* name) -> bool
    {
      uint8_t value = 0;
      if (ReadSingle(reg, value) != LibXR::ErrorCode::OK ||
          (value & mask) != expected)
      {
        XR_LOG_WARN("LSM6DSV16X: %s verify failed", name);
        return false;
      }
      return true;
    };

    if (!verify(REG_CTRL3, ctrl3, ctrl3, "CTRL3"))
    {
      return false;
    }
    if (!verify(REG_CTRL6, 0x0F, ctrl6, "CTRL6"))
    {
      return false;
    }
    if (!verify(REG_CTRL8, 0x03, ctrl8, "CTRL8"))
    {
      return false;
    }
    if (!verify(REG_CTRL1, 0x0F, ctrl1, "CTRL1"))
    {
      return false;
    }
    if (!verify(REG_CTRL2, 0x0F, ctrl2, "CTRL2"))
    {
      return false;
    }

    return true;
  }

  /**
   * @brief Polling worker that reads one complete sample frame and publishes topics.
   * @brief_cn 轮询采样线程：读取一帧完整传感器数据并发布话题。
   */
  static void ThreadFunc(LSM6DSV16X* self)
  {
    while (true)
    {
      self->PollOnce();
      LibXR::Thread::Sleep(self->PollIntervalMs());
    }
  }

  /**
   * @brief Read, parse, and publish one sensor sample.
   * @brief_cn 读取、解析并发布一帧传感器采样。
   */
  void PollOnce()
  {
    if (ReadBurst(REG_OUT_TEMP_L, buffer_.data(), BURST_SIZE) == LibXR::ErrorCode::OK)
    {
      consecutive_read_errors_ = 0;
      Parse();
      const auto sample_ts = last_sample_ts_;
      topic_accl_.Publish(accl_data_, sample_ts);
      topic_gyro_.Publish(gyro_data_, sample_ts);
      return;
    }

    consecutive_read_errors_++;
    if (consecutive_read_errors_ == 1 || consecutive_read_errors_ % 100 == 0)
    {
      XR_LOG_WARN("LSM6DSV16X: sample read failed %u times",
                  consecutive_read_errors_);
    }
  }

  /**
   * @brief Clamp and convert the configured polling interval to milliseconds.
   * @brief_cn 限幅并转换配置的轮询间隔到毫秒。
   */
  uint32_t PollIntervalMs() const
  {
    return static_cast<uint32_t>(
        std::clamp(poll_interval_ms_, 1.0f, 1000.0f));
  }

  /**
   * @brief Wait for a period while keeping sampling alive on single-thread builds.
   * @brief_cn 等待指定时间；在无线程构建中同步驱动采样。
   */
  void WaitWithSampling(uint32_t milliseconds)
  {
#if defined(LIBXR_NOT_SUPPORT_MUTI_THREAD) && (LIBXR_NOT_SUPPORT_MUTI_THREAD)
    uint32_t elapsed = 0;
    const auto interval = PollIntervalMs();
    while (elapsed < milliseconds)
    {
      PollOnce();
      const auto step = std::min(interval, milliseconds - elapsed);
      LibXR::Thread::Sleep(step);
      elapsed += step;
    }
#else
    LibXR::Thread::Sleep(milliseconds);
#endif
  }

  /**
   * @brief Read one LSM6DSV16X register through SPI MemRead.
   * @brief_cn 通过 SPI MemRead 读取单个 LSM6DSV16X 寄存器。
   */
  LibXR::ErrorCode ReadSingle(uint8_t reg, uint8_t& data)
  {
    LibXR::Mutex::LockGuard lock(spi_mutex_);
    cs_->Write(false);
    const auto ans = spi_->MemRead(reg, LibXR::RawData(&data, 1), op_spi_);
    cs_->Write(true);
    return ans;
  }

  /**
   * @brief Read a contiguous register block with one CS frame.
   * @brief_cn 在一次片选帧内读取连续寄存器块。
   */
  LibXR::ErrorCode ReadBurst(uint8_t reg, uint8_t* data, size_t len)
  {
    LibXR::Mutex::LockGuard lock(spi_mutex_);
    cs_->Write(false);
    const auto ans = spi_->MemRead(reg, LibXR::RawData(data, len), op_spi_);
    cs_->Write(true);
    return ans;
  }

  /**
   * @brief Write one LSM6DSV16X register through SPI MemWrite.
   * @brief_cn 通过 SPI MemWrite 写入单个 LSM6DSV16X 寄存器。
   */
  LibXR::ErrorCode WriteSingle(uint8_t reg, uint8_t data)
  {
    LibXR::Mutex::LockGuard lock(spi_mutex_);
    cs_->Write(false);
    const auto ans = spi_->MemWrite(reg, LibXR::ConstRawData(&data, 1), op_spi_);
    cs_->Write(true);
    return ans;
  }

  /**
   * @brief Parse raw temperature, gyroscope, and accelerometer data.
   * @brief_cn 解析温度、陀螺仪和加速度计原始数据。
   *
   * @details Gyroscope output is converted to rad/s; accelerometer output is
   * converted to g. The configured rotation is applied before publication.
   * @details_cn 陀螺仪输出转换为 rad/s，加速度计输出转换为 g，并在发布前应用
   * 配置的坐标旋转。
   */
  void Parse()
  {
    const int16_t temp_raw = MakeInt16(buffer_[1], buffer_[0]);
    const int16_t gx = MakeInt16(buffer_[3], buffer_[2]);
    const int16_t gy = MakeInt16(buffer_[5], buffer_[4]);
    const int16_t gz = MakeInt16(buffer_[7], buffer_[6]);
    const int16_t ax = MakeInt16(buffer_[9], buffer_[8]);
    const int16_t ay = MakeInt16(buffer_[11], buffer_[10]);
    const int16_t az = MakeInt16(buffer_[13], buffer_[12]);

    const float gyro_lsb = GetGyroLSB();
    const float accl_lsb = GetAcclLSB();

    Vector3f gyro(static_cast<float>(gx) * gyro_lsb * DEG2RAD,
                  static_cast<float>(gy) * gyro_lsb * DEG2RAD,
                  static_cast<float>(gz) * gyro_lsb * DEG2RAD);
    Vector3f accl(static_cast<float>(ax) * accl_lsb,
                  static_cast<float>(ay) * accl_lsb,
                  static_cast<float>(az) * accl_lsb);

    gyro.x() -= gyro_bias_key_.data_.x();
    gyro.y() -= gyro_bias_key_.data_.y();
    gyro.z() -= gyro_bias_key_.data_.z();

    gyro_data_ = rotation_ * gyro;
    accl_data_ = rotation_ * accl;
    temperature_ = 25.0f + static_cast<float>(temp_raw) / 256.0f;

    if (in_cali_)
    {
      gyro_cali_[0] += gx;
      gyro_cali_[1] += gy;
      gyro_cali_[2] += gz;
      cali_counter_++;
    }

    const auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = now - last_sample_ts_;
    last_sample_ts_ = now;
  }

  /**
   * @brief Combine two little-endian bytes into a signed 16-bit sample.
   * @brief_cn 将两个小端字节合成为有符号 16 位采样值。
   */
  static int16_t MakeInt16(uint8_t msb, uint8_t lsb)
  {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
  }

  /**
   * @brief Return accelerometer sensitivity in g/LSB.
   * @brief_cn 返回加速度计灵敏度，单位 g/LSB。
   */
  float GetAcclLSB() const
  {
    switch (accl_range_)
    {
      case AcclRange::RANGE_2G:
        return 0.061f / 1000.0f;
      case AcclRange::RANGE_4G:
        return 0.122f / 1000.0f;
      case AcclRange::RANGE_8G:
        return 0.244f / 1000.0f;
      case AcclRange::RANGE_16G:
        return 0.488f / 1000.0f;
    }
    return 0.061f / 1000.0f;
  }

  /**
   * @brief Return gyroscope sensitivity in dps/LSB.
   * @brief_cn 返回陀螺仪灵敏度，单位 dps/LSB。
   */
  float GetGyroLSB() const
  {
    switch (gyro_range_)
    {
      case GyroRange::DPS_125:
        return 4.375f / 1000.0f;
      case GyroRange::DPS_250:
        return 8.750f / 1000.0f;
      case GyroRange::DPS_500:
        return 17.50f / 1000.0f;
      case GyroRange::DPS_1000:
        return 35.0f / 1000.0f;
      case GyroRange::DPS_2000:
        return 70.0f / 1000.0f;
      case GyroRange::DPS_4000:
        return 140.0f / 1000.0f;
    }
    return 70.0f / 1000.0f;
  }

  /**
   * @brief Convert a float to a scaled integer for no-float printf builds.
   * @brief_cn 为关闭浮点 printf 的构建将浮点值转换为缩放整数。
   */
  static int32_t ScaleToInt(float value, float scale)
  {
    return static_cast<int32_t>(value * scale);
  }

  /**
   * @brief RamFS command entry: whoami, show, list_offset, and cali.
   * @brief_cn RamFS 命令入口：whoami、show、list_offset 和 cali。
   */
  static int CommandFunc(LSM6DSV16X* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<"  whoami\r\n">();
      LibXR::STDIO::Printf<"  show [time_ms] [interval_ms]\r\n">();
      LibXR::STDIO::Printf<"  list_offset\r\n">();
      LibXR::STDIO::Printf<"  cali\r\n">();
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "whoami") == 0)
    {
      uint8_t whoami = 0;
      const auto ans = self->ReadSingle(REG_WHO_AM_I, whoami);
      if (ans != LibXR::ErrorCode::OK)
      {
        LibXR::STDIO::Printf<"WHO_AM_I read failed: %d\r\n">(
            static_cast<int>(ans));
        return -1;
      }
      LibXR::STDIO::Printf<"WHO_AM_I: 0x%02X\r\n">(whoami);
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "list_offset") == 0)
    {
      LibXR::STDIO::Printf<"bias_mrad_s: %d %d %d\r\n">(
          ScaleToInt(self->gyro_bias_key_.data_.x(), 1000.0f),
          ScaleToInt(self->gyro_bias_key_.data_.y(), 1000.0f),
          ScaleToInt(self->gyro_bias_key_.data_.z(), 1000.0f));
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "cali") == 0)
    {
      self->gyro_bias_key_.data_.setZero();
      self->gyro_cali_ = CaliVector{0, 0, 0};
      self->cali_counter_ = 0;
      self->in_cali_ = true;

      LibXR::STDIO::Printf<"Starting LSM6DSV16X gyroscope calibration. Keep still.\r\n">();
      self->WaitWithSampling(3000);
      for (int i = 0; i < 10; i++)
      {
        LibXR::STDIO::Printf<"Progress: %d / 10\r">(i + 1);
        self->WaitWithSampling(1000);
      }
      LibXR::STDIO::Printf<"\r\nProgress: Done\r\n">();

      self->in_cali_ = false;
      if (self->cali_counter_ == 0)
      {
        LibXR::STDIO::Printf<"LSM6DSV16X calibration failed: no samples.\r\n">();
        return -1;
      }

      const float scale = self->GetGyroLSB() * DEG2RAD;
      self->gyro_bias_key_.data_.x() =
          static_cast<float>(self->gyro_cali_[0]) /
          static_cast<float>(self->cali_counter_) * scale;
      self->gyro_bias_key_.data_.y() =
          static_cast<float>(self->gyro_cali_[1]) /
          static_cast<float>(self->cali_counter_) * scale;
      self->gyro_bias_key_.data_.z() =
          static_cast<float>(self->gyro_cali_[2]) /
          static_cast<float>(self->cali_counter_) * scale;
      self->gyro_bias_key_.Set(self->gyro_bias_key_.data_);

      LibXR::STDIO::Printf<"bias_mrad_s saved: %d %d %d\r\n">(
          ScaleToInt(self->gyro_bias_key_.data_.x(), 1000.0f),
          ScaleToInt(self->gyro_bias_key_.data_.y(), 1000.0f),
          ScaleToInt(self->gyro_bias_key_.data_.z(), 1000.0f));
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0)
    {
      int time = std::atoi(argv[2]);
      int delay = std::clamp(std::atoi(argv[3]), 2, 1000);
      while (time > 0)
      {
        LibXR::STDIO::Printf<
            "acc_mg:%d %d %d | gyr_mrad_s:%d %d %d | temp_centi_c:%d\r\n">(
            ScaleToInt(self->accl_data_.x(), 1000.0f),
            ScaleToInt(self->accl_data_.y(), 1000.0f),
            ScaleToInt(self->accl_data_.z(), 1000.0f),
            ScaleToInt(self->gyro_data_.x(), 1000.0f),
            ScaleToInt(self->gyro_data_.y(), 1000.0f),
            ScaleToInt(self->gyro_data_.z(), 1000.0f),
            ScaleToInt(self->temperature_, 100.0f));
        LibXR::Thread::Sleep(delay);
        time -= delay;
      }
      return 0;
    }

    LibXR::STDIO::Printf<"bad args\r\n">();
    return -1;
  }

  DataRate gyro_datarate_;
  DataRate accel_datarate_;
  AcclRange accl_range_;
  GyroRange gyro_range_;

  std::array<uint8_t, BURST_SIZE> buffer_{};
  Vector3f gyro_data_{0.0f, 0.0f, 0.0f};
  Vector3f accl_data_{0.0f, 0.0f, 0.0f};

  Rotation rotation_;
  float poll_interval_ms_ = 2.0f;
  float temperature_ = 0.0f;

  LibXR::Topic topic_gyro_;
  LibXR::Topic topic_accl_;

  LibXR::GPIO* cs_ = nullptr;
  LibXR::SPI* spi_ = nullptr;

  LibXR::Semaphore sem_spi_;
  LibXR::SPI::OperationRW op_spi_;
  LibXR::Mutex spi_mutex_;

  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<BiasVector> gyro_bias_key_;

  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  uint32_t consecutive_read_errors_ = 0;
  CaliVector gyro_cali_{0, 0, 0};

  LibXR::MicrosecondTimestamp last_sample_ts_ = 0;
  LibXR::MicrosecondTimestamp::Duration dt_ = 0;
  LibXR::MillisecondTimestamp last_poll_ms_ = 0;

  LibXR::Thread thread_;
};
