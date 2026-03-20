#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <EscDriver.h>
#include "Debug_Espfc.h"
#include "ModelConfig.h"
#include "ModelState.h"
#include "Utils/Storage.h"
#include "Utils/Logger.h"
#include "Utils/Math.hpp"

namespace Espfc {

/**
 * @brief Trung tâm trạng thái và cấu hình của toàn bộ flight controller
 *
 * `Model` là struct dữ liệu chia sẻ duy nhất, được truyền bằng tham chiếu vào
 * mọi subsystem. Gồm hai phần:
 *   - `config` (`ModelConfig`): cấu hình lưu vào EEPROM, đọc/ghi qua `load()`/`save()`
 *   - `state` (`ModelState`): trạng thái runtime — gyro, accel, PID, output, mode masks
 *
 * Trên ESP32 dual-core, `state` được truy cập từ cả hai core. Các trường nhạy cảm
 * (ví dụ: `appQueue`) sử dụng cơ chế đồng bộ nội bộ. Tránh thêm global state ngoài class này.
 *
 * @note `state` KHÔNG được reset khi khởi động trên board thật (tránh WDT reset).
 *       Chỉ reset trong `UNIT_TEST`.
 */
class Model
{
  public:
    Model()
    {
      initialize();
    }

    /**
     * @brief Khởi tạo `config` về giá trị mặc định
     *
     * Reset `ModelConfig` về default. Trong `UNIT_TEST` cũng reset `ModelState`.
     * Trên board thật, `state` KHÔNG được reset ở đây (tránh WDT reset do zero-fill bộ nhớ lớn).
     */
    void initialize()
    {
      config = ModelConfig();
      #ifdef UNIT_TEST
      state = ModelState(); // FIXME: causes board wdt reset
      #endif
      //config.brobot();
    }

    /**
     * @brief Kiểm tra một flight mode có đang active không
     *
     * @param mode Flight mode cần kiểm tra (ví dụ: MODE_ARMED, MODE_ANGLE, MODE_ALTHOLD)
     * @return true nếu mode đang được bật
     */
    bool isModeActive(FlightMode mode) const
    {
      return state.mode.mask & (1 << mode);
    }

    /**
     * @brief Kiểm tra một flight mode có thay đổi trạng thái so với chu kỳ trước không
     *
     * So sánh `mask` hiện tại với `maskPrev` để phát hiện transition on/off.
     *
     * @param mode Flight mode cần kiểm tra
     * @return true nếu trạng thái mode đã thay đổi trong chu kỳ hiện tại
     */
    bool hasChanged(FlightMode mode) const
    {
      return (state.mode.mask & (1 << mode)) != (state.mode.maskPrev & (1 << mode));
    }

    /**
     * @brief Tắt một flight mode và lưu trạng thái trước đó vào maskPrev
     *
     * @param mode Flight mode cần tắt
     */
    void clearMode(FlightMode mode)
    {
      state.mode.maskPrev |= state.mode.mask & (1 << mode);
      state.mode.mask &= ~(1 << mode);
    }

    /**
     * @brief Cập nhật toàn bộ flight mode mask từ RC input
     *
     * Lưu mask cũ vào `maskPrev` trước khi ghi mask mới, phục vụ phát hiện transition.
     *
     * @param mask Bitmask flight mode mới (tính từ RC switches)
     */
    void updateModes(uint32_t mask)
    {
      state.mode.maskPrev = state.mode.mask;
      state.mode.mask = mask;
    }

    /**
     * @brief Kiểm tra trạng thái switch vật lý của một flight mode (chưa qua logic armed)
     *
     * @param mode Flight mode cần kiểm tra
     * @return true nếu switch RC tương ứng đang ở vị trí bật
     */
    bool isSwitchActive(FlightMode mode) const
    {
      return state.mode.maskSwitch & (1 << mode);
    }

    /**
     * @brief Cập nhật trạng thái switch RC (maskSwitch) từ RC input processor
     *
     * @param mask Bitmask switch RC mới
     */
    void updateSwitchActive(uint32_t mask)
    {
      state.mode.maskSwitch = mask;
    }

    /**
     * @brief Disarm cưỡng bức và ghi lý do vào log
     *
     * Tắt MODE_ARMED và MODE_AIRMODE, ghi `disarmReason`, gửi EVENT_DISARM
     * vào `appQueue` để pidTask xử lý motor stop.
     *
     * @param r Lý do disarm (failsafe, throttle thấp, RC mất tín hiệu, v.v.)
     * @warning Gọi hàm này từ bất kỳ task nào đều an toàn — appQueue là thread-safe.
     */
    void disarm(DisarmReason r)
    {
      state.mode.disarmReason = r;
      clearMode(MODE_ARMED);
      clearMode(MODE_AIRMODE);
      state.appQueue.send(Event(EVENT_DISARM));
    }

    /**
     * @brief Kiểm tra một feature có được bật trong cấu hình không
     *
     * @param feature Feature cần kiểm tra (ví dụ: FEATURE_SOFTSERIAL, FEATURE_TELEMETRY)
     * @return true nếu feature đang được bật
     */
    bool isFeatureActive(Feature feature) const
    {
      return config.featureMask & feature;
    }

    /**
     * @brief Kiểm tra Airmode có đang active không
     *
     * @return true nếu MODE_AIRMODE đang bật
     */
    bool isAirModeActive() const
    {
      return isModeActive(MODE_AIRMODE);// || isFeatureActive(FEATURE_AIRMODE);
    }

    /**
     * @brief Kiểm tra throttle có ở mức thấp dưới ngưỡng minCheck không
     *
     * Dùng để quyết định có reset I-term (khi không ở Airmode) hay không.
     *
     * @return true nếu tín hiệu throttle (µs) nhỏ hơn `config.input.minCheck`
     */
    bool isThrottleLow() const
    {
      return state.input.us[AXIS_THRUST] < config.input.minCheck;
    }

    /**
     * @brief Kiểm tra blackbox có được cấu hình và bật hay không
     *
     * @return true nếu thiết bị blackbox là serial hoặc flash, và pDenom > 0
     */
    bool blackboxEnabled() const
    {
      // serial or flash
      return (config.blackbox.dev == BLACKBOX_DEV_SERIAL || config.blackbox.dev == BLACKBOX_DEV_FLASH) && config.blackbox.pDenom > 0;
    }

    /**
     * @brief Kiểm tra gyro có sẵn sàng hoạt động không
     *
     * @return true nếu gyro được phát hiện và không bị disable trong config
     * @note Hàm này có thể được gọi trong gyroTask (IRAM context)
     */
    bool gyroActive() const /* IRAM_ATTR */
    {
      return state.gyro.present && config.gyro.dev != GYRO_NONE;
    }

    /**
     * @brief Kiểm tra GPS có đang hoạt động không
     *
     * @return true nếu GPS đã được phát hiện
     */
    bool gpsActive() const /* IRAM_ATTR */
    {
      return state.gps.present;
    }

    /**
     * @brief Kiểm tra accelerometer có đang hoạt động không
     *
     * @return true nếu accel được phát hiện và không bị disable trong config
     */
    bool accelActive() const
    {
      return state.accel.present && config.accel.dev != GYRO_NONE;
    }

    /**
     * @brief Kiểm tra magnetometer có đang hoạt động không
     *
     * @return true nếu mag được phát hiện và không bị disable trong config
     */
    bool magActive() const
    {
      return state.mag.present && config.mag.dev != MAG_NONE;
    }

    /**
     * @brief Kiểm tra barometer có đang hoạt động không
     *
     * @return true nếu baro được phát hiện và không bị disable trong config
     */
    bool baroActive() const
    {
      return state.baro.present && config.baro.dev != BARO_NONE;
    }

    /**
     * @brief Kiểm tra có cảm biến nào đang trong quá trình hiệu chỉnh không
     *
     * @return true nếu gyro, accel, hoặc mag đang ở trạng thái calibration (không phải IDLE)
     */
    bool calibrationActive() const
    {
      return state.accel.calibrationState != CALIBRATION_IDLE || state.gyro.calibrationState != CALIBRATION_IDLE || state.mag.calibrationState != CALIBRATION_IDLE;
    }

    /**
     * @brief Bắt đầu quá trình hiệu chỉnh gyro (và accel nếu có)
     *
     * Đặt `calibrationState = CALIBRATION_START` cho gyro và accel.
     * Quá trình hiệu chỉnh diễn ra trong `GyroSensor::calibrate()` qua nhiều chu kỳ.
     */
    void calibrateGyro()
    {
      state.gyro.calibrationState = CALIBRATION_START;
      if(accelActive())
      {
        state.accel.calibrationState = CALIBRATION_START;
      }
    }

    void calibrateMag()
    {
      state.mag.calibrationState = CALIBRATION_START;
    }

    void finishCalibration()
    {
      if(state.gyro.calibrationState == CALIBRATION_SAVE)
      {
        //save();
        state.buzzer.push(BUZZER_GYRO_CALIBRATED);
        logger.info().log(F("GYRO BIAS")).log(Utils::toDeg(state.gyro.bias.x)).log(Utils::toDeg(state.gyro.bias.y)).logln(Utils::toDeg(state.gyro.bias.z));
      }
      if(state.accel.calibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log(F("ACCEL BIAS")).log(state.accel.bias.x).log(state.accel.bias.y).logln(state.accel.bias.z);
      }
      if(state.mag.calibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log(F("MAG BIAS")).log(state.mag.calibrationOffset.x).log(state.mag.calibrationOffset.y).logln(state.mag.calibrationOffset.z);
        logger.info().log(F("MAG SCALE")).log(state.mag.calibrationScale.x).log(state.mag.calibrationScale.y).logln(state.mag.calibrationScale.z);
      }
    }

    /**
     * @brief Kiểm tra arming có bị chặn bởi bất kỳ flag nào không
     *
     * @return true nếu `armingDisabledFlags != 0` (không thể arm)
     * @warning Nếu build với `-DESPFC_DEV_PRESET_UNSAFE_ARMING`, luôn trả về false —
     *          CHỈ dùng cho dev/debug, KHÔNG dùng trong firmware thực tế.
     * @note Có thể được gọi trong gyroTask (IRAM context)
     */
    bool armingDisabled() const /* IRAM_ATTR */
    {
#if defined(ESPFC_DEV_PRESET_UNSAFE_ARMING)
      return false;
#warning "Danger macro used ESPFC_DEV_PRESET_UNSAFE_ARMING"
#else
      return state.mode.armingDisabledFlags != 0;
#endif
    }

    /**
     * @brief Bật hoặc tắt một arming disabled flag
     *
     * @param flag Flag cần thay đổi (ví dụ: ARMING_DISABLED_THROTTLE, ARMING_DISABLED_NO_GYRO)
     * @param value true để bật flag (chặn arming), false để tắt
     */
    void setArmingDisabled(ArmingDisabledFlags flag, bool value)
    {
      if(value) state.mode.armingDisabledFlags |= flag;
      else state.mode.armingDisabledFlags &= ~flag;
    }

    /**
     * @brief Kiểm tra một arming disabled flag cụ thể có đang bật không
     *
     * @param flag Flag cần kiểm tra
     * @return true nếu flag đang bật (arming bị chặn bởi lý do này)
     */
    bool getArmingDisabled(ArmingDisabledFlags flag)
    {
      return state.mode.armingDisabledFlags & flag;
    }

    /**
     * @brief Thiết lập trạng thái output saturation cho tất cả PID
     *
     * Khi output bị bão hòa (motor đã full hoặc zero), thông báo cho
     * tất cả inner/outer PID để giới hạn I-term tích lũy (anti-windup).
     *
     * @param val true nếu output đang bão hòa
     */
    void setOutputSaturated(bool val)
    {
      state.output.saturated = val;
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        state.innerPid[i].outputSaturated = val;
        state.outerPid[i].outputSaturated = val;
      }
    }

    /**
     * @brief Kiểm tra có motor nào đang quay không (dựa trên output disarmed)
     *
     * Dùng để phát hiện trường hợp motor đang chạy khi chưa arm (safety check).
     *
     * @return true nếu bất kỳ motor nào có output khác `minCommand`
     */
    bool areMotorsRunning() const
    {
      size_t count = state.currentMixer.count;
      for(size_t i = 0; i < count; i++)
      {
        if(config.output.channel[i].servo) continue;
        if(state.output.disarmed[i] != config.output.minCommand) return true;
        //if(state.output.us[i] != config.output.minCommand) return true;
      }
      return false;
    }

    /**
     * @brief Ghi giá trị debug nếu debug mode khớp
     *
     * @param mode Debug mode cần khớp với `config.debug.mode`
     * @param index Chỉ số slot debug (0–7)
     * @param value Giá trị cần ghi
     * @note Không làm gì nếu mode không khớp — an toàn để gọi thường xuyên
     */
    void inline setDebug(DebugMode mode, size_t index, int16_t value)
    {
      if(index >= 8) return;
      if(config.debug.mode != mode) return;
      state.debug[index] = value;
    }

    /**
     * @brief Lưu vị trí GPS hiện tại làm điểm home (dùng cho RTH)
     *
     * Chỉ lưu khi GPS có fix và đủ số vệ tinh (`config.gps.minSats`).
     * Nếu `config.gps.setHomeOnce = true`, chỉ lưu lần đầu tiên.
     *
     * @param force true để ghi đè điều kiện fix/sats (dùng khi arming thủ công)
     */
    void setGpsHome(bool force = false)
    {
      if(force || (state.gps.fix && state.gps.numSats >= config.gps.minSats))
      {
        if(!state.gps.homeSet || !config.gps.setHomeOnce)
        {
          state.gps.location.home = state.gps.location.raw;
          state.gps.homeSet = true;
        }
      }
    }

    Device::SerialDevice * getSerialStream(SerialPort i)
    {
      return state.serial[i].stream;
    }

    Device::SerialDevice * getSerialStream(SerialFunction sf)
    {
      for(size_t i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if(config.serial[i].functionMask & sf) return state.serial[i].stream;
      }
      return nullptr;
    }

    int getSerialIndex(SerialPortId id)
    {
      switch(id)
      {
#ifdef ESPFC_SERIAL_0
        case SERIAL_ID_UART_1: return SERIAL_UART_0;
#endif
#ifdef ESPFC_SERIAL_1
        case SERIAL_ID_UART_2: return SERIAL_UART_1;
#endif
#ifdef ESPFC_SERIAL_2
        case SERIAL_ID_UART_3: return SERIAL_UART_2;
#endif
#ifdef ESPFC_SERIAL_USB
        case SERIAL_ID_USB_VCP: return SERIAL_USB;
#endif
#ifdef ESPFC_SERIAL_SOFT_0
        case SERIAL_ID_SOFTSERIAL_1: return SERIAL_SOFT_0;
#endif
        default: break;
      }
      return -1;
    }

    /**
     * @brief Tính giá trị RSSI từ kênh RC được cấu hình
     *
     * Ánh xạ giá trị RC [-1.0, 1.0] sang RSSI [0, 1023].
     *
     * @return Giá trị RSSI [0, 1023], hoặc 0 nếu kênh không hợp lệ
     */
    uint16_t getRssi() const
    {
      size_t channel = config.input.rssiChannel;
      if(channel < 4 || channel > state.input.channelCount) return 0;
      float value = state.input.ch[channel - 1];
      return Utils::clamp(lrintf(Utils::map(value, -1.0f, 1.0f, 0.0f, 1023.0f)), 0l, 1023l);
    }

    /**
     * @brief Tải cấu hình từ EEPROM và thực hiện post-processing
     *
     * Khởi tạo logger, đọc `ModelConfig` từ EEPROM qua `Utils::Storage`,
     * rồi gọi `postLoad()` để tính các giá trị derived.
     *
     * @return 1 nếu thành công
     * @note Phải gọi trước `begin()`. Trên `UNIT_TEST`: bỏ qua EEPROM.
     */
    int load()
    {
      logger.begin();
      #ifndef UNIT_TEST
      _storage.begin();
      logger.info().log(F("F_CPU")).logln(F_CPU);
      _storageResult = _storage.load(config);
      logStorageResult();
      #endif
      postLoad();
      return 1;
    }

    /**
     * @brief Lưu cấu hình hiện tại vào EEPROM
     *
     * Gọi `preSave()` để chuẩn hóa trước khi ghi, sau đó ghi `config` qua Storage.
     * @note Trên `UNIT_TEST`: bỏ qua thao tác ghi thực tế.
     */
    void save()
    {
      preSave();
      #ifndef UNIT_TEST
      _storageResult = _storage.save(config);
      logStorageResult();
      #endif
    }

    /**
     * @brief Tải lại và áp dụng cấu hình (gọi `begin()` để reinit subsystems)
     *
     * Dùng sau khi cấu hình thay đổi qua MSP/CLI để áp dụng ngay.
     */
    void reload()
    {
      begin();
    }

    /**
     * @brief Reset cấu hình về mặc định và khởi động lại
     *
     * Gọi `initialize()` (reset config) rồi `reload()` để áp dụng.
     */
    void reset()
    {
      initialize();
      //save();
      reload();
    }

    /**
     * @brief Kiểm tra và điều chỉnh cấu hình về giá trị hợp lệ
     *
     * Thực hiện sau `load()` và khi có thay đổi cấu hình:
     *   - Điều chỉnh gyro rate theo bus (SPI nhanh hơn I2C)
     *   - Tính loopRate = gyroRate / loopSync
     *   - Điều chỉnh ESC protocol (DSHOT → sync, async tương thích, v.v.)
     *   - Thiết lập loopSync = 1 nếu là lần đầu tiên (EEPROM trống)
     */
    void sanitize()
    {
      // for spi gyro allow full speed mode
      if (state.gyro.dev && state.gyro.dev->getBus()->isSPI())
      {
        state.gyro.rate = Utils::alignToClock(state.gyro.clock, ESPFC_GYRO_SPI_RATE_MAX);
      }
      else
      {
        state.gyro.rate = Utils::alignToClock(state.gyro.clock, ESPFC_GYRO_I2C_RATE_MAX);
        // first usage
        if(_storageResult == STORAGE_ERR_BAD_MAGIC || _storageResult == STORAGE_ERR_BAD_SIZE || _storageResult == STORAGE_ERR_BAD_VERSION)
        {
          config.loopSync = 1;
        }
      }

      int loopSyncMax = 1;
      //if(config.mag.dev != MAG_NONE || config.baro.dev != BARO_NONE) loopSyncMax /= 2;

      config.loopSync = std::max((int)config.loopSync, loopSyncMax);
      state.loopRate = state.gyro.rate / config.loopSync;

      config.output.protocol = ESC_PROTOCOL_SANITIZE(config.output.protocol);

      switch(config.output.protocol)
      {
        case ESC_PROTOCOL_BRUSHED:
          config.output.async = true;
          break;
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_PROSHOT:
          config.output.async = false;
          break;
      }

      if(config.output.async)
      {
        // for async limit pwm rate
        switch(config.output.protocol)
        {
          case ESC_PROTOCOL_PWM:
            config.output.rate = constrain(config.output.rate, 50, 480);
            break;
          case ESC_PROTOCOL_ONESHOT125:
            config.output.rate = constrain(config.output.rate, 50, 2000);
            break;
          case ESC_PROTOCOL_ONESHOT42:
            config.output.rate = constrain(config.output.rate, 50, 4000);
            break;
          case ESC_PROTOCOL_BRUSHED:
          case ESC_PROTOCOL_MULTISHOT:
            config.output.rate = constrain(config.output.rate, 50, 8000);
            break;
          default:
            config.output.rate = constrain(config.output.rate, 50, 2000);
            break;
        }
      }
      else
      {
        // for synced and standard PWM limit loop rate and pwm pulse width
        if(config.output.protocol == ESC_PROTOCOL_PWM && state.loopRate > 500)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((state.gyro.rate + 499) / 500)); // align loop rate to lower than 500Hz
          state.loopRate = state.gyro.rate / config.loopSync;
          if(state.loopRate > 480 && config.output.maxThrottle > 1940)
          {
            config.output.maxThrottle = 1940;
          }
        }
        // for onshot125 limit loop rate to 2kHz
        if(config.output.protocol == ESC_PROTOCOL_ONESHOT125 && state.loopRate > 2000)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((state.gyro.rate + 1999) / 2000)); // align loop rate to lower than 2000Hz
          state.loopRate = state.gyro.rate / config.loopSync;
        }
      }

      // sanitize throttle and motor limits
      if(config.output.throttleLimitType < 0 || config.output.throttleLimitType >= THROTTLE_LIMIT_TYPE_MAX) {
        config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_NONE;
      }

      if(config.output.throttleLimitPercent < 1 || config.output.throttleLimitPercent > 100) {
        config.output.throttleLimitPercent = 100;
      }

      if(config.output.motorLimit < 1 || config.output.motorLimit > 100) {
        config.output.motorLimit = 100;
      }

      // configure serial ports
      constexpr uint32_t serialFunctionAllowedMask = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_RX_SERIAL | SERIAL_FUNCTION_BLACKBOX | 
        SERIAL_FUNCTION_GPS | SERIAL_FUNCTION_TELEMETRY_FRSKY | SERIAL_FUNCTION_TELEMETRY_HOTT | SERIAL_FUNCTION_TELEMETRY_IBUS | SERIAL_FUNCTION_VTX_SMARTAUDIO;
      uint32_t featureAllowMask =  FEATURE_RX_PPM | FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP | FEATURE_SOFTSERIAL | FEATURE_GPS |
        FEATURE_TELEMETRY | FEATURE_RX_SPI;// | FEATURE_AIRMODE;

      // allow dynamic filter only above 1k sampling rate
      if(state.loopRate >= DynamicFilterConfig::MIN_FREQ)
      {
        featureAllowMask |= FEATURE_DYNAMIC_FILTER;
      }

      config.featureMask &= featureAllowMask;

      for(int i = 0; i < SERIAL_UART_COUNT; i++) {
        config.serial[i].functionMask &= serialFunctionAllowedMask;
      }

      // only few beeper modes allowed
      config.buzzer.beeperMask &=
        1 << (BUZZER_GYRO_CALIBRATED - 1) |
        1 << (BUZZER_SYSTEM_INIT - 1) |
        1 << (BUZZER_RX_LOST - 1) |
        1 << (BUZZER_RX_SET - 1) |
        1 << (BUZZER_DISARMING - 1) |
        1 << (BUZZER_ARMING - 1) |
        1 << (BUZZER_BAT_LOW - 1);

        if(config.gyro.dynamicFilter.count > DYN_NOTCH_COUNT_MAX)
        {
          config.gyro.dynamicFilter.count = DYN_NOTCH_COUNT_MAX;
        }
    }

    /**
     * @brief Khởi tạo toàn bộ timer và filter từ cấu hình đã sanitize
     *
     * Gọi `sanitize()` trước, sau đó:
     *   - Khởi tạo tất cả `Utils::Timer` (gyro, accel, loop, mixer, input, mag, ...)
     *   - Khởi tạo tất cả `Utils::Filter` (gyro LPF, notch, dynamic notch, RPM filter, ...)
     *   - Thiết lập board alignment quaternion
     *
     * Phải được gọi sau `Hardware::begin()` (cần `state.gyro.dev` để xác định gyro rate).
     * Có thể gọi lại bằng `reload()` khi cấu hình thay đổi.
     */
    void begin()
    {
      sanitize();

      // init timers
      // sample rate = clock / ( divider + 1)
      state.gyro.timer.setRate(state.gyro.rate);
      int accelRate = Utils::alignToClock(state.gyro.timer.rate, 500);
      state.accel.timer.setRate(state.gyro.timer.rate, state.gyro.timer.rate / accelRate);
      state.loopTimer.setRate(state.gyro.timer.rate, config.loopSync);
      state.mixer.timer.setRate(state.loopTimer.rate, config.mixerSync);
      int inputRate = Utils::alignToClock(state.gyro.timer.rate, 1000);
      state.input.timer.setRate(state.gyro.timer.rate, state.gyro.timer.rate / inputRate);
      state.actuatorTimer.setRate(50);
      state.gyro.dynamicFilterTimer.setRate(50);
      state.telemetryTimer.setInterval(config.telemetryInterval * 1000);
      state.stats.timer.setRate(3);
      if(magActive())
      {
        state.mag.timer.setRate(state.mag.rate);
      }

      state.boardAlignment.init(VectorFloat(Utils::toRad(config.boardAlignment[0]), Utils::toRad(config.boardAlignment[1]), Utils::toRad(config.boardAlignment[2])));

      const uint32_t gyroPreFilterRate = state.gyro.timer.rate;
      const uint32_t gyroFilterRate = state.loopTimer.rate;
      const uint32_t inputFilterRate = state.input.timer.rate;

      // configure filters
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        if(isFeatureActive(FEATURE_DYNAMIC_FILTER))
        {
          for(size_t p = 0; p < (size_t)config.gyro.dynamicFilter.count; p++)
          {
            state.gyro.dynNotchFilter[p][i].begin(FilterConfig(FILTER_NOTCH_DF1, 400, 380), gyroFilterRate);
          }
        }
        state.gyro.notch1Filter[i].begin(config.gyro.notch1Filter, gyroFilterRate);
        state.gyro.notch2Filter[i].begin(config.gyro.notch2Filter, gyroFilterRate);
        if(config.gyro.dynLpfFilter.cutoff > 0)
        {
          state.gyro.filter[i].begin(FilterConfig((FilterType)config.gyro.filter.type, config.gyro.dynLpfFilter.cutoff), gyroFilterRate);
        }
        else
        {
          state.gyro.filter[i].begin(config.gyro.filter, gyroFilterRate);
        }
        state.gyro.filter2[i].begin(config.gyro.filter2, gyroFilterRate);
        state.gyro.filter3[i].begin(config.gyro.filter3, gyroPreFilterRate);
        state.attitude.filter[i].begin(FilterConfig(FILTER_PT1, state.accel.timer.rate / 3), gyroFilterRate);
        for(size_t m = 0; m < RPM_FILTER_MOTOR_MAX; m++)
        {
          state.gyro.rpmFreqFilter[m].begin(FilterConfig(FILTER_PT1, config.gyro.rpmFilter.freqLpf), gyroFilterRate);
          for(size_t n = 0; n < config.gyro.rpmFilter.harmonics; n++)
          {
            int center = Utils::mapi(m * RPM_FILTER_HARMONICS_MAX + n, 0, RPM_FILTER_MOTOR_MAX * config.gyro.rpmFilter.harmonics, config.gyro.rpmFilter.minFreq, gyroFilterRate / 2);
            state.gyro.rpmFilter[m][n][i].begin(FilterConfig(FILTER_NOTCH_DF1, center, center * 0.98f), gyroFilterRate);
          }
        }
        if(magActive())
        {
          state.mag.filter[i].begin(config.mag.filter, state.mag.timer.rate);
        }
      }

      for(size_t i = 0; i < 4; i++)
      {
        if (config.input.filterType == INPUT_FILTER)
        {
          state.input.filter[i].begin(config.input.filter, inputFilterRate);
        }
        else
        {
          state.input.filter[i].begin(FilterConfig(FILTER_PT3, 25), inputFilterRate);
        }
      }

      // ensure disarmed pulses
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.output.disarmed[i] = config.output.channel[i].servo ? config.output.channel[i].neutral : config.output.minCommand; // ROBOT
      }

      state.buzzer.beeperMask = config.buzzer.beeperMask;

      state.customMixer = MixerConfig(config.customMixerCount, config.customMixes);

      // override temporary
      //state.telemetryTimer.setRate(100);
    }

    void postLoad()
    {
      // load current sensor calibration
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        state.gyro.bias.set(i, config.gyro.bias[i] / 1000.0f);
        state.accel.bias.set(i, config.accel.bias[i] / 1000.0f);
        state.mag.calibrationOffset.set(i, config.mag.offset[i] / 10.0f);
        state.mag.calibrationScale.set(i, config.mag.scale[i] / 1000.0f);
      }
    }

    void preSave()
    {
      // store current sensor calibration
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        config.gyro.bias[i] = lrintf(state.gyro.bias[i] * 1000.0f);
        config.accel.bias[i] = lrintf(state.accel.bias[i] * 1000.0f);
        config.mag.offset[i] = lrintf(state.mag.calibrationOffset[i] * 10.0f);
        config.mag.scale[i] = lrintf(state.mag.calibrationScale[i] * 1000.0f);
      }
    }

    ModelState state;
    ModelConfig config;
    Utils::Logger logger;

    void logStorageResult()
    {
#ifndef UNIT_TEST
      switch(_storageResult)
      {
        case STORAGE_LOAD_SUCCESS:    logger.info().logln(F("EEPROM load ok")); break;
        case STORAGE_SAVE_SUCCESS:    logger.info().logln(F("EEPROM save ok")); break;
        case STORAGE_SAVE_ERROR:      logger.err().logln(F("EEPROM save failed")); break;
        case STORAGE_ERR_BAD_MAGIC:   logger.err().logln(F("EEPROM wrong magic")); break;
        case STORAGE_ERR_BAD_VERSION: logger.err().logln(F("EEPROM wrong version")); break;
        case STORAGE_ERR_BAD_SIZE:    logger.err().logln(F("EEPROM wrong size")); break;
        case STORAGE_NONE:
        default:
          logger.err().logln(F("EEPROM unknown result")); break;
      }
#endif
    }

  private:
    #ifndef UNIT_TEST
    Utils::Storage _storage;
    #endif
    StorageResult _storageResult;
};

}

#endif
