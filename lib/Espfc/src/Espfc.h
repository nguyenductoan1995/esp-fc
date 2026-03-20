#pragma once

#include "Model.h"
#include "Hardware.h"
#include "Control/Controller.h"
#include "Input.h"
#include "Control/Actuator.h"
#include "SensorManager.h"
#include "TelemetryManager.h"
#include "SerialManager.h"
#include "Output/Mixer.h"
#include "Blackbox/Blackbox.h"
#include "Connect/Buzzer.hpp"

namespace Espfc {

/**
 * @brief Lớp điều phối cấp cao nhất của flight controller esp-fc
 *
 * Sở hữu và quản lý vòng đời của tất cả các subsystem: cảm biến, bộ điều khiển,
 * mixer, telemetry, blackbox, v.v. Điểm vào duy nhất từ `main.cpp` / `setup()` / `loop()`.
 *
 * Trên ESP32 dual-core (ESPFC_MULTI_CORE):
 *   - `update()` chạy trên core 1 (gyroTask, 4kHz, IRAM) — đọc cảm biến, PID
 *   - `updateOther()` chạy trên core 0 (pidTask) — telemetry, blackbox, arming
 *
 * Trên single-core (ESP8266, RP2040):
 *   - Cả hai vòng lặp chạy tuần tự trong `loop()`.
 */
class Espfc
{
  public:
    Espfc();

    /**
     * @brief Tải cấu hình từ EEPROM và khởi tạo hàng đợi sự kiện
     *
     * Phải được gọi đầu tiên trước `begin()`. Đọc `ModelConfig` từ flash/EEPROM
     * thông qua `Utils::Storage`.
     *
     * @return 1 nếu thành công
     */
    int load();

    /**
     * @brief Khởi tạo toàn bộ phần cứng và subsystem
     *
     * Gọi `begin()` theo thứ tự phụ thuộc: serial → hardware → model → mixer →
     * sensor → input → actuator → controller → blackbox → buzzer.
     * Phát tín hiệu buzzer `BUZZER_SYSTEM_INIT` khi hoàn tất.
     *
     * @return 1 nếu thành công
     * @note Phải gọi `load()` trước
     */
    int begin();

    /**
     * @brief Vòng lặp chính — đọc cảm biến, tính PID, xuất motor
     *
     * Trên ESPFC_MULTI_CORE: chạy trên gyroTask (core 1, 4kHz).
     * Trên single-core: chạy trên `loop()`, tự kiểm tra timer gyro.
     *
     * @param externalTrigger true nếu được gọi từ hardware timer ISR (ESP32 dual-core),
     *                        false nếu tự kiểm tra timer bên trong
     * @return 1 nếu đã xử lý, 0 nếu chưa đến thời điểm cập nhật
     * @note Đánh dấu FAST_CODE_ATTR — phải ở IRAM trên ESP32
     */
    int update(bool externalTrigger = false);

    /**
     * @brief Vòng lặp phụ — xử lý các tác vụ ít nhạy cảm với thời gian
     *
     * Trên ESPFC_MULTI_CORE: chạy trên pidTask (core 0), nhận sự kiện từ `appQueue`
     * (EVENT_GYRO_READ, EVENT_ACCEL_READ) và gọi controller/fusion/mixer/blackbox.
     * Trên single-core: hàm này không làm gì (toàn bộ đã được xử lý trong `update()`).
     *
     * @return 1 nếu đã xử lý sự kiện, 0 nếu hàng đợi rỗng
     * @note Đánh dấu FAST_CODE_ATTR. Giao tiếp cross-core qua `appQueue`.
     */
    int updateOther();

    /**
     * @brief Lấy chu kỳ gyro tính bằng microsecond
     *
     * @return Khoảng thời gian giữa hai lần đọc gyro (µs), ví dụ: 250 ứng với 4kHz
     */
    int getGyroInterval() const
    {
      return _model.state.gyro.timer.interval;
    }

  private:
    Model _model;                    ///< Trạng thái chia sẻ toàn hệ thống và cấu hình EEPROM
    Hardware _hardware;              ///< Khởi tạo phần cứng: pin, I2C/SPI bus
    Control::Controller _controller; ///< Vòng lặp điều khiển: rates PID, angle PID, altitude hold
    TelemetryManager _telemetry;     ///< Quản lý telemetry CRSF và text output
    Input _input;                    ///< Xử lý RC input (SBUS/IBUS/CRSF/PPM/ESP-NOW)
    Control::Actuator _actuator;     ///< Arming, failsafe, kiểm tra điều kiện bay
    SensorManager _sensor;           ///< Điều phối đọc gyro, accel, mag, baro
    Output::Mixer _mixer;            ///< Motor mixing và xuất lệnh ESC
    Blackbox::Blackbox _blackbox;    ///< Ghi log dữ liệu bay (serial hoặc SPI flash)
    Connect::Buzzer _buzzer;         ///< Điều khiển buzzer thông báo
    SerialManager _serial;           ///< Quản lý MSP và CLI qua UART/USB
    uint32_t _loop_next;             ///< Thời điểm cho phép cập nhật mixer/blackbox tiếp theo (µs)
};

}
