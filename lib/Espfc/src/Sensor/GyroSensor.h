#pragma once

#include "BaseSensor.h"
#include "Model.h"
#include "Device/GyroDevice.h"
#include "Utils/Sma.hpp"
#ifdef ESPFC_DSP
#include "Utils/FFTAnalyzer.hpp"
#else
#include "Utils/FreqAnalyzer.hpp"
#endif

namespace Espfc {

namespace Sensor {

/**
 * @brief Quản lý đọc và xử lý dữ liệu gyroscope
 *
 * Thực hiện toàn bộ pipeline gyro:
 *   1. `read()` — đọc raw ADC từ GyroDevice, áp dụng bias, scale, alignment
 *   2. `filter()` — lọc LPF/Notch/Dynamic Notch/RPM filter
 *   3. `postLoop()` — cập nhật RPM filter và dynamic notch sau mỗi vòng lặp
 *   4. `calibrate()` — tính bias trong N mẫu đầu tiên
 *
 * `read()` được gọi trong gyroTask (core 1, 4kHz, IRAM) trên dual-core ESP32.
 * `dynNotchFilterUpdate()` và `rpmFilterUpdate()` chạy ở tần số thấp hơn
 * (được chia tần bởi `_dyn_notch_denom`).
 *
 * Kết quả sau lọc được lưu vào `_model.state.gyro.adc[]`.
 *
 * @note Trên ESP32 với `ESPFC_DSP`, dùng FFTAnalyzer 128-point thay cho FreqAnalyzer.
 */
class GyroSensor: public BaseSensor
{
  public:
    /**
     * @brief Khởi tạo GyroSensor với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    GyroSensor(Model& model);
    ~GyroSensor();

    /**
     * @brief Khởi tạo gyro device và cấu hình filter
     *
     * Phát hiện và khởi tạo GyroDevice, cấu hình LPF, notch filter,
     * dynamic notch filter, và RPM filter theo `ModelConfig`.
     *
     * @return 1 nếu thành công, 0 nếu không tìm thấy gyro
     */
    int begin();

    /**
     * @brief Đọc dữ liệu raw từ gyro hardware
     *
     * Đọc ADC, trừ bias, áp dụng scale (rad/s), căn chỉnh trục theo `align`.
     * Kết quả lưu vào `_model.state.gyro.raw[]`.
     *
     * @return 1 nếu đọc thành công, 0 nếu chưa sẵn sàng
     * @note Gọi trong gyroTask (4kHz). Tránh thao tác blocking trong hàm này.
     */
    int read();

    /**
     * @brief Áp dụng chuỗi filter lên dữ liệu gyro raw
     *
     * Thứ tự: LPF1 → Dynamic Notch → RPM Notch → LPF2 → Notch1 → Notch2
     * Kết quả lưu vào `_model.state.gyro.adc[]`.
     *
     * @return 1 nếu thành công
     * @note Gọi trong gyroTask sau `read()`
     */
    int filter();

    /**
     * @brief Cập nhật analyzer tần số sau vòng lặp gyro
     *
     * Feed dữ liệu gyro đã lọc vào FreqAnalyzer/FFTAnalyzer.
     * Chạy ở tần số thấp hơn gyro (chia tần bởi `_dyn_notch_denom`).
     */
    void postLoop();

    /**
     * @brief Cập nhật RPM notch filter từ eRPM telemetry DSHOT
     *
     * Tính lại tần số notch cho từng motor × harmonic (tối đa 3 harmonics).
     * Được gọi sau `readTelemetry()` trong Mixer.
     *
     * @note Chỉ hoạt động khi `_rpm_enabled == true` (DSHOT bidirectional)
     */
    void rpmFilterUpdate();

    /**
     * @brief Cập nhật dynamic notch filter từ kết quả phân tích tần số
     *
     * Lấy tần số đỉnh từ FreqAnalyzer/FFTAnalyzer và cập nhật các notch filter.
     * Chạy ở tần số thấp hơn gyro để tiết kiệm CPU.
     */
    void dynNotchFilterUpdate();

  private:
    /**
     * @brief Thu thập mẫu để tính bias gyro (offset khi đứng yên)
     *
     * Chạy trong N chu kỳ đầu tiên (CALIBRATION_START → CALIBRATION_SAVE).
     * Tính trung bình SMA từ `_sma`. Lưu bias vào `_model.state.gyro.bias`.
     */
    void calibrate();

    Utils::Sma<VectorFloat, 8> _sma;           ///< Simple Moving Average để tính bias calibration
    Utils::Sma<VectorFloat, 8> _dyn_notch_sma; ///< SMA để ổn định tần số trước khi feed vào analyzer
    size_t _dyn_notch_denom;    ///< Hệ số chia tần để chạy dynamic notch ở tần số thấp hơn
    size_t _dyn_notch_count;    ///< Bộ đếm chu kỳ cho dyn notch update
    bool _dyn_notch_enabled;    ///< true nếu dynamic notch filter được cấu hình
    bool _dyn_notch_debug;      ///< true nếu ghi debug thông tin tần số notch
    bool _rpm_enabled;          ///< true nếu RPM filter được bật (yêu cầu DSHOT bidirectional)
    size_t _rpm_motor_index;    ///< Index motor đang được cập nhật (vòng lặp tuần tự qua 4 motor)
    float _rpm_weights[3];      ///< Trọng số fade-in cho 3 harmonics của RPM filter
    float _rpm_fade_inv;        ///< Nghịch đảo của dải fade RPM (tính trước để tối ưu)
    float _rpm_min_freq;        ///< Tần số RPM tối thiểu hợp lệ (Hz)
    float _rpm_max_freq;        ///< Tần số RPM tối đa hợp lệ (Hz)
    float _rpm_q;               ///< Q factor của RPM notch filter

    Model& _model;               ///< Tham chiếu đến shared state
    Device::GyroDevice * _gyro;  ///< Driver phần cứng gyro (MPU6000, ICM42688, v.v.)

#ifdef ESPFC_DSP
    Utils::FFTAnalyzer<128> _fft[3];
#else
    Utils::FreqAnalyzer _freqAnalyzer[3];
#endif

};

}

}
