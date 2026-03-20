#pragma once

#include "Model.h"
#include "EscDriver.h"

namespace Espfc {

namespace Output {

/**
 * @brief Motor mixing và xuất lệnh ESC/servo
 *
 * Nhận PID output từ `Controller` (thông qua `_model.state.output.ch[]`) và throttle,
 * pha trộn theo bảng mixer (`MixerConfig`) để tạo ra lệnh cho từng motor/servo,
 * sau đó xuất qua `EscDriver` (PWM/OneShot/DSHOT tuỳ cấu hình).
 *
 * Ngoài ra đọc telemetry DSHOT (eRPM) để cung cấp dữ liệu cho RPM filter.
 *
 * @note `update()` chạy trong gyroTask (IRAM) trên dual-core,
 *       hoặc trong `loop()` sau `controller.update()` trên single-core.
 */
class Mixer
{
  public:
    /**
     * @brief Khởi tạo Mixer với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    Mixer(Model& model);

    /**
     * @brief Khởi tạo EscDriver cho motor và servo
     *
     * Cấu hình EscDriver với protocol (PWM/DSHOT/OneShot), pin assignment,
     * và tần số xuất từ `ModelConfig`. Hỗ trợ motor và servo riêng biệt.
     *
     * @return 1 nếu thành công
     */
    int begin();

    /**
     * @brief Thực hiện một chu kỳ mixing và xuất lệnh ESC
     *
     * Thứ tự: `updateMixer()` → `writeOutput()` → `readTelemetry()`.
     * Khi disarmed: xuất `minCommand` cho tất cả motor.
     * Khi armed: pha trộn throttle + PID output theo bảng mixer.
     *
     * @return 1 nếu thành công
     */
    int update();

    /**
     * @brief Tính lệnh cho từng motor theo bảng mixer
     *
     * Pha trộn throttle và các trục PID (roll, pitch, yaw) theo trọng số
     * trong `MixerConfig`. Áp dụng giới hạn output và clipping.
     *
     * @param mixer  Cấu hình mixer hiện tại (số motor, trọng số)
     * @param outputs Mảng output đã tính [0..motorCount-1], range [-1.0, 1.0]
     */
    void updateMixer(const MixerConfig& mixer, float * outputs);

    /**
     * @brief Giới hạn throttle theo kiểu và mức giới hạn cấu hình
     *
     * Hỗ trợ: SCALE (co giãn toàn bộ), CLIP (cắt cứng), hoặc NONE.
     *
     * @param thrust  Giá trị throttle đầu vào [-1.0, 1.0]
     * @param type    Kiểu giới hạn (ThrottleLimitType)
     * @param limit   Giới hạn phần trăm (0–100)
     * @return Giá trị throttle đã giới hạn
     */
    float limitThrust(float thrust, ThrottleLimitType type, int8_t limit);

    /**
     * @brief Ánh xạ và giới hạn output của một kênh về dải µs hợp lệ
     *
     * Chuyển đổi giá trị float [-1.0, 1.0] sang microsecond [min, max]
     * theo cấu hình từng kênh output.
     *
     * @param output  Giá trị output đã pha trộn [-1.0, 1.0]
     * @param occ     Cấu hình kênh output (min, max, reverse, servo/motor)
     * @param limit   Giá trị giới hạn tối đa (µs)
     * @return Giá trị đã chuyển đổi (µs)
     */
    float limitOutput(float output, const OutputChannelConfig& occ, int limit);

    /**
     * @brief Ghi lệnh đã tính ra EscDriver (motor và servo)
     *
     * @param mixer Cấu hình mixer hiện tại
     * @param out   Mảng giá trị µs đã tính cho từng kênh
     */
    void writeOutput(const MixerConfig& mixer, float * out);

    /**
     * @brief Đọc dữ liệu telemetry DSHOT (eRPM) từ ESC
     *
     * Đọc eRPM từ tất cả motor qua DSHOT bidirectional, lưu vào
     * `_model.state.output.erpm[]` để RPM filter sử dụng.
     */
    void readTelemetry();

    /**
     * @brief Chuyển đổi eRPM sang Hz
     *
     * @param erpm Giá trị electrical RPM từ DSHOT telemetry
     * @return Tần số Hz (eRPM / 60 / số cực)
     */
    float inline erpmToHz(float erpm);

    /**
     * @brief Chuyển đổi eRPM sang RPM cơ học
     *
     * @param erpm Giá trị electrical RPM từ DSHOT telemetry
     * @return RPM cơ học (eRPM / số cực / 2)
     */
    float inline erpmToRpm(float erpm);

    /**
     * @brief Kiểm tra điều kiện dừng motor (disarm hoặc failsafe)
     *
     * @return true nếu cần xuất minCommand cho tất cả motor
     */
    bool inline _stop(void);

  private:
    Model& _model;         ///< Tham chiếu đến shared state
    EscDriver * _motor;    ///< Con trỏ đến ESC driver dùng cho motor
    EscDriver * _servo;    ///< Con trỏ đến ESC driver dùng cho servo

    EscDriver escMotor;            ///< ESC driver instance cho motor
    EscDriver escServo;            ///< ESC driver instance cho servo
    uint32_t _statsCounter;        ///< Bộ đếm chu kỳ dùng để tính thống kê
    uint32_t _statsCounterMax;     ///< Số chu kỳ tối đa trong một window thống kê
    float _erpmToHz;               ///< Hệ số chuyển đổi eRPM → Hz (tính từ số cực motor)
};

}

}
