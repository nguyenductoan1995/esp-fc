#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Device/GyroDevice.h"

namespace Espfc::Sensor {

/**
 * @brief Quản lý đọc và xử lý dữ liệu accelerometer
 *
 * Trên hầu hết phần cứng (MPU6050, ICM42688, v.v.), accelerometer dùng chung
 * chip với gyro nên sử dụng cùng `GyroDevice`. AccelSensor đọc từ thanh ghi
 * accel riêng và áp dụng pipeline:
 *   1. `read()` — đọc raw, trừ bias, scale sang đơn vị g
 *   2. `filter()` — LPF (cắt nhiễu rung động)
 *   3. `calibrate()` — tính bias khi khởi động
 *
 * Dữ liệu accel được dùng trong AHRS Fusion (tilt correction) và Altitude estimator.
 *
 * `update()` thường chạy ở tần số thấp hơn gyro (ví dụ: 1kHz so với 4kHz gyro).
 * Trên dual-core ESP32, kết quả được gửi qua EVENT_ACCEL_READ đến pidTask để chạy Fusion.
 */
class AccelSensor: public BaseSensor
{
  public:
    /**
     * @brief Khởi tạo AccelSensor với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    AccelSensor(Model& model);

    /**
     * @brief Khởi tạo accel device và cấu hình filter
     *
     * Chia sẻ GyroDevice với GyroSensor. Khởi tạo LPF cho 3 trục.
     *
     * @return 1 nếu thành công, 0 nếu không tìm thấy device
     */
    int begin();

    /**
     * @brief Đọc và lọc dữ liệu accel trong một bước (read + filter)
     *
     * @return 1 nếu có dữ liệu mới, 0 nếu chưa đến chu kỳ
     */
    int update();

    /**
     * @brief Đọc dữ liệu raw từ accel hardware
     *
     * Đọc ADC, trừ bias, scale sang đơn vị g, căn chỉnh trục.
     * Kết quả lưu vào `_model.state.accel.raw[]`.
     *
     * @return 1 nếu đọc thành công
     */
    int read();

    /**
     * @brief Áp dụng LPF lên dữ liệu accel raw
     *
     * Lọc nhiễu rung động motor. Kết quả lưu vào `_model.state.accel.adc[]`.
     *
     * @return 1 nếu thành công
     */
    int filter();

  private:
    /**
     * @brief Thu thập mẫu để tính bias accel (offset khi nằm phẳng)
     *
     * Tính giá trị bias cho trục X, Y (lý tưởng = 0g) và Z (lý tưởng = 1g).
     */
    void calibrate();

    Model& _model;                         ///< Tham chiếu đến shared state
    Device::GyroDevice * _gyro;            ///< Driver dùng chung với GyroSensor
    Utils::Filter _filter[AXIS_COUNT_RPY]; ///< LPF cho 3 trục (X, Y, Z)
};

}
