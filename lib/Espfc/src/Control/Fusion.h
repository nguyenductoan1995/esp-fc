#pragma once

#include "Model.h"
#include <Madgwick.h>
#include <Mahony.h>

namespace Espfc {

namespace Control {

/**
 * @brief Bộ ước lượng tư thế AHRS (Attitude and Heading Reference System)
 *
 * Kết hợp dữ liệu gyro, accelerometer, và magnetometer (nếu có) để ước lượng
 * góc nghiêng (roll/pitch/yaw) và quaternion của thiết bị.
 *
 * Hỗ trợ nhiều thuật toán fusion có thể chọn qua `config.imu.fusion`:
 *   - Madgwick / Mahony (thư viện ngoài, trong thư mục AHRS/)
 *   - Complementary filter
 *   - Kalman filter
 *   - RTQF (Rate + Tilt Quaternion Filter)
 *
 * `update()` được gọi từ SensorManager khi có dữ liệu accel mới (EVENT_ACCEL_READ),
 * chạy trên pidTask (core 0) trong chế độ dual-core.
 *
 * Kết quả được lưu vào `_model.state.attitude` (euler angles, quaternion, cosTheta).
 */
class Fusion
{
  public:
    /**
     * @brief Khởi tạo Fusion với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    Fusion(Model& model);

    /**
     * @brief Khởi tạo thuật toán fusion và các tham số ban đầu
     *
     * Cài gain ban đầu (Madgwick beta, Mahony Kp/Ki) để hội tụ nhanh lúc khởi động.
     * Sau khi ổn định, `restoreGain()` giảm gain về giá trị runtime.
     *
     * @return 1 nếu thành công
     */
    int begin();

    /**
     * @brief Khôi phục gain về giá trị cấu hình sau khi hội tụ lúc khởi động
     *
     * Được gọi sau khi ước lượng tư thế đã ổn định (thường sau vài giây đầu).
     */
    void restoreGain();

    /**
     * @brief Thực thi một bước fusion — cập nhật tư thế từ cảm biến
     *
     * Gọi thuật toán fusion phù hợp dựa trên `config.imu.fusion`.
     * Cập nhật `_model.state.attitude.euler`, `.quaternion`, và `.cosTheta`.
     *
     * @return 1 nếu thành công
     */
    int update();

    /**
     * @brief Thuật toán fusion thử nghiệm (in development)
     */
    void experimentalFusion();

    /**
     * @brief Fusion đơn giản chỉ dùng accelerometer (không dùng gyro)
     *
     * Tính góc trực tiếp từ gia tốc kế. Chỉ phù hợp với thiết bị tĩnh.
     */
    void simpleFusion();

    /**
     * @brief Fusion dùng Kalman filter
     */
    void kalmanFusion();

    /**
     * @brief Complementary filter kết hợp gyro và accelerometer
     *
     * Công thức: angle = α × (angle + gyro × dt) + (1-α) × accel_angle
     * Nhanh hơn Madgwick, ít tính toán hơn.
     */
    void complementaryFusion();

    /**
     * @brief Phiên bản cũ của complementary filter (giữ để tham khảo)
     */
    void complementaryFusionOld();

    /**
     * @brief RTQF (Rate + Tilt Quaternion Filter)
     *
     * Kết hợp quaternion từ gyro integration với tilt correction từ accel.
     */
    void rtqfFusion();

    /**
     * @brief Cập nhật tư thế trực tiếp từ accel và mag (không dùng gyro)
     *
     * Dùng để khởi tạo quaternion lúc startup hoặc khi cần reference tuyệt đối.
     */
    void updatePoseFromAccelMag();

    /**
     * @brief Fusion dùng linear interpolation (thử nghiệm)
     */
    void lerpFusion();

    /**
     * @brief Fusion dùng thuật toán Madgwick
     *
     * Sử dụng thư viện Madgwick trong AHRS/. Tích hợp gyro + accel (+ mag nếu có).
     * @note Thử nghiệm — dùng `complementaryFusion` hoặc `rtqfFusion` cho production
     */
    void madgwickFusion();

    /**
     * @brief Fusion dùng thuật toán Mahony
     *
     * Sử dụng thư viện Mahony trong AHRS/. PI controller dựa trên accel error.
     * @note Thử nghiệm — dùng `complementaryFusion` hoặc `rtqfFusion` cho production
     */
    void mahonyFusion();

  private:
    Model& _model;    ///< Tham chiếu đến shared state
    bool _first;      ///< Đánh dấu chu kỳ đầu tiên — khởi tạo quaternion từ accel
    Madgwick _madgwick; ///< Instance thuật toán Madgwick
    Mahony _mahony;     ///< Instance thuật toán Mahony
};

}

}
