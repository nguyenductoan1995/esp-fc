#pragma once

#include "Model.h"
#include "Control/Rates.h"
#include "Control/Altitude.hpp"
#include "Control/GpsNavigation.hpp"

namespace Espfc::Control {

/**
 * @brief Vòng lặp điều khiển bay chính — outer loop và inner loop PID
 *
 * Thực hiện hai tầng điều khiển:
 *   - **Outer loop**: chuyển đổi RC input thành rate/angle setpoint
 *     (angle mode → PID góc, rate mode → tỷ lệ trực tiếp, GPS → position PID)
 *   - **Inner loop**: PID rate cho Roll/Pitch/Yaw và velocity PID cho altitude hold
 *
 * Chạy trong gyroTask (core 1, FAST_CODE_ATTR) trên ESP32 dual-core,
 * hoặc trong `loop()` trên single-core.
 *
 * @note Các hệ số PID được scale từ giá trị integer trong `ModelConfig`
 *       sang float bằng các hằng số PTERM_SCALE, ITERM_SCALE, DTERM_SCALE, FTERM_SCALE.
 */
class Controller
{
  public:
    /**
     * @brief Khởi tạo controller với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    Controller(Model& model);

    /**
     * @brief Khởi tạo tất cả PID filter và hệ số
     *
     * Cấu hình inner loop PID (roll/pitch/yaw), outer loop PID (angle/level),
     * altitude hold PID, và GPS navigation. Phải gọi sau `_model.begin()`.
     *
     * @return 1 nếu thành công
     */
    int begin();

    /**
     * @brief Thực thi một chu kỳ điều khiển đầy đủ (outer + inner loop)
     *
     * Gọi theo thứ tự: `resetIterm()` → `outerLoop()` → `innerLoop()`.
     * Nếu mixer type là GIMBAL, dùng `outerLoopRobot()`/`innerLoopRobot()` thay thế.
     *
     * @return 1 nếu thành công
     * @note Chạy trong gyroTask tại tần số loopTimer (thường bằng gyroRate / loopSync)
     * @note Đánh dấu FAST_CODE_ATTR — nằm trong IRAM trên ESP32
     */
    int update();

    /**
     * @brief Outer loop cho chế độ robot/gimbal
     *
     * Tính angle setpoint từ speed filter kết hợp output và gyro.
     * Chỉ dùng khi `mixer.type == FC_MIXER_GIMBAL`.
     */
    void outerLoopRobot();

    /**
     * @brief Inner loop cho chế độ robot/gimbal
     *
     * Chạy PID góc (pitch + yaw) nếu góc nghiêng trong giới hạn `angleLimit`.
     * Reset I-term và dừng output nếu góc nghiêng vượt quá giới hạn.
     */
    void innerLoopRobot();

    /**
     * @brief Outer loop cho chế độ bay thông thường
     *
     * Tính rate setpoint cho roll/pitch/yaw từ RC input theo mode:
     *   - POSHOLD / GPS_RESCUE: angle setpoint từ GPS navigation PID
     *   - ANGLE mode: angle setpoint từ RC input × angleLimit
     *   - Rate mode: setpoint trực tiếp từ rates profile
     *   - ALTHOLD / GPS: tính thrust setpoint từ `calcualteAltHoldSetpoint()`
     *
     * @note Đánh dấu FAST_CODE_ATTR
     */
    void outerLoop();

    /**
     * @brief Inner loop PID cho roll/pitch/yaw và altitude hold
     *
     * Chạy PID rate cho 3 trục (× TPA factor) và PID velocity cho thrust.
     * Khi không ở ALTHOLD, I-term của thrust PID được giữ theo RC throttle
     * để transition vào/ra altitude hold được mượt.
     *
     * @note Đánh dấu FAST_CODE_ATTR
     */
    void innerLoop();

    /**
     * @brief Tính hệ số giảm PID theo throttle (Throttle PID Attenuation)
     *
     * Giảm D-term gain khi throttle cao để hạn chế nhiễu motor tần số cao.
     *
     * @return Hệ số TPA trong khoảng [1 - tpaScale, 1.0], bằng 1.0 khi throttle < tpaBreakpoint
     */
    inline float getTpaFactor() const;

    /**
     * @brief Reset I-term của tất cả PID khi điều kiện yêu cầu
     *
     * Reset khi: chưa arm, hoặc (throttle thấp VÀ không phải airmode VÀ
     * `iterm.lowThrottleZeroIterm` được bật).
     */
    inline void resetIterm();

    /**
     * @brief Chuyển đổi RC input [-1, 1] thành rate setpoint (rad/s)
     *
     * Áp dụng rates profile (SuperRate/Betaflight/Actual) theo cấu hình.
     * Trục YAW được đảo dấu input trước khi tính.
     *
     * @param axis   Trục cần tính (AXIS_ROLL, AXIS_PITCH, AXIS_YAW)
     * @param input  Giá trị RC đã chuẩn hóa [-1.0, 1.0]
     * @return Rate setpoint tính bằng rad/s
     */
    float calculateSetpointRate(int axis, float input) const;

    /**
     * @brief Tính climb rate setpoint từ throttle stick cho altitude hold
     *
     * Áp dụng deadband ±10% quanh center. Ánh xạ:
     *   - Dưới center: descent tối đa -2.0 m/s
     *   - Trên center: climb tối đa +4.0 m/s
     *
     * @return Tốc độ thay đổi độ cao mục tiêu (m/s), dương = lên cao
     */
    float calcualteAltHoldSetpoint() const;

  private:
    /**
     * @brief Khởi tạo PID altitude hold (velocity loop cho AXIS_THRUST)
     */
    void beginAltHold();

    /**
     * @brief Khởi tạo inner loop PID cho một trục cụ thể
     *
     * @param axis Trục cần khởi tạo (AXIS_ROLL, AXIS_PITCH, AXIS_YAW)
     */
    void beginInnerLoop(size_t axis);

    /**
     * @brief Khởi tạo outer loop PID (angle/level) cho một trục cụ thể
     *
     * @param axis Trục cần khởi tạo (AXIS_ROLL hoặc AXIS_PITCH)
     */
    void beginOuterLoop(size_t axis);

    Model& _model;              ///< Tham chiếu đến shared state
    Rates _rates;               ///< Tính rate setpoint từ RC input theo rates profile
    Utils::Filter _speedFilter; ///< Bộ lọc tốc độ dùng cho chế độ robot/gimbal
    GpsNavigation _gpsNav;      ///< Điều khiển vị trí GPS (position hold, RTH)
};

}
