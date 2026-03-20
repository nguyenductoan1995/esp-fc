#pragma once

#include <cstdint>

namespace Espfc {

namespace Utils {

/**
 * @brief Bộ định thời dùng để lập lịch các tác vụ định kỳ (rate-based scheduling)
 *
 * `Timer` theo dõi thời gian thực thi và kiểm tra xem đã đến chu kỳ tiếp theo chưa.
 * Được dùng cho tất cả vòng lặp có tần số khác nhau trong esp-fc:
 *   - `gyro.timer`: 4kHz (hoặc theo gyro rate)
 *   - `loopTimer`: tần số PID (thường = gyro rate / loopSync)
 *   - `accel.timer`, `mag.timer`, `baro.timer`: tần số cảm biến chậm hơn
 *   - `input.timer`: RC input processing
 *
 * Trên ESP32 dual-core, `iteration` được khai báo `volatile` vì được ghi từ
 * gyroTask và đọc từ pidTask (không có mutex, dùng atomic read).
 *
 * Cách sử dụng điển hình:
 * ```cpp
 * timer.setRate(4000);      // 4kHz
 * if(timer.check()) {       // trả về true mỗi 250µs
 *   doWork();
 * }
 * ```
 */
class Timer
{
  public:
    Timer();

    /**
     * @brief Đặt chu kỳ timer bằng microsecond
     *
     * @param interval Chu kỳ (µs), ví dụ: 250 cho 4kHz
     * @return 1 nếu thành công
     */
    int setInterval(uint32_t interval);

    /**
     * @brief Đặt tần số timer (Hz) với hệ số chia tần tùy chọn
     *
     * `interval = 1e6 / (rate / denom)`
     *
     * @param rate  Tần số mục tiêu (Hz)
     * @param denom Hệ số chia (mặc định: 1) — dùng để tạo tần số con
     * @return 1 nếu thành công
     */
    int setRate(uint32_t rate, uint32_t denom = 1u);

    /**
     * @brief Kiểm tra timer dùng `micros()` nội bộ
     *
     * @return true nếu đã đến chu kỳ tiếp theo (không cập nhật state)
     */
    bool check();

    /**
     * @brief Cập nhật `last`, `next`, `iteration`, `delta` sau khi check() = true
     *
     * @return 1 nếu đã cập nhật
     */
    int update();

    /**
     * @brief Kiểm tra timer với timestamp cung cấp từ bên ngoài
     *
     * @param now Thời điểm hiện tại (µs từ micros())
     * @return true nếu đã đến chu kỳ tiếp theo
     */
    bool check(uint32_t now);

    /**
     * @brief Cập nhật timer với timestamp cung cấp từ bên ngoài
     *
     * @param now Thời điểm hiện tại (µs)
     * @return 1 nếu đã cập nhật
     */
    int update(uint32_t now);

    /**
     * @brief Đồng bộ timer này với timer cha theo slot
     *
     * Kiểm tra xem `iteration` của timer cha có chia hết cho `interval/t.interval`
     * và rơi vào đúng `slot` hay không. Dùng để tạo sub-rate từ timer cha mà không
     * cần thêm `micros()` call.
     *
     * Ví dụ: `loopTimer.syncTo(gyroTimer)` — tạo loopTimer chạy mỗi N chu kỳ gyro.
     *
     * @param t    Timer cha để đồng bộ với
     * @param slot Vị trí slot trong chu kỳ (mặc định: 0)
     * @return true nếu đây là chu kỳ của timer này
     */
    bool syncTo(const Timer& t, uint32_t slot = 0u);

    uint32_t interval;           ///< Chu kỳ (µs)
    uint32_t rate;               ///< Tần số (Hz)
    uint32_t denom;              ///< Hệ số chia tần

    uint32_t last;               ///< Timestamp lần thực thi cuối (µs)
    uint32_t next;               ///< Timestamp lần thực thi tiếp theo (µs)
    volatile uint32_t iteration; ///< Số lần đã thực thi (volatile: đọc cross-core an toàn)
    uint32_t delta;              ///< Thời gian thực tế giữa hai lần thực thi gần nhất (µs)
    float intervalf;             ///< Chu kỳ dạng float (giây) — dùng cho tính toán dt trong filter
};

}

}
