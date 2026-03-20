#pragma once
#include <cstdint>
#include <cstddef>

namespace Espfc::Connect
{

/**
 * @brief Loại LED được sử dụng
 */
enum LedType
{
  LED_SIMPLE, ///< LED đơn kết nối trực tiếp với GPIO (digitalWrite)
  LED_STRIP,  ///< LED WS2812 addressable (điều khiển qua I2S DMA, chỉ ESP32)
};

/**
 * @brief Trạng thái flight controller hiển thị qua LED
 *
 * Mỗi trạng thái tương ứng với một pattern nhấp nháy và màu sắc riêng.
 * Xem chi tiết tại `StatusLed::setStatus()`.
 */
enum LedStatus
{
  LED_OFF,          ///< Tắt LED hoàn toàn
  LED_OK,           ///< Disarmed, sẵn sàng arm — xanh dương, nhấp nháy chậm 1Hz
  LED_ERROR,        ///< Arming bị chặn (lỗi cấu hình/cảm biến) — đỏ, 3 nhịp + nghỉ
  LED_ON,           ///< Đang khởi động — xanh trắng, sáng liên tục
  LED_ARMED,        ///< Đã arm, đang bay — xanh lá, sáng liên tục
  LED_ALTHOLD,      ///< Altitude Hold đang hoạt động — cyan, nhấp nháy nhanh 3Hz
  LED_GPS,          ///< GPS Position Hold / RTH đang hoạt động — tím, 2 nhịp + nghỉ
  LED_FAILSAFE,     ///< Failsafe được kích hoạt — đỏ sáng, nhấp nháy rất nhanh 6Hz
  LED_CALIBRATING,  ///< Cảm biến đang hiệu chỉnh — vàng, sáng liên tục
  LED_LOW_BATTERY,  ///< Cảnh báo pin thấp — cam, nhấp nháy rất nhanh 6Hz
};

/**
 * @brief Điều khiển LED trạng thái của flight controller
 *
 * Hỗ trợ hai loại LED:
 *   - **LED_SIMPLE**: LED đơn qua GPIO (tất cả platform)
 *   - **LED_STRIP**: WS2812 addressable RGB qua I2S DMA (chỉ ESP32, khi build với `ESPFC_LED_WS2812`)
 *
 * Pattern nhấp nháy được điều khiển bằng state machine đơn giản trong `update()`,
 * định nghĩa bởi mảng int[] chứa thời gian ON/OFF xen kẽ (ms), kết thúc bằng 0.
 *
 * @note `update()` phải được gọi định kỳ từ vòng lặp chính (không phải gyroTask).
 *       Gọi từ `Espfc::update()` sau khi xử lý cảm biến.
 */
class StatusLed
{

public:
  StatusLed();

  /**
   * @brief Khởi tạo LED và đặt trạng thái ban đầu `LED_ON`
   *
   * @param pin    Số pin GPIO kết nối LED (-1 để disable)
   * @param type   Loại LED (LED_SIMPLE hoặc LED_STRIP)
   * @param invert true nếu logic LED bị đảo (active-low)
   */
  void begin(int8_t pin, uint8_t type, uint8_t invert);

  /**
   * @brief Cập nhật trạng thái nhấp nháy theo pattern hiện tại
   *
   * Phải được gọi định kỳ trong vòng lặp chính. Dùng `millis()` để kiểm tra
   * thời gian — không chặn, không dùng delay.
   */
  void update();

  /**
   * @brief Chuyển sang trạng thái LED mới với pattern và màu tương ứng
   *
   * Nếu `force = false` và trạng thái mới giống trạng thái hiện tại,
   * không làm gì (tránh reset pattern đang chạy).
   *
   * @param newStatus Trạng thái mới cần hiển thị
   * @param force     true để buộc cập nhật dù trạng thái không thay đổi
   */
  void setStatus(LedStatus newStatus, bool force = false);

private:
  /**
   * @brief Ghi giá trị vật lý ra LED (hỗ trợ cả LED_SIMPLE và LED_STRIP)
   *
   * @param val HIGH (1) = bật LED, LOW (0) = tắt LED
   * @note Đối với LED_STRIP WS2812: gửi màu hiện tại (_r, _g, _b) qua I2S DMA
   */
  void _write(uint8_t val);

  int8_t _pin;        ///< Pin GPIO (-1 nếu không được cấu hình)
  uint8_t _type;      ///< Loại LED (LED_SIMPLE hoặc LED_STRIP)
  uint8_t _invert;    ///< Logic đảo (1 = active-low)
  LedStatus _status;  ///< Trạng thái LED hiện tại
  uint32_t _next;     ///< Thời điểm tiếp theo cần cập nhật (ms từ millis())
  bool _state;        ///< Trạng thái vật lý hiện tại (HIGH/LOW)
  size_t _step;       ///< Bước hiện tại trong mảng pattern
  int * _pattern;     ///< Con trỏ đến mảng pattern nhấp nháy hiện tại
  uint8_t _r, _g, _b; ///< Màu hoạt động cho WS2812 (lưu dạng RGB, gửi dạng GRB)
};

}
