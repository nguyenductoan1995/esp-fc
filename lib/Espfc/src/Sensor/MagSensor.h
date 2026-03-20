#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Device/MagDevice.h"

namespace Espfc {

namespace Sensor {

/**
 * @brief Quản lý đọc và hiệu chỉnh magnetometer (la bàn điện tử)
 *
 * Hỗ trợ các cảm biến mag ngoài (HMC5883L, QMC5883L, IST8310, v.v.)
 * kết nối qua I2C. Pipeline xử lý:
 *   1. `read()` — đọc raw từ MagDevice, scale sang đơn vị Gauss
 *   2. `filter()` — LPF để lọc nhiễu điện từ motor/ESC
 *   3. `applyCalibration()` — bù offset và scale từ hiệu chỉnh hard/soft iron
 *
 * Hiệu chỉnh mag (hard iron / soft iron):
 *   - Hard iron: offset do từ trường vĩnh cửu trong board/frame
 *   - Soft iron: scale correction do từ trường không đẳng hướng
 *   - Hiệu chỉnh được thực hiện bằng cách xoay thiết bị theo `calibrateGryro()`
 *     từ MSP/CLI và lưu vào EEPROM.
 *
 * Kết quả được dùng trong Fusion để ước lượng heading (yaw tuyệt đối).
 * Mag thường chạy ở 10–80Hz, chậm hơn nhiều so với gyro.
 */
class MagSensor: public BaseSensor
{
  public:
    /**
     * @brief Khởi tạo MagSensor với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    MagSensor(Model& model);

    /**
     * @brief Khởi tạo mag device và cấu hình filter
     *
     * Phát hiện và khởi tạo MagDevice, khởi tạo LPF.
     *
     * @return 1 nếu thành công, 0 nếu không tìm thấy mag
     */
    int begin();

    /**
     * @brief Đọc, lọc và áp dụng hiệu chỉnh cho mag
     *
     * Bao gồm cả kiểm tra chu kỳ và gọi `calibrate()` nếu đang hiệu chỉnh.
     *
     * @return 1 nếu có dữ liệu mới
     */
    int update();

    /**
     * @brief Đọc dữ liệu raw từ mag hardware
     *
     * @return 1 nếu đọc thành công
     */
    int read();

    /**
     * @brief Áp dụng LPF lên dữ liệu mag raw
     *
     * @return 1 nếu thành công
     */
    int filter();

  private:
    /**
     * @brief Quản lý quá trình thu thập mẫu khi đang hiệu chỉnh
     *
     * Gọi `updateCalibration()` trong mỗi chu kỳ để cập nhật min/max.
     * Khi kết thúc, gọi `applyCalibration()` để lưu offset và scale.
     */
    void calibrate();

    /**
     * @brief Đặt lại trạng thái hiệu chỉnh về ban đầu
     *
     * Reset min/max arrays về giá trị cực để bắt đầu thu thập mới.
     */
    void resetCalibration();

    /**
     * @brief Cập nhật min/max từ mẫu mag hiện tại
     *
     * Theo dõi giá trị min/max trên 3 trục để tính offset hard iron.
     */
    void updateCalibration();

    /**
     * @brief Tính và lưu offset/scale từ dữ liệu hiệu chỉnh đã thu thập
     *
     * Hard iron offset = (max + min) / 2
     * Soft iron scale = tỷ lệ giữa khoảng (max - min) trên các trục
     */
    void applyCalibration();

    Model& _model;           ///< Tham chiếu đến shared state
    Device::MagDevice * _mag; ///< Driver phần cứng magnetometer
};

}

}
