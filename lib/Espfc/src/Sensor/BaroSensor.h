#ifndef _ESPFC_SENSOR_BARO_SENSOR_H_
#define _ESPFC_SENSOR_BARO_SENSOR_H_

#include "BaseSensor.h"
#include "Model.h"
#include "Device/BaroDevice.h"
#include "Utils/Filter.h"

namespace Espfc {

namespace Sensor {

/**
 * @brief Quản lý đọc và xử lý dữ liệu barometer (cảm biến áp suất khí quyển)
 *
 * Hỗ trợ các cảm biến như BMP280, MS5611, DPS310 kết nối qua I2C/SPI.
 * Pipeline xử lý dùng state machine để xen kẽ đọc nhiệt độ và áp suất
 * (nhiều sensor yêu cầu đặt lệnh → đợi → đọc, không thể đọc liên tục):
 *
 *   BARO_STATE_INIT → BARO_STATE_TEMP_GET → BARO_STATE_PRESS_GET → lặp lại
 *
 * Dữ liệu sau lọc được lưu vào:
 *   - `_model.state.baro.altitudeGround` — độ cao so với mặt đất khi khởi động (m)
 *   - `_model.state.baro.vario` — tốc độ thay đổi độ cao (m/s)
 *
 * Cả hai được dùng làm đầu vào cho `Control::Altitude` (complementary filter).
 *
 * @note Barometer thường chạy ở 25–50Hz, chậm hơn nhiều so với gyro.
 *       Median filter được dùng để loại bỏ spike trước LPF.
 */
class BaroSensor: public BaseSensor
{
  public:
    /**
     * @brief Trạng thái state machine đọc barometer
     *
     * Vì nhiều sensor baro cần thời gian chuyển đổi, đọc được chia thành
     * hai bước riêng biệt với `_wait` delay giữa các bước.
     */
    enum BaroState
    {
      BARO_STATE_INIT,       ///< Đang khởi tạo — đặt lệnh đo nhiệt độ lần đầu
      BARO_STATE_TEMP_GET,   ///< Chờ và đọc kết quả nhiệt độ
      BARO_STATE_PRESS_GET,  ///< Chờ và đọc kết quả áp suất, tính độ cao
    };

    /**
     * @brief Khởi tạo BaroSensor với tham chiếu đến model
     *
     * @param model Tham chiếu đến shared state model
     */
    BaroSensor(Model& model);

    /**
     * @brief Khởi tạo baro device và cấu hình filter
     *
     * Phát hiện và khởi tạo BaroDevice, khởi tạo các filter:
     * median filter (loại spike) và LPF (làm mượt) cho nhiệt độ, áp suất,
     * độ cao, và vario.
     *
     * @return 1 nếu thành công, 0 nếu không tìm thấy baro
     */
    int begin();

    /**
     * @brief Cập nhật state machine đọc baro
     *
     * Kiểm tra `_wait` timer và chuyển giữa các state TEMP_GET/PRESS_GET.
     * Không blocking — trả về ngay nếu chưa đến thời điểm đọc.
     *
     * @return 1 nếu có dữ liệu mới được xử lý
     */
    int update();

    /**
     * @brief Đọc dữ liệu từ baro device theo state hiện tại
     *
     * @return 1 nếu đọc thành công
     */
    int read();

  private:
    /**
     * @brief Đọc và lọc giá trị nhiệt độ, sau đó đặt lệnh đo áp suất
     */
    void readTemperature();

    /**
     * @brief Đọc và lọc giá trị áp suất, tính độ cao, sau đó đặt lệnh đo nhiệt độ
     */
    void readPressure();

    /**
     * @brief Tính độ cao (m) và vario (m/s) từ áp suất đã lọc
     *
     * Dùng công thức barometric altitude với temperature compensation.
     * Lưu kết quả vào `_model.state.baro.altitudeGround` và `.vario`.
     */
    void updateAltitude();

    Model& _model;                         ///< Tham chiếu đến shared state
    Device::BaroDevice * _baro;            ///< Driver phần cứng barometer
    BaroState _state;                      ///< Trạng thái state machine hiện tại
    Utils::Filter _temperatureFilter;      ///< LPF cho nhiệt độ
    Utils::Filter _temperatureMedianFilter; ///< Median filter để loại spike nhiệt độ
    Utils::Filter _pressureFilter;         ///< LPF cho áp suất
    Utils::Filter _pressureMedianFilter;   ///< Median filter để loại spike áp suất
    Utils::Filter _altitudeFilter;         ///< LPF cho độ cao (m)
    Utils::Filter _varioFilter;            ///< LPF cho tốc độ thay đổi độ cao (m/s)
    uint32_t _wait;                        ///< Timestamp (µs) cho lần đọc tiếp theo
    int32_t _counter;                      ///< Bộ đếm để tính vario (delta altitude / time)
};

}

}
#endif