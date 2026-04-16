# ESP-FC Flight Controller

Bộ điều khiển bay nhỏ gọn, tự làm, chi phí thấp, hiệu suất cao dựa trên ESP32 dành cho người đam mê hobbyist.

# Tính Năng

* Hỗ trợ vi điều khiển Espressif (ESP32, ESP32-S3)
* Giao thức ESC (PWM, Oneshot125/42, Multishot, Brushed, Dshot150/300/600 hai chiều)
* Hỗ trợ bộ thu PPM, SBUS, IBUS và CRSF
* Bộ thu ESP-NOW tích hợp và cấu hình qua WiFi [đọc thêm...](/docs/wireless.md)
* Hỗ trợ module gyro SPI và I2C (MPU6050, MPU9250, ICM20602, BMI160)
* Chế độ bay (ACRO, ANGLE, AIRMODE)
* Khung máy (Quad X)
* Tương thích công cụ cấu hình Betaflight (v10.10)
* Bộ lọc Gyro có thể cấu hình (LPF, Dynamic Notches, dTerm, RPM)
* Ghi Blackbox (OpenLog/OpenLager/Flash)
* Vòng lặp gyro/điều khiển lên đến 4kHz trên ESP32 với gyro SPI
* Giao tiếp MSP và CLI
* Ánh xạ tài nguyên/chân cắm (Resource/Pin mapping)
* Chỉnh PID trong khi bay
* Buzzer, đèn LED và giám sát điện áp
* Chế độ Failsafe
* **Altitude Hold** — giữ độ cao tự động sử dụng barometer
* **GPS Navigation** — giữ vị trí (Position Hold) và quay về nhà (Return to Home)

# Tài Liệu

Kho lưu trữ này chứa mã nguồn firmware cho phép bạn tự xây dựng bộ điều khiển bay của riêng mình. Để tiện lợi, firmware mô phỏng khả năng tương thích Betaflight 4.2, cho phép cấu hình bằng [betaflight-configurator](https://github.com/betaflight/betaflight-configurator). Ngoài ra, [online blackbox-log-viewer](https://blackbox.betaflight.com/) có thể được dùng để phân tích log blackbox. Về phần lớn các tính năng, firmware tương tự Betaflight, do đó nhiều hướng dẫn cấu hình và tuning đều có thể áp dụng. Tuy nhiên, cần lưu ý rằng phần mềm này không phải là Betaflight — có một số hạn chế và khác biệt về chức năng và hiệu suất.

> [!IMPORTANT]
> Trước khi bắt đầu, **hãy đọc kỹ tài liệu sau đây trước!**.

 * [Hướng Dẫn Cài Đặt](/docs/setup.md)
 * [Sơ Đồ Đấu Dây](/docs/wiring.md)
 * [Lệnh CLI](/docs/cli.md)
 * [WiFi và Bộ Thu ESP-NOW](/docs/wireless.md)

Tham gia **[Kênh Discord](https://discord.gg/jhyPPM5UEH)** để được hỗ trợ

# Bắt Đầu Nhanh

## Yêu Cầu

Phần cứng:
* Board ESP32 hoặc ESP32-S3
* Gyro MPU9250 SPI hoặc MPU6050 I2C (GY-88, GY-91, GY-521 hoặc tương tự)
* PDB có 5V BEC
* Buzzer và một số linh kiện điện tử (tùy chọn).

Phần mềm:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.10)
* [Driver CH340 usb-serial converter](https://sparks.gogo.co.nz/ch340.html)

## Nạp Firmware

1. Tải xuống và giải nén firmware từ [Trang Releases](https://github.com/rtlopez/esp-fc/releases)
2. Truy cập [ESP Tool Website](https://espressif.github.io/esptool-js/)
3. Nhấn "Connect" và chọn cổng thiết bị trong hộp thoại
4. Thêm file firmware và đặt Flash Address là `0x00`
5. Nhấn "Program"
6. Sau khi thành công, tắt và bật lại nguồn board

![ESP-FC Flashing](/docs/images/esptool-js-flash-connect.png)

## Cài Đặt

Sau khi nạp firmware, bạn cần cấu hình một số thứ trước:

 1. Cấu hình pinout theo sơ đồ đấu dây của bạn, đặc biệt là các chức năng chân cắm — xem thêm tại [CLI Reference](/docs/cli.md)
 2. Kết nối với [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) và thiết lập theo sở thích của bạn
 3. Kiểm tra động cơ không lắp cánh quạt
 4. Chúc vui vẻ ;)

> [!NOTE]
> Không phải tất cả các chức năng hiển thị trong configurator đều có trong firmware. Nguyên tắc chung là: nếu bạn không thể thay đổi một tùy chọn cụ thể trong Betaflight Configurator, tức là tính năng đó chưa được hỗ trợ — thường sẽ rollback về giá trị cũ sau khi lưu. Khuyến nghị mạnh mẽ nên làm theo [hướng dẫn cài đặt](/docs/setup.md).

## Sơ Đồ Đấu Dây

[![Sơ đồ đấu dây ESP-FC](/docs/images/espfc_wiring_combined.png)](/docs/wiring.md)

## Các Module Được Hỗ Trợ

 * **ESP32** - khuyến nghị
 * **ESP32-S3** - khuyến nghị
 * **ESP32-S2** - thử nghiệm
 * **ESP32-C3** - thử nghiệm, hiệu năng hạn chế, không có FPU
 * **RP2350** - thử nghiệm, hoạt động một phần
 * **RP2040** - thử nghiệm, hiệu năng hạn chế, không có FPU
 * **ESP8266** - lỗi thời, không còn phát triển

## Cảm Biến và Giao Thức Được Hỗ Trợ

 * Gyro: MPU6050, MPU6000, MPU6500, MPU9250, ICM20602, BMI160
 * Barometer: BMP180, BMP280, SPL06
 * Magnetometer: HMC5883, QMC5883, AK8963, QMC5883P
 * Bộ thu: PPM, SBUS, IBUS, CRSF/ELRS
 * Giao thức ESC: PWM, BRUSHED, ONESHOT125, ONESHOT42, MULTISHOT, DSHOT150, DSHOT300, DSHOT600
 * Giao thức khác: MSP, CLI, BLACKBOX, ESPNOW

# Kiến Trúc

## Mô Hình Thực Thi

**Single-core** (ESP8266, RP2040): `setup()` → `espfc.begin()`, sau đó `loop()` → `espfc.update()` + `espfc.updateOther()`.

**Dual-core ESP32 (FreeRTOS)**:
- **gyroTask** (core 1, ưu tiên cao): 4kHz điều khiển bằng hardware timer, chạy `espfc.update(true)` — đọc gyro, IMU fusion, PID
- **pidTask** (core 0, ưu tiên thấp hơn): chạy `espfc.updateOther()` — telemetry, RC input, arming động cơ, CLI/MSP

## Các Thành Phần Cốt Lõi (`lib/Espfc/src/`)

| Thành Phần | Mục Đích |
|-----------|---------|
| `Espfc.h/cpp` | Điều phối cấp cao nhất — sở hữu tất cả các hệ thống con |
| `Model.h` | Trạng thái chia sẻ có thể thay đổi + cấu hình lưu trên EEPROM |
| `Hardware.h` | Khởi tạo phần cứng nền tảng (chân cắm, bus) |
| `Control/Controller.h` | Vòng lặp điều khiển bay chính (rates → PIDs → mixer) |
| `Control/Fusion.h` | Ước tính tư thế AHRS (Madgwick/Mahony/Kalman) |
| `Control/Altitude.hpp` | Giữ độ cao tự động sử dụng bộ lọc bù (barometer) |
| `Control/GpsNavigation.hpp` | Điều hướng GPS — Position Hold và Return to Home |
| `Device/` | Driver phần cứng (gyros, baros, mags, receivers, buses) |
| `Sensor/` | Quản lý cảm biến bọc ngoài Device drivers |
| `Output/Mixer.h` | Mixer động cơ (throttle + PID outputs → lệnh động cơ) |
| `Connect/Msp.h` | Giao thức MSP — giao tiếp với Betaflight Configurator |
| `Connect/Cli.h` | CLI qua Serial để cấu hình |
| `Connect/StatusLed.h` | Điều khiển đèn LED trạng thái (LED đơn và WS2812 RGB) |
| `Blackbox/` | Ghi log dữ liệu bay (serial hoặc SPI flash) |
| `Telemetry/` | Đầu ra telemetry CRSF và text |
| `Utils/Filter.h` | Bộ lọc số: LPF, biquad, dynamic notch, RPM notch |
| `Utils/Timer.h` | Lập lịch tác vụ theo tần suất |

## Thư Viện Ngoài (`lib/`)

- **AHRS/** — Thuật toán fusion Madgwick, Mahony, Kalman
- **EscDriver/** — Driver giao thức ESC PWM/OneShot/DSHOT (theo platform: ESP32, C3, RP2040)
- **EspWire/** — Wrapper I2C/SPI
- **betaflight/** — Driver cảm biến và bộ lọc từ Betaflight
- **Gps/** — Phân tích dữ liệu GPS

## Luồng Dữ Liệu

```
Đầu vào RC (SBUS/IBUS/CRSF/PPM/ESP-NOW)
    → Input → RcCommand
    → Controller (rates) → PID → setpoint
    → Fusion (gyro + accel + mag → tư thế)
    → Mixer → lệnh động cơ
    → EscDriver → đầu ra ESC (PWM/DSHOT)
```

`Model` là trạng thái chia sẻ trung tâm được truy cập bởi tất cả các thành phần. Cấu hình được lưu qua `Utils/Storage.h` (EEPROM).

# Tính Năng Nâng Cao

## Altitude Hold (Giữ Độ Cao)

Altitude Hold sử dụng barometer kết hợp với bộ lọc bù hai trạng thái (complementary filter):

- **Baro**: cung cấp tham chiếu độ cao tuyệt đối, chậm nhưng chính xác về dài hạn
- **Accel (IMU)**: cung cấp động lực học thẳng đứng nhanh (sau khi loại bỏ trọng lực và bù nghiêng)
- Bộ lọc bù hợp nhất hai nguồn để ước tính độ cao và vận tốc thẳng đứng mượt mà

Chế độ này được kích hoạt qua switch `MODE_ALTHOLD` trong Betaflight Configurator.

## GPS Navigation (Điều Hướng GPS)

Module `GpsNavigation` cung cấp hai chế độ hoạt động:

- **Position Hold** (`MODE_POSHOLD`): giữ vị trí GPS hiện tại, sử dụng bộ điều khiển P kép (vị trí → vận tốc → góc nghiêng)
- **Return to Home / GPS Rescue** (`MODE_GPS_RESCUE`): tự động bay về điểm takeoff, sử dụng haversine distance + bearing để tính đường về

Yêu cầu: GPS module với fix hợp lệ và đủ số vệ tinh (cấu hình qua `gps.minSats`).

# Phát Triển

## Công Cụ

* Visual Studio Code
* Extension [PlatformIO](https://platformio.org/install/ide?install=vscode)
* Git

## Build System

Sử dụng **PlatformIO** với Arduino framework. Các target được hỗ trợ: `esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp8266`, `rp2040`, `rp2350`.

```bash
# Build target cụ thể
pio run -e esp32

# Build và nạp lên thiết bị
pio run -e esp32 -t upload

# Chạy unit tests (build trên máy host)
pio test -e native

# Chạy một test suite cụ thể
pio test -e native -f test_fc

# Dùng Docker (không cần cài PlatformIO)
docker-compose run pio pio run -e esp32
docker-compose run pio pio test -e native
```

Build flags đáng chú ý (bỏ comment trong `platformio.ini` để kích hoạt):
- `-DESPFC_DEBUG_SERIAL` — bật đầu ra debug qua serial
- `-DESPFC_DEV_PRESET_DSHOT` / `-DESPFC_DEV_PRESET_BRUSHED` — preset cho giao thức động cơ khi phát triển
- `-DESPFC_DEV_PRESET_BLACKBOX_SERIAL=1` — bật blackbox qua serial

## Testing

Tests nằm trong `test/` và sử dụng **Unity** + **ArduinoFake** (mock Arduino API cho build trên máy host).

```
test/
├── test_fc/         # Test lõi flight controller
├── test_esc/        # Test driver ESC
├── test_input_crsf/ # Test phân tích giao thức CRSF
├── test_msp/        # Test giao thức MSP
└── test_math/       # Test toán/bộ lọc
```

## Quy Ước Lập Trình

- **C++17** xuyên suốt (`-std=gnu++17`)
- Guard nền tảng dùng `#ifdef UNIT_TEST`, `#ifdef ESP32`, `#ifdef ARCH_RP2040`, v.v.
- `IRAM_ATTR` đánh dấu các hàm quan trọng về thời gian (ISR/gyroTask); được định nghĩa rỗng (`""`) trên native/RP2040
- Driver thiết bị theo pattern: `begin()` để khởi tạo, `update()` để đọc định kỳ, trả về mã trạng thái
- Struct `Model` được truyền bằng reference vào hầu hết các thành phần — tránh thêm biến global ở nơi khác
- Đường xử lý gyro quan trọng về thời gian phải lock-free; dùng `SemaphoreHandle_t` hoặc atomic flag cho giao tiếp cross-core

## CI/CD

GitHub Actions (`.github/workflows/platformio.yml`) chạy `pio test -e native` trước, sau đó build tất cả 7 target nếu tests pass. Các artifact firmware được upload lên release assets. Ảnh firmware hợp nhất (`firmware_0x00.bin`) được tạo bởi `merge_firmware.py` cho các biến thể ESP32.

# Vấn Đề / Báo Lỗi

Bạn có thể báo cáo lỗi qua GitHub [tracker](https://github.com/rtlopez/esp-fc/issues).
Bạn cũng có thể tham gia [Kênh Discord](https://discord.gg/jhyPPM5UEH)

# Giấy Phép

Dự án này được phân phối dưới Giấy Phép MIT. Lưu ý rằng:

PHẦN MỀM ĐƯỢC CUNG CẤP "NGUYÊN TRẠNG", KHÔNG CÓ BẤT KỲ BẢO ĐẢM NÀO, RÕ RÀNG HAY NGỤ Ý, BAO GỒM NHƯNG KHÔNG GIỚI HẠN Ở BẢO ĐẢM VỀ KHẢ NĂNG BÁN HÀNG, PHÙ HỢP CHO MỤC ĐÍCH CỤ THỂ VÀ KHÔNG VI PHẠM. TRONG MỌI TRƯỜNG HỢP, CÁC TÁC GIẢ HOẶC CHỦ SỞ HỮU BẢN QUYỀN SẼ KHÔNG CHỊU TRÁCH NHIỆM VỀ BẤT KỲ KHIẾU NẠI, THIỆT HẠI HOẶC TRÁCH NHIỆM PHÁP LÝ NÀO PHÁT SINH TỪ PHẦN MỀM NÀY.

# Ủng Hộ Dự Án

Nếu bạn thích dự án này và muốn nó tiếp tục được phát triển, bạn có thể ủng hộ một chút.

* BTC: 1Lopez7yPtbyjfLGe892JfheDFJMMt43tW
* LTC: LV3G3sJxz9AYpDMYUp8e1LCmerFYxVY3ak

set pin_input_rx -1
set pin_output_0 1
set pin_output_1 2
set pin_output_2 3
set pin_output_3 4
set pin_button -1
set pin_buzzer -1
set pin_led 48
set pin_serial_0_tx -1
set pin_serial_0_rx -1
set pin_serial_1_tx 6
set pin_serial_1_rx 5
set pin_serial_2_tx -1
set pin_serial_2_rx -1
set pin_i2c_scl 10
set pin_i2c_sda 9
set pin_input_adc_0 -1
set pin_input_adc_1 -1
set pin_spi_0_sck 12
set pin_spi_0_mosi 11
set pin_spi_0_miso 13
set pin_spi_cs_0 8
set pin_spi_cs_1 7
set pin_spi_cs_2 -1
set pin_buzzer_invert 1
set pin_led_invert 0
set pin_led_type STRIP