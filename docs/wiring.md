# Ví Dụ Nối Dây và Ánh Xạ PIN ESP-FC

ESP32 MCU cho phép remap pin, vì vậy sơ đồ nối dây không cố định và có thể điều chỉnh theo nhu cầu. Để thay đổi chức năng pin, vào CLI và dùng lệnh `get pin` để kiểm tra gán hiện tại. Ví dụ: đặt đầu ra đầu tiên về pin 1:

`set pin_output_0 1`

Để bỏ gán chức năng pin, dùng -1 làm số pin:

`set pin_output_3 -1`

> [!NOTE]
> Vẫn có một số giới hạn khi remap. Không phải tất cả pin đều có thể dùng cho tất cả chức năng. Một số pin chỉ là input, một số là strapping pin và có thể ngăn boot nếu dùng sai. Vui lòng tham khảo tài liệu MCU để biết các tùy chọn khả dụng. Layout mặc định đã được kiểm chứng hoạt động.

## Ánh Xạ Pin I2C Mặc Định Cho Module Gyro

| Chân Module | Tên CLI          | ESP32 | ESP32-S3 |
|-------------|------------------|------:|---------:|
| SCK/SCL     | `pin_i2c_scl`    | 22    | 10       |
| SDA/SDI     | `pin_i2c_sda`    | 21    | 9        |

> [!NOTE]
> Driver I2C chỉ chấp nhận pin từ 1 đến 31.

## Ánh Xạ Pin SPI Mặc Định Cho Module Gyro

| Chân Module | Tên CLI          | ESP32 | ESP32-S3 |
|-------------|------------------|------:|---------:|
| SCK/SCL     | `pin_spi_0_sck`  | 18    | 12       |
| SDA/SDI     | `pin_spi_0_mosi` | 23    | 11       |
| SAO/SDO/ADO | `pin_spi_0_miso` | 19    | 13       |
| NCS         | `pin_spi_cs_0`   |  5    |  8       |
| CSB*        | `pin_spi_cs_1`   | 13    |  7       |

**Lưu ý:** `CSB` cần thiết cho barometer trên các module MPU-9250 10-DOF.

> [!TIP]
> Ưu tiên dùng SPI để có hiệu năng tốt hơn.

## Ánh Xạ Đầu Ra Servo/Motor Mặc Định

| Motor  | Tên CLI        | ESP32 | ESP32-S3 |
|-------:|----------------|------:|---------:|
| 1      | `pin_output_0` | 27    | 39       |
| 2      | `pin_output_1` | 25    | 40       |
| 3      | `pin_output_2` | 4     | 41       |
| 4      | `pin_output_3` | 12    | 42       |

## Ánh Xạ Pin Uart/Serial Mặc Định

| Uart | Tên CLI           | ESP32 | ESP32-S3 |
|-----:|-------------------|------:|---------:|
| RX 1 | `pin_serial_0_rx` |  3    | 44       |
| TX 1 | `pin_serial_0_tx` |  1    | 43       |
| RX 2 | `pin_serial_1_rx` | 32    | 15       |
| TX 2 | `pin_serial_1_tx` | 33    | 16       |
| RX 3 | `pin_serial_2_rx` | 16    | 17       |
| TX 4 | `pin_serial_2_tx` | 17    | 18       |

## Ánh Xạ Pin Analog Mặc Định

| Chức Năng | Tên CLI           | ESP32 | ESP32-S3 |
|----------:|-------------------|------:|---------:|
| Voltage   | `pin_input_adc_0` |  36   | 1        |
| Current   | `pin_input_adc_1` |  19   | 4        |

> [!NOTE]
> Trên ESP32, chỉ dùng các pin thuộc kênh ADC1.

## Ánh Xạ Pin PPM Receiver Mặc Định

| Chức Năng | Tên CLI        | ESP32 | ESP32-S3 |
|----------:|----------------|------:|---------:|
| PPM       | `pin_input_rx` |  35   | 6        |

## Các Chức Năng Pin Khác

| Tên CLI             | ESP32 | ESP32-S3 | Ghi Chú       |
|---------------------|------:|---------:|---------------|
| `pin_buzzer`        |  0    | 5        | Còi trạng thái |
| `pin_led`           |  26   | -        | LED trạng thái |

## Ví Dụ Nối Dây ESP32 SPI MPU-6500/MPU-9250

![ESP-FC ESP32 SPI Wiring](./images/esp-fc-esp32_spi_wiring.png)

## Ví Dụ Nối Dây ESP32 I2C MPU-6050

![ESP-FC ESP32 I2C Wiring](./images/esp-fc-esp32_i2c_wiring.png)

## Ví Dụ Nối Dây ESP8266 I2C MPU-6050

ESP8266 có khả năng remap pin hạn chế — dùng lệnh `get pin` để liệt kê các tùy chọn khả dụng.

![ESP-FC ESP8266 I2C Wiring](./images/espfc_wemos_d1_mini_wiring.png)
