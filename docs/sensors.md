# Cảm Biến Được Hỗ Trợ

## Tổng Quan

ESP-FC hỗ trợ nhiều loại cảm biến phần cứng. Tất cả driver cảm biến nằm trong `lib/Espfc/src/Device/`. Hệ thống tự động phát hiện thiết bị khi `dev = AUTO` (mặc định).

---

## Gyroscope / Accelerometer (IMU)

IMU là cảm biến bắt buộc. Gyroscope cung cấp tốc độ xoay cho PID controller; accelerometer cung cấp vector gia tốc cho AHRS fusion.

### Danh Sách Thiết Bị Hỗ Trợ

| Tên Driver | Chip | Địa Chỉ I2C | Hỗ Trợ SPI | Ghi Chú |
|-----------|------|:-----------:|:----------:|---------|
| `GyroMPU6050` | MPU-6000, MPU-6050 | 0x68 / 0x69 | Không | Phổ biến nhất, giá rẻ |
| `GyroMPU6500` | MPU-6500, MPU-6000S | 0x68 / 0x69 | Có | WHOAMI: 0x70 hoặc 0x75 |
| `GyroMPU9250` | MPU-9250 | 0x68 / 0x69 | Có | Tích hợp AK8963 compass |
| `GyroICM20602` | ICM-20602 | 0x68 / 0x69 | Có | WHOAMI: 0x12 |
| `GyroLSM6DSO` | LSM6DSO, LSM6DSOX | 0x6A / 0x6B | Có | STMicroelectronics |
| `GyroBMI160` | BMI160 | 0x69 / 0x68 | Có | Bosch, hiệu năng cao |

> [!NOTE]
> `GYRO_AUTO` (mặc định) tự động thử lần lượt tất cả driver theo thứ tự cho đến khi tìm thấy thiết bị phản hồi đúng WHO_AM_I.

### Cấu Hình Gyro/Accel

```
# Chọn thiết bị (AUTO = tự động phát hiện)
set gyro_dev AUTO
set accel_dev AUTO

# Chọn bus giao tiếp
set gyro_bus AUTO    # AUTO, I2C, SPI

# DLPF (Digital Low Pass Filter) phần cứng trong chip
# Giảm DLPF → tần số cao hơn → nhiều nhiễu hơn nhưng ít latency hơn
set gyro_dlpf 256Hz  # 256Hz, 188Hz, 98Hz, 42Hz, 20Hz, 10Hz, 5Hz

# Căn chỉnh hướng cảm biến (nếu gyro không đặt theo hướng mặc định)
set gyro_align DEFAULT
# Các giá trị: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG
#              CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP

# Offset hiệu chỉnh (tự động qua lệnh cal gyro)
set gyro_offset_x 0
set gyro_offset_y 0
set gyro_offset_z 0

# Căn chỉnh accel riêng (nếu khác gyro)
set accel_offset_x 0
set accel_offset_y 0
set accel_offset_z 0
```

### Hiệu Chỉnh Gyro

```
# Trong CLI — đặt board nằm phẳng trước khi chạy
cal gyro
```

Hoặc qua Betaflight Configurator: tab **Setup** → **Calibrate Accelerometer**.

---

## Barometer (Cảm Biến Khí Áp)

Barometer dùng để ước lượng độ cao thông qua áp suất không khí. Cần cho tính năng Altitude Hold.

### Danh Sách Thiết Bị Hỗ Trợ

| Tên Driver | Chip | Địa Chỉ I2C | Hỗ Trợ SPI | Ghi Chú |
|-----------|------|:-----------:|:----------:|---------|
| `BaroBMP085` | BMP085 | 0x77 | Không | Cũ, thay thế bằng BMP280 |
| `BaroBMP280` | BMP280 | 0x76 / 0x77 | Có | Phổ biến nhất, giá rẻ |
| `BaroSPL06` | SPL06-001 | 0x76 / 0x77 | Có | WHOAMI: 0x10, chính xác cao |
| `BaroMS5611` | MS5611 | 0x77 | Không | Độ chính xác cao, dùng cho drone chuyên nghiệp |

### Cấu Hình Barometer

```
# Chọn thiết bị (NONE = tắt baro)
set baro_dev AUTO    # AUTO, NONE, BMP085, MS5611, BMP280, SPL06
set baro_bus AUTO

# Filter cho dữ liệu baro
set baro_lpf_type BIQUAD   # PT1, BIQUAD, PT2, PT3
set baro_lpf_freq 3        # Tần số cắt (Hz) — thấp hơn = mượt hơn
```

> [!NOTE]
> Barometer rất nhạy với nhiễu áp suất từ cánh quạt. Nên che chắn baro bằng foam xốp mỏng để cải thiện độ chính xác.

---

## Magnetometer / Compass (Từ Kế)

Từ kế dùng để xác định hướng tuyệt đối (Bắc/Nam). Cải thiện độ chính xác heading cho GPS navigation và yaw estimation.

### Danh Sách Thiết Bị Hỗ Trợ

| Tên Driver | Chip | Địa Chỉ I2C | Ghi Chú |
|-----------|------|:-----------:|---------|
| `MagHMC5883L` | HMC5883L | 0x1E | Cũ nhưng phổ biến |
| `MagQMC5883L` | QMC5883L | 0x0D | Clone của HMC5883L, giá rẻ |
| `MagQMC5883P` | QMC5883P | - | Phiên bản mới hơn |
| `MagAK8963` | AK8963 | 0x0C | Tích hợp trong MPU-9250 |
| `MagAK8975` | AK8975 | 0x0C | Phiên bản cũ hơn AK8963 |

### Cấu Hình Magnetometer

```
# Chọn thiết bị
set mag_dev NONE    # NONE, HMC5883, AK8975, AK8963, QMC5883, QMC5883P
set mag_bus AUTO

# Căn chỉnh hướng từ kế (nếu không đặt cùng hướng gyro)
set mag_align DEFAULT

# Filter
set mag_filter_type BIQUAD
set mag_filter_lpf 10       # Tần số cắt (Hz)

# Hiệu chỉnh Hard Iron (offset)
set mag_offset_x 0
set mag_offset_y 0
set mag_offset_z 0

# Hiệu chỉnh Soft Iron (scale)
set mag_scale_x 1000
set mag_scale_y 1000
set mag_scale_z 1000
```

> [!NOTE]
> Cần thực hiện hard iron calibration (xoay máy bay theo nhiều hướng) để có dữ liệu compass chính xác. Từ trường từ dây điện và motor có thể gây nhiễu nặng nếu từ kế đặt quá gần.

---

## GPS Module

GPS dùng cho tính năng Position Hold và Return to Home. Xem tài liệu chi tiết tại [GPS Navigation](/docs/gps-navigation.md).

### Giao Tiếp

GPS kết nối qua UART (serial). Hỗ trợ các giao thức NMEA và UBX (u-blox).

### Module Phổ Biến

| Module | Hệ Thống Vệ Tinh | Tần Số Update | Giao Tiếp |
|--------|-----------------|:-------------:|-----------|
| Neo-6M | GPS | 5–10Hz | UART |
| Neo-7M | GPS + GLONASS | 10Hz | UART |
| Neo-8M | GPS + GLONASS + Galileo | 10Hz | UART |
| Neo-M8N | GPS + GLONASS + Galileo + BeiDou | 10Hz | UART |
| BN-220 | GPS + GLONASS | 10Hz | UART |

### Cấu Hình GPS

```
set feature_gps 1           # Bật GPS feature
set gps_min_sats 8          # Số vệ tinh tối thiểu
set gps_set_home_once 1     # 1 = lưu Home khi arm, 0 = cập nhật liên tục

# Serial port cho GPS (ví dụ: serial_1 = UART 2)
set serial_1 2 115200 0     # function=2 (GPS), baud=115200
```

---

## Cách Kết Nối Cảm Biến

### Giao Tiếp I2C

I2C dùng 2 dây (SCL + SDA), cho phép kết nối nhiều thiết bị trên cùng bus.

```
Cảm biến    → ESP32      ESP32-S3
VCC (3.3V)  → 3.3V       3.3V
GND         → GND        GND
SCL         → GPIO22      GPIO10   (pin_i2c_scl)
SDA         → GPIO21      GPIO9    (pin_i2c_sda)
```

Nhiều module có thể chia sẻ cùng bus I2C nếu địa chỉ không trùng nhau.

> [!TIP]
> Cài đặt tốc độ I2C:
> ```
> set i2c_speed 800    # kHz — 400 hoặc 800 khuyến nghị
> ```

### Giao Tiếp SPI

SPI nhanh hơn I2C, dùng 4 dây (SCK + MOSI + MISO + CS). Mỗi thiết bị cần một dây CS riêng.

```
Cảm biến    → ESP32      ESP32-S3
VCC (3.3V)  → 3.3V       3.3V
GND         → GND        GND
SCK         → GPIO18      GPIO12   (pin_spi_0_sck)
MOSI/SDA    → GPIO23      GPIO11   (pin_spi_0_mosi)
MISO/SDO    → GPIO19      GPIO13   (pin_spi_0_miso)
CS (Gyro)   → GPIO5       GPIO8    (pin_spi_cs_0)
CS (Baro)   → GPIO13      GPIO7    (pin_spi_cs_1)
```

> [!TIP]
> SPI cho hiệu năng tốt hơn I2C. Ưu tiên dùng SPI nếu module hỗ trợ.

---

## Phát Hiện Tự Động

Khi `gyro_dev = AUTO`, `baro_dev = AUTO`, `mag_dev = AUTO`, firmware tự động thử từng driver theo thứ tự:

1. Kiểm tra trên bus đã chọn (hoặc thử I2C trước, rồi SPI nếu là AUTO)
2. Đọc register WHO_AM_I
3. So sánh với giá trị kỳ vọng của từng chip
4. Driver đầu tiên phản hồi đúng sẽ được sử dụng

Dùng lệnh `status` trong CLI để xác nhận thiết bị đã phát hiện:

```
status
...
     devices: BMI160/I2C
```

---

## Lưu Ý Phần Cứng

- **Điện áp**: Tất cả cảm biến liệt kê dùng 3.3V. Không kết nối vào 5V.
- **Pull-up I2C**: Bus I2C cần điện trở pull-up (thường 4.7kΩ đến 10kΩ) trên SCL và SDA. Nhiều module breakout đã có sẵn pull-up.
- **Rung động**: Gắn IMU trên đệm chống rung (foam tape) để giảm nhiễu cơ học truyền vào gyro.
- **Nhiệt độ**: Một số IMU (MPU-6050) có thể bị drift khi nhiệt độ thay đổi. Cho phép warm-up ~30 giây trước khi calibrate.
