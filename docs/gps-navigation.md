# GPS Navigation

## Tổng Quan

ESP-FC hỗ trợ hai chế độ điều hướng GPS:
- **POSHOLD** (Position Hold) — giữ vị trí ngang hiện tại
- **GPS_RESCUE** (Return to Home) — tự động bay về điểm cất cánh

Cả hai chế độ đều được triển khai trong `lib/Espfc/src/Control/GpsNavigation.hpp`.

---

## Nguyên Lý Hoạt Động

### Tính Toán Khoảng Cách và Hướng Bay

Sử dụng **công thức Haversine** để tính khoảng cách và hướng bay từ vị trí hiện tại về điểm Home:

```
gpsDistanceBearing(home.lat, home.lon, current.lat, current.lon,
                   distanceToHome, bearingToHome)
```

- `distanceToHome`: khoảng cách từ vị trí hiện tại về Home (mét)
- `bearingToHome`: hướng về Home (radian, 0 = Bắc, chiều kim đồng hồ)
- `directionToHome`: góc cần bay về Home tương đối với hướng mũi máy bay (độ, -180 đến 180)

### Position Controller (2 Tầng)

**Tầng 1 — Position P Controller:**
```
velSetpoint = min(distanceToHome * posP, GPS_MAX_VELOCITY)
```
Chuyển sai số khoảng cách (mét) thành velocity setpoint (m/s), giới hạn tối đa 5 m/s.

**Tầng 2 — Velocity P Controller:**
```
rollSetpoint  = clamp((velEastSp  - velEast)  * velP, -maxLean, maxLean)
pitchSetpoint = clamp((velNorthSp - velNorth) * velP, -maxLean, maxLean)
```
Chuyển sai số tốc độ thành góc nghiêng (radian), giới hạn bởi `gps.maxLeanAngle`.

### Lọc Vận Tốc GPS

Vận tốc GPS (mm/s) được lọc qua PT1 filter 2Hz trước khi đưa vào velocity controller:
```
_velNorth = PT1_filter(gps.velocity.north * 0.001f)  // mm/s → m/s
_velEast  = PT1_filter(gps.velocity.east  * 0.001f)
```

---

## Chế Độ POSHOLD (Position Hold)

Khi POSHOLD được kích hoạt, máy bay giữ nguyên vị trí ngang (latitude/longitude) tại thời điểm kích hoạt.

Controller tính toán góc nghiêng Roll/Pitch cần thiết để giữ vị trí dựa trên:
1. Khoảng cách từ vị trí hiện tại đến điểm giữ
2. Vận tốc GPS hiện tại

---

## Chế Độ GPS_RESCUE (Return to Home)

Khi GPS_RESCUE được kích hoạt, máy bay tự động bay về điểm Home đã lưu lúc arm.

Điểm Home được lưu khi:
- Máy bay được arm lần đầu
- Hoặc khi `gps_set_home_once = 0`: cập nhật liên tục mỗi khi có GPS fix

```
set gps_set_home_once 1   # 1: lưu home một lần khi arm (khuyến nghị)
                          # 0: cập nhật home liên tục
```

---

## Tham Số Cấu Hình

### Cài Đặt GPS Cơ Bản

```
set feature_gps 1           # Bật tính năng GPS
set gps_min_sats 8          # Số vệ tinh tối thiểu để có GPS fix hợp lệ
set gps_set_home_once 1     # Lưu điểm Home một lần khi arm
```

### PID Điều Hướng

```
# FC_PID_POS: Position P controller (khoảng cách → velocity setpoint)
# Giá trị 0–255 được scale thành 0.0–2.55 m/s per meter
set pid_pos_p 20            # P gain cho position controller

# FC_PID_POSR: Velocity P controller (velocity error → lean angle)
# Giá trị 0–255 được scale thành 0.0–0.255 rad per (m/s)
set pid_posr_p 50           # P gain cho velocity controller
```

### Giới Hạn Bay

```
set gps_max_lean_angle 30   # Góc nghiêng tối đa khi điều hướng (độ)
```

---

## Cách Bật GPS Navigation

### Qua Betaflight Configurator

1. Vào tab **Configuration** → bật `GPS` trong Features
2. Trong tab **Ports**: cấp phát UART cho GPS (chọn `GPS` trong cột Sensor Input)
3. Vào tab **Modes**:
   - Thêm range cho POSHOLD (nếu cần)
   - Thêm range cho GPS_RESCUE

### Qua CLI

```
# Bật GPS feature
set feature_gps 1

# Cấu hình cổng serial cho GPS (ví dụ: serial_1)
# GPS thường dùng 9600 hoặc 115200 baud
set serial_1 2 115200 0     # function 2 = GPS

# Cài GPS minimum satellites
set gps_min_sats 6

# Bật chế độ POSHOLD trên AUX3 (channel 6), range 1300-1700
set mode_4 7 6 1300 1700 0 0

# Bật chế độ GPS_RESCUE trên AUX3 (channel 6), range 1700-2100
set mode_5 8 6 1700 2100 0 0

save
reboot
```

Mode IDs:
- `7`: MODE_POSHOLD
- `8`: MODE_GPS_RESCUE

---

## Yêu Cầu Phần Cứng

### Module GPS

Bất kỳ module GPS UART nào đều tương thích (NMEA hoặc UBX protocol). Các module phổ biến:

| Module | Giao Tiếp | Ghi Chú |
|--------|-----------|---------|
| Neo-6M | UART | Phổ biến, giá rẻ, chính xác trung bình |
| Neo-7M | UART | Cải thiện độ nhạy so với Neo-6M |
| Neo-8M | UART | Hỗ trợ GLONASS, khuyến nghị |
| Neo-M8N | UART | Đa hệ thống (GPS+GLONASS+Galileo), tốt nhất |
| BN-220 | UART | Nhỏ gọn, phổ biến cho FPV |

### Kết Nối GPS

Kết nối module GPS với UART của ESP32/ESP32-S3:

| Module GPS | ESP32 | ESP32-S3 | Mô Tả |
|-----------|-------|---------|-------|
| TX (GPS) | RX2 (GPIO32) | RX2 (GPIO15) | Dữ liệu từ GPS về FC |
| RX (GPS) | TX2 (GPIO33) | TX2 (GPIO16) | Lệnh từ FC đến GPS |
| VCC | 3.3V hoặc 5V | 3.3V hoặc 5V | Tùy module |
| GND | GND | GND | Nối đất |

---

## Hạn Chế và Lưu Ý

- **Số vệ tinh**: Cần ít nhất `gps_min_sats` vệ tinh mới kích hoạt điều hướng. Mặc định là 8. Ở trong nhà hoặc gần công trình cao, số vệ tinh có thể không đủ.
- **Warm-up GPS**: Module GPS cần 30–60 giây sau khi cấp nguồn để có cold fix. Khi đã có fix, subsequent fix nhanh hơn nhiều.
- **Độ chính xác vị trí**: GPS dân dụng có độ chính xác ~2–5m. POSHOLD sẽ dao động trong bán kính này.
- **Compass (từ kế)**: Để GPS_RESCUE hoạt động chính xác, nên có compass — giúp FC xác định đúng hướng mũi máy bay. Xem [tài liệu cảm biến](/docs/sensors.md#magnetometer).
- **Không có telemetry altitude**: Altitude Hold và GPS navigation hiện tại độc lập — altitude của GPS không được fuse vào Altitude Hold.
- **Indoor**: GPS không hoạt động trong nhà. Không kích hoạt GPS modes khi test trong nhà.
