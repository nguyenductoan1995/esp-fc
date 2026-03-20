# Giao Thức ESC

## Tổng Quan

ESP-FC hỗ trợ nhiều giao thức truyền thông giữa flight controller và ESC (Electronic Speed Controller). Việc chọn giao thức phù hợp ảnh hưởng đến độ trễ điều khiển motor, độ chính xác và tính năng bidirectional (phản hồi RPM).

Driver ESC được triển khai theo từng platform trong `lib/EscDriver/src/`.

---

## Danh Sách Giao Thức

### PWM (Analog)

Giao thức analog truyền thống, dùng độ rộng xung (pulse width) để truyền lệnh.

- **Dải giá trị**: 1000–2000 µs
- **Tần số**: thường 50–400Hz
- **Latency**: cao (2–20ms)
- **Đặc điểm**: Tương thích với mọi ESC. Không khuyến nghị cho multirotor hiện đại.

---

### OneShot125

Phiên bản nhanh hơn của PWM, thu nhỏ dải xung 8 lần.

- **Dải giá trị**: 125–250 µs
- **Latency**: ~125–250 µs
- **Đặc điểm**: Cần ESC hỗ trợ OneShot. Tốt hơn PWM, nhưng vẫn analog.

---

### OneShot42

Thu nhỏ dải xung thêm 3 lần so với OneShot125.

- **Dải giá trị**: 42–84 µs
- **Latency**: ~42–84 µs

---

### Multishot

Giao thức analog nhanh nhất.

- **Dải giá trị**: 5–25 µs
- **Latency**: ~5–25 µs
- **Đặc điểm**: Cần ESC được thiết kế đặc biệt cho Multishot.

---

### Brushed

Giao thức cho motor brushed qua FET driver (không phải ESC brushless).

- **Đặc điểm**: Điều khiển trực tiếp duty cycle PWM. Dùng cho quad brushed nhỏ.

---

### DSHOT150

Giao thức digital tốc độ thấp. Bit rate: 150 kbit/s.

- **Đặc điểm**: Không cần calibrate ESC, không bị ảnh hưởng bởi nhiễu điện áp, truyền giá trị digital chính xác.
- **Khuyến nghị**: Cho hệ thống 1kHz PID loop trở xuống.

---

### DSHOT300

Giao thức digital tiêu chuẩn. Bit rate: 300 kbit/s.

- **Khuyến nghị**: Cho hệ thống 2kHz PID loop. Lựa chọn mặc định tốt nhất.

---

### DSHOT600

Giao thức digital tốc độ cao. Bit rate: 600 kbit/s.

- **Khuyến nghị**: Cho hệ thống 4kHz PID loop trở lên.

---

## Hỗ Trợ Theo Platform

| Giao Thức | ESP8266 | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | RP2040 |
|-----------|:-------:|:-----:|:--------:|:--------:|:--------:|:------:|
| PWM | Có | Có | Có | Có | Có | Có |
| OneShot125 | Có | Có | Có | Có | Có | Có |
| OneShot42 | Có | Có | Có | Có | Có | Có |
| Multishot | Có | Có | Có | Có | Có | Có |
| Brushed | Có | Có | Có | Có | Có | Có |
| DSHOT150 | Có | Có | Có | Có | Có | Có |
| DSHOT300 | Có | Có | Có | Có | Có | Có |
| DSHOT600 | Hạn chế | Có | Có | Có | Có | Có |
| DSHOT Bidirectional | Không | Có | Có | Có | Có | Có |

> [!NOTE]
> ESP8266 có timer hạn chế — DSHOT600 có thể không ổn định ở một số cấu hình. Khuyến nghị dùng DSHOT150/300 trên ESP8266.

### Số Kênh ESC Theo Platform

| Platform | Số Kênh Tối Đa |
|----------|:--------------:|
| ESP8266 | 4 |
| ESP32-C3 | 4 |
| ESP32-S2, ESP32-S3 | 4 |
| ESP32 | `RMT_CHANNEL_MAX` (thường 8) |
| RP2040 | 8 |

---

## DSHOT Bidirectional và RPM Filter

### DSHOT Bidirectional

DSHOT Bidirectional cho phép ESC gửi phản hồi RPM về FC trong cùng dây tín hiệu (half-duplex). FC dùng thông tin RPM để tính toán chính xác tần số rung động từ motor và áp dụng RPM Notch Filter.

Yêu cầu:
- ESC hỗ trợ DSHOT Bidirectional (BLHeli_32, AM32 hoặc BLHeli_S với firmware hỗ trợ)
- Platform: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, RP2040

Kích hoạt:
```
set output_motor_protocol DSHOT300
set output_dshot_telemetry 1
save
reboot
```

### Cấu Hình RPM Filter

Sau khi bật bidirectional telemetry, cấu hình RPM Notch Filter:

```
set output_motor_poles 14       # Số cực từ motor (đọc từ thông số kỹ thuật motor)
                                # Motor 2204 = 12 cực, motor 2306 = thường 14 cực

set gyro_rpm_harmonics 3        # Số harmonic: 1 = chỉ tần số cơ bản
                                #              3 = tần số cơ bản + 2 harmonic (2×, 3×)

set gyro_rpm_q 500              # Q factor × 10 (500 = Q=50, notch hẹp hơn)
set gyro_rpm_min_freq 100       # Bỏ qua RPM dưới tần số này (Hz)
set gyro_rpm_fade 30            # Fade-in khi RPM vào dải hợp lệ (Hz width)
set gyro_rpm_weight_1 100       # Trọng số harmonic 1 (%)
set gyro_rpm_weight_2 100       # Trọng số harmonic 2 (%)
set gyro_rpm_weight_3 100       # Trọng số harmonic 3 (%)
set gyro_rpm_tlm_lpf_freq 150   # LPF cho tín hiệu RPM telemetry đến (Hz)
```

### Tính Số Cực Motor

```
số cực từ = số nam châm trên stator / 2

Ví dụ:
- Motor có 14 nam châm → output_motor_poles = 14
- Công thức RPM: motor_RPM = (eRPM × 60) / (poles × ERPM_PER_LSB)
```

---

## Cấu Hình ESC Protocol

### Qua Betaflight Configurator

Tab **Motors** → chọn **ESC/Motor Protocol**.

### Qua CLI

```
# Chọn giao thức
set output_motor_protocol DSHOT300

# Tốc độ output (Hz) — 0 = sync với PID loop
set output_motor_rate 480

# Tách tốc độ PWM khỏi PID rate (async mode)
set output_motor_async 0        # 0 = sync, 1 = async

# Idle throttle cho DSHOT (giữ motor quay khi armed)
set output_dshot_idle 550       # giá trị DSHOT (0–2000), ~5.5% throttle

# Giới hạn throttle tối thiểu và tối đa
set output_min_throttle 1070
set output_max_throttle 2000
set output_min_command 1000     # Lệnh khi disarmed (motor dừng hoặc idle)

save
reboot
```

---

## Khuyến Nghị Chọn Giao Thức

| Loại Build | Giao Thức Khuyến Nghị |
|-----------|----------------------|
| Multirotor hiện đại + ESC BLHeli_32/AM32 | `DSHOT300` + Bidirectional |
| Multirotor + ESC cũ (BLHeli_S) | `DSHOT150` hoặc `OneShot125` |
| Multirotor + ESC rất cũ (chỉ analog) | `OneShot125` |
| Brushed tiny whoop | `Brushed` |
| ESP8266 (giới hạn tốc độ) | `DSHOT150` hoặc `OneShot125` |

> [!NOTE]
> Không khuyến nghị dùng `PWM` cho multirotor vì latency cao và thiếu chính xác.

---

## Calibrate ESC (Giao Thức Analog)

Chỉ cần calibrate khi dùng PWM/OneShot/Multishot. DSHOT không cần calibrate.

1. Trong tab **Motors**, nhấn "I understand the risk..."
2. Kéo thanh trượt Master lên maximum
3. Kết nối pin
4. Chờ ESC phát tiếng beep calibration
5. Kéo thanh trượt Master xuống minimum
6. ESC phát tiếng beep xác nhận — hoàn thành
