# Tính Năng Altitude Hold

## Tổng Quan

Altitude Hold (giữ độ cao) cho phép flight controller tự động duy trì độ cao hiện tại bằng cách điều chỉnh ga (throttle). Tính năng này sử dụng ước lượng độ cao từ **complementary filter** kết hợp dữ liệu từ barometer và accelerometer.

---

## Nguyên Lý Hoạt Động

### Complementary Filter (Baro + Accel)

Nguồn: `lib/Espfc/src/Control/Altitude.hpp`

Barometer cung cấp tham chiếu độ cao tuyệt đối nhưng chậm và nhiễu (thường ~50–100Hz). Accelerometer cung cấp động học thẳng đứng nhanh hơn nhưng tích lũy sai số drift theo thời gian.

Complementary filter kết hợp hai nguồn theo hai bước:

**Bước Predict (driven bởi accel):**
```
vel    += accelZ_world * accelTrust * dt
height += vel * dt
```

**Bước Correct (pulled bởi baro khi có dữ liệu mới):**
```
height += BARO_GAIN_HEIGHT * (baroHeight - height)
vel    += BARO_GAIN_VARIO  * (baroVario  - vel)
```

### Bù Góc Nghiêng

Khi máy bay nghiêng, thành phần lực đẩy thẳng đứng giảm xuống. Hệ số `accelTrust` được tính như sau:

```
accelTrust = max(0, (cosTheta - 0.5) * 2)
```

- Bằng 0 khi góc nghiêng >= 60°
- Bằng 1 khi bay thẳng đứng (cosTheta = 1)

Điều này ngăn accel đóng góp sai vào ước lượng khi máy bay đang maneuver mạnh.

### Hệ Số Gain

| Hằng Số | Giá Trị | Ý Nghĩa |
|---------|---------|---------|
| `BARO_GAIN_HEIGHT` | 0.02 | Độ hiệu chỉnh height theo baro mỗi update |
| `BARO_GAIN_VARIO` | 0.04 | Độ hiệu chỉnh velocity theo baro mỗi update |
| `MAX_VARIO_MS` | 10.0 m/s | Giới hạn tốc độ thay đổi độ cao để ngăn runaway |

Giá trị gain nhỏ hơn = mượt hơn (accel-driven), phản ứng chậm hơn. Giá trị gain lớn hơn = bám baro nhanh hơn, nhạy hơn với nhiễu baro.

### Bộ Lọc Pre-processing

| Bộ Lọc | Loại | Tần Số Cắt | Mục Đích |
|--------|------|-----------|---------|
| `_baroFilter` | PT2 | 10 Hz | Làm mượt dữ liệu baro thô trước khi đưa vào complementary filter |
| `_baroVarioFilter` | PT1 | 10 Hz | Làm mượt vario (tốc độ thay đổi độ cao) từ baro |
| `_accelZFilter` | PT2 | 10 Hz | Lọc nhiễu rung động từ accel theo trục Z |

---

## Kết Quả Đầu Ra

Sau mỗi chu kỳ update, kết quả được ghi vào:
- `model.state.altitude.height` — độ cao ước lượng (đơn vị: mét)
- `model.state.altitude.vario` — tốc độ thay đổi độ cao (đơn vị: m/s)

---

## Tần Số Cập Nhật

`Altitude::update()` chạy tại tần số của accel timer — thường 400Hz đến 1kHz tùy cấu hình PID loop.

---

## Cách Bật Altitude Hold

### Qua Betaflight Configurator

1. Vào tab **Modes**
2. Thêm range cho channel AUX tương ứng với chế độ `ALTHOLD`
3. Nhấn **Save**

### Qua CLI

```
# Bật chế độ ALTHOLD trên channel AUX2 (channel index 5), range 1700-2100
set mode_3 3 5 1700 2100 0 0
save
reboot
```

Mode ID cho ALTHOLD là `3` (xem [tham chiếu mode IDs](/docs/cli.md#điều-kiện-chế-độ-bay)).

---

## Tham Số Cấu Hình Liên Quan

### Barometer

```
set baro_dev AUTO           # Tự động phát hiện thiết bị baro (BMP280, SPL06, MS5611...)
set baro_bus AUTO           # Tự động chọn bus (I2C/SPI)
set baro_lpf_type BIQUAD    # Loại filter cho dữ liệu baro
set baro_lpf_freq 3         # Tần số cắt filter baro (Hz) — thấp hơn = mượt hơn
```

### PID Altitude (khi Altitude Hold hoạt động)

Altitude Hold sử dụng PID controller riêng để điều chỉnh throttle. Các tham số PID liên quan:

```
# PID cho altitude (FC_PID_ALT)
set pid_level_p 45          # P gain — phản ứng với sai số độ cao
set pid_level_i 0           # I gain — loại bỏ sai số tĩnh
set pid_level_d 0           # D gain
```

### Debug Mode

Kích hoạt debug để xem dữ liệu altitude trong tab Blackbox:

```
set debug_mode ALTITUDE
```

Khi bật, 6 kênh debug sẽ xuất:
- `debug[0]` — baro height (cm)
- `debug[1]` — baro vario (cm/s)
- `debug[2]` — fused height (cm)
- `debug[3]` — fused vario (cm/s)
- `debug[4]` — vertical accel (cm/s²)
- `debug[5]` — tilt trust × 1000

---

## Yêu Cầu Phần Cứng

Barometer bắt buộc. Accelerometer (thường tích hợp cùng IMU) cũng được dùng để cải thiện độ chính xác ước lượng.

| Cảm Biến | Vai Trò | Bắt Buộc? |
|---------|---------|----------|
| Barometer (BMP280/SPL06/MS5611) | Tham chiếu độ cao tuyệt đối | Có |
| Accelerometer (tích hợp trong IMU) | Động học thẳng đứng | Không (cải thiện chất lượng) |

---

## Hạn Chế và Lưu Ý

- **Nhiễu barometer**: Barometer rất nhạy với luồng khí từ cánh quạt. Nên che chắn baro bằng foam mỏng để giảm nhiễu áp suất.
- **Độ chính xác**: Ước lượng độ cao có sai số ±0.5m tùy điều kiện. Không phù hợp cho hạ cánh chính xác tự động.
- **Khởi tạo**: Hệ thống cần vài giây sau khi khởi động để ổn định giá trị baro baseline. Không nên arm ngay khi bật nguồn.
- **Gió**: Gió mạnh ảnh hưởng đến baro. Altitude Hold có thể kém ổn định trong điều kiện gió lớn.
- **Không có GPS**: Altitude Hold chỉ dùng baro đơn thuần — không fuse dữ liệu GPS altitude.
