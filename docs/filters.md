# Hệ Thống Bộ Lọc (Filters)

## Tổng Quan

ESP-FC sử dụng nhiều loại bộ lọc số để làm sạch tín hiệu từ gyroscope, accelerometer, barometer và RC input trước khi đưa vào PID controller. Hệ thống filter được triển khai trong `lib/Espfc/src/Utils/Filter.h`.

---

## Các Loại Filter

### PT1 — First-Order IIR Low-Pass

Bộ lọc thông thấp bậc 1. Công thức cơ bản:

```
output = output + k × (input - output)
trong đó: k = dt / (dt + 1/(2π × freq))
```

- **Dốc cắt**: -20 dB/decade
- **CPU**: thấp nhất trong tất cả loại filter
- **Ứng dụng**: gyro LPF thứ cấp, RC input filtering, yaw LPF

| Tham Số | Mô Tả |
|---------|-------|
| `freq` | Tần số cắt (Hz) — giảm để lọc nhiều hơn |

---

### PT2 — Second-Order IIR Low-Pass (2× PT1 Cascade)

Hai PT1 mắc nối tiếp. Dốc cắt sắc hơn PT1.

- **Dốc cắt**: -40 dB/decade
- **Ứng dụng**: baro pre-filter, accel vertical filter trong Altitude estimator

---

### PT3 — Third-Order IIR Low-Pass (3× PT1 Cascade)

Ba PT1 mắc nối tiếp.

- **Dốc cắt**: -60 dB/decade
- **Ứng dụng**: RC input LP filter (`input_lpf_type PT3`), feed-forward LP filter

---

### BIQUAD — Biquad IIR (LPF / Notch / BPF)

Bộ lọc IIR bậc 2 theo bilinear transform. Hỗ trợ ba chế độ:
- **LPF** — Low-pass filter
- **Notch** — Band-reject filter (triệt tần số cụ thể)
- **BPF** — Band-pass filter

Hai variant triển khai:
- `FILTER_BIQUAD` / `FILTER_NOTCH` — Transposed Form II (TF2)
- `FILTER_NOTCH_DF1` — Direct Form I (DF1), ổn định hơn khi reconfigure động

- **Dốc cắt LPF**: -40 dB/decade
- **CPU**: trung bình
- **Ứng dụng**: gyro LPF chính, D-term LPF, accel LPF, mag LPF, static notch

---

### FO — First-Order IIR (Direct Form)

Bộ lọc bậc 1 dạng Direct Form, linh hoạt hơn PT1.

- **Ứng dụng**: gyro LPF3 (`gyro_lpf3_type FO`)

---

### FIR2 — 2-Tap FIR Moving Average

Bộ lọc FIR 2 mẫu:

```
output = (input + prev_input) / 2
```

- **Ứng dụng**: pre-filter trước notch filter, giảm phase shift so với IIR

---

### MEDIAN3 — Median Filter 3 Mẫu

Lấy median của 3 mẫu liên tiếp. Hiệu quả để loại bỏ spike/glitch.

- **Ứng dụng**: baro và accel để loại outlier trước LPF

---

### Dynamic LPF (Dyn LPF)

Bộ lọc LPF có tần số cắt thay đổi động theo throttle. Khi throttle tăng, motor quay nhanh hơn và tần số rung động cao hơn → cutoff tần số tự động tăng theo.

```
cutoff = map(throttle, 0%, 100%, gyro_dyn_lpf_min, gyro_dyn_lpf_max)
```

Tham số:
```
set gyro_dyn_lpf_min 170    # Tần số cắt tối thiểu (Hz) ở throttle thấp
set gyro_dyn_lpf_max 425    # Tần số cắt tối đa (Hz) ở full throttle
```

---

### Dynamic Notch Filter

Tự động phát hiện và triệt các tần số rung động chính qua FFT/frequency analysis. Mỗi trục gyro (Roll/Pitch/Yaw) có thể có tới `gyro_dyn_notch_count` notch song song.

Yêu cầu: PID loop ≥ 1kHz.

Tham số:
```
set feature_dyn_notch 1           # Bật Dynamic Notch
set gyro_dyn_notch_count 4        # Số notch song song mỗi trục (1–6)
set gyro_dyn_notch_q 300          # Q factor × 10 (300 = Q=30, notch hẹp)
set gyro_dyn_notch_min 80         # Tần số tối thiểu theo dõi (Hz)
set gyro_dyn_notch_max 400        # Tần số tối đa theo dõi (Hz)
```

---

### RPM Notch Filter

Triệt tần số ứng với tốc độ quay của từng motor (và các harmonic). Cần DSHOT Bidirectional để lấy RPM từ ESC.

```
notch_freq = (RPM / 60) × harmonic
```

Tham số:
```
set output_dshot_telemetry 1      # Bật DSHOT Bidirectional telemetry (RPM feedback)
set output_motor_poles 14         # Số cực từ tính của motor (thường 14)
set gyro_rpm_harmonics 3          # Số harmonic cần triệt (1–3)
set gyro_rpm_q 500                # Q factor × 10 (500 = Q=50)
set gyro_rpm_min_freq 100         # Tần số RPM tối thiểu (Hz)
set gyro_rpm_fade 30              # Fade-in range (Hz) khi RPM vào dải hợp lệ
set gyro_rpm_weight_1 100         # Trọng số harmonic 1 (%)
set gyro_rpm_weight_2 100         # Trọng số harmonic 2 (%)
set gyro_rpm_weight_3 100         # Trọng số harmonic 3 (%)
set gyro_rpm_tlm_lpf_freq 150     # LPF cho RPM telemetry (Hz)
```

---

## Chuỗi Filter Gyro

Dữ liệu gyro thô đi qua nhiều tầng lọc trước khi vào PID:

```
Gyro ADC (raw)
    │
    ▼ gyro_lpf (PT1 / BIQUAD / PT2 / PT3 / FO)  ← lọc chính
    │  tham số: gyro_lpf_type, gyro_lpf_freq
    │
    ▼ gyro_lpf2 (PT1 / BIQUAD / PT2 / PT3)       ← lọc thứ hai (tùy chọn)
    │  tham số: gyro_lpf2_type, gyro_lpf2_freq
    │
    ▼ Dynamic LPF (nếu bật)                       ← cutoff thay đổi theo throttle
    │  tham số: gyro_dyn_lpf_min, gyro_dyn_lpf_max
    │
    ▼ Static Notch 1 (nếu cài đặt)                ← triệt tần số cố định
    │  tham số: gyro_notch1_freq, gyro_notch1_cutoff
    │
    ▼ Static Notch 2 (nếu cài đặt)
    │  tham số: gyro_notch2_freq, gyro_notch2_cutoff
    │
    ▼ Dynamic Notch (nếu bật)                     ← FFT-based, tự động
    │  tham số: gyro_dyn_notch_count, gyro_dyn_notch_q, ...
    │
    ▼ RPM Notch (nếu bật)                         ← motor RPM-based
    │  tham số: gyro_rpm_harmonics, gyro_rpm_q, ...
    │
    ▼ PID Controller
```

---

## Chuỗi Filter D-term

D-term PID đặc biệt nhạy với nhiễu — có chuỗi filter riêng:

```
D-term (raw)
    │
    ▼ pid_dterm_lpf (PT1 / BIQUAD)
    │  tham số: pid_dterm_lpf_type, pid_dterm_lpf_freq
    │
    ▼ pid_dterm_lpf2 (PT1 / BIQUAD)               ← tầng lọc D-term thứ hai
    │  tham số: pid_dterm_lpf2_type, pid_dterm_lpf2_freq
    │
    ▼ Dynamic LPF cho D-term (nếu cài đặt)
    │  tham số: pid_dterm_dyn_lpf_min, pid_dterm_dyn_lpf_max
    │
    ▼ D-term Notch (nếu cài đặt)
       tham số: pid_dterm_notch_freq, pid_dterm_notch_cutoff
```

---

## Hướng Dẫn Chọn Filter

| Tình Huống | Khuyến Nghị |
|-----------|------------|
| Gyro LPF tiêu chuẩn | `BIQUAD` hoặc `PT2`, cutoff 100–200Hz |
| Giảm latency | `PT1` với cutoff cao hơn (200–300Hz) |
| Motor có rung động cố định | Static Notch ở đúng tần số |
| Motor rung động biến thiên theo throttle | Dynamic Notch + Dynamic LPF |
| Có DSHOT Bidirectional | RPM Notch (chính xác nhất) |
| RC input smoothing | `PT3`, `input_lpf_freq 0` (tự động từ `input_lpf_factor`) |

---

## Tham Số CLI Đầy Đủ

```
# Gyro LPF stage 1
set gyro_lpf_type PT1
set gyro_lpf_freq 100

# Gyro LPF stage 2
set gyro_lpf2_type PT1
set gyro_lpf2_freq 213

# Gyro LPF stage 3 (FO)
set gyro_lpf3_type FO
set gyro_lpf3_freq 150

# Static Notch
set gyro_notch1_freq 0       # 0 = tắt
set gyro_notch1_cutoff 0
set gyro_notch2_freq 0
set gyro_notch2_cutoff 0

# Dynamic LPF
set gyro_dyn_lpf_min 170
set gyro_dyn_lpf_max 425

# Dynamic Notch
set feature_dyn_notch 0      # 0 = tắt
set gyro_dyn_notch_q 300
set gyro_dyn_notch_count 4
set gyro_dyn_notch_min 80
set gyro_dyn_notch_max 400

# RPM Notch
set output_dshot_telemetry 0 # 0 = tắt bidirectional
set gyro_rpm_harmonics 3
set gyro_rpm_q 500
set gyro_rpm_min_freq 100
set gyro_rpm_fade 30

# D-term filter
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 128
set pid_dterm_lpf2_type PT1
set pid_dterm_lpf2_freq 128
set pid_dterm_dyn_lpf_min 60
set pid_dterm_dyn_lpf_max 145
```
