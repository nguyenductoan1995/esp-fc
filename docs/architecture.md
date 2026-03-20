# Kiến Trúc Tổng Thể ESP-FC

## Tổng Quan

ESP-FC là firmware flight controller mã nguồn mở cho các board ESP32/ESP8266/RP2040, tương thích với hệ sinh thái Betaflight (giao thức MSP, Betaflight Configurator). Toàn bộ codebase được viết bằng C++17 và xây dựng bằng PlatformIO với Arduino framework.

---

## Mô Hình Thực Thi

### Single-Core (ESP8266, RP2040)

```
setup()
  └─ espfc.load()       ← Đọc cấu hình từ EEPROM/flash
  └─ espfc.begin()      ← Khởi tạo toàn bộ phần cứng và subsystem

loop()
  ├─ espfc.update()     ← Gyro, IMU fusion, PID, Mixer, ESC output
  └─ espfc.updateOther() ← Telemetry, RC input, Blackbox, CLI/MSP
```

Cả hai vòng lặp chạy tuần tự trên một core duy nhất. `update()` tự kiểm tra timer gyro bên trong.

---

### Dual-Core ESP32 (FreeRTOS)

```
Core 1 — gyroTask (độ ưu tiên cao, hardware timer 4kHz)
  └─ espfc.update(externalTrigger=true)
       ├─ Đọc gyro (IRAM, không blocking)
       ├─ IMU fusion (Mahony/Madgwick)
       ├─ PID controller
       └─ Gửi sự kiện vào appQueue (EVENT_GYRO_READ, EVENT_ACCEL_READ)

Core 0 — pidTask (độ ưu tiên thấp hơn)
  └─ espfc.updateOther()
       ├─ Nhận sự kiện từ appQueue
       ├─ Telemetry (CRSF)
       ├─ RC input processing
       ├─ Motor arming / failsafe
       ├─ Blackbox logging
       └─ CLI / MSP (Betaflight Configurator)
```

**Giao tiếp cross-core** được thực hiện qua `appQueue` (FreeRTOS queue) — lock-free từ phía gyroTask. Các cờ trạng thái dùng `SemaphoreHandle_t` hoặc biến atomic.

> [!IMPORTANT]
> Đường dẫn gyro (gyroTask) phải không blocking. Tất cả hàm trong gyroTask đều được đánh dấu `IRAM_ATTR` để đảm bảo thực thi từ IRAM, không bị cache miss.

---

## Bảng Các Component

| Component | File | Vai Trò |
|-----------|------|---------|
| `Espfc` | `Espfc.h/cpp` | Điều phối cấp cao nhất — sở hữu vòng đời tất cả subsystem |
| `Model` | `Model.h` | Shared mutable state + cấu hình lưu EEPROM |
| `ModelConfig` | `ModelConfig.h` | Cấu trúc cấu hình tĩnh (PID, filter, pin, protocol...) |
| `ModelState` | `ModelState.h` | Trạng thái động thời gian thực (gyro, attitude, altitude...) |
| `Hardware` | `Hardware.h/cpp` | Khởi tạo phần cứng platform (pin, bus I2C/SPI) |
| `Controller` | `Control/Controller.h/cpp` | Vòng lặp điều khiển bay chính (rates → PID → mixer) |
| `Fusion` | `Control/Fusion.h/cpp` | AHRS — ước lượng attitude (Madgwick/Mahony/Kalman) |
| `Altitude` | `Control/Altitude.hpp` | Ước lượng độ cao — complementary filter (baro + accel) |
| `GpsNavigation` | `Control/GpsNavigation.hpp` | Position Hold và Return to Home qua GPS |
| `Pid` | `Control/Pid.h/cpp` | Thuật toán PID với iterm relax, TPA, D-term filter |
| `Rates` | `Control/Rates.h/cpp` | Chuyển đổi RC stick → setpoint tốc độ xoay |
| `Actuator` | `Control/Actuator.h/cpp` | Xử lý arming, failsafe, flight mode activation |
| `SensorManager` | `SensorManager.h/cpp` | Quản lý vòng đời tất cả cảm biến |
| `Mixer` | `Output/Mixer.h` | Motor mixing (throttle + PID outputs → motor commands) |
| `EscDriver` | `lib/EscDriver/` | Driver ESC theo platform (PWM/OneShot/DSHOT) |
| `Input` | `Input.h/cpp` | Xử lý RC input đầu vào |
| `Msp` | `Connect/Msp.h` | Giao thức MSP — giao tiếp với Betaflight Configurator |
| `Cli` | `Connect/Cli.h` | CLI serial để cấu hình |
| `Blackbox` | `Blackbox/` | Ghi log dữ liệu bay (serial hoặc SPI flash) |
| `Telemetry` | `Telemetry/` | CRSF và telemetry văn bản |
| `Filter` | `Utils/Filter.h` | Bộ lọc số: LPF, Biquad, Dynamic Notch, RPM Notch |
| `Timer` | `Utils/Timer.h` | Lập lịch tác vụ theo tần số |
| `Storage` | `Utils/Storage.h` | Lưu/đọc cấu hình qua EEPROM |

---

## Luồng Dữ Liệu

```
RC Input (SBUS / IBUS / CRSF / PPM / ESP-NOW)
    │
    ▼
Input → RcCommand (giá trị kênh thô 1000–2000 µs)
    │
    ▼
Rates (ACTUAL/BETAFLIGHT/RACEFLIGHT)
    │  chuyển đổi stick position → setpoint tốc độ xoay (deg/s)
    ▼
Controller
    ├── Fusion (Gyro + Accel + Mag → quaternion attitude)
    │       └─ Mahony/Madgwick/Kalman
    ├── PID (setpoint - gyro_rate → correction)
    │       ├─ P: phản ứng tức thì với sai số
    │       ├─ I: loại bỏ sai số tích lũy
    │       ├─ D: giảm rung (D-term filter)
    │       └─ F: feed-forward từ RC stick
    └── Altitude Hold (baro + accel → throttle correction)
    │
    ▼
Mixer (QuadX / Custom)
    │  throttle + [roll, pitch, yaw] PID outputs → motor[0..3]
    ▼
EscDriver
    │  PWM / OneShot125 / DSHOT150/300/600
    ▼
ESC → Motor → Cánh quạt
```

**Nhánh song song (GPS Navigation):**
```
GPS Module (UART/serial)
    │
    ▼
GpsNavigation
    ├─ Haversine distance + bearing to home
    ├─ Position P controller → velocity setpoint
    └─ Velocity P controller → lean angle setpoint
    │
    ▼
Controller (overrides Roll/Pitch setpoint khi POSHOLD/GPS_RESCUE)
```

---

## Lập Lịch Task

### ESP32 Dual-Core

```
Core 1 — gyroTask
┌─────────────────────────────────────────┐
│  Hardware Timer ISR (4kHz interval)     │
│    └─ espfc.update(true)                │
│         ├─ GyroSensor::update()  4kHz   │
│         ├─ AccelSensor::update() 1kHz   │
│         ├─ Fusion::update()      1kHz   │
│         └─ PID / Mixer          1–4kHz  │
└─────────────────────────────────────────┘

Core 0 — pidTask
┌─────────────────────────────────────────┐
│  espfc.updateOther()                    │
│    ├─ BaroSensor::update()   ~100Hz     │
│    ├─ MagSensor::update()    ~100Hz     │
│    ├─ GpsSensor::update()    ~10Hz      │
│    ├─ Telemetry              1Hz        │
│    ├─ Blackbox               ~1kHz      │
│    └─ Serial/MSP/CLI         ~1kHz      │
└─────────────────────────────────────────┘
```

### Single-Core (ESP8266 / RP2040)

```
loop()
  ├─ espfc.update()       ← tất cả cảm biến + PID + Mixer
  └─ espfc.updateOther()  ← telemetry, blackbox, MSP/CLI
```

---

## Thư Viện Ngoài

| Thư Viện | Đường Dẫn | Mô Tả |
|----------|-----------|-------|
| `AHRS` | `lib/AHRS/` | Thuật toán Madgwick, Mahony, Kalman fusion |
| `EscDriver` | `lib/EscDriver/` | Driver PWM/OneShot/DSHOT theo platform |
| `EspWire` | `lib/EspWire/` | Wrapper I2C/SPI |
| `betaflight` | `lib/betaflight/` | Sensor driver và filter từ Betaflight |
| `Gps` | `lib/Gps/` | Parser GPS (NMEA/UBX) |

---

## Quy Ước Lập Trình

- **C++17** toàn bộ (`-std=gnu++17`)
- Platform guard: `#ifdef ESP32`, `#ifdef ESP8266`, `#ifdef ARCH_RP2040`, `#ifdef UNIT_TEST`
- `IRAM_ATTR` đánh dấu hàm nhạy thời gian (ISR, gyroTask) — được định nghĩa là chuỗi rỗng trên native/RP2040
- Driver thiết bị theo pattern: `begin()` khởi tạo, `update()` đọc định kỳ
- `Model` được truyền tham chiếu vào hầu hết component — tránh thêm global state
- Đường dẫn gyro phải lock-free; dùng `SemaphoreHandle_t` hoặc cờ atomic cho giao tiếp cross-core
