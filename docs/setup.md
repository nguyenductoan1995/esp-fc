# Thiết Lập Ban Đầu

Trang này mô tả các bước bắt buộc phải thực hiện trước chuyến bay đầu tiên.

## Kết Nối với Betaflight Configurator

Sau khi tải và cài đặt [Betaflight configurator](https://github.com/betaflight/betaflight-configurator), cần thay đổi một số tùy chọn trước. Mở configurator, vào tab **Options**, sau đó:
1. Tắt `Advanced CLI AutoComplete`
2. Bật `Show all serial devices`
3. Tắt `Auto-Connect`

![Options](/docs/images/bfc/bfc_options.png)

Sau đó chọn thiết bị từ danh sách và nhấn connect.

> [!NOTE]
> Không phải tất cả chức năng hiển thị trong configurator đều có sẵn trong firmware. Nguyên tắc chung: nếu không thể thay đổi một tùy chọn cụ thể trong Betaflight Configurator thì tùy chọn đó không được hỗ trợ — thường sẽ tự rollback về giá trị trước đó sau khi lưu.

> [!NOTE]
> Khi rời một số tab, đặc biệt là CLI, cần nhấn nút "Disconnect" hai lần rồi nhấn "Connect" lại.

## Cấu Hình Sơ Đồ Nối Dây

Vào tab `CLI` và nhập `get pin`. Lệnh này hiển thị các pin được gán cho từng chức năng. Nếu sơ đồ nối dây khác, có thể điều chỉnh tại đây. Xem thêm [Tham chiếu CLI chức năng pin](/docs/cli.md#chức-năng-pin).

![CLI Pins](/docs/images/bfc/bfc_cli_pins.png)

## Hiệu Chỉnh Gyro

Vào tab `Setup`, đảm bảo model đang ở trạng thái nằm ngang và tĩnh. Sau đó nhấn nút **"Calibrate Accelerometer"** và chờ hai giây.

> [!CAUTION]
> Đảm bảo rằng model trong bản xem trước di chuyển đúng như thực tế. Nếu không, vào tab `Configuration` và cấu hình `Board orientation` hoặc `Sensor Alignment`.

![Sensor Alignment](/docs/images/bfc/bfc_configuration.png)

## Cấu Hình Hệ Thống

Trong tab `Configuration`, đặt `Pid loop frequency`. Giá trị khuyến nghị là 1kHz đến 2kHz. Chú ý CPU Load hiển thị ở phía dưới — không nên vượt quá 50%.

## Thiết Lập Receiver

Nếu muốn dùng receiver nối tiếp (SBUS, IBUS, CRSF), cần cấp phát cổng UART cho nó. Thực hiện trong tab `Ports` bằng cách bật công tắc ở cột `Serial Rx`.

Sau đó vào tab `Receiver`, chọn `Receiver mode` và `Serial Receiver Provider`.

Để dùng ESP-NOW receiver, chọn chế độ "SPI Rx (e.g. built-in Rx)" trong tab Receiver. Cần module transmitter tương thích. Đọc thêm tại [Chức Năng Không Dây ESP-FC](/docs/wireless.md).

## Thiết Lập Motor

Trong tab `Motors` cần cấu hình:

### Mixer

Chọn loại mixer phù hợp với loại máy bay đang cấu hình.

**Quan trọng!** Đối với multirotor, phải đảm bảo:

1. Motor được đánh số đúng kết nối với đầu ra đúng và đặt đúng vị trí trong máy bay theo hướng của gyro, thể hiện trên hình ảnh.
2. Motor quay đúng chiều theo hình ảnh.

> [!CAUTION]
> Nếu các điều kiện trên không được đáp ứng, quad sẽ mất kiểm soát ngay khi khởi động và có thể gây hư hỏng hoặc thương tích.

Để kiểm tra, bật **test mode** và quay từng motor riêng lẻ:
1. Tháo toàn bộ cánh quạt,
2. Kết nối pin,
3. Nhấn "I understand the risk...",
4. Kéo thanh trượt của motor cần kiểm tra.

Nếu dùng giao thức analog (PWM, OneShot, Multishot), cần hiệu chỉnh ESC tại đây:
1. Nhấn "I understand the risk...",
2. Kéo thanh trượt chính lên giá trị cao nhất,
3. Kết nối pin,
4. Khi ESC phát ra tiếng beep hiệu chỉnh, kéo thanh trượt chính xuống thấp nhất,
5. ESC sẽ phát tiếng beep xác nhận.

> [!NOTE]
> Hiện tại chỉ hỗ trợ Quad X. Trình hướng dẫn chiều motor không được hỗ trợ — phải cấu hình ESC riêng. Cách dễ nhất để đổi chiều motor là hoán đổi dây motor.

### Chiều Motor Bị Đảo

Tùy chọn này thông báo cho FC rằng motor đang quay theo chiều ngược. Tùy chọn này không đổi chiều motor — phải thực hiện qua phần mềm ESC configurator hoặc hoán đổi dây motor.

### Giao Thức ESC

Chọn giao thức phù hợp với ESC đang dùng. Với multirotor, khuyến nghị dùng `DSHOT150` hoặc `DSHOT300`. ESC cũ có thể không hỗ trợ giao thức digital — trong trường hợp đó nên dùng tối thiểu là `OneShot125`. Giao thức `Brushed` dành cho motor brushed qua FET driver.

Không khuyến nghị dùng `PWM` cho multirotor.

### Tách Tốc Độ Motor PWM Khỏi Tốc Độ PID

Tùy chọn này cho phép tách tần số PWM. Nếu PID loop đặt ở 1k nhưng ESC chỉ nhận tối đa 333Hz, PWM sẽ bị giới hạn ở 480Hz.

## Chế Độ Bay

Các chế độ bay được cấu hình trong tab `Modes`.

### Arm

Bắt buộc bật để có thể bay. Khi armed, motor sẽ quay (trừ khi bật tùy chọn Motor stop).

### Angle

Mặc định, ACRO mode được kích hoạt nếu không có chế độ bay nào đang hoạt động. Chế độ Angle bật tự cân bằng — điều khiển góc nghiêng thay vì tốc độ xoay. Chỉ ảnh hưởng đến trục Roll và Pitch; trục Yaw hoạt động giống Acro mode.

### Air Mode

Tăng dải điều khiển ở các vị trí ga cực đoan. Khuyến nghị bật — có thể dùng cùng kênh RC với Arm để có hiệu ứng thường trực. Kích hoạt khi ga đạt khoảng 40% và duy trì đến khi disarm.

### Buzzer

Kích hoạt còi — ví dụ để tìm model khi bị rơi trong đám cỏ cao.

### Fail Safe

Kích hoạt quy trình Failsafe.

## Blackbox

Có thể thu thập dữ liệu bay theo hai cách: qua `cổng serial` hoặc `flash onboard`.

Flash onboard lưu được khoảng 2.5MB dữ liệu — tương đương 2–3 phút bay. Đủ để tune PID.

Nếu cần nhiều hơn, chọn `Serial Port` và thiết bị serial như [D-ronin OpenLager](https://github.com/d-ronin/openlager) hoặc [OpenLog](https://github.com/sparkfun/OpenLog). Cách cấu hình:
1. Trong `Ports`, chọn uart để tạo luồng dữ liệu và chọn `Blackbox logging` trong cột Peripherals trên cổng tự do.
2. Trong tab `Blackbox`, chọn `Serial Port` hoặc `Onboard flash` làm `logging device`.

> [!NOTE]
> Tốc độ cổng từ cột `Configuration/MSP` được dùng, và thiết bị ghi log phải dùng cùng tốc độ _(có thể thay đổi trong phiên bản tương lai)_.

Cài đặt khuyến nghị:
- Log ở tốc độ 1k cần thiết bị nhận dữ liệu ở 500kbps.
- Log ở tốc độ 2k cần 1Mbps.

OpenLager xử lý dễ dàng. Nếu dùng OpenLog, có thể cần flash [blackbox-firmware](https://github.com/cleanflight/blackbox-firmware) để hỗ trợ tốc độ hơn 115.2kbps. Cả 250kbps và 500kbps đều hoạt động tốt với thẻ SD hiện đại lên đến 32GB.

## Hạn Chế

### Tab Configuration

Trong Other features chỉ có thể bật `Dynamic Filter` và `SoftSerial`.

`AirMode` chỉ có trong tab modes. Để bật vĩnh viễn, dùng cùng kênh điều khiển với ARM.

### Tab Failsafe

Chỉ có quy trình "Drop" ở Stage 2.

### Tab Presets

Presets không được hỗ trợ — không thử áp dụng bất kỳ preset nào.

### Tab PID Tuning

1. Chỉ có một PID profile và một rate profile.
2. Feed-forward transition không có hiệu lực.
3. Iterm-rotation không có hiệu lực.
4. Dynamic damping không có hiệu lực.
5. Throttle boost không có hiệu lực.
6. Miscellaneous settings không có hiệu lực.
7. Dynamic Notch hoạt động từ PID loop 1k, nhưng không hiển thị trong configurator ở tốc độ này — để cấu hình lại, chuyển PID loop sang 2k.

Ngoài ra, hầu hết nguyên tắc Betaflight có thể áp dụng cho PID và filter tuning. Tuy nhiên, các tip tune rất aggressive không khuyến nghị áp dụng vì có thể cho kết quả khác.

### Receiver

1. Không phải tất cả giao thức đều được triển khai — hiện tại chỉ hỗ trợ PPM, CRSF, SBUS, IBUS.
2. Chỉ CRSF telemetry và rssi_adc.
3. RC deadband áp dụng cho RPY, không có Yaw deadband riêng.

### Modes

Chỉ hỗ trợ add range, không hỗ trợ add link.

### Adjustments

Chưa triển khai — thay thế bằng [pid scaler CLI](/docs/cli.md#cấu-hình-scaler).

### Servos

Chưa triển khai — thay thế bằng [output CLI](/docs/cli.md#cấu-hình-kênh-output) để thay đổi giao thức và tốc độ output.

### Motors

Không hỗ trợ tính năng 3D, chỉ hỗ trợ Quad-X Mixer.

### Video Transmitter

Chưa triển khai, đang trong quá trình phát triển.

### OSD

Chưa triển khai, nhưng MW_OSD có thể hoạt động qua cổng serial và giao thức MSP.
