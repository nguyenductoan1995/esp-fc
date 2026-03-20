# Chức Năng Không Dây ESP-FC

Các module Espressif có WiFi tích hợp, có thể dùng để cấu hình và điều khiển.

## Cấu Hình WiFi

> [!NOTE]
> Để cấu hình thiết bị qua kết nối WiFi, cần bật chức năng `SOFTSERIAL`. WiFi chỉ dùng để cấu hình thiết bị. Trong khi WiFi đang hoạt động, không thể ARM controller — cần reboot.

![Enable WiFi](/docs/images/espfc_wifi_ap_enable.png)

WiFi sẽ tự động tạo Access Point nếu board không nhận được tín hiệu receiver trong ít nhất 30 giây (ở trạng thái failsafe từ khi khởi động). Tên AP là `ESP-FC` và là mạng mở. Sau khi kết nối AP này, chọn `Manual Selection` và nhập `tcp://192.168.4.1:1111`.

![Connect to ESP-FC AP](/docs/images/espfc_wifi_ap_connect.png)

Có thể cấu hình ESP-FC tự động kết nối vào mạng nhà. Trong tab CLI, nhập tên mạng và mật khẩu:
```
set wifi_ssid MY-HOME-NET
set wifi_pass MY-HOME-PASS
```
> [!NOTE]
> Tên mạng và mật khẩu không được chứa dấu cách.

Kiểm tra trạng thái kết nối bằng lệnh `wifi` trong tab CLI:
```
wifi
ST IP4: tcp://0.0.0.0:1111
ST MAC: 30:30:F9:6E:10:74
AP IP4: tcp://192.168.4.1:1111
AP MAC: 32:30:F9:6E:10:74
```
Trong chế độ này, cần tìm địa chỉ IP được DHCP cấp. Dòng `ST IP4` là địa chỉ cần dùng trong configurator. Nếu hiển thị `0.0.0.0` nghĩa là FC không kết nối được với mạng nhà.

## Điều Khiển ESP-NOW

ESP-NOW là giao thức truyền thông không dây độc quyền của Espressif — cho phép điều khiển trực tiếp, nhanh và tiết kiệm điện mà không cần router.

Vì hiện tại không có transmitter thương mại nào dùng giao thức này, cần tự lắp module phát. Nếu đã có RC transmitter với khe JR bay, có thể dùng thêm một module ESP32. Trong trường hợp đó, làm theo hướng dẫn tại [espnow-rclink-tx](https://github.com/rtlopez/espnow-rclink-tx).

Trong ESP-FC, chọn `SPI Rx (e.g. built-in Rx)` làm Receiver mode trong tab Receiver.

![ESP-FC ESP-NOW Receiver](/docs/images/espfc_receiver.png)

Transmitter và receiver tự động bắt cặp sau khi khởi động — không cần thao tác thêm. Quy trình khởi động khuyến nghị:
1. Bật transmitter trước,
2. Sau đó cấp nguồn cho receiver/flight controller.
