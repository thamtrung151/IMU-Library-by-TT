# IMU Library by TT

Thư viện C++ xử lý IMU 6 trục cho **ESP32 / STM32**, với lõi tính góc port từ **INAV 9.0.1**.

**v0.1.0 — bản đầu để thử nghiệm:** đã kiểm tra phép tính và biên dịch ví dụ ESP32-S3/STM32F411. Chưa đo drift trên bo thật; cần bước đó trước khi dùng dữ liệu để điều khiển bot.

## Có gì?

- ICM42605, ICM42688-P, MPU6500: SPI hoặc I²C. MPU6050: I²C.
- Tự hiệu chuẩn gyro khi khởi động; rung thì tự thử lại.
- Bộ lọc gyro/acc, gyro Kalman, quaternion và roll/pitch/yaw độ phân giải float.
- Đọc thêm gyro (rad/s), acc (m/s²), nhiệt độ, timestamp và trạng thái sẵn sàng.
- Trang HTML xem đồ thị qua USB, mở/ghi CSV, hiệu chuẩn lại và đặt mốc yaw.
- Lõi không cấp phát heap khi chạy. Có interface để dùng ESP-IDF/STM32 HAL ngoài Arduino.

## Nối dây và chạy

Ví dụ SPI: sửa pin cho đúng bo của bạn, dùng logic **3,3 V**, chung GND.

| IMU | ESP32-S3 trong ví dụ | STM32F411 trong ví dụ |
|---|---|---|
| SCK | GPIO12 | PA5 |
| SDO / MISO | GPIO13 | PA6 |
| SDI / MOSI | GPIO11 | PA7 |
| CS | GPIO10 | PA4 |

I²C dùng SDA/SCL của bo, địa chỉ thường `0x68` hoặc `0x69`. MPU6050 không dùng SPI.

1. Tải ZIP thư viện từ [Releases](https://github.com/thamtrung151/IMU-Library-by-TT/releases), cài bằng Arduino IDE → **Add .ZIP Library**.
2. Mở ví dụ phù hợp: [ESP32-S3 + đồ thị](examples/SerialPlotter/SerialPlotter.ino), [STM32 SPI](examples/STM32_SPI/STM32_SPI.ino), [MPU6050 I²C](examples/MPU6050_I2C/MPU6050_I2C.ino).
3. Sửa pin, build/nạp cho bo của bạn. Giữ IMU đứng yên khoảng **2 giây** khi khởi động.

Cách lấy dữ liệu sau khi đã khởi tạo như ví dụ:

```cpp
void loop() {
    if (imu.update() && imu.ready()) {
        const auto& d = imu.data();
        // d.roll, d.pitch, d.yaw       : độ
        // d.quaternion.w/x/y/z
        // d.gyro.x/y/z                : rad/s
        // d.accel.x/y/z               : m/s², gồm trọng lực
        // Đưa dữ liệu vào bộ điều khiển của bạn ở đây.
    }
}
```

Gọi `update()` thường xuyên, tránh `delay()` trong vòng đọc. Mặc định lấy mẫu/tính góc 1 kHz, phù hợp cả SPI/I²C. Muốn thử ICM qua SPI ở mức 4 kHz/2 kHz: truyền `ttimu::Config::inav9()` vào `begin()`.

## Xem đồ thị

Nạp một ví dụ ở trên (cả SPI/I²C đều gửi cùng định dạng đồ thị), đóng Serial Monitor rồi chạy trong thư mục repo:

```sh
python tools/serve_viewer.py
```

Mở **http://localhost:8080** bằng Chrome/Edge → **Kết nối USB**, baud `921600`. Dữ liệu giữ trên máy; không cần tài khoản/cloud. Nút “Xem thử giao diện” dùng dữ liệu mô phỏng và luôn ghi rõ trên màn hình.

STM32: chọn USB CDC trong Arduino IDE nếu dùng cổng USB trên bo; hoặc dùng bộ USB–UART nối với chân Serial. Cấu hình PlatformIO kèm repo bật USB CDC cho BlackPill.

![Trang theo dõi IMU — dữ liệu mô phỏng](docs/monitor.png)

## Tiến độ

- [x] Lõi góc INAV 9, hiệu chuẩn gyro, lọc và API C++.
- [x] Driver bốn IMU, ví dụ ESP32-S3/STM32, giao diện đồ thị/CSV.
- [x] Kiểm tra trên máy và biên dịch chéo cho ESP32-S3/STM32F411.
- [ ] Kiểm tra từng IMU trên bo thật, đo drift/nhiệt/rung và chỉnh profile theo log.
- [ ] Hoàn thiện hiệu chuẩn acc 6 tư thế, bù nhiệt và các tối ưu theo kết quả đo.

Yaw là hướng **tương đối**, không phải la bàn. Bản này dùng profile dành cho robot, không chứa toàn bộ flight stack/bộ lọc theo động cơ của INAV. Chi tiết cho người sửa code: [API](docs/API.md), [nguồn và thay đổi](docs/PORTING.md), [kiểm thử](docs/TESTING.md).

**GPL-3.0-or-later** — kế thừa mã INAV; xem [giấy phép](LICENSE) và [ghi nhận nguồn](THIRD_PARTY_NOTICES.md).
