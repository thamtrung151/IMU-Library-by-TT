# Dùng trong firmware

`IMULibrary.h` là API portable C++11, không phụ thuộc Arduino. `ttimu/ArduinoBus.h` là adapter Arduino có sẵn.

## Các hàm thường dùng

| Hàm | Dùng để làm gì |
|---|---|
| `begin(bus, Sensor::Auto, config)` | Nhận dạng, cấu hình chip và bắt đầu hiệu chuẩn; trả false nếu lỗi |
| `update()` | Đọc theo lịch, không chờ vòng lặp; true khi có snapshot mới |
| `ready()` | Dữ liệu hiện tại đã qua hiệu chuẩn và không bị lỗi đọc/gap vừa phát hiện |
| `data()` | Tham chiếu tới snapshot mới nhất; sao chép nếu cần giữ lâu |
| `startCalibration()` | Hiệu chuẩn lại gyro; giữ bo yên |
| `zeroYaw()` | Đổi mốc yaw hiện tại về 0, cập nhật cả quaternion |
| `diagnostics()` / `error()` | Xem retry, mất mẫu, chu kỳ thực và lỗi |
| `calibration()` / `setCalibration()` | Lưu/nạp hệ số qua flash/NVS do ứng dụng quản lý |

Mỗi instance và bus do **một task** sở hữu. Nếu task khác cần dữ liệu, copy snapshot dưới cơ chế đồng bộ của ứng dụng. Không gọi `update()` từ ISR. Không truyền dữ liệu cũ sang controller khi `ready()` false hoặc timestamp đã quá cũ.

## Cấu hình

```cpp
ttimu::Config cfg;                  // sample/fusion 1000/1000 Hz
cfg.axes[0] = 2; cfg.axes[1] = -1; cfg.axes[2] = 3; // ví dụ bo xoay 90°
imu.begin(bus, ttimu::Sensor::ICM42688, cfg);
```

`axes` là hoán vị có dấu từ trục sensor sang trục bo, phải giữ hệ thuận. Kiểm tra roll/pitch khi nghiêng bo trước khi dùng controller.

`Config::inav9()` chọn sample 4000 Hz/fusion 2000 Hz cho **ICM qua SPI**. MPU và I²C dùng 500 hoặc 1000 Hz. Tần số fusion phải chia hết tần số sample. Filter phải thấp hơn Nyquist; cấu hình không hợp lệ bị từ chối.

Mặc định: gyro anti-alias 250 Hz, gyro main PT1 60 Hz, acc biquad 15 Hz, gyro Kalman bật, window gyro 2000 ms, threshold raw 32. `gyroKalman=false` dành cho phép thử có kiểm soát, không tự coi là profile tương đương.

`setArmed(bool)` giữ ngữ nghĩa INAV để kiểm soát startup gain/rate-ignore; mặc định false phù hợp dùng FC làm bộ đo không arm. `setRateSetpoint(Vector3)` nhận **độ/giây** nếu ứng dụng muốn cung cấp rate target cho gyro Kalman, mặc định bằng 0. Không có PID điều khiển động cơ bên trong thư viện.

## Output và hiệu chuẩn

- Euler theo dấu của INAV MSP, nhưng là float: roll/pitch theo độ, yaw `[0,360)`.
- Quaternion Hamilton `(w,x,y,z)` giữ biểu diễn nội bộ INAV. Yaw Euler dùng dấu âm so với phép atan2 quaternion thông thường. Khi tích hợp NED/ENU hoặc robotics khác, xác nhận basis/sign trước.
- Gyro `rad/s`, acc `m/s²` gồm trọng lực, nhiệt `°C`; timestamp `µs` tương đối từ sample đầu và mở rộng wrap `micros()` 32 bit.
- Mặc định tự hiệu chuẩn gyro mỗi boot. Calibration của một chip không dùng lại cho chip khác dù cùng model.
- `Calibration` cho phép nạp offset/gain acc đã đo. Công cụ tự hiệu chuẩn acc sáu tư thế chưa có trong v0.1.0; hệ số mặc định là offset 0, gain 1.
- Muốn dùng gyro bias đã lưu, đặt `startupCalibration=false`, gọi `begin()`, sau đó `setCalibration(saved)`. Cho tới khi nạp được hệ số đúng model/version, `ready()` vẫn false. NVS/flash và kiểm tra CRC của bản lưu do ứng dụng quản lý.

## ESP-IDF / STM32 HAL

Kế thừa `RegisterBus`, cung cấp `read`, `write`, `delayMs`, `micros32`, `isSPI`. Read phải đọc cả burst trong một transaction. Giữ CS thấp suốt địa chỉ và payload; write MPU không quá 1 MHz. Các hàm delay chỉ được dùng khi cấu hình chip, không nằm trong vòng xử lý bình thường.

Có thể bỏ transport và gọi `beginRaw(sensor, config)` / `feed(rawSample)` để chạy custom acquisition hoặc replay. Raw phải đúng ±2000 dps, ±16g/2048 LSB/g và timestamp duy nhất theo rate đã cấu hình; không feed dữ liệu đã lọc/scale vào API raw này.
