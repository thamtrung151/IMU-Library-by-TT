# Kiểm thử bản 0.1.3

Kiểm tra cập nhật ngày **08/11/2026**. Phân biệt rõ: build được và đúng phép tính chưa đồng nghĩa đã đo chất lượng trên phần cứng.

## Đã qua

- **Host C++11:** build bằng CMake/Ninja, GCC 15.2.0 trên Windows. Kiểm tra bias/calib retry, đơn vị, nhiều instance, wrap timer, mất mẫu, cấu hình sai, I/O lỗi, byte order và thanh ghi của bốn model.
- **Header tích hợp:** hồi quy lỗi macro `sq`/`radians` khi Arduino.h đứng trước thư viện; giữ nguyên macro của ứng dụng. Các ví dụ nhúng cũng dùng thứ tự include này trên cả ESP32/STM32.
- **Đối chiếu INAV:** 100.000 bước ở 2 kHz với các trạng thái/gate khác nhau. Sai khác component quaternion cực đại bằng 0 trong lần chạy host này, có xét q và -q. So cả integral và acc weight. Reference dùng hàm Mahony đầy đủ từ snapshot gốc, cùng các math primitives được giữ nguyên. Đây là phép thử numeric hợp lệ, không phải toàn firmware hay ground truth vật lý.
- **Parser đồ thị:** 8 test Node.js cho chunk/CRLF/dòng lỗi, reconnect và CSV 17/25/26 cột, roll_sign, unwrap roll/yaw; bộ đệm có giới hạn.
- **Startup:** sáu mặt gravity, khởi động úp/nghiêng, freefall/gravity lỗi, ready, map trục FC và hiệu chuẩn lại ở uptime 150 giây. Không còn tự hội tụ từ pose phẳng giả định trong các ca đứng yên này.
- **Chạy lâu / dấu roll:** mô phỏng 200 giây chuyển động đi qua mốc 150 giây và wrap micros, không có lần gọi calib chủ động và không tự calib. Đổi rollSign không đổi pitch/yaw/quaternion/gyro; rollRateRadS cùng dấu roll. Không phải bài chạy dài trên bo thật.
- **Trình duyệt thật:** desktop 1440×900, mobile 390×844; bảy đồ thị cùng hiển thị, status/temp ở đầu, không tràn ngang/không lỗi JS. Test hover đọc đúng cùng mẫu, pause giữ hình/số khi dữ liệu vẫn nhận, serial giả lập 26 cột, CALIBRATE/ZERO_YAW và CSV export giữ giá trị firmware. Kiểm tra nền đồ thị trắng, nền ngoài đen, nút chính xanh Hatsune, số góc lớn ở giữa. Mở được log thật 25 cột; không trực tiếp kết nối USB bo trong QA này.
- **Biên dịch chéo:** bốn cấu hình PlatformIO thành công: ESP32-S3 SPI, ESP32-S3 I²C, STM32F411 SPI, STM32F411 I²C. Các ví dụ dùng chung giao thức đồ thị; STM32 bật USB CDC.
- **FC tự chế:** `tt_f411_icm42605` build/link, HSE 8 MHz/CPU 96 MHz/USB 48 MHz/SPI2/PC13. [Hai log v0.1.1](../reports/2026-11-07-stationary/README.md) chỉ ra lỗi startup/trục. [Log v0.1.2 có chuyển động](../reports/2026-11-08-motion/README.md) và phản hồi người dùng xác nhận cải thiện; đổi roll firmware ở v0.1.3 còn chờ nạp/kiểm tra vật lý. Chưa có đo dài hạn.
- **Dự án robot độc lập:** ESP32-S3 dùng `lib_deps` local symlink, không có `-I src` của repo; include Arduino.h trước thư viện, build/link thành công ở v0.1.1. Bản cũ v0.1.0 được phát hiện lỗi include trong phép thử này và đã được sửa ở v0.1.1.

Chi tiết build và footprint của **toàn ví dụ, gồm Arduino core/USB**, ở [build-results.json](build-results.json). `sizeof(IMU)` trên host ABI là 2424 byte; không phải tổng RAM firmware. Chưa đo CPU/latency trên MCU đang chạy.

Toolchain được dùng: PlatformIO 6.2.0; Espressif32 6.6.0 / Arduino ESP32 2.0.14 / Xtensa 8.4.0; STM32 platform 19.7.1 / Arduino STM32 2.12.0 / ARM GCC 12.3.1. Có warning `uartSetPins` trong Arduino ESP32 2.0.14, ngoài source thư viện; build/link vẫn thành công.

## Chạy lại

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
pio run -e esp32s3 -e stm32f411 -e esp32s3_i2c -e stm32f411_i2c -e tt_f411_icm42605
```

Host test cần CMake ≥3.20, compiler C++11, Python 3 và Node ≥18 cho parser. Build nhúng cần PlatformIO. CI trong repo tự chạy lại host và năm cấu hình nhúng; kết quả CI được xem trực tiếp trên GitHub, tách với kiểm tra local này.

`tools/qa_viewer.mjs` dùng Chrome headless riêng ở cổng 9227, viewer 8093 để tránh đụng server 8080 người dùng đang chạy. Có thể đổi bằng TTIMU_QA_CDP_URL/TTIMU_QA_VIEWER_URL. Không dùng hồ sơ trình duyệt của bạn.

## Chưa kiểm tra — việc tiếp theo trên bo thật

1. Xác nhận nhận dạng chip, trục và nhiệt độ; để nằm yên lúc boot, kiểm tra `ready` và retry.
2. Đo yaw 5/15/30 phút ở cold/warm start, lưu CSV cùng board/chip/config.
3. Quay ±90°/180° và nghiêng bo; thử rung/động cơ/gia tốc thực của bot.
4. So với FC INAV mà bạn đang tin dùng và một baseline AHRS cấu hình hợp lý trên cùng sensor/điều kiện.
5. Đo CPU, latency, mất mẫu khi Wi-Fi/RTOS/UART bận; điều chỉnh profile theo log.

Không có tuyên bố drift bằng 0, tốt hơn bao nhiêu phần trăm hoặc tương đương toàn bộ INAV firmware trong bản này. Chỉ dùng cho điều khiển sau khi đã kiểm tra trên tổ hợp bo/IMU thực tế của bạn.
