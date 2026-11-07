# Debug hai log đứng yên ngày 07/11/2026

Bo: FC F411 + ICM42605, HSE 8 MHz, SPI2/CS PC13, baseline project v0.1.1 theo phi?n b?n ?? giao (CSV c? kh?ng ch?a phi?n b?n firmware). Người dùng xác nhận robot đứng yên hoàn toàn và roll/pitch bị đổi chỗ. CSV là telemetry sau lọc khoảng 100 Hz; không phải raw sensor 1 kHz. Tệp gốc chỉ đọc tại máy người dùng, không được chép vào repo.

| Chỉ số | Log 16-43-32 | Log 16-44-26 |
|---|---:|---:|
| Số mẫu | 2090 | 2509 |
| Thời gian ghi | 20,989 s | 25,199 s |
| Uptime đầu log | 134,647 s | 19,788 s |
| Thay đổi roll, đã unwrap | −168,058° | −0,803° |
| Thay đổi yaw | −117,651° | +1,839° |
| Xu hướng yaw trong đoạn ghi | Đang hội tụ sai tư thế, không dùng làm drift rate | +4,362°/phút (fit đoạn ngắn) |
| Acc Z trung bình | −9,8131 m/s² | −9,8214 m/s² |
| Độ lớn acc trung bình | 9,8178 m/s² | 9,8219 m/s² |
| Độ lệch chuẩn độ lớn acc | 0,00496 m/s² | 0,00449 m/s² |
| dt được ghi, min–max | 1000–1016 µs | 1000–1015 µs |
| Counter gaps / late | 0 / 0 | 0 / 0 |
| Counter thử lại calib | 1 | 1 |

Không có dấu hiệu gia tốc thay đổi đủ để giải thích góc tự quay lớn. Counter bằng 0 chỉ nói phần mềm không phát hiện late/gap theo ngưỡng hiện tại; không chứng minh không hề mất mẫu ODR. Hai log không bao gồm cửa sổ gyro calibration, nên chưa xác minh được vì sao retry=1.

## Kết luận có bằng chứng

1. **Sai alignment trong profile bo tự chế.** Z âm khi đặt bot theo tư thế sử dụng, kèm thông tin roll/pitch đổi chỗ. Stock `CW180_DEG_FLIP` không phù hợp bo này. Đổi sang `{-2,1,3}` (`CW270_DEG`): so với trục output cũ, `(X',Y',Z')=(Y,X,-Z)`. Phép quay có determinant +1, giữ hệ thuận.
2. **Startup/ready của bản portable chưa đúng.** Wrapper cũ reset quaternion về tư thế phẳng và chạy feedback/integral ngay trong thời gian gyro calibration. Với gravity gần ngược hướng giả định, góc có thể quay lớn trong khi gyro bằng 0. `ready` chỉ kiểm tra gyro calibration nên vẫn báo sẵn sàng khi góc đang hội tụ.
3. **Log 2 còn hiệu chỉnh không được ghi.** Dùng góc và gyro đã ghi, tốc độ yaw ước tính từ gyro chỉ khoảng +0,177°/phút. Thêm feedback P theo cấu hình mặc định cho khoảng −0,141°/phút, trong khi yaw ghi tăng +4,362°/phút. Điều này phù hợp có trạng thái integral còn dư; CSV cũ không có integral/quaternion nên chưa thể xác minh trực tiếp giá trị trạng thái hoặc nguồn lịch sử của nó.

Roll log 2 đi qua −180°/+180°; đó là wrap Euler. Không được diễn giải bước hiển thị gần 360° thành robot quay một vòng.

Log 1 và phép tính P khớp dạng hội tụ rất lớn. Test C++ với **gyro chính xác bằng 0**, gia tốc cố định bằng trung bình log 1 và pose đầu lấy từ log tái hiện được dạng lỗi. Đây là mô hình kiểm tra cơ chế, không phải replay raw hoặc phép đo chất lượng firmware mới.

## Thay đổi v0.1.2

- Chờ gyro calibration hoàn thành, dùng gravity đã lọc để dựng roll/pitch ban đầu và đặt mốc yaw tương đối 0. Áp dụng cả khi recalibrate sau nhiều phút.
- Không tích lũy feedback/integral trên tư thế phẳng giả định trong thời gian calibration. Xóa integral khi khởi tạo tư thế mới.
- Chỉ báo ready sau khi cả gyro và tư thế đã được khởi tạo. Đợi nếu gravity không hữu hạn hoặc nằm ngoài 0,8–1,2 g.
- Tách startup policy trong `InavStartup.cpp`; hàm cập nhật INAV được trích xuất và các gains giữ nguyên. Không khóa yaw hoặc tự ép gyro đứng yên về 0 để làm đẹp đồ thị.
- Sửa profile trục bo. Dấu X/Y theo đầu robot còn cần kiểm tra: `CW90_DEG` là lựa chọn khác cách 180° cũng thỏa log đứng yên và việc đổi chỗ. Không tự suy được phía trước robot từ dữ liệu tĩnh.
- CSV thêm quaternion `qw/qx/qy/qz`, integral `ix/iy/iz` (rad/s) và `acc_weight`. Viewer tiếp tục đọc CSV 17 cột cũ.

Các test mới kiểm tra sáu mặt gravity, bắt đầu úp/nghiêng, gravity lỗi, ready, map gyro/acc cùng hệ trục, và recalibrate ở uptime 150 giây **bằng một lần gọi chủ động trong test PC**. Firmware không có lịch tự hiệu chuẩn ở mốc đó. Test native update đối chiếu full hàm INAV vẫn chạy riêng; không tuyên bố toàn pipeline còn byte-equivalent sau thay đổi startup.

## Đồ thị và số liệu

- [Hai log phần cứng](stationary-logs.png).
- [Mô hình regression gyro=0](startup-regression.png) — đường mới là kết quả mô hình, **không phải log phần cứng sau sửa**.
- [Metrics và SHA256 tệp gốc](metrics.json).

Công cụ: `tools/analyze_stationary.py <log1.csv> <log2.csv>`; cần NumPy/Matplotlib. Để tái tạo đồ thị regression sau build host: chạy `build/ttimu_startup_tests` với tham số `reports/2026-11-07-stationary/startup-model.csv`, rồi chạy analyzer. CSV mô hình được sinh tự động và không lưu trong Git.

Nguồn nguyên lý tham khảo: [attitude từ gravity](https://ahrs.readthedocs.io/en/latest/filters/tilt.html). Phần sửa dùng công thức Euler/quaternion cho quy ước INAV của dự án, không đổi sang estimator khác.

## Kiểm tra tiếp trên bo

Nạp v0.1.2 và Ctrl+F5 trang HTML trước khi nối USB. Ghi 60–120 giây từ sau boot, để bot hoàn toàn yên; thêm một lần hiệu chuẩn lại sau ít nhất 30 giây. Ghi hai đoạn riêng: chỉ nghiêng dọc, chỉ nghiêng ngang để chốt dấu trục. Chưa có log sau sửa, chưa xác nhận drift/latency/rung/nhiệt hoặc mức cải thiện trên phần cứng.
