# Log sau sửa khởi tạo — 08/11/2026

Tệp `ttimu-2026-11-07T17-18-31-583Z.csv`, 3556 mẫu, dài 35,717 giây, uptime 44,257–79,974 giây. Định dạng 25 cột của v0.1.2. Người dùng báo cải thiện rõ; pitch/yaw đúng dấu, roll cần đảo dấu.

Log **có chuyển động**, peak gyro khoảng 510,63°/s. Không lấy chênh lệch yaw đầu–cuối cả log làm drift. Theo tiêu chí gyro norm <0,3°/s và liên tục ≥1 giây, có ba đoạn ít chuyển động:

| Uptime đầu–cuối | Thời lượng | Yaw thay đổi |
|---|---:|---:|
| 44,257–45,583 s | 1,326 s | +0,010° |
| 52,255–56,233 s | 3,979 s | +0,037° |
| 71,726–79,974 s | 8,249 s | +0,086° |

Các đoạn này ngắn; ngưỡng gyro không phải bằng chứng độc lập robot hoàn toàn đứng yên. Không ngoại suy thành chất lượng drift dài hạn hoặc phần trăm cải thiện so với log cũ có điều kiện khác.

- Quaternion norm nằm trong 0,99999929–1,00000072.
- `ready=1` suốt log; gaps/late/retries đều 0 theo counter của firmware.
- dt ghi 1000–1016 µs, nhiệt độ 30,25–30,78°C.
- [Số liệu và SHA256](metrics.json). Không chép CSV gốc vào repo.

## Vì sao cải thiện

v0.1.2 sửa hai sai lệch lớn: trục bo và giả định tư thế phẳng lúc khởi động. Sau gyro calibration, wrapper dựng tilt theo gravity đã đo, xóa integral cũ rồi mới chạy cập nhật INAV. Nhờ vậy không phải tự hội tụ qua gần 180° khi bot thực đang yên. Đây là thay đổi chính sách khởi tạo; công thức/gain update INAV giữ nguyên.

## Bản tiếp theo v0.1.3

- Đổi roll đầu ra của riêng profile FC bằng `eulerRollSign=-1`, giữ pitch/yaw và hệ quaternion/gyro/acc gốc. CSV ghi `roll_sign`; rate dùng theo dấu roll có `rollRateRadS()`.
- Dashboard gọn: health/nhiệt độ ở đầu, một đồ thị góc lớn và sáu đồ thị nhỏ. Số và đường dùng cùng màu/thời điểm; tạm dừng không ngừng nhận/ghi dữ liệu.
- Test PC mới mô phỏng 200 giây chuyển động liên tục, đi qua 150 giây và wrap micros mà không gọi hiệu chuẩn: không có tự reset/calib. Lần gọi ở 150 giây trong test cũ là thao tác cố ý của bài test, không nằm trong firmware.

Chưa có log phần cứng v0.1.3 hoặc log chạy dài. Firmware chỉ hiệu chuẩn khi boot hoặc được yêu cầu; thời gian chạy dài vẫn cần đo bias/nhiệt thực tế.
