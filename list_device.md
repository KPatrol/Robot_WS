BẢNG TỔNG HỢP PHẦN CỨNG HỆ THỐNG ROBOT K-PATROL
1️⃣ Khối Robot di chuyển (K-Patrol Bot)
STT	Tên phần cứng	Số lượng	Công dụng	Ghi chú
1	Khung robot 4 bánh Mecanum	1 bộ	Kết cấu cơ khí cho robot tuần tra	Nhôm / thép / in 3D
2	Bánh xe Mecanum	4	Di chuyển đa hướng (tiến/lùi/ngang/xoay)	Trái – phải đối xứng
3	Động cơ DC có Encoder (12V)	4	Truyền động chính cho robot	Encoder 2 kênh A/B
4	Encoder tích hợp động cơ	4	Phản hồi tốc độ & quãng đường	Dùng cho điều khiển chính xác
2️⃣ Khối điều khiển động cơ
STT	Tên phần cứng	Số lượng	Công dụng	Ghi chú
5	Driver BTS7960	4	Điều khiển đảo chiều & tốc độ động cơ	Công suất cao, 43A
6	Nguồn DC 12V	1	Cấp nguồn cho động cơ	Pin / Adapter
7	Dây nguồn motor (Đỏ/Trắng)	4 bộ	Cấp nguồn động cơ	Đỏ: +12V, Trắng: GND
8	Dây tín hiệu PWM/EN	Nhiều	Điều khiển driver	Từ ESP32 S3
3️⃣ Khối vi điều khiển & xử lý
STT	Tên phần cứng	Số lượng	Công dụng	Ghi chú
9	ESP32-S3 Dev Module	1	Điều khiển driver motor, giao tiếp Pi	Trung tâm điều khiển
10	ESP32 Dev Module (WROOM)	1	Đọc encoder 4 động cơ	Giảm tải chân GPIO
11	Giao tiếp UART (TX/RX)	1 kênh	Truyền dữ liệu giữa 2 ESP32	Chuẩn Serial
12	Dây USB A-C	1	Kết nối ESP32-S3 với Raspberry Pi	/dev/ttyACM0
4️⃣ Khối Encoder (chi tiết dây)
STT	Thành phần	Màu dây	Công dụng
13	Encoder nguồn	Đỏ	+5V / +3.3V cho encoder
14	Encoder GND	Đen	Mass encoder
15	Encoder Channel A	Vàng	Xung A
16	Encoder Channel B	Xanh lá	Xung B
17	Encoder Index / NC	Xanh dương	Không dùng
18	Encoder Shield / NC	Trắng	Không dùng

👉 Encoder KHÔNG cần cấp nguồn để động cơ quay, nhưng cần cấp nguồn để đọc tốc độ.

5️⃣ Khối máy tính & điều khiển cấp cao
STT	Tên phần cứng	Số lượng	Công dụng	Ghi chú
19	Raspberry Pi	1	Điều khiển cấp cao, AI, Web	Pi 4 8GB
20	Thẻ nhớ SD	1	Lưu hệ điều hành & dữ liệu	32GB
21	Nguồn Raspberry Pi	1	Cấp nguồn cho Pi	5V – 3A
6️⃣ Khối giao tiếp & điều khiển từ xa
STT	Tên phần cứng	Số lượng	Công dụng	Ghi chú
22	Tay điều khiển RC (SBUS/PPM)	1	Điều khiển thủ công robot	Tùy chọn (Chưa có, đang cần thiết kế)
23	Module thu RC	1	Nhận tín hiệu tay điều khiển	Gắn ESP32 (Chưa có, đnag cần thiết kế0
24	WiFi	Tích hợp	Điều khiển từ Web/Mobile	Đã có LAN dùng trên thiết bị phát mạng cho Pi 4 sử dụng để điều khiển tất cả.
7️⃣ Khối mở rộng IoT & AI (đề xuất)
STT	Tên phần cứng	Số lượng	Công dụng	Trạng thái
25	Camera	1–2	Quan sát & AI vision	Mở rộng

🧠 Tổng kết kiến trúc phần cứng

ESP32-S3:
👉 Điều khiển driver BTS7960 + giao tiếp Raspberry Pi

ESP32 Dev Module:
👉 Đọc encoder 4 động cơ, gửi tốc độ qua UART

Raspberry Pi:
👉 Điều khiển cấp cao, Web, Mobile, AI, IoT, và ESP32 -S3 và ESP32 Dev Module

Driver BTS7960:
👉 Điều khiển độc lập từng bánh Mecanum