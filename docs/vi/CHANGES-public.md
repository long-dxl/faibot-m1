# Faibot M1: thay đổi ở bản v0.5

Bản v0.5 chuyển M1 sang hai tay đặt trên một trạm tháo lắp được, đổi sang kẹp gốc của AM-ARM200 và bổ sung đủ hồ sơ để mua linh kiện và lắp ráp. Robot vẫn ở giai đoạn trước EVT: đã có thiết kế và mô phỏng, chưa có robot thật. Mọi số liệu dưới đây là tính toán từ CAD, sẽ đo lại trên bản đầu tiên.

## Trạm hai tay
- Hai tay AM-ARM200 gắn trên khung nhôm chữ U, khoảng cách hai vai 0,475 m.
- Trạm có thể tháo khỏi robot và đặt lên bàn (đế bàn bán riêng, tuỳ chọn). Khi gắn lại, trạm định vị bằng 2 chốt côn, giữ bằng 2 khoá kéo, nối điện qua một giắc D-sub 21W4.
- Robot chỉ cấp nguồn cho tay khi trạm đã khoá chắc. Tháo trạm hoặc mở khoá giữa chừng thì tay ngắt mô-men.
- Camera đầu đặt trên cột của trạm, chúi xuống 50°, chỉnh được.

## Kẹp
- Kẹp gốc của AM-ARM200 (giấy phép Apache-2.0): đầu ngón mở tới 97 mm, 173 g gồm servo và giá camera cổ tay.
- Tầm với từ trục vai tới tâm kẹp khi duỗi ngang: 0,57 m.

## Tay 6+1 (thay đổi mới nhất)
- Mỗi tay có đủ 6 khớp và kẹp, đúng như AM-ARM200 gốc: thêm khớp yaw cổ tay (servo STS3215, ID 21 tay trái, ID 22 tay phải). Chi tiết khoá yaw bỏ đi.
- Tổng 18 servo, 11 cầu chì 3 A (8 cho tay, 3 cho bánh), ngân sách dòng firmware 56,1 A, 204 sợi dây.
- Bộ leader cho gói teleop là AM-ARM200 6+1 (14 servo).
- Đế chạy tối đa 0,40 m/s, quay tối đa 0,25 rad/s, tăng và giảm tốc theo dốc 0,05 m/s². Hệ số chống lật: tĩnh 1,638, động 1,503.
- Tải tính lại theo 6+1: đỉnh 2,0 kg ở tầm 0,25 m; vật khuyến nghị ≤ 0,05 kg. Yaw giới hạn −81…+80° theo quét va chạm CAD, hẹp lại theo góc gập cổ tay.
- Gập cổ tay giới hạn −77,5…+70° (trước +87,5°): quét va chạm CAD trên cẳng tay gốc AM-ARM200 cho thấy vỏ cổ tay chạm cẳng tay từ 73,4°. Tư thế gốc đổi thành khuỷu 102°, gập cổ tay 68°, kẹp chĩa thẳng xuống.
- Ảnh render và mô hình 3D web cập nhật theo bản v05f.

## Trục nâng
- Trục nâng thanh răng, hành trình 0,45 m, có chốt hãm.
- Trục nâng không cần cơ cấu cân bằng phụ. Bánh răng nhỏ hơn (20 răng) giữ tải trên servo trục nâng trong mức cho phép.

## Thông số chính
- 18 servo: 7 STS3095 (pan, vai, khuỷu mỗi tay và trục nâng) và 11 STS3215 (gập, yaw, cuộn cổ tay, kẹp, 3 bánh).
- Pin 12 V 3S4P cell 21700, 172,8 Wh, có BMS, nút dừng khẩn và cầu chì.
- Khối lượng theo CAD khoảng 21,5 kg. Hệ số chống lật 1,64 tĩnh và 1,50 khi chạy với dốc gia tốc của đế.
- Tải tính trên mô hình, chưa đo: đỉnh 2,0 kg ở tầm 0,25 m khi kẹp chúi xuống; liên tục 0 kg ở tầm với tối đa; vật khuyến nghị ≤ 0,05 kg ở tầm 0,45 m khi kẹp ngang. Thời gian chạy pin và tốc độ đế thật: chờ đo trên robot thật.

## Phần mềm và mô phỏng
- SDK Python `faibotic` có hai cấu hình: robot di động và trạm trên bàn. SDK tự nhận cấu hình và từ chối chạy tay khi trạm chưa khoá.
- Mô phỏng MuJoCo và mô hình 3D trên web cập nhật theo trạm hai tay và kẹp mới.

## Hồ sơ lắp ráp
- Bảng phần cứng dạng thẻ, danh sách chi tiết in 3D (vật liệu, khối lượng, giờ in ước tính).
- Hướng dẫn lắp theo thứ tự, có ảnh từng bước, và phiếu kiểm cho từng khâu: nhận linh kiện, trong khi lắp, trước khi giao.
- Bảng giấy phép cho từng phần kế thừa (AM-ARM200, AlohaMini2: Apache-2.0), kèm ghi công.

## Việc còn lại trước bản đầu tiên
- Đo tải, nhiệt servo trục nâng và lực bánh răng trên robot thật.
- Giới hạn góc cuộn cổ tay ở một vùng giá camera chạm vỏ cổ tay.
- Kiểm giắc trạm và mạch dừng khẩn trên phần cứng.
