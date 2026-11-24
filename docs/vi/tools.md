# Dụng cụ để lắp Faibot M1

Danh sách này khớp trang "Dụng cụ" trong bản PDF hướng dẫn lắp (`docs/vi/M1_assembly_A5_v0.5.pdf`), có thêm cỡ và loại để chuẩn bị đủ trước khi bắt đầu. Cỡ lục giác suy ra từ ốc trong `media/hardware/bang-oc.csv`. Bản này dựng từ CAD và bảng dây, chưa lắp thử trên robot thật, nên mục nào còn là giả định đều ghi rõ.

Cột "Dùng ở bước" là gợi ý theo hướng dẫn hiện có, chưa kiểm trên robot thật. Số bước ở cột "Dùng ở bước" theo mục của `product/06-lap-rap.md`; tên cụm theo `docs/vi/assembly-guide.md`.

## Dụng cụ siết

| Dụng cụ | Dùng để | Dùng ở bước |
|---|---|---|
| Lục giác 1,5 mm | Ốc M2×12 giữ camera đầu và camera cổ tay | Bước 7 (camera), cụm Trạm |
| Lục giác 2,5 mm | Ốc M3: phần lớn ốc của hai tay, bánh răng trục nâng, khoá yaw, bộ M3×18 | Bước 3 (trục nâng), 4, 5, 6 (tay, kẹp) |
| Lục giác 3 mm | Ốc M4 trong bộ ốc | Bước 1 (đế), 2, 3, 4 |
| Lục giác 4 mm | Ốc M5 của con trượt T, ke góc, gá vai; núm khoá M5 của cột camera | Bước 1, 3, 4, 4A (đế ngàm) |
| Lục giác 5 mm | Vít M6 của đệm tựa thép; vít đầu dù M8×35 của cọc điện | Bước 2 (nguồn), 4A |
| Cờ lê 13 mm | Đai ốc M8 của cọc BAT_F, SERVO+, GND | Bước 2 (nguồn và E-stop) |
| Tua vít Phillips PH0 và PH1 | Ốc M2×12 Phillips của giá camera, vít đầu cắm | Bước 7, 8 (Pi, bo bus) |
| Cờ lê lực hoặc tua vít lực | Đo mô-men siết rồi ghi vào phiếu IPQC. Chưa có số mô-men chốt; số chốt sau lần lắp thử đầu tiên. Không bắt buộc nếu chỉ siết theo cảm giác tay | Mọi bước siết, đặc biệt cọc M8, ốc vào hạt chèn nhiệt |
| Kìm mỏ nhọn hoặc kẹp nhỏ | Giữ đai ốc M3 và con trượt T trong lúc siết | Bước 4, 5 |

## Dụng cụ hàn và ép nhiệt

| Dụng cụ | Dùng để | Dùng ở bước |
|---|---|---|
| Mỏ hàn, kèm đầu ép hạt chèn nhiệt M3 | Ép 8 hạt chèn nhiệt M3 vào cổ tay (4 hạt mỗi tay), và hàn các đầu dây đánh dấu HAN. Dùng đầu mỏ chuyên cho hạt chèn, không dùng đầu hàn thường | Bước 5 (tay), 9 (đi dây) |
| Thiếc hàn, nhựa thông | Hàn dây vào cọc và đầu giắc | Bước 2, 8, 9 |
| Súng khò hoặc bật lửa | Co ống co nhiệt sau khi bấm cos hoặc hàn. Bật lửa chỉ dùng cho ống nhỏ, giữ xa dây và chi tiết in | Bước 2, 9 |

## Dụng cụ bấm dây

| Dụng cụ | Dùng để | Dùng ở bước |
|---|---|---|
| Kìm tuốt dây, kìm cắt cạnh | Cắt dây đủ chiều dài, tuốt vỏ, cắt đầu buộc dây | Bước 2, 8, 9 |
| Kìm bấm cos thuỷ lực hoặc dùng búa, cỡ 8–35 mm² | Cos SC25, SC35 (dây 2 AWG, 4 AWG), SB120 và cos trơn SC10-6 (8 AWG). Kìm bấm thường không đủ lực cho các cỡ này | Bước 2, 4 (hộp cầu chì, cọc 21W4), 9 |
| Kìm bấm cos có bánh cóc, loại bọc nhựa | Cos nhỏ có vỏ cách điện: RV1.25, RV5.5, FDD6.3, PTV1.25 | Bước 4, 8, 9 |
| Kìm bấm đầu cắm chân 2,54 mm (có bánh cóc) | Đầu 5264, Dupont, XH: bấm chân kim loại rồi cài vào vỏ nhựa | Bước 5 (đuôi servo), 8, 9 |
| Kìm bấm đầu cốt kim E7508 | Đầu E7508 cho dây 0,75 mm² | Bước 4, 9 |
| Ống co nhiệt nhiều cỡ | Bọc mối bấm và mối hàn | Bước 2, 9 |

Chân giắc 21W4 bấm hay hàn tuỳ loại chân mua; chưa kiểm bố trí chân thật, đối chiếu giắc thật trước khi hàn hoặc bấm (xem `docs/vi/bom.md`).

## Dụng cụ đo và kiểm

| Dụng cụ | Dùng để | Dùng ở bước |
|---|---|---|
| Đồng hồ vạn năng (thông mạch, điện áp) | Kiểm thông mạch từng dây, kiểm điện áp trước khi cắm Pi và servo | Bước 2, 9, 10 |
| Kẹp dòng DC | Đo dòng khi cấp điện lần đầu | Bước 10 |
| Nguồn bàn 12 V có giới hạn dòng | Cấp điện thử lần đầu, từng nhóm servo. Không dùng pin thật trước khi qua kiểm cuối | Bước 10 |
| Thước kẹp, thước đo góc, ke | Đo khoảng vai, kiểm vuông khung và khe mối trụ | Bước 3, 4 |
| Cân treo | Cân khối lượng robot ở kiểm cuối (FQC) | Sau khi lắp |
| Máy tính hoặc Pi 5 có bo bus servo | Gán ID servo, hiệu chuẩn, quét bus | Bước 0, 10 |

## Vật tư dùng kèm

| Vật tư | Dùng để | Dùng ở bước |
|---|---|---|
| Keo epoxy hai thành phần | Dán các mối nối của trụ nâng và khung xe nâng; để khô hẳn trước khi gắn trạm | Bước 3 |
| Dây rút, kẹp dây | Buộc dây, giữ vòng dịch vụ | Bước 9 |
| Nhãn dán và bút | Dán nhãn servo theo vị trí, nhãn dây | Bước 0, 9 |

## Máy in 3D (nếu tự in chi tiết)

Robot cần 47 chi tiết in theo `product/danh-sach-in.md`, vật liệu PETG. Cỡ cần của máy in lấy từ hình học các tệp STEP chi tiết in của tay và trụ (đo bằng cách chia lưới, bỏ bù lệch):

| Chi tiết | Cỡ lớn nhất, mm [đo từ STEP] |
|---|---|
| Trụ đoạn 4 (O_Main_Assembly_Post4) | khoảng 252 × 219 × 151, chi tiết lớn nhất lấy từ upstream |
| Trụ đoạn 1–3, đế nối trụ, đế tay J1 | không quá 215 × 181 × 145 |
| Chi tiết còn lại của tay và kẹp | không quá 201 × 104 × 40 |
| Khay (M1-ST-010) | 300 × 200 × 5 theo CAD nháp, chưa có STEP [giả định] |
| Sàn đế (M1-BS-037) | đường kính 370 mm, lớn hơn bàn in thông thường, in nhiều mảnh rồi ghép [giả định] |

Yêu cầu tối thiểu: bàn in **256 × 256 mm trở lên, cao 160 mm trở lên** để in trụ đoạn 4 mà không phải xoay chéo. Máy có bàn 220 × 220 mm vẫn in được hầu hết chi tiết nhưng trụ đoạn 4 phải đặt chéo hoặc nghiêng và có thể không vừa. Khay 300 × 200 mm cần bàn rộng ít nhất 300 mm theo một chiều, hoặc cắt khay thành hai mảnh. Cần đầu phun 0,4 mm; thành và infill theo cột "cài đặt" trong `product/danh-sach-in.csv`. Thời gian in ước 128–168 giờ trên một máy [ước], chưa tính vào thời gian lắp (`docs/gio-lap.md`).
