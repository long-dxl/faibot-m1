# Tài liệu công khai cho trang web

Các file trong thư mục này là bản sao nguyên byte từ `docs/vi/` của repo. Không sửa tay: sửa ở `docs/vi/`, rồi chép lại và cập nhật bảng dưới.

Chép lần đầu lúc tái cấu trúc (B1), nền `b5fbd0d`. Chép lại 24/11/2026 từ `docs/vi/` (nền `16a338e`, sau sửa BOM 6+1 và tải liên tục), rồi chép lại lần nữa sau khi cập nhật tài liệu 6+1 (18 servo, tải 2,0 kg, đế 0,25 rad/s). Chép lại 25/11/2026 sau khi thêm khay đế (sàn laptop/Jetson, dây đai, khai báo SDK) và tự hành mặc định tính toán từ xa; sha256 dưới đây là của bản chép lại này. `bom.md` chép lại 25/11/2026 sau khi làm lại BOM công khai (một dòng một hạng mục, cột Link quốc tế / Link VN, bảng ID servo chuyển sang mục "Servo ID" của hướng dẫn lắp ráp) và thêm `bom.en.md`. PDF A5 chưa dựng lại, giữ bản cũ.

| File | Nguồn | sha256 |
|---|---|---|
| `specifications.md` | `docs/vi/specifications.md` | `3f9066f3373f353c0bf12043c06ee7e033eab8708118390de9b2d17e69cc0245` |
| `bom.md` | `docs/vi/bom.md` (bản công khai, không giá) | `6e09c4fca83eb2d673bf5e1e919ccdce4931f51bfd78aed7cc7a25a2f624ed8f` |
| `bom.en.md` | `docs/en/bom.md` (bản tiếng Anh, không giá) | `a4535a54cdd6215be731287ff7a22b3ca5153236738da7b1bc5007f1e11da708` |
| `assembly-guide.md` | `docs/vi/assembly-guide.md` | `dc921503e328f508c83c938a1da61ed9db3b44256c4b063373a983af24420da7` |
| `M1_assembly_A5_v0.5.pdf` | `docs/vi/M1_assembly_A5_v0.5.pdf` | `3162306459ac9b79aa6aae243a3a1a35484613a1b3f87e692c30e4d5a1243abb` |
| `tools.md` | `docs/vi/tools.md` (assembly-guide dẫn tới) | `9ddbd6c47fcbc871d805eb7b0db4cab16c7a567b9216f983b13456bed89ba9fb` |

## Liên kết tương đối

Các file .md giữ nguyên nội dung của `docs/vi/`. Liên kết `../../media/...` trỏ tới `media/` ở gốc repo (từ `web-assets/docs/` cũng tới đúng chỗ đó); `tools.md` được chép kèm. Khi dựng trang web cần chép thêm các file sau (đường dẫn theo repo) và ánh xạ lại liên kết:

`assembly-guide.md` dẫn tới:
- `web-assets/lap-rap/*-800.webp`: `buoc-01` đến `buoc-22`, `cum-de`, `cum-truc-nang`, `cum-tram`, `dien-khoang`, `servo-id`, `sau-khi-lap`, `tram-u`, `tram-gan`, `estop` (mô tả ở `web-assets/lap-rap/README.md`). Từ `web-assets/docs/` đường `../../web-assets/lap-rap/` cũng tới đúng chỗ đó.

`specifications.md` nhắc `docs/gio-lap.md` và `docs/M1-LITE/tai-cong-bo-PL.md` (tên file trong văn bản, không phải liên kết bấm được). Chưa chép các file này vào đây; nếu web cần hiển thị thì phải rà bản công khai trước.

`bom.md` không có liên kết tương đối; các liên kết ngoài giữ nguyên.

Kiểm trước khi công khai: `python3 scripts/public_scan.py <thư mục có docs/…>` (luật về nơi bán chỉ được nới cho `docs/vi/bom.md` và bản sao nguyên byte `web-assets/docs/bom.md`), và `bom.md` không chứa ký hiệu tiền tệ hay cột giá.
