# Ảnh lắp ráp cho wiki `/wiki/faibot-m1/lap-rap/` và trang sản phẩm M1

Mỗi ảnh có bản 1600 px (`<tên>.webp`) và 800 px (`<tên>-800.webp`). Không chữ trên ảnh. Nền #F5F5F7. Chưa vào `web-assets/manifest.json`.
Bộ đủ ảnh.

## Danh sách ảnh (32/32)

| Ảnh | Nội dung |
|---|---|
| `buoc-NN` | một ảnh mỗi bước trong `cad/faibotic/trinh_tu_lap.yaml` (NN = thứ tự bước). Camera sát nhóm đang lắp, chi tiết mới cam #C2410C nổ ngắn (tối đa 1,4 lần kích thước nhóm), viền cam mảnh ở vị trí lắp, mũi tên cam kết thúc đúng mép vị trí lắp, phần xa mờ dần |
| `tram-u` | trạm chữ U đứng riêng, hai tay 6+1 và camera đầu, màu sản phẩm |
| `tram-gan` | trạm gắn trên đế ngàm của xe nâng, hai chốt côn Ø8 (M1-TR-021) và hai khoá kéo (M1-TR-008) cam; đã ẩn hai tay; lùi camera để thấy cả đế ngàm và hai chốt |
| `estop` | cận nút E-stop 2NC (M1-TR-012b, cam) trên hộp E-stop ở tai trái của trạm; đã ẩn hai tay |
| `cum-<cụm>` | tổng quan cụm: cụm màu sản phẩm, chi tiết của bước cuối cam, phần còn lại xám nhạt |
| `dien-khoang` | hai khoang điện (M1-BS-038, cam) trên sàn đế, màu sản phẩm; đã ẩn khay, cột đỡ và thanh 2040. Dùng cho các bước 1–5, 7, 8 của cụm Điện vì pack, BMS, tấm điện và Pi 5 chưa có chi tiết trong CAD. Camera như `cum-de` (cao 28, phương vị -50) |
| `sau-khi-lap`, `servo-id` | robot đủ bộ (cùng camera); `servo-id` thêm 18 chấm servo trung tính, toạ độ trong `servo-id.json` |

Ảnh có trong lượt này: buoc-01, buoc-02, buoc-03, buoc-04, buoc-05, buoc-06, buoc-07, buoc-08, buoc-09, buoc-10, buoc-11, buoc-12, buoc-13, buoc-14, buoc-15, buoc-16, buoc-17, buoc-18, buoc-19, buoc-20, buoc-21, buoc-22, cum-de, cum-truc-nang, cum-tram, cum-dien, dien-khoang, servo-id, sau-khi-lap, tram-u, tram-gan, estop.

## Ảnh web giữ ảnh cũ

(không có: đủ bộ). `cum-dien` còn trong thư mục nhưng hướng dẫn lắp không dùng (cam bị che).

Bước theo tên trong yaml: 01=de-1, 02=de-2, 03=de-3, 04=de-4, 05=de-5, 06=truc-1, 07=truc-2, 08=truc-3, 09=tram-1, 10=tram-2, 11=tram-3, 12=tram-4, 13=tram-5, 14=tay-1, 15=tay-2, 16=tay-3, 17=tay-4, 18=tay-5, 19=tay-6, 20=dien-1, 21=dien-2, 22=dat-tram.

## Nguồn

- Mã nguồn: commit này (nền: main mới nhất lúc dựng). Mesh: `assembly_pl.py --mesh DIR --lift 0 --pose ready` (máy render, 163 chi tiết, tay 6+1 có servo yaw, khay đế 4 mảnh). Không dùng ảnh cũ 5+1 hay khay B.
- Ảnh bước (`cad/faibotic/render_pdf_buoc.py --style web`): Blender 4.0.2 Cycles CPU trên máy render (không có EGL nên không dùng Workbench), 20 mẫu cộng khử nhiễu OIDN, nền trong suốt rồi ghép lên #F5F5F7 ở Pillow, nét Freestyle xám mỏng. Camera trực giao, một góc cho cả cụm (`web_goc` trong yaml: de [28,-50], truc [20,-50], tram [24,-50], dien [22,-50]); khung 1600 x 1200 vừa khít nhóm đang lắp (cả vị trí nổ), đệm 1,10. Màu: mới cam #C2410C, đã lắp xám nhạt.
- Ảnh sản phẩm (`media/renders-web/src/render_scene.py`, thêm `--sel`, `--el`, `--az`, `--alpha`, `--floor-z`, `--markers`): Cycles 36 mẫu, đèn studio và vật liệu của bộ v05f, PNG RGBA ghép lên #F5F5F7 bằng `cad/faibotic/web_hoan_thien.py`. Chốt côn, khoá kéo, nút E-stop cam #C2410C. Góc camera trong `views.json` (tram-gan: phương vị 22, nâng 9 độ, 85 mm, lấp 0,50; estop: 135, 16 độ, 85 mm; tram-u: 32, 12 độ, 70 mm).
- Dựng cả bộ: `cad/faibotic/lap_rap_web.sh MESH OUT servo-markers.json [buoc|cyc|all]`; vị trí servo: `cad/faibotic/servo_markers.py` (servo có thân trong CAD lấy tâm mesh, servo pan/vai/khuỷu/gập/cuộn đặt tại gốc khớp theo FK tay vì CAD không dựng thân servo).
- `dien-khoang`: `render_scene.py` view `cum`, `--el 28 --az -50`, `--sel` {hi: M1-BS-038_ebay0/1, prod: các chi tiết M1-BS còn lại, hide: M1-ST-011, M1-ST-012 ×4, M1-BS-006}, 36 mẫu, ghép nền bằng `web_hoan_thien.py`.
- PNG gốc ngoài git: máy render.

## sha256

| File | sha256 |
|---|---|
| buoc-01.webp | 8e490f09272ee7484f80c93aad7d67e272631f1ba6172b98e590094e6eee0aec |
| buoc-01-800.webp | 2e74ea9d2dd7d383754571dc849136a0fae21591da0e924cd557911ab38e09d2 |
| buoc-02.webp | 3680342d879040c65742ea8a9c28b4805a1a95526ad327ff9fcae4804fe75a19 |
| buoc-02-800.webp | bdb757ce2bf5ec946d4def08e56a600f02a17b11f48135eca457fe3229d63396 |
| buoc-03.webp | 8e75fc9d8f5901d7936b726b7c9c621681fd0635481d10b044419af496ec8696 |
| buoc-03-800.webp | 3eaabbff8e8df1670e1ac9850c5656e90d85380bb80c49ee1c9caa10656d03ee |
| buoc-04.webp | 0b1aaa82f02deb98887862529e47b725ef7e7c26418013b9b7bdb44197fe9496 |
| buoc-04-800.webp | c7d99b9be7dda99c535c121fea70ae90181b500e74bbac067afcd640bb115514 |
| buoc-05.webp | a50448311481648f3ced27f19b4ba1970104799d8275513cff3946c3209cfe8d |
| buoc-05-800.webp | 58a947a16b1129d386992a519a991bb6eee002141be1c855db8e85c0291e5917 |
| buoc-06.webp | 8d9f40c6b1f438f100a5bd0bbe5ee57014cfc5cfb46dbce5d9953ecd2b400990 |
| buoc-06-800.webp | 4c5e6ed6f39659734d5496116aba272dfa6cce2c0bd3204c4b4f198436f83a29 |
| buoc-07.webp | 276d26d11028fd078ed8c4e31b6fc644c6c4289a1006ea7ab9054b36e6a55483 |
| buoc-07-800.webp | 46239206fb9faacfc64db30943e98d86d885b14eed09ab9b7d23393e23811c1a |
| buoc-08.webp | 81b185441981ce7e6b1e474373e32290a496dc606fa14a22efb7f5fb1a1a99e7 |
| buoc-08-800.webp | b759a770d2fa677f7444ba930b420050423d30fca79409c455d6ffd3d3e3c924 |
| buoc-09.webp | 59d765082ce47f4242c4344a19c536e5156fdd6f3158c0abf4d9c252d3fb27be |
| buoc-09-800.webp | b7b440d061a4e20e4ed6e13ac231c166b9cbf75e0c190de03a56321c5d11286a |
| buoc-10.webp | e1c1a173010a44daf724578201575aeb23786508b316420bbb5d265c7d56b4e1 |
| buoc-10-800.webp | 9c47e2b7d2303dde44465e27283c5e1a0a6be72729d0fd72c85e00b68d8bf72f |
| buoc-11.webp | a5fbbf82d2a1e88a7db982582b5e4b51bbc0a484d8545ff7d81e84d1ba0b8430 |
| buoc-11-800.webp | faa5a03d34eb8d7e58c32d17ea174b5f494a7117c341bd15c6b95f5e65d02c73 |
| buoc-12.webp | ac9f89a1a0c9ef6e2d3c0e9e575e9572152adaa82bcd3f6631ce6140154b59d3 |
| buoc-12-800.webp | d16e3c2355b5bee9431038e0b25eeb58372448368a83802037e3a9d79a9c85d2 |
| buoc-13.webp | 3bb28253a3f0d79a16076e4182788304e702865b84ed06ec71610445f1f43bb0 |
| buoc-13-800.webp | 891b7c217af26f043851cb3a7cbc21a1766a3bcaf2a13746c8a0f90f5a74ea19 |
| buoc-14.webp | 75528472051a83e06f16d3a76ffdbca11373efcde8b703788b2ce6276eeae3f0 |
| buoc-14-800.webp | 29f9aea022db5c912b3cbdfd50e2972fb39194e1ed8de04095e090d4740c371e |
| buoc-15.webp | c0b276c32fd5c61ae689274492def705f8c56323139f2ff89ee76b81dc2aa31a |
| buoc-15-800.webp | 494adabcda966c43f874d8d9a093b28364d05e8963d405310bb5d602045898a5 |
| buoc-16.webp | 3222b75b9edf9b9371e2e086bdb3eaad4eb72a91e96e03dceba80cb870a48ba0 |
| buoc-16-800.webp | 91e2aa89d11fc6ecebfb2ead997046adeffbd72799be927f46e9787c0e3842ec |
| buoc-17.webp | 2d7a31afba4db410ac38a0f5305c799c6793ca2216d208ad11d728c6ab62e36f |
| buoc-17-800.webp | f7d87c3cd01858404f2ae4f455482e2780b42da080bb93179e09078d24ff1438 |
| buoc-18.webp | 286c7c9082ae00bc4a10a36b6c80ecbb2dd3572b22447c33bf8bfcbcedc0efe1 |
| buoc-18-800.webp | 90a521cfc56109195423b85987292b77190ded8fac7c0433496503d45354dd14 |
| buoc-19.webp | 2cced344b4c6e2eec55d1ce466e69c2f05087ae3149a55f4ec11a786972cb6be |
| buoc-19-800.webp | 885828dafecae37d0c121fab63e969fbdf5bbd81522ec555bc843226fd9554f1 |
| buoc-20.webp | 53e8ee1c17992285987f0622d05143c5edf37c9191ba5c32d337f34a87076414 |
| buoc-20-800.webp | 8dbcb9e7a21fc9a0587411b4035c5d28bec957a775aa61561a00d5cf6bedad3f |
| buoc-21.webp | 02262c51688bb388d26d6b287ebd426bbda94d7e1224f9b591ca5a2e11117318 |
| buoc-21-800.webp | 312a412026e856a1a88fad8122597d6a2d6719c508f243af5a0b0a09fba4d127 |
| buoc-22.webp | 69b2c483ba90fd4db61c0e229e726391e521f5cfeb407c2befd79bed3c69aa4c |
| buoc-22-800.webp | 8a90bec41d1064721ae5440632210935fad645f6c956c4c12cb04b5ae05ec490 |
| cum-de.webp | a1bd448cc261bd85570d5b2500d9975d6fc627032099a96945f1a4940e766fe7 |
| cum-de-800.webp | e0c8bbdcca94e6f2d349cc9829841ee34cd05ec967f55292380f47bafff7aa27 |
| cum-truc-nang.webp | 3d16ccd95a19e020e53aefc6b668209be3d13f5bbb12a3813ec1e6025212bab4 |
| cum-truc-nang-800.webp | b840612dbd8287f8a45f912a8f8ac8037333ea6e6cbcb05dbbea74b5d676dad2 |
| cum-tram.webp | affdb68b9e8debce74b131659844709666884ee5157a740ea77cdaf91591d134 |
| cum-tram-800.webp | 258b3d61f4cc044d4e99d2ebf99cbea55c2e084673b4683fae2941fd627da4f0 |
| cum-dien.webp | 4e6be79a1a05dab3c97b264b8d4a8e5722878a7ccf39312e075d606f6405b24e |
| cum-dien-800.webp | f4ff6994364ebfe22f214b213fe9eb980c41cc6a57cc7e243446b2bd2d745d61 |
| dien-khoang.webp | c348ba28d3df35947f0c1bf5a7f24dbb41909369a02e0bc92dcd4218de7014fb |
| dien-khoang-800.webp | 89b0982aa4d8ae860c2dda8246d7b5cd860a70011fb09ebc87b2ce4b45a671c4 |
| servo-id.webp | a5977243672fbb3291b4f288a3e4dcac25ca6d0f7ef6f12d4f1444758f57465d |
| servo-id-800.webp | 57a872e30432414a4f47c9b9375395cc07997b5d0b0a83c8e8232ad34225159d |
| sau-khi-lap.webp | 42f8f3c9c22d1063cfd4aee9c7281beefe9b887bc046aba1bac05820c57d6abc |
| sau-khi-lap-800.webp | f39977ffbfb029f4ca741c6863c1c64a0d2b8ddf3e355c24e62f246a960bad52 |
| tram-u.webp | d9518d4e2a452095ae55d1eec1bea73e116b90f36d8de47db4f91c6d144e03c8 |
| tram-u-800.webp | be5ebc27c8ebfa84a96935372e7f8cde3a0e49289d9079699cd1893f936fd852 |
| tram-gan.webp | d27744537059aa5af4809a67234234f82ffac8d7f3936fa11cac1ae2936b0b80 |
| tram-gan-800.webp | ccb8c02dc6466b8ba97c2f57f4e5c8429a8fcf6534afc235138c0289f2f2a16c |
| estop.webp | ca90c467c1a15f8fb259b85e6ce523f0c574d2e79e628a3c4aff65958afa0008 |
| estop-800.webp | 653a22ebdce737a7e8977b91e3650ca1526b2f814752379614a01e99056d1a96 |
| servo-id.json | cfcd0729d6ec814fe1fb95dafe2ebceff77099e967375ce75109c3f3fc2c6e10 |
