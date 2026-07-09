# Grid Swarm Robot — Swarm AMR phân loại hàng theo màu trên lưới mịn 50×50 (ARGoS3)

Mô phỏng nhà kho thông minh kiểu Amazon Kiva / Geek+: **10 robot vi sai
(foot-bot)** di chuyển trên **bản đồ lưới rời rạc 50×50 ô, mỗi ô 0.2 m
× 0.2 m** (đúng đường kính thân foot-bot 0.17 m + 0.03 m biên an toàn —
ô chỉ vừa khít một robot). Robot định vị bằng "camera gầm đọc mã QR
sàn" (xóa sạch odometry drift), nhận hộp hàng 3 màu từ 3 băng chuyền và
giao vào các ô ngăn xếp (vật cản cứng) đang yêu cầu đúng màu. Điều phối
**hoàn toàn phi tập trung**: đặt chỗ ô theo tick + ưu tiên bất đối xứng
qua Range-and-Bearing, không có máy chủ trung tâm áp đặt đường đi.

## 1. Build & Chạy

```bash
cd /home/dvt/argos && ./run.sh          # mở container argos3

cd ~/workspace/grid-swarm-robot
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

cd ~/workspace/grid-swarm-robot          # .argos dùng đường dẫn tương đối
argos3 -c experiments/grid_swarm.argos   # GUI mở ở trạng thái tạm dừng, bấm ▶
```

Headless đo hiệu năng:
```bash
sed -e 's|length="0"|length="1200"|' -e '/<visualization>/,/<\/visualization>/d' \
    experiments/grid_swarm.argos > /tmp/gs.argos
argos3 -c /tmp/gs.argos 2>&1 | grep -E "Delivered total|Emergencies|VA CHAM"
```
Headless chạy rất nhanh: ~7000 tick mô phỏng/giây thực (1200 tick ≈ 0.15 s).

**Kết quả đo thực tế** (seed 42, 20 phút mô phỏng = 12 000 tick, bố cục
hiện tại — kệ dồn lên trên/băng chuyền dồn xuống đáy):
**32 chuyến giao** (Đỏ 12 / Lá 13 / Dương 7), **10 lần khẩn cấp pin**
(quãng đường dock↔kệ dài hơn bố cục cũ nên emergency kích hoạt thường
xuyên hơn — đúng cơ chế thiết kế), **29 lần dạt làn cục bộ**, hàng đợi
băng chuyền giữ ổn định ở mức tối đa 9/9 hộp (3 băng × 3 hộp) hầu hết
thời gian, quãng đường tổng 860 m, chốt QR ~4300 lần. Xem mục 8 về 17
lần "va chạm biên" ghi nhận được và lý do.

## 2. Bố cục nhà kho (nguồn chân lý: `grid_layout.h`)

Công thức ánh xạ tọa độ liên tục (x,y) ⇄ ma trận nguyên (Row,Col) —
**Row bám trục X, Col bám trục Y** (khác quy ước row~y/col~x thường
gặp, đây là lựa chọn tường minh của bố cục này):
```
Row = (int)((x + 5.0) / 0.2)        x_tâm = -5.0 + (Row + 0.5) * 0.2
Col = (int)((y + 5.0) / 0.2)        y_tâm = -5.0 + (Col + 0.5) * 0.2
```

3 hàng ngăn xếp dồn lên khu vực **phía trên** bản đồ, chỉ cách nhau
**4 Row** (3 Row hành lang giữa 2 kệ liền nhau); 3 băng chuyền dồn
xuống **đáy**; 10 dock giữ nguyên vị trí trung tâm bản đồ:
```
Col=0 (y=-4.9)                                        Col=49 (y=+4.9)

Row=38 (x=+2.7)  ████████████████████████████████  ← ngăn xếp 1 (VẬT CẢN CỨNG)
Row=39..41                hành lang 3 hàng trống
Row=42 (x=+3.5)  ████████████████████████████████  ← ngăn xếp 2 (VẬT CẢN CỨNG)
Row=43..45                hành lang 3 hàng trống
Row=46 (x=+4.3)  ████████████████████████████████  ← ngăn xếp 3 (VẬT CẢN CỨNG)
Row=47..49                đệm sát tường Bắc

Row=22..26   dock TÂY (5 ô) .......................... dock ĐÔNG (5 ô)
Row=3..37                    lưu thông tự do rộng rãi

Row=2 (x=-4.5), Col=12/25/38: 3 miệng băng chuyền (đáy bản đồ)
```

- **3 hàng ngăn xếp = vật cản vật lý cứng thật** (`<box movable="false"
  size="0.2,6.0,0.5">` đè khít trong `.argos`, khớp `CELL_OBSTACLE`
  trong ma trận logic) — robot không thể và không được lập kế hoạch đi
  xuyên qua. *Lưu ý:* kích thước box đảo lại thành `0.2,6.0,0.5` (không
  phải `6.0,0.2,0.5` như số liệu gốc) để khớp đúng ngữ nghĩa "1 Row cố
  định, Col trải dài" — nếu giữ nguyên thứ tự gốc, vật cản sẽ xoay
  90° so với ma trận logic.
- **Giao hàng qua "ô mặt kệ"**: vì ô ngăn xếp là vật cản không vào
  được, robot giao hàng bằng cách đứng ở ô lưu thông trống liền kề
  (`StackFaceCell`, phía trước **hoặc** sau dải vật cản) — chọn mặt
  gần hơn theo Manhattan, `DeliverBox()` chấp nhận cả hai mặt.
- **10 dock sạc chia đều 2 biên** (Col=0 và Col=49, Row 22-26), ẩn danh
  hoàn toàn (`DockFree`/`NearestFreeDock` không phân biệt ID).
- **Băng chuyền trưng bày tối đa 3 hộp cùng lúc** (hàng đợi hiển thị,
  không bắt buộc lấy đúng thứ tự): còn chỗ trống thì tự sinh thêm hộp
  màu ngẫu nhiên theo chu kỳ `box_respawn_min/max`; hết chỗ thì dừng
  sinh tới khi có robot lấy bớt.

## 3. Kiến trúc mã nguồn

| File | Vai trò |
|---|---|
| `grid_layout.h` | Hằng số lưới, công thức ánh xạ, vật cản/dock/băng chuyền, giao thức RAB |
| `footbot_grid.{h,cpp}` | FSM: `IDLE→TO_PICKUP→PICKING→DELIVERING→DROPPING→RETURNING/RESTING`, `EMERGENCY_CHARGE`; pin trễ 20%/70% |
| `footbot_grid_nav.cpp` | Dead-reckoning + chốt QR sàn, A* cấm `CELL_OBSTACLE`, lái tỷ lệ V_Base 10/3 cm/s |
| `footbot_grid_traffic.cpp` | Đặt chỗ `Grid[Tick][(Row,Col)]=RobotID`, ưu tiên bất đối xứng, **dạt làn cục bộ 3 bước** |
| `grid_loop_functions.{h,cpp}` | Bảng đen: ma trận đặt chỗ theo tick, bảng việc (băng chuyền hàng đợi 3 hộp), pin định lượng, thống kê |
| `grid_floor_render.cpp` | `GetFloorColor()` — **nguồn cảm biến sàn thật**: lưới 0.2 m, đĩa QR, viền loại ô |
| `grid_qt_user_functions.cpp` | `DrawInWorld()` (tương đương *PostDraw*): lớp phủ che đĩa QR (chỉ hiển thị), kẻ lưới 3D, hologram màu, nhãn robot |

> Controller gọi trực tiếp API bảng đen của loop functions (phụ thuộc
> 2 chiều) nên biên dịch chung **một `.so`** (`libgrid_loop_functions`).

## 4. Đặt chỗ theo tick + dạt làn cục bộ 3 bước (footbot_grid_traffic.cpp)

Ma trận chiếm dụng đúng cấu trúc yêu cầu:
`std::map<Tick, std::map<(Row,Col), RobotID>> m_sGridReservations`
(thêm kiểu giá trị `RobotID` cho tầng trong so với gợi ý `std::set`
gốc, vì set không có chỗ lưu ID mà biểu thức "=Robot_ID" đòi hỏi), có
dọn rác tick quá khứ mỗi `PostStep`. Robot phải đặt chỗ **toàn bộ cửa
sổ tick** trước khi bước sang ô kế tiếp; `CellReserver()` là hàm tra
cứu tiện ích cho A*/dạt làn, không phải kho lưu trữ.

**Ưu tiên bất đối xứng** (hàm thuần của FSM, không nhấp nháy giữa
đường): 1 = `EMERGENCY_CHARGE`, 2 = `DELIVERING`, 3 = còn lại; hòa →
Id nhỏ thắng. Robot thắng giữ nguyên V_Base=10 cm/s, không bẻ lái.

**Robot thua** thực hiện đúng 3 bước (không gọi A* toàn cục):
1. Rẽ ngang sang 1 trong 2 ô vuông góc còn trống/chưa đặt chỗ
2. Tịnh tiến song song 1 ô, vượt qua robot đối diện
3. Rẽ ngược về làn cũ, 1 ô sau vị trí từng bị chặn — buộc vẽ lại A*
   từ đây để tiếp tục lộ trình gốc

Cả 3 bước được đặt chỗ ngay khi bắt đầu (không chỉ bước 1) để giảm rủi
ro robot thứ ba chen vào giữa lúc 2 bên đang sát nhau. Chỉ khi **cả
hai** ô vuông góc đều nghẽn, robot mới `TRAFFIC_YIELDING` (đứng im 1-2
tick rồi thử lại). Lưới an toàn cuối cho kẹt bệnh lý (hiếm khi kích
hoạt): 3 s → A* né ô bị đặt chỗ quanh mình, 8 s → né luôn vị trí hàng
xóm RAB.

## 5. Định vị "QR sàn" (footbot_grid_nav.cpp)

Giữa hai tâm ô, robot chỉ dùng odometry encoder (tích phân trung điểm,
trôi dần). **Phát hiện quan trọng khi kiểm thử**: foot-bot thật trong
ARGoS có đúng 8 cảm biến sàn gắn cố định trên vòng bán kính ~0.08 m
quanh tâm thân robot (hằng số phần cứng) — nếu vẽ đĩa QR bán kính
đúng 0.02 m như số liệu "vùng tiếp cận" gốc, đĩa nằm hoàn toàn *bên
trong* vòng cảm biến và **không cảm biến nào chạm tới được** (kiểm
chứng thực nghiệm: 0 lần chốt QR dù robot đi qua chính xác tâm ô). Đã
tách hai khái niệm:
- **Kích thước vật lý đĩa QR = 0.085 m** (đủ lớn cho phần cứng "thấy").
- **Ngưỡng tin cậy vị trí ≤ 0.02 m** (`m_fQrSnapRadius`, đúng số đề
  bài) — chỉ THỰC THI gán cưỡng bức (x,y,góc) khi ước lượng đã đủ gần
  tâm ô, tách bạch với việc cảm biến nhìn thấy đĩa hay không.

Sau khi sửa: chốt QR ~2 500-4 300 lần/20 phút, tái định vị cứu hộ (trôi
>0.6 m chưa gặp QR) gần như bằng 0.

**Ẩn đĩa QR khỏi hiển thị (chỉ về mặt hình ảnh):** đĩa đen 0.085 m rất
dày đặc (gần như mọi ô đều có) nên nhìn khá chói/rối mắt. Cảm biến sàn
gọi thẳng `CFloorEntity::GetColorAtPoint()` → `GetFloorColor()` — một
đường gọi hoàn toàn tách biệt với những gì được vẽ trong lớp 3D
`DrawInWorld()` của `grid_qt_user_functions.cpp`. Vì vậy có thể phủ một
lớp phẳng cùng màu nền (trắng/xanh dock/xám băng chuyền/be mặt kệ) ở độ
cao thấp hơn lưới kẻ ngay trong `DrawInWorld()` để che khuất đĩa đen
trong tầm mắt người xem, mà **hàm `GetFloorColor()` dùng cho định vị
không hề bị đụng tới** — dùng vài mảng phẳng lớn (nền trắng toàn arena +
2 dải dock + từng ô băng chuyền + các ô mặt kệ đang có yêu cầu) thay vì
phủ ~2400 hình tròn riêng lẻ, rẻ hơn nhiều cho GPU.

## 6. Pin định lượng (hysteresis 20% / 70%)

Đơn vị tuyệt đối `full_charge=10000` (đúng `starting_capacity` đề bài
— đây chính là tên thuộc tính thật của ARGoS, không phải một field
riêng). *Model `time_motion` cài sẵn của ARGoS bị lỗi NaN trong
`ToAngleAxis` khi robot xoay* nên xả pin được tính thủ công ở loop
functions mỗi tick, dùng đúng 2 hệ số đề bài:

```
xả (chỉ khi đang DI CHUYỂN, đo bằng quãng đường tick thực > 0):
  Δ = discharging_factor(0.05) × BASE_DISCHARGE_RATE(16.0) = 0.8 /tick
sạc (chỉ khi ĐỨNG YÊN trong ô dock, suy ra vận tốc bánh ≈ 0):
  Δ = charging_factor(2.0) × BASE_CHARGE_RATE(1.0) = 2.0 /tick
```
BASE_* hiệu chỉnh để ở giá trị mặc định, robot di chuyển LIÊN TỤC (không
ngừng) cạn 100%→20% sau ~10 000 tick = **16.7 phút** (trong khoảng 15-20
phút yêu cầu); sạc 20%→70% trong ~2 500 tick ≈ 4.2 phút.

- **< 20%**: `STATE_EMERGENCY_CHARGE` — hủy/bàn giao nhiệm vụ, ưu tiên 1,
  tìm dock trống gần nhất; chỉ giải phóng về `IDLE` khi **đã ở dock và**
  pin ≥ 70%.
- **AfterTaskDone giữ biên an toàn 25%** (không phải đúng 20%) trước khi
  nhận việc mới — đây là lý do trong bài đo 20 phút, robot **chủ động**
  quay dock sạc trước khi kịp chạm mốc khẩn cấp thật (`khan cap=0`),
  một hành vi quản lý năng lượng hợp lý chứ không phải lỗi; cơ chế khẩn
  cấp vẫn kích hoạt đúng khi robot bị kẹt / dock đầy trong lúc trên
  đường về (đã quan sát được trong các lần chạy dài hơn).
- Idle > 100 tick (`idle_rest_ticks`) → `STATE_RESTING` (ngủ đông tại
  dock, dò việc thưa hơn để tiết kiệm năng lượng).

## 7. Đọc hiểu hình ảnh GUI

| Hình ảnh | Ý nghĩa |
|---|---|
| Sàn trắng sạch, chỉ còn lưới kẻ mảnh | Đĩa QR đã bị che (chỉ ẩn hình ảnh — định vị vẫn hoạt động, xem mục 5) |
| Nền xanh nhạt (2 dải giữa bản đồ) | Khu vực 10 ô dock sạc (Row 22-26, 2 biên) |
| Nền xám (3 ô rời rạc ở đáy) | 3 miệng băng chuyền |
| Nền be (đổi động) | Ô mặt kệ đang có yêu cầu màu (xuất hiện/biến mất theo demand) |
| Tháp hộp xếp chồng trên băng chuyền (tối đa 3 tầng) | Hàng đợi hộp đang trưng bày chờ bốc |
| Khối "hologram" lơ lửng cạnh dải vật cản (3 hàng trên cùng) | Ô mặt kệ đang yêu cầu màu đó |
| Khối màu nhỏ trên lưng robot + LED | Đang chở hộp |
| Nhãn `fb3 87% DELIVERING *` | Id, %pin, trạng thái; `*` = đang dạt làn/yield |
| LED cam | IDLE/RESTING (đậu dock) — LED tím: EMERGENCY_CHARGE |

## 8. Giới hạn vật lý đã biết: biên an toàn 0.03 m

CELL_SIZE=0.2m đúng bằng đường kính robot (0.17m) + 0.03m theo yêu
cầu — biên an toàn giữa hai robot ở hai làn kề nhau (tâm cách nhau
đúng 0.2m) chỉ còn 0.03m. Trong 20 phút mô phỏng (bố cục hiện tại), đo
được 17 sự kiện "va chạm" (khoảng cách thân < 0.17m) — **tất cả đều
nằm trong khoảng 0.169-0.170m**, tức chỉ chạm đúng biên lý thuyết,
không lún sâu. Đã
thử tăng độ bền (đặt chỗ trước cả 3 bước dạt làn thay vì chỉ bước 1)
nhưng số lượng không đổi đáng kể — xác nhận đây không phải lỗi logic
đặt chỗ (hệ thống đúng-chức-năng chưa từng cho 2 robot cùng chiếm 1 ô),
mà là hệ quả vật lý cố hữu: bất kỳ lúc nào hai robot đi qua nhau ở 2
làn kề (không chỉ lúc dạt làn), biên 3cm quá mỏng so với dung sai
chuyển động liên tục thực (quán tính, làm tròn tick) của vật lý
dynamics2d. Nếu cần an toàn tuyệt đối, tăng `CELL_SIZE` lên 0.22-0.25m
trong `grid_layout.h` (và box vật cản/spawn tương ứng) sẽ cho biên
thoải mái hơn.

## 9. Tinh chỉnh (experiments/grid_swarm.argos)

| Tham số | Ý nghĩa |
|---|---|
| `wheel cruise_speed_cms/pivot_speed_cms` | V_Base chạy thẳng (10) / rẽ 90° (3) |
| `wheel qr_snap_radius` | Ngưỡng tin cậy vị trí trước khi chốt QR (0.02m) |
| `battery low_threshold/leave_threshold` | Hysteresis 20%/70% |
| `grid discharging_factor/charging_factor` | Hệ số pin (đúng số đề bài 0.05/2.0) |
| `grid max_active_demands/demand_period` | Số yêu cầu màu mở đồng thời |
| `grid box_respawn_min/max` | Nhịp sinh hộp mới khi băng chuyền còn chỗ (< 3 hộp) |

Đổi hình học kho: sửa `grid_layout.h` **và** vị trí spawn/box vật cản
trong `.argos` cho khớp (cả hai phải nhất quán, không có kiểm tra
runtime chéo file).
