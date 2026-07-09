# Grid Swarm Robot — Swarm AMR phân loại hàng theo màu trên lưới 20×20 (ARGoS3)

Mô phỏng nhà kho thông minh kiểu Amazon Kiva / Geek+: **10 robot vi sai
(foot-bot)** di chuyển trên **bản đồ lưới rời rạc 20×20 ô (1 m × 1 m)**,
định vị bằng "camera gầm đọc mã QR sàn" (xóa sạch odometry drift), nhận
hộp hàng 3 màu từ 3 băng chuyền và nạp vào các ô ngăn xếp đang yêu cầu
đúng màu. Toàn bộ điều phối là **phi tập trung**: đặt chỗ ô + trao đổi
mức ưu tiên cục bộ qua Range-and-Bearing, không có máy chủ trung tâm.

## 1. Build & Chạy

```bash
# Từ host: mở container (giống các project khác trong workspace)
cd /home/dvt/argos && ./run.sh

# Build (lần đầu):
cd ~/workspace/grid-swarm-robot
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# Chạy GUI (từ thư mục gốc project — .argos dùng đường dẫn tương đối):
cd ~/workspace/grid-swarm-robot
argos3 -c experiments/grid_swarm.argos
```

Bấm ▶ Play trên toolbar (mô phỏng mở ở trạng thái tạm dừng).

Chạy headless đo hiệu năng (ví dụ 2400 s):

```bash
sed -e 's|length="0"|length="2400"|' \
    -e '/<visualization>/,/<\/visualization>/d' \
    experiments/grid_swarm.argos > /tmp/gs.argos
argos3 -c /tmp/gs.argos 2>&1 | grep -E "Delivered total|Emergencies|Conflicts"
```

Kết quả tham chiếu (seed 42, 2400 s): **60 chuyến giao** (Đỏ 25 / Lá 20 /
Dương 15), **4 lần khẩn cấp pin** đều tự về dock trống sạc rồi làm tiếp,
**12 lần dạt làn nhường ưu tiên**, **0 va chạm** trên tổng 3 716 m di chuyển.

## 2. Bố cục nhà kho (nguồn chân lý: `grid_layout.h`)

```
hàng 19   . . . . S S S S S S S S S S S S . . . .   ← ngăn xếp 3
hàng 17-18   ═════ HÀNH LANG 2 Ô (hai chiều) ═════
hàng 16   . . . . S S S S S S S S S S S S . . . .   ← ngăn xếp 2
hàng 14-15   ═════ HÀNH LANG 2 Ô (hai chiều) ═════
hàng 13   . . . . S S S S S S S S S S S S . . . .   ← ngăn xếp 1
hàng 12   .        (sàn lưu thông tự do)        .
 ...      D ← dock trái (cột 0)   dock phải (cột 19) → D
hàng  0   . . . C . . . . . C . . . . . C . . . .   ← 3 băng chuyền
```

- **10 dock sạc chia đều 2 biên** (cột 0 và cột 19, các hàng 3/6/9/12/15).
  Robot sinh ra ngay trên dock, 5 con mỗi bên, quay mặt vào trung tâm —
  khi vận hành đội xe tràn từ hai hướng đối đỉnh vào giữa kho.
- **Dock ẩn danh**: dock "trống" ⇔ không ai đặt chỗ ô đó + không thân
  robot nào đứng trong ô. Bất kỳ robot nào cũng cắm được bất kỳ dock
  trống nào — ai đến trước dùng trước (`NearestFreeDock` / `DockFree`).
- **Hành lang giữa các hàng ngăn xếp rộng đúng 2 ô** → hai robot từ hai
  phía có thể đi song song hoặc đối đầu rồi né cục bộ.

## 3. Kiến trúc mã nguồn

| File | Vai trò |
|---|---|
| `grid_layout.h` | Hằng số lưới, **công thức ánh xạ (x,y) ⇄ (Row,Col)**, vị trí dock/băng chuyền/ngăn xếp, giao thức bản tin RAB 16 byte |
| `controllers/footbot_grid/footbot_grid.{h,cpp}` | FSM nhiệm vụ, **chu kỳ pin trễ 20 % / 70 %**, tự nhận việc trên bảng đen |
| `controllers/footbot_grid/footbot_grid_nav.cpp` | Dead-reckoning encoder + **chốt QR sàn xóa drift**, A* 4 hướng phạt cua, **bộ lái tỷ lệ bám tâm ô** |
| `controllers/footbot_grid/footbot_grid_traffic.cpp` | Đặt chỗ ô trước khi đi, quét xung đột RAB, **nhường đường theo ưu tiên** (đứng im / dạt làn / lùi) |
| `loop_functions/grid_loop_functions/grid_loop_functions.{h,cpp}` | "Bảng đen": ma trận chiếm dụng `Grid[Row][Col]`+cửa sổ tick, bảng việc, sạc dock, sinh hộp/yêu cầu, thống kê |
| `loop_functions/grid_loop_functions/grid_floor_render.cpp` | Vẽ sàn: lưới, **đĩa QR đen tâm ô**, màu trạm |
| `loop_functions/grid_loop_functions/grid_qt_user_functions.cpp` | Vẽ debug 3D (tương đương *PostDraw*: ARGoS gọi là `DrawInWorld`): kẻ lưới, hộp hàng, nhãn pin/trạng thái |
| `experiments/grid_swarm.argos` | Kịch bản: 10 foot-bot trên 10 dock, pin lệch pha 0.65–1.0, camera trên đỉnh |

> Controller gọi trực tiếp API bảng đen của loop functions (phụ thuộc
> 2 chiều) nên cả hai được **biên dịch vào một `.so` duy nhất**
> (`libgrid_loop_functions`) — file nguồn vẫn tách thư mục rõ ràng.

## 4. Ba cơ chế lõi

### 4.1 Ánh xạ tọa độ liên tục ⇄ ma trận (grid_layout.h)
```
Col = floor((x + 10) / 1.0)        x_tâm = -10 + (Col + 0.5)
Row = floor((y + 10) / 1.0)        y_tâm = -10 + (Row + 0.5)
```
Arena đặt tâm ở gốc nên tịnh tiến +10 m về góc Tây-Nam rồi lấy phần
nguyên; chiều ngược cộng 0.5 để ra đúng **hồng tâm ô** — nơi dán QR.

### 4.2 Định vị "camera gầm đọc QR" (footbot_grid_nav.cpp)
Giữa hai tâm ô, robot chỉ dùng **odometry encoder** (tích phân trung
điểm — sai số tích lũy như robot thật). Mỗi tâm ô có **đĩa đen r=10 cm**;
khi 4 cảm biến sàn đọc được độ xám < 0.08 và ước lượng đang gần tâm ô
(< 35 cm), robot "giải mã QR" (đọc positioning một lần) và **ghi đè cả
(x, y, góc) — drift về 0 tuyệt đối**, nên đội hình luôn thẳng làn.
Nếu chạy 3 m không gặp QR nào (lạc làn) → tái định vị cứu hộ và đếm
sự kiện (`cuu ho lac` trong báo cáo; bằng 0 là chuẩn).

### 4.3 Nhường đường theo quyền ưu tiên (footbot_grid_traffic.cpp)
Mỗi tick robot phát RAB 16 byte: `[id, ưu tiên, ô hiện tại, ô sắp vào,
cờ chở hàng/nhường, %pin]`. Trước khi bước sang ô kế tiếp robot phải
**đặt chỗ** ô đó trên bảng chiếm dụng; gặp xung đột thì so quyền:

1. **Khẩn cấp** — pin < 20 % đang tìm trạm sạc
2. **Đang chở hộp hàng** đi giao
3. **Chạy không tải** / về dock nghỉ (hòa → Id nhỏ thắng)

Kẻ thua *nhả đặt chỗ*, rồi hoặc **đứng im**, hoặc **dạt sang ô trống
vuông góc** (làn thứ hai của hành lang), hoặc **lùi lại ô vừa rời**;
đối thủ đi khỏi ô tranh chấp thì tự vẽ lại lộ trình. Kẹt lâu leo thang:
3 s → A* né các ô bị đặt chỗ, 8 s → né luôn vị trí hàng xóm (tự chuyển
sang hành lang khác).

## 5. Chu kỳ pin trễ (hysteresis 20 % / 70 %)

- Xả **tuyến tính theo thời gian di chuyển** mỗi tick:
  `Δ = time_factor + pos_factor·|quãng đường|` (mặc định 0.001 %/tick +
  0.5 %/m). *Ghi chú:* model `time_motion` cài sẵn của ARGoS dính lỗi
  NaN trong `ToAngleAxis` nên cùng công thức được tính tại loop
  functions; entity pin vẫn là kho lưu và cảm biến `battery` đọc như thường.
- **< 20 %**: hủy/bàn giao nhiệm vụ ngay — chưa bốc hộp thì trả cả hộp
  lẫn yêu cầu về bảng đen; đã ôm hộp thì giữ quyền giao, sạc xong đi
  giao nốt. Chuyển ưu tiên 1, tìm dock trống gần nhất ở một trong hai biên.
- **≥ 70 %** mới được rời dock nhận việc (đậu chờ vẫn sạc tiếp lên 100 %).
- Hết việc robot **không chạy lòng vòng**: tự về dock trống gần nhất
  nghỉ; đang trên đường về mà có việc mới + đủ pin thì quay xe luôn.

## 6. Đọc hiểu hình ảnh GUI

| Hình ảnh | Ý nghĩa |
|---|---|
| Đĩa đen giữa mọi ô | "Mã QR sàn" để robot xóa drift |
| 10 ô xanh nhạt hai biên | Dock sạc; nhãn robot đổi màu theo mức pin |
| Ô có vành khăn màu ở đáy | Băng chuyền đang có hộp chờ (kèm khối hộp 3D) |
| Vành khăn màu trên 3 hàng ngăn xếp | Ô "đang yêu cầu 1 hộp màu này" (khối lơ lửng + số đơn đã nạp) |
| Khối màu trên lưng robot + LED | Hộp đang chở |
| Nhãn `fbN 87% DI-GIAO` | Id, %pin, trạng thái; thêm `*` khi đang nhường đường |
| LED cam / tím | Đang sạc / khẩn cấp tìm dock |

Trạng thái: `SAC, CHO, DI-LAY, BOC, DI-GIAO, HA, VE-NGHI, KHAN-CAP`.

## 7. Tinh chỉnh nhanh (experiments/grid_swarm.argos)

| Tham số | Ý nghĩa |
|---|---|
| `wheel max_speed_cms / kp_heading` | Tốc độ hành trình, độ gắt bẻ lái P |
| `battery low_threshold / leave_threshold` | Hai ngưỡng trễ 20 % / 70 % |
| `grid box_respawn_min/max` | Nhịp băng chuyền nhả hộp (giảm để tăng tải) |
| `grid max_active_demands / demand_period` | Số yêu cầu màu mở đồng thời |
| `grid charge_rate / time_factor / pos_factor` | Tốc độ sạc và mô hình xả |

Muốn đổi hình học kho (số dock, hàng ngăn xếp, băng chuyền): sửa
`grid_layout.h` **và** vị trí spawn robot trong `.argos` cho khớp.
