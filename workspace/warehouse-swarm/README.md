# Warehouse Swarm — Kho hàng tự động kiểu Amazon (ARGoS3)

10 robot AMR (foot-bot) **hoàn toàn phi tập trung** vận hành một kho hàng:
3 băng truyền ở tường Đông liên tục nhả kiện hàng vào thùng chứa, mỗi kiện
mang nhãn địa chỉ **a–e**; robot đến thùng lấy hàng rồi giao đến đúng ô địa
chỉ **A–E** (5 ô màu ở tường Tây). Robot rảnh chờ trong **chuồng (depot)**
ở giữa kho — nơi cả 10 con spawn lúc khởi động.

Điểm nhấn: **không có máy chủ điều phối giao việc**. Mỗi robot tự quyết
định đi băng truyền nào bằng thuật toán đấu giá cục bộ — cân bằng tải tự
nổi lên từ luật đơn giản mà mọi con cùng chạy.

Dự án dùng chung "core swarm" với `ball-collector` (trường thế nhân tạo,
tách đàn Lennard-Jones, giao tiếp RAB, né va chạm 2 tầng) nhưng thay tầng
nhiệm vụ: từ "thăm dò tìm mục tiêu ngẫu nhiên" thành "phân việc theo
trạm cố định".

---

## 1. Build & Chạy

```bash
# Từ host: mở container
cd /home/dvt/argos && ./run.sh

# Build (lần đầu):
cd ~/workspace/warehouse-swarm
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# Chạy (từ thư mục gốc project — .argos dùng đường dẫn tương đối):
cd ~/workspace/warehouse-swarm
argos3 -c experiments/warehouse.argos
```

Bấm ▶ Play trên toolbar (mô phỏng mở ở trạng thái tạm dừng).

Chạy headless đo hiệu năng:

```bash
sed -e 's|length="0"|length="600"|' \
    -e '/<visualization>/,/<\/visualization>/d' \
    experiments/warehouse.argos > /tmp/wh.argos
argos3 -c /tmp/wh.argos 2>&1 | grep "Delivered total"
```

## 2. Đọc hiểu hình ảnh

| Hình ảnh | Ý nghĩa |
|---|---|
| 3 khối hộp ở tường Đông | Băng truyền; hàng chờ hiện thành **hàng ô vuông màu** trước mặt |
| 5 ô màu nhạt tường Tây + con số | Ô địa chỉ giao hàng (đỏ=A, lục=B, lam=C, vàng=D, tím=E); con số = số đơn đã giao thành công vào ô đó |
| 2 bãi xám trái & phải, mỗi bãi 5 ô | Bãi đỗ kiêm trạm sạc; ô **CAM = robot vừa vào, đang chờ 5 s kết nối**; ô **XANH LÁ = đang sạc**; xám = trống |
| Ô vuông màu trên lưng robot | Kiện đang chở — **màu = địa chỉ đích** |
| LED xanh lá / cyan / đỏ / xanh dương / tắt | Rảnh / đến băng truyền / giao hàng / **đi sạc–đang sạc** / hết pin |
| Thanh pin trên đầu robot | Mức pin (xanh >50%, cam >25%, đỏ ≤25%, xanh dương = đang sạc); luôn xoay theo hướng nhìn màn hình |
| Tia nối các robot | Gói tin RAB (liên kết giao tiếp) |
| 5 trụ tròn (1 giữa + 4 góc) | Cột kết cấu nhà kho — robot tự vòng qua |

## 3. Thuật toán

### Kế thừa từ ball-collector (core di chuyển)
1. **Trường thế nhân tạo** — vector tổng = mục tiêu + tách đàn + né vật cản
2. **Tách đàn Lennard-Jones** (phần đẩy, 45 cm) qua range-and-bearing
3. **Né va chạm 2 tầng** — vượt ngưỡng proximity: rẽ gắt bỏ qua mọi lực;
   vượt 3× ngưỡng: xoay tại chỗ (không thể chạm về mặt hình học)
4. **Bearing-only proportional steering** — lái theo góc lệch tới mục tiêu

### Mới — Né theo quyền ưu tiên (right-of-way)
Hai robot gặp nhau **không cùng né** (né đối xứng = lãng phí gấp đôi, như
2 người cùng nhường đường rồi cùng bước sang một bên). Thứ bậc ưu tiên:
**đang chở hàng (2) > đang đi lấy hàng (1) > đang rảnh (0)** — ngang cấp
thì so ID (thấp nhường). Trong lực tách đàn, con bị nhường nhân trọng số
1.85, con được ưu tiên chỉ 0.15 (tổng 2.0 = giữ nguyên tổng lực của cặp,
chỉ dồn việc né cho một bên). Robot đỏ (có hàng) đi thẳng như xe ưu tiên,
robot xanh tự dạt ra. Tầng né khẩn cấp (proximity) vẫn đối xứng — lưới
an toàn cuối không phân biệt ai.

### Mới — Phân việc đấu giá phi tập trung (market-based task allocation)
Mỗi robot mỗi tick nhận **độ dài hàng đợi 3 băng truyền** (phát toàn xưởng
như WMS/IoT thật — thông tin công khai), và nghe qua RAB **hàng xóm đang
nhắm băng nào, cách bao xa**. Nó tự tính điểm cho từng băng:

```
U(b) = 1.5 × hàng_chờ(b) − 1.0 × số_robot_đến_trước(b) − 0.35 × khoảng_cách(b) + nhiễu(0..0.3)
```

- **Chỉ đếm robot GẦN BĂNG HƠN MÌNH** (họ đến trước mình) → cả đàn tự sắp
  thành hàng đợi ngầm theo thứ tự khoảng cách, không cần xếp hàng tường minh
- **U tốt nhất < 0 → về chuồng chờ** (việc đã đủ người làm) — không chen chúc
- **Nhiễu nhỏ** phá thế đối xứng (10 con giống hệt nhau không cùng chọn 1 băng)
- **Đấu giá so le theo ID robot cả khi rảnh** — quyết định rời bãi đỗ nhỏ
  giọt từng con (kịp nghe claim của nhau) thay vì cả chuồng cùng ùa ra vì
  một kiện hàng rồi lũ lượt quay đầu
- **Ngưỡng rời chuồng** (`LEAVE_UTILITY` = 0.3) — robot đang đỗ yên chỉ
  rời ô khi việc đủ hấp dẫn (hàng gần / hàng đợi dài); robot đang lang
  thang giữa sàn thì nhận việc ngay khi U ≥ 0. Kiện ở băng xa sẽ chờ gom
  đủ ~2 kiện mới có robot xuất phát — batching tự nhiên, throughput không đổi
- **Hysteresis 0.8** — không nhảy băng vì chênh lệch vặt

Handover tại thùng: robot tay không ở trong bán kính 0.35 m gần nhất nhận
kiện đầu hàng đợi (1 kiện/băng/tick — như công nhân đưa từng thùng một).

### Mới — Né cột kết cấu bằng bản đồ + trường xoáy (vortex field)
Cảm biến proximity chỉ nhìn ~10 cm — quá muộn để định tuyến quanh cột ở
tốc độ 36 cm/s (thử nghiệm: throughput sập từ 150 → 24 đơn khi chỉ dựa
proximity). Robot AMR thật biết bản đồ mặt bằng, nên mỗi cột (khai trong
`stations`) tạo một trường lực tầm 0.8 m: **đẩy xa cột + thành phần tiếp
tuyến cùng chiều cố định** cho mọi robot — cả đàn vòng qua cột theo cùng
một chiều như vòng xuyến mini, không kẹt trước cột (local minimum), không
đối đầu trên vành cột. Kèm "hướng né dính" trong phản xạ proximity (chọn
1 bên và giữ nguyên đến khi thoát) chống giật trái-phải trước vật cản
chính diện. Kết quả: 132 đơn/10 phút với 5 cột (≈88% bản không cột).

### Mới — Pin & quản lý năng lượng dự đoán (predictive energy management)
Mỗi robot có pin thật của ARGoS (`<battery discharge_model="time_motion">`)
— hao theo quãng đường (0.8%/m), góc quay và thời gian. **Ô đỗ = trạm
sạc**: đứng trên ô là được nạp (0.4%/tick → đầy trong ~25 s), tận dụng
luôn giao thức nhận-ô qua RAB đã có. Các cơ chế:

- **Đi sạc theo dự đoán, không theo ngưỡng mù**: robot biết mô hình tiêu
  hao của chính nó, mỗi tick tính năng lượng cần để về tới trạm; khi
  `pin < cần_về_trạm × 1.4 + 6% dự trữ` → tự về sạc. Đang chở hàng thì
  giao xong mới đi (biên an toàn đã tính đủ cho việc đó).
- **Sạc có hysteresis**: sạc đến 90% mới rời trạm (không nhấp nhổm);
  robot rảnh đỗ ở bãi luôn được nạp cơ hội (opportunity charging).
- **Đấu giá biết pin**: chi phí khoảng cách nhân `(2 − pin)` — robot yếu
  thấy việc xa "đắt" hơn, tự nhường cho đồng đội khỏe; dưới 30% pin
  không nhận việc mới (loop functions cũng không giao kiện cho nó).
- **Quyền ưu tiên cao nhất** cho robot đi sạc (LED xanh dương) — sắp hết
  pin được cả kho nhường đường.
- **Hết pin = chết đứng tại chỗ** (LED tắt, thanh pin đen) thành vật cản
  — nhưng metric `dead robot-ticks: 0` chứng minh quản lý dự đoán không
  bao giờ để điều đó xảy ra; mức thấp nhất từng ghi nhận: 7.7%.
- Pin khởi đầu ngẫu nhiên 55–95% mỗi con — so le chu kỳ sạc cả đàn.
- **Tranh chấp ô sạc giải theo QUYỀN SỞ HỮU**: con gần ô hơn giữ ô (tính
  vị trí đối phương từ range+bearing) — robot đang đỗ sạc không bao giờ
  bị con ở xa "cướp" ô qua RAB; hoà mới so ID. Robot tụt dưới 30% giữa
  đường cũng tự nhả claim băng tải để quay về sạc thay vì cắm trại ở thùng.
  (Hai luật này sửa đúng lỗi "robot hết pin không chịu về dock" từng gặp.)
- **Ghép nối chính xác 2 tầng (precise docking)**: cách ô < 0.35 m tắt
  trường thế (bò thẳng, phản xạ né vẫn bảo vệ); < 0.18 m tắt nốt phản xạ
  né — thay bằng luật **đứng-chờ** nếu có vật chắn phía trước (không húc,
  không bị đá văng). Trước đây lực đẩy/phản xạ từ robot đỗ cạnh giữ robot
  lơ lửng ngoài bán kính sạc, hoặc đá cả robot đang đỗ bật khỏi ô. Độ
  khít yêu cầu: 5 cm quanh tâm ô; bán kính nhận sạc 0.18 m phủ toàn vùng
  mù nên kể cả lúc đứng-chờ vẫn được sạc.
- **Warm-up 5 s**: robot vào ô phải đứng yên 5 giây "bắt tay" (ô hiện màu
  cam) rồi điện mới vào (ô chuyển xanh lá); rời ô là phải bắt tay lại.
  Robot chết giữa đường tự nhả claim ô sạc (xác không được khoá trạm).
- Kiểm chứng 20 phút × 2 seed: 0 robot chết, pin thấp nhất ~23%,
  0 va chạm, 219–243 đơn.

### Mới — Bãi đỗ dạng lưới (docking)
Robot rảnh không lượn lờ: nó **nhận một ô đỗ** trong lưới 2×5 (ô gần nhất
chưa ai nhận — ô đang nhận được phát trong gói RAB, tranh chấp thì ID thấp
giữ, ID cao tự chọn lại), lái vào với **arrival damping** (giảm tốc theo
khoảng cách để dừng đúng tâm ô thay vì lượn quanh), **xoay về cùng hướng**
(hướng ra băng truyền, sẵn sàng xuất phát) rồi **đứng im hoàn toàn** —
khi đỗ, robot bỏ qua trường thế (không bị robot đi ngang kéo lệch hàng),
chỉ giữ phản xạ né khẩn cấp proximity. Có việc là nhả ô, đi làm.
Robot **spawn đúng từng ô đỗ** ngay từ đầu (distribute `method="grid"`
khớp lưới dock, hướng constant = hướng đỗ) — mở mô phỏng là thấy đội hình
thẳng hàng, và loại luôn va chạm lúc khởi động của cách rải ngẫu nhiên cũ.
Kiểm chứng: kho không hàng 30 s → 10/10 đỗ nguyên ô, 0 pair-tick;
benchmark 10 phút → 151 đơn, **0 pair-tick tuyệt đối** (kể cả lúc spawn).

## 4. Tham số chính (`experiments/warehouse.argos`)

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `spawn_period` | 12 | tick giữa 2 kiện hàng ra băng (1.2 s) |
| `queue_cap` | 6 | sức chứa thùng mỗi băng |
| `pickup_radius` | 0.35 | bán kính nhận hàng tại thùng (m) |
| `zone_half` | 0.4 | nửa cạnh ô địa chỉ (m) |
| `belt0..2`, `addr_a..e`, `dock_center` | — | tọa độ trạm (khai 2 chỗ: controller & loop functions, phải khớp) |
| `dock_rows`/`dock_cols`/`dock_spacing_x`/`dock_spacing_y` | 5 / 2 / 0.55 / 6.9 | 2 hàng đỗ-sạc trái (y=+3.45) & phải (y=−3.45), 5 ô/hàng. spacing_y lớn tách 2 hàng ra 2 phía. MỖI HÀNG ĐƠN là có chủ đích: ô nằm sau lưng ô khác sẽ bị chặn lối, robot cạn pin không vào được. Đổi thì sửa cả `<distribute>` grid |
| `max_speed` | 36 | tốc độ robot (cm/s) |
| Trọng số đấu giá `W_QUEUE/W_INBOUND/W_DISTANCE` | 1.5/1.0/0.35 | trong `footbot_warehouse.cpp` |
| `pillar0..4` / `pillar_radius` / `pillar_range` | 5 cột / 0.075 / 0.8 | bản đồ cột (phải khớp các `<cylinder>` trong arena) |
| `<battery>` trên foot-bot: `pos_factor`/`time_factor` | 0.008 / 2e-5 | hao pin theo mét / theo tick (khớp `<energy>` của controller) |
| `<energy>`: `safety_factor`/`reserve_charge` | 1.6 / 0.08 | tầng dự đoán: về sạc khi pin < cần-về-trạm × 1.6 + 8% |
| `<energy>`: `hard_charge_threshold` | 0.05 | ngưỡng CỨNG: dưới 5% buộc về sạc vô điều kiện (lưới an toàn cuối; tầng dự đoán thường kích hoạt sớm hơn vì từ góc xa cần ~6.5% mới về tới trạm) |
| `<energy>`: `resume_charge`/`min_work_charge` | 0.42 / 0.40 | sạc tới >40% mới được rời trạm đi làm; việc MỚI đòi ≥40%, việc đang dở giữ được tới sàn 25% (hysteresis chống nhấp nhổm sạc-làm-sạc) |
| `charge_rate` (warehouse) | 0.004 | tốc độ nạp mỗi tick khi đứng trên ô sạc |
| `charge_warmup` (warehouse) | 50 | số tick đứng trên ô trước khi điện vào (5 s) |

## 5. Kết quả benchmark (seed 124, headless)

| Chỉ số | 10 phút (speed 18) | 10 phút (speed 36 — hiện tại) |
|---|---|---|
| Đơn giao thành công | 74 | **150** (≈15 đơn/phút, scale tuyến tính ×2) |
| Phân bố theo địa chỉ | 14/15/13/16/16 | 26/23/26/41/34 (lệch do rút nhãn ngẫu nhiên) |
| Va chạm robot-robot thật | 0 (sát nhất 0.180 m) | **0** (sát nhất 0.180 m) |
| Khoảng hở tường nhỏ nhất | 0.56 m | 0.45 m |

Lưu ý khi đổi tốc độ: các lực an toàn phải scale theo — khi tăng
`max_speed` ×2 thì `gain` (separation) và `obstacle_gain` cũng ×2, và
`hard_avoid_threshold` giảm nhẹ (0.05 → 0.04) để phản ứng sớm hơn vì
quãng phanh dài ra. Nếu chỉ đổi mỗi `max_speed`, lực mục tiêu sẽ đè
lực né và va chạm quay lại.

Loop functions tự in dòng thống kê này khi kết thúc mỗi lần chạy.

## 6. Kiến trúc

```
warehouse-swarm/
├── controllers/footbot_warehouse/      # não robot: đấu giá chọn băng,
│                                       #  lái trường thế, RAB, né va chạm
├── loop_functions/warehouse_loop_functions/
│   ├── warehouse_loop_functions.*      # "nhà xưởng": spawn kiện, handover
│   │                                   #  tại thùng, nhận hàng tại ô, thống kê
│   └── warehouse_qt_user_functions.*   # vẽ: kiện trong thùng, kiện trên lưng,
│                                       #  chữ A–E, bộ đếm
└── experiments/warehouse.argos         # layout kho, 10 robot, tham số
```

## 7. Hướng mở rộng

- Kiện hàng ưu tiên (express) — trọng số riêng trong hàm utility
- Trạm sạc + mức pin — robot tự về sạc khi yếu (threshold-based)
- Kệ hàng làm vật cản giữa kho — thử trường thế với local minima
- Tăng 20–30 robot xem điểm bão hòa của 3 băng truyền
