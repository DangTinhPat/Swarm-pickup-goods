# Ball Collector — Mô phỏng bầy đàn thu thập bóng (ARGoS3)

Bầy 10 robot foot-bot **hoàn toàn phi tập trung** (không có điều phối trung
tâm) tìm và thu thập các quả bóng spawn ngẫu nhiên liên tục trên bản đồ
8×8 m, mang về bãi tập kết ở chính giữa. Mỗi robot tự quyết định dựa trên
cảm biến cục bộ và tin nhắn từ hàng xóm — đúng nguyên tắc swarm robotics:
**hành vi bầy đàn thông minh nổi lên (emergent) từ các luật cục bộ đơn giản**.

Việc "dính bóng" là phi vật lý: robot chạm vào là bóng gắn lên lưng
(loop functions xử lý), robot tự về đích, thả bóng, rồi đi tìm tiếp.

---

## 1. Môi trường & Build

### Yêu cầu
- Chạy trong Docker container `argos3-dev:ubuntu22` (xem `/home/dvt/argos/README.md`
  của repo cha — ARGoS3 đã cài system-wide trong image).
- Trên host chỉ cần Docker + X11 (GUI hiển thị qua X forwarding).

### Build lần đầu

```bash
# Từ host: mở container
cd /home/dvt/argos && ./run.sh

# Trong container:
cd ~/workspace/ball-collector
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Build lại sau khi sửa code C++

```bash
cd ~/workspace/ball-collector/build
make -j$(nproc)
```

Sửa file `.argos` (tham số, bản đồ) thì **không cần build lại**.

---

## 2. Cách dùng

### Chạy có giao diện

```bash
# QUAN TRỌNG: chạy từ thư mục gốc project (file .argos dùng đường dẫn
# tương đối build/controllers/...)
cd ~/workspace/ball-collector
argos3 -c experiments/ball_collection.argos
```

Cửa sổ mở ra ở trạng thái **tạm dừng** — bấm ▶ (Play) trên toolbar.
Các nút khác: ⏸ pause, ⏭ chạy từng bước, tốc độ mô phỏng, camera
(kéo chuột xoay, lăn chuột zoom).

### Đọc hiểu hình ảnh trong GUI

| Hình ảnh | Ý nghĩa |
|---|---|
| Trụ cam trên sàn | Quả bóng chưa nhặt |
| Trụ cam trên lưng robot | Bóng đang được mang |
| Vòng tròn xám giữa bản đồ | Bãi tập kết (nest) |
| Chữ "Balls: N" | Tổng số bóng đã thu |
| Tia nối giữa các robot | Gói tin RAB đang truyền (liên kết giao tiếp) |
| LED **xanh lá** | Đang thăm dò (Lévy walk) |
| LED **cyan** | Đã claim một quả bóng, đang tới nhặt |
| LED **đỏ** | Đang mang bóng về nest |

Cảnh đáng xem: 2 robot cyan cùng nhắm 1 quả bóng → 1 con đột ngột chuyển
xanh lá và quay đầu 180° bỏ đi — đó là **đàm phán phân công thành công**.

### Chạy không giao diện (đo hiệu năng / chạy batch)

```bash
cd ~/workspace/ball-collector
sed -e 's|length="0"|length="120"|' \
    -e '/<visualization>/,/<\/visualization>/d' \
    experiments/ball_collection.argos > /tmp/headless.argos
argos3 -c /tmp/headless.argos 2>&1 | grep -c "deposited"   # đếm bóng thu được
```

---

## 3. Kiến trúc project

```
ball-collector/
├── CMakeLists.txt                       # find_package(ARGoS), C++17
├── controllers/footbot_collector/        # NÃO ROBOT (chạy trên từng robot)
│   ├── footbot_collector.h
│   └── footbot_collector.cpp
├── loop_functions/collection_loop_functions/
│   ├── collection_loop_functions.*       # "THƯỢNG ĐẾ" mô phỏng: spawn bóng,
│   │                                     #  xử lý nhặt/thả, điểm, camera ảo
│   └── collection_qt_user_functions.*    # Vẽ: bóng, bóng trên lưng, điểm số
└── experiments/ball_collection.argos     # Bản đồ, số robot, mọi tham số
```

Phân vai quan trọng:
- **Controller** = code chạy độc lập trên từng robot, chỉ được dùng cảm
  biến + tin nhắn cục bộ. Toàn bộ trí thông minh bầy đàn nằm ở đây.
- **Loop functions** = code của môi trường mô phỏng, biết mọi thứ. Ở đây
  nó chỉ làm 3 việc "vật lý": spawn bóng, cho nhặt khi chạm, cho thả khi
  vào nest — và đóng vai **camera ảo** (báo cho robot vị trí quả bóng gần
  nhất trong tầm nhìn 1.2 m, thay cho camera thật để giữ mô phỏng đơn giản).
  Nó KHÔNG điều phối robot.

---

## 4. Cơ chế điều khiển & di chuyển

### 4.1. Máy trạng thái (mỗi robot)

```
                 thấy bóng (camera ảo / được tuyển mộ)
   ┌─────────┐ ──────────────────────────────────────▶ ┌────────────┐
   │ EXPLORE │                                          │ GO_TO_BALL │
   │ (xanh)  │ ◀────────────────────────────────────── │  (cyan)    │
   └─────────┘   thua đàm phán (tabu) / bóng biến mất   └────────────┘
        ▲                                                     │ chạm bóng
        │  thả bóng tại nest                                  ▼
        │                    ┌──────────┐                     
        └─────────────────── │  RETURN  │ ◀───────────────────┘
                             │  (đỏ)    │
                             └──────────┘
```

### 4.2. Di chuyển bằng trường thế nhân tạo (vector field)

Mỗi tick (0.1 s), robot tính **tổng các vector lực** rồi đổi ra tốc độ
2 bánh xe:

```
V_tổng = V_mục_tiêu + V_tách_đàn + V_né_vật_cản [+ V_phân_tán khi thăm dò]
```

| Vector | Nguồn cảm biến | Công thức |
|---|---|---|
| `V_mục_tiêu` | positioning | hướng tới nest / bóng / hướng Lévy, độ lớn = max_speed |
| `V_tách_đàn` | RAB (range + bearing hàng xóm) | Lennard-Jones phần đẩy, chỉ khi gần hơn 40 cm, giới hạn ≤ 60% max_speed |
| `V_né_vật_cản` | 24 cảm biến proximity | đẩy ngược hướng tổng các tia chạm (tường + robot sát cạnh) |
| `V_phân_tán` | RAB | đẩy nhẹ khỏi mọi hàng xóm trong 1.5 m, giảm tuyến tính (chỉ khi EXPLORE) |
| (+ đẩy khỏi nest) | positioning | khi EXPLORE trong bán kính 1.2 m quanh nest |

Vector tổng được đổi thành tốc độ bánh trái/phải qua bộ ba ngưỡng
NO_TURN / SOFT_TURN / HARD_TURN (mượn từ ví dụ flocking chính thức của
ARGoS): lệch ít → đi thẳng, lệch vừa → 2 bánh lệch tốc, lệch nhiều →
xoay tại chỗ.

### 4.3. Giao tiếp: gói tin RAB 12 byte, phát mỗi tick

```
[0]     state       (EXPLORE / GO_TO_BALL / RETURN)
[1-2]   id          hash 16-bit của tên robot — dùng tie-break
[3-6]   sighting    x,y (cm) quả bóng đang thấy — dùng cho tuyển mộ
[7-10]  claim       x,y (cm) quả bóng đang nhắm — dùng phân công
[11]    claim_dist  khoảng cách của người gửi tới claim (đơn vị 4 cm)
```

Tầm phát 3 m — **phải ≥ 2× tầm nhìn** (1.2 m) để hai robot ở hai phía
một quả bóng vẫn nghe được claim của nhau mà đàm phán.

---

## 5. Danh sách thuật toán bầy đàn

Tất cả đều **cục bộ, phi tập trung** — không robot nào biết toàn cục,
không có trọng tài:

1. **Trường thế nhân tạo (Artificial Potential Fields)** — di chuyển là
   tổng vector lực hút/đẩy (mục 4.2). Nền tảng: physicomimetics
   (Spears et al.).

2. **Tách đàn Lennard-Jones** — phần đẩy của thế LJ tổng quát
   `LJ(d) = -G/d · [(δ/d)^2E - (δ/d)^E]`, kích hoạt khi 2 robot gần hơn
   `target_distance` = 40 cm. Cùng công thức với ví dụ flocking chính
   thức của ARGoS.

3. **Giao tiếp cục bộ + gossip (Range-and-Bearing)** — mỗi robot phát
   gói 12 byte mỗi tick cho hàng xóm trong 3 m; thông tin (bóng thấy được,
   claim) lan truyền robot-với-robot, không có bảng tin toàn cục.

4. **Phân công nhiệm vụ phi tập trung (greedy claim + yield + tabu)** —
   - Robot nhắm bóng nào thì "claim" bóng đó trong mọi gói tin, kèm
     khoảng cách của mình tới bóng.
   - Nghe thấy claim trùng bóng mình (≤ 25 cm): **con xa hơn thua**;
     chênh ≤ 5 cm coi là hoà → **ID nhỏ hơn thắng** (tie-break tất định,
     đúng 1 con nhường, không dằng co).
   - Con thua ghi bóng vào **tabu list** 10 s: không claim lại, không
     nhận tuyển mộ cho quả đó, và chủ động quay đầu đi 1 m hướng ngược
     lại. Nhờ tabu, nhường là nhường thật chứ không phải 1 tick.

5. **Tuyển mộ (recruitment)** — robot đang mang bóng (không nhặt thêm
   được) vẫn phát vị trí bóng nó đi ngang qua; hàng xóm rảnh (EXPLORE)
   nhận làm mục tiêu. Mô phỏng cơ chế chia sẻ nguồn thức ăn của kiến/ong.

6. **Lévy walk** — khi thăm dò, đi các đoạn thẳng độ dài rút từ phân bố
   đuôi nặng Pareto `L = L_min · U^(-1/α)` (α = 1.5, chặn tại L_max),
   đổi hướng ngẫu nhiên giữa các đoạn. Là chiến lược tìm kiếm tối ưu cho
   mục tiêu thưa, quan sát được ở ong, kiến, hải âu, cá mập.

7. **Phân tán tìm kiếm (dispersion)** — robot EXPLORE đẩy nhẹ nhau trên
   toàn tầm giao tiếp (giảm tuyến tính về 0 tại 1.5 m) → bầy tự giãn đều,
   mỗi con một vùng, không cần ai chia vùng.

8. **Rời tổ có định hướng (central-place departure)** — vùng quanh nest
   không bao giờ có bóng, nên: (a) robot EXPLORE bị trường đẩy ra khỏi
   bán kính 1.2 m quanh nest; (b) vừa thả bóng xong là chọn hướng rời đi
   trong nón ±60° hướng ra ngoài — như kiến rời tổ đi kiếm ăn.

---

## 6. Tham số (trong `experiments/ball_collection.argos`)

### Môi trường (loop functions)
| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `quantity` (trong `<distribute>`) | 10 | số robot |
| `spawn_period` | 15 | tick giữa 2 lần spawn bóng (10 tick = 1 s) |
| `max_balls` | 20 | số bóng tối đa trên sàn cùng lúc |
| `sight_range` | 1.2 | tầm camera ảo (m) |
| `pickup_radius` | 0.18 | khoảng cách tính là "chạm" bóng (m) |
| `nest` / `nest_radius` | 0,0 / 0.45 | vị trí + bán kính bãi tập kết |
| `spawn_min` / `spawn_max` | ±3.7 | vùng spawn bóng |

### Robot (controller params)
| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `max_speed` (wheel_turning) | 15 | tốc độ bánh xe (cm/s) |
| `target_distance` (separation) | 40 | cự ly giãn cách LJ (cm) |
| `gain` / `exponent` (separation) | 300 / 2 | độ mạnh + độ dốc thế LJ |
| `levy_alpha` / `min_step` / `max_step` | 1.5 / 0.6 / 5.0 | tham số Lévy walk |
| `nest_avoid_radius` / `nest_repel_gain` | 1.2 / 15 | vùng cấm-tìm quanh nest + lực đẩy |
| `disperse_gain` / `disperse_range` | 8 / 150 | lực giãn đàn + tầm (cm) |
| `obstacle_gain` (navigation) | 30 | độ mạnh né vật cản |
| `give_up_range` (navigation) | 0.7 | tới gần claim mức này mà không thấy bóng → bỏ (m) |
| `rab_range` (trên `<foot-bot>`) | 3 | tầm giao tiếp (m), **phải ≥ 2× sight_range** |
| `rab_data_size` (trên `<foot-bot>`) | 12 | byte gói tin, phải khớp `Broadcast()` |

Lưu ý: `nest` xuất hiện 2 chỗ (navigation của controller + loop
functions) — sửa phải khớp nhau.

### Hằng số trong code (`footbot_collector.cpp`)
| Hằng | Giá trị | Ý nghĩa |
|---|---|---|
| `CLAIM_MATCH_RADIUS` | 0.25 m | 2 claim gần hơn mức này = cùng 1 quả bóng |
| `CLAIM_TIE_MARGIN` | 0.05 m | chênh khoảng cách ≤ mức này = hoà → so ID |
| `TABU_RADIUS` / `TABU_TICKS` | 0.3 m / 100 | bán kính khớp + tuổi thọ tabu (10 s) |

---

## 7. Kết quả benchmark (2 phút mô phỏng, headless)

| Phiên bản | Bóng thu được |
|---|---|
| Random walk cố định (bản đầu, map 4×4) | 26 |
| + RAB, claim/yield, Lévy, LJ (map 4×4) | 30 |
| + dispersion, nest repulsion (map 4×4) | 34 |
| Map 8×8 (diện tích ×4), nest giữa | 27 |
| + tabu, tie-break ID, RAB 3 m (**hiện tại**, map 8×8) | **33** |

---

## 8. Hướng mở rộng gợi ý

- Thay camera ảo bằng **omnidirectional camera** thật của foot-bot
  (bóng làm light/LED entity) — bỏ hoàn toàn "thượng đế".
- **Pheromone ảo**: loop functions vẽ vệt mờ dần nơi robot đi qua,
  robot tránh vùng đã được thăm gần đây (stigmergy như kiến thật).
- **Phân vai động**: một phần đàn chuyên chở (ở gần nest), phần chuyên
  thăm dò xa — threshold-based task allocation.
- Tăng số robot lên 20–50 xem hiệu ứng tắc nghẽn tại nest và cách
  separation tự giải quyết.
