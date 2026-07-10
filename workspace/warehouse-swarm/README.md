# Warehouse Swarm

> Mô phỏng **đội robot AMR phi tập trung** vận hành kho hàng kiểu Amazon trên ARGoS3 —
> không có máy chủ điều phối, mọi hành vi tập thể tự nổi lên từ luật cục bộ mỗi robot cùng chạy.

<p>
<strong>10 robot</strong> · <strong>8×8 m</strong> · <strong>3 băng tải → 5 ô giao hàng</strong> ·
điều hướng trường thế · đấu giá phân việc · tin đồn cục bộ · stigmergy · quản lý pin
</p>

---

## Mục lục

1. [Kho hàng phi tập trung: 10 robot, 3 băng tải, 5 ô giao](#1-kho-hàng-phi-tập-trung-10-robot-3-băng-tải-5-ô-giao)
2. [Build và chạy mô phỏng](#2-build-và-chạy-mô-phỏng)
3. [Giải mã màn hình mô phỏng](#3-giải-mã-màn-hình-mô-phỏng)
4. [Máy trạng thái điều khiển robot](#4-máy-trạng-thái-điều-khiển-robot)
5. [Ngăn xếp thuật toán bầy đàn](#5-ngăn-xếp-thuật-toán-bầy-đàn)
6. [Kiểm chứng throughput, an toàn và mở rộng](#6-kiểm-chứng-throughput-an-toàn-và-mở-rộng)
7. [Tham chiếu tham số](#7-tham-chiếu-tham-số)
8. [Kiến trúc mã nguồn (controller và loop functions)](#8-kiến-trúc-mã-nguồn-controller-và-loop-functions)
9. [Lộ trình phát triển](#9-lộ-trình-phát-triển)

---

## 1. Kho hàng phi tập trung: 10 robot, 3 băng tải, 5 ô giao

Kho hàng có 3 **băng tải** (tường Đông) liên tục nhả kiện, mỗi kiện mang nhãn địa chỉ
`a–e`. Đội **10 robot foot-bot** đến băng lấy kiện rồi giao tới đúng **ô địa chỉ** `A–E`
(tường Tây). Khi rảnh hoặc yếu pin, robot tự về **bãi đỗ kiêm trạm sạc** ở giữa kho.

```
   Băng tải (Đông)            Robot AMR ×10              Ô địa chỉ (Tây)
  ┌──────────┐   lấy kiện   ┌────────────┐  giao kiện   ┌──────────┐
  │ belt 0   │ ───────────► │  foot-bot  │ ───────────► │   A B C  │
  │ belt 1   │              │  tự đấu giá │              │   D E    │
  │ belt 2   │              └─────┬──────┘              └──────────┘
  └──────────┘                    │  hết việc / yếu pin
                                  ▼
                          ┌────────────────┐
                          │  Bãi đỗ + Sạc  │  (giữa kho — lưới 2×5 ô)
                          └────────────────┘
```

**Nguyên tắc thiết kế:** không một thực thể nào biết toàn cục. Mỗi robot chỉ dựa vào
cảm biến của chính nó + tin nhắn tầm ngắn từ hàng xóm. Cân bằng tải, xếp hàng, né tắc
nghẽn... đều là **hành vi nổi (emergent)** chứ không được lập trình tập trung.

**Sáu trụ cột hành vi**

| | |
|---|---|
| 🧭 **Điều hướng** | Trường thế nhân tạo (mục tiêu + tách đàn + né cột + né vật cản) |
| 🏷️ **Phân việc** | Đấu giá thị trường phi tập trung, **chống lưỡng lự** (sticky commitment) |
| 📡 **Thông tin** | Cảm nhận cục bộ + tin đồn lan truyền (epidemic gossip), không oracle toàn tri |
| 🐜 **Stigmergy** | Vệt "mùi" trên sàn để né đông bãi đỗ, đọc bằng cảm biến sàn thật |
| 🔋 **Năng lượng** | Pin thật, tự về sạc, sạc cơ hội, ưu tiên đường cho robot yếu |
| 🛟 **An toàn** | Né va chạm 2 tầng + cứu hộ thoát kẹt 2 cấp — **0 robot chết** qua mọi test |

---

## 2. Build và chạy mô phỏng

```bash
# 1. Mở môi trường (container ARGoS3)
cd /home/dvt/argos && ./run.sh

# 2. Build (lần đầu)
cd ~/workspace/warehouse-swarm && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# 3. Chạy có giao diện (mở ở trạng thái tạm dừng — bấm ▶ Play)
cd ~/workspace/warehouse-swarm
argos3 -c experiments/warehouse.argos
```

**Chạy headless để đo hiệu năng** (`length` tính bằng **giây**; `1800 s = 30 phút mô phỏng`):

```bash
sed -e 's|length="0"|length="1800"|' \
    -e '/<visualization>/,/<\/visualization>/d' \
    experiments/warehouse.argos > /tmp/wh.argos
argos3 -c /tmp/wh.argos 2>&1 | grep -E "Delivered total|dead robot"
```

Loop functions tự in dòng thống kê (đơn giao, va chạm, khoảng hở, pin) khi kết thúc.

---

## 3. Giải mã màn hình mô phỏng

| Nhìn thấy | Ý nghĩa |
|---|---|
| 3 khối hộp tường Đông, có **ô vuông màu** phía trước | Băng tải + hàng đợi trong bin |
| 5 ô màu nhạt tường Tây + con số | Ô giao hàng (🔴A 🟢B 🔵C 🟡D 🟣E); số = tổng đơn đã giao vào ô |
| 2 dải xám (trái/phải), mỗi dải 5 ô | Bãi đỗ kiêm trạm sạc — 🟠 vừa vào (chờ 5 s) · 🟢 đang sạc · xám trống |
| Sàn quanh bãi đỗ **xám đậm dần** | Vệt stigmergy — bên càng đông, càng đậm |
| Ô vuông màu **trên lưng** robot | Kiện đang chở (màu = địa chỉ đích) |
| **LED** robot | 🟢 rảnh · 🟡 đi tuần tra băng · 🔵(cyan) đến băng lấy hàng · 🔴 giao hàng · 🔵 đi/đang sạc · ⚫ hết pin |
| Thanh pin trên đầu robot | Mức pin (xanh >50% · cam >25% · đỏ ≤25% · lam = đang sạc) |
| Tia nối các robot | Liên kết giao tiếp RAB |
| 5 trụ tròn (1 giữa + 4 góc) | Cột kết cấu — robot tự vòng qua |

---

## 4. Máy trạng thái điều khiển robot

Bộ điều khiển là một **máy trạng thái** chạy mỗi tick (10 tick/giây):

```
          có việc            nhận kiện tại băng        vào đúng ô đích
  IDLE ───────────► TO_BELT ──────────────────► DELIVER ──────────────► (IDLE)
   ▲ │                                                                     │
   │ └────────── hết việc / thông tin cũ → về dock hoặc đi tuần tra        │
   │                                                                       │
   │  pin ≥ resume(70%)                            pin < 10% (và tay không)│
   └──────────── CHARGE ◄──────────────────────────────────────────────────┘
                    │
                    └── pin = 0 → DEAD (đứng im, thành vật cản cho robot khác né)
```

| Trạng thái | Ý nghĩa | LED |
|---|---|---|
| `IDLE` | Rảnh: về ô đỗ chờ (sạc cơ hội) hoặc đi tuần tra băng nếu thông tin cũ | 🟢 / 🟡 |
| `TO_BELT` | Đã nhận việc, đang lái tới bin băng tải | 🔵 cyan |
| `DELIVER` | Đang chở kiện tới ô địa chỉ | 🔴 |
| `CHARGE` | Pin < 10%: về bãi sạc, chỉ rời khi ≥ 70% | 🔵 |
| `DEAD` | Pin cạn: đứng im tại chỗ | ⚫ |

---

## 5. Ngăn xếp thuật toán bầy đàn

### 5.1 Điều hướng bằng trường thế nhân tạo

Mỗi tick, vector điều khiển = tổng của **mục tiêu + tách đàn + né cột + né vật cản**,
rồi quy đổi ra tốc độ 2 bánh (theo ví dụ flocking chính thức của ARGoS).

| Thành phần | Thuật toán | Ghi chú |
|---|---|---|
| Hút về mục tiêu | Bearing-only proportional | lái theo góc lệch tới đích |
| Tách đàn | **Lennard-Jones** (chỉ phần đẩy) qua RAB | giữ khoảng cách 45 cm |
| Né cột | **Trường xoáy (vortex)**: đẩy xa + tiếp tuyến cùng chiều | cả đàn vòng cột như bùng binh, không kẹt local-minimum |
| Né vật cản | Proximity phản xạ | lưới an toàn tầm gần |

### 5.2 Né va chạm theo quyền ưu tiên (right-of-way)

Hai robot gặp nhau **không cùng né** (né đối xứng = lãng phí). Thứ bậc:
`đang sạc > đang giao (chở hàng) > đi lấy hàng > rảnh`, ngang cấp thì so ID.
Con nhường nhân trọng số **1.85**, con ưu tiên **0.15** (tổng = 2.0, giữ nguyên tổng
lực). Robot chở hàng đi thẳng như xe ưu tiên; robot đang cứu hộ được nhường tuyệt đối.

### 5.3 Phân việc đấu giá thị trường và diệt lưỡng lự

Mỗi robot tự chấm điểm từng băng và chọn băng tốt nhất — **không có bộ điều phối**:

```
U(b) = 1.5·hàng_chờ(b) − 1.0·số_robot_đến_trước(b) − 0.35·khoảng_cách(b)·(2−pin) + nhiễu
```

- Chỉ đếm robot **gần băng hơn mình** ⇒ cả đàn tự xếp thành hàng đợi ngầm theo khoảng cách.
- Nhân `(2−pin)` ⇒ robot yếu thấy việc xa "đắt" hơn, tự nhường cho đồng đội khỏe.
- Nhiễu nhỏ phá thế đối xứng (10 con giống hệt không cùng chọn một băng).

> #### 🎯 Diệt lưỡng lự (anti-dithering) từ hai nguồn khác nhau
>
> **Triệu chứng:** một số robot **đi ra khỏi dock rồi quay vào, lặp đi lặp lại mà không
> giao được kiện nào** — di chuyển phụ thuộc đàn, mất tính độc lập, còn gây tranh chấp.
> Truy vết từng tick (bám một robot cụ thể) cho thấy **hai** nguồn dao động khác nhau, và
> cả hai đều được sửa **về mặt cấu trúc** (không phải chỉnh tham số cho đỡ):
>
> **① Robot đang làm việc — cam kết dính** ([`footbot_task_allocation.cpp`](controllers/footbot_warehouse/footbot_task_allocation.cpp)):
> đã nhận băng thì **không bao giờ quay về dock** vì một dao động nhỏ. Chỉ bỏ việc khi có
> **sự thật cứng** (hết pin / băng bị chặn / tới đủ gần thấy bin rỗng). Vẫn được **đổi sang
> băng khác tốt hơn rõ rệt** (cân bằng tải có ích, khác lưỡng lự ra-vào dock). Kèm **khoá
> quyết định** giữ mỗi quyết định tối thiểu 3 giây.
>
> **② Robot dư (surplus) — nghỉ tại chỗ thay vì tuần tra vô tận** ([`footbot_comms.cpp`](controllers/footbot_warehouse/footbot_comms.cpp)):
> robot mà thị trường xác định là "chưa cần" (luôn có đồng đội gần băng hơn) trước đây cứ
> đi tuần tra → làm mới thông tin → về → tin lại cũ → tuần tra tiếp, **mãi mãi**. Hai luật
> sửa việc này: (a) **chỉ tuần tra khi thực sự "mù"** — mọi băng đều hết hạn thông tin; nếu
> còn dù chỉ một băng có tin tươi thì robot cứ đấu giá trên đó, không cần đi đâu; (b) **thời
> gian nghỉ (patrol cooldown)** — sau một chuyến do thám, robot đỗ nghỉ ~25 s mới được đi lại,
> lệch pha theo ID để cả đàn không do thám đồng loạt. Kết quả: robot dư **đứng yên nghỉ tại
> ô đỗ**, chỉ kích hoạt khi tin đồn báo có việc — đúng vai trò "quân dự bị theo nhu cầu".
>
> **Kết quả:** throughput tăng **~3× so với trước khi sửa** và đều giữa các seed (xem
> [§6](#6-kiểm-chứng-throughput-an-toàn-và-mở-rộng)); robot dư không còn chen lấn cản đường đội đang làm.

### 5.4 Thông tin băng tải: cảm nhận cục bộ cộng tin đồn

Không có "máy chủ WMS toàn tri". Mỗi robot chỉ biết trạng thái thật của một băng khi
ở đủ gần để **cảm nhận** (`belt_sense_range = 1.2 m`), rồi **lan truyền** niềm tin đó qua
RAB kiểu dịch tễ (epidemic gossip) — tin cũ dần theo thời gian, tin mới hơn đè tin cũ hơn.

| Cơ chế | Chi tiết |
|---|---|
| Dấu thời gian tuyệt đối | Lưu `OriginTick` (lúc xác nhận thật), tuổi luôn tính lại `= now − OriginTick` |
| Chống vòng lặp tự củng cố | `OriginTick` chỉ tiến, không lùi ⇒ 2 robot cạnh nhau không "làm mới" tin cũ cho nhau |
| Bù trễ 1 tick của RAB | Trừ đúng 1 tick khi suy ra origin ⇒ tin không "trẻ hoá" tích luỹ qua mỗi chặng relay |
| Mã hoá "độ tươi" | Truyền `127 − tuổi` thay vì tuổi thô ⇒ gói rác toàn-0 lúc khởi động không đầu độc niềm tin |
| Tuần tra chỉ khi "mù" | Robot rảnh chỉ đi do thám khi **mọi** băng đều hết hạn tin (không còn gì để đấu giá); còn dù một băng tươi thì đứng yên nghỉ. Kèm nghỉ ~25 s giữa 2 chuyến (LED 🟡) |
| Cờ "bị chặn" | Robot chết nằm trên điểm nhận hàng ⇒ băng "không dùng được", đi chung kênh gossip, không cần kênh cấp cứu riêng |

> 3 lỗi gossip (đầu độc gói rác · vòng lặp tự củng cố · sai số dồn qua chặng) đều được
> tìm ra bằng bám vết một robot qua từng tick, không suy đoán. Chi tiết "vì sao" nằm gọn
> trong comment của [`footbot_comms.cpp`](controllers/footbot_warehouse/footbot_comms.cpp).

### 5.5 Stigmergy né đông bãi đỗ

Loop functions tô sàn quanh mỗi bên bãi đỗ **đậm dần theo lượng robot vừa đỗ** (tăng khi
có robot mới đỗ, **phai dần mỗi tick** — đúng chất vệt pheromone của kiến). Robot đọc lại
độ đậm bằng **cảm biến sàn thật** (`footbot_motor_ground`) và cộng một khoản phạt mềm vào
điểm chọn ô ở bên đang đông. Vì là stigmergy, nó **chỉ tác dụng khi robot đã ở gần bên đó**
— không thể ảnh hưởng quyết định từ xa.

### 5.6 Năng lượng và cạm bẫy NaN của time_motion

Pin dùng entity pin của ARGoS làm **kho lưu trữ**, còn việc **xả pin tự tính trong
loop functions** (theo quãng đường + hao nền mỗi tick) — xem hộp cảnh báo bên dưới về lý
do. Chính sách **ngưỡng cứng**, không dự đoán khoảng cách:

- Pin **< 10%** (và tay không) → về sạc; **≥ 70%** mới được rời trạm đi làm.
- **Sạc cơ hội:** robot rảnh chờ ở dock vẫn được nạp, không cần đợi tới ngưỡng khẩn.
- **Warm-up 5 s** khi vào ô (bắt tay) — có kẹp sàn bảo vệ 1% để robot không chết mid-handshake.
- **Ưu tiên đường** cao nhất cho robot đi sạc; pin khởi đầu ngẫu nhiên 55–95% để so le chu kỳ sạc.

> #### ⚠️ Không dùng `discharge_model="time_motion"` — nó sinh NaN
>
> Bản ARGoS trong container có lỗi: model xả pin `time_motion`/`motion` gọi `acos(1+ε)` khi
> robot **xoay** → trả về **NaN**, và `Max(0, NaN)` giữ NaN nên pin hỏng **vĩnh viễn** (cảm
> biến kẹt ở NaN, robot coi như chết đứng ở dock — đúng triệu chứng "robot chết ở dock sạc").
> Nguy hiểm là **mọi thống kê bị NaN đánh lừa**: phép so sánh với NaN luôn cho `false` nên
> `dead robot-ticks` không tăng và "pin thấp nhất" đóng băng ở giá trị cuối còn hợp lệ — test
> headless báo "0 chết" trong khi GUI vẫn thấy robot hỏng.
>
> **Cách sửa** (đã áp dụng, kiểm chứng `globalNaN=0` + pin lên/xuống thật): khai báo
> `<battery discharge_model="time" delta="0.0" ...>` (entity chỉ lưu điện, model nội bộ
> không xả gì), rồi loop functions tự trừ pin mỗi tick theo `drain_time + drain_move × quãng_đường`
> (xem [`warehouse_robot_update.cpp`](loop_functions/warehouse_loop_functions/warehouse_robot_update.cpp)).

### 5.7 Cứu hộ thoát kẹt hai cấp

Theo dõi **khoảng cách-tới-đích** (không phải quãng đường thô — thứ này không phân biệt
được "đang tiến" với "đang dạt qua lại"). Nếu 2 cửa sổ 3 giây liền không rút ngắn được:

- **Cấp 1 (nhẹ):** lao theo hướng ngẫu nhiên 1.5 s (phá thế đối đầu 2 robot).
- **Cấp 2 (sâu):** sau nhiều lần cấp 1 thất bại — **lùi thẳng** rồi lao ra 3 s, phát cờ
  "đang cứu hộ" để cả đàn nhường đường tuyệt đối.

---

## 6. Kiểm chứng throughput, an toàn và mở rộng

Tất cả đo **headless, 30 phút mô phỏng/seed**. Bán kính thân robot 0.085 m ⇒ khoảng cách
tâm-tâm ≥ 0.17 m nghĩa là **không chồng lấn thân**.

### 6.1 Trước / sau khi diệt lưỡng lự (5 seed, cấu hình chuẩn 10 robot)

| Seed | Nguyên bản (còn lưỡng lự) | Sau khi sửa | Robot chết |
|---:|---:|---:|:---:|
| 1 | 88 | **267** | 0 |
| 2 | 172 | **334** | 0 |
| 3 | 233 | **361** | 0 |
| 4 | — | **331** | 0 |
| 5 | — | **288** | 0 |
| **Trung bình 5 seed** | — | **≈ 316** | **0** |

Bỏ được lãng phí của các robot dư (trước đây chỉ chen lấn quanh dock, vừa không giao được
gì vừa cản đội đang làm), throughput tăng **~3×** so với bản nguyên gốc và **đều giữa các seed**.

### 6.2 Chi tiết an toàn (5 seed sau khi sửa)

| Seed | Đơn giao | Va chạm (pair-tick) | Sát nhất robot | Pin thấp nhất | Robot chết |
|---:|---:|---:|---:|---:|:---:|
| 1 | 267 | 0 | 0.182 m | 55.7% | 0 |
| 2 | 334 | 0 | 0.197 m | 60.6% | 0 |
| 3 | 361 | 93 | 0.172 m | 56.0% | 0 |
| 4 | 331 | 0 | 0.189 m | 58.2% | 0 |
| 5 | 288 | 94 | 0.170 m | 59.9% | 0 |

> Va chạm pair-tick (0–94 trên 18 000 tick ≈ ≤ 0.5% thời gian) là **cọ nhẹ thoáng qua** khi
> giao thông đông, không phải đâm cứng: mọi seed đều **0 robot chết**, **không có chồng lấn thân**.
> Đã xác nhận thêm bằng một run **2 giờ mô phỏng**: vẫn 0 robot chết, pin thấp nhất giữ ~56%.

### 6.3 Khả năng mở rộng (5 và 30 robot, giữ nguyên 10 ô sạc cố định)

| Số robot | Đơn giao | Sát nhất robot | Sát nhất tường | Robot chết |
|---:|---:|---:|---:|:---:|
| 5 | 106 | 0.172 m | 0.110 m | 0 |
| 10 (chuẩn) | 267–361 | ≥ 0.170 m | ≥ 0.083 m | 0 |
| 30 | 446 | 0.167 m | 0.083 m | 0 |

Từ 5 → 10 robot throughput tăng gần **×3** (chưa chạm trần); từ 10 → 30 chỉ tăng ~40% vì
**3 băng tải + 10 ô sạc cố định** đã bão hoà — đúng đường cong bão hoà hạ tầng của kho thật.
Dù 30 robot chia nhau 10 ô sạc (quá tải ×3), vẫn **0 robot chết**, không kẹt cứng.
**Không cần đổi một dòng code**, chỉ đổi tham số `.argos`.

---

## 7. Tham chiếu tham số

Tất cả trong [`experiments/warehouse.argos`](experiments/warehouse.argos).

### 7.1 Kho và luồng hàng

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `spawn_period` | 36 tick | Khoảng giữa 2 kiện ra băng (3.6 s) |
| `queue_cap` | 6 | Sức chứa mỗi bin |
| `pickup_radius` | 0.35 m | Bán kính nhận hàng tại bin |
| `zone_half` | 0.4 m | Nửa cạnh ô địa chỉ |
| `dock_rows`/`cols`/`spacing_x`/`spacing_y` | 5 / 2 / 0.55 / 6.9 | Lưới bãi đỗ: 2 hàng (trái y>0, phải y<0), 5 ô/hàng |
| `pillar_radius` / `pillar_range` | 0.075 / 0.8 m | Cột & tầm trường xoáy né cột |

### 7.2 Chuyển động và né va chạm

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `max_speed` | 36 cm/s | Tốc độ robot |
| `separation` target/gain/exp | 45 / 800 / 2 | Lennard-Jones tách đàn |
| `obstacle_gain` / `hard_avoid_threshold` | 200 / 0.04 | Né vật cản proximity |
| `belt_sense_range` | 1.2 m | Tầm cảm nhận **thật** trạng thái băng (khác tầm nghe RAB) |
| Trọng số đấu giá `W_QUEUE`/`W_INBOUND`/`W_DISTANCE` | 1.5 / 1.0 / 0.35 | Trong `footbot_task_allocation.cpp` |
| `HYSTERESIS` / `DECISION_LOCK` | 0.8 / 30 tick | Ngưỡng đổi băng & thời gian khoá quyết định (chống lưỡng lự robot đang làm) |
| `PATROL_COOLDOWN` | 250 tick | Thời gian robot dư nghỉ giữa 2 chuyến do thám (trong `footbot_warehouse.cpp`, lệch pha theo ID) |

### 7.3 Năng lượng và stigmergy

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `<battery> full_charge`/`start_charge` | 6.0 | Dung lượng ×6 mặc định ARGoS; `discharge_model="time" delta=0` (không dùng model nội bộ — xem §5.6) |
| `drain_move`/`drain_time` (warehouse) | 0.008 / 2e-5 | Xả pin tự tính: theo mét đi / theo tick (thay cho `pos_factor`/`time_factor` cũ) |
| `<energy> resume_charge`/`min_work_charge` | 0.70 / 0.70 | Sạc tới ≥70% mới rời trạm / nhận việc mới |
| `<energy> hard_charge_threshold` | 0.10 | Pin < 10% buộc về sạc (việc đang dở giữ tới sàn 25%) |
| `charge_rate` / `charge_warmup` | 0.002667 / 50 | Tốc độ nạp mỗi tick / số tick bắt tay trước khi có điện |
| `stigmergy_decay` / `stigmergy_gain` | 0.998 / 1.0 | Tốc độ phai vệt sàn / lượng cộng khi có robot mới đỗ |
| `rab_data_size` | 14 | Payload byte: state+id+belt+dist+addr+dock+rescuing (8) + gossip 3 băng×2 (6) |

---

## 8. Kiến trúc mã nguồn (controller và loop functions)

Mỗi class được tách thành **nhiều file `.cpp` theo tập tính chức năng** (dùng chung 1 header),
để sửa một tính năng chỉ cần mở đúng một file nhỏ. Tách thuần cơ học — đã kiểm chứng cho
kết quả **giống hệt bit-for-bit** trước khi tách.

```
warehouse-swarm/
├── controllers/footbot_warehouse/         # ── NÃO ROBOT ──
│   ├── footbot_warehouse.h                #  1 header: mọi state + chữ ký hàm, có banner phân nhóm
│   ├── footbot_warehouse.cpp              #  vòng đời (Init/Reset) + máy trạng thái ControlStep
│   ├── footbot_navigation.cpp             #  trường thế: mục tiêu, tách đàn, né cột, né vật cản
│   ├── footbot_comms.cpp                  #  gói tin RAB + tin đồn băng tải (gossip)
│   ├── footbot_task_allocation.cpp        #  đấu giá chọn băng + CHỐNG LƯỠNG LỰ
│   ├── footbot_docking.cpp                #  nhận ô đỗ + stigmergy
│   └── footbot_rescue.cpp                 #  phát hiện kẹt + cứu hộ 2 cấp
├── loop_functions/warehouse_loop_functions/  # ── NHÀ XƯỞNG ──
│   ├── warehouse_loop_functions.cpp       #  vòng đời + PreStep điều phối
│   ├── warehouse_floor_render.cpp         #  tô sàn: bãi đỗ + stigmergy, ô địa chỉ
│   ├── warehouse_spawning.cpp             #  spawn kiện, ground truth băng, handover
│   ├── warehouse_robot_update.cpp         #  1 vòng/tick: xả pin + sạc + dock + stigmergy, metric, giao hàng
│   ├── warehouse_metrics.cpp              #  va chạm, khoảng hở tường
│   └── warehouse_qt_user_functions.cpp    #  vẽ: kiện trong bin, kiện trên lưng, chữ A–E, bộ đếm
└── experiments/warehouse.argos            #  layout kho, 10 robot, toàn bộ tham số
```

**Ranh giới trách nhiệm:** *controller* = "não" một robot (chỉ thấy cảm biến của mình +
tin RAB); *loop functions* = "nhà xưởng" (spawn kiện, phát hiện handover/giao hàng, tô sàn,
xả/sạc pin, thống kê). Loop functions gọi vào controller qua vài hàm API (`SetBeltGroundTruth`,
`AssignItem`, `Deliver`, `WantsWork`, `IsCarrying`...), không bao giờ đọc trạng thái nội bộ.

---

## 9. Lộ trình phát triển

- [ ] **Điều hướng theo lưới (grid + A\*/reservation)** — tầng lập lộ trình toàn cục đặt trên
  trường thế vi mô hiện tại (giống fleet AMR Kiva/Amazon thật). Thay đổi kiến trúc lớn nhất.
  Xem dự án chị em [`grid-swarm-robot`](../grid-swarm-robot/README.md) đã hiện thực hướng này.
- [ ] **Định vị thật** thay `<positioning>` lý tưởng — cố ý hoãn: rủi ro cao, đụng toàn bộ
  hệ điều hướng đã ổn định.
- [ ] Kiện ưu tiên (express) — trọng số riêng trong hàm utility.
- [ ] Kệ hàng làm vật cản giữa kho — thử trường thế với local minima phức tạp hơn.
- [x] ~~Trạm sạc + quản lý pin~~ · ~~sửa bug NaN pin~~ · ~~gossip thay oracle~~ · ~~stigmergy né đông~~ ·
  ~~test quy mô 5–30 robot~~ · ~~diệt lưỡng lự~~ — **đã xong**.
