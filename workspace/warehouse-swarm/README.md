# Warehouse Swarm

> Mô phỏng **đội robot AMR phi tập trung** vận hành kho hàng kiểu Amazon trên ARGoS3 —
> không có máy chủ điều phối, mọi hành vi tập thể tự nổi lên từ luật cục bộ mỗi robot cùng chạy.

<p>
<strong>10 robot</strong> · <strong>8×8 m</strong> · <strong>3 băng tải → 5 ô giao hàng</strong> ·
điều hướng trường thế · đấu giá phân việc · tin đồn cục bộ · stigmergy · quản lý pin ·
bảng điều khiển vận hành
</p>

> 🎬 **Video demo:** [xem đội robot vận hành](../../demo/warehouse_free_space.mp4)
> *(mp4 · 8 MB — GitHub phát trực tiếp trong trình duyệt)*

---

## Mục lục

1. [Kho hàng phi tập trung: 10 robot, 3 băng tải, 5 ô giao](#1-kho-hàng-phi-tập-trung-10-robot-3-băng-tải-5-ô-giao)
2. [Build và chạy mô phỏng](#2-build-và-chạy-mô-phỏng)
3. [Giải mã màn hình mô phỏng](#3-giải-mã-màn-hình-mô-phỏng)
4. [Bảng điều khiển vận hành (operator console)](#4-bảng-điều-khiển-vận-hành-operator-console)
5. [Máy trạng thái điều khiển robot](#5-máy-trạng-thái-điều-khiển-robot)
6. [Ngăn xếp thuật toán bầy đàn](#6-ngăn-xếp-thuật-toán-bầy-đàn)
7. [Kiểm chứng throughput, an toàn và mở rộng](#7-kiểm-chứng-throughput-an-toàn-và-mở-rộng)
8. [Tham chiếu tham số](#8-tham-chiếu-tham-số)
9. [Kiến trúc mã nguồn (controller và loop functions)](#9-kiến-trúc-mã-nguồn-controller-và-loop-functions)
10. [Lộ trình phát triển](#10-lộ-trình-phát-triển)

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

**Bảy trụ cột hành vi**

| | |
|---|---|
| 🧭 **Điều hướng** | Trường thế nhân tạo (mục tiêu + tách đàn + né cột + né vật cản) |
| 🏷️ **Phân việc** | Đấu giá thị trường phi tập trung, **chống lưỡng lự** (sticky commitment) |
| 📡 **Thông tin** | Cảm nhận cục bộ + tin đồn lan truyền (epidemic gossip), không oracle toàn tri |
| 🐜 **Stigmergy** | Vệt "mùi" trên sàn để né đông bãi đỗ, đọc bằng cảm biến sàn thật |
| 🔋 **Năng lượng** | Pin thật, tự về sạc, sạc cơ hội, ưu tiên đường cho robot yếu |
| 🛟 **An toàn** | Né va chạm 2 tầng + cứu hộ thoát kẹt 2 cấp — **0 robot chết** qua mọi test |
| 🎛️ **Vận hành** | Bảng điều khiển operator: e-stop / triệu hồi / trạng thái từng robot ([§4](#4-bảng-điều-khiển-vận-hành-operator-console)) |

---

## 2. Build và chạy mô phỏng

```bash
# 1. Mở môi trường (container ARGoS3) — chạy từ thư mục gốc repo
./run.sh

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
| **LED** robot | 🟢 rảnh · 🟡 đi tuần tra băng · 🔵(cyan) đến băng lấy hàng · 🔴 giao hàng · 🔵 đi/đang sạc · 🟠 bị E-STOP (console) · ⚫ hết pin |
| Thanh pin trên đầu robot | Mức pin (xanh >50% · cam >25% · đỏ ≤25% · lam = đang sạc) |
| Tia nối các robot | Liên kết giao tiếp RAB |
| 5 trụ tròn (1 giữa + 4 góc) | Cột kết cấu — robot tự vòng qua |

---

## 4. Bảng điều khiển vận hành (operator console)

GUI có sẵn một **bảng điều khiển kiểu fleet-manager thật** vẽ đè lên khung nhìn (HUD,
góc trái trên): người vận hành dừng khẩn cấp, triệu hồi về sạc và kiểm tra trạng thái
làm việc của từng robot. Đây là **lớp lệnh của con người** đè lên tự hành — đúng vai
trò e-stop/recall của kho AMR thật — **không phải điều phối trung tâm**: dưới chế độ
TỰ ĐỘNG robot vẫn tự đấu giá, tự điều hướng 100%. Chạy headless không nạp GUI nên
mọi benchmark không bị ảnh hưởng bởi console.

### 4.1 Lệnh và cách tương tác

| Lệnh | Nút trên HUD | Phím | Tác dụng |
|---|---|:---:|---|
| **E-STOP toàn đàn** | `E-STOP` | `X` | Mọi robot đóng băng tại chỗ (vẫn phát RAB để đồng đội né); việc đang dở được giữ nguyên |
| **Chạy lại toàn đàn** | `CHẠY LẠI` | `R` | Cả đàn về TỰ ĐỘNG — robot tiếp tục đúng việc bị đóng băng |
| **Triệu hồi toàn đàn** | `VỀ SẠC` | `H` | Giao nốt kiện đang cầm (không bao giờ vứt hàng), rồi về ô đỗ và **ở lại** tới khi được thả |
| **Kiểm tra trạng thái robot** | `KIỂM TRA TRẠNG THÁI` | `T` | Bật/tắt bảng theo dõi từng robot: chế độ, trạng thái, % pin, kiện đang cầm |
| **Chọn 1 robot** | — | `Shift+Click` | Chọn robot; hàng nút `DỪNG / VỀ SẠC / TỰ ĐỘNG` (phím `1/2/3`) áp cho đúng robot đó |

Phím camera của ARGoS (`W/A/S/D/Q/E`, chuột kéo) không bị chiếm — console chỉ dùng
`X/R/H/T/1/2/3`.

### 4.2 Ba chế độ override và bảo chứng an toàn

| Chế độ | LED | Hành vi |
|---|:---:|---|
| `TỰ ĐỘNG` | theo trạng thái | Tự hành hoàn toàn (mặc định — headless chỉ có chế độ này) |
| `DỪNG` (e-stop) | 🟠 | Đứng im, **không nhận kiện tại bin, không phát claim băng** (không làm đồng đội né việc ảo); giữ nguyên việc để tiếp tục khi được thả |
| `VỀ SẠC` (recall) | 🔵 | Đi theo đúng luồng sạc hiện có, nhưng bị giữ ở bãi qua cả ngưỡng 70% tới khi operator thả |

Hiện thực: controller thêm đúng một lớp `EOverride` kiểm tra trước logic tự hành
([`footbot_warehouse.cpp`](controllers/footbot_warehouse/footbot_warehouse.cpp));
toàn bộ HUD/phím/chuột nằm trong
[`warehouse_qt_user_functions.cpp`](loop_functions/warehouse_loop_functions/warehouse_qt_user_functions.cpp).

---

## 5. Máy trạng thái điều khiển robot

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

## 6. Ngăn xếp thuật toán bầy đàn

### 6.1 Điều hướng bằng trường thế nhân tạo

Mỗi tick, vector điều khiển = tổng của **mục tiêu + tách đàn + né cột + né vật cản**,
rồi quy đổi ra tốc độ 2 bánh (theo ví dụ flocking chính thức của ARGoS).

| Thành phần | Thuật toán | Ghi chú |
|---|---|---|
| Hút về mục tiêu | Bearing-only proportional | lái theo góc lệch tới đích |
| Tách đàn | **Lennard-Jones** (chỉ phần đẩy) qua RAB | giữ khoảng cách 45 cm |
| Né cột | **Trường xoáy (vortex)**: đẩy xa + tiếp tuyến cùng chiều | cả đàn vòng cột như bùng binh, không kẹt local-minimum |
| Né vật cản | Proximity phản xạ | lưới an toàn tầm gần |

### 6.2 Né va chạm theo quyền ưu tiên (right-of-way)

Hai robot gặp nhau **không cùng né** (né đối xứng = lãng phí). Thứ bậc:
`đang sạc > đang giao (chở hàng) > đi lấy hàng > rảnh`, ngang cấp thì so ID.
Con nhường nhân trọng số **1.85**, con ưu tiên **0.15** (tổng = 2.0, giữ nguyên tổng
lực). Robot chở hàng đi thẳng như xe ưu tiên; robot đang cứu hộ được nhường tuyệt đối.

### 6.3 Phân việc đấu giá thị trường và diệt lưỡng lự

Mỗi robot tự chấm điểm từng băng và chọn băng tốt nhất — **không có bộ điều phối**:

```
U(b) = 1.5·hàng_chờ(b) − 1.0·số_robot_đến_trước(b) − 0.35·khoảng_cách(b)·(2−pin) + nhiễu
```

- Chỉ đếm robot **gần băng hơn mình** ⇒ cả đàn tự xếp thành hàng đợi ngầm theo khoảng cách.
- Nhân `(2−pin)` ⇒ robot yếu thấy việc xa "đắt" hơn, tự nhường cho đồng đội khỏe.
- Nhiễu nhỏ phá thế đối xứng (10 con giống hệt không cùng chọn một băng).

> #### 🎯 Ổn định quyết định: cam kết dính và quân dự bị
>
> Đấu giá phi tập trung có một chế độ hỏng kinh điển: nếu robot **tái lượng giá liên
> tục** trên thông tin gossip vốn nhấp nháy, cả đàn dao động theo nhau — robot rời dock
> vì một tin đồn, tin đổi chiều, quay về, rồi lại rời đi, không giao được kiện nào.
> Thiết kế ở đây loại bỏ dao động **về mặt cấu trúc** (không dựa vào tinh chỉnh tham số),
> bằng hai nguyên tắc:
>
> **① Cam kết dính (sticky commitment)** ([`footbot_task_allocation.cpp`](controllers/footbot_warehouse/footbot_task_allocation.cpp)):
> đã nhận băng thì **không bao giờ quay về dock** vì một dao động utility nhỏ. Chỉ bỏ
> việc khi có **sự thật cứng** (hết pin / băng bị chặn / tới đủ gần tự thấy bin rỗng).
> Đổi sang **băng khác tốt hơn rõ rệt** thì vẫn được phép — cân bằng tải là hành vi có
> ích, khác bản chất với dao động ra-vào dock. Mỗi quyết định được giữ tối thiểu 3 giây
> (**decision lock**).
>
> **② Quân dự bị nghỉ tại chỗ** ([`footbot_comms.cpp`](controllers/footbot_warehouse/footbot_comms.cpp)):
> robot mà thị trường xác định là "chưa cần" (luôn có đồng đội gần băng hơn) không được
> phép tuần tra vô tận theo vòng lặp *đi do thám → tin lại cũ → do thám tiếp*. Hai luật:
> (a) **chỉ tuần tra khi thực sự "mù"** — mọi băng đều hết hạn thông tin; còn dù một băng
> có tin tươi thì cứ đấu giá trên đó, không cần đi đâu; (b) **patrol cooldown ~25 s**
> giữa hai chuyến do thám, lệch pha theo ID để cả đàn không do thám đồng loạt. Robot dư
> vì vậy **đứng yên nghỉ tại ô đỗ** và chỉ kích hoạt khi tin đồn báo có việc — đúng vai
> trò quân dự bị theo nhu cầu.
>
> Ablation ở [§7.1](#7-kiểm-chứng-throughput-an-toàn-và-mở-rộng): so với biến thể tái
> lượng giá liên tục, hai nguyên tắc này cho throughput **~3×** và phương sai giữa các
> seed hẹp hơn hẳn.

### 6.4 Thông tin băng tải: cảm nhận cục bộ cộng tin đồn

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

> Ba dòng đầu của bảng tương ứng ba cạm bẫy kinh điển của giao thức gossip — gói mặc
> định toàn-0 đầu độc niềm tin, hai node "làm mới" tin cũ cho nhau, và sai số tuổi dồn
> qua mỗi chặng relay. Phân tích chi tiết từng cạm bẫy nằm trong comment của
> [`footbot_comms.cpp`](controllers/footbot_warehouse/footbot_comms.cpp).

### 6.5 Stigmergy né đông bãi đỗ

Loop functions tô sàn quanh mỗi bên bãi đỗ **đậm dần theo lượng robot vừa đỗ** (tăng khi
có robot mới đỗ, **phai dần mỗi tick** — đúng chất vệt pheromone của kiến). Robot đọc lại
độ đậm bằng **cảm biến sàn thật** (`footbot_motor_ground`) và cộng một khoản phạt mềm vào
điểm chọn ô ở bên đang đông. Vì là stigmergy, nó **chỉ tác dụng khi robot đã ở gần bên đó**
— không thể ảnh hưởng quyết định từ xa.

### 6.6 Năng lượng và cạm bẫy NaN của time_motion

Pin dùng entity pin của ARGoS làm **kho lưu trữ**, còn việc **xả pin tự tính trong
loop functions** (theo quãng đường + hao nền mỗi tick) — xem hộp cảnh báo bên dưới về lý
do. Chính sách **ngưỡng cứng**, không dự đoán khoảng cách:

- Pin **< 10%** (và tay không) → buộc về sạc; đã vào chu kỳ sạc thì **≥ 70%** mới được
  rời trạm (`resume_charge` — hysteresis của riêng chu kỳ sạc).
- **Nhận việc MỚI** chỉ cần pin ≥ **30%** (`min_work_charge`), việc đang dở giữ tới sàn
  25% — robot làm việc liên tục suốt dải 30–100%, không có "dải cấm việc".
- **Sạc cơ hội:** robot rảnh chờ ở dock vẫn được nạp, không cần đợi tới ngưỡng khẩn.
- **Warm-up 5 s** khi vào ô (bắt tay) — có kẹp sàn bảo vệ 1% để robot không chết mid-handshake.
- **Ưu tiên đường** cao nhất cho robot đi sạc; pin khởi đầu ngẫu nhiên 55–95% để so le chu kỳ sạc.

> #### ⚠️ Cạm bẫy cấu hình: không đặt `min_work_charge` = `resume_charge`
>
> Hai ngưỡng này mang ngữ nghĩa khác nhau và không được phép trộn lẫn: 70% thuộc về
> **rời trạm sau một chu kỳ sạc** (`resume_charge`); nếu nhân đôi nó sang **ngưỡng nhận
> việc mới** (`min_work_charge`), nó trở thành lệnh **cấm làm việc toàn cục dưới 70%** —
> robot giao xong một kiện, tụt dưới ngưỡng, từ chối mọi việc và về dock ngồi sạc dù
> hàng chất đống (pin khởi tạo 55–95% nên gần nửa đàn bất động ngay từ đầu). Dấu hiệu
> nhận biết khi đo: "pin thấp nhất" không bao giờ xuống dưới ~55%. Controller **cảnh báo
> lúc khởi động** nếu `min_work_charge > 0.5` để chặn cấu hình này.

> #### ⚠️ Lỗi đã biết của ARGoS: `discharge_model="time_motion"` sinh NaN
>
> Model xả pin `time_motion`/`motion` của bản ARGoS đi kèm gọi `acos(1+ε)` khi robot
> **xoay** → trả về **NaN**, và `Max(0, NaN)` giữ NaN nên số đo pin hỏng **vĩnh viễn**
> (robot chết đứng tại dock). Điểm hiểm: **mọi thống kê bị NaN vô hiệu hoá âm thầm** —
> so sánh với NaN luôn cho `false`, nên `dead robot-ticks` không tăng và "pin thấp nhất"
> đóng băng ở giá trị hợp lệ cuối cùng; benchmark headless vẫn báo "0 chết" trong khi
> robot hỏng thật trên GUI.
>
> Vì vậy dự án **không dùng model xả nội bộ**: entity pin chỉ là kho lưu trữ
> (`<battery discharge_model="time" delta="0.0">`), còn loop functions tự trừ pin mỗi
> tick theo `drain_time + drain_move × quãng_đường`
> (xem [`warehouse_robot_update.cpp`](loop_functions/warehouse_loop_functions/warehouse_robot_update.cpp)).

### 6.7 Cứu hộ thoát kẹt hai cấp

Theo dõi **khoảng cách-tới-đích** (không phải quãng đường thô — thứ này không phân biệt
được "đang tiến" với "đang dạt qua lại"). Nếu 2 cửa sổ 3 giây liền không rút ngắn được:

- **Cấp 1 (nhẹ):** lao theo hướng ngẫu nhiên 1.5 s (phá thế đối đầu 2 robot).
- **Cấp 2 (sâu):** sau nhiều lần cấp 1 thất bại — **lùi thẳng** rồi lao ra 3 s, phát cờ
  "đang cứu hộ" để cả đàn nhường đường tuyệt đối.

---

## 7. Kiểm chứng throughput, an toàn và mở rộng

Tất cả đo **headless, 30 phút mô phỏng/seed**. Bán kính thân robot 0.085 m ⇒ khoảng cách
tâm-tâm ≥ 0.17 m nghĩa là **không chồng lấn thân**.

### 7.1 Ablation: chính sách cam kết trong đấu giá (3 seed đối chứng)

Cùng một cấu hình, chỉ thay chính sách quyết định — biến thể **tái lượng giá liên tục**
(mỗi chu kỳ đấu giá đều được phép đổi ý) so với **cam kết dính + quân dự bị** ([§6.3](#6-ngăn-xếp-thuật-toán-bầy-đàn)):

| Seed | Tái lượng giá liên tục | Cam kết dính + quân dự bị |
|---:|---:|---:|
| 1 | 88 | **267** |
| 2 | 172 | **334** |
| 3 | 233 | **361** |
| **Trung bình** | 164 | **321 (×2)** |

Biến thể tái lượng giá không chỉ kém mà còn **bất định** (phương sai 88–233 giữa các
seed): throughput phụ thuộc may rủi của các vòng dao động. Cam kết dính vừa nhanh vừa ổn định.

### 7.2 Cấu hình hiện hành (5 seed × 30 phút, 10 robot)

| Seed | Đơn giao | Va chạm (pair-tick) | Sát nhất robot | Pin thấp nhất | Robot chết |
|---:|---:|---:|---:|---:|:---:|
| 1 | 366 | 1 | 0.181 m | 27.8% | 0 |
| 2 | 350 | 0 | 0.194 m | 52.5% | 0 |
| 3 | 357 | 4 | 0.177 m | 45.7% | 0 |
| 4 | 352 | 0 | 0.182 m | 48.0% | 0 |
| 5 | 353 | 0 | 0.183 m | 59.9% | 0 |

> Va chạm pair-tick (≤ 4 trên 18 000 tick) là cọ nhẹ thoáng qua khi giao thông đông,
> không phải đâm cứng: mọi seed đều **0 robot chết**, khoảng cách tâm-tâm nhỏ nhất
> ≥ 0.177 m (> 0.17 m = không chồng lấn thân). Pin thấp nhất 27.8% cho thấy đội xe
> khai thác trọn dải làm việc 25–100% thay vì co cụm quanh mức sạc đầy.

### 7.3 Khả năng mở rộng (5 và 30 robot, giữ nguyên 10 ô sạc cố định)

| Số robot | Đơn giao | Sát nhất robot | Sát nhất tường | Robot chết |
|---:|---:|---:|---:|:---:|
| 5 | 203 | 0.210 m | 0.175 m | 0 |
| 10 (chuẩn) | 350–366 | ≥ 0.177 m | ≥ 0.084 m | 0 |
| 30 | 481 | 0.168 m | 0.084 m | 0 |

Trần nguồn cung là ~500 kiện/30 phút (`spawn_period` = 3.6 s/kiện): 10 robot đạt ~71%
trần, 30 robot đạt ~96% — đường cong bão hoà hạ tầng đúng như kho thật, nghẽn nằm ở
băng tải chứ không phải đội xe. Dù 30 robot chia nhau 10 ô sạc (quá tải ×3), vẫn
**0 robot chết**, không kẹt cứng. Đổi quy mô chỉ cần sửa tham số `.argos`, không đổi code.

---

## 8. Tham chiếu tham số

Tất cả trong [`experiments/warehouse.argos`](experiments/warehouse.argos).

### 8.1 Kho và luồng hàng

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `spawn_period` | 36 tick | Khoảng giữa 2 kiện ra băng (3.6 s) |
| `queue_cap` | 6 | Sức chứa mỗi bin |
| `pickup_radius` | 0.35 m | Bán kính nhận hàng tại bin |
| `zone_half` | 0.4 m | Nửa cạnh ô địa chỉ |
| `dock_rows`/`cols`/`spacing_x`/`spacing_y` | 5 / 2 / 0.55 / 6.9 | Lưới bãi đỗ: 2 hàng (trái y>0, phải y<0), 5 ô/hàng |
| `pillar_radius` / `pillar_range` | 0.075 / 0.8 m | Cột & tầm trường xoáy né cột |

### 8.2 Chuyển động và né va chạm

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `max_speed` | 36 cm/s | Tốc độ robot |
| `separation` target/gain/exp | 45 / 800 / 2 | Lennard-Jones tách đàn |
| `obstacle_gain` / `hard_avoid_threshold` | 200 / 0.04 | Né vật cản proximity |
| `belt_sense_range` | 1.2 m | Tầm cảm nhận **thật** trạng thái băng (khác tầm nghe RAB) |
| Trọng số đấu giá `W_QUEUE`/`W_INBOUND`/`W_DISTANCE` | 1.5 / 1.0 / 0.35 | Trong `footbot_task_allocation.cpp` |
| `HYSTERESIS` / `DECISION_LOCK` | 0.8 / 30 tick | Ngưỡng đổi băng & thời gian khoá quyết định (chống lưỡng lự robot đang làm) |
| `PATROL_COOLDOWN` | 250 tick | Thời gian robot dư nghỉ giữa 2 chuyến do thám (trong `footbot_warehouse.cpp`, lệch pha theo ID) |

### 8.3 Năng lượng và stigmergy

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `<battery> full_charge`/`start_charge` | 6.0 | Dung lượng ×6 mặc định ARGoS; `discharge_model="time" delta=0` (không dùng model nội bộ — xem §6.6) |
| `drain_move`/`drain_time` (warehouse) | 0.008 / 2e-5 | Xả pin tự tính: theo mét đi / theo tick (thay cho `pos_factor`/`time_factor` cũ) |
| `<energy> resume_charge` | 0.70 | Đã vào chu kỳ sạc thì sạc tới ≥70% mới rời trạm (hysteresis của riêng chu kỳ sạc) |
| `<energy> min_work_charge` | 0.30 | Mức an toàn nhỏ để nhận việc MỚI — **không đặt cao** (>0.5 sẽ bị cảnh báo lúc khởi động, xem §6.6) |
| `<energy> hard_charge_threshold` | 0.10 | Pin < 10% buộc về sạc (việc đang dở giữ tới sàn 25%) |
| `charge_rate` / `charge_warmup` | 0.002667 / 50 | Tốc độ nạp mỗi tick / số tick bắt tay trước khi có điện |
| `stigmergy_decay` / `stigmergy_gain` | 0.998 / 1.0 | Tốc độ phai vệt sàn / lượng cộng khi có robot mới đỗ |
| `rab_data_size` | 14 | Payload byte: state+id+belt+dist+addr+dock+rescuing (8) + gossip 3 băng×2 (6) |

---

## 9. Kiến trúc mã nguồn (controller và loop functions)

Mỗi class được tách thành **nhiều file `.cpp` theo tập tính chức năng**, dùng chung một
header duy nhất — sửa một tính năng chỉ cần mở đúng một file nhỏ, và ranh giới giữa các
tập tính hiện rõ ngay trên cây thư mục.

```
warehouse-swarm/
├── controllers/footbot_warehouse/         # ── NÃO ROBOT ──
│   ├── footbot_warehouse.h                #  1 header: mọi state + chữ ký hàm, có banner phân nhóm
│   ├── footbot_warehouse.cpp              #  vòng đời (Init/Reset) + máy trạng thái ControlStep
│   ├── footbot_navigation.cpp             #  trường thế: mục tiêu, tách đàn, né cột, né vật cản
│   ├── footbot_comms.cpp                  #  gói tin RAB + tin đồn băng tải (gossip)
│   ├── footbot_task_allocation.cpp        #  đấu giá chọn băng + cam kết dính
│   ├── footbot_docking.cpp                #  nhận ô đỗ + stigmergy
│   └── footbot_rescue.cpp                 #  phát hiện kẹt + cứu hộ 2 cấp
├── loop_functions/warehouse_loop_functions/  # ── NHÀ XƯỞNG ──
│   ├── warehouse_loop_functions.cpp       #  vòng đời + PreStep điều phối
│   ├── warehouse_floor_render.cpp         #  tô sàn: bãi đỗ + stigmergy, ô địa chỉ
│   ├── warehouse_spawning.cpp             #  spawn kiện, ground truth băng, handover
│   ├── warehouse_robot_update.cpp         #  1 vòng/tick: xả pin + sạc + dock + stigmergy, metric, giao hàng
│   ├── warehouse_metrics.cpp              #  va chạm, khoảng hở tường
│   └── warehouse_qt_user_functions.cpp    #  vẽ kiện/pin/bộ đếm + BẢNG ĐIỀU KHIỂN VẬN HÀNH (HUD)
└── experiments/warehouse.argos            #  layout kho, 10 robot, toàn bộ tham số
```

**Ranh giới trách nhiệm:** *controller* = "não" một robot (chỉ thấy cảm biến của mình +
tin RAB); *loop functions* = "nhà xưởng" (spawn kiện, phát hiện handover/giao hàng, tô sàn,
xả/sạc pin, thống kê). Loop functions gọi vào controller qua vài hàm API (`SetBeltGroundTruth`,
`AssignItem`, `Deliver`, `WantsWork`, `IsCarrying`...), không bao giờ đọc trạng thái nội bộ.

---

## 10. Lộ trình phát triển

- **Điều hướng theo lưới (grid + A\*/reservation)** — tầng lập lộ trình toàn cục đặt trên
  trường thế vi mô hiện tại, theo mô hình fleet AMR Kiva/Amazon. Dự án chị em
  [`grid-swarm-robot`](../grid-swarm-robot/README.md) đã hiện thực trường phái này để đối chứng.
- **Định vị thực tế** thay cảm biến `<positioning>` lý tưởng (odometry + mốc sàn) — thay đổi
  đụng tới toàn bộ hệ điều hướng, cần thực hiện như một giai đoạn riêng.
- **Kiện ưu tiên (express)** — thêm trọng số ưu tiên riêng trong hàm utility đấu giá.
- **Kệ hàng giữa kho** — vật cản lớn tạo local minima phức tạp hơn cho trường thế.
