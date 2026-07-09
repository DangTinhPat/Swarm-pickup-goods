# Warehouse Swarm — Kho hàng tự động kiểu Amazon (ARGoS3)

10 robot AMR (foot-bot) **hoàn toàn phi tập trung** vận hành một kho hàng:
3 băng truyền ở tường Đông liên tục nhả kiện hàng vào thùng chứa, mỗi kiện
mang nhãn địa chỉ **a–e**; robot đến thùng lấy hàng rồi giao đến đúng ô địa
chỉ **A–E** (5 ô màu ở tường Tây). Robot rảnh chờ trong **chuồng (depot)**
ở giữa kho — nơi cả 10 con spawn lúc khởi động.

Điểm nhấn: **không có máy chủ điều phối giao việc**. Mỗi robot tự quyết
định đi băng truyền nào bằng thuật toán đấu giá cục bộ — cân bằng tải tự
nổi lên từ luật đơn giản mà mọi con cùng chạy.

**Hết việc thì tự về dock sạc chờ, có hàng thì tự đi làm tiếp** — không
cần bật thêm gì, đây là hành vi mặc định của `STATE_IDLE`: khi cả 3 băng
đều hết hàng chờ (hoặc hàng đã có robot khác gần hơn lo), robot rảnh tự
lái về một ô đỗ, được **sạc cơ hội** miễn phí trong lúc chờ (không cần
đợi tới ngưỡng 10%), và cứ mỗi vài giây tự kiểm tra lại xem có băng nào
vừa có hàng để nhận việc ngay — không cần chờ được "gọi".

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
Mỗi robot tự tính điểm cho từng băng dựa trên **niềm tin riêng** về độ dài
hàng đợi (xem mục "Cảm nhận + tin đồn băng tải" bên dưới — không còn máy
chủ WMS toàn tri phát cho cả xưởng), và nghe qua RAB **hàng xóm đang nhắm
băng nào, cách bao xa**:

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

### Mới — Pin & chính sách năng lượng (ngưỡng cứng, không dự đoán)
Mỗi robot có pin thật của ARGoS (`<battery discharge_model="time_motion">`,
`full_charge="6.0"` — dung lượng **×6 so với mặc định ARGoS**: bản đầu ×3,
lần này nhân thêm ×2 nữa) — hao theo quãng đường, góc quay và thời gian;
vật lý hao/mét không đổi, chỉ dung lượng tổng tăng nên đi được xa gấp 6
lần mới cần sạc. **Ô đỗ = trạm sạc**: đứng trên ô là được nạp
(`charge_rate=0.002667`/tick → đầy trong ~37.5 s — **giữ nguyên số này**
khi tăng dung lượng, vì tốc độ sạc là % pin/tick, không phụ thuộc dung
lượng tổng, nên thời gian sạc tự động không đổi dù pin có to hay nhỏ).
Tận dụng luôn giao thức nhận-ô qua RAB đã có. Các cơ chế:

- **Ngưỡng cứng, không dự đoán khoảng cách**: bản trước tính toán "còn đủ
  pin về trạm không" mỗi tick (dự đoán khoảng cách × hệ số an toàn); bản
  này đơn giản hơn theo đúng ý bạn — **chỉ về sạc khi pin < 10%**, không
  tính toán gì thêm. Với dung lượng ×6, 10% pin vẫn tương đương ~75 m
  quãng đường di chuyển được (đường chéo xa nhất trong kho 8×8 m chỉ
  ~11 m) — dư an toàn rất nhiều nên ngưỡng đơn giản này vẫn an toàn.
  Đang chở hàng thì giao xong mới đi.
- **Hysteresis 70/70/25%**: việc MỚI đòi pin ≥ 70%; việc đang dở giữ được
  tới sàn 25% (không camp ở thùng); **sạc tới ≥ 70% mới được rời trạm đi
  làm lại**.
- **Đấu giá biết pin**: chi phí khoảng cách nhân `(2 − pin)` — robot yếu
  thấy việc xa "đắt" hơn, tự nhường cho đồng đội khỏe.
- **Quyền ưu tiên cao nhất** cho robot đi sạc (LED xanh dương) — sắp hết
  pin được cả kho nhường đường.
- **Hết pin = chết đứng tại chỗ** (LED tắt, thanh pin đen) thành vật cản.
- Pin khởi đầu ngẫu nhiên 55–95% × dung lượng đầy — so le chu kỳ sạc cả đàn.

**Kiểm chứng ngưỡng 10% bằng số liệu, không suy đoán:**
- Cấu hình thật (×6, 90 phút, seed 7): **0 lần kích hoạt** ngưỡng 10% —
  không phải lỗi, mà vì **sạc cơ hội** (robot rảnh việc ghé dock chờ vẫn
  được nạp) đã đủ giữ pin luôn cao (thấp nhất ghi nhận ~56%), khớp đúng
  mục tiêu "giảm đi-về vô ích" của bạn — hiếm khi cần khẩn cấp về sạc.
- Để xác nhận bản thân ngưỡng hoạt động đúng, test riêng với dung lượng
  giả lập rất nhỏ (0.15) ép robot cạn pin nhanh: **kích hoạt đúng tại
  0.080–0.0997**, sát ngưỡng 0.10 như thiết kế (một trường hợp xuống tới
  0.009 vì đang giao hàng dở — theo đúng luật "không bỏ dở đơn hàng").
- Đã xoá hẳn code dự đoán khoảng cách cũ (không dùng nữa) cho code gọn.
- **Tranh chấp ô sạc giải theo QUYỀN SỞ HỮU**: con gần ô hơn giữ ô (tính
  vị trí đối phương từ range+bearing) — robot đang đỗ sạc không bao giờ
  bị con ở xa "cướp" ô qua RAB; hoà mới so ID. Robot tụt dưới ngưỡng giữ
  cũng tự nhả claim băng tải để quay về sạc thay vì cắm trại ở thùng.
- **Ghép nối chính xác (precise docking)**: cách ô < 0.35 m tắt trường
  thế (bò thẳng bằng vector mục tiêu thuần, không bị lực tách đàn của
  robot đỗ cạnh kéo lệch). Phản xạ né vẫn BẬT xuyên suốt pha bò vào —
  chỉ tắt hẳn khi đã khít < 5 cm (đứng yên tuyệt đối, không phản ứng khi
  hàng xóm đỗ cạnh). Bán kính nhận sạc 0.18 m > độ khít 5 cm nên luôn có
  biên an toàn.
- **Warm-up 5 s**: robot vào ô phải đứng yên 5 giây "bắt tay" (ô hiện màu
  cam) rồi điện mới vào (ô chuyển xanh lá); rời ô là phải bắt tay lại.
  Robot chết giữa đường tự nhả claim ô sạc (xác không được khoá trạm).
  Quyền sở hữu ô: **đã đỗ khít là bất khả xâm phạm**, tuyệt đối không ai
  cướp được (chỉ robot còn đang tiến vào mới có thể thua tranh chấp).

### Đã sửa — "Robot chết ngay trên ô sạc" (lỗi trong pha warm-up)
Bài học từ báo cáo thực tế của người dùng, tìm ra bằng đọc code chứ không
suy đoán: trong 5 giây "bắt tay" trước khi sạc thật sự bắt đầu, loop
functions **không hề nạp pin** — nhưng robot vẫn hao pin nhẹ theo thời
gian (thành phần hao-theo-tick của mô hình pin ARGoS `time_motion`, tồn
tại kể cả khi đứng yên tuyệt đối). Nếu robot về tới ô với pin chỉ nhỉnh
hơn ngưỡng chết (0.5%) một chút, nó tụt qua ngưỡng chết **ngay trong lúc
chờ bắt tay**, dù đang đứng đúng trên ô sạc — đúng như báo cáo "chết ở
dock sạc". Lỗi này **không phụ thuộc kích thước pin** (luôn tồn tại ở
vùng gần 0%, dù pin to hay nhỏ), nên các stress-test bằng pin giả lập nhỏ
trước đó không bắt được (chúng làm robot chết vì thiếu tầm di chuyển
trước khi tới nơi — một lỗi khác, không phải lỗi này); chỉ phiên chơi
thực tế dài mới đủ xác suất bắt được ca hiếm gặp này.

**Sửa**: trong pha warm-up, nếu pin sắp tụt dưới 1% thì kẹp giữ ở đúng
1% (không cho rơi tiếp) — mô phỏng việc một trạm sạc thật, dù đang trong
giai đoạn "bắt tay", vẫn không bao giờ để thiết bị cắm điện chết nguồn.
Không ảnh hưởng gì đến robot có pin khoẻ (chỉ kích hoạt khi pin sắp chạm
đáy), không làm sạc nhanh hơn hay tắt luôn warm-up.

### Mới — Cứu hộ 2 cấp cho MỌI trạng thái (không chỉ ở dock)
Bài học từ 2 lỗi thật, phát hiện qua stress-test dài chứ không phải đoán:

**Lỗi A — Livelock nhiều robot ở dock.** Khi nhiều robot cùng dồn về sạc,
phản xạ né 2 con (right-of-way) đôi khi khiến 3+ con quay vòng/dạt qua
lại vô tận quanh nhau — "có di chuyển" nhưng KHÔNG BAO GIỜ tiến gần ô đích
(livelock, khác deadlock). Sửa: robot tự đo **khoảng cách còn lại tới mục
tiêu** (không phải quãng đường di chuyển thô — thứ này không phân biệt
được "đang tiến" với "đang dạt qua lại tại chỗ") mỗi 3 giây; 2 chu kỳ liền
(6 giây) không rút ngắn được ≥ 5 cm → **cấp 1 (nhẹ)**: chọn hướng ngẫu
nhiên lao đi 1.5 giây phá cụm.

**Lỗi B — Phản xạ né khẩn cấp chiếm quyền vĩnh viễn ở góc tường/cột trụ.**
Sau khi sửa lỗi A, test vẫn thấy robot kẹt ở toạ độ đúng 4 cột trụ góc
(`±2,±2`) khi đang đi lấy/giao hàng (không phải lúc vào dock). Nguyên
nhân: phản xạ né khẩn cấp (dựa proximity) được kiểm tra **trước** và trả
về sớm mỗi tick — nếu proximity luôn vượt ngưỡng (kẹt ở góc, 2 bức tường
gần nhau khiến sensor không bao giờ "sạch"), phản xạ chiếm quyền **mọi
tick**, cơ chế thoát-kẹt (đặt sau đó) không bao giờ được gọi tới. Sửa:
đảo thứ tự — kiểm tra thoát-kẹt **trước tiên**, áp dụng cho **cả 3 trạng
thái di chuyển** (đi lấy hàng, giao hàng, vào dock), không chỉ riêng dock.

**Cấp 2 (sâu) — khi cấp 1 thất bại liên tiếp 3 lần:** đây là "cứu hộ sâu"
thật sự — **lùi thẳng 1.5 giây** trước (bẻ lái tại chỗ không gỡ được vật
kẹt sát sườn, phải lùi ra), rồi **lao theo hướng ngẫu nhiên 3 giây** để
thoát hẳn vùng kẹt, đồng thời phát cờ "đang cứu hộ" qua RAB — robot khác
tự nhường đường tuyệt đối cho robot đang cứu hộ (không được cả 2 bên cùng
né kiểu thông thường).

**Kiểm chứng bằng log, không suy đoán:** 30 phút, seed 7 — thoát-kẹt cấp 1
kích hoạt **36 lần**, đúng tại toạ độ 4 cột trụ và các góc tường, trải
đều cả 3 trạng thái (đi lấy hàng, giao hàng, rảnh/về dock); cấp 2 (sâu)
**0 lần** — cấp 1 đã đủ giải quyết mọi ca trong test này, cấp 2 vẫn sẵn
sàng cho trường hợp nặng hơn.
**Trước khi sửa lỗi A** (seed 42, 30 phút): 6299 tick va chạm, 3593 tick
robot chết đói ngay cạnh dock trống.
**Sau khi sửa cả A và B** (3 seed × 30 phút): **0 robot chết** toàn bộ,
va chạm chỉ còn 1–35 tick (chạm nhẹ thoáng qua khi cứu hộ, không phải
kẹt cứng), pin thấp nhất luôn ≥ 55% nhờ ngưỡng 70% mới.
- Kiểm chứng 20 phút × 2 seed (bản trước): 0 robot chết, pin thấp nhất ~23%,
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

### Mới — Cảm nhận + tin đồn băng tải (belt gossip) — bỏ máy chủ WMS toàn tri
Bản trước mỗi robot nhận độ dài 3 hàng đợi qua một broadcast toàn xưởng
mỗi tick — tiện nhưng **không thật**: một hệ thống bầy đàn thật không có
máy nào biết hết mọi thứ tức thời. Bản này thay bằng cảm nhận cục bộ +
tin đồn lan truyền (epidemic gossip) qua RAB, đúng cách IoT/WMS thật hoạt
động (cảm biến tại chỗ + mesh relay):

- **`SenseBelts()`**: chỉ khi ở trong `belt_sense_range` (1.2 m) của một
  băng, robot mới đọc được trạng thái THẬT (hàng đợi + có bị chặn không)
  của băng đó, đóng dấu bằng **tick tuyệt đối** lúc cảm nhận (`OriginTick`).
- **Lan truyền**: mỗi tick, robot phát niềm tin hiện tại về cả 3 băng qua
  RAB; hàng xóm chỉ **ghi đè** niềm tin của mình nếu thông tin nhận được
  mới hơn (so `OriginTick` sau khi suy ra từ gói tin) — thông tin cũ hơn
  bị bỏ qua dù ai gửi.
- **Cờ "bị chặn" đi chung một kênh với cờ cấp cứu**: robot chết/kẹt đứng
  ngay điểm nhận hàng của một băng khiến băng đó "bị chặn" — sự thật này
  được loop functions dò ra (ai đang chết + đứng đâu) rồi nạp vào field
  `Blocked` của CHÍNH struct niềm tin về băng đó, lan truyền qua đúng cơ
  chế gossip trên — không cần kênh cấp cứu riêng, vì với cả đàn "băng này
  không dùng được" là cùng một sự thật dù nguyên nhân là hàng đầy hay có
  xác chắn lối.
- **Tuần tra khi thông tin quá cũ (`NeedsBeltPatrol`)**: không có oracle
  thì phải có ai đó tự đi xem — robot rảnh (`STATE_IDLE`) nếu niềm tin về
  BẤT KỲ băng nào quá 100 tick (10 s) chưa cập nhật sẽ chủ động lái đến
  băng đó để cảm nhận trực tiếp (nhả ô đỗ đang giữ, LED vàng lúc tuần
  tra) — đây là cơ chế phá vỡ nguy cơ "bế tắc khởi động": nếu không ai
  từng đi ngang một băng, không robot nào biết nó có hàng, cả đàn đứng
  yên mãi.

**3 lỗi thật tìm ra qua debug bằng log, không suy đoán** (bám 1 robot cụ
thể qua từng tick, so niềm tin với sự thật):
1. **Gói rác đầu phiên đầu độc niềm tin**: tick đầu tiên RAB trả gói toàn
   byte 0 cho robot chưa từng phát (mặc định của ARGoS) — không phân biệt
   được với dữ liệu thật "tuổi = 0 = mới nhất", khiến cả đàn tin ngay từ
   đầu là mọi băng đều trống. Sửa: đổi chiều mã hoá, truyền "độ tươi"
   (127 − tuổi) thay vì tuổi thô — gói rác toàn 0 giờ decode ra tuổi tối
   đa (127), vô hại.
2. **Vòng lặp tự củng cố tin cũ**: 2 robot đứng cạnh nhau mãi mãi "làm
   mới" niềm tin cũ cho nhau (mỗi tick, ai vừa +1 tuổi lại nghe được số
   tuổi thấp hơn 1 tick trước từ người kia) — vì lưu "tuổi" như một số
   nguyên bị mutate liên tục. Sửa: đổi sang lưu `OriginTick` **tuyệt đối,
   chỉ được tiến, không lùi** — tuổi luôn tính lại từ tick hiện tại, đơn
   điệu, miễn nhiễm vòng lặp.
3. **Sai số dồn qua mỗi chặng relay**: RAB có đúng 1 tick độ trễ truyền,
   công thức suy ra `OriginTick` ban đầu không trừ độ trễ này → mỗi lần
   tin được relay lại, tin cũ trông "tươi hơn thật" đúng 1 tick — dồn dần
   qua nhiều chặng khiến tin rất cũ vẫn trông gần như mới. Sửa: trừ thẳng
   1 tick bù trễ khi suy ra `OriginTick` — đúng tuyệt đối bất kể số chặng.

**Kiểm chứng** (3 seed × 30 phút thật, sau khi sửa cả 3 lỗi): giao hàng
88/172/233 đơn, **0 tick robot chết** cả 3 seed, va chạm 64/8/4 pair-tick,
khoảng hở tường thấp nhất 0.083–0.114 m — không hồi quy so với hệ thống
broadcast toàn tri cũ.

### Mới — Stigmergy né đông đúc bãi đỗ (qua màu sàn thật)
Khi nhiều robot cùng dồn về MỘT bên bãi đỗ (trái hoặc phải), tắc nghẽn
cục bộ hình thành — nhưng một robot thật không thể "biết" bên kia đông
hay vắng trừ khi tự đi qua đó. Đây là stigmergy đúng nghĩa (giao tiếp
gián tiếp qua môi trường, kiểu vệt pheromone của kiến), không phải một
con số truyền thẳng vào bộ não robot:

- Loop functions giữ `m_fSideActivity[2]` (1 giá trị mỗi bên trái/phải),
  **tăng khi có robot mới vừa đỗ khít** (cạnh tăng, không tăng liên tục
  khi đã đỗ sẵn), **phai dần mỗi tick** (`stigmergy_decay=0.998` → vệt mờ
  hẳn sau vài phút không ai đỗ thêm — đúng chất "dấu vết", không phải sổ
  ghi chép vĩnh viễn).
- Vùng sàn quanh bãi đỗ được tô màu xám đậm dần theo mức hoạt động của
  bên đó (`GetFloorColor()`, qua `CFloorEntity` thật — không phải overlay
  debug) — và robot đọc lại màu này bằng **cảm biến sàn thật**
  (`footbot_motor_ground`, `rot_z_only`), qua `GroundDarkness()`.
- `ChooseDockSlot()` cộng thêm một khoảng-cách-tương-đương (không loại
  trừ cứng) vào điểm số của các ô cùng bên với vệt tối robot đang cảm
  nhận — **chỉ có tác dụng khi bánh xe robot đã ở trên vùng tô màu**
  (điều kiện `fDarkness > 0.3`), tức không thể ảnh hưởng quyết định từ xa
  — đúng bản chất cục bộ của stigmergy. Một ô gần hơn nhiều ở bên đông
  vẫn thắng nếu bên vắng ở quá xa, cơ chế này chỉ lật kèo khi 2 lựa chọn
  đã gần ngang nhau.
- Tách biệt hoàn toàn với hệ **nhận-ô qua RAB** (chống 2 robot cùng nhận
  1 ô vật lý) — stigmergy chỉ nghiêng lựa chọn "nên nhắm bên nào" trước
  khi bước tranh chấp-ô-cụ-thể đó diễn ra.

## 3b. Kiểm thử khả năng mở rộng (scalability: 5 & 30 robot)
Giữ nguyên bản đồ 8×8 m và **hạ tầng sạc cố định 10 ô** (KHÔNG scale theo
số robot — đúng thực tế: bãi sạc là chi phí đầu tư cố định, không phải
thứ tăng 1:1 theo từng robot mới mua), chỉ đổi số lượng + rải robot ngẫu
nhiên (`position method="uniform"`, hướng ngẫu nhiên) thay vì spawn sẵn
trên ô đỗ — để bài test cũng phải chạy qua đúng pha "khởi động lạnh"
(tuần tra tìm băng, lan tin đồn từ số không).

| Số robot | Giao hàng (30 phút) | Va chạm (pair-tick) | Sát nhất robot-robot | Sát nhất tường | Pin thấp nhất | Tick robot chết |
|---|---|---|---|---|---|---|
| 5 | 56 | 0 | 0.191 m | 0.145 m | 10.3% | **0** |
| 10 (chuẩn) | 88–233 (3 seed) | 4–64 | 0.166–0.175 m | 0.083–0.114 m | 30–53% | **0** |
| 30 | 492 | 303 | 0.168 m | 0.083 m | 51.8% | **0** |

Nhận xét: throughput tăng gần tuyến tính theo số robot (30 robot giao
gấp ~8× so với 5 robot dù chỉ có 3 băng và 10 ô sạc cố định — nghẽn hạ
tầng chưa tới hạn). Va chạm pair-tick tăng theo mật độ (đúng dự đoán —
30 robot chia nhau đúng 10 ô sạc, tức quá tải hạ tầng gấp 3 lần so với tỉ
lệ 1:1 ở cấu hình 10 robot) nhưng khoảng hở robot-robot và robot-tường
**không giảm** ở quy mô lớn hơn — không có va chạm cứng/kẹt cứng, né va
chạm 2 tầng + cứu hộ sâu vẫn giữ an toàn tuyệt đối. **0 tick robot chết ở
cả hai đầu quy mô** — đấu giá phân việc, né theo quyền ưu tiên, gossip
băng tải và quản lý năng lượng đều hoạt động đúng từ 5 đến 30 robot mà
không cần đổi một dòng code nào, chỉ đổi tham số `.argos`.

## 4. Tham số chính (`experiments/warehouse.argos`)

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `spawn_period` | 36 | tick giữa 2 kiện hàng ra băng (3.6 s — đã giảm ×3 tốc độ ra hàng theo yêu cầu, bản gốc 12) |
| `queue_cap` | 6 | sức chứa thùng mỗi băng |
| `pickup_radius` | 0.35 | bán kính nhận hàng tại thùng (m) |
| `zone_half` | 0.4 | nửa cạnh ô địa chỉ (m) |
| `belt0..2`, `addr_a..e`, `dock_center` | — | tọa độ trạm (khai 2 chỗ: controller & loop functions, phải khớp) |
| `dock_rows`/`dock_cols`/`dock_spacing_x`/`dock_spacing_y` | 5 / 2 / 0.55 / 6.9 | 2 hàng đỗ-sạc trái (y=+3.45) & phải (y=−3.45), 5 ô/hàng. spacing_y lớn tách 2 hàng ra 2 phía. MỖI HÀNG ĐƠN là có chủ đích: ô nằm sau lưng ô khác sẽ bị chặn lối, robot cạn pin không vào được. Đổi thì sửa cả `<distribute>` grid |
| `max_speed` | 36 | tốc độ robot (cm/s) |
| Trọng số đấu giá `W_QUEUE/W_INBOUND/W_DISTANCE` | 1.5/1.0/0.35 | trong `footbot_warehouse.cpp` |
| `pillar0..4` / `pillar_radius` / `pillar_range` | 5 cột / 0.075 / 0.8 | bản đồ cột (phải khớp các `<cylinder>` trong arena) |
| `<battery>` trên foot-bot: `pos_factor`/`time_factor` | 0.008 / 2e-5 | hao pin theo mét / theo tick (khớp `<energy>` của controller) |
| `<energy>`: `resume_charge`/`min_work_charge` | 0.70 / 0.70 | sạc tới ≥70% mới được rời trạm đi làm; việc MỚI đòi ≥70% (việc đang dở giữ được tới sàn 25%, xem `KEEP_WORKING_CHARGE` trong code) |
| `<energy>`: `hard_charge_threshold` | 0.10 | ngưỡng duy nhất, không dự đoán khoảng cách: pin < 10% buộc về sạc vô điều kiện |
| `full_charge`/`start_charge` (`<battery>`) | 6.0 | dung lượng pin ×6 so với mặc định ARGoS (1.0), thời gian sạc giữ nguyên (xem `charge_rate`) |
| `rab_data_size` (trên `<foot-bot>`) | 14 | payload byte: state+id+belt+dist+addr+dock+rescuing (8) + gossip 3 băng × 2 byte (6) — phải khớp `Broadcast()` |
| `belt_sense_range` (`<navigation>`) | 1.2 | bán kính (m) robot phải vào để CẢM NHẬN thật trạng thái 1 băng (không phải tầm nghe RAB — xem mục gossip) |
| `stigmergy_decay`/`stigmergy_gain` (warehouse) | 0.998 / 1.0 | tốc độ phai vệt mùi sàn mỗi tick / lượng cộng khi có robot mới đỗ khít một bên (xem mục stigmergy) |
| `charge_rate` (warehouse) | 0.002667 | tốc độ nạp mỗi tick (đã ×1.5 thời gian sạc so với bản gốc 0.004) |
| `charge_warmup` (warehouse) | 50 | số tick đứng trên ô trước khi điện vào (5 s) |

## 5. Kết quả benchmark cũ (seed 124, headless, pin gốc)

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

Mỗi class (não robot / nhà xưởng) tách thành **nhiều file .cpp theo tập
tính chức năng**, dùng chung 1 file header — cách tổ chức thông thường
của C++ cho class lớn, giúp sửa 1 tính năng chỉ cần mở đúng 1 file nhỏ
thay vì lội qua một file cả nghìn dòng. Đây là tách file thuần tuý
(refactor cơ học, không đổi logic) — đã kiểm chứng bằng stress-test 3
seed × 30 phút cho kết quả **giống hệt bit-for-bit** bản trước khi tách.

```
warehouse-swarm/
├── controllers/footbot_warehouse/
│   ├── footbot_warehouse.h             # 1 header cho cả class — chữ ký mọi
│   │                                   #  hàm + toàn bộ state, có banner
│   │                                   #  phân nhóm khớp từng file .cpp dưới
│   ├── footbot_warehouse.cpp           # vòng đời (Init/Reset) + state machine
│   │                                   #  điều phối ControlStep()
│   ├── footbot_navigation.cpp          # trường thế: mục tiêu, tách đàn,
│   │                                   #  né cột, né vật cản, quy đổi bánh xe
│   ├── footbot_rescue.cpp              # phát hiện kẹt + cứu hộ 2 cấp
│   ├── footbot_comms.cpp               # gói tin RAB + tin đồn băng tải
│   │                                   #  (belt gossip)
│   ├── footbot_task_allocation.cpp     # đấu giá chọn băng + vòng đời đơn hàng
│   └── footbot_docking.cpp             # nhận ô đỗ + nghiêng chọn theo
│                                       #  stigmergy
├── loop_functions/warehouse_loop_functions/
│   ├── warehouse_loop_functions.h      # 1 header cho cả class, banner khớp
│   ├── warehouse_loop_functions.*      # vòng đời (Init/Reset/Destroy) +
│   │                                   #  PreStep() điều phối
│   ├── warehouse_floor_render.cpp      # tô màu sàn: bãi đỗ+stigmergy, ô địa chỉ
│   ├── warehouse_spawning.cpp          # spawn kiện, ground truth băng tải
│   │                                   #  cho gossip, handover tại thùng
│   ├── warehouse_robot_update.cpp      # 1 vòng lặp/tick: sạc+dock+stigmergy,
│   │                                   #  metric năng lượng, phát hiện giao hàng
│   ├── warehouse_metrics.cpp           # va chạm, khoảng hở tường
│   └── warehouse_qt_user_functions.*   # vẽ: kiện trong thùng, kiện trên lưng,
│                                       #  chữ A–E, bộ đếm
└── experiments/warehouse.argos         # layout kho, 10 robot, tham số
```

## 7. Hướng mở rộng

- Kiện hàng ưu tiên (express) — trọng số riêng trong hàm utility
- Kệ hàng làm vật cản giữa kho — thử trường thế với local minima
- **Định vị thật thay vì `<positioning>` lý tưởng** — mục duy nhất CHƯA
  làm trong đợt bổ sung tập tính bầy đàn này (cố ý bỏ qua theo lựa chọn:
  đây là thay đổi kiến trúc rủi ro cao nhất, đụng tới toàn bộ hệ điều
  hướng/trường thế/né va chạm đã ổn định qua rất nhiều vòng sửa lỗi — cần
  làm riêng, không gộp chung đợt này)
- Đã xong ở đợt này: trạm sạc + mức pin, tăng quy mô 5–30 robot (mục 3b),
  cảm nhận+tin đồn thay máy chủ toàn tri, stigmergy né đông bãi đỗ
