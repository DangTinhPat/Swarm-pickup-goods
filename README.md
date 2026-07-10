# Swarm Pickup Goods — ARGoS3 Warehouse Robotics Lab

> Bộ mô phỏng **robot bầy đàn (swarm) phi tập trung cho kho hàng tự động** trên ARGoS3,
> đóng gói sẵn trong Docker: clone về, `docker compose build`, và chạy — không phụ thuộc
> mạng hay cài đặt thủ công.

![ARGoS3](https://img.shields.io/badge/ARGoS-3-1f6feb)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Compose-2496ED?logo=docker&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Decentralized](https://img.shields.io/badge/control-decentralized-brightgreen)

Repo này gói **hai dự án mô phỏng bầy đàn hoàn chỉnh** cùng một **môi trường phát triển
ARGoS3 chuẩn hoá bằng Docker**. Toàn bộ điều phối là **phi tập trung** — không có máy chủ
trung tâm ra lệnh; hành vi tập thể (cân bằng tải, né va chạm, xếp hàng) tự nổi lên từ luật
cục bộ mà mỗi robot cùng chạy.

---

## Mục lục

1. [Hai dự án: điều hướng lưới và điều hướng trường thế](#1-hai-dự-án-điều-hướng-lưới-và-điều-hướng-trường-thế)
2. [Kiến trúc và phân vùng domain](#2-kiến-trúc-và-phân-vùng-domain)
3. [Yêu cầu hệ thống](#3-yêu-cầu-hệ-thống)
4. [Clone và khởi chạy](#4-clone-và-khởi-chạy)
5. [Build và chạy từng dự án](#5-build-và-chạy-từng-dự-án)
6. [Vòng đời một phiên làm việc](#6-vòng-đời-một-phiên-làm-việc)
7. [Cấu trúc thư mục](#7-cấu-trúc-thư-mục)
8. [GPU, X11 và các quyết định môi trường](#8-gpu-x11-và-các-quyết-định-môi-trường)
9. [Giấy phép và tác giả](#9-giấy-phép-và-tác-giả)

---

## 1. Hai dự án: điều hướng lưới và điều hướng trường thế

Cả hai đều mô phỏng kho hàng kiểu Amazon/Kiva với 10 robot foot-bot phi tập trung, nhưng
đại diện cho **hai trường phái điều hướng khác nhau** — tiện để so sánh trực tiếp:

| Dự án | Mô hình điều hướng | Điểm nhấn | Tài liệu |
|---|---|---|---|
| 🟦 **Grid Swarm Robot** | **Lưới rời rạc + MAPF** (đặt chỗ không-thời gian) | Robot tự định vị bằng "camera gầm đọc mã QR sàn", đặt chỗ ô lưới theo từng tick, đàm phán nhường đường cục bộ; phân loại hàng theo màu | [→ `workspace/grid-swarm-robot/`](workspace/grid-swarm-robot/README.md) |
| 🟩 **Warehouse Swarm** | **Trường thế liên tục** (potential field) | Đấu giá thị trường chống lưỡng lự, tin đồn cục bộ (gossip) thay oracle toàn tri, stigmergy né đông bãi đỗ, quản lý pin | [→ `workspace/warehouse-swarm/`](workspace/warehouse-swarm/README.md) |

> Mỗi dự án có README riêng rất chi tiết (thuật toán, sơ đồ, bảng benchmark, tham số). Bấm
> vào cột **Tài liệu** để đọc.

---

## 2. Kiến trúc và phân vùng domain

Kho code chia làm **hai domain tách bạch** — đây là điểm cốt lõi giúp môi trường tái lập
được trên mọi máy:

### 🐳 Domain 1 — Môi trường (Docker container)

Container đóng vai trò như một **"venv cho C++"**: chứa toolchain + ARGoS3 cài system-wide
(`/usr/local`), **không** chứa code của bạn. Dựng lại y hệt trên bất kỳ máy nào bằng một
lệnh build. GUI (Qt/OpenGL) render thẳng lên màn hình host qua X11 + GPU thật (đã xác nhận
`GL_RENDERER = RENOIR`, không phải software rendering).

### 💻 Domain 2 — Mã nguồn (host `workspace/`)

Toàn bộ code nằm trên host trong [`workspace/`](workspace/), **mount thẳng** vào container
tại `/home/argos/workspace`. Bạn sửa bằng VS Code bình thường (file thuộc user của bạn),
build/chạy bên trong container. ARGoS3 core được **vendored** (đóng kèm trong repo tại
[`workspace/argos3/`](workspace/argos3/)) nên khách chỉ cần clone là build được ngay, không
phải tải từ upstream.

### 🧩 Phân vùng bên trong mỗi dự án mô phỏng

Cả hai dự án tuân theo cùng một mô hình tách trách nhiệm của ARGoS3:

| Vùng | Vai trò | "Ai biết gì" |
|---|---|---|
| `controllers/` | **Não robot** — chạy mỗi tick, ra quyết định di chuyển | Chỉ thấy cảm biến của chính robot + tin nhắn tầm ngắn (RAB) |
| `loop_functions/` | **Nhà xưởng** — spawn hàng, phát hiện giao/nhận, tô sàn, thống kê | Thấy toàn cục (là "môi trường"), nhưng chỉ *đưa dữ liệu cảm biến*, không ra lệnh |
| `common/` *(grid)* | Thư viện dùng chung — bản đồ lưới, hình học, giao thức gói tin | Định nghĩa struct/hằng số chia sẻ giữa controller và loop functions |
| `experiments/*.argos` | Cấu hình thí nghiệm — layout kho, số robot, tham số | Khai báo XML, không có logic |

Ranh giới này được giữ nghiêm ngặt: controller **không bao giờ** đọc trạng thái toàn cục;
mọi thông tin "từ xa" đều phải đến qua cảm biến hoặc gói tin — đúng như robot thật.

---

## 3. Yêu cầu hệ thống

- **Docker** + **Docker Compose** (v2)
- **Linux** với X11 (để hiện GUI) và, nếu muốn tăng tốc, GPU có `/dev/dri`
- ~2 GB dung lượng cho image + build

Không cần cài ARGoS3, CMake, hay trình biên dịch C++ trên host — tất cả nằm trong image.

---

## 4. Clone và khởi chạy

```bash
# 1. Clone
git clone https://github.com/DangTinhPat/Swam-pickup-goods.git
cd Swam-pickup-goods

# 2. Build image môi trường (chỉ 1 lần, ~2 phút)
docker compose build

# 3. Mở container + cấp quyền hiển thị GUI trên host
./run.sh
```

`run.sh` tự chạy `xhost +local:docker`, khởi động container rồi mở shell bên trong. Bạn
sẽ vào thẳng thư mục `~/workspace`. Từ đây build và chạy một dự án bất kỳ (xem mục dưới).

> **UID/GID:** container chạy user khớp `${UID}/${GID}` trong [`.env`](.env) (mặc định
> `1000/1000`). Nếu máy bạn dùng UID khác, sửa `.env` rồi `docker compose build` lại.

---

## 5. Build và chạy từng dự án

Bên trong container (sau `./run.sh`), build và chạy theo cùng một quy trình cho cả hai dự án:

```bash
# Ví dụ với warehouse-swarm (đổi 'warehouse-swarm' thành 'grid-swarm-robot' cho dự án kia)
cd ~/workspace/warehouse-swarm
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# Chạy (từ thư mục gốc dự án — file .argos dùng đường dẫn tương đối)
cd ~/workspace/warehouse-swarm
argos3 -c experiments/warehouse.argos      # grid: experiments/grid_swarm.argos
```

Cửa sổ mô phỏng 3D hiện ra ở trạng thái tạm dừng — bấm **▶ Play**. Xem README từng dự án
để hiểu ý nghĩa hình ảnh và cách chạy headless đo hiệu năng.

---

## 6. Vòng đời một phiên làm việc

```bash
./run.sh                     # mở/attach container + bật quyền X11 + vào shell
# ... code trên VS Code (host), build & chạy trong shell container ...
exit                         # rời shell (container vẫn chạy nền)

docker compose down          # dừng hẳn container khi xong việc
xhost -local:docker          # (tuỳ chọn) thu hồi quyền X11
```

Có sẵn [`.devcontainer/`](.devcontainer/) nếu muốn dùng extension **Dev Containers** của
VS Code để mở thẳng vào container (IntelliSense C++ chạy trong container, file vẫn trên host).

---

## 7. Cấu trúc thư mục

```
Swam-pickup-goods/
├── Dockerfile              # build & cài ARGoS3 system-wide vào image
├── docker-compose.yml      # X11 forwarding + GPU passthrough + bind mount
├── entrypoint.sh           # seed workspace/argos3 lần đầu chạy
├── run.sh                  # launcher tiện dụng
├── .env                    # UID/GID host
├── .devcontainer/          # cấu hình VS Code Dev Containers (tuỳ chọn)
└── workspace/              # ── TOÀN BỘ MÃ NGUỒN (mount vào container) ──
    ├── argos3/             # ARGoS3 core vendored (build sẵn, ít khi sửa)
    ├── grid-swarm-robot/   # 🟦 Dự án 1 — điều hướng lưới + MAPF
    └── warehouse-swarm/    # 🟩 Dự án 2 — điều hướng trường thế
```

---

## 8. GPU, X11 và các quyết định môi trường

- **GPU:** `/dev/dri` được pass-through + `group_add` (video/render). Nếu máy khác có GID
  khác, kiểm tra `getent group video render` và sửa `docker-compose.yml`.
- **network_mode: host:** dùng host networking để X11 và các giao thức multi-process của
  ARGoS hoạt động không cần map port thủ công.
- **ARGoS core vendored:** đóng kèm trong repo để build offline. Nếu `src/core/` từng bị
  thiếu do `.gitignore` (một `core` pattern cũ vô tình loại cả thư mục nguồn `core/`), điều
  đó đã được sửa — thư mục nguồn ARGoS giờ được track đầy đủ.
- Chi tiết build ARGoS core, X11, và các cờ CMake nằm trong comment của
  [`Dockerfile`](Dockerfile) và [`entrypoint.sh`](entrypoint.sh).

---

## 9. Giấy phép và tác giả

- **ARGoS3** (`workspace/argos3/`) thuộc bản quyền của nhóm tác giả gốc
  ([ilpincy/argos3](https://github.com/ilpincy/argos3)), kèm theo repo dưới dạng vendored —
  xem [`workspace/argos3/UPSTREAM.txt`](workspace/argos3/UPSTREAM.txt).
- Mã của hai dự án mô phỏng (`grid-swarm-robot/`, `warehouse-swarm/`) do tác giả repo viết.
- Tác giả: **DangTinh** · [github.com/DangTinhPat](https://github.com/DangTinhPat)
