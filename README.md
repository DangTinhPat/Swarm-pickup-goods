# ARGoS3 dev environment (Docker, Ubuntu 22.04)

Đã build và test thành công trên máy này: `argos3` chạy được ngay sau
`docker build`, và GUI (Qt/OpenGL) hiển thị được trên màn hình host qua X11
(đã xác nhận bằng GPU thật — `GL_RENDERER = RENOIR`, không phải software
rendering).

## Cấu trúc

Container chỉ là môi trường build/run (giống một cái "venv" cho C++) —
toolchain, ARGoS3 cài system-wide (`/usr/local`), không chứa code của bạn.
**Toàn bộ code nằm trong `workspace/` trên host**, mount thẳng vào container,
sửa bằng VS Code bình thường:

```
argos/
├── Dockerfile          # build & cài ARGoS3 system-wide vào image (venv)
├── entrypoint.sh        # lần đầu chạy: seed workspace/argos3/ từ image
├── docker-compose.yml   # X11 forwarding + GPU passthrough + bind mount
├── .env                 # UID/GID host (mặc định 1000/1000)
├── .devcontainer/        # để mở bằng VS Code "Dev Containers" (tuỳ chọn)
├── run.sh               # script tiện: xhost + docker compose up + attach
└── workspace/            # >>> TOÀN BỘ CODE — mở bằng VS Code <<<
    ├── argos3/            # core framework (đã build sẵn, ít khi cần sửa)
    ├── argos3-examples/    # ví dụ chính thức (foraging, diffusion, ...)
    └── <project của bạn>/  # code bầy đàn riêng bạn viết, đặt cùng cấp
```

Trong container, `workspace/` được mount vào `/home/argos/workspace` — đây
cũng là thư mục mặc định khi mở shell (`working_dir`), nên bạn `cd` tới
project con là build/chạy ngay, không cần nhớ đường dẫn dài.

## Cách dùng

### 1. Build (chỉ cần 1 lần, ~2 phút)

```bash
docker compose build
```

### 2. Chạy container + cấp quyền hiển thị GUI trên host

```bash
./run.sh
```

Script này tự chạy `xhost +local:docker` (cho phép container vẽ lên màn
hình host), khởi động container, rồi mở shell bên trong. Lần đầu chạy,
`entrypoint.sh` sẽ tự seed `workspace/argos3/` (core source) trên host —
từ đó mở thư mục `workspace/` bằng VS Code (bình thường, không cần
extension gì đặc biệt), file thuộc sở hữu user của bạn (`dvt:dvt`).

### 3. Build lại core sau khi sửa code

Trong container (qua `./run.sh` hoặc `docker compose exec argos3 bash`):

```bash
cd ~/workspace/argos3/build_simulator      # đã có sẵn, chỉ cần make lại
make -j$(nproc)
```

### 4. Thêm project mới (ví dụ: argos3-examples, hoặc code bầy đàn của bạn)

Vì ARGoS3 đã cài system-wide trong image, mọi project con chỉ cần
`find_package(ARGoS)` / `pkg-config`, không cần biết gì tới `argos3/src`:

```bash
cd ~/workspace
git clone https://github.com/ilpincy/argos3-examples.git
cd argos3-examples
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
```

Chạy thử một thí nghiệm bầy đàn (chạy từ thư mục gốc project, vì `.argos`
tham chiếu đường dẫn tương đối `build/controllers/...`):

```bash
cd ~/workspace/argos3-examples
argos3 -c experiments/diffusion_1.argos
```

Cửa sổ mô phỏng 3D sẽ hiện ra trực tiếp trên desktop host (đã test thành
công — window "ARGoS v.." xuất hiện, quản lý bởi window manager của host,
GPU thật `RENOIR`, không phải software rendering).

## Ghi chú kỹ thuật

- **ARGOS_DOCUMENTATION=OFF**: có một bug trong CMake của ARGoS3 khiến
  `make install` báo lỗi thiếu `argos3.1.gz` (man page target không nằm
  trong `ALL`). Tắt build tài liệu để né lỗi này — không ảnh hưởng đến
  simulator. Nếu cần doc, chạy `cmake .. -DARGOS_DOCUMENTATION=ON && make doc`
  thủ công trong workspace.
- **UID/GID**: container chạy user `argos` với UID/GID khớp `${UID}/${GID}`
  trong `.env` (mặc định 1000/1000, khớp máy này). Nếu máy bạn dùng UID
  khác, sửa `.env` rồi `docker compose build` lại.
- **GPU**: `/dev/dri` được pass-through + `group_add: [44, 110]` (video,
  render trên máy này). Nếu máy khác có GID khác, kiểm tra bằng
  `getent group video render` và sửa `docker-compose.yml`.
- **network_mode: host**: dùng host networking để X11 + các giao thức
  TCP/UDP mà robot bầy đàn hay dùng (multi-process ARGoS, ROS bridge, …)
  hoạt động không cần map port thủ công.
- Có sẵn `.devcontainer/devcontainer.json` nếu bạn muốn dùng extension
  "Dev Containers" của VS Code để mở thẳng vào container (IntelliSense
  C++ chạy trong container, files vẫn nằm trên host).

## Dừng / dọn dẹp

```bash
docker compose down          # dừng container
xhost -local:docker          # (tuỳ chọn) thu hồi quyền X11
```
