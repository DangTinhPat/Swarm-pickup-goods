########################################################################
# ARGoS3 dev image — Ubuntu 22.04
#
# - Builds & installs ARGoS3 (https://github.com/ilpincy/argos3) system-wide
#   so the image works standalone right after `docker build`.
# - Runs as a non-root user whose UID/GID match the host user, so files
#   bind-mounted from the host (edited in VS Code) keep correct ownership.
# - Includes X11 client libs so the ARGoS3 Qt/OpenGL visualization can
#   render on the host's display (see docker-compose.yml).
########################################################################

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Etc/UTC
ARG USER_NAME=argos
ARG USER_UID=1000
ARG USER_GID=1000

ENV TZ=${TZ} \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ---- OS + ARGoS3 build dependencies ----------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        ca-certificates \
        pkg-config \
        sudo \
        # ARGoS3 required deps
        libfreeimage-dev \
        libfreeimageplus-dev \
        qtbase5-dev \
        qtbase5-dev-tools \
        qt5-qmake \
        freeglut3-dev \
        libxi-dev \
        libxmu-dev \
        liblua5.3-dev \
        lua5.3 \
        # docs (optional but harmless, small)
        doxygen \
        graphviz \
        libgraphviz-dev \
        asciidoctor \
        # optional but recommended plugins
        libeigen3-dev \
        # X11 / OpenGL runtime so the Qt visualizer can show on the host
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        libglu1-mesa \
        mesa-utils \
        x11-apps \
        libxext6 \
        libxrender1 \
        libsm6 \
        # convenience tools for dev-in-container
        vim \
        nano \
        gdb \
        less \
    && rm -rf /var/lib/apt/lists/*

# ---- Non-root user matching the host UID/GID --------------------------
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

# ---- Build & install ARGoS3 from the vendored source -------------------
# The ARGoS3 source is tracked in this repo (workspace/argos3, see its
# UPSTREAM.txt for provenance), so cloning the repo + docker compose build
# is fully self-contained — no network fetch from upstream needed.
# Kept in /opt/argos3-src so entrypoint.sh can re-seed an empty workspace.
COPY workspace/argos3 /opt/argos3-src
RUN mkdir -p /opt/argos3-src/build \
    && cd /opt/argos3-src/build \
    && cmake ../src \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DARGOS_DOCUMENTATION=OFF \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && chown -R ${USER_NAME}:${USER_NAME} /opt/argos3-src

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/workspace
ENV DISPLAY=:0 \
    QT_X11_NO_MITSHM=1

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
