FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# 系統/GUI/編譯依賴（cvxpy 需要）
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget sudo vim \
    python3 python3-pip python3-tk python3-opengl \
    build-essential pkg-config cmake \
    libgl1-mesa-glx libgl1-mesa-dri libegl1-mesa libglu1-mesa mesa-utils \
    libblas-dev liblapack-dev gfortran \
    fontconfig fonts-dejavu-core xvfb \
 && rm -rf /var/lib/apt/lists/*

# Python 套件
# 先升級 pip，再安裝 PyTorch (cu129) 與其它依賴
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
      torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu129 && \
    pip3 install --no-cache-dir numpy scipy matplotlib pybullet cvxpy

WORKDIR /workspace
COPY . /workspace

CMD ["/bin/bash"]
