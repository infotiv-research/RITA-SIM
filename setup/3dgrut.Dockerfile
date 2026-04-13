FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    gcc-11 \
    g++-11 \
    cmake \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Set GCC 11 as default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100

# Install Miniconda
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -p /opt/conda \
    && rm /tmp/miniconda.sh

ENV PATH="/opt/conda/bin:$PATH"

# Accept conda Terms of Service for default channels
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main \
    && conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# Clone 3DGRUT
RUN git clone --recursive https://github.com/nv-tlabs/3dgrut.git /opt/3dgrut

# Install 3DGRUT conda environment
WORKDIR /opt/3dgrut
RUN chmod +x install_env.sh && ./install_env.sh 3dgrut

# Activate conda env by default
RUN echo "conda activate 3dgrut" >> ~/.bashrc
SHELL ["conda", "run", "-n", "3dgrut", "/bin/bash", "-c"]

WORKDIR /workspace
