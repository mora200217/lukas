FROM ubuntu:22.04

# Dependencias
RUN apt-get update && apt-get install -y \
    python3 python3-pip git curl wget unzip \
    udev minicom \
    && rm -rf /var/lib/apt/lists/*

# Instalar PlatformIO
RUN pip3 install -U platformio

# Crear workspace
WORKDIR /workspace
