FROM python:3.10-slim

# Install library OS yang dibutuhkan oleh Pygame untuk GUI (X11)
RUN apt-get update && apt-get install -y \
    python3-tk \
    libx11-6 \
    libgl1 \
    libxext6 \
    libxrender1 \
    libsm6 \
    libice6 \
    libxft2 \
    libxinerama1 \
    libxi6 \
    libxcursor1 \
    libxrandr2 \
    libxxf86vm1 \
    libxss1 \
    libdbus-1-3 \
    libegl1 \
    libsdl2-2.0-0 \
    libsdl2-ttf-2.0-0 \
    libsdl2-image-2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install library Python
RUN pip install --no-cache-dir pygame numpy paho-mqtt matplotlib

# Jalankan script mapping_space.py (mode simulasi default)
# Untuk mode real (ESP32), gunakan: --real
CMD ["python", "mapping_space.py"]
