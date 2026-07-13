FROM python:3.10-slim

# Install library OS yang dibutuhkan oleh matplotlib untuk GUI (X11)
RUN apt-get update && apt-get install -y \
    python3-tk \
    libx11-6 \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install library Python
RUN pip install --no-cache-dir matplotlib numpy paho-mqtt

# Jalankan script utama
CMD ["python", "radar_mapper.py"]