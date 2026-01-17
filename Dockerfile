FROM ros:jazzy-ros-base

WORKDIR /app

# System-Abhängigkeiten
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Wir installieren die Libraries direkt. 
# So weiß Docker: "Das sind externe Abhängigkeiten, kein lokales Paket."
RUN pip3 install --no-cache-dir \
    fastapi \
    uvicorn \
    psutil \
    loguru \
    netifaces \
    zeroconf \
    --break-system-packages

# Jetzt erst den Code kopieren.
# Änderungen am Code triggern so NICHT die Neuinstallation der Libraries (schneller!)
COPY . .

EXPOSE 8000

# Startbefehl mit geladener ROS-Umgebung
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && uvicorn backend.main:app --host 0.0.0.0 --port 8000"]