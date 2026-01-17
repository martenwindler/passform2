cat << 'EOF' > setup_docker.sh
#!/bin/bash
# Docker Installations-Skript für Pi 5 (Ubuntu 24.04)

sudo apt update
sudo apt install -y ca-certificates curl gnupg

# Docker GPG Key hinzufügen
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Repository einrichten
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# User zur Docker-Gruppe hinzufügen (damit kein 'sudo' nötig ist)
sudo usermod -aG docker $USER
echo "[+] Docker installiert! Bitte logge dich einmal aus und wieder ein."
EOF

chmod +x setup_docker.sh
./setup_docker.sh