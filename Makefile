# --- KONFIGURATION ---
# Pfade relativ zum Speicherort des Makefile (Wurzelverzeichnis passform2)

# Prüfen, ob secret.key existiert, sonst Fehler
ifeq ($(wildcard secret.key),)
$(error [!] secret.key nicht gefunden. Bitte erstelle die Datei mit deinen Credentials!)
endif

# Variablen aus der secret.key extrahieren
SSID    = $(shell sed -n '1p' secret.key)
WLAN_PW = $(shell sed -n '2p' secret.key)
PI_IP   = $(shell sed -n '3p' secret.key)
PI_USER = $(shell sed -n '4p' secret.key)

FRONTEND_DIR=passform2_agent_frontend/passform2_agent_frontend
BACKEND_DIR_REST=passform2_agent_backend
BACKEND_DIR_ROS=passform2_ws
VENV=.venv
PYTHON=$(VENV)/bin/python3

.PHONY: help frontend backend backend-rest backend-ros start-all install clean

help:
	@echo "Verfügbare Befehle:"
	@echo "  make install      - Erstellt .venv und installiert alle Abhängigkeiten"
	@echo "  make frontend     - Startet das Elm/Vite Frontend"
	@echo "  make backend-ros  - Startet ROS-Bridge, Monitor & Test-Agent"
	@echo "  make backend-rest - Startet das Python REST-API Backend"
	@echo "  make start-all    - Startet ALLES parallel (Frontend + beide Backends)"

# --- EINZELSTART ---

frontend:
	@echo ">>> Starte Frontend..."
	cd $(FRONTEND_DIR) && ./../start__frontend.sh

backend-rest:
	@echo ">>> Starte REST-Backend..."
	# Wir führen einfach dein Skript aus, wie gewohnt
	cd $(BACKEND_DIR_REST) && bash ./start__backend.sh

backend-ros:
	@echo ">>> Starte ROS-Backend (Clean Build & Jazzy)..."
	# Aufräumen für sauberen Build
	rm -rf $(BACKEND_DIR_ROS)/build $(BACKEND_DIR_ROS)/install $(BACKEND_DIR_ROS)/log
	# Das Skript übernimmt die Aktivierung der VENV und ROS Jazzy
	cd $(BACKEND_DIR_ROS) && ./start__passform.sh

# --- PARALLELER START ---
# Startet Frontend, REST-API und ROS-Zentrale gleichzeitig
start-all:
	@echo ">>> Starte Gesamtsystem..."
	$(MAKE) -j3 frontend backend-rest backend-ros

# --- INSTALLATION & SETUP ---
install:
	@echo ">>> Erstelle Virtual Environment mit ROS-Anbindung..."
	# --system-site-packages ist essentiell für Jazzy/rclpy
	python3 -m venv $(VENV) --system-site-packages
	@echo ">>> Installiere Python-Abhängigkeiten..."
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install python-socketio eventlet flask-cors flask
	@echo ">>> Installiere Frontend-Abhängigkeiten..."
	cd $(FRONTEND_DIR) && npm install
	@echo "✅ Installation abgeschlossen. Nutze 'make start-all' zum Starten."

clean:
	@echo ">>> Bereinige temporäre Dateien..."
	find . -type d -name "__pycache__" -exec rm -rf {} +
	rm -rf $(BACKEND_DIR_ROS)/build $(BACKEND_DIR_ROS)/install $(BACKEND_DIR_ROS)/log

deploy:
	rsync -avz --delete --exclude '.git' --exclude 'venv/' ./ $(PI_USER)@$(PI_IP):~/passform2/
	ssh $(PI_USER)@$(PI_IP) "cd ~/passform2 && docker compose up --build -d"

logs:
	ssh $(PI_USER)@$(PI_IP) "cd ~/passform2 && docker compose logs -f"