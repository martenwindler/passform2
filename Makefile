# ──────────────────────────────────────────────────────────────
# PASSFORM 2.0 - MASTER MAKEFILE
# ──────────────────────────────────────────────────────────────

# --- KONFIGURATION ---
SHELL := /bin/bash

# Prüfen, ob secret.key existiert
ifeq ($(wildcard secret.key),)
$(error [!] secret.key nicht gefunden. Bitte erstelle die Datei mit deinen Credentials!)
endif

# Variablen aus secret.key extrahieren
PI_IP   = $(shell sed -n '3p' secret.key)
PI_USER = $(shell sed -n '4p' secret.key)

# Pfade
FRONTEND_DIR = passform2_agent_frontend
BACKEND_DIR  = passform2_agent_backend
WS_DIR       = passform2_ws
VENV         = .venv
PYTHON       = $(VENV)/bin/python3

# Liste der zu überschreibenden Pakete für Colcon (Jazzy Overrides)
ALLOW_OVERRIDE = action_msgs builtin_interfaces diagnostic_msgs geometry_msgs \
                 rcl_interfaces rosgraph_msgs rosidl_core_runtime \
                 rosidl_default_generators rosidl_default_runtime service_msgs \
                 std_msgs test_interface_files test_msgs unique_identifier_msgs

.PHONY: help install backend backend-legacy frontend start-all clean build-ws deploy logs dev

help:
	@echo "PassForm 2.0 - Management Console"
	@echo "--------------------------------"
	@echo "  make install         - Erstellt .venv und installiert Abhängigkeiten"
	@echo "  make build-ws        - Fixiert & baut den ROS 2 Workspace (Jazzy)"
	@echo "  make backend         - Startet das NEUE Rust-Backend (Port 8080)"
	@echo "  make backend-legacy  - Startet das ALTE Python-Backend (Port 8080)"
	@echo "  make frontend        - Startet das Vite/Elm Frontend"
	@echo "  make start-all       - Startet Frontend + Rust-Backend parallel"
	@echo "  make clean           - Löscht alle Build-Artefakte und Caches"

# --- SETUP ---

install:
	@echo ">>> Erstelle Virtual Environment..."
	python3 -m venv $(VENV) --system-site-packages
	@echo ">>> Installiere Python-Abhängigkeiten..."
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install python-socketio eventlet flask-cors flask uvicorn fastapi
	@echo ">>> Installiere Frontend-Abhängigkeiten..."
	cd $(FRONTEND_DIR) && npm install
	@echo "✅ Installation abgeschlossen."

# --- DEVELOPMENT ---
dev:
	@echo "Checking Frontend dependencies..."
	@if [ ! -d "passform2_agent_frontend/node_modules" ]; then \
		cd passform2_agent_frontend && npm install; \
	fi
	@echo "Starting Tauri Environment..."
	cd passform2_agent_backend && cargo tauri dev

# Manueller Build des ROS Workspaces (Löst den Symlink-Fehler)
build-ws:
	@echo ">>> Bereinige alte Artefakte um Symlink-Fehler zu vermeiden..."
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log
	@echo ">>> Baue ROS 2 Workspace mit Overrides..."
	cd $(WS_DIR) && source /opt/ros/jazzy/setup.bash && colcon build \
		--symlink-install \
		--allow-overriding $(ALLOW_OVERRIDE) \
		--packages-up-to rclrs passform_agent_resources
	@echo "✅ Workspace erfolgreich gebaut."

# Startet das neue Rust-Backend
backend:
	@echo ">>> Starte Modern Rust Backend..."
	cd $(BACKEND_DIR) && ./start__backend.sh

# Startet das alte Python-Backend
backend-legacy:
	@echo ">>> Starte Legacy Python Backend..."
	cd $(BACKEND_DIR) && ./start__backend.sh --legacy

# Startet das Frontend
frontend:
	@echo ">>> Starte Vite/Elm Frontend..."
	cd $(FRONTEND_DIR) && ./start__frontend.sh

# --- ORCHESTRIERUNG ---

# Startet das moderne Gesamtsystem parallel
start-all:
	@echo ">>> Starte PassForm 2.0 (Modern Mode)..."
	$(MAKE) -j2 backend frontend

# --- CLEANUP & DEPLOY ---

clean:
	@echo ">>> Bereinige Caches und Build-Ordner..."
	find . -type d -name "__pycache__" -exec rm -rf {} +
	rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log
	rm -rf $(BACKEND_DIR)/target
	@echo "✨ System bereinigt."

deploy:
	@echo ">>> Deploye auf Zielsystem $(PI_IP)..."
	rsync -avz --delete \
		--exclude '.git' \
		--exclude '.venv' \
		--exclude 'target' \
		--exclude 'node_modules' \
		./ $(PI_USER)@$(PI_IP):~/passform2/
	ssh $(PI_USER)@$(PI_IP) "cd ~/passform2 && docker compose up --build -d"

logs:
	ssh $(PI_USER)@$(PI_IP) "cd ~/passform2 && docker compose logs -f"