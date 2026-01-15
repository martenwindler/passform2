# --- KONFIGURATION ---
FRONTEND_DIR=passform2_agent_frontend/passform2_agent_frontend
BACKEND_DIR=passform2_agent_backend

.PHONY: help frontend backend start-all install

help:
	@echo "Verfügbare Befehle:"
	@echo "  make frontend   - Startet das Frontend"
	@echo "  make backend    - Startet das Backend"
	@echo "  make start-all  - Startet beides parallel"

# --- EINZELSTART ---

frontend:
	@echo ">>> Starte Frontend in $(FRONTEND_DIR)..."
	cd $(FRONTEND_DIR) && bash ../start__frontend.sh

backend:
	@echo ">>> Starte Backend in $(BACKEND_DIR)..."
	cd $(BACKEND_DIR) && bash ./start__backend.sh

# --- PARALLELER START ---
start-all:
	@echo ">>> Starte Gesamtsystem..."
	$(MAKE) -j2 frontend backend

# --- INSTALLATION ---
install:
	@echo ">>> Installiere Abhängigkeiten..."
	cd $(FRONTEND_DIR) && npm install