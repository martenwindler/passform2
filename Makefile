SHELL := /bin/bash
PKG_NAME := passform2

.PHONY: all build clean run help

all: build run

build:
	colcon build --packages-select $(PKG_NAME) --symlink-install

run:
	source install/setup.bash && ros2 launch $(PKG_NAME) $(PKG_NAME)_launch.py

clean:
	bash utils/clean__all.sh --quiet
	rm -rf build/ install/ log/

help:
	@echo "Verfügbare Befehle:"
	@echo "  make build  - Baut das Paket $(PKG_NAME)"
	@echo "  make run    - Startet das Launch-File von $(PKG_NAME)"
	@echo "  make all    - Baut und startet das Paket"
	@echo "  make clean  - Löscht build, install und log Ordner"