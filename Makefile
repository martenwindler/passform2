.PHONY: build-doc build-develop build-main build-test build-training clean create-files data-source-all init-all init-local init-docker ruff

clean:
	bash scripts/clean__all.sh --quiet

build-doc:
	bash ./scripts/create__doc.sh

build-main:
	@bash -c "printf '\n%.0s' {1..100}"
	$(MAKE) clean
	@echo "==> Ruff **/**/*.py check..."
	$(MAKE) ruff
	@echo "==> Update doc to prev. state..."
	$(MAKE) build-doc
	@echo "==> Activate python virtual environment..."
	@bash -c "source .venv/bin/activate && \
		echo '==> Restarting services...' && \
		sh scripts/service__restart_all.sh && \
		echo '==> Start building BA__Programmierung.main:main...' && \
		pip install . --quiet \
	"
	ba-programmierung
	sleep 1
	$(MAKE) build-test

build-develop:
	@bash -c "printf '\n%.0s' {1..100}"
	$(MAKE) clean
	@echo "==> Ruff **/**/*.py check..."
	$(MAKE) ruff
	@echo "==> Activate python virtual environment..."
	@bash -c "source .venv/bin/activate && \
		echo '==> Restarting services...' && \
		sh scripts/service__restart_all.sh && \
		echo '==> Start building BA__Programmierung.main:main...' && \
		pip install . --quiet \
	"
	ba-programmierung
	sleep 1
	$(MAKE) build-test

build-test:
	@echo "==> Create and execute test-environment..."
	@bash -c "source .venv/bin/activate && \
		echo '==> Find tests and execute...' && \
		sh scripts/test__all.sh"

build-training:
	@echo "==> Activate python virtual environment..."
	@bash -c "source .venv/bin/activate && \
		echo '==> Restarting services...' && \
		sh scripts/service__restart_all.sh && \
		echo '==> Starting EDNN training...' && \
		cd BA__Programmierung/ml/ && python3 ednn_regression__iris.py && \
		echo '==> Starting EDNN loss-visualization...' && \
		cd ../viz/ && python3 viz__ednn_regression__iris.py"

build-jupyter: 
	@echo "==> Activate jupyter environment..."
	@bash -c "jupyter lab /assets/repos/BA__U-i-mlb-Sm-f-d-s-V-a-S__tex/prototypes/jupyternotebook/a7.ipynb"

check-all:
	scripts/check__all.sh

create-files:
	@echo "==> Create files by token..."
	@bash -c "sh scripts/create__files_by_token.sh"

data-source-all:
	@echo "==> Processing datasets in ./assets/data/sources/ ..."
	@bash -c "source .venv/bin/activate && \
		bash scripts/data_source__all.sh"

init-all:
	$(MAKE) init-local
	$(MAKE) init-docker

init-docker:
	#@echo "==> Test Docker daemon"
	#docker --version

	sleep 3

	@echo "==> Starte Docker Desktop unter Windows"
	@cmd.exe /c start "" "C:\\Program Files\\Docker\\Docker\\Docker Desktop.exe"

	sleep 3

	@echo "==> Build Docker image"
	docker build -t ba__projekt .

	@echo "==> Run Docker container"
	docker run --rm ba__projekt:latest

init-local:
	@echo "==> Set project path env variable"
	sh scripts/setup__project_base_path.sh

	@echo "==> Reload .bashrc"
	bash -c "source ~/.bashrc"

	# @echo "==> Re-Enter .venv"
	# bash -c "source ~/.venv/bin/activate"

	@echo "==> Ensure required system packages are installed"
	sudo apt-get update && sudo apt-get install -y python3 tar unzip make

	@echo "==> Create virtual environment"
	python3 -m venv .venv

	@echo "==> Activate and install required Python packages"
	. .venv/bin/activate && \
	pip install --upgrade pip && \
	pip install duckdb matplotlib notebook numpy pandas torch scikit-learn scipy tensorflow torchvision

	@echo "==> Install local Python package"
	. .venv/bin/activate && pip install . --quiet

	@echo "==> Run project main entry point"
	. .venv/bin/activate && ba-programmierung

	bash scripts/clean__egg_info.sh

ruff:
	ruff check . --fix

schreiben: 
	bash -c "cd BA__Projekt/assets/repos/BA__U-i-mlb-Sm-f-d-s-V-a-S__tex/"
	sleep 1s 
	make clean