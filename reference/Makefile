# FARMSim documentation. The FARMS packages must be installed in the active
# environment (the API, configuration and CLI pages are generated from them).
export DISABLE_MKDOCS_2_WARNING = true
export MUJOCO_GL ?= egl

.PHONY: install check build serve

install:  ## Clone and install the FARMS packages of farms-packages.yaml
	python tools/install_farms.py

check:  ## Check the hand-written pages against the installed code
	python tools/check_docs.py

build: check  ## Build the site (strict)
	mkdocs build --strict

serve:  ## Live preview on http://127.0.0.1:8000
	mkdocs serve
