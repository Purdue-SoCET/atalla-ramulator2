.SILENT: sim
.ONESHELL: sim

SHELL := /bin/bash

TOPDIR    := .
AXIROOT   := $(TOPDIR)/src/axi
DPIROOT   := $(TOPDIR)/src/dpi_wrapper
WAVEROOT  := $(TOPDIR)/waves
CFGROOT   := $(TOPDIR)/configs
SCRATCH   := work

# ---- Tool overrides ----
VLIB ?= vlib
VLOG ?= vlog
VSIM ?= vsim
GUI      ?= OFF
COVERAGE ?= OFF         # set to ON to enable coverage instrumentation

# ---- Coverage flags (must be consistent between vlog and vsim) ----
VLOG_COV_FLAGS  :=
VSIM_COV_FLAGS  :=
ifeq ($(COVERAGE),ON)
  VLOG_COV_FLAGS  := -cover bcesft
  VSIM_COV_FLAGS  := -coverage
endif

# ---- Build configuration ----
CFG ?= $(CFGROOT)/ddr4_config.yaml

# ---- Ramulator shared library ----
RAMULATOR_LIB    := libramulator_dpi
RAMULATOR_TARGET := ramulator_dpi_shared

# ---- Source files ----
AXI_SRCS := \
	$(AXIROOT)/axi_bus_pkg.sv \
	$(AXIROOT)/axi_bus_if.sv

DPI_SRCS := \
	$(DPIROOT)/ramulator_sv_wrapper.sv \
	$(DPIROOT)/test_ramulator.sv

# ---- Phony targets ----
.PHONY: all sim sim_gui ram_lib clean

all: sim

# ---- Build Ramulator DPI shared library via CMake ----
ram_lib:
	@echo "[ram_lib] configuring..."
	cmake -S $(TOPDIR) -B $(TOPDIR)/build -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" > /dev/null 2>&1 || true
	@echo "[ram_lib] building $(RAMULATOR_TARGET)..."
	cmake --build $(TOPDIR)/build --target $(RAMULATOR_TARGET) -j$$(nproc) 2>&1 | tail -5

# ---- Simulation (batch or GUI) ----
# Usage:
#   make sim              # batch, default config
#   make sim GUI=ON       # waveform GUI, default config
#   make sim CFG=configs/my.yaml
#   make sim_gui          # shorthand for GUI=ON

sim: ram_lib
	@\
	echo "[sim] cleaning stale work library..."; \
	rm -rf $(SCRATCH); \
	$(VLIB) $(SCRATCH); \
	\
	echo "[sim] compiling SV sources..."; \
	$(VLOG) -sv -mfcu -work $(SCRATCH) +acc $(VLOG_COV_FLAGS) \
	    +incdir+$(AXIROOT) \
	    $(AXI_SRCS) \
	    $(DPI_SRCS); \
	\
	echo "[sim] running $(if $(filter ON,$(GUI)),GUI,batch) simulation (config: $(CFG))..."; \
	if [ "$(GUI)" = "ON" ]; then \
	    $(VSIM) $(VSIM_COV_FLAGS) -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SCRATCH).test_ramulator \
	        -onfinish stop \
	        -do "view objects; do $(WAVEROOT)/test_ramulator.do; run -all"; \
	else \
	    $(VSIM) $(VSIM_COV_FLAGS) -c -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SCRATCH).test_ramulator \
	        -do "run -all" || true; \
	fi

sim_gui:
	@$(MAKE) sim GUI=ON

# ---- Clean ----
clean:
	rm -rf $(SCRATCH) transcript vsim.wlf vsim_stacktrace.vstf modelsim.ini
