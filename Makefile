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
VLIB ?= vlib -64
VLOG ?= vlog -64
VSIM ?= vsim -64
GUI      ?= OFF
COVERAGE ?= OFF         # set to ON to enable coverage instrumentation

# ---- Coverage flags (must be consistent between vlog and vsim) ----
VLOG_COV_FLAGS  :=
VSIM_COV_FLAGS  :=
ifeq ($(COVERAGE),ON)
  VLOG_COV_FLAGS  := -cover bcesft
  VSIM_COV_FLAGS  := -coverage
endif

# ---- DRAM standard selection ----
# Usage: make sim DRAM=hbm2   (default: ddr4)
# Or override directly: make sim CFG=configs/my.yaml
DRAM ?= ddr4

DRAM_CFG_ddr3   := $(CFGROOT)/ddr3_config.yaml
DRAM_CFG_ddr4   := $(CFGROOT)/ddr4_config.yaml
DRAM_CFG_ddr5   := $(CFGROOT)/ddr5_config.yaml
DRAM_CFG_gddr6  := $(CFGROOT)/gddr6_config.yaml
DRAM_CFG_hbm2   := $(CFGROOT)/hbm2_config.yaml
DRAM_CFG_hbm3   := $(CFGROOT)/hbm3_config.yaml
DRAM_CFG_lpddr5 := $(CFGROOT)/lpddr5_config.yaml

# CFG can still be overridden directly; otherwise resolved from DRAM
CFG ?= $(DRAM_CFG_$(DRAM))

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
.PHONY: all sim sim_gui ram_lib clean \
        ddr3 ddr4 ddr5 gddr6 hbm2 hbm3 lpddr5

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
	$(VLOG) -64 -sv -mfcu -work $(SCRATCH) +acc $(VLOG_COV_FLAGS) \
	    +incdir+$(AXIROOT) \
	    $(AXI_SRCS) \
	    $(DPI_SRCS); \
	\
	echo "[sim] running $(if $(filter ON,$(GUI)),GUI,batch) simulation (config: $(CFG))..."; \
	if [ "$(GUI)" = "ON" ]; then \
	    $(VSIM) -64 $(VSIM_COV_FLAGS) -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SCRATCH).test_ramulator \
	        -onfinish stop \
	        -do "view objects; do $(WAVEROOT)/test_ramulator.do; run -all"; \
	else \
	    $(VSIM) -64 $(VSIM_COV_FLAGS) -c -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SCRATCH).test_ramulator \
	        -do "run -all" 2>/dev/null || true; \
	    if grep -q "=== PASSED ===" transcript 2>/dev/null; then \
	        echo "[sim] Result: PASSED"; \
	    elif grep -q "=== FAILED ===" transcript 2>/dev/null; then \
	        echo "[sim] Result: FAILED"; exit 1; \
	    else \
	        echo "[sim] Result: UNKNOWN (simulation may have crashed or timed out)"; \
	    fi; \
	fi

sim_gui:
	@$(MAKE) sim GUI=ON

# ---- Per-standard shorthand targets ----
# e.g. "make hbm2" runs batch sim with HBM2 config
ddr3:   ; @$(MAKE) sim DRAM=ddr3
ddr4:   ; @$(MAKE) sim DRAM=ddr4
ddr5:   ; @$(MAKE) sim DRAM=ddr5
gddr6:  ; @$(MAKE) sim DRAM=gddr6
hbm2:   ; @$(MAKE) sim DRAM=hbm2
hbm3:   ; @$(MAKE) sim DRAM=hbm3
lpddr5: ; @$(MAKE) sim DRAM=lpddr5

# ---- Clean ----
clean:
	rm -rf $(SCRATCH) transcript vsim.wlf vsim_stacktrace.vstf modelsim.ini
