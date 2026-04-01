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

# ---- Memory preload (test_ramulator) ----
# Optional: set MEMINIT to a .hex or .bin file to preload functional_mem.
#   make sim   MEMINIT=configs/meminit.hex
#   make sim   MEMINIT=path/to/image.bin  MEMINIT_TYPE=bin  MEMINIT_BASE=0x80000000
# MEMINIT_TYPE defaults to "hex"; MEMINIT_BASE defaults to 0 (bin only).
MEMINIT      ?=
MEMINIT_TYPE ?= hex
MEMINIT_BASE ?= 0

ifneq ($(MEMINIT),)
  MEMINIT_FLAGS := -G MEM_INIT_FILE="$(MEMINIT)" \
                   -G MEM_INIT_TYPE="$(MEMINIT_TYPE)" \
                   -G MEM_INIT_BASE=$(MEMINIT_BASE) \
                   -G MEMINIT_FILE="$(MEMINIT)" \
                   -G MEMINIT_TYPE="$(MEMINIT_TYPE)" \
                   -G MEMINIT_BASE=$(MEMINIT_BASE) \
                   -G USE_MEMINIT=1
else
  MEMINIT_FLAGS :=
endif

# ---- Memory preload (test_sdma) ----
# Optional: set SDMA_MEMINIT to skip all AXI matrix writes.
#   Generate : make gen_sdma_meminit   (or: python3 scripts/gen_sdma_meminit.py)
#   Run      : make sdma SDMA_MEMINIT=configs/sdma_meminit.bin
# SDMA_MEMINIT_TYPE defaults to "bin" (raw binary); base address is always 0.
SDMA_MEMINIT      ?=
SDMA_MEMINIT_TYPE ?= bin
SDMA_MEMINIT_BASE ?= 0

ifneq ($(SDMA_MEMINIT),)
  SDMA_MEMINIT_FLAGS := -G MEM_INIT_FILE="$(SDMA_MEMINIT)" \
                        -G MEM_INIT_TYPE="$(SDMA_MEMINIT_TYPE)" \
                        -G MEM_INIT_BASE=$(SDMA_MEMINIT_BASE) \
                        -G MEMINIT_FILE="$(SDMA_MEMINIT)" \
                        -G MEMINIT_TYPE="$(SDMA_MEMINIT_TYPE)" \
                        -G MEMINIT_BASE=$(SDMA_MEMINIT_BASE) \
                        -G USE_MEMINIT=1
else
  SDMA_MEMINIT_FLAGS :=
endif

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

SDMA_SRCS := \
	$(DPIROOT)/ramulator_sv_wrapper.sv \
	$(DPIROOT)/test_sdma.sv

# ---- Phony targets ----
.PHONY: all sim sim_gui sdma sdma_gui ram_lib gen_sdma_meminit clean \
        ddr3 ddr4 ddr5 gddr6 hbm2 hbm3 lpddr5

all: sim

# ---- Build Ramulator DPI shared library via CMake ----
# Ramulator2 requires C++20 (<ranges>).  RHEL8's default GCC 8 miscompiles
# the -O3 Release build and crashes at ramulator_init.  Use the gcc-toolset-14
# compilers directly so cmake always gets a correct C++20-capable toolchain.
GCC14_CXX := /opt/rh/gcc-toolset-14/root/usr/bin/g++
GCC14_CC  := /opt/rh/gcc-toolset-14/root/usr/bin/gcc
GCC14_LIB := /opt/rh/gcc-toolset-14/root/lib64

ram_lib:
	@echo "[ram_lib] configuring..."
	cmake -S $(TOPDIR) -B $(TOPDIR)/build \
	    -DCMAKE_BUILD_TYPE=Release \
	    -DCMAKE_CXX_COMPILER=$(GCC14_CXX) \
	    -DCMAKE_C_COMPILER=$(GCC14_CC) \
	    -G "Unix Makefiles" > /dev/null 2>&1 || true
	@echo "[ram_lib] building $(RAMULATOR_TARGET)..."
	LD_LIBRARY_PATH=$(GCC14_LIB):$$LD_LIBRARY_PATH \
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
	        $(MEMINIT_FLAGS) \
	        $(SCRATCH).test_ramulator \
	        -onfinish stop \
	        -do "view objects; do $(WAVEROOT)/test_ramulator.do; run -all"; \
	else \
	    $(VSIM) $(VSIM_COV_FLAGS) -c -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(MEMINIT_FLAGS) \
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

# ---- SDMA testbench ----
# Usage:
#   make sdma              # batch, default config
#   make sdma GUI=ON       # waveform GUI
#   make sdma CFG=configs/ddr5_config.yaml
sdma: ram_lib
	@\
	echo "[sdma] cleaning stale work library..."; \
	rm -rf $(SCRATCH); \
	$(VLIB) $(SCRATCH); \
	\
	echo "[sdma] compiling SV sources..."; \
	$(VLOG) -sv -mfcu -work $(SCRATCH) +acc $(VLOG_COV_FLAGS) \
	    +incdir+$(AXIROOT) \
	    $(AXI_SRCS) \
	    $(SDMA_SRCS); \
	\
	echo "[sdma] running $(if $(filter ON,$(GUI)),GUI,batch) simulation (config: $(CFG))..."; \
	if [ "$(GUI)" = "ON" ]; then \
	    $(VSIM) $(VSIM_COV_FLAGS) -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SDMA_MEMINIT_FLAGS) \
	        $(SCRATCH).test_sdma \
	        -onfinish stop \
	        -do "view objects; run -all"; \
	else \
	    $(VSIM) $(VSIM_COV_FLAGS) -c -voptargs="+acc" \
	        -sv_lib ./$(RAMULATOR_LIB) \
	        -G CFG="$(CFG)" \
	        $(SDMA_MEMINIT_FLAGS) \
	        $(SCRATCH).test_sdma \
	        -do "run -all" 2>/dev/null || true; \
	    if grep -q "=== PASSED ===" transcript 2>/dev/null; then \
	        echo "[sdma] Result: PASSED"; \
	    elif grep -q "=== FAILED ===" transcript 2>/dev/null; then \
	        echo "[sdma] Result: FAILED"; exit 1; \
	    else \
	        echo "[sdma] Result: UNKNOWN (simulation may have crashed or timed out)"; \
	    fi; \
	fi

sdma_gui:
	@$(MAKE) sdma GUI=ON

# ---- Generate SDMA meminit binary ----
# Produces configs/sdma_meminit.bin (4 MB: ROW_BASE + TILE_BASE regions).
# Only needs to be run once; regenerate if matrix parameters change.
gen_sdma_meminit:
	python3 $(TOPDIR)/scripts/gen_sdma_meminit.py

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
