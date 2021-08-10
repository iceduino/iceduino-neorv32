ifndef NEORV32_ROOT
    $(error NEORV32_ROOT is undefined)
endif

NEORV32_LOCAL_COPY ?= $(NEORV32_ROOT)/sim/work

TARGET_SIM   ?= ghdl
TARGET_FLAGS ?= $(RISCV_TARGET_FLAGS)

ifeq ($(shell command -v $(TARGET_SIM) 2> /dev/null),)
    $(error Target simulator executable '$(TARGET_SIM)` not found)
endif

NEORV32_MARCH ?= rv32i

RISCV_PREFIX   ?= riscv32-unknown-elf-
RISCV_GCC      ?= $(RISCV_PREFIX)gcc
RISCV_OBJDUMP  ?= $(RISCV_PREFIX)objdump
RISCV_OBJCOPY  ?= $(RISCV_PREFIX)objcopy
RISCV_READELF  ?= $(RISCV_PREFIX)readelf
RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -march=$(NEORV32_MARCH) -mabi=ilp32

NEORV32_LINK ?= link.imem_rom.ld

COMPILE_TARGET ?= \
	$$(RISCV_GCC) $(1) $$(RISCV_GCC_OPTS) \
	$$(RISCV_TARGET_FLAGS) \
	-I$(ROOTDIR)/riscv-test-suite/env/ \
	-I$(TARGETDIR)/$(RISCV_TARGET)/ \
	-T$(TARGETDIR)/$(RISCV_TARGET)/$(NEORV32_LINK) \
	$$(<) -o $$@

NEORV32_CPU_EXTENSION_RISCV_C ?= false
NEORV32_CPU_EXTENSION_RISCV_M ?= false

NEORV32_SOFTWARE_EXAMPLE ?= $(NEORV32_LOCAL_COPY)/sw/example/blink_led

RUN_TARGET ?= \
	cd $(work_dir_isa); \
	echo ">"; \
	rm -f $(NEORV32_LOCAL_COPY)/*.out; \
	echo "copying/using SIM-only IMEM (ROM!)"; \
	rm -f $(NEORV32_LOCAL_COPY)/rtl/core/neorv32_imem.vhd; \
	cp -f $(NEORV32_LOCAL_COPY)/sim/neorv32_imem.simple.vhd $(NEORV32_LOCAL_COPY)/rtl/core/neorv32_imem.vhd; \
	make -C $(NEORV32_SOFTWARE_EXAMPLE) main.elf; \
	cp -f $< $(NEORV32_SOFTWARE_EXAMPLE)/main.elf; \
	make -C $(NEORV32_SOFTWARE_EXAMPLE) main.bin install; \
	touch $(NEORV32_LOCAL_COPY)/neorv32.uart0.sim_mode.data.out; \
	GHDL_DEVNULL=true $(shell which time) -v $(NEORV32_LOCAL_COPY)/sim/ghdl.run.sh \
	  --stop-time=$(SIM_TIME) \
	  -gCPU_EXTENSION_RISCV_A=false \
	  -gCPU_EXTENSION_RISCV_C=$(NEORV32_CPU_EXTENSION_RISCV_C) \
	  -gCPU_EXTENSION_RISCV_E=false \
	  -gCPU_EXTENSION_RISCV_M=$(NEORV32_CPU_EXTENSION_RISCV_M) \
	  -gCPU_EXTENSION_RISCV_U=false \
	  -gCPU_EXTENSION_RISCV_Zicsr=true \
	  -gCPU_EXTENSION_RISCV_Zifencei=false \
	  -gEXT_IMEM_C=false \
	  -gMEM_INT_IMEM_SIZE='2097152'; \
	cp $(NEORV32_LOCAL_COPY)/sim/neorv32.uart0.sim_mode.data.out $(*).signature.output; \
	echo "<";
