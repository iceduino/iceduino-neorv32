NEORV32_HOME := ../../../neorv32

OLIMEX_HOME := ../../..

NEORV32_RTL_CORE := $(NEORV32_HOME)/rtl/core

NEORV32_PKG := $(NEORV32_RTL_CORE)/neorv32_package.vhd

NEORV32_APPIMG_SRC := \
  $(NEORV32_RTL_CORE)/neorv32_application_image \
  $(NEORV32_RTL_CORE)/neorv32_application_image

NEORV32_MEM_SRC := \
  $(NEORV32_RTL_CORE)/neorv32_imem.vhd \
  $(NEORV32_RTL_CORE)/neorv32_dmem.vhd \

NEORV32_CORE_SRC := \
  $(NEORV32_RTL_CORE)/neorv32_boot_rom.vhd \
  $(NEORV32_RTL_CORE)/neorv32_bus_keeper.vhd \
  $(NEORV32_RTL_CORE)/neorv32_busswitch.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cfs.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_alu.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_bus.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_control.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_cp_fpu.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_cp_muldiv.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_cp_shifter.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_decompressor.vhd \
  $(NEORV32_RTL_CORE)/neorv32_cpu_regfile.vhd \
  $(NEORV32_RTL_CORE)/neorv32_debug_dm.vhd \
  $(NEORV32_RTL_CORE)/neorv32_debug_dtm.vhd \
  $(NEORV32_RTL_CORE)/neorv32_fifo.vhd \
  $(NEORV32_RTL_CORE)/neorv32_gpio.vhd \
  $(NEORV32_RTL_CORE)/neorv32_icache.vhd \
  $(NEORV32_RTL_CORE)/neorv32_mtime.vhd \
  $(NEORV32_RTL_CORE)/neorv32_neoled.vhd \
  $(NEORV32_RTL_CORE)/neorv32_pwm.vhd \
  $(NEORV32_RTL_CORE)/neorv32_slink.vhd \
  $(NEORV32_RTL_CORE)/neorv32_spi.vhd \
  $(NEORV32_RTL_CORE)/neorv32_sysinfo.vhd \
  $(NEORV32_RTL_CORE)/neorv32_top.vhd \
  $(NEORV32_RTL_CORE)/neorv32_trng.vhd \
  $(NEORV32_RTL_CORE)/neorv32_twi.vhd \
  $(NEORV32_RTL_CORE)/neorv32_uart.vhd \
  $(NEORV32_RTL_CORE)/neorv32_wdt.vhd \
  $(NEORV32_RTL_CORE)/neorv32_wishbone.vhd \
  $(NEORV32_RTL_CORE)/neorv32_xirq.vhd

OLIMEX_APPIMG_SRC := \
  $(OLIMEX_HOME)/osflow/boards/olimex/neorv32_application_image.vhd \
  $(OLIMEX_HOME)/osflow/boards/olimex/neorv32_bootloader_image.vhd

NEORV32_SRC := ${OLIMEX_APPIMG_SRC} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}

ICE40_SRC := \
  $(OLIMEX_HOME)/osflow/device/ice40/sb_ice40_components.vhd

PCF_SRC := \
  $(OLIMEX_HOME)/osflow/constraint/olimex.pcf

OLIMEX_SRC := \
  $(OLIMEX_HOME)/osflow/boards/olimex/olimex_led.vhd \
  $(OLIMEX_HOME)/osflow/boards/olimex/olimex_button.vhd \
  $(OLIMEX_HOME)/osflow/boards/olimex/olimex_gpio.vhd \
  $(OLIMEX_HOME)/osflow/boards/olimex/neorv32_olimex_top.vhd 
  
DESIGN_NAME := olimex
IMPL_NAME := ${DESIGN_NAME}_impl

.PHONY: all json asc bin clean

# Some phony targets for convenience
all: bin
json: ${IMPL_NAME}.json
asc: ${IMPL_NAME}.asc
bin: ${IMPL_NAME}.bin

ice40-obj08.cf: ${ICE40_SRC}
	ghdl -a --std=08 --work=iCE40 ${ICE40_SRC}

neorv32-obj08.cf: ice40-obj08.cf ${NEORV32_PKG} ${NEORV32_SRC}
	ghdl -a --std=08 --work=neorv32 ${NEORV32_PKG} ${NEORV32_SRC}

olimex-obj08.cf: neorv32-obj08.cf ${OLIMEX_SRC}
	ghdl -a --std=08 --work=olimex ${OLIMEX_SRC}
	
${IMPL_NAME}.json: olimex-obj08.cf
	yosys ${GHDL_MODULE} -p 'ghdl --std=08 --work=olimex --no-formal neorv32_olimex_top; synth_ice40 -dsp -json $@' 2>&1 | tee yosys-report.txt

${IMPL_NAME}.asc: ${IMPL_NAME}.json ${PCF_SRC}
	nextpnr-ice40 --ignore-loops --freq 50 \
	  --hx8k --package ct256 --json $< --pcf ${PCF_SRC} \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL_NAME}.bin: ${IMPL_NAME}.asc
	icepack $< $@

program: ${IMPL_NAME}.bin
	iceprog $<

clean:
	rm -f *.cf *.asc *.json *-report.txt *.bin

timing: ${IMPL_NAME}.asc
	icetime -p $(PCF_SRC) -P tq144 \
	-d hx4k $<
