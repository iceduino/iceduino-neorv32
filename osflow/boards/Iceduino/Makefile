RTL_CORE_SRC := ../../../neorv32/rtl/core

NEORV32_PKG := $(RTL_CORE_SRC)/neorv32_package.vhd

NEORV32_APPIMG_SRC := \
  ../../device/Iceduino/neorv32_application_image.vhd \
  ../../device/Iceduino/neorv32_bootloader_image.vhd

NEORV32_MEM_SRC := \
  $(RTL_CORE_SRC)/neorv32_imem.vhd \
  $(RTL_CORE_SRC)/neorv32_dmem.vhd \

NEORV32_CORE_SRC := \
  $(RTL_CORE_SRC)/neorv32_boot_rom.vhd \
  $(RTL_CORE_SRC)/neorv32_bus_keeper.vhd \
  $(RTL_CORE_SRC)/neorv32_busswitch.vhd \
  $(RTL_CORE_SRC)/neorv32_cfs.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_alu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_bus.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_control.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_fpu.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_muldiv.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_cp_shifter.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_decompressor.vhd \
  $(RTL_CORE_SRC)/neorv32_cpu_regfile.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dm.vhd \
  $(RTL_CORE_SRC)/neorv32_debug_dtm.vhd \
  $(RTL_CORE_SRC)/neorv32_fifo.vhd \
  $(RTL_CORE_SRC)/neorv32_gpio.vhd \
  $(RTL_CORE_SRC)/neorv32_icache.vhd \
  $(RTL_CORE_SRC)/neorv32_mtime.vhd \
  $(RTL_CORE_SRC)/neorv32_neoled.vhd \
  $(RTL_CORE_SRC)/neorv32_pwm.vhd \
  $(RTL_CORE_SRC)/neorv32_slink.vhd \
  $(RTL_CORE_SRC)/neorv32_spi.vhd \
  $(RTL_CORE_SRC)/neorv32_sysinfo.vhd \
  $(RTL_CORE_SRC)/neorv32_top.vhd \
  $(RTL_CORE_SRC)/neorv32_trng.vhd \
  $(RTL_CORE_SRC)/neorv32_twi.vhd \
  $(RTL_CORE_SRC)/neorv32_uart.vhd \
  $(RTL_CORE_SRC)/neorv32_wdt.vhd \
  $(RTL_CORE_SRC)/neorv32_wishbone.vhd \
  $(RTL_CORE_SRC)/neorv32_xirq.vhd

NEORV32_SRC := ${NEORV32_APPIMG_SRC} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}
TOP_SRC := \
  neorv32_iceduino_top.vhd

ICE40_SRC := \
  ../../device/ice40/sb_ice40_components.vhd
PCF_SRC := \
  ../../constraint/iceduino.pcf
ICEDUINO_SRC := \
  ../../device/Iceduino/iceduino_dummy_slave1.vhd 
  
DESIGN_NAME := iceduino
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

top-obj08.cf: neorv32-obj08.cf ${TOP_SRC}
	ghdl -a --std=08 --work=top ${TOP_SRC}
	
${IMPL_NAME}.json: top-obj08.cf
	yosys -p 'ghdl --std=08 --work=top --no-formal neorv32_iceduino_top; synth_ice40 -dsp -json $@' 2>&1 | tee yosys-report.txt

${IMPL_NAME}.asc: ${IMPL_NAME}.json ${PCF_SRC}
	nextpnr-ice40 --ignore-loops --freq 50 --timing-allow-fail \
	  --hx8k --package tq144:4k --json $< --pcf ${PCF_SRC} \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL_NAME}.bin: ${IMPL_NAME}.asc
	icepack $< $@

clean:
	rm -f *.cf *.asc *.json *-report.txt *.bin
