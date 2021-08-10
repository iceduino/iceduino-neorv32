RTL_CORE_SRC := ../src/rtl/core

NEORV32_PKG := $(RTL_CORE_SRC)/neorv32_package.vhd

NEORV32_APP_SRC := \
  $(RTL_CORE_SRC)/neorv32_application_image.vhd \

NEORV32_MEM_SRC := \
  ../src/setups/osflow/devices/ice40/neorv32_imem.ice40up_spram.vhd \
  ../src/setups/osflow/devices/ice40/neorv32_dmem.ice40up_spram.vhd

NEORV32_CORE_SRC := \
  $(RTL_CORE_SRC)/neorv32_bootloader_image.vhd \
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
  
#NEORV32_SRC := ${NEORV32_PKG} ${NEORV32_APP_SRC} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}
NEORV32_SRC := ${NEORV32_APP_SRC} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}
UPDUINO_SRC := \
  neorv32_UPduino_top.vhd

ICE40_SRC := \
  ../src/setups/osflow/devices/ice40/sb_ice40_components.vhd
PCF_SRC := \
  ../src/setups/osflow/constraints/UPduino_v3.pcf
DESIGN_NAME := UPduino_v3
IMPL_NAME := ${DESIGN_NAME}_impl_1

.PHONY: all syn impl bin clean

# Some phony targets for convenience
all: bin
syn: ${IMPL_NAME}.json
impl: ${IMPL_NAME}.asc
bin: ${IMPL_NAME}.bin

ice40up-obj08.cf: ${ICE40_SRC}
	ghdl -a --std=08 --work=iCE40 ${ICE40_SRC}

neorv32-obj08.cf: ice40up-obj08.cf ${NEORV32_PKG} ${NEORV32_SRC}
	ghdl -a --std=08 --work=neorv32 ${NEORV32_PKG} ${NEORV32_SRC}

work-obj08.cf: neorv32-obj08.cf ${UPDUINO_SRC}
	ghdl -a --std=08 --work=work ${UPDUINO_SRC}
	
${IMPL_NAME}.json: work-obj08.cf
	yosys -p 'ghdl --std=08 --no-formal neorv32_UPduino_top; synth_ice40 -dsp -json $@' 2>&1 | tee yosys-report.txt

${IMPL_NAME}.asc: ${IMPL_NAME}.json ${PCF_SRC}
	nextpnr-ice40 --ignore-loops --timing-allow-fail \
	  --up5k --package sg48 --json $< --pcf ${PCF_SRC} \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL_NAME}.bin: ${IMPL_NAME}.asc
	icepack $< $@

clean:
	rm -f *.cf *.asc *.json *-report.txt
