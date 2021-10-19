NEORV32_HOME := ../neorv32
NEORV32_RTL_CORE := $(NEORV32_HOME)/rtl/core
NEORV32_PKG := $(NEORV32_RTL_CORE)/neorv32_package.vhd
NEORV32_MEM_SRC := \
  $(NEORV32_RTL_CORE)/neorv32_imem.vhd \
  $(NEORV32_RTL_CORE)/neorv32_dmem.vhd  
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
ICEDUINO_HOME := ../
ICEDUINO_APPIMG_SRC := \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/neorv32_application_image.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/neorv32_bootloader_image.vhd
NEORV32_SRC := ${ICEDUINO_APPIMG_SRC} ${NEORV32_MEM_SRC} ${NEORV32_CORE_SRC}
ICE40_SRC := \
  $(ICEDUINO_HOME)/osflow/device/ice40/sb_ice40_components.vhd
PCF_SRC := \
  $(ICEDUINO_HOME)/osflow/constraint/iceduino.pcf
  
ICEDUINO_SRC := \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_led.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_switch.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_button.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_arduino_gpio.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_pmod.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_arduino_uart.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_arduino_spi.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_arduino_i2c.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/iceduino_arduino_adc.vhd \
  $(ICEDUINO_HOME)/osflow/boards/iceduino/neorv32_iceduino_top.vhd 
DESIGN_NAME := iceduino_tb


.PHONY: all wave clean

all:	
	rm -f *.cf
	ghdl -a --std=08 --work=iCE40 ${ICE40_SRC}
	ghdl -a --std=08 --work=neorv32 ${NEORV32_PKG} ${NEORV32_SRC}
	ghdl -a --std=08 --work=iceduino ${ICEDUINO_SRC}
	ghdl -i --std=08 --work=work -P../osflow/boards/iceduino neorv32_iceduino_top_tb.vhd
	ghdl -a --std=08 --work=work -P../osflow/boards/iceduino neorv32_iceduino_top_tb.vhd
	ghdl -m --std=08 --work=work -P../osflow/boards/iceduino neorv32_iceduino_top_tb

wave:
	ghdl -r --std=08 --work=work -P../osflow/boards/iceduino neorv32_iceduino_top_tb --wave=neorv32_iceduino_top_tb.ghw --stop-time="1000us" --ieee-asserts=disable
	gtkwave --rcvar 'do_initial_zoom_fit yes' --rcvar 'hier_max_level 1' --rcvar 'splash_disable on' neorv32_iceduino_top_tb.ghw
	
clean:
	rm -rf *.cf *.ghw *.sim_mode.* work/
