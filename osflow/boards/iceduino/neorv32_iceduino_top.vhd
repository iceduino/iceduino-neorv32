-- #################################################################################################
-- # << NEORV32 - Setup for the Iceduino Board >>
-- # Schematics available at https://github.com/iceduino/iceduino                                  #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Patryk Janik, Christopher Parnow. All rights reserved.                    #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library iCE40;
use iCE40.components.all; -- for device primitives and macros

library iceduino;

library neorv32;
use neorv32.neorv32_package.all; -- for device primitives and macros

entity neorv32_iceduino_top is
  port (
    -- Clocks --
    clk_12mhz : in  std_ulogic;
    clk_50mhz : in  std_ulogic;

    -- Simple I/Os --
    led : out std_ulogic_vector(7 downto 0);    
    btn : in std_ulogic_vector(4 downto 0);
    sw  : in std_ulogic_vector(1 downto 0);

    -- -- PMOD --
    pmod_pwr_en : out std_ulogic;
    pmod1 : inout std_logic_vector(7 downto 0);
    pmod2 : inout std_logic_vector(7 downto 0);
    pmod3 : inout std_logic_vector(7 downto 0);

    -- -- Arduino Header --
    -- oe_j5 : out std_ulogic;
    -- oe_j6 : out std_ulogic;
    io_d : inout std_ulogic_vector(9 downto 2);
    io_tx : out std_ulogic;
    io_rx : in std_ulogic;
    io_scl : out std_ulogic;
    io_sda : inout std_ulogic;
    io_miso : in std_ulogic;
    io_mosi : out std_ulogic;
    io_sck : out std_ulogic;
    io_ss : out std_ulogic;

    -- -- ADC --
    adc_scl : out std_ulogic;
    adc_sda : inout std_ulogic;

    -- USB - UART/RS232
    uart_tx : out std_ulogic;
    uart_rx : in  std_ulogic;
    uart_rts : out std_ulogic;
    uart_cts : in std_ulogic;

    -- SPI --
    -- In standard mode:
    -- flash_io[0] --> outgoing data
    -- flash_io[1] --> ingoing data
    -- flash_io[2..3] --> should be set to '1' at all times
    -- flash_io : inout std_ulogic_vector(3 downto 0); 
    flash_sdo : out std_ulogic;
    flash_sdi : in std_ulogic;
    flash_sck : out std_ulogic;
    flash_csn : out std_ulogic
  );
end entity;

architecture neorv32_iceduino_top_rtl of neorv32_iceduino_top is

  -- configuration --
    constant f_clock_c : natural := 50000000; -- clock frequency in Hz
	signal external_rstn : std_ulogic;
	
    -- wishbone: shared with all slaves
    signal reg_adr_o       : std_ulogic_vector(31 downto 0):=(others => '0'); -- address	
    signal reg_dat_o       : std_ulogic_vector(31 downto 0):=(others => '0'); -- write data
    signal reg_we_o        : std_ulogic:= '0'; -- read/write
    signal reg_stb_o       : std_ulogic := '0'; -- strobe
    signal reg_cyc_o       : std_ulogic := '0'; -- valid cycle
    -- wishbone: arbiter for slave outputs
    signal arb_dat_i       : std_logic_vector(31 downto 0):=(others => 'U'); -- read data
    signal arb_ack_i       : std_logic := 'L'; -- transfer acknowledge
	signal led_dat_i       : std_logic_vector(31 downto 0):=(others => 'L'); 
	signal sw_dat_i       : std_logic_vector(31 downto 0):=(others => 'L'); 
	signal btn_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal pmod1_dat_i       : std_logic_vector(31 downto 0):=(others => 'L'); 
	signal pmod2_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal pmod3_dat_i       : std_logic_vector(31 downto 0):=(others => 'L'); 
	signal gpio_header_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal arduino_i2c_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal arduino_spi_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal arduino_uart_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	signal adc_dat_i       : std_logic_vector(31 downto 0):=(others => 'L');
	
	-- internal IO connection --
    signal con_gpio : std_ulogic_vector(63 downto 0);
    signal con_spi_csn  : std_ulogic_vector(07 downto 0);
    
    -- external IO connection --
    signal con_led : std_ulogic_vector(7 downto 0);   
    --signal con_pmod_en : std_ulogic; 
    signal con_pmod1 : std_logic_vector(7 downto 0);   
    signal con_pmod2 : std_logic_vector(7 downto 0);   
    signal con_pmod3 : std_logic_vector(7 downto 0);
    signal con_io : std_logic_vector(7 downto 0);
    signal con_tx : std_logic;
    signal con_scl : std_logic;
    signal con_sda : std_logic;
    signal con_mosi : std_logic;
    signal con_sck : std_logic;
    signal con_ss : std_logic;    
    signal con_adc_scl : std_logic;
    signal con_adc_sda : std_logic;

begin

	process(clk_50mhz) is
	variable cnt : unsigned(7 downto 0) := (others => '0');
	begin
	  if rising_edge(clk_50mhz) then
		if cnt < 255 then
		  cnt := cnt + 1;
		  external_rstn <= '0';
		else
		  external_rstn <= '1';
		end if;
		
		--arbiter for data slave -> master
        if(reg_adr_o(31 downto 8) = x"FFFF80") then
            case reg_adr_o(7 downto 0) is
                when x"00" =>
                        arb_dat_i <= led_dat_i;						
                when x"08" =>
                        arb_dat_i <= sw_dat_i;
                when x"0A" =>
                        arb_dat_i <= btn_dat_i;	
                when x"10" =>
                        arb_dat_i <= pmod1_dat_i;
				when x"18" =>
                        arb_dat_i <= pmod1_dat_i;		
                when x"20" =>
                        arb_dat_i <= pmod2_dat_i;
				when x"28" =>
                        arb_dat_i <= pmod2_dat_i;		
                when x"30" =>
                        arb_dat_i <= pmod3_dat_i;
				when x"38" =>
                        arb_dat_i <= pmod3_dat_i;		
                when x"40" =>
                        arb_dat_i <= gpio_header_dat_i;
				when x"48" =>
                        arb_dat_i <= gpio_header_dat_i;		
                when x"50" =>
                        arb_dat_i <= arduino_i2c_dat_i;	
                when x"51" =>
                        arb_dat_i <= arduino_spi_dat_i;	
                when x"52" =>
                        arb_dat_i <= arduino_uart_dat_i;	
                when x"58" =>
                        arb_dat_i <= adc_dat_i;	
                when others =>
                        arb_dat_i <= (others => 'L');						
            end case;
        end if;
	  end if;
	end process;

	neorv32_inst: neorv32_top
	generic map (
		-- General --
		CLOCK_FREQUENCY              => f_clock_c,           -- clock frequency of clk_i in Hz
		HW_THREAD_ID                 => 0,      -- hardware thread id (32-bit)
		INT_BOOTLOADER_EN            => true,  -- boot configuration: true = boot explicit bootloader, false = boot from int/ext (I)MEM

		-- On-Chip Debugger (OCD) --
		ON_CHIP_DEBUGGER_EN          => false,  -- implement on-chip debugger

		-- RISC-V CPU Extensions --
		CPU_EXTENSION_RISCV_A        => false,  -- implement atomic extension?
		CPU_EXTENSION_RISCV_C        => false,  -- implement compressed extension?
		CPU_EXTENSION_RISCV_E        => false,  -- implement embedded RF extension?
		CPU_EXTENSION_RISCV_M        => false,  -- implement mul/div extension?
		CPU_EXTENSION_RISCV_U        => false,  -- implement user mode extension?
		CPU_EXTENSION_RISCV_Zfinx    => false,  -- implement 32-bit floating-point extension (using INT regs!)
		CPU_EXTENSION_RISCV_Zicsr    => true,   -- implement CSR system?
		CPU_EXTENSION_RISCV_Zifencei => false,  -- implement instruction stream sync.?
		CPU_EXTENSION_RISCV_Zmmul    => false,  -- implement multiply-only M sub-extension?

		-- Extension Options --
		FAST_MUL_EN                  => false,  -- use DSPs for M extension's multiplier
		FAST_SHIFT_EN                => false,  -- use barrel shifter for shift operations
		CPU_CNT_WIDTH                => 64,     -- total width of CPU cycle and instret counters (0..64)
		CPU_IPB_ENTRIES              => 2,      -- entries is instruction prefetch buffer, has to be a power of 2

		-- Physical Memory Protection (PMP) --
		PMP_NUM_REGIONS              => 0,      -- number of regions (0..64)
		PMP_MIN_GRANULARITY          => 64*1024, -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes

		-- Hardware Performance Monitors (HPM) --
		HPM_NUM_CNTS                 => 0,      -- number of implemented HPM counters (0..29)
		HPM_CNT_WIDTH                => 40,     -- total size of HPM counters (0..64)

		-- Internal Instruction memory (IMEM) --
		MEM_INT_IMEM_EN              => true,  -- implement processor-internal instruction memory
		MEM_INT_IMEM_SIZE            => 8*1024, -- size of processor-internal instruction memory in bytes

		-- Internal Data memory (DMEM) --
		MEM_INT_DMEM_EN              => true,  -- implement processor-internal data memory
		MEM_INT_DMEM_SIZE            => 2*1024, -- size of processor-internal data memory in bytes

		-- Internal Cache memory (iCACHE) --
		ICACHE_EN                    => false,  -- implement instruction cache
		ICACHE_NUM_BLOCKS            => 4,      -- i-cache: number of blocks (min 1), has to be a power of 2
		ICACHE_BLOCK_SIZE            => 64,     -- i-cache: block size in bytes (min 4), has to be a power of 2
		ICACHE_ASSOCIATIVITY         => 1,      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

		-- External memory interface (WISHBONE) --
		MEM_EXT_EN                   => true,  -- implement external memory bus interface?
		MEM_EXT_TIMEOUT              => 0,    -- cycles after a pending bus access auto-terminates (0 = disabled, default = 255)
		MEM_EXT_PIPE_MODE            => false,  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
		MEM_EXT_BIG_ENDIAN           => false,  -- byte order: true=big-endian, false=little-endian
		MEM_EXT_ASYNC_RX             => false,  -- use register buffer for RX data when false

		-- Stream link interface (SLINK) --
		SLINK_NUM_TX                 => 0,      -- number of TX links (0..8)
		SLINK_NUM_RX                 => 0,      -- number of TX links (0..8)
		SLINK_TX_FIFO                => 1,      -- TX fifo depth, has to be a power of two
		SLINK_RX_FIFO                => 1,      -- RX fifo depth, has to be a power of two

		-- External Interrupts Controller (XIRQ) --
		XIRQ_NUM_CH                  => 0,      -- number of external IRQ channels (0..32)
		XIRQ_TRIGGER_TYPE            => x"FFFFFFFF", -- trigger type: 0=level, 1=edge
		XIRQ_TRIGGER_POLARITY        => x"FFFFFFFF", -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

		-- Processor peripherals --
		IO_GPIO_EN                   => true,  -- implement general purpose input/output port unit (GPIO)?
		IO_MTIME_EN                  => true,  -- implement machine system timer (MTIME)?
		IO_UART0_EN                  => true,  -- implement primary universal asynchronous receiver/transmitter (UART0)?
		IO_UART1_EN                  => false,  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
		IO_SPI_EN                    => true,  -- implement serial peripheral interface (SPI)?
		IO_TWI_EN                    => false,  -- implement two-wire interface (TWI)?
		IO_PWM_NUM_CH                => 0,      -- number of PWM channels to implement (0..60), 0 = disabled
		IO_WDT_EN                    => true,  -- implement watch dog timer (WDT)?
		IO_TRNG_EN                   => false,  -- implement true random number generator (TRNG)?
		IO_CFS_EN                    => false,  -- implement custom functions subsystem (CFS)?
		IO_CFS_CONFIG                => x"00000000", -- custom CFS configuration generic
		IO_CFS_IN_SIZE               => 32,    -- size of CFS input conduit in bits
		IO_CFS_OUT_SIZE              => 32,    -- size of CFS output conduit in bits
		IO_NEOLED_EN                 => false,  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
		IO_NEOLED_TX_FIFO            => 1       -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
	)
	port map (
		-- Global control --
		clk_i          => clk_50mhz, -- global clock, rising edge
		rstn_i         => external_rstn, -- global reset, low-active, async

		-- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
		jtag_trst_i    => 'U', -- low-active TAP reset (optional)
		jtag_tck_i     => 'U', -- serial clock
		jtag_tdi_i     => 'U', -- serial data input
		jtag_tdo_o     => open,        -- serial data output
		jtag_tms_i     => 'U', -- mode select

		-- Wishbone bus interface (available if MEM_EXT_EN = true) --
		wb_tag_o       => open, -- request tag
		wb_adr_o       => reg_adr_o, -- address
		wb_dat_i       => arb_dat_i, -- read data
		wb_dat_o       => reg_dat_o, -- write data
		wb_we_o        => reg_we_o, -- read/write
		wb_sel_o       => open, -- byte enable
		wb_stb_o       => reg_stb_o, -- strobe
		wb_cyc_o       => reg_cyc_o, -- valid cycle
		wb_lock_o      => open, -- exclusive access request
		wb_ack_i       => arb_ack_i, -- transfer acknowledge
		wb_err_i       => '0', -- transfer error


		-- Advanced memory control signals (available if MEM_EXT_EN = true) --
		fence_o        => open, -- indicates an executed FENCE operation
		fencei_o       => open, -- indicates an executed FENCEI operation

		-- TX stream interfaces (available if SLINK_NUM_TX > 0) --
		slink_tx_dat_o => open, -- output data
		slink_tx_val_o => open, -- valid output
		slink_tx_rdy_i => (others => 'L'), -- ready to send

		-- RX stream interfaces (available if SLINK_NUM_RX > 0) --
		slink_rx_dat_i => (others => (others => 'U')), -- input data
		slink_rx_val_i => (others => 'L'), -- valid input
		slink_rx_rdy_o => open, -- ready to receive

		-- GPIO (available if IO_GPIO_EN = true) --
		gpio_o         => con_gpio, -- parallel output
		gpio_i         => (others => 'U'), -- parallel input

		-- primary UART0 (available if IO_UART0_EN = true) --
		uart0_txd_o    => uart_tx, -- UART0 send data
		uart0_rxd_i    => uart_rx, -- UART0 receive data
		uart0_rts_o    => uart_rts, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
		uart0_cts_i    => uart_cts, -- hw flow control: UART0.TX allowed to transmit, low-active, optional

		-- secondary UART1 (available if IO_UART1_EN = true) --
		uart1_txd_o    => open, -- UART1 send data
		uart1_rxd_i    => 'U', -- UART1 receive data
		uart1_rts_o    => open, -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
		uart1_cts_i    => 'L', -- hw flow control: UART1.TX allowed to transmit, low-active, optional

		-- SPI (available if IO_SPI_EN = true) --
		spi_sck_o      => flash_sck, -- SPI serial clock
		spi_sdo_o      => flash_sdo, -- controller data out, peripheral data in
		spi_sdi_i      => flash_sdi, -- controller data in, peripheral data out
		spi_csn_o      => con_spi_csn, -- chip-select

		-- TWI (available if IO_TWI_EN = true) --
		-- twi_sda_io     => 'U', -- twi serial data line
		-- twi_scl_io     => 'U', -- twi serial clock line

		-- PWM (available if IO_PWM_NUM_CH > 0) --
		pwm_o          => open, -- pwm channels

		-- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
		cfs_in_i       => (others => 'U'), -- custom CFS inputs conduit
		cfs_out_o      => open, -- custom CFS outputs conduit

		-- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
		neoled_o       => open, -- async serial data line

		-- System time --
		mtime_i        => (others => 'U'), -- current system time from ext. MTIME (if IO_MTIME_EN = false)
		mtime_o        => open, -- current system time from int. MTIME (if IO_MTIME_EN = true)

		-- External platform interrupts (available if XIRQ_NUM_CH > 0) --
		xirq_i         => (others => 'L'), -- IRQ channels

		-- CPU interrupts --
		nm_irq_i       => 'L', -- non-maskable interrupt
		mtime_irq_i    => 'L', -- machine timer interrupt, available if IO_MTIME_EN = false
		msw_irq_i      => 'L', -- machine software interrupt
		mext_irq_i     => 'L'  -- machine external interrupt
	);

  
	-- module instance led --
    iceduino_led_inst: entity iceduino.iceduino_led
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        --wishbone-
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  led_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        led_o       =>  con_led --io       
    );
    
	-- module instance switch --
    iceduino_switch_inst: entity iceduino.iceduino_switch
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  sw_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        switch_i    =>  sw --io

    );
    
	-- module instance button --
    iceduino_button_inst: entity iceduino.iceduino_button
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  btn_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        button_i    => 	btn --io
    );
    
	-- module instance pmod1 --
    iceduino_gpio_pmod1_inst: entity iceduino.iceduino_pmod
    generic map (
        pmod_instance_addr_i  => x"FFFF8018",
        pmod_instance_addr_o  => x"FFFF8010"
    )
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  pmod1_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,                           
        pmod_io     => 	con_pmod1 --io 
    );
    
	-- module instance pmod2 --
    iceduino_gpio_pmod2_inst: entity iceduino.iceduino_pmod
    generic map (
        pmod_instance_addr_i  => x"FFFF8028",
        pmod_instance_addr_o  => x"FFFF8020"
    )
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  pmod2_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,                           
        pmod_io     => 	con_pmod2 --io 
    );
    
	-- module instance pmod3 --
    iceduino_gpio_pmod3_inst: entity iceduino.iceduino_pmod
    generic map (
        pmod_instance_addr_i  => x"FFFF8038",
        pmod_instance_addr_o  => x"FFFF8030"
    )
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  pmod3_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,                           
        pmod_io     => 	con_pmod3 --io 
    );
    
	-- module instance arduino gpio --
    iceduino_arduino_gpio_inst: entity iceduino.iceduino_arduino_gpio
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  gpio_header_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        io       	=>  con_io --io
    );
    
    -- module instance arduino uart --
    iceduino_arduino_uart_inst: entity iceduino.iceduino_arduino_uart
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  arduino_uart_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        tx_o       	=>  con_tx,
        rx_i       	=>  io_rx
    );
    
    -- module instance arduino spi --
    iceduino_arduino_spi_inst: entity iceduino.iceduino_arduino_spi
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  arduino_spi_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        miso_i      =>  io_miso,
        mosi_o      =>  con_mosi,
        sck_o       =>  con_sck,
        ss_o        =>  con_ss
    );
    
    -- module instance arduino i2c --
    iceduino_arduino_i2c_inst: entity iceduino.iceduino_arduino_i2c
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  arduino_i2c_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        scl_o       =>  con_scl,
        sda         =>  con_sda       
    );
    
    -- module instance adc --
    iceduino_arduino_adc_inst: entity iceduino.iceduino_arduino_adc
    port map (
        clk_i  		=>  clk_50mhz,
        rstn_i 		=>  external_rstn,
        adr_i		=>	reg_adr_o,
        dat_i	    =>  reg_dat_o, --write to slave
        dat_o	    =>  adc_dat_i,
        we_i        =>  reg_we_o,
        stb_i		=>	reg_stb_o,
        cyc_i       =>  reg_cyc_o,
        ack_o       =>  arb_ack_i,
        scl_o       =>  con_adc_scl,
        sda         =>  con_adc_sda       
    );
	
	-- outputs internal --
	flash_csn <= con_spi_csn(0);
	
	-- outputs external --	
    led <= con_led;   
    pmod_pwr_en <= '1';
    pmod1 <= con_pmod1;
    pmod2 <= con_pmod2;
    pmod3 <= con_pmod3;
    io_d <= con_io;
    io_tx <= con_tx;
    io_mosi <= con_mosi;
    io_sck <= con_sck;
    io_ss <= con_ss;
    io_scl <= con_scl;
    io_sda <= con_sda;
    adc_scl <= con_adc_scl;
    adc_sda <= con_adc_sda;
    
end architecture;