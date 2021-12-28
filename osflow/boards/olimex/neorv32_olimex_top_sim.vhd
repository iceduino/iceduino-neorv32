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

library olimex;

library neorv32;
use neorv32.neorv32_package.all; -- for device primitives and macros

entity neorv32_olimex_top_sim is
port (
    SYSCLK : in  std_ulogic;

    -- Simple I/Os --    
    LED : out std_ulogic_vector(1 downto 0);
    
    BTN : in std_ulogic_vector(1 downto 0);
    IOL_1A : inout std_ulogic;
    IOL_1B : inout std_ulogic;
    IOL_2A : inout std_ulogic;
    IOL_2B : inout std_ulogic;
    IOL_3A : inout std_ulogic;
    IOL_3B : inout std_ulogic;
    IOL_4A : inout std_ulogic;
    IOL_4B : inout std_ulogic;
    IOL_5A : inout std_ulogic;
    IOL_5B : inout std_ulogic;
    IOL_6A : inout std_ulogic;
    IOL_6B : inout std_ulogic;
    IOL_7A : inout std_ulogic;
    IOL_7B : inout std_ulogic;
    IOL_8A : inout std_ulogic;
    IOL_8B : inout std_ulogic;
    IOL_9A : inout std_ulogic;
    IOL_9B : inout std_ulogic;
    IOL_10A : inout std_ulogic;
    IOL_10B : inout std_ulogic;
    IOL_11A : inout std_ulogic;
    IOL_11B : inout std_ulogic;
    IOL_12A : inout std_ulogic;
    IOL_12B : inout std_ulogic;
    IOL_13A : inout std_ulogic;
    IOL_13B : inout std_ulogic;
    IOL_14B : inout std_ulogic;
    IOL_15A : inout std_ulogic
);
end entity;

architecture neorv32_olimex_top_rtl of neorv32_olimex_top_sim is

-- configuration --
constant f_clock_c : natural := 50000000; -- clock frequency in Hz
signal external_rstn : std_ulogic;




-- bus_wishbone --
type bus_wishbone_t is record
    tag_o       : std_ulogic_vector(02 downto 0); -- request tag
    adr_o       : std_ulogic_vector(31 downto 0); -- address      
    dat_o       : std_ulogic_vector(31 downto 0); -- write data
    we_o        : std_ulogic; -- read/write
    sel_o       : std_ulogic_vector(03 downto 0); -- byte enable
    stb_o       : std_ulogic; -- strobe
    cyc_o       : std_ulogic; -- valid cycle
    lock_o      : std_ulogic; -- exclusive access request      
end record;    
signal master_bus : bus_wishbone_t;
	
-- bus_wishbone slave response  --
type slave_resp_t is record
    rdata_i : std_ulogic_vector(31 downto 0);
    ack_i   : std_ulogic;
    err_i   : std_ulogic;
end record;
constant slave_resp_default : slave_resp_t := (rdata_i => (others => '0'), ack_i => '0', err_i => '0');
signal active_slave_resp : slave_resp_t := slave_resp_default;
signal led_resp : slave_resp_t := slave_resp_default;
signal button_resp : slave_resp_t := slave_resp_default;
signal gpio_resp : slave_resp_t := slave_resp_default;


begin

    process(SYSCLK) is
    variable cnt : unsigned(7 downto 0) := (others => '0');
    begin
        if rising_edge(SYSCLK) then
            if cnt < 255 then
                cnt := cnt + 1;
                external_rstn <= '0';
            else
                external_rstn <= '1';
            end if;
        end if;
    end process;


    -- external bus multiplexer --
    bus_multiplexer: process(master_bus, led_resp,button_resp)
    begin
        active_slave_resp.rdata_i <= (others => '0');
        active_slave_resp.ack_i <= '0';
        active_slave_resp.err_i <= '0';
        if(master_bus.adr_o(31 downto 8) = x"F00000") then
            case master_bus.adr_o(7 downto 0) is
                when x"00" =>
                        active_slave_resp.rdata_i <= led_resp.rdata_i; 
                        active_slave_resp.ack_i <= led_resp.ack_i;
                        active_slave_resp.err_i <= led_resp.err_i;
                when x"10" =>
                        active_slave_resp.rdata_i <= button_resp.rdata_i; 
                        active_slave_resp.ack_i <= button_resp.ack_i;
                        active_slave_resp.err_i <= button_resp.err_i;              
                when x"48" =>
                        active_slave_resp.rdata_i <= gpio_resp.rdata_i; 
                        active_slave_resp.ack_i <= gpio_resp.ack_i;
                        active_slave_resp.err_i <= gpio_resp.err_i;  
                when x"50" =>
                        active_slave_resp.rdata_i <= gpio_resp.rdata_i; 
                        active_slave_resp.ack_i <= gpio_resp.ack_i;
                        active_slave_resp.err_i <= gpio_resp.err_i;               
                when others =>
                        active_slave_resp.rdata_i <= (others => '0');
                        active_slave_resp.ack_i <= '0';
                        active_slave_resp.err_i <= '0';
            end case;
        end if;
    end process;

	neorv32_inst: neorv32_top
	generic map (
		-- General --
		CLOCK_FREQUENCY              => f_clock_c,           -- clock frequency of clk_i in Hz
		HW_THREAD_ID                 => 0,      -- hardware thread id (32-bit)
		INT_BOOTLOADER_EN            => false,  -- boot configuration: true = boot explicit bootloader, false = boot from int/ext (I)MEM
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
		-- Internal Instruction memory (IMEM) --
		MEM_INT_IMEM_EN              => true,  -- implement processor-internal instruction memory
		MEM_INT_IMEM_SIZE            => 8*1024, -- size of processor-internal instruction memory in bytes
		-- Internal Data memory (DMEM) --
		MEM_INT_DMEM_EN              => true,  -- implement processor-internal data memory
		MEM_INT_DMEM_SIZE            => 2*1024, -- size of processor-internal data memory in bytes
		-- External memory interface (WISHBONE) --
		MEM_EXT_EN                   => true,  -- implement external memory bus interface?
		MEM_EXT_TIMEOUT              => 255,    -- cycles after a pending bus access auto-terminates (0 = disabled, default = 255)
		MEM_EXT_PIPE_MODE            => false,  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
		MEM_EXT_BIG_ENDIAN           => false,  -- byte order: true=big-endian, false=little-endian
		MEM_EXT_ASYNC_RX             => false  -- use register buffer for RX data when false
	)
	port map (
		-- Global control --
		clk_i          => SYSCLK, -- global clock, rising edge
		rstn_i         => external_rstn, -- global reset, low-active, async
		-- Wishbone bus interface (available if MEM_EXT_EN = true) --
		wb_tag_o       => open, -- request tag
		wb_adr_o       => master_bus.adr_o, -- address
		wb_dat_i       => active_slave_resp.rdata_i, -- read data
		wb_dat_o       => master_bus.dat_o, -- write data
		wb_we_o        => master_bus.we_o, -- read/write
		wb_sel_o       => open, -- byte enable
		wb_stb_o       => master_bus.stb_o, -- strobe
		wb_cyc_o       => master_bus.cyc_o, -- valid cycle
		wb_lock_o      => open, -- exclusive access request
		wb_ack_i       => active_slave_resp.ack_i, -- transfer acknowledge
		wb_err_i       => active_slave_resp.err_i, -- transfer error
		-- Advanced memory control signals (available if MEM_EXT_EN = true) --
		fence_o        => open, -- indicates an executed FENCE operation
		fencei_o       => open, -- indicates an executed FENCEI operation
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
    olimex_led_inst: entity olimex.olimex_led
    generic map (
        led_addr        =>  x"F0000000"
    )
    port map (
        clk_i  		=>  SYSCLK,
        rstn_i 		=>  external_rstn,       
        adr_i		=>	master_bus.adr_o,
        dat_i	    =>  master_bus.dat_o,
        dat_o	    =>  led_resp.rdata_i,
        we_i        =>  master_bus.we_o,
        stb_i		=>	master_bus.stb_o,
        cyc_i       =>  master_bus.cyc_o,
        ack_o       =>  led_resp.ack_i,
        err_o       =>  led_resp.err_i,
        led_o       =>  LED      
    );
    
    -- module instance button --
    olimex_button_inst: entity olimex.olimex_button
    generic map (
        button_addr        =>  x"F0000010"
    )
    port map (   
        clk_i  		=>  SYSCLK,
        rstn_i 		=>  external_rstn,
        adr_i		=>	master_bus.adr_o,
        dat_i	    =>  master_bus.dat_o,
        dat_o	    =>  button_resp.rdata_i,
        we_i        =>  master_bus.we_o,
        stb_i		=>	master_bus.stb_o,
        cyc_i       =>  master_bus.cyc_o,
        ack_o       =>  button_resp.ack_i,
        err_o       =>  button_resp.err_i,
        button_i    => 	BTN
    );
     
	-- module instance gpio --
    olimex_gpio_inst: entity olimex.olimex_gpio
    generic map (
        gpio_addr_o => x"F0000048",
        gpio_addr_i => x"F0000050"
    )
    port map (
        clk_i  		=>  SYSCLK,
        rstn_i 		=>  external_rstn,
        adr_i		=>	master_bus.adr_o,
        dat_i	    =>  master_bus.dat_o,
        dat_o	    =>  gpio_resp.rdata_i,
        we_i        =>  master_bus.we_o,
        stb_i		=>	master_bus.stb_o,
        cyc_i       =>  master_bus.cyc_o,
        ack_o       =>  gpio_resp.ack_i,
        err_o       =>  gpio_resp.err_i,
        io       	=>  open --io
    ); 

    
end architecture;
