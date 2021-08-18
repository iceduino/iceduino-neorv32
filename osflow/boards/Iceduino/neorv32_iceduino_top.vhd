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

    -- PMOD --
    pmod_pwr_en : out std_ulogic;
    pmod1 : inout std_logic_vector(7 downto 0);
    pmod2 : inout std_logic_vector(7 downto 0);
    pmod3 : inout std_logic_vector(7 downto 0);

    -- Arduino Header --
    oe_j5 : out std_ulogic;
    oe_j6 : out std_ulogic;
    io_d : inout std_ulogic_vector(9 downto 2);
    io_tx : out std_ulogic;
    io_rx : in std_ulogic;
    io_scl : out std_ulogic;
    io_sda : inout std_ulogic;
    io_miso : in std_ulogic;
    io_mosi : out std_ulogic;
    io_sck : out std_ulogic;
    io_ss : out std_ulogic;

    -- ADC --
    adc_scl : out std_ulogic;
    adc_sda : inout std_ulogic;

    -- USB - UART/RS232
    uart_tx : out std_ulogic;
    uart_rx : in  std_ulogic;

    -- SPI --
    -- In standard mode:
    -- flash_io[0] --> outgoing data
    -- flash_io[1] --> ingoing data
    -- flash_io[2..3] --> should be set to '1' at all times
    flash_io : inout std_ulogic_vector(3 downto 0); 
    flash_sck : out std_ulogic;
    flash_csn : out std_ulogic
  );
end entity;

architecture neorv32_iceduino_top_rtl of neorv32_iceduino_top is

  -- configuration --
  constant f_clock_c : natural := 50000000; -- PLL output clock frequency in Hz

  -- Globals
  -- signal pll_rstn : std_logic;
  -- signal pll_clk  : std_logic;

  -- internal IO connection --
  signal con_gpio : std_logic_vector(63 downto 0);

  signal con_spi_csn  : std_ulogic_vector(07 downto 0);

begin

  -- -- System PLL -----------------------------------------------------------------------------
  -- -- -------------------------------------------------------------------------------------------
  -- -- Settings generated by icepll -i 12 -o 18:
  -- -- F_PLLIN:      12.000 MHz (given)
  -- -- F_PLLOUT:     18.000 MHz (requested)
  -- -- F_PLLOUT:     18.000 MHz (achieved)
  -- -- FEEDBACK:     SIMPLE
  -- -- F_PFD:        12.000 MHz
  -- -- F_VCO:        576.000 MHz
  -- -- DIVR:         0 (4'b0000)
  -- -- DIVF:        47 (7'b0101111)
  -- -- DIVQ:         5 (3'b101)
  -- -- FILTER_RANGE: 1 (3'b001)
  -- Pll_inst : SB_PLL40_CORE
  -- generic map (
  --   FEEDBACK_PATH => "SIMPLE",
  --   DIVR          =>  x"0",
  --   DIVF          => 7x"2F",
  --   DIVQ          => 3x"5",
  --   FILTER_RANGE  => 3x"1"
  -- )
  -- port map (
  --   REFERENCECLK    => clk_i,
  --   PLLOUTCORE      => open,
  --   PLLOUTGLOBAL    => pll_clk,
  --   EXTFEEDBACK     => '0',
  --   DYNAMICDELAY    => x"00",
  --   LOCK            => pll_rstn,
  --   BYPASS          => '0',
  --   RESETB          => '1',
  --   LATCHINPUTVALUE => '0',
  --   SDO             => open,
  --   SDI             => '0',
  --   SCLK            => '0'
  -- );

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: neorv32_top
  generic map (
	CLOCK_FREQUENCY              => f_clock_c,      -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => true,   -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    USER_CODE                    => x"0001ce40",
    HW_THREAD_ID                 => 0,      -- hardware thread id (32-bit)

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => false,   -- implement atomic extension?
    CPU_EXTENSION_RISCV_C        => false,   -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => false,  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => false,   -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => false,  -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => false,  -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicsr    => true,   -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => false,  -- implement instruction stream sync.?

    -- Extension Options --
    FAST_MUL_EN                  => false,  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => false,  -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => 2,     -- total width of CPU cycle and instret counters (0..64) default: 34

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => 0,       -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => 8, -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes, default: 64*1024

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => 0,       -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => 0,      -- total size of HPM counters (0..64) default: 40

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => true,    -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => 8*1024, -- size of processor-internal instruction memory in bytes

    -- Internal Data memory --
    MEM_INT_DMEM_EN              => true,    -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => 2*1024, -- size of processor-internal data memory in bytes

    -- Internal Cache memory --
    ICACHE_EN                    => false,  -- implement instruction cache
    ICACHE_NUM_BLOCKS            => 4,      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => 64,     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => 1,      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- Processor peripherals --
    IO_GPIO_EN                   => true,   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => true,   -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true,   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_SPI_EN                    => true,
    IO_PWM_NUM_CH                => 0,      -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    => true    -- implement watch dog timer (WDT)?
  )
  port map (
    -- Global control --
    clk_i      => clk_50mhz,
    rstn_i     => '1',

    -- GPIO --
    gpio_o     => con_gpio,

    -- primary UART --
    uart0_txd_o => uart_tx, -- UART0 send data
    uart0_rxd_i => uart_rx, -- UART0 receive data
    uart0_rts_o => open, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i => '0',  -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o   => flash_sck,                  -- SPI serial clock
    spi_sdo_o   => flash_io(0),                  -- controller data out, peripheral data in
    spi_sdi_i   => flash_io(1),                  -- controller data in, peripheral data out
    spi_csn_o   => con_spi_csn                  -- SPI CS
  );
  
  flash_csn <= con_spi_csn(0);

  led <= con_gpio(7 downto 0);
    
end architecture;
