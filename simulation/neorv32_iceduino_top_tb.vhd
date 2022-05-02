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

library iceduino;

entity neorv32_iceduino_top_tb is
end entity;

architecture neorv32_iceduino_top_sim of neorv32_iceduino_top_tb is

  signal clk : std_ulogic := '0';
  signal led : std_ulogic_vector(7 downto 0);
  signal sw : std_ulogic_vector(1 downto 0);
  signal btn : std_ulogic_vector(4 downto 0);

begin

  clk <= not clk after 10 ns;
  sw <= "11", "10" after 550 us, "01" after 560 us, "00" after 570 us; --low_active
  btn <= "11111", "11110" after 580 us, "11100" after 590 us, "11000" after 600 us, "10000" after 610 us, "00000" after 620 us; --low_active

  neorv32_iceduino_inst: entity iceduino.neorv32_iceduino_top
  port map (
    clk_12mhz   => '0',
    clk_50mhz   => clk,
    led         => led, 
    sw          => sw,
    btn         => btn,
    pmod1       => open,
    pmod2       => open,
    pmod3       => open,
    oe_j5       => open,
    oe_j6       => open,
    io_d        => open,
    io_tx       => open,
    io_rx       => '0',
    io_scl      => open,
    io_sda      => open,
    io_miso     => '0',
    io_mosi     => open,
    io_sck      => open,
    io_ss       => open,
    adc_scl     => open,
    adc_sda     => open,
    uart_tx     => open,
    uart_rx     => '0',
    uart_rts    => open,
    uart_cts    =>'0',    
    flash_sdo   => open,
    flash_sdi   => '0',
    flash_sck   => open,
    flash_csn   => open
  );


    
end architecture;
