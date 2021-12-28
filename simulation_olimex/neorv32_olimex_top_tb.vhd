-- #################################################################################################
-- # << NEORV32 - Setup for the Iceduino Board >>
-- # Schematics available at https://github.com/olimex/olimex                                  #
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

library olimex;

entity neorv32_olimex_top_tb is
end entity;

architecture neorv32_olimex_top_sim of neorv32_olimex_top_tb is

  signal clk : std_ulogic := '0';
  signal btn : std_ulogic_vector(1 downto 0);
  signal led : std_ulogic_vector(1 downto 0);

begin

  clk <= not clk after 10 ns;
  btn <= "00", "01" after 550 us, "00" after 560 us, "01" after 570 us;

  neorv32_olimex_inst: entity olimex.neorv32_olimex_top_sim
  port map (
    SYSCLK  => clk,
    LED        => led,    
    BTN         => btn,
    IOL_1A      => open,
    IOL_1B      => open,
    IOL_2A      => open,
    IOL_2B      => open,
    IOL_3A      => open,
    IOL_3B      => open,
    IOL_4A      => open,
    IOL_4B      => open,
    IOL_5A      => open,
    IOL_5B      => open,
    IOL_6A      => open,
    IOL_6B      => open,
    IOL_7A      => open,
    IOL_7B      => open,
    IOL_8A      => open,
    IOL_8B      => open,
    IOL_9A      => open,
    IOL_9B      => open,
    IOL_10A      => open,
    IOL_10B      => open,
    IOL_11A      => open,
    IOL_11B      => open,
    IOL_12A      => open,
    IOL_12B      => open,
    IOL_13A      => open,
    IOL_13B      => open,
    IOL_14B      => open,
    IOL_15A      => open
  );


    
end architecture;
