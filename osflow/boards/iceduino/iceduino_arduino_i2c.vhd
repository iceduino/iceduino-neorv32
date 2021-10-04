library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--#########
--##dummy
--#########

entity iceduino_arduino_i2c is
  port (
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i 	: in  std_ulogic; -- global reset line, low-active
    --wishbone-
    adr_i 	: in  std_ulogic_vector(31 downto 0); 
    dat_i	: in  std_ulogic_vector(31 downto 0); --write to slave
    dat_o	: out std_ulogic_vector(31 downto 0);       
    we_i  	: in  std_ulogic;
    stb_i  	: in  std_ulogic;
    cyc_i  	: in  std_ulogic;
    ack_o  	: out  std_ulogic;
    scl_o  	: out  std_ulogic;
    sda  	: inout  std_ulogic
  );
end entity;

architecture iceduino_arduino_i2c_rtl of iceduino_arduino_i2c is

  signal module_active : std_ulogic;
  signal module_addr   : std_ulogic_vector(31 downto 0);
  constant m_addr : std_ulogic_vector(31 downto 0) := x"FFFF8068"; 


begin

  -- module active
  module_active <= '1' when ((adr_i = m_addr) and (cyc_i = '1' and stb_i = '1')) else '0';
  module_addr   <= adr_i;

  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
       -- handshake
      if (module_active = '1') then
        ack_o <= '1';
      else   
        ack_o <= '0';
      end if;
      -- read access --
      dat_o <= (others => '0');


    end if;
  end process rw_access;

  -- output --
  sda <= 'Z';
  scl_o <= '0';


end architecture ;
