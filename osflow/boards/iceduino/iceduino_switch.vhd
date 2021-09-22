library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_switch is
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
    -- io
    switch_i : in  std_ulogic_vector(1 downto 0)
  );
end entity;

architecture iceduino_switch_rtl of iceduino_switch is

  signal module_active : std_ulogic;
  signal module_addr   : std_ulogic_vector(31 downto 0);
  signal reg_switch  : std_ulogic_vector(1 downto 0);  
  constant switch_addr : std_ulogic_vector(31 downto 0) := x"FFFF8009";

begin
  -- module active
  module_active <= '1' when ((adr_i = switch_addr) and (cyc_i = '1' and stb_i = '1')) else 'Z';
  module_addr   <= adr_i;

  r_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
	  -- handshake
      ack_o <= module_active;
	  -- read access
      reg_switch <= switch_i; 
      dat_o <= (others => 'Z');
      if ((module_active and (not we_i)) = '1') then
        if (module_addr = switch_addr) then
            dat_o(31 downto 2) <= (others => '0');
            dat_o(1 downto 0) <= reg_switch;
        end if;
      end if;
    end if;
  end process r_access;

 


end architecture ;
