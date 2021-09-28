library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_button is
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
    button_i : in  std_ulogic_vector(4 downto 0)
  );
end entity;

architecture iceduino_button_rtl of iceduino_button is

  signal module_active : std_ulogic;
  signal module_addr   : std_ulogic_vector(31 downto 0);
  signal reg_button  : std_ulogic_vector(4 downto 0);  
  constant button_addr : std_ulogic_vector(31 downto 0) := x"FFFF8010"; 

begin
  -- module active
  module_active <= '1' when ((adr_i = button_addr) and (cyc_i = '1' and stb_i = '1')) else '0';
  module_addr   <= adr_i;

  r_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
	  -- handshake
      if (module_active = '1') then
        ack_o <= '1';
      else   
        ack_o <= 'Z';
      end if;
	  -- read access	
      reg_button <= button_i(4 downto 0); 	  
      dat_o <= (others => 'Z');
      if ((module_active and (not we_i)) = '1') then
        if (module_addr = button_addr) then
            dat_o(31 downto 5) <= (others => '0');
            dat_o(4 downto 0) <= reg_button;
        end if;
      end if;
    end if;
  end process r_access;



end architecture ;
