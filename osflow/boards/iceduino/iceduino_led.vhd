library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_led is
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
    -- parallel io --
    led_o : out std_ulogic_vector(7 downto 0)
  );
end entity;

architecture iceduino_led_rtl of iceduino_led  is

  signal module_active : std_ulogic;
  constant led_addr : std_ulogic_vector(31 downto 0) := x"FFFF8000";
  signal module_addr   : std_ulogic_vector(31 downto 0);
  signal reg_led : std_ulogic_vector(7 downto 0);
  

begin
  -- module active
  module_active <= '1' when ((adr_i = led_addr) and (cyc_i = '1' and stb_i = '1')) else '0';
  module_addr   <= adr_i;
  
  w_access: process(clk_i)
  begin
    if rising_edge(clk_i) then    
      -- handshake
      if (module_active = '1') then
        ack_o <= '1';
      else   
        ack_o <= 'Z';
      end if;
      -- write access
	  dat_o <= (others => 'L');
      if ((module_active and we_i) = '1') then
        if (module_addr = led_addr) then
          reg_led <= dat_i(7 downto 0);          
          dat_o(31 downto 8) <= (others => '0');
		  dat_o(7 downto 0) <= reg_led;
        end if;  
      end if;
    end if;
  end process w_access;

  -- output
  led_o <= reg_led;

end architecture ;
