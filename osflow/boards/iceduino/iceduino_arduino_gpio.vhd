library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_arduino_gpio is
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
    io : inout std_logic_vector(7 downto 0)
  );
end entity;

architecture iceduino_arduino_gpio_rtl of iceduino_arduino_gpio is

  signal module_active : std_ulogic;
  signal module_addr   : std_ulogic_vector(31 downto 0);
  constant gpio_addr_o : std_ulogic_vector(31 downto 0) := x"FFFF8050"; 
  constant gpio_addr_i : std_ulogic_vector(31 downto 0) := x"FFFF8048"; 
  signal din  : std_logic_vector(7 downto 0); 
  signal dout : std_logic_vector(7 downto 0); 

begin

  -- module active
  module_active <= '1' when (((adr_i = gpio_addr_i) or (adr_i = gpio_addr_o)) and (cyc_i = '1' and stb_i = '1')) else '0';
  module_addr   <= adr_i;

  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
       -- handshake
      if (module_active = '1') then
        ack_o <= '1';
      else   
        ack_o <= 'Z';
      end if;
     
      
      -- write access --
      if ((module_active and we_i) = '1') then
        if (module_addr = gpio_addr_o) then
          dout <= dat_i(7 downto 0);
        end if;  
      end if;

      -- input buffer --
      din <= io(7 downto 0);    

      -- read access --
      dat_o <= (others => 'Z');
      if ((module_active and (not we_i)) = '1') then
        case module_addr is
          when gpio_addr_i  => dat_o(7 downto 0) <= din;         
          when gpio_addr_o => dat_o(7 downto 0) <= dout;       
          when others             => dat_o <= (others => '0');
        end case;
      end if;

    end if;
  end process rw_access;

  -- output --
  io <= dout when we_i = '0' else (others => 'Z');


end architecture ;
