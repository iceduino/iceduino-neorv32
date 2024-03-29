library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_pmod is
  generic (
    pmod_addr_o : std_ulogic_vector(31 downto 0);
    pmod_addr_i : std_ulogic_vector(31 downto 0)
  );
  port (
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i 	: in  std_ulogic; -- global reset line, low-active
    --wishbone
    adr_i 	: in  std_ulogic_vector(31 downto 0); 
    dat_i	: in  std_ulogic_vector(31 downto 0); --write to slave
    dat_o	: out std_ulogic_vector(31 downto 0);       
    we_i  	: in  std_ulogic;
    stb_i  	: in  std_ulogic;
    cyc_i  	: in  std_ulogic;
    ack_o  	: out  std_ulogic;
    err_o  	: out  std_ulogic;
    -- io 
    pmod_en : out  std_ulogic;
    pmod_io : inout std_logic_vector(7 downto 0) 
  );
end entity;

architecture iceduino_pmod_rtl of iceduino_pmod is

  signal module_active : std_ulogic; 
  signal module_addr   : std_ulogic_vector(31 downto 0);
  signal din  : std_ulogic_vector(7 downto 0); --read reg
  signal dout : std_ulogic_vector(7 downto 0); --write reg

begin

  -- module active
  module_active <= '1' when ((adr_i = pmod_addr_o or adr_i = pmod_addr_i) and (cyc_i = '1' and stb_i = '1')) else '0';
  module_addr   <= adr_i;

  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
	    -- handshake
      err_o <= '0';
      if (module_active = '1') then
        ack_o <= '1';
      else   
        ack_o <= '0';
      end if;        
      --write  
      if (module_active = '1' and we_i = '1') then
        dout <= dat_i(7 downto 0);          
      end if;
      --read    
      din <= pmod_io(7 downto 0);
      dat_o <= (others => '0');
      if (module_active = '1' and we_i = '0') then
		    if(module_addr = pmod_addr_o) then
			    dat_o(7 downto 0) <= dout; 
        end if;
        if(module_addr = pmod_addr_i) then
			    dat_o(7 downto 0) <= din; 
        end if;       
      end if;
    end if;
  end process rw_access;
 
  pmod_io <= dout when we_i = '1' else (others => 'Z');

end architecture ;
