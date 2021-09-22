library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_pmod is
  port (
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i 	: in  std_ulogic; -- global reset line, low-active
    --wishbone
    tag_i  	: in  std_ulogic_vector(2 downto 0);
    adr_i 	: in  std_ulogic_vector(31 downto 0); 
    dat_i	: in  std_ulogic_vector(31 downto 0); --write to slave
    dat_o	: out std_ulogic_vector(31 downto 0);       
    we_i  	: in  std_ulogic;
    sel_i  	: in  std_ulogic_vector(3 downto 0);
    stb_i  	: in  std_ulogic;
    cyc_i  	: in  std_ulogic;
    lock_i  : in  std_ulogic;
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
  constant pmod1_addr_o : std_ulogic_vector(31 downto 0) := x"FFFF8028";
  constant pmod1_addr_i : std_ulogic_vector(31 downto 0) := x"FFFF8030"; 
  --constant pmod2_addr_o : std_ulogic_vector(31 downto 0) := x"FFFF8038";
  --constant pmod2_addr_i : std_ulogic_vector(31 downto 0) := x"FFFF8040";
  --constant pmod3_addr_o : std_ulogic_vector(31 downto 0) := x"FFFF8048";
  --constant pmod3_addr_i : std_ulogic_vector(31 downto 0) := x"FFFF8050";
  signal din  : std_ulogic_vector(7 downto 0); --read reg
  signal dout : std_ulogic_vector(7 downto 0); --write reg

begin

  -- module active
  module_active <= '1' when (((adr_i = pmod1_addr_i) or (adr_i = pmod1_addr_o)) and (cyc_i = '1' and stb_i = '1')) else 'Z';
  module_addr   <= adr_i;

  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
	   -- handshake
      ack_o <= module_active;
      err_o <= '0';      

      if ((module_active and we_i) = '1') then
        if (module_addr(3 downto 0) = x"8") then
          dout <= dat_i(7 downto 0);
        end if;  
      end if;

      din <= pmod_io(7 downto 0);  

      dat_o <= (others => 'Z');
      if ((module_active and (not we_i)) = '1') then
		if(module_addr(3 downto 0) = x"8") then
            dat_o(31 downto 8) <= (others => '0');
			dat_o(7 downto 0) <= dout; 
		end if;
		if(module_addr(3 downto 0) = x"0") then
            dat_o(31 downto 8) <= (others => '0');
			dat_o(7 downto 0) <= din; 
		end if;       
      end if;

    end if;
  end process rw_access;

  pmod_en <= '1' when module_active = '1' else '0'; 
  pmod_io <= dout when we_i = '1' else (others => 'Z');
  
  dat_o <= din when module_active and not we_i else (others => 'L');


end architecture ;
