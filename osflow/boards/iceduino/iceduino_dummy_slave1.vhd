library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity iceduino_dummy_slave1 is
	port (      
		clk_i	: in  std_ulogic; -- global clock
		rstn_i 	: in  std_ulogic; -- global reset line, low-active
		--wishbone-
		tag_i  	: in  std_ulogic_vector(2 downto 0);
		adr_i 	: in  std_ulogic_vector(31 downto 0); 
		dat_i	: in  std_ulogic_vector(31 downto 0); --write to led
		dat_o	: out std_ulogic_vector(31 downto 0);       
		we_i  	: in  std_ulogic;
		sel_i  	: in  std_ulogic_vector(3 downto 0);
		stb_i  	: in  std_ulogic;
		cyc_i  	: in  std_ulogic;
		lock_i  : in  std_ulogic;
		ack_o  	: out  std_ulogic;
		err_o  	: out  std_ulogic 
	);
end entity;

architecture iceduino_dummy_slave1_rtl of iceduino_dummy_slave1 is

	constant dummy_adr : std_ulogic_vector(31 downto 0) := x"A0000000"; --address of slave
	signal active : std_ulogic:='0';
begin
    process(rstn_i, clk_i)
    begin
        if rising_edge(clk_i) then           
            active <= '1' when (adr_i(31 downto 0) = dummy_adr(31 downto 0)) and (cyc_i = '1' and stb_i = '1') else 'Z';        
        end if;
    end process;
    
    --outputs
	ack_o <= active;
	err_o <= 'Z';	
	dat_o <= (others => 'L');
	
end architecture;
 
