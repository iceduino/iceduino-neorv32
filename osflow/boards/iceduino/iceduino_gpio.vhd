library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity iceduino_gpio is
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
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
    err_o  	: out  std_ulogic; 
    -- parallel io --
    gpio_o : out std_ulogic_vector(31 downto 0);
    gpio_i : in  std_ulogic_vector(31 downto 0)
  );
end entity;

architecture iceduino_gpio_rtl of iceduino_gpio is

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address

  -- accessible regs --
  signal din  : std_ulogic_vector(31 downto 0); -- r/-
  signal dout : std_ulogic_vector(31 downto 0); -- r/w
  
  constant gpio_addr_i : std_ulogic_vector(31 downto 0) := x"A0000000"; 
  constant gpio_addr_o : std_ulogic_vector(31 downto 0) := x"A0000004"; 

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (((adr_i = gpio_addr_i) or (adr_i = gpio_addr_o)) and (cyc_i = '1' and stb_i = '1')) else '0';
  addr   <= adr_i;

  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus handshake --
      ack_o <= acc_en;
      err_o <= '0';
      

      -- write access --
      if ((acc_en and we_i) = '1') then
        if (addr = gpio_addr_o) then
          dout <= dat_i;
        end if;  
      end if;

      -- input buffer --
      din <= gpio_i(31 downto 00);    

      -- read access --
      dat_o <= (others => '0');
      if ((acc_en and (not we_i)) = '1') then
        case addr is
          when gpio_addr_i  => dat_o <= din;         
          when gpio_addr_o => dat_o <= dout;       
          when others             => dat_o <= (others => '0');
        end case;
      end if;

    end if;
  end process rw_access;

  -- output --
  gpio_o <= dout;


end architecture ;
