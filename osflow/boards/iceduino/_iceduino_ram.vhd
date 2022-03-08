library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all

entity iceduino_ram is
    generic (
        ram_addr_inst : std_ulogic_vector(31 downto 0);
        ram_width_inst : natural
        ram_addr_data : std_ulogic_vector(31 downto 0);
        ram_width_data : natural
    );
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
        err_o  	: out  std_ulogic;
        tag_i   : in std_ulogic(2 downto 0);
        -- ram io --
        ram_data : inout std_ulogic_vector(7 downto 0);
        ram_addr : out std_ulogic_vector(18 downto 0);
        ram_ce : out std_ulogic_vector;
        ram_oe : out std_ulogic_vector;
        ram_we : out std_ulogic_vector
    );
end entity;

architecture iceduino_ram_rtl of iceduino_ram  is

    function get_data_start(inst_en:boolean; inst_end:std_ulogic_vector; i1:integer) 
    return integer is
    begin
        if s = '1' then
            return i1;
        else
            return i0;
        end if;
    end function;

    constant inst_enabled : boolean := ram_width_inst > 0;
    constant data_enabled : boolean := ram_width_data > 0;
    constant inst_start : std_ulogic_vector(18 downto 0) := others => '0';
    constant data_start : std_ulogic_vector(18 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(inst_start) + ram_width_inst);
    constant inst_end : std_ulogic_vector(18 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(data_start) - to_unsigned(1, 19));
    constant data_end : std_ulogic_vector(18 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(data_start) + ram_width_data - to_unsigned(1, 19));
    signal module_active : std_ulogic;
    signal module_addr   : std_ulogic_vector(18 downto 0);
    type t_access is (INSTRUCTION, DATA);
    signal access_type : t_access;
    type t_state is (REQUEST, RESPONSE);
    state : t_state = REQUEST;

begin
    -- module active
    module_active <= '1' when ((((UNSIGNED(adr_i) >= ram_addr_inst) and (UNSIGNED(adr_i) <= (UNSIGNED(ram_addr_inst) + ram_width_inst))) 
                        or ((UNSIGNED(adr_i) >= ram_addr_data) and (UNSIGNED(adr_i) <= (UNSIGNED(ram_addr_data) + ram_width_data))))
                        and (cyc_i = '1' and stb_i = '1')) 
                        else '0';
    access_type <= INSTRUCTION when tag_i = '1' else DATA
    module_addr <= adr_i when access_type = INSTRUCTION else UNSIGNED(adr_i)
    
    w_access: process(clk_i)
    begin
        if rising_edge(clk_i) then 
            case state is
                when REQUEST =>
                    if(module_active = '1') then
                        if(we_i = '1') then
                            ram_ce <= '0';
                            ram_we <= '0';
                when RESPONSE =>

                when others =>
                    state <= REQUEST;
            end case;
        end if;
    end process w_access;
    
    -- output
    led_o <= reg_led;  
  
end architecture ;


        -- w_access: process(clk_i)
    --     variable index : integer := 0;
    --     variable bytes_count : integer;
    -- begin
    --     if rising_edge(clk_i) then 
    --         if (rstn_i = '0') then
    --             ack_o <= '0';
    --             err_o <= '0';
    --             ram_ce <= '1';
    --             ram_oe <= '1';
    --             ram_we <= '1';
    --             ram_data <= (others => 'Z');
    --             ram_addr <= (others => '0');
    --             index := 0;
    --             state <= IDLE;
    --             last_state <= IDLE;
    --         else
    --             ram_addr <= STD_LOGIC_VECTOR(to_unsigned(to_integer(UNSIGNED(addr_reg)) + index, ram_addr_width));
    --             last_state <= state;
    --             case state is
    --                 when IDLE =>
    --                     if(module_active = '1' and last_state = IDLE) then
    --                         if((access_type = INSTRUCTION and UNSIGNED(module_addr) <= UNSIGNED(inst_end))
    --                             or (access_type = DATA and UNSIGNED(module_addr) >= UNSIGNED(data_start))) then
                                

                                
    --                             err_o <= '0';
    --                             ram_ce <= '0';
    --                             byte_select_reg <= sel_i;
    --                             bytes_count := count_ones(sel_i);
    --                             addr_reg <= module_addr;
    --                             ram_addr <= module_addr;
    --                             if(we_i = '1') then
    --                                 ack_o <= '1';
    --                                 data_reg <= muxed_data;
    --                                 ram_oe <= '1';
    --                                 ram_we <= '0';
    --                                 ram_data <= muxed_data(7 downto 0);
    --                                 index := index + 1;
    --                                 if (bytes_count > 1) then
    --                                     state <= WRITING;
    --                                 end if;
    --                             else 
    --                                 ack_o <= '0';
    --                                 ram_oe <= '0';
    --                                 ram_we <= '1';
    --                                 ram_data <= (others => 'Z');
    --                                 state <= READING;
    --                             end if;
    --                         else  
    --                             ack_o <= '0';
    --                             err_o <= '1';
    --                             ram_ce <= '1';
    --                             ram_oe <= '1';
    --                             ram_we <= '1';
    --                             ram_data <= (others => 'Z');
    --                             index := 0;
    --                         end if;
    --                     else
    --                         ack_o <= '0';
    --                         err_o <= '0';
    --                         ram_ce <= '1';
    --                         ram_oe <= '1';
    --                         ram_we <= '1';
    --                         ram_data <= (others => 'Z');
    --                         index := 0;
    --                     end if;
    --                 when WRITING =>
    --                     ack_o <= '0';
    --                     err_o <= '0';
    --                     ram_ce <= '0';
    --                     ram_oe <= '1';
    --                     ram_we <= '0';
    --                     ram_data <= data_reg(index * 8 + 7 downto index * 8);
    --                     index := index + 1;
    --                     if (index = bytes_count) then
    --                         index := 0;
    --                         state <= IDLE;
    --                     end if;
    --                 when READING =>
    --                     ack_o <= '0';
    --                     err_o <= '0';
    --                     ram_ce <= '0';
    --                     ram_oe <= '0';
    --                     ram_we <= '1';
    --                     ram_data <= (others => 'Z');
    --                     data_reg(index * 8 + 7 downto index * 8) <= ram_data;
    --                     index := index + 1;
    --                     if (index = bytes_count) then
    --                         index := 0;
    --                         ack_o <= '1';
    --                         state <= IDLE;
    --                     end if;
    --                 when others =>
    --                     ack_o <= '0';
    --                     err_o <= '0';
    --                     ram_ce <= '1';
    --                     ram_oe <= '1';
    --                     ram_we <= '1';
    --                     ram_data <= (others => 'Z');
    --                     index := 0;
    --                     state <= IDLE;
    --             end case;
    --         end if;
    --     end if;
    -- end process w_access;