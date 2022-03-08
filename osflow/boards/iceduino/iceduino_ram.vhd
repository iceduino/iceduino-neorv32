library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity iceduino_ram is
    generic (
        ram_addr_inst : std_ulogic_vector(31 downto 0);
        ram_width_inst : natural;
        ram_addr_data : std_ulogic_vector(31 downto 0);
        ram_width_data : natural;
        ram_addr_width : natural;
        WRITE_BUFFER_WIDTH : natural
    );
    port (
        clk_i       : in  std_ulogic; -- global clock line
        rstn_i 	    : in  std_ulogic; -- global reset line, low-active
        --wishbone-
        adr_i 	    : in  std_ulogic_vector(31 downto 0); 
        dat_i	    : in  std_ulogic_vector(31 downto 0); --write to slave
        dat_o	    : out std_ulogic_vector(31 downto 0);       
        sel_i       : in std_ulogic_vector(3 downto 0);
        we_i  	    : in  std_ulogic;
        stb_i  	    : in  std_ulogic;
        cyc_i  	    : in  std_ulogic;
        ack_o  	    : out  std_ulogic;
        err_o  	    : out  std_ulogic;
        tag_i       : in std_ulogic_vector(2 downto 0);
        fence_i     : in std_ulogic;
        fencei_i    : in std_ulogic;
        -- ram io --
        ram_data    : inout std_logic_vector(7 downto 0);
        -- ram_data_out : out std_ulogic_vector(7 downto 0);
        -- ram_data_in : in std_ulogic_vector(7 downto 0);
        ram_addr    : out std_ulogic_vector(ram_addr_width-1 downto 0);
        ram_ce      : out std_ulogic;
        ram_oe      : out std_ulogic;
        ram_we      : out std_ulogic
        -- ram_sb_io_oe : out std_ulogic
    );
end entity;

architecture iceduino_ram_rtl of iceduino_ram is

    function count_ones (vector:std_ulogic_vector(3 downto 0))
        return integer is
        variable count : integer;
    begin
        case vector is
            when "0000" => count := 0;
            when "0001" => count := 1;
            when "0010" => count := 1;
            when "0011" => count := 2;
            when "0100" => count := 1;
            when "0101" => count := 2;
            when "0110" => count := 2;
            when "0111" => count := 3;
            when "1000" => count := 1;
            when "1001" => count := 2;
            when "1010" => count := 2;
            when "1011" => count := 3;
            when "1100" => count := 2;
            when "1101" => count := 3;
            when "1110" => count := 3;
            when "1111" => count := 4;
            when others => count := 0;
        end case;
        return count;
    end function;

    constant inst_enabled : boolean := ram_width_inst > 0;
    constant data_enabled : boolean := ram_width_data > 0;
    constant inst_start : std_ulogic_vector(ram_addr_width-1 downto 0) := STD_ULOGIC_VECTOR(to_unsigned(0, ram_addr_width));
    constant data_start : std_ulogic_vector(ram_addr_width-1 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(inst_start) + ram_width_inst);
    constant inst_end : std_ulogic_vector(ram_addr_width-1 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(data_start) - to_unsigned(1, ram_addr_width));
    constant data_end : std_ulogic_vector(ram_addr_width-1 downto 0) := STD_ULOGIC_VECTOR(UNSIGNED(data_start) + ram_width_data - to_unsigned(1, ram_addr_width));
    -- signal reg_dat_i : std_ulogic_vector(31 downto 0) := (others => '0');
    -- signal reg_adr_i : std_ulogic_vector(31 downto 0) := (others => '0');
    -- signal reg_sel_i : std_ulogic_vector(3 downto 0) := (others => '0');
    signal reg_ram_data_in : std_ulogic_vector(31 downto 0) := (others => '0');
    signal module_active : std_ulogic;
    signal module_addr : std_ulogic_vector(ram_addr_width-1 downto 0);
    signal module_addr_inst : std_ulogic_vector(31 downto 0);
    signal module_addr_data : std_ulogic_vector(31 downto 0);
    signal muxed_data : std_ulogic_vector(31 downto 0);
    -- signal access_valid : std_ulogic;
    type t_access is (INSTRUCTION, DATA, INVALID);
    signal access_type : t_access;
    -- type t_state is (IDLE, WRITING1, WRITING1_DONE, WRITING2, WRITING2_DONE, WRITING3, WRITING3_DONE, WRITING4, WRITING4_DONE, READING1, READING2, READING3, READING4, READING_DONE);
    type t_state is (STANDBY, IDLE, WRITING, READING);
    signal state : t_state := STANDBY;
    type t_write_buffer_cell is record
        data : std_ulogic_vector(7 downto 0);
        addr : std_ulogic_vector(ram_addr_width-1 downto 0);
    end record;
    type t_write_buffer is array (0 to WRITE_BUFFER_WIDTH-1) of t_write_buffer_cell;
    signal write_buffer : t_write_buffer;
    signal current_write : t_write_buffer_cell;
    signal current_write_data : std_ulogic_vector(7 downto 0);
    signal current_write_addr : std_ulogic_vector(ram_addr_width-1 downto 0);
    signal current_read_addr : std_ulogic_vector(ram_addr_width-1 downto 0);
    signal write_buffer_index : integer := 0;
    signal ram_out_enable : std_ulogic := '0';
    signal reading_index : integer := 0;
    signal flushing_buffer : std_ulogic := '0';

begin
    module_active <= '1' when access_type /= INVALID and cyc_i = '1' and stb_i = '1' else '0';
    access_type_generate: if inst_enabled and data_enabled generate
        access_type <= INSTRUCTION when UNSIGNED(adr_i) >= UNSIGNED(ram_addr_inst) and UNSIGNED(adr_i) < (UNSIGNED(ram_addr_inst) + ram_width_inst)
            else DATA when UNSIGNED(adr_i) >= UNSIGNED(ram_addr_data) and UNSIGNED(adr_i) < (UNSIGNED(ram_addr_data) + ram_width_data)
            else INVALID;
    elsif inst_enabled generate
        access_type <= INSTRUCTION when UNSIGNED(adr_i) >= UNSIGNED(ram_addr_inst) and UNSIGNED(adr_i) < (UNSIGNED(ram_addr_inst) + ram_width_inst)
            else INVALID;
    elsif data_enabled generate
        access_type <= DATA when UNSIGNED(adr_i) >= UNSIGNED(ram_addr_data) and UNSIGNED(adr_i) < (UNSIGNED(ram_addr_data) + ram_width_data)
            else INVALID;
    else generate
        access_type <= INVALID;
    end generate access_type_generate;
    -- access_valid <= '1' when module_active = '1' and ((access_type = INSTRUCTION and we_i = '0') or access_type = DATA) else '0';
    module_addr_inst <= (STD_LOGIC_VECTOR(UNSIGNED(adr_i) - UNSIGNED(ram_addr_inst)));
    module_addr_data <= (STD_LOGIC_VECTOR(UNSIGNED(adr_i) - UNSIGNED(ram_addr_data) + UNSIGNED(data_start)));
    module_addr <= module_addr_inst(ram_addr_width-1 downto 0) when access_type = INSTRUCTION 
        else module_addr_data(ram_addr_width-1 downto 0) when access_type = DATA
        else (others => '0');

    muxed_data <= x"000000"     & dat_i(7 downto 0)                                 when sel_i = "0001"
        else x"000000"          & dat_i(15 downto 8)                                when sel_i = "0010"
        else x"0000"            & dat_i(15 downto 0)                                when sel_i = "0011"
        else x"000000"          & dat_i(23 downto 16)                               when sel_i = "0100"
        else x"0000"            & dat_i(23 downto 16)   & dat_i(7 downto 0)     when sel_i = "0101"
        else x"0000"            & dat_i(23 downto 8)                                when sel_i = "0110"
        else x"00"              & dat_i(23 downto 0)                                when sel_i = "0111"
        else x"000000"          & dat_i(31 downto 24)                               when sel_i = "1000"
        else x"0000"            & dat_i(31 downto 24)   & dat_i(7 downto 0)     when sel_i = "1001"
        else x"0000"            & dat_i(31 downto 24)   & dat_i(15 downto 8)    when sel_i = "1010"
        else x"00"              & dat_i(31 downto 24)   & dat_i(15 downto 0)    when sel_i = "1011"
        else x"0000"            & dat_i(31 downto 16)                               when sel_i = "1100"
        else x"00"              & dat_i(31 downto 16)   & dat_i(7 downto 0)     when sel_i = "1101"
        else x"00"              & dat_i(31 downto 8)                                when sel_i = "1110"
        else dat_i when sel_i = "1111"
        else (others => '0');
    
    dat_o <= x"000000"                      & reg_ram_data_in(7 downto 0)                                                                   when sel_i = "0001"
        else x"0000"                        & reg_ram_data_in(7 downto 0)   & x"00"                                                         when sel_i = "0010"
        else x"0000"                        & reg_ram_data_in(15 downto 0)                                                                  when sel_i = "0011"
        else x"00"                          & reg_ram_data_in(7 downto 0)   & x"0000"                                                       when sel_i = "0100"
        else x"00"                          & reg_ram_data_in(15 downto 8)  & x"00"                         & reg_ram_data_in(7 downto 0)   when sel_i = "0101"
        else x"00"                          & reg_ram_data_in(15 downto 0)  & x"00"                                                         when sel_i = "0110"
        else x"00"                          & reg_ram_data_in(23 downto 0)                                                                  when sel_i = "0111"
        else reg_ram_data_in(7 downto 0)    & x"000000"                                                                                     when sel_i = "1000"
        else reg_ram_data_in(15 downto 8)   & x"0000"                       & reg_ram_data_in(7 downto 0)                                   when sel_i = "1001"
        else reg_ram_data_in(15 downto 8)   & x"00"                         & reg_ram_data_in(7 downto 0)   & x"00"                         when sel_i = "1010"
        else reg_ram_data_in(23 downto 16)  & x"00"                         & reg_ram_data_in(15 downto 0)                                  when sel_i = "1011"
        else reg_ram_data_in(15 downto 0)   & x"0000"                                                                                       when sel_i = "1100"
        else reg_ram_data_in(23 downto 8)   & x"00"                         & reg_ram_data_in(7 downto 0)                                   when sel_i = "1101"
        else reg_ram_data_in(23 downto 0)   & x"00"                                                                                         when sel_i = "1110"
        else reg_ram_data_in(31 downto 0)                                                                                                   when sel_i = "1111"
        else (others => '0');

    
    state_machine: process(clk_i, rstn_i) is
    begin
        if rstn_i = '0' then
            state <= STANDBY;
            write_buffer_index <= 0;
            reading_index <= 0;
            ram_ce <= '1';
            ram_we <= '1';
            ram_oe <= '1';
            ack_o <= '0';
            err_o <= '0';
            reg_ram_data_in <= (others => '0');
            ram_out_enable <= '0';
            flushing_buffer <= '0';
        elsif rising_edge(clk_i) then
            ram_out_enable <= '0';
            flushing_buffer <= fence_i or fencei_i when flushing_buffer = '0' else flushing_buffer;
            if module_active = '1' and we_i = '1' then
                -- if access_valid = '0' then
                --     err_o <= '1';
                -- else
                    if WRITE_BUFFER_WIDTH >= write_buffer_index + 4 and WRITE_BUFFER_WIDTH > 4 then
                        for i in 0 to 3 loop
                            write_buffer(write_buffer_index + i + 1).data <= muxed_data(i*8 + 7 downto i*8);
                            write_buffer(write_buffer_index + i + 1).addr <= STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + i);
                        end loop;
                        write_buffer_index <= write_buffer_index + count_ones(sel_i);
                        ack_o <= '1';
                    end if;
                -- end if;
            end if;
            case state is
            when STANDBY =>
                write_buffer_index <= 0;
                reading_index <= 0;
                ram_ce <= '1';
                ram_we <= '1';
                ram_oe <= '1';
                ack_o <= '0';
                err_o <= '0';
                reg_ram_data_in <= (others => '0');
                ram_out_enable <= '0';
                if module_active = '1' or flushing_buffer = '1' then
                    -- if access_valid = '1' then
                        state <= IDLE;
                        ram_ce <= '0';
                        ram_oe <= '0';
                    -- else
                    --     err_o <= '1';
                    -- end if;
                end if;
            when IDLE =>
                ack_o <= '0';
                err_o <= '0';
                ram_ce <= '0';
                ram_oe <= '0';
                ram_we <= '1';
                if flushing_buffer = '0' and fence_i = '0' and fencei_i = '0' then
                    if module_active = '1' then
                        -- if access_valid = '0' then
                        --     err_o <= '1';
                        -- else
                            if we_i = '0' then
                                state <= READING;
                                reading_index <= 0;
                            else
                                for i in 0 to WRITE_BUFFER_WIDTH - 2 loop
                                    write_buffer(i) <= write_buffer(i + 1);
                                end loop;
                                if WRITE_BUFFER_WIDTH >= write_buffer_index + 4 then
                                    write_buffer_index <= write_buffer_index + count_ones(sel_i) - 1;
                                    for i in 0 to 3 loop
                                        write_buffer(write_buffer_index + i).data <= muxed_data(i*8 + 7 downto i*8);
                                        write_buffer(write_buffer_index + i).addr <= STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + i);
                                    end loop;
                                    ack_o <= '1';
                                else write_buffer_index <= write_buffer_index - 1;
                                end if;
                                state <= WRITING;
                                ram_we <= '0';
                                -- ram_oe <= '1';
                                ram_out_enable <= '1';
                            end if;
                        -- end if;
                    else
                        if write_buffer_index > 0 then
                            write_buffer_index <= write_buffer_index - 1;
                            for i in 0 to WRITE_BUFFER_WIDTH - 2 loop
                                write_buffer(i) <= write_buffer(i + 1);
                            end loop;
                            state <= WRITING;
                            ram_we <= '0';
                            -- ram_oe <= '1';
                            ram_out_enable <= '1';
                        else
                            state <= STANDBY;
                        end if;
                    end if;
                else
                    if write_buffer_index > 0 then
                        write_buffer_index <= write_buffer_index - 1;
                        for i in 0 to WRITE_BUFFER_WIDTH - 2 loop
                            write_buffer(i) <= write_buffer(i + 1);
                        end loop;
                        state <= WRITING;
                        ram_we <= '0';
                        -- ram_oe <= '1';
                        ram_out_enable <= '1';
                    else
                        flushing_buffer <= '0';
                    end if;
                end if;
            when READING =>
                reg_ram_data_in(reading_index*8 + 7 downto reading_index*8) <= ram_data;
                reading_index <= (reading_index + 1) mod 4;
                state <= IDLE when module_active = '0' or ack_o = '1';
                ack_o <= '1' when reading_index = count_ones(sel_i) - 1 and module_active = '1' else '0';
            when WRITING =>
                state <= IDLE;
                ram_out_enable <= '1';
                ram_we <= '1';
                -- ram_oe <= '1';
                ack_o <= '0';
                err_o <= '0';
                reg_ram_data_in <= (others => '0');
            when others =>
                state <= STANDBY;
                write_buffer_index <= 0;
                reading_index <= 0;
                ram_ce <= '1';
                ram_we <= '1';
                ram_oe <= '1';
                ack_o <= '0';
                err_o <= '0';
                reg_ram_data_in <= (others => '0');
                ram_out_enable <= '0';
                flushing_buffer <= '0';
            end case;
        end if;
    end process;
    
    --     input_register_process: process(clk_i, rstn_i)
    -- begin
    --     if rstn_i = '0' then
    --         reg_adr_i <= (others => '0');
    --         reg_dat_i <= (others => '0');
    --         reg_sel_i <= (others => '0');
    --         reg_ram_data_in <= (others => '0');
    --     elsif rising_edge(clk_i) then
    --         case state is
    --             when IDLE =>
    --                 reg_adr_i <= adr_i;
    --                 reg_dat_i <= dat_i;
    --                 reg_sel_i <= sel_i;
    --                 reg_ram_data_in <= (others => '0');
    --             when READING1 =>
    --                 reg_adr_i <= reg_adr_i;
    --                 reg_dat_i <= reg_dat_i;
    --                 reg_sel_i <= reg_sel_i;
    --                 reg_ram_data_in(7 downto 0) <= ram_data;
    --             when READING2 =>
    --                 reg_adr_i <= reg_adr_i;
    --                 reg_dat_i <= reg_dat_i;
    --                 reg_sel_i <= reg_sel_i;
    --                 reg_ram_data_in(15 downto 8) <= ram_data;
    --             when READING3 =>
    --                 reg_adr_i <= reg_adr_i;
    --                 reg_dat_i <= reg_dat_i;
    --                 reg_sel_i <= reg_sel_i;
    --                 reg_ram_data_in(23 downto 16) <= ram_data;
    --             when READING4 =>
    --                 reg_adr_i <= reg_adr_i;
    --                 reg_dat_i <= reg_dat_i;
    --                 reg_sel_i <= reg_sel_i;
    --                 reg_ram_data_in(31 downto 24) <= ram_data;
    --             when READING_DONE => NULL;
    --             when others =>
    --                 reg_adr_i <= reg_adr_i;
    --                 reg_dat_i <= reg_dat_i;
    --                 reg_sel_i <= reg_sel_i;
    --                 reg_ram_data_in <= (others => '0');
    --         end case;
    --     end if;
    -- end process; -- input register process

    -- state_machine: process(clk_i, rstn_i)
    -- begin
    --     if rstn_i = '0' then
    --         state <= IDLE;

    --     elsif rising_edge(clk_i) then
    --         case state is
    --             when IDLE =>
    --                 if(module_active = '1' and access_valid = '1') then
    --                     if we_i = '1' then
    --                         state <= WRITING1;
    --                     else
    --                         state <= READING1;
    --                     end if;
    --                 end if;
    --             when WRITING1 =>
    --                 state <= WRITING1_DONE;
    --             when WRITING1_DONE =>
    --                 if count_ones(reg_sel_i) = 1 then
    --                     state <= IDLE;
    --                 else
    --                     state <= WRITING2;
    --                 end if;
    --             when WRITING2 =>
    --                 state <= WRITING2_DONE;
    --             when WRITING2_DONE =>
    --                 if count_ones(reg_sel_i) = 2 then
    --                     state <= IDLE;
    --                 else
    --                     state <= WRITING3;
    --                 end if;
    --             when WRITING3 =>
    --                 state <= WRITING3_DONE;
    --             when WRITING3_DONE =>
    --                 if count_ones(reg_sel_i) = 3 then
    --                     state <= IDLE;
    --                 else
    --                     state <= WRITING4;
    --                 end if;
    --             when WRITING4 =>
    --                 state <= WRITING4_DONE;
    --             when WRITING4_DONE =>
    --                 state <= IDLE;
    --             when READING1 =>
    --                 state <= READING_DONE when count_ones(reg_sel_i) = 1 else READING2;
    --             when READING2 =>
    --                 state <= READING_DONE when count_ones(reg_sel_i) = 2 else READING3;
    --             when READING3 =>
    --                 state <= READING_DONE when count_ones(reg_sel_i) = 3 else READING4;
    --             when READING4 =>
    --                 state <= READING_DONE;
    --             when READING_DONE =>
    --                 state <= IDLE;
    --             when others => state <= IDLE;
    --         end case;
    --     end if;
    -- end process; -- state machine process

    -- err_o <= '1' when module_active = '1' and access_valid = '0' else '0';
    -- ack_o <= '1' when (module_active = '1' and access_valid = '1' and we_i = '1') or state = READING_DONE else '0';

    current_write           <= write_buffer(0);
    current_write_data      <= current_write.data;
    current_write_addr      <= current_write.addr;
    current_read_addr       <= STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + reading_index);

    ram_data                <= current_write_data when ram_out_enable = '1' else (others => 'Z');
    ram_addr                <= current_write_addr when ram_out_enable = '1' else current_read_addr;

    -- ram_ce <= '0' when state /= IDLE else '1';
    -- ram_oe <= '0' when state = READING1 or state = READING2 or state = READING3 or state = READING4 else '1';
    -- ram_we <= '0' when state = WRITING1 or state = WRITING2 or state = WRITING3 or state = WRITING4 else '1';
    -- ram_sb_io_oe <= '1' when state = WRITING1 or state = WRITING2 or state = WRITING3 or state = WRITING4 or state = WRITING1_DONE or state = WRITING2_DONE or state = WRITING3_DONE or state = WRITING4_DONE else '0';

    -- ram_data <= muxed_data(7 downto 0) when state = WRITING1 or state = WRITING1_DONE else
    --     muxed_data(15 downto 8) when state = WRITING2 or state = WRITING2_DONE else
    --     muxed_data(23 downto 16) when state = WRITING3 or state = WRITING3_DONE else
    --     muxed_data(31 downto 24) when state = WRITING4 or state = WRITING4_DONE else
    --     (others => 'Z');
    
    -- ram_addr <= module_addr when state = READING1 or state = WRITING1 or state = WRITING1_DONE else
    --     STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + 1) when state = READING2 or state = WRITING2 or state = WRITING2_DONE else
    --     STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + 2) when state = READING3 or state = WRITING3 or state = WRITING3_DONE else
    --     STD_ULOGIC_VECTOR(UNSIGNED(module_addr) + 3) when state = READING4 or state = WRITING4 or state = WRITING4_DONE else
    --     (others => '0');
        
end architecture ;
