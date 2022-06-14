-- Cache controller

--Todo: 1   -- Make fetches start early (internal and external) external by setting AR in the read controller one clock earlier
            -- Wide word width for data
            -- Look for further improvements


library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;
use work.cpu_pack.all;

entity cache_cntrl is
    generic (
        -- cpu params
        cpu_addr_width       : integer := 32;
        cpu_data_width       : integer := 32;
        -- cache params       
        cache_way_width_instruction      : integer := 0;            -- # blocks per cache line; associativity = 2^cache_way_width
        cache_index_width_instruction    : integer := 9;            -- # of cache lines = 2^cache_index_width
        cache_offset_width_instruction   : integer := 2;            -- # of bytes per block = 2^cache_offset_width 
        cache_address_width_instruction  : integer := 25;           -- address width for cacheable range

        cache_way_width_data      : integer := 0;            -- # blocks per cache line; associativity = 2^cache_way_width
        cache_index_width_data    : integer := 9;            -- # of cache lines = 2^cache_index_width
        cache_offset_width_data   : integer := 2;            -- # of bytes per block = 2^cache_offset_width
        cache_address_width_data  : integer := 25;           -- address width for cacheable range

        cache_replace_policy      : string := "RR"           -- replacement policy when cache miss: "RR"
    );               
    port ( 
        aclk                 : in  std_logic;
        aresetn              : in  std_logic;
        cpu_next_address     : in  std_logic_vector(cpu_addr_width-1 downto 0);
        instruction_fetch    : in  std_logic;
        cpu_wr_data          : in  std_logic_vector(cpu_data_width-1 downto 0);
        cpu_wr_byte_en       : in  std_logic_vector(cpu_data_width/8-1 downto 0);
        cpu_rd_data          : out std_logic_vector(cpu_data_width-1 downto 0) := (others=>'0');
        cpu_pause            : out std_logic;
        mem_wr_en            : out std_logic;
        mem_wr_addr          : out std_logic_vector(cpu_addr_width-1 downto 0) := (others=>'0');
        mem_wr_data          : out std_logic_vector(cpu_data_width-1 downto 0) := (others=>'0');
        mem_wr_byte_en       : out std_logic_vector(cpu_data_width/8-1 downto 0) := (others=>'0');
        mem_wr_ready         : in  std_logic;
        mem_wr_valid         : out std_logic;
        mem_rd_en            : out std_logic;
        mem_rd_addr          : out std_logic_vector(cpu_addr_width-1 downto 0) := (others=>'0');
        mem_rd_data          : in  std_logic_vector(cpu_data_width-1 downto 0);
        mem_rd_ready         : out std_logic;
        mem_rd_valid         : in  std_logic;
        mem_cache_line       : out std_logic;
        stream_buffer_burst  : out std_logic;
        stream_buffer_burst_length : out std_logic_vector(7 downto 0);
        burst_last                 : in std_logic;
        bypass_fifo                : out std_logic;
        early_araddr                : out std_logic_vector(cpu_addr_width-1 downto 0) := (others=>'0');
        early_arvalid               : out std_logic;
        early_burst                 : out std_logic_vector(7 downto 0)
    ); 
end cache_cntrl;

architecture Behavioral of cache_cntrl is

    --Constants
    constant stream_buffer_depth             : integer := 12;
    constant tag_width_instruction           : integer := cache_address_width_instruction-cache_index_width_instruction-cache_offset_width_instruction;
    constant block_word_width_instruction    : integer := cache_offset_width_instruction-clogb2(cpu_data_width/8);   
    constant tag_width_data                  : integer := cache_address_width_data-cache_index_width_data-cache_offset_width_data;
    constant block_word_width_data           : integer := cache_offset_width_data-clogb2(cpu_data_width/8);  
    constant lfsr_width                      : integer := 16;   

    --Type define: Debug
    type debug_type_main_controller         is (NONE, REFILL, MEM_INSTRUCTION, MEM_DATA, NON_CACHE, HIT_INSTRUCTION, HIT_STREAM, HIT_DATA, MISS_INSTRUCTION, MISS_DATA, WAITING_FOR_FILL);
    signal debug_signal : debug_type_main_controller;

    --Type definition of BRAM elements for instruction cache
    type block_rows_type_instruction         is array(0 to 2**cache_index_width_instruction-1) of std_logic_vector(2**cache_way_width_instruction*2**cache_offset_width_instruction*8-1 downto 0);
    type tag_rows_type_instruction           is array(0 to 2**cache_index_width_instruction-1) of std_logic_vector(2**cache_way_width_instruction*tag_width_instruction-1 downto 0);
    type valid_rows_type_instruction         is array(0 to 2**cache_index_width_instruction-1) of std_logic_vector(2**cache_way_width_instruction-1 downto 0);

    --Type definition of BRAM elements for data cache
    type block_rows_type_data         is array(0 to 2**cache_index_width_data-1) of std_logic_vector(2**cache_way_width_data*2**cache_offset_width_data*8-1 downto 0);
    type tag_rows_type_data           is array(0 to 2**cache_index_width_data-1) of std_logic_vector(2**cache_way_width_data*tag_width_data-1 downto 0);
    type valid_rows_type_data         is array(0 to 2**cache_index_width_data-1) of std_logic_vector(2**cache_way_width_data-1 downto 0);
    type dirty_rows_type_data         is array(0 to 2**cache_index_width_data-1) of std_logic_vector(2**cache_way_width_data-1 downto 0);

    --Type definition for the stream_buffer
    type tag_stream_buffer_type           is array(0 to stream_buffer_depth-1) of std_logic_vector(tag_width_instruction+cache_index_width_instruction-1 downto 0);
    type available_stream_buffer_type     is array(0 to stream_buffer_depth-1) of std_logic;
    type block_stream_buffer_type         is array(0 to stream_buffer_depth-1) of std_logic_vector(2**cache_offset_width_instruction*8-1 downto 0);

    --Defining memory access types
    type memory_access_mode_type is (READ_BLOCK,WRITE_BLOCK,EXCHANGE_BLOCK,WRITE_WORD,READ_WORD);

    type debug_type_stream_type             is (NONE, REFILL, FILL);
    signal debug_stream_buffer              : debug_type_stream_type;

    ---BRAM elements for instruction cache
    signal block_rows_instruction            : block_rows_type_instruction := (others=>(others=>'0'));
    signal tag_rows_instruction              : tag_rows_type_instruction := (others=>(others=>'0'));
    signal valid_rows_instruction            : valid_rows_type_instruction := (others=>(others=>'0'));

    --BRAM elements for data cache
    signal block_rows_data                   : block_rows_type_data := (others=>(others=>'0'));
    signal tag_rows_data                     : tag_rows_type_data := (others=>(others=>'0'));
    signal valid_rows_data                   : valid_rows_type_data := (others=>(others=>'0'));
    signal dirty_rows_data                   : dirty_rows_type_data := (others=>(others=>'0')); --helpful to reduce the frequency of exchanges, Todo: benefit from this by using wide cache lines with bursts for read blocks

    --Stream buffer elements
    signal tag_stream_buffer                : tag_stream_buffer_type := (others=>(others=>'0'));
    signal available_stream_buffer          : available_stream_buffer_type := (others=>'0');
    signal block_stream_buffer              : block_stream_buffer_type := (others=>(others=>'0'));

    --Stream buffer interface
    signal shift_stream_buffer              : std_logic;
    signal head_comparator_stream_buffer1   : std_logic;
    signal head_comparator_stream_buffer2   : std_logic;
    signal busy_stream_buffer               : Boolean :=False;         
    signal refill_stream_buffer             : std_logic;  
    signal refill_fetch_address             : std_logic_vector(cache_address_width_instruction-1 downto cache_offset_width_instruction);
    signal refill_counter                   : integer range 0 to stream_buffer_depth-1:= 0;
    signal fill_counter                     : integer range 0 to stream_buffer_depth-1:= 0;
    signal ignore_counter                   : integer range 0 to stream_buffer_depth-1:= 0;
    signal final_counter_value              : integer range 0 to stream_buffer_depth-1:= 0;
    signal mem_rd_en_stream_buff            : std_logic := '0';
    signal mem_rd_ready_stream_buff         : std_logic := '0';    
    signal mem_rd_addr_stream_buff          : std_logic_vector(cache_address_width_instruction-1 downto 0);  
    signal initial_cycle_refill             : Boolean := True;
    signal initial_cycle_fill               : Boolean := True;
    signal refilling_stream_buffer          : Boolean := False;
    signal filling_stream_buffer            : Boolean := False;
    signal just_started_refill              : Boolean :=True;
    signal depth_available                  : integer range 0 to stream_buffer_depth;     
    signal refill_queued                    : Boolean := False;
    signal burst_size_buff                  : integer range 0 to 2**stream_buffer_burst_length'length - 1;
    signal axi_finished_read_stream         : Boolean;


    -- Tag - Index - Offset for an instruction access
    signal cpu_tag_instruction               : std_logic_vector(tag_width_instruction-1 downto 0) := (others=>'0');
    signal cpu_index_instruction             : integer range 0 to 2**cache_index_width_instruction-1 := 0;
    signal cpu_offset_instruction            : integer range 0 to 2**cache_offset_width_instruction-1 := 0;

    -- Tag - Index - Offset for a data access
    signal cpu_tag_data               : std_logic_vector(tag_width_data-1 downto 0) := (others=>'0');
    signal cpu_index_data             : integer range 0 to 2**cache_index_width_data-1 := 0;
    signal cpu_offset_data            : integer range 0 to 2**cache_offset_width_data-1 := 0;

    -- Hit signals for an instruction access
    signal cpu_way_instruction               : integer range 0 to 2**cache_way_width_instruction-1 := 0;
    signal cache_hit_instruction             : Boolean := False;

    -- Hit signals for a data access
    signal cpu_way_data               : integer range 0 to 2**cache_way_width_data-1 := 0;
    signal cache_hit_data             : Boolean := False;


    --Flag whether the access is cacheable or not
    signal cacheable_range_instruction       : Boolean := False;
    signal cacheable_range_data              : Boolean := False;

    --Buffer the the pause signal
    signal cpu_pause_buff        : std_logic := '0';

    --LFSR signals 
    signal replace_way_instruction                  : integer range 0 to 2**cache_way_width_instruction-1 := 0;
    signal replace_way_data                         : integer range 0 to 2**cache_way_width_data-1 := 0;
    --Storing the write enables and data
    signal replace_write_enables                    : std_logic_vector(cpu_data_width/8-1 downto 0) := (others=>'0');
    signal replace_write_data                       : std_logic_vector(cpu_data_width-1 downto 0) := (others=>'0');
    --Storing the offset coming from the CPU address
    signal replace_offset_instruction               : integer range 0 to 2**cache_offset_width_instruction-1 := 0;
    signal replace_offset_data                      : integer range 0 to 2**cache_offset_width_data-1 := 0;
    signal flush_stream_buffer_address              : std_logic_vector(cache_address_width_instruction-1 downto cache_offset_width_instruction);


    --Buffering the LFSR signals
    signal mem_way_instruction   : integer range 0 to 2**cache_way_width_instruction-1 := 0;
    signal mem_way_data          : integer range 0 to 2**cache_way_width_data-1 := 0;

    --Flag whether we need a memory access
    signal mem_access_needed     : Boolean := False;

    --Flag whether we are waiting for the stream buffer
    signal stream_buffer_pending : Boolean := False;

    --Buffering whether the access in an instruction or data access
    signal instruction_access    : Boolean := False; 

    --Memory access type out of the 4 defined types
    signal mem_access_mode       : memory_access_mode_type := READ_BLOCK;

    --Counters for tracking the number of accesses done: more relevant if the cache line is wide
    signal memory_wr_count_instruction       : integer range 0 to 2**block_word_width_instruction-1 := 0;
    signal memory_rd_count_instruction       : integer range 0 to 2**block_word_width_instruction-1 := 0;
    signal memory_wr_count_data              : integer range 0 to 2**block_word_width_data-1        := 0;
    signal memory_rd_count_data              : integer range 0 to 2**block_word_width_data-1        := 0;

    --Buffers for the AXI handshake
    signal mem_wr_en_buff        : std_logic := '0';
    signal mem_rd_en_buff        : std_logic := '0';
    signal mem_wr_valid_buff     : std_logic := '0';
    signal mem_rd_ready_buff     : std_logic := '0';
    signal mem_rd_addr_buff      : std_logic_vector(cpu_addr_width-1 downto 0);

    --Flags for finishing an AXI operation
    signal axi_finished_read     : Boolean := False;
    signal axi_finished_write    : Boolean := False; 

    --Saving the index of the access
    --Todo: maybe just one signal is needed, with the range being the widest of the two
    signal mem_index_instruction             : integer range 0 to 2**cache_index_width_instruction-1 := 0;
    signal mem_index_data                    : integer range 0 to 2**cache_index_width_data-1 := 0;

    signal interesting_pause                : std_logic;
    signal non_cacheable_range_general      : Boolean;
    signal miss_general                     : Boolean;
    signal hit_stream_buffer               : Boolean;

    signal bypass_fifo_stream, bypass_fifo_cache        : std_logic;
    signal early_araddr_cache                           : std_logic_vector(cpu_addr_width-1 downto 0) := (others=>'0');
    signal early_arvalid_stream, early_arvalid_cache    : std_logic;
    signal early_araddr_stream                          : std_logic_vector(cache_address_width_instruction-1 downto 0) := (others=>'0'); 
    signal early_burst_stream, early_burst_cache        : std_logic_vector(7 downto 0);
    --LFSR
    signal lfsr                                         : std_logic_vector(lfsr_width-1 downto 0) := X"ace1"; 


begin


    process(early_araddr_stream, early_araddr_cache, early_arvalid_stream, early_arvalid_cache)
    begin
        if early_arvalid_stream = '1' then
            early_arvalid <= '1';
            early_burst  <= std_logic_vector(to_unsigned(burst_size_buff, early_burst'length));
            early_araddr(cpu_addr_width-1 downto cpu_addr_width-4)                <= "0001";
            early_araddr(cpu_addr_width-5 downto cache_address_width_instruction) <= (others => '0');
            early_araddr(cache_address_width_instruction -1 downto 0)             <= early_araddr_stream;
        elsif early_arvalid_cache = '1' then
            early_burst     <= early_burst_cache;
            early_arvalid   <= '1';
            early_araddr    <= early_araddr_cache;
        else
            early_burst     <= (others => '0');
            early_arvalid   <= '0';
            early_araddr    <= (others => '0');
        end if;
    end process;

    bypass_fifo <= bypass_fifo_cache or bypass_fifo_stream;

    non_cacheable_range_general <= not ((cacheable_range_instruction and instruction_fetch = '1') or (cacheable_range_data and instruction_fetch = '0'));
    miss_general                <= (not cache_hit_data and instruction_fetch = '0') or (not cache_hit_instruction and instruction_fetch = '1');
    hit_stream_buffer  <= ((head_comparator_stream_buffer1 = '1' and shift_stream_buffer = '0') or (head_comparator_stream_buffer2 = '1' and shift_stream_buffer = '1')) and instruction_fetch = '1';

    --CPU Pause
    cpu_pause       <= cpu_pause_buff;

    interesting_pause <= '1' when debug_signal = WAITING_FOR_FILL else '0';

    --Tag Index Offset instruction 
    cpu_tag_instruction         <= cpu_next_address(cache_address_width_instruction-1 downto cache_offset_width_instruction+cache_index_width_instruction);
    cpu_index_instruction       <= to_integer(unsigned(cpu_next_address(cache_offset_width_instruction+cache_index_width_instruction-1 downto cache_offset_width_instruction)));
    cpu_offset_instruction      <= to_integer(unsigned(cpu_next_address(cache_offset_width_instruction-1 downto 0)));

    --Tag Index Offset data
    cpu_tag_data         <= cpu_next_address(cache_address_width_data-1 downto cache_offset_width_data+cache_index_width_data);
    cpu_index_data       <= to_integer(unsigned(cpu_next_address(cache_offset_width_data+cache_index_width_data-1 downto cache_offset_width_data)));
    cpu_offset_data      <= to_integer(unsigned(cpu_next_address(cache_offset_width_data-1 downto 0)));

    --Boolean whether the address is cachable
    cacheable_range_instruction <= True when (cpu_next_address(cpu_addr_width-1 downto cpu_addr_width-4) = "0001" and or_reduce(cpu_next_address(cpu_addr_width-5 downto cache_address_width_instruction))='0') else False;
    cacheable_range_data        <= True when (cpu_next_address(cpu_addr_width-1 downto cpu_addr_width-4) = "0001" and or_reduce(cpu_next_address(cpu_addr_width-5 downto cache_address_width_data))       ='0') else False;
    

    stream_buffer_burst_length <= std_logic_vector(to_unsigned(burst_size_buff, stream_buffer_burst_length'length));

    --AXI Interface reading from buffers

    --Demux to chose between reading via the stream buffer or the main cache control process
    process(busy_stream_buffer, mem_rd_ready_buff, mem_rd_en_buff, mem_rd_ready_stream_buff, mem_rd_en_stream_buff, mem_rd_addr_buff, mem_rd_addr_stream_buff, mem_wr_en_buff, mem_wr_valid_buff)
    begin
        if not busy_stream_buffer then
            mem_rd_ready    <= mem_rd_ready_buff;
            mem_rd_en       <= mem_rd_en_buff;
            mem_rd_addr     <= mem_rd_addr_buff;

            mem_wr_en       <= mem_wr_en_buff;
            mem_wr_valid    <= mem_wr_valid_buff;
        else
            mem_rd_ready    <= mem_rd_ready_stream_buff;
            mem_rd_en       <= mem_rd_en_stream_buff;

            mem_rd_addr(cpu_addr_width-1 downto cpu_addr_width-4)                <= "0001";
            mem_rd_addr(cpu_addr_width-5 downto cache_address_width_instruction) <= (others => '0');
            mem_rd_addr(cache_address_width_instruction-1 downto 0)                <= mem_rd_addr_stream_buff;

            mem_wr_en       <= '0';  --Todo: make sure writes can go in parallel with the stream buffer
            mem_wr_valid    <= '0';
        end if;            
    end process;


    --Combinatorial process to determine whether instruction access is hit or not
    process(cpu_index_instruction, cpu_tag_instruction, tag_rows_instruction, valid_rows_instruction)
        variable cpu_hit_buff : Boolean;
        variable cpu_way_buff : integer range 0 to 2**cache_way_width_instruction-1 := 0;
        variable tag_buff     : std_logic_vector(tag_width_instruction-1 downto 0);
    begin
        cpu_hit_buff := False;
        cpu_way_buff := 0;
        for each_way in 0 to 2**cache_way_width_instruction-1 loop
            tag_buff := tag_rows_instruction(cpu_index_instruction)((each_way+1)*tag_width_instruction-1 downto each_way*tag_width_instruction);
            if tag_buff=cpu_tag_instruction and valid_rows_instruction(cpu_index_instruction)(each_way)='1' then
                cpu_hit_buff := True;
                cpu_way_buff := each_way;
                exit;
            end if;
        end loop;
        cache_hit_instruction <= cpu_hit_buff;
        cpu_way_instruction   <= cpu_way_buff;
    end process; 

    --Combinatorial process to determine whether data access is hit or not
    process(cpu_index_data, cpu_tag_data, tag_rows_data, valid_rows_data)
        variable cpu_hit_buff : Boolean;
        variable cpu_way_buff : integer range 0 to 2**cache_way_width_data-1 := 0;
        variable tag_buff     : std_logic_vector(tag_width_data-1 downto 0);
    begin
        cpu_hit_buff := False;
        cpu_way_buff := 0;
        for each_way in 0 to 2**cache_way_width_data-1 loop
            tag_buff := tag_rows_data(cpu_index_data)((each_way+1)*tag_width_data-1 downto each_way*tag_width_data);
            if tag_buff=cpu_tag_data and valid_rows_data(cpu_index_data)(each_way)='1' then
                cpu_hit_buff := True;
                cpu_way_buff := each_way;
                exit;
            end if;
        end loop;
        cache_hit_data <= cpu_hit_buff;
        cpu_way_data   <= cpu_way_buff;
    end process; 
    
    --LFSR to determine which way to replace into for instruction cache
    generate_replace_policy_RR:
    if cache_replace_policy= "RR" generate
        process (aclk)
            variable lfsr_buff_0 : unsigned(lfsr_width-1 downto 0);
            variable lfsr_buff_1 : unsigned(lfsr_width-1 downto 0);
            variable lfsr_buff_2 : unsigned(lfsr_width-1 downto 0);
        begin
            if rising_edge(aclk) then
                lfsr_buff_0 := unsigned(lfsr);
                lfsr_buff_1 := ((lfsr_buff_0 srl 0) xor (lfsr_buff_0 srl 2) xor (lfsr_buff_0 srl 3) xor (lfsr_buff_0 srl 5)) and to_unsigned(1,lfsr_width);
                lfsr_buff_2 := (lfsr_buff_0 srl 1) or (lfsr_buff_1 sll 15);
                lfsr <= std_logic_vector(lfsr_buff_2);
            end if;
        end process;
        process (lfsr)
        begin
            if cache_way_width_instruction/=0 then
                replace_way_instruction <= to_integer(unsigned(lfsr(cache_way_width_instruction-1 downto 0)));
            else
                replace_way_instruction <= 0;
            end if; 

            if cache_way_width_data/=0 then
                replace_way_data <= to_integer(unsigned(lfsr(cache_way_width_data-1 downto 0)));
            else
                replace_way_data <= 0;
            end if; 
        end process;
    end generate
    generate_replace_policy_RR;



    ------------------------------------------
    ---Stream Buffer
    ------------------------------------------


    --Read head and head comparator
    process(cpu_next_address, tag_stream_buffer, available_stream_buffer)
    begin

        if cpu_next_address(cache_address_width_instruction-1 downto cache_offset_width_instruction) = tag_stream_buffer(0) and available_stream_buffer(0) = '1' then
            head_comparator_stream_buffer1 <= '1';
        else
            head_comparator_stream_buffer1 <= '0';
        end if;

        if cpu_next_address(cache_address_width_instruction-1 downto cache_offset_width_instruction) = tag_stream_buffer(1) and available_stream_buffer(1) = '1' then
            head_comparator_stream_buffer2 <= '1';
        else
            head_comparator_stream_buffer2 <= '0';
        end if;
    end process;


    --Sequential process for control
    process(aclk) --Todo: optiimize by removing the pushes and doing it directly
        variable mem_rd_handshake_stream :Boolean := False;
    begin
        if rising_edge(aclk) then

            early_araddr_stream     <= (others => '0');
            early_arvalid_stream    <= '0';
            early_burst_stream      <= (others => '0');
            bypass_fifo_stream      <= '0';

            if aresetn = '0' then
                refill_fetch_address            <= (others => '0');
                mem_rd_addr_stream_buff         <= (others => '0');
                available_stream_buffer         <= (others => '0');
                depth_available                 <= 0;
                stream_buffer_burst             <= '0';
                mem_rd_en_stream_buff           <= '0';
                ignore_counter                  <= 0;
                fill_counter                    <= 0;



            elsif (refill_stream_buffer = '1' or refilling_stream_buffer or refill_queued) and not filling_stream_buffer then    --Refill the stream buffer

                bypass_fifo_stream <= '1';

                if (initial_cycle_refill) then        --Initial values that are transfered from the main control process
                    busy_stream_buffer      <=  True;   --the stream buffer is busy
                    available_stream_buffer <= (others => '0');
                    depth_available  <= 0;

                    refilling_stream_buffer <= True;
                    refill_queued           <= False;
                    refill_fetch_address    <= flush_stream_buffer_address; --The initial address for refill (where the jump happened)

                    mem_rd_addr_stream_buff(cache_address_width_instruction-1 downto cache_offset_width_instruction)    <= flush_stream_buffer_address; --Read address
                    mem_rd_addr_stream_buff(cache_offset_width_instruction-1 downto 0)                                  <= (others=>'0');

                    early_araddr_stream(cache_address_width_instruction-1 downto cache_offset_width_instruction)    <= flush_stream_buffer_address; --Read address
                    early_araddr_stream(cache_offset_width_instruction-1 downto 0)                                  <= (others=>'0');  

                    early_arvalid_stream <= '1';                  

                    tag_stream_buffer(0)     <= flush_stream_buffer_address;    --Tag of the head: address where the jump happened
                 
                    initial_cycle_refill       <= False;

                    mem_rd_en_stream_buff       <= '1';
                    stream_buffer_burst         <= '1';
                    
                    mem_rd_addr_stream_buff(cache_address_width_instruction-1 downto cache_offset_width_instruction)    <= flush_stream_buffer_address; 
                    mem_rd_addr_stream_buff(cache_offset_width_instruction-1 downto 0)                                  <= (others=>'0');

                    case flush_stream_buffer_address(cache_offset_width_instruction+2 downto cache_offset_width_instruction) is
                        when "000"  => burst_size_buff      <= 7;
                                       final_counter_value  <= 7;
                                       

                        when "001"  => burst_size_buff     <= 7;
                                       ignore_counter      <= 1;
                                       final_counter_value <= 6;
                                       mem_rd_addr_stream_buff(cache_offset_width_instruction) <= '0';

                        when "010"  => burst_size_buff     <= 1;
                                       final_counter_value <= 1;

                        when "011"  => burst_size_buff      <= 0;
                                       final_counter_value  <= 0;
                                       

                        when "100"  => burst_size_buff     <= 3;
                                       final_counter_value <= 3;

                        when "101"  => burst_size_buff      <= 3;
                                       ignore_counter       <= 1;
                                       final_counter_value  <= 2;
                                       mem_rd_addr_stream_buff(cache_offset_width_instruction) <= '0';

                        when "110"  => burst_size_buff      <= 1;
                                       final_counter_value  <= 1;

                        when others => burst_size_buff      <= 0;
                                       final_counter_value  <= 0;

                    end case;
                end if;

                -- 11 1111 1100 + 3 = 11 111 1111
                --Allowed: 

                mem_rd_handshake_stream   := mem_rd_valid='1' and mem_rd_ready_stream_buff='1'; --Handshake
                
                if mem_rd_handshake_stream then

                    if ignore_counter = 0 then

                        block_stream_buffer(refill_counter)     <= mem_rd_data;
                        available_stream_buffer(refill_counter) <= '1';
                        depth_available                         <= depth_available + 1;

                        if refill_counter = final_counter_value then
                            mem_rd_en_stream_buff              <= '0';
                            stream_buffer_burst                <= '0';    
                            mem_rd_ready_stream_buff           <= '0';      

                            busy_stream_buffer              <= False;
                            refilling_stream_buffer         <= False;

                            --Reset counter
                            refill_counter                  <= 0;  

                            --Next cycle the stream buffer is gonna be active is going to be the initial cycle where initializations happen
                            initial_cycle_refill           <= TrUe;
            
                        else
                            mem_rd_ready_stream_buff <= '1';
                            tag_stream_buffer(refill_counter+1) <= std_logic_vector(unsigned(refill_fetch_address) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction)); 
                            refill_counter  <= refill_counter+1;
                            refill_fetch_address    <= std_logic_vector(unsigned(refill_fetch_address) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction));    --Increment the fetch address 
                        end if;
                    else
                        ignore_counter           <= ignore_counter - 1;
                        mem_rd_ready_stream_buff <= '1';
                    end if;
                else
                    mem_rd_ready_stream_buff <= '1';
                end if;
                   
    
            elsif (depth_available < 9 and (not mem_access_needed) and ((not miss_general) or hit_stream_buffer) and (not non_cacheable_range_general) and depth_available /= 0) or filling_stream_buffer then

                bypass_fifo_stream <= '1';

                if (initial_cycle_fill) then        --Initial values that are transfered from the main control process
                    initial_cycle_fill      <= False;
                    busy_stream_buffer      <=  True;   --the stream buffer is busy

                    filling_stream_buffer <= True;

                    mem_rd_addr_stream_buff(cache_address_width_instruction-1 downto cache_offset_width_instruction)    <= std_logic_vector(unsigned(tag_stream_buffer(depth_available-1)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction)); --Read address
                    mem_rd_addr_stream_buff(cache_offset_width_instruction-1 downto 0)                                  <= (others=>'0');

                    early_araddr_stream(cache_address_width_instruction-1 downto cache_offset_width_instruction)    <= std_logic_vector(unsigned(tag_stream_buffer(depth_available-1)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction)); --Read address
                    early_araddr_stream(cache_offset_width_instruction-1 downto 0)                                  <= (others=>'0');

                    early_arvalid_stream <= '1';

                    mem_rd_en_stream_buff       <= '1';
                    stream_buffer_burst         <= '1';
                    burst_size_buff             <= 3;

                    if shift_stream_buffer = '0' then
                        tag_stream_buffer(depth_available)     <= std_logic_vector(unsigned(tag_stream_buffer(depth_available-1)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction));   

                    else --If we are shifting
                        for each_element in 0 to stream_buffer_depth - 2 loop
                            block_stream_buffer(each_element) <= block_stream_buffer(each_element + 1);
                            available_stream_buffer(each_element) <= available_stream_buffer(each_element + 1);
                            if each_element = depth_available - 1 then
                                tag_stream_buffer(each_element) <= std_logic_vector(unsigned(tag_stream_buffer(depth_available-1)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction));
                            else
                                tag_stream_buffer(each_element) <= tag_stream_buffer(each_element + 1);
                            end if;
                        end loop;  
                            --Todo: not necessarily the last element, but the tail, also for the tag that is incremented. This is to allow the race in the future.
                        block_stream_buffer(stream_buffer_depth-1)      <= (others => '0');
                        available_stream_buffer(stream_buffer_depth-1)  <= '0';
                        tag_stream_buffer(stream_buffer_depth-1)        <= (others => '0');

                        depth_available <= depth_available - 1;
                    end if;
                end if;
            
                mem_rd_handshake_stream   := mem_rd_valid='1' and mem_rd_ready_stream_buff='1'; --Handshake


                if (mem_rd_handshake_stream) then   --If we have a handshake

                    if shift_stream_buffer = '0' then
                        block_stream_buffer(depth_available)     <= mem_rd_data;
                        available_stream_buffer(depth_available) <= '1';
                        depth_available                          <= depth_available + 1;
                    else

                        for each_element in 0 to stream_buffer_depth - 2 loop
                            tag_stream_buffer(each_element) <= tag_stream_buffer(each_element + 1);
                            if each_element = depth_available - 1 then
                                block_stream_buffer(each_element)     <= mem_rd_data;
                                available_stream_buffer(each_element) <= '1';
                            else
                                block_stream_buffer(each_element) <= block_stream_buffer(each_element + 1);
                                available_stream_buffer(each_element) <= available_stream_buffer(each_element + 1);
                            end if;
                        end loop;  
                            --Todo: not necessarily the last element, but the tail, also for the tag that is incremented. This is to allow the race in the future.
                        block_stream_buffer(stream_buffer_depth-1)      <= (others => '0');
                        available_stream_buffer(stream_buffer_depth-1)  <= '0';
                        tag_stream_buffer(stream_buffer_depth-1)        <= (others => '0');

                    end if;

                    if fill_counter = burst_size_buff then

                        --If the counter reached the final value
                        --Stop reading
                        mem_rd_en_stream_buff           <= '0';
                        stream_buffer_burst             <= '0';
                        mem_rd_ready_stream_buff        <= '0';

                        
                        if refill_stream_buffer = '0' and not refill_queued then
                            busy_stream_buffer <= False;
                        end if;
                        filling_stream_buffer           <= False;

                        fill_counter                    <= 0;

                        --Next cycle the stream buffer is gonna be active is going to be the initial cycle where initializations happen
                        initial_cycle_fill           <= True;


                    else
                        mem_rd_ready_stream_buff <= '1';
                        if shift_stream_buffer = '0' then
                            tag_stream_buffer(depth_available+1) <= std_logic_vector(unsigned(tag_stream_buffer(depth_available)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction)); 
                        else
                            tag_stream_buffer(depth_available) <= std_logic_vector(unsigned(tag_stream_buffer(depth_available)) + to_unsigned(1, cache_address_width_instruction - cache_offset_width_instruction));  
                        end if;
                        fill_counter <= fill_counter + 1;
                    end if;

                     --Stream buffer is not busy anymore
                    if refill_stream_buffer = '1' then
                        refill_queued <= True;
                    end if;

                else    --Perform a read operation
                    --mem_rd_en_stream_buff       <= '1';
                    mem_rd_ready_stream_buff    <= '1';  

                    if refill_stream_buffer = '1' then
                        refill_queued <= True;
                    end if;

                    if shift_stream_buffer = '1' and not initial_cycle_fill then
                        for each_element in 0 to stream_buffer_depth - 2 loop
                            block_stream_buffer(each_element) <= block_stream_buffer(each_element + 1);
                            available_stream_buffer(each_element) <= available_stream_buffer(each_element + 1);
                            tag_stream_buffer(each_element) <= tag_stream_buffer(each_element + 1);
                        end loop;  


                            --Todo: not necessarily the last element, but the tail, also for the tag that is incremented. This is to allow the race in the future.
                        block_stream_buffer(stream_buffer_depth-1)      <= (others => '0');
                        available_stream_buffer(stream_buffer_depth-1)  <= '0';
                        tag_stream_buffer(stream_buffer_depth-1)        <= (others => '0');

                        depth_available <= depth_available - 1;
                    end if;     

                end if;                
            
            elsif (shift_stream_buffer = '1') then --If we are shifting up and replacing the tail
 
                for each_element in 0 to stream_buffer_depth - 2 loop
                    block_stream_buffer(each_element) <= block_stream_buffer(each_element + 1);
                    available_stream_buffer(each_element) <= available_stream_buffer(each_element + 1);
                    tag_stream_buffer(each_element) <= tag_stream_buffer(each_element + 1);
                end loop;  


                    --Todo: not necessarily the last element, but the tail, also for the tag that is incremented. This is to allow the race in the future.
                block_stream_buffer(stream_buffer_depth-1)      <= (others => '0');
                available_stream_buffer(stream_buffer_depth-1)  <= '0';
                tag_stream_buffer(stream_buffer_depth-1)        <= (others => '0');

                depth_available <= depth_available - 1;

            end if;


        end if;
    end process;               

              

    -------------------------------------------------------------
    ---The main sequenctial control process
    -------------------------------------------------------------
    process (aclk)
        variable mem_wr_handshake         : Boolean;
        variable mem_rd_handshake         : Boolean;
        variable mem_access_exread_block  : Boolean;
        variable mem_access_exwrite_block : Boolean;
        variable mem_access_word          : Boolean;
    begin
        if rising_edge(aclk) then
            --Default values
            shift_stream_buffer     <= '0';
            refill_stream_buffer    <= '0';
            cpu_pause_buff          <= '0';
            debug_signal            <= NONE;
            bypass_fifo_cache       <= '0';
            early_araddr_cache      <= (others => '0');
            early_arvalid_cache     <= '0';
            early_burst_cache       <= (others => '0');

            if aresetn='0' then
                mem_access_needed <= False;
                mem_wr_en_buff    <= '0';
                mem_wr_valid_buff <= '0';
                mem_rd_ready_buff <= '0';
                mem_rd_en_buff    <= '0';
                cpu_pause_buff    <= '0';
                mem_cache_line    <= '0';
                valid_rows_instruction        <= (others=>(others=>'0'));
                valid_rows_data               <= (others=>(others=>'0'));
                dirty_rows_data               <= (others=>(others=>'0'));
            else
                if stream_buffer_pending then
                    debug_signal <= REFILL;
                    --Todo: make this general for wide lines
                    cpu_pause_buff <= '1';
                    if (not busy_stream_buffer) and (not just_started_refill) then

                        block_rows_instruction(cpu_index_instruction)((replace_way_instruction+1)*2**cache_offset_width_instruction*8 - 1  downto replace_way_instruction*2**cache_offset_width_instruction*8) <= block_stream_buffer(0);
                        tag_rows_instruction(cpu_index_instruction)((1+replace_way_instruction)*tag_width_instruction-1 downto replace_way_instruction*tag_width_instruction) <= cpu_tag_instruction;
                        valid_rows_instruction(cpu_index_instruction)(replace_way_instruction) <= '1';

                        cpu_rd_data <= block_stream_buffer(0);   --Todo: generalize this for wide lines    

                        stream_buffer_pending <= False; 

                        just_started_refill <= True;
                        cpu_pause_buff  <= '0';

                        shift_stream_buffer <= '1';
                    else
                        just_started_refill <= False;   
                    end if;      

                           
                ------------------------------------------
                -- Memory access Instruction Fetch --Only for non_cacheable, cacheable goes through stream buffer
                ------------------------------------------
                elsif mem_access_needed and instruction_access then
                    debug_signal <= MEM_INSTRUCTION;
                    cpu_pause_buff <= '1';               
                    mem_rd_handshake         := mem_rd_valid='1' and mem_rd_ready_buff='1'; 
       
                    if not busy_stream_buffer then
                        bypass_fifo_cache <= '1';
                        if mem_rd_handshake then 
                            cpu_rd_data <= mem_rd_data; --If we have a read word and a handshake, simply forward the data
                            mem_rd_en_buff <= '0';
                            mem_rd_ready_buff <= '0';
                            mem_access_needed <= False;

                            cpu_pause_buff    <= '0'; --continue
                            instruction_access <= False;
                        else
                            mem_rd_ready_buff <= '1'; 
                        end if;
                    end if;

                ------------------------------------------
                -- Memory access Data operation
                ------------------------------------------
                elsif mem_access_needed and not instruction_access then    
                    debug_signal <= MEM_DATA;

                    cpu_pause_buff <= '1'; 


                    if (not busy_stream_buffer) then
                        mem_wr_handshake         := mem_wr_valid_buff='1' and mem_wr_ready='1';
                        mem_rd_handshake         := mem_rd_valid='1' and mem_rd_ready_buff='1';
                        mem_access_exread_block  := mem_access_mode=READ_BLOCK or mem_access_mode=EXCHANGE_BLOCK;
                        mem_access_exwrite_block := mem_access_mode=WRITE_BLOCK or mem_access_mode=EXCHANGE_BLOCK;
                        mem_access_word          := mem_access_mode=READ_WORD or mem_access_mode=WRITE_WORD;   

                        if (mem_access_exread_block and not mem_access_exwrite_block) then
                            bypass_fifo_cache <= '1';
                        end if;

                        --These signals are relevant if the block width is 1 word           
                        if (mem_rd_handshake) then 
                            axi_finished_read <= True;
                        end if;
                        if (mem_wr_handshake) then
                            axi_finished_write <= True;
                        end if;

                        --If we have a write handshake, a write operation and the counter is still low enough, write data into memory
                        if mem_wr_handshake and mem_access_exwrite_block and memory_wr_count_data/=2**block_word_width_data-1 then
                            mem_wr_data <= block_rows_data(mem_index_data)(mem_way_data*2**cache_offset_width_data*8+(memory_wr_count_data+2)*cpu_data_width-1 downto mem_way_data*2**cache_offset_width_data*8+(memory_wr_count_data+1)*cpu_data_width);
                        end if;

                        --If we have a read handshake, a read operation and the counter is still low enough, read data from memory, counter is not needed here
                        if mem_rd_handshake and mem_access_exread_block then
                            block_rows_data(mem_index_data)(mem_way_data*2**cache_offset_width_data*8+(memory_rd_count_data+1)*cpu_data_width-1 downto mem_way_data*2**cache_offset_width_data*8+memory_rd_count_data*cpu_data_width) <= mem_rd_data;
                        end if;

                        --If we have a read word and a handshake, simply forward the data
                        if mem_rd_handshake and mem_access_mode=READ_WORD then
                            cpu_rd_data <= mem_rd_data;
                        end if;        


                        if (mem_access_exwrite_block and memory_wr_count_data=2**block_word_width_data-1 and block_word_width_data/=0) or (mem_access_exwrite_block and mem_wr_handshake and block_word_width_data=0) or (mem_access_mode=WRITE_WORD and mem_wr_handshake) then
                            mem_wr_en_buff <= '0';
                        end if;
                        if (mem_access_exread_block and memory_rd_count_data=2**block_word_width_data-1 and block_word_width_data/=0) or (mem_access_exread_block and mem_rd_handshake and block_word_width_data=0) or (mem_access_mode=READ_WORD and mem_rd_handshake) then
                            mem_rd_en_buff <= '0';
                        end if;                   
                        if (mem_access_exwrite_block and memory_wr_count_data/=2**block_word_width_data-1 and block_word_width_data/=0) or (mem_access_exwrite_block and not mem_wr_handshake and not axi_finished_write and block_word_width_data=0) or (mem_access_mode=WRITE_WORD and not mem_wr_handshake) then
                            mem_wr_valid_buff <= '1';
                        else
                            mem_wr_valid_buff <= '0';
                        end if;
                        if ((mem_access_mode=READ_BLOCK or (mem_access_mode=EXCHANGE_BLOCK and memory_rd_count_data/=memory_wr_count_data)) and memory_rd_count_data/=2**block_word_width_data-1 and block_word_width_data/=0) or (mem_access_mode=READ_BLOCK and not mem_rd_handshake and block_word_width_data=0  and not axi_finished_read) or (mem_access_mode=EXCHANGE_BLOCK and not mem_rd_handshake and block_word_width_data=0  and not axi_finished_read) or (mem_access_mode=READ_WORD and not mem_rd_handshake) then
                            mem_rd_ready_buff <= '1';
                        else
                            mem_rd_ready_buff <= '0';
                        end if;                    
                        if mem_access_exwrite_block and mem_wr_handshake and memory_wr_count_data/=2**block_word_width_data-1 then
                            memory_wr_count_data <= memory_wr_count_data+1;
                        end if;
                        if (mem_access_mode=READ_BLOCK or (mem_access_mode=EXCHANGE_BLOCK and memory_rd_count_data/=memory_wr_count_data)) and mem_rd_handshake and memory_rd_count_data/=2**block_word_width_data-1 then
                            memory_rd_count_data <= memory_rd_count_data+1;
                        end if;           
        

                        --Doing the final operation which was originally requested by the CPU
                        if mem_access_exread_block and (memory_rd_count_data=2**block_word_width_data-1) then
                            for each_byte in 0 to cpu_data_width/8-1 loop
                                if or_reduce(replace_write_enables)='1' then    -- Write operation
                                    if replace_write_enables(each_byte)='1' then
                                        block_rows_data(mem_index_data)(mem_way_data*2**cache_offset_width_data*8+replace_offset_data*8+(each_byte+1)*8-1 downto mem_way_data*2**cache_offset_width_data*8+replace_offset_data*8+each_byte*8) <= replace_write_data(7+each_byte*8 downto 0+each_byte*8);
                                        dirty_rows_data(mem_index_data)(mem_way_data) <= '1';
                                    end if;
                                else    --Read operation
                                    cpu_rd_data(7+each_byte*8 downto 0+each_byte*8) <= block_rows_data(mem_index_data)(mem_way_data*2**cache_offset_width_data*8+replace_offset_data*8+(each_byte+1)*8-1 downto mem_way_data*2**cache_offset_width_data*8+replace_offset_data*8+each_byte*8);
                                end if;
                            end loop;
                        end if;

                        --Exit this state
                        if ((mem_access_exwrite_block or mem_access_exread_block) and mem_wr_en_buff='0' and mem_rd_en_buff='0') or (mem_access_word and (mem_wr_handshake or mem_rd_handshake)) then
                            mem_access_needed <= False;
                            cpu_pause_buff    <= '0';
                            mem_cache_line  <= '0';
                        end if;
                    end if;

                ------------------------------------------
                -- Address is not in the cacheable range
                ------------------------------------------ 

                elsif non_cacheable_range_general then
                    debug_signal <= NON_CACHE;
                    --This is added for symmetry, Todo: remove it to make the code cleaner
                    if (instruction_fetch = '1') then
                        instruction_access <= True;
                    else
                        instruction_access <= False;
                    end if;

                    cpu_pause_buff    <= '1';
                    mem_access_needed <= True;
                    if or_reduce(cpu_wr_byte_en)='1' then --if we have a write operation
                        --Already start with the write access
                        mem_access_mode <= WRITE_WORD;
                        mem_wr_addr     <= cpu_next_address;
                        mem_wr_byte_en  <= cpu_wr_byte_en;
                        mem_wr_data     <= cpu_wr_data;
                        --memory_wr_count <= 0;
                        mem_wr_en_buff  <= '1';
                    else    --if we have a read operation
                        --Already start with the read access
                        mem_access_mode <= READ_WORD;
                        mem_rd_addr_buff     <= cpu_next_address;
                        if (not busy_stream_buffer) then
                            early_araddr_cache   <= cpu_next_address;
                            early_arvalid_cache  <= '1';
                        end if;
                        --memory_rd_count <= 0;
                        mem_rd_en_buff  <= '1';
                    end if;
                ------------------------------------------
                -- Cache hit Instruction
                ------------------------------------------ 
                elsif cache_hit_instruction = True and instruction_fetch = '1' then --If we have an instruction fetch
                    debug_signal <= HIT_INSTRUCTION;
                    for each_byte in 0 to cpu_data_width/8-1 loop --Only a read is possible
                        cpu_rd_data(7+each_byte*8 downto 0+each_byte*8) <= block_rows_instruction(cpu_index_instruction)(cpu_way_instruction*2**cache_offset_width_instruction*8+cpu_offset_instruction*8+(each_byte+1)*8-1 downto cpu_way_instruction*2**cache_offset_width_instruction*8+cpu_offset_instruction*8+each_byte*8);
                    end loop;

                elsif hit_stream_buffer then --If we have a hit from the stream buffer
                    debug_signal <= HIT_STREAM;
                    --Todo: make this general for wide lines
                    if shift_stream_buffer = '0' then
                        block_rows_instruction(cpu_index_instruction)((replace_way_instruction+1)*2**cache_offset_width_instruction*8 - 1  downto replace_way_instruction*2**cache_offset_width_instruction*8) <= block_stream_buffer(0);
                        cpu_rd_data <= block_stream_buffer(0);   --Todo: generalize this for wide lines
                    else
                        block_rows_instruction(cpu_index_instruction)((replace_way_instruction+1)*2**cache_offset_width_instruction*8 - 1  downto replace_way_instruction*2**cache_offset_width_instruction*8) <= block_stream_buffer(1); 
                        cpu_rd_data <= block_stream_buffer(1);   --Todo: generalize this for wide lines
                    end if;
                    tag_rows_instruction(cpu_index_instruction)((1+replace_way_instruction)*tag_width_instruction-1 downto replace_way_instruction*tag_width_instruction) <= cpu_tag_instruction;
                    valid_rows_instruction(cpu_index_instruction)(replace_way_instruction) <= '1';
                    shift_stream_buffer <= '1';
     
                ------------------------------------------
                -- Cache hit Data
                ------------------------------------------ 
                elsif cache_hit_data = True and instruction_fetch = '0' then --If we have a data operation
                    debug_signal <= HIT_DATA;
                    for each_byte in 0 to cpu_data_width/8-1 loop
                        if or_reduce(cpu_wr_byte_en)='1' then   --if we have a write
                            if cpu_wr_byte_en(each_byte)='1' then --Check whether the byte should be written
                                block_rows_data(cpu_index_data)(cpu_way_data*2**cache_offset_width_data*8+cpu_offset_data*8+(each_byte+1)*8-1 downto cpu_way_data*2**cache_offset_width_data*8+cpu_offset_data*8+each_byte*8) <= cpu_wr_data(7+each_byte*8 downto 0+each_byte*8);
                                dirty_rows_data(cpu_index_data)(cpu_way_data) <= '1';
                            end if;
                        else  --Read
                            cpu_rd_data(7+each_byte*8 downto 0+each_byte*8) <= block_rows_data(cpu_index_data)(cpu_way_data*2**cache_offset_width_data*8+cpu_offset_data*8+(each_byte+1)*8-1 downto cpu_way_data*2**cache_offset_width_data*8+cpu_offset_data*8+each_byte*8);
                        end if;
                    end loop;


                ------------------------------------------
                -- Cache miss
                ------------------------------------------ 
                elsif miss_general then
                    replace_write_enables <= cpu_wr_byte_en; --Store the replace write enables

                        ------------------------------------------
                        -- Data access Cache miss
                        ------------------------------------------                             
                        if instruction_fetch = '0' then --Data access --Todo: writes should be allowed directly
                                debug_signal <= MISS_DATA;
                                cpu_pause_buff <= '1'; --We gotta get the CPU to wait
                                instruction_access <= False; --Buffer that this is an instruction access operation
                                mem_access_needed  <= True;

                                mem_way_data       <= replace_way_data; --Buffer the replace way
                                mem_cache_line     <= '1'; --Tell the main memory that this is a cache line access, TODO: look into how this is related to bursts
                                replace_write_data <= cpu_wr_data; --Buffer the write data
                                replace_offset_data     <= cpu_offset_data; --Buffer the offset
                                --Store the tag directly into cache: better than buffering it
                                tag_rows_data(cpu_index_data)((1+replace_way_data)*tag_width_data-1 downto replace_way_data*tag_width_data) <= cpu_tag_data;
                                mem_index_data          <= cpu_index_data; --Buffer the index
                                if valid_rows_data(cpu_index_data)(replace_way_data)='1' and dirty_rows_data(cpu_index_data)(replace_way_data) = '1' then --If valid and dirty, exchange block 
                                    mem_access_mode <= EXCHANGE_BLOCK;

                                    --We start a write access already to gain performance
                                    mem_wr_addr(cpu_addr_width-1 downto cpu_addr_width-4)        <= "0001";
                                    mem_wr_addr(cpu_addr_width-5 downto cache_address_width_data)     <= (others => '0');
                                    mem_wr_addr(cache_address_width_data-1 downto cache_offset_width_data) <= tag_rows_data(cpu_index_data)((1+replace_way_data)*tag_width_data-1 downto replace_way_data*tag_width_data) & std_logic_vector(to_unsigned(cpu_index_data,cache_index_width_data));
                                    mem_wr_addr(cache_offset_width_data-1 downto 0)                   <= (others=>'0');
                                    mem_wr_byte_en     <= (others=>'1');
                                    mem_wr_en_buff     <= '1';
                                    mem_wr_data        <= block_rows_data(cpu_index_data)(replace_way_data*2**cache_offset_width_data*8+cpu_data_width-1 downto replace_way_data*2**cache_offset_width_data*8);

                                    memory_wr_count_data    <= 0;
                                    axi_finished_write <= False;

                                    dirty_rows_data(cpu_index_data)(replace_way_data) <= '0';

                                else --If not valid or dirty, just read
                                    valid_rows_data(cpu_index_data)(replace_way_data)   <= '1';
                                    mem_access_mode                                     <= READ_BLOCK;
                                end if;   
                                --Start already with the read instruction
                                mem_rd_addr_buff(cpu_addr_width-1 downto cpu_addr_width-4)                  <= "0001";
                                mem_rd_addr_buff(cpu_addr_width-5 downto cache_address_width_data)          <= (others => '0');
                                mem_rd_addr_buff(cache_address_width_data-1 downto cache_offset_width_data) <= cpu_tag_data & std_logic_vector(to_unsigned(cpu_index_data,cache_index_width_data));
                                mem_rd_addr_buff(cache_offset_width_data-1 downto 0)                        <= (others=>'0');
                                
                                mem_rd_en_buff                                               <= '1';

                                if (not busy_stream_buffer) then
                                    early_araddr_cache(cpu_addr_width-1 downto cpu_addr_width-4)                  <= "0001";
                                    early_araddr_cache(cpu_addr_width-5 downto cache_address_width_data)          <= (others => '0');
                                    early_araddr_cache(cache_address_width_data-1 downto cache_offset_width_data) <= cpu_tag_data & std_logic_vector(to_unsigned(cpu_index_data,cache_index_width_data));
                                    early_araddr_cache(cache_offset_width_data-1 downto 0)                        <= (others=>'0');
                                    early_arvalid_cache  <= '1';
                                end if;
                                memory_rd_count_data   <= 0;
                                axi_finished_read <= False;    
                        ------------------------------------------
                        -- Intruction Fetch Cache miss
                        ------------------------------------------                        
                        elsif (not busy_stream_buffer) or (busy_stream_buffer and ((available_stream_buffer(0) = '1' and shift_stream_buffer = '0') or (available_stream_buffer(1) = '1' and shift_stream_buffer = '1'))) then --instruction access
                            debug_signal <= MISS_INSTRUCTION;
                            cpu_pause_buff <= '1'; --We gotta get the CPU to wait
                            --Give commands to the stream buffer to Flush and start re-filling
                            refill_stream_buffer <= '1';
                            flush_stream_buffer_address <= cpu_tag_instruction & std_logic_vector(to_unsigned(cpu_index_instruction,cache_index_width_instruction)); --send the address to the stream buffer

                            --Buffer some needed information to replace later
                            mem_way_instruction             <= replace_way_instruction; --Buffer the replace way
                            replace_offset_instruction      <= cpu_offset_instruction;   --Buffer the offset
                            mem_index_instruction           <= cpu_index_instruction;    --Buffer the index

                            --Perform a Stream Buffer Refill operation
                            --mem_access_mode                                                         <= REFILL_STREAM_BUFF;
                            stream_buffer_pending                                                   <= True;

                            --Start already with the read instruction --Todo
                            --mem_rd_addr_buff(cache_address_width_instruction-1 downto cache_offset_width_instruction)           <= cpu_tag_instruction & std_logic_vector(to_unsigned(cpu_index_instruction,cache_index_width_instruction));
                            --mem_rd_addr_buff(cache_offset_width_instruction-1 downto 0)                                         <= (others=>'0');
                            --mem_rd_en_buff                                                                          <= '1';
                            --mem_rd_ready_buff                                                                       <= '1';

                        else
                            cpu_pause_buff <= '1';
                            debug_signal <= WAITING_FOR_FILL;
                 
                        end if;
                
                end if;

            end if;
        end if;
    end process;
    
end Behavioral;