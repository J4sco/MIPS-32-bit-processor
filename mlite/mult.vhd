---------------------------------------------------------------------
-- TITLE: Multiplication and Division Unit
-- ORIGINAL AUTHORS: Steve Rhoads (rhoadss@yahoo.com)
-- DATE CREATED: 1/31/01
-- FILENAME: mult.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--            Software 'as is' without warranty.  Author liable for nothing.
-- AUTHORS: Jacopo Costantini
-- DESCRIPTION:
--    Implements the multiplication in 16 clocks
--               the division unit in 32 clocks for standard execution     
----------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;    -- all the operators are called as the number involved are unsigned
use IEEE.std_logic_arith.all;
use work.mlite_pack.all;

entity mult is
   generic(mult_type  : string := "DEFAULT");
   port(clk       : in std_logic;
        reset_in  : in std_logic;
        a, b      : in std_logic_vector(31 downto 0);
        mult_func : in mult_function_type;
        c_mult    : out std_logic_vector(31 downto 0);
        pause_out : out std_logic);
end; --entity mult

architecture logic of mult is

   constant MODE_MULT : std_logic := '1';
   constant MODE_DIV  : std_logic := '0';

   signal mode_reg    : std_logic;
   signal negate_reg_LO  : std_logic;
   signal negate_reg_HI  : std_logic;
   signal sign_reg    : std_logic;
   signal sign2_reg   : std_logic;
   signal count_reg   : std_logic_vector(5 downto 0);
   signal aa_reg      : std_logic_vector(31 downto 0);
   signal bb_reg      : std_logic_vector(31 downto 0);
   signal upper_reg   : std_logic_vector(31 downto 0);
   signal lower_reg   : std_logic_vector(31 downto 0);

   signal a_neg       : std_logic_vector(31 downto 0);
   signal b_neg       : std_logic_vector(31 downto 0);
   signal sum         : std_logic_vector(32 downto 0);
   
   signal init0            : std_logic := '0';
   signal is_signed        : boolean;
   signal a_triple_std     : std_logic_vector(33 downto 0);

begin
 
   -- Result
   c_mult <= lower_reg when mult_func = MULT_READ_LO and negate_reg_LO = '0' else 
             bv_negate(lower_reg) when mult_func = MULT_READ_LO and negate_reg_LO = '1' else
             upper_reg when mult_func = MULT_READ_HI and negate_reg_HI = '0' else 
             bv_negate(upper_reg) when mult_func = MULT_READ_HI and negate_reg_HI = '1' else
             ZERO;
   pause_out <= '1' when (count_reg /= "000000") and 
             (mult_func = MULT_READ_LO or mult_func = MULT_READ_HI) else '0';

   -- ABS and remainder signals
   a_neg <= bv_negate(a);
   b_neg <= bv_negate(b);
   sum <= cs_adder(upper_reg, aa_reg, mode_reg);
   a_triple_std <= ('0' & '0' & aa_reg) + ('0' & aa_reg & '0');
  
   --multiplication/division unit
   mult_proc: process(clk, reset_in, a, b, mult_func,
      a_neg, b_neg, sum, sign_reg, mode_reg, negate_reg_LO, 
      count_reg, aa_reg, bb_reg, upper_reg, lower_reg)
      
      variable count : std_logic_vector(2 downto 0);
      variable booth_sum : std_logic_vector(32 downto 0);
      variable radix4_sum : std_logic_vector(34 downto 0);
   
   begin
      count := "001";
      if reset_in = '1' then
         mode_reg <= '0';
         negate_reg_LO <= '0';
         negate_reg_HI <= '0';
         sign_reg <= '0';
         sign2_reg <= '0';
         count_reg <= "000000";
         aa_reg <= ZERO;
         bb_reg <= ZERO;
         upper_reg <= ZERO;
         lower_reg <= ZERO;
          --radix 4 mult
         init0 <= '0';
         booth_sum := (others => '0');
         is_signed <= False;
         radix4_sum := (others => '0');
         
      elsif rising_edge(clk) then
         case mult_func is
            when MULT_WRITE_LO =>
               lower_reg <= a;
               negate_reg_LO <= '0';
               negate_reg_HI <= '0';
            when MULT_WRITE_HI =>
               upper_reg <= a;
               negate_reg_LO <= '0';
               negate_reg_HI <= '0';
            when MULT_MULT =>
               mode_reg <= MODE_MULT;
               aa_reg <= a;
               bb_reg <= b;
               upper_reg <= ZERO;
               count_reg <= "100000";
               negate_reg_LO <= '0';
               negate_reg_HI <= '0';
               sign_reg <= '0';
               sign2_reg <= '0';
               is_signed <= False;
               radix4_sum := (others => '0');
            when MULT_SIGNED_MULT =>
               mode_reg <= MODE_MULT;
               aa_reg <= a;
               bb_reg <= b;
               upper_reg <= ZERO;
               count_reg <= "100000";
               negate_reg_LO <= '0';
               negate_reg_HI <= '0';
               is_signed <= True;
               init0 <= '0';
               booth_sum := (others => '0');
            when MULT_DIVIDE =>
               mode_reg <= MODE_DIV;
               aa_reg <= b(0) & ZERO(30 downto 0);
               bb_reg <= b;
               upper_reg <= a;
               count_reg <= "100000";
               negate_reg_LO <= '0';
               negate_reg_HI <= '0';
            when MULT_SIGNED_DIVIDE =>
               mode_reg <= MODE_DIV;
               if b(31) = '0' then
                  aa_reg(31) <= b(0);
                  bb_reg <= b;
               else
                  aa_reg(31) <= b_neg(0);
                  bb_reg <= b_neg;
               end if;
               if a(31) = '0' then
                  upper_reg <= a;
               else
                  upper_reg <= a_neg;
               end if;
               aa_reg(30 downto 0) <= ZERO(30 downto 0);
               count_reg <= "100000";
               negate_reg_LO <= a(31) xor b(31);
               negate_reg_HI <= a(31);
            when others =>

               if count_reg /= "000000" then
                  if mode_reg = MODE_MULT then
                    -----------------------------------------
                     ----------- MULTIPLICATION --------------
                     -----------------------------------------
                     if is_signed then 
                        ---- MODIFIED RADIX4 BOOTH ALGORITHM ----
                           count := "010";
                           case (bb_reg(1 downto 0) & init0) is
                              when "001" | "010" =>   	-- add multiplicand
                                 booth_sum := (upper_reg(31) & upper_reg) + (aa_reg(31) & aa_reg);
                              when "011" => 					-- add 2*multiplicand
                                 booth_sum := (upper_reg(31) & upper_reg) + (aa_reg & '0');
                              when "100" => 					-- subtract 2*multiplicand
                                 booth_sum := (upper_reg(31) & upper_reg) - (aa_reg & '0');
                              when "101" | "110" =>   	-- subract multiplicand
                                 booth_sum := (upper_reg(31) & upper_reg) - (aa_reg(31) & aa_reg);
                              when others => 				-- add 0 or subtract 0: just shift
                                 booth_sum := upper_reg(31) & upper_reg;
                           end case;
                           upper_reg <= (booth_sum(32) & booth_sum(32 downto 2));  -- shift out 2 bits (insert the MSB)
                           lower_reg <= (booth_sum(1 downto 0) & lower_reg(31 downto 2));   --shift out 2 bits
                           bb_reg <= "00" & bb_reg(31 downto 2);   --shift out b 
                           init0 <= bb_reg(1);
                     else
                        ---- STANDARD RADIX4 ALGORITHM ----
                        count := "010";
                        case (bb_reg(1 downto 0)) is
                           when "01" =>   	-- add multiplicand
                              radix4_sum := ("000" & upper_reg) + ("000" & aa_reg);
                           when "10" => 	-- add 2*multiplicand
                              radix4_sum := ("000" & upper_reg) + ("00" & aa_reg & '0');
                           when "11" => 	-- add 3*multiplicand
                              radix4_sum := ("000" & upper_reg) + ('0' & a_triple_std);
                           when others => 	-- just shift
                              radix4_sum := "000" & upper_reg;
                        end case;
                        upper_reg <= radix4_sum(33 downto 2);  -- shift out 2 bits
                        lower_reg <= (radix4_sum(1 downto 0) & lower_reg(31 downto 2));   --shift out 2 bits
                        bb_reg <= "00" & bb_reg(31 downto 2);   --shift out b
                        
                    end if;  -- is_signed
                  else   -- mode_reg /= MODE_MULT
                  -----------------------------------------
                  --------------- DIVISION ----------------
                  -----------------------------------------
                     if sum(32) = '0' and aa_reg /= ZERO and 
                           bb_reg(31 downto 1) = ZERO(31 downto 1) then
                        upper_reg <= sum(31 downto 0);
                        lower_reg(0) <= '1';
                     else
                        lower_reg(0) <= '0';
                     end if;
                     aa_reg <= bb_reg(1) & aa_reg(31 downto 1);
                     lower_reg(31 downto 1) <= lower_reg(30 downto 0);
                     bb_reg <= '0' & bb_reg(31 downto 1);
                  end if;
                  count_reg <= count_reg - count;
               end if; --count

         end case;
         
      end if;

   end process;
    
end; --architecture logic