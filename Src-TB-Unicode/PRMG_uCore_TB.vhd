----------------------------------------------------------------------------------
-- Copyright (C) 2016 SN
--
-- Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- 2. Redistributions in binary form must reproduce the above copyright notice,
-- this list of conditions and the following disclaimer in the documentation and/or
-- other materials provided with the distribution.
--
-- 3. Neither the name of the copyright holder nor the names of its contributors
-- may be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
-- IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
-- INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
-- DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
-- LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
-- OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
-- OF THE POSSIBILITY OF SUCH DAMAGE.
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity PRMG_uCore_TB is
end PRMG_uCore_TB;

architecture ArcPRMG_uCore_TB of PRMG_uCore_TB is

component PRMG_uCore is
  generic (
    FracDividerWidth : Natural := 10; -- Разрядность дробного делителя.
    AuxDividerWidth : Natural := 7; -- Разрядность счётчика дополнительного деления частоты на 125.
    AuxDividerRatio : Natural := 125; -- Коэффициент пересчёта счётчика дополнительного деления частоты на 125.
    DualModeDividerWidth : Natural := 5; -- Разрядность двухрежимного счётчика-делителя.
    CycleCounterWidth : Natural := 8 -- Разрядность счётчик подсчёта циклов формирования сигнала ПРМГ.
  );
  port (
    -- Вход сброса.
    Res : in STD_LOGIC;
    -- Вход тактового сигнала.
    Clk : in STD_LOGIC;
    -- Входы ключей. Два входа задают 4 комбинации - выбор одного кода из 4х возможных.
    SW0, SW1 : in STD_LOGIC;
    -- Выход стробов 54.6 кГц.
    Strobe54K6 : out STD_LOGIC;
    -- Выход сигнала 1300 Гц.
    Gate1300 : out STD_LOGIC;
    -- Выход сигнала 2100 Гц.
    Gate2100 : out STD_LOGIC;
    -- Выход стробов 25 Гц.
    Strobe25 : out STD_LOGIC;
    -- Выход стробов 12.5 Гц.
    Strobe12p5 : out STD_LOGIC;
    -- Выходы к светодиодам.
    LD0, LD1, LD2, LD3 : out STD_LOGIC
  );
end component;

-- Clock period definitions
constant BaseTimeStep : time := 10 ns;

-- Constants
constant FracDividerWidth_TB : Natural := 10;
constant AuxDividerWidth_TB : Natural := 7;
constant AuxDividerRatio_TB : Natural := 5; -- Для удобства проверки в симуляторе этот коэффициент деления уменьшен в 25 раз со 125 до 5.
constant DualModeDividerWidth_TB : Natural := 5;
constant CycleCounterWidth_TB : Natural := 8;

-- Inputs of UUT
signal Res : STD_LOGIC := '0';
signal Clk : STD_LOGIC;
signal SW0, SW1 : STD_LOGIC := '0';

-- Outputs of UUT
signal Strobe54K6 : STD_LOGIC;
signal Gate1300 : STD_LOGIC;
signal Gate2100 : STD_LOGIC;
signal Strobe25 : STD_LOGIC;
signal Strobe12p5 : STD_LOGIC;
signal LD0, LD1, LD2, LD3 : STD_LOGIC;

-- Auxiliary signals, etc.
signal Clk_Orig : STD_LOGIC := '0';
signal Clk_Gate : STD_LOGIC := '0';

begin

  -- Instantiate the Unit Under Test (UUT)
  UUT: PRMG_uCore
    generic map (
      FracDividerWidth => FracDividerWidth_TB,
      AuxDividerWidth => AuxDividerWidth_TB,
      AuxDividerRatio => AuxDividerRatio_TB,
      DualModeDividerWidth => DualModeDividerWidth_TB,
      CycleCounterWidth => CycleCounterWidth_TB)
    port map (
      Res => Res,
      Clk => Clk,
      SW0 => SW0,
      SW1 => SW1,
      Strobe54K6 => Strobe54K6,
      Gate1300 => Gate1300,
      Gate2100 => Gate2100,
      Strobe25 => Strobe25,
      Strobe12p5 => Strobe12p5,
      LD0 => LD0,
      LD1 => LD1,
      LD2 => LD2,
      LD3 => LD3);

  -- Clock process definitions
  Clk_Orig_process : process
    begin
      Clk_Orig <= '0';
      wait for BaseTimeStep;
      Clk_Orig <= '1';
      wait for BaseTimeStep;
    end process;

  Clk_Gate_process : process
    begin
      Clk_Gate <= '0';
      wait for BaseTimeStep * 8;
      Clk_Gate <= '1';
      wait; -- will wait forever
    end process;

  Clk <= Clk_Orig and Clk_Gate;

  -- Stimulus process
  Stim_process : process
    begin

      wait for BaseTimeStep * 4;

      Res <= '1'; -- Сброс.
      wait for BaseTimeStep * 2;

      Res <= '0'; -- Снятие сброса.
      wait for BaseTimeStep * 4;

      wait; -- will wait forever

  end process;

end;
