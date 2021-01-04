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

entity DynamicPRMG_TB is
end DynamicPRMG_TB;

architecture ArcDynamicPRMG_TB of DynamicPRMG_TB is

component DynamicPRMG is
  generic (
    FracDividerWidth : Natural := 10; -- Разрядность дробного делителя.
    AuxDividerWidth : Natural := 7; -- Разрядность счётчика дополнительного деления частоты на 125.
    AuxDividerRatio : Natural := 125; -- Коэффициент пересчёта счётчика дополнительного деления частоты на 125.
    DualModeDividerWidth : Natural := 5; -- Разрядность двухрежимного счётчика-делителя.
    CycleCounterWidth : Natural := 8; -- Разрядность счётчик подсчёта циклов формирования сигнала ПРМГ.
    --
    SignalBits : Natural := 8 -- Разрядность обрабатываемых сигналов.
  );
  port (
    -- Вход сброса.
    Res : in STD_LOGIC;
    -- Вход тактового сигнала.
    Clk : in STD_LOGIC;
    -- Входы ключей. Два входа задают 4 комбинации - выбор одного кода из 4х возможных.
    SW0, SW1 : in STD_LOGIC;
    -- Включение режима имитации отклонений в динамике.
    DynamicMode_In : in STD_LOGIC;
    -- Включение удвоенной скорости в режиме имитации отклонений в динамике.
    DynamicSpeed2X_In : in STD_LOGIC;
    -- Стробы записи в регистры амплитуд для сигналов 1300 Гц и 2100 Гц, амплитуды используются при имитации в статике.
    Amp1300Hz_Wr, Amp2100Hz_Wr : in STD_LOGIC;
    -- Шина данных для приёма амплитуд сигналов 1300 Гц и 2100 Гц.
    AmpBus : in STD_LOGIC_VECTOR ((SignalBits - 1) downto 0); -- Шина приёма данных.
    -- Шина данных для выдачи амплитуд сигналов 1300 Гц и 2100 на ЦАП.
    DACBus : out STD_LOGIC_VECTOR ((SignalBits - 1) downto 0); -- Шина приёма данных.
    -- Выходы к светодиодам.
    LD0, LD1, LD2, LD3 : out STD_LOGIC
  );
end component;

-- Clock period definitions
constant BaseTimeStep : time := 10 ns;

-- Constants
constant FracDividerWidth_TB : Natural := 10;
constant AuxDividerWidth_TB : Natural := 7;
constant AuxDividerRatio_TB : Natural := 2; -- Для удобства проверки в симуляторе этот коэффициент деления уменьшен в 62.5 раза со 125 до 2.
constant DualModeDividerWidth_TB : Natural := 5;
constant CycleCounterWidth_TB : Natural := 8;
constant SignalBits_TB : Natural := 8;

-- Inputs of UUT
signal Res : STD_LOGIC := '0';
signal Clk : STD_LOGIC;
signal SW0, SW1 : STD_LOGIC := '0';
signal DynamicMode_In : STD_LOGIC := '0';
signal DynamicSpeed2X_In : STD_LOGIC := '0';
signal Amp1300Hz_Wr : STD_LOGIC := '0';
signal Amp2100Hz_Wr : STD_LOGIC := '0';
signal AmpBus : STD_LOGIC_VECTOR ((SignalBits_TB - 1) downto 0) := (others => '0');

-- Outputs of UUT
signal DACBus : STD_LOGIC_VECTOR ((SignalBits_TB - 1) downto 0);
signal LD0, LD1, LD2, LD3 : STD_LOGIC;

-- Auxiliary signals, etc.
signal Clk_Orig : STD_LOGIC := '0';
signal Clk_Gate : STD_LOGIC := '0';

begin

  -- Instantiate the Unit Under Test (UUT)
  UUT: DynamicPRMG
    generic map (
      FracDividerWidth => FracDividerWidth_TB,
      AuxDividerWidth => AuxDividerWidth_TB,
      AuxDividerRatio => AuxDividerRatio_TB,
      DualModeDividerWidth => DualModeDividerWidth_TB,
      CycleCounterWidth => CycleCounterWidth_TB,
      SignalBits => SignalBits_TB)
    port map (
      Res => Res,
      Clk => Clk,
      SW0 => SW0,
      SW1 => SW1,
      DynamicMode_In => DynamicMode_In,
      DynamicSpeed2X_In => DynamicSpeed2X_In,
      Amp1300Hz_Wr => Amp1300Hz_Wr,
      Amp2100Hz_Wr => Amp2100Hz_Wr,
      AmpBus => AmpBus,
      DACBus => DACBus,
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

      DynamicMode_In <= '1'; -- Включение режима работы в динамике.
      wait for BaseTimeStep * 4;

      wait; -- will wait forever

  end process;

end;
