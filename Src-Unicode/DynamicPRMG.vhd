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
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- ================================================================================
-- Имитатор сигналов ПРМГ в статическом и динамическом режимах.

-- Данный модуль не предназначен для использования в качестве Top Level Entity,
-- поэтому в нём нет необходимых регистров для хранения входных данных. 
-- Модуль содержит минимальный набор логики и регистров для обеспечения
-- имитации отклонений в статике и динамике.

-- ================================================================================

-- Сигнал Res введён только для возможности симуляции в ISim.

entity DynamicPRMG is
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
    -- Включение режима имитации отклонений в динамике.
    DynamicMode : in STD_LOGIC;
    -- Включение удвоенной скорости в режиме имитации отклонений в динамике.
    DynamicSpeed2X : in STD_LOGIC;
    -- Шина данных для приёма амплитуды сигнала 1300 Гц, которая будет использоваться при имитации в статике.
    StaticAmp1300Bus : in STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
    -- Шина данных для приёма амплитуды сигнала 2100 Гц, которая будет использоваться при имитации в статике.
    StaticAmp2100Bus : in STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
    -- Шина данных для выдачи амплитуд сигналов 1300 Гц и 2100 на ЦАП.
    DACBus : out STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
    -- Шина данных для выдачи текущей амплитуды сигнала 1300 Гц на схему индикации.
    DynamicAmp1300Bus : out STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
    -- Шина данных для выдачи текущей амплитуды сигнала 2100 Гц на схему индикации.
    DynamicAmp2100Bus : out STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
    -- Выход стробов 54.6 кГц.
    Strobe54K6Out : out STD_LOGIC;
    -- Выход стробов 25 Гц.
    Strobe25Out : out STD_LOGIC;
    -- Выход стробов 12.5 Гц.
    Strobe12p5Out : out STD_LOGIC
  );
end DynamicPRMG;

architecture ArcDynamicPRMG of DynamicPRMG is

-- ==================== Сигналы 1300 Гц и 2100 Гц. ====================
signal Gate1300, Gate2100 : STD_LOGIC;
-- Эти же сигналы, но с задержкой на 1 такт работы схемы.
signal Gate1300_Adjusted, Gate2100_Adjusted : STD_LOGIC;

-- ==================== Стробы сигналов 25 Гц и 12.5 Гц. ====================
signal Strobe25, Strobe12p5 : STD_LOGIC;

-- ==================== Всё, что касается счётчика формирования интервалов циклограммы сигнала ПРМГ. ====================
-- Счётчик формирования интервалов циклограммы сигнала ПРМГ.
signal IntervalCounter : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
-- Признак переполнения счётчика формирования интервалов циклограммы сигнала ПРМГ.
signal IntervalCounterOvf : STD_LOGIC;
-- Признак нуля счётчика формирования интервалов циклограммы сигнала ПРМГ.
signal IntervalCounterZero : STD_LOGIC;

-- ==================== Всё, что касается конечного автомата управления генерацией циклограммы сигнала ПРМГ. ====================
-- Определение набора состояний конечного автомата управления генерацией циклограммы сигнала ПРМГ
type DynGeneration_States is (DynIdle_1st, Dyn_1300Hz_going_down, DynIdle_1300Hz_low, Dyn_1300Hz_going_up,
                              DynIdle_2nd, Dyn_2100Hz_going_down, DynIdle_2100Hz_low, Dyn_2100Hz_going_up);
-- Определение сигнала для конечного автомата управления генерацией циклограммы сигнала ПРМГ
signal DynGeneration_State : DynGeneration_States;
-- Для упрощения логики количество состояний конечного автомата сокращено, но введён дополнительный сигнал,
-- который является признаком первого или второго прохода по состояниям конечного автомата.
signal DynGeneration_HalfPeriod : STD_LOGIC;

-- ==================== Буфер для значения, подаваемого на ЦАП. ====================
signal DACBuffer : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);

-- ==================== Буферы для значений текущих амплитуд сигналов 1300 Гц и 2100 Гц, подаваемых на схему индикации. ====================
signal DynamicAmp1300Buffer, DynamicAmp2100Buffer : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);

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
    -- Выход стробов 54.6 кГц.
    Strobe54K6 : out STD_LOGIC;
    -- Выход сигнала 1300 Гц.
    Gate1300 : out STD_LOGIC;
    -- Выход сигнала 2100 Гц.
    Gate2100 : out STD_LOGIC;
    -- Выход стробов 25 Гц.
    Strobe25 : out STD_LOGIC;
    -- Выход стробов 12.5 Гц.
    Strobe12p5 : out STD_LOGIC
  );
end component;

begin

  -- Подключение компонента PRMG_uCore.
  Lab0_PRMG_uCore : PRMG_uCore
    generic map (
      FracDividerWidth => FracDividerWidth,
      AuxDividerWidth => AuxDividerWidth,
      AuxDividerRatio => AuxDividerRatio,
      DualModeDividerWidth => DualModeDividerWidth,
      CycleCounterWidth => CycleCounterWidth)
    port map (
      Res => Res,
      Clk => Clk,
      Strobe54K6 => Strobe54K6Out,
      Gate1300 => Gate1300,
      Gate2100 => Gate2100,
      Strobe25 => Strobe25,
      Strobe12p5 => Strobe12p5);

  -- Сквозная передача на выход стробов 25 Гц и 12.5 Гц.
  Strobe25Out <= Strobe25;
  Strobe12p5Out <= Strobe12p5;

  -- Сдвиг на 1 такт для сигналов 1300 Гц и 2100 Гц.
  process (Res, Clk)
  begin
    if (Res = '1') then
      Gate1300_Adjusted <= '0';
      Gate2100_Adjusted <= '0';
    elsif (Clk'event) then
      Gate1300_Adjusted <= Gate1300;
      Gate2100_Adjusted <= Gate2100;
    end if;
  end process;

  -- Счётчик формирования интервалов циклограммы сигнала ПРМГ.
  process (Res, Clk)
  begin
    if (Res = '1') then
      IntervalCounter <= (others => '1');
    elsif (Clk'event) then
      if (DynamicMode = '0') then
        IntervalCounter <= (others => '1');
      elsif ((Strobe25 = '1') and (DynamicSpeed2X = '1')) or (Strobe12p5 = '1') then

        -- DynIdle_1st, DynIdle_2nd.
        if (DynGeneration_State = DynIdle_1st) or (DynGeneration_State = DynIdle_2nd) then
          IntervalCounter <= IntervalCounter - 1;

        -- Dyn_1300Hz_going_down, Dyn_2100Hz_going_down.
        elsif (DynGeneration_State = Dyn_1300Hz_going_down) or (DynGeneration_State = Dyn_2100Hz_going_down) then
          if (IntervalCounterZero = '0') then
            IntervalCounter <= IntervalCounter - 1;
          end if;

        -- DynIdle_1300Hz_low, DynIdle_2100Hz_low.
        elsif (DynGeneration_State = DynIdle_1300Hz_low) or (DynGeneration_State = DynIdle_2100Hz_low) then
          IntervalCounter <= IntervalCounter + 1;

        -- Dyn_1300Hz_going_up, Dyn_2100Hz_going_up.
        else
          if (IntervalCounterOvf = '0') then
            IntervalCounter <= IntervalCounter + 1;
          end if;
        --
        end if;
      end if;
    end if;
  end process;

  -- Переполнение счётчика формирования интервалов циклограммы сигнала ПРМГ.
  IntervalCounterOvf <= '1' when (IntervalCounter = (IntervalCounter'Range => '1')) else '0';
  -- Ноль счётчика формирования интервалов циклограммы сигнала ПРМГ.
  IntervalCounterZero <= '1' when (IntervalCounter = (IntervalCounter'Range => '0')) else '0';

  -- Конечный автомат управления генерацией циклограммы сигнала ПРМГ.
  process (Res, Clk)
  begin
    if (Res = '1') then
      DynGeneration_State <= DynIdle_1st;
      DynGeneration_HalfPeriod <= '0';
    elsif (Clk'event) then
      if (DynamicMode = '0') then
        DynGeneration_State <= DynIdle_1st;
        DynGeneration_HalfPeriod <= '0';
      elsif ((Strobe25 = '1') and (DynamicSpeed2X = '1')) or (Strobe12p5 = '1') then
        case (DynGeneration_State) is

          --
          when DynIdle_1st =>
            if (IntervalCounterZero = '1') then
              DynGeneration_State <= Dyn_1300Hz_going_down;
              DynGeneration_HalfPeriod <= '0';
            end if;

          --
          when Dyn_1300Hz_going_down =>
            if (IntervalCounterZero = '1') then
              if (DynGeneration_HalfPeriod = '0') then
                DynGeneration_State <= DynIdle_1300Hz_low;
              else
                DynGeneration_State <= Dyn_1300Hz_going_up;
              end if;
              DynGeneration_HalfPeriod <= DynGeneration_HalfPeriod;
            end if;

          --
          when DynIdle_1300Hz_low =>
            if (IntervalCounterOvf = '1') then
              DynGeneration_State <= Dyn_1300Hz_going_up;
              DynGeneration_HalfPeriod <= '0';
            end if;

          --
          when Dyn_1300Hz_going_up =>
            if (IntervalCounterOvf = '1') then
              if (DynGeneration_HalfPeriod = '0') then
                DynGeneration_State <= DynIdle_2nd;
              else
                DynGeneration_State <= Dyn_2100Hz_going_down;
              end if;
              DynGeneration_HalfPeriod <= DynGeneration_HalfPeriod;
            end if;

          --
          when DynIdle_2nd =>
            if (IntervalCounterZero = '1') then
              DynGeneration_State <= Dyn_2100Hz_going_down;
              DynGeneration_HalfPeriod <= '0';
            end if;

          --
          when Dyn_2100Hz_going_down =>
            if (IntervalCounterZero = '1') then
              if (DynGeneration_HalfPeriod = '0') then
                DynGeneration_State <= DynIdle_2100Hz_low;
              else
                DynGeneration_State <= Dyn_2100Hz_going_up;
              end if;
              DynGeneration_HalfPeriod <= DynGeneration_HalfPeriod;
            end if;

          --
          when DynIdle_2100Hz_low =>
            if (IntervalCounterOvf = '1') then
              DynGeneration_State <= Dyn_2100Hz_going_up;
              DynGeneration_HalfPeriod <= '0';
            end if;

          --
          when Dyn_2100Hz_going_up =>
            if (IntervalCounterOvf = '1') then
              if (DynGeneration_HalfPeriod = '0') then
                DynGeneration_State <= Dyn_1300Hz_going_down;
              else
                DynGeneration_State <= DynIdle_1st;
              end if;
              DynGeneration_HalfPeriod <= not DynGeneration_HalfPeriod; -- Сигнал инвертируется и после первого прохода запускается второй, а после второго первый.
            end if;
          --
        end case;
      end if;
    end if;
  end process;

  -- Буфер для значения, подаваемого на ЦАП.
  -- Буферы для значений текущих амплитуд сигналов 1300 Гц и 2100 Гц подаваемых на схему индикации.
  process (Res, Clk)
  begin
    if (Res = '1') then
      DACBuffer <= (others => '0');
      DynamicAmp1300Buffer <= (others => '0');
      DynamicAmp2100Buffer <= (others => '0');
    elsif (Clk'event) then
      if (DynamicMode = '0') then
        -- Работа в статическом режиме.
        if (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
          DACBuffer <= StaticAmp1300Bus;
        elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
          DACBuffer <= StaticAmp2100Bus;
        else
          DACBuffer <= (others => '0');
        end if;
        DynamicAmp1300Buffer <= StaticAmp1300Bus;
        DynamicAmp2100Buffer <= StaticAmp2100Bus;
      else
        -- Работа в динамическом режиме.
        case (DynGeneration_State) is
          --
          when DynIdle_1st =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '1');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '1');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '1');
            DynamicAmp2100Buffer <= (others => '1');
          --
          when Dyn_1300Hz_going_down =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= IntervalCounter;
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '1');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= IntervalCounter;
            DynamicAmp2100Buffer <= (others => '1');
          --
          when DynIdle_1300Hz_low =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '0');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '1');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '0');
            DynamicAmp2100Buffer <= (others => '1');
          --
          when Dyn_1300Hz_going_up =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= IntervalCounter;
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '1');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= IntervalCounter;
            DynamicAmp2100Buffer <= (others => '1');
          --
          when DynIdle_2nd =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '1');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '1');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '1');
            DynamicAmp2100Buffer <= (others => '1');
          --
          when Dyn_2100Hz_going_down =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '1');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= IntervalCounter;
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '1');
            DynamicAmp2100Buffer <= IntervalCounter;
          --
          when DynIdle_2100Hz_low =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '1');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= (others => '0');
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '1');
            DynamicAmp2100Buffer <= (others => '0');
          --
          when Dyn_2100Hz_going_up =>
            if    (Gate1300_Adjusted = '1') and (Gate2100_Adjusted = '0') then
              DACBuffer <= (others => '1');
            elsif (Gate1300_Adjusted = '0') and (Gate2100_Adjusted = '1') then
              DACBuffer <= IntervalCounter;
            else
              DACBuffer <= (others => '0');
            end if;
            DynamicAmp1300Buffer <= (others => '1');
            DynamicAmp2100Buffer <= IntervalCounter;
          --
        end case;
      end if;
    end if;
  end process;

  -- Выдача амплитуд сигналов 1300 Гц и 2100 на ЦАП.
  DACBus <= DACBuffer;

  -- Выдача текущих амплитуд сигналов 1300 Гц и 2100 на схему индикации.
  DynamicAmp1300Bus <= DynamicAmp1300Buffer;
  DynamicAmp2100Bus <= DynamicAmp2100Buffer;

end ArcDynamicPRMG;
