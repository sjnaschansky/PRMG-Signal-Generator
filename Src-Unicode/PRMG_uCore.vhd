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
-- Микроядро имитатора посадочных радиомаяков системы ПРМГ.
-- Тактовая частота на входе - 8 МГц, частота переключения регистров в 2 раза выше, т.е. 16 МГц.
-- Из 16 МГц частоты 1300 и 2100 Гц нельзя получить простым делением, необходим дробный делитель.
-- Для выработки обеих частот сигнала ПРМГ необходимо получение частоты 54.6 кГц.
-- 54600 = 2 * 3 * 7 * 13 * 100
-- 16000000 = 54600 * 80000 / (3 * 7 * 13)
-- т.е. коэффициент пересчёта дробного делителя должен быть (3 * 7 * 13) / 80000.
-- 3 * 7 * 13 = 273, знаменатель дробного делителя не может быть меньше числителя,
-- т.е. должен превышать 273.
-- Простейший вариант - построить дробный делитель с коэффициентом 273 / 640, и дополнить его
-- обычным делителем с коэффициентом пересчёта 1 / 125.
-- В принципе можно также использовать делитель 273 / 320 с дополнительным делением на 250.
-- Знаменатель дробного делителя не является степенью двойки, что усложнит дробный делитель.
-- Но с другой стороны 640 = 128 * 5 = 2^7 * 5. Такое удачное разбиение знаменателя на множители
-- не приводит к существенному усложнению дробного делителя.
-- В итоге у регистра дробного делителя по специальному алгоритму корректируется только 3
-- старших бита. Оставшиеся 7 младших битов нет необходимости корректировать.
-- По сути если значение в дробном делителе превышает 639, то 3 старших разряда
-- принимают значения 101, 110 и 111. Остальные разряды анализировать не требуется,
-- в дополнение их не требуется и корректировать.

-- Рассмотрение различных вариантов построения дробного делителя приведено в файле
-- PRMG_FracDividersTest.mcd, наилучший результат достигается одноступенчатым дробным делителем.

-- ================================================================================

-- Сигнал Res введён только для возможности симуляции в ISim.

entity PRMG_uCore is
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
end PRMG_uCore;

architecture ArcPRMG_uCore of PRMG_uCore is

-- ==================== Всё, что касается дробного делителя. ====================
-- Счётчик дробного делителя частоты.
signal FracDivider : STD_LOGIC_VECTOR ((FracDividerWidth - 1) downto 0);
-- Выход дробного делителя.
signal FracDividerOut : STD_LOGIC;

-- Константа, определяющая числитель в дроби коэффициента пересчёта дробного делителя.
signal FracDividerStep : Integer := 273;
-- Знаменателем этой дроби является число 640.

-- ==================== Всё, что касается дополнительного делителя. ====================
-- Счётчик дополнительного делителя частоты.
signal AuxDivider : STD_LOGIC_VECTOR ((AuxDividerWidth - 1) downto 0);
-- Признак переполнения счётчика дополнительного делителя частоты.
signal AuxDividerOvf : STD_LOGIC;

-- Лимит для счётчика дополнительного делителя частоты.
constant AuxDividerMax : Integer := AuxDividerRatio - 1;

-- ==================== Базовая частота 54.6 КГц. ====================
-- Стробы Strobe_54K6 для управления тактированием тракта посадки.
signal Strobe_54K6 : STD_LOGIC;
-- Стробы Strobe_54K6 с задержкой на 1 такт работы схемы.
-- Эта задержка нужна для обеспечения взаимной синхронности стробов Strobe54K6, Strobe25 и Strobe12p5.
signal Strobe_54K6_Adjusted : STD_LOGIC;

-- ==================== Всё, что касается двухрежимного счётчика. ====================
-- Двухрежимный счётчик-делитель частоты.
signal DualModeDivider : STD_LOGIC_VECTOR ((DualModeDividerWidth - 1) downto 0);
-- Признак переполнения счётчика двухрежимного делителя.
signal DualModeDividerOvf : STD_LOGIC;
-- Признак нуля счётчика двухрежимного делителя.
signal DualModeDividerZero : STD_LOGIC;

-- Лимит для счётчика двухрежимного делителя частоты при получении частоты 2600 Гц на первом интервале 5 + 35 = 40 мс.
constant DualModeDividerMax_2600Hz : Integer := 21 - 1;
-- Лимит для счётчика двухрежимного делителя частоты при получении частоты 4200 Гц на втором интервале 5 + 35 = 40 мс.
constant DualModeDividerMax_4200Hz : Integer := 13 - 1;

-- ==================== Всё, что касается счётчика подсчёта циклов формирования сигнала ПРМГ. ====================
-- Счётчик подсчёта циклов формирования сигнала ПРМГ.
signal CycleCounter : STD_LOGIC_VECTOR ((CycleCounterWidth - 1) downto 0);
-- Признак переполнения счётчика подсчёта циклов формирования сигнала ПРМГ.
signal CycleCounterOvf : STD_LOGIC;
-- Признак нуля счётчика подсчёта циклов формирования сигнала ПРМГ.
signal CycleCounterZero : STD_LOGIC;

-- Лимит для счётчика подсчёта циклов формирования сигнала ПРМГ при формировании 1-й паузы 5 мс.
constant CycleCounterMax_5ms_1st : Integer := 13 - 1;
-- Лимит для счётчика подсчёта циклов формирования сигнала ПРМГ при формировании импульсов частоты 1300 Гц на интервале 35 мс.
constant CycleCounterMax_1300Hz_35ms : Integer := 91 - 1;
-- Лимит для счётчика подсчёта циклов формирования сигнала ПРМГ при формировании 2-й паузы 5 мс.
constant CycleCounterMax_5ms_2nd : Integer := 21 - 1;
-- Лимит для счётчика подсчёта циклов формирования сигнала ПРМГ при формировании импульсов частоты 2100 Гц на интервале 35 мс.
constant CycleCounterMax_2100Hz_35ms : Integer := 147 - 1;

-- ==================== Всё, что касается конечного автомата управления генерацией сигнала ПРМГ. ====================
-- Определение набора состояний конечного автомата управления генерацией сигнала ПРМГ.
type SigGen_States is (Idle_5ms_1st, Gen_1300Hz_35ms, Idle_5ms_2nd, Gen_2100Hz_35ms);
-- Определение сигнала для конечного автомата управления генерацией сигнала ПРМГ.
signal SigGen_State : SigGen_States;

-- ==================== Сигналы 1300 Гц и 2100 Гц. ====================
signal Gate_1300, Gate_2100 : STD_LOGIC;

-- ==================== Стробы сигналов 25 Гц и 12.5 Гц. ====================
signal Strobe_25, Strobe_12p5 : STD_LOGIC;

begin

-- ==================== Получение стробов базовой частоты 54.6 КГц. ====================
  -- Дробный делитель частоты.
  -- Выход дробного делителя частоты.
  process (Res, Clk)
  variable FracDividerNew : STD_LOGIC_VECTOR ((FracDividerWidth - 1) downto 0);
  begin
    if (Res = '1') then
      FracDivider <= (others => '0');
      FracDividerOut <= '0';
    elsif (Clk'event) then
      FracDividerNew := FracDivider + FracDividerStep;
      -- Обработка переполнения.
      -- Конструкция ниже появилась из-за неравенства знаменателя степени двойки.
      -- Её суть вычислить остаток от деления значения FracDividerNew на 640.
      if (FracDividerNew ((FracDividerWidth - 1) downto (FracDividerWidth - 3)) = "101") or
         (FracDividerNew ((FracDividerWidth - 1) downto (FracDividerWidth - 3)) = "110") or
         (FracDividerNew ((FracDividerWidth - 1) downto (FracDividerWidth - 3)) = "111") then
        FracDivider <= (FracDividerNew ((FracDividerWidth - 1) downto (FracDividerWidth - 3)) + 3) & FracDividerNew ((FracDividerWidth - 4) downto 0);
        FracDividerOut <= '1';
      else
        FracDivider <= FracDividerNew;
        FracDividerOut <= '0';
      end if;
      --
    end if;
  end process;

  -- Дополнительный делитель частоты.
  process (Res, Clk)
  begin
    if (Res = '1') then
      AuxDivider <= (others => '0');
    elsif (Clk'event) then
      if (FracDividerOut = '1') then
        if (AuxDividerOvf = '1') then
          AuxDivider <= (others => '0');
        else
          AuxDivider <= AuxDivider + 1;
        end if;
      end if;
    end if;
  end process;

  -- Переполнение дополнительного делителя частоты.
  AuxDividerOvf <= '1' when (AuxDivider = AuxDividerMax) else '0';

  -- Получение стробов Strobe_54K6 а также копии этого сигнала с задержкой на 1 такт.
  process (Res, Clk)
  begin
    if (Res = '1') then
      Strobe_54K6 <= '0';
      Strobe_54K6_Adjusted <= '0';
    elsif (Clk'event) then
      Strobe_54K6 <= FracDividerOut and AuxDividerOvf;
      Strobe_54K6_Adjusted <= Strobe_54K6;
    end if;
  end process;

-- ==================== Получение самого сигнала ПРМГ. ====================

  -- Двухрежимный счётчик-делитель частоты.
  process (Res, Clk)
  begin
    if (Res = '1') then
      DualModeDivider <= (others => '0');
    elsif (Clk'event) then
      if (Strobe_54K6 = '1') then
        if (DualModeDividerOvf = '1') then
          DualModeDivider <= (others => '0');
        else
          DualModeDivider <= DualModeDivider + 1;
        end if;
      end if;
    end if;
  end process;

  -- Переполнение двухрежимного счётчика делителя частоты.
  DualModeDividerOvf <= '1' when ((SigGen_State = Idle_5ms_1st) or (SigGen_State = Gen_1300Hz_35ms)) and (DualModeDivider = DualModeDividerMax_2600Hz) else
                        '1' when ((SigGen_State = Idle_5ms_2nd) or (SigGen_State = Gen_2100Hz_35ms)) and (DualModeDivider = DualModeDividerMax_4200Hz) else '0';
  -- Ноль двухрежимного счётчика делителя частоты.
  DualModeDividerZero <= '1' when (DualModeDivider = 0) else '0';

  -- Счётчик подсчёта циклов формирования сигнала ПРМГ.
  process (Res, Clk)
  begin
    if (Res = '1') then
      CycleCounter <= (others => '0');
    elsif (Clk'event) then
      if (Strobe_54K6 = '1') then
        if (DualModeDividerOvf = '1') then
          if (CycleCounterOvf = '1') then
            CycleCounter <= (others => '0');
          else
            CycleCounter <= CycleCounter + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- Переполнение счётчика подсчёта циклов формирования сигнала ПРМГ.
  CycleCounterOvf <= '1' when (SigGen_State = Idle_5ms_1st)    and (CycleCounter = CycleCounterMax_5ms_1st)     else
                     '1' when (SigGen_State = Gen_1300Hz_35ms) and (CycleCounter = CycleCounterMax_1300Hz_35ms) else
                     '1' when (SigGen_State = Idle_5ms_2nd)    and (CycleCounter = CycleCounterMax_5ms_2nd)     else
                     '1' when (SigGen_State = Gen_2100Hz_35ms) and (CycleCounter = CycleCounterMax_2100Hz_35ms) else '0';

  -- Признак нуля счётчика подсчёта циклов формирования сигнала ПРМГ.
  CycleCounterZero <= '1' when (CycleCounter = 0) else '0';

  -- Конечный автомат управления генерацией сигнала ПРМГ.
  process (Res, Clk)
  begin
    if (Res = '1') then
      SigGen_State <= Idle_5ms_1st;
    elsif (Clk'event) then
      if (Strobe_54K6 = '1') then
        if (DualModeDividerOvf = '1') then
          if (CycleCounterOvf = '1') then
            case (SigGen_State) is
              when Idle_5ms_1st =>    SigGen_State <= Gen_1300Hz_35ms;
              when Gen_1300Hz_35ms => SigGen_State <= Idle_5ms_2nd;
              when Idle_5ms_2nd =>    SigGen_State <= Gen_2100Hz_35ms;
              when Gen_2100Hz_35ms => SigGen_State <= Idle_5ms_1st;
            end case;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- Получение сигналов 1300 Гц и 2100 Гц.
  process (Res, Clk)
  begin
    if (Res = '1') then
      Gate_1300 <= '0';
      Gate_2100 <= '0';
    elsif (Clk'event) then
      if (Strobe_54K6 = '1') then
        if (DualModeDividerOvf = '1') then
          case (SigGen_State) is
            --
            when Idle_5ms_1st =>
              if (CycleCounterOvf = '1') then
                Gate_1300 <= not Gate_1300;
              else
                Gate_1300 <= '0';
              end if;
              Gate_2100 <= '0';
            --
            when Gen_1300Hz_35ms =>
              Gate_1300 <= not Gate_1300;
              Gate_2100 <= '0';
            --
            when Idle_5ms_2nd =>
              Gate_1300 <= '0';
              if (CycleCounterOvf = '1') then
                Gate_2100 <= not Gate_2100;
              else
                Gate_2100 <= '0';
              end if;
            --
            when Gen_2100Hz_35ms =>
              Gate_1300 <= '0';
              Gate_2100 <= not Gate_2100;
            --
          end case;
        end if;
      end if;
    end if;
  end process;

  -- Получение стробов 25 Гц и 12.5 Гц.
  process (Res, Clk)
  begin
    if (Res = '1') then
      Strobe_25 <= '0';
      Strobe_12p5 <= '0';
    elsif (Clk'event) then
      if (Strobe_54K6 = '1') and (DualModeDividerOvf = '1') and (CycleCounterOvf = '1') and ((SigGen_State = Gen_1300Hz_35ms) or (SigGen_State = Gen_2100Hz_35ms)) then
        Strobe_25 <= '1';
      else
        Strobe_25 <= '0';
      end if;
      if (Strobe_54K6 = '1') and (DualModeDividerOvf = '1') and (CycleCounterOvf = '1') and (SigGen_State = Gen_2100Hz_35ms) then
        Strobe_12p5 <= '1';
      else
        Strobe_12p5 <= '0';
      end if;
    end if;
  end process;

-- ==================== Выходы. ====================
  Strobe54K6 <= Strobe_54K6_Adjusted;

  Gate1300 <= Gate_1300;
  Gate2100 <= Gate_2100;

  Strobe25 <= Strobe_25;
  Strobe12p5 <= Strobe_12p5;

end ArcPRMG_uCore;
