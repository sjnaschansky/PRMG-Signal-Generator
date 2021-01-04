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
-- Имитатор сигналов ПРМГ в статическом и динамическом режимах на плате
-- CoolRunner-II CPLD Starter Board от Digilent.

-- ================================================================================

-- Сигнал Res введён только для возможности симуляции в ISim.

entity PRMG_SigGen is
  generic (
    FracDividerWidth : Natural := 10; -- Разрядность дробного делителя.
    AuxDividerWidth : Natural := 7; -- Разрядность счётчика дополнительного деления частоты на 125.
    AuxDividerRatio : Natural := 125; -- Коэффициент пересчёта счётчика дополнительного деления частоты на 125.
    DualModeDividerWidth : Natural := 5; -- Разрядность двухрежимного счётчика-делителя.
    CycleCounterWidth : Natural := 8; -- Разрядность счётчик подсчёта циклов формирования сигнала ПРМГ.
    --
    SignalBits : Natural := 8; -- Разрядность обрабатываемых сигналов.
    --
    UserInterfaceAuxCtrWidth : Natural := 5; -- Разрядность вспомогательного счётчика получения частот пользовательского интерфейса.
    ButtonTimeOutCtrWidth : Natural := 8; -- Разрядность счётчика формирователя задержки для обработки сигналов кнопки.
    SyncCounterWidth : Natural := 6; -- Разрядность счётчика формирователя сигнала синхронизации.
    SPITransmissionStateCounterWidth : Natural := 6 -- Разрядность счётчика передачи данных по SPI.
  );
  port (
    -- Вход тактового сигнала.
    Clk : in STD_LOGIC;
    -- Входы переключателей.
    SW0, SW1 : in STD_LOGIC; -- Переключатели задают режим работы имитатора.
    -- Входы кнопок.
    BTN0, BTN1 : in STD_LOGIC; -- Кнопки используются для установки амплитуд сигналов 1300 Гц и 2100 Гц.
    -- Выход сигнала синхронизации и дополнительный выход для этого же сигнала.
    OSC_Sync : out STD_LOGIC;
    OSC_Sync_Copy : out STD_LOGIC;
    -- Выход к первому модулю ЦАП.
    DAC12_Clk : out STD_LOGIC;
    DAC12_nLE_R : out STD_LOGIC;
    DAC1_Data_R, DAC2_Data_R : out STD_LOGIC;
    -- Дополнительные выводы, к которым может быть подключен первый модуль ЦАП.
    DAC12_Clk_Copy : out STD_LOGIC;
    DAC12_nLE_R_Copy : out STD_LOGIC;
    DAC1_Data_R_Copy, DAC2_Data_R_Copy : out STD_LOGIC;
    -- Выход ко второму модулю ЦАП.
    DAC34_Clk : out STD_LOGIC;
    DAC34_nLE_R : out STD_LOGIC;
    DAC3_Data_R, DAC4_Data_R : out STD_LOGIC;
    -- Дополнительные выводы, к которым может быть подключен второй модуль ЦАП.
    DAC34_Clk_Copy : out STD_LOGIC;
    DAC34_nLE_R_Copy : out STD_LOGIC;
    DAC3_Data_R_Copy, DAC4_Data_R_Copy : out STD_LOGIC;
    -- Выход к третьему модулю ЦАП.
    DAC56_Clk : out STD_LOGIC;
    DAC56_nLE_R : out STD_LOGIC;
    DAC5_Data_R, DAC6_Data_R : out STD_LOGIC;
    -- Дополнительные выводы, к которым может быть подключен третий модуль ЦАП.
    DAC56_Clk_Copy : out STD_LOGIC;
    DAC56_nLE_R_Copy : out STD_LOGIC;
    DAC5_Data_R_Copy, DAC6_Data_R_Copy : out STD_LOGIC;
    -- Выход к четвёртому модулю ЦАП.
    DAC78_Clk : out STD_LOGIC;
    DAC78_nLE_R : out STD_LOGIC;
    DAC7_Data_R, DAC8_Data_R : out STD_LOGIC;
    -- Дополнительные выводы, к которым может быть подключен четвёртый модуль ЦАП.
    DAC78_Clk_Copy : out STD_LOGIC;
    DAC78_nLE_R_Copy : out STD_LOGIC;
    DAC7_Data_R_Copy, DAC8_Data_R_Copy : out STD_LOGIC;
    -- Выходы к светодиодам.
    LD0, LD1, LD2, LD3 : out STD_LOGIC;
    -- Семисегментный индикатор.
    -- Аноды.
    Digit0, Digit1, Digit2, Digit3 : out STD_LOGIC;
    -- Катоды.
    CA, CB, CC, CD, CE, CF, CG, DP : out STD_LOGIC
  );
end PRMG_SigGen;

architecture ArcPRMG_SigGen of PRMG_SigGen is

-- ==================== Сигнал сброса. В проекте сброс не используется, но сам сигнал предусмотрен в компонентах. ====================
signal Res : STD_LOGIC := '0'; -- Для удобства симуляции.

-- ==================== Выходные сигналы компонента DynamicPRMG. ====================
-- Шина неупакованного сигнала ПРМГ.
signal DACBus : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
-- Шина амплитуды сигнала 1300 Гц для схемы индикации.
signal DynamicAmp1300Bus : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
-- Шина амплитуды сигнала 2100 Гц для схемы индикации.
signal DynamicAmp2100Bus : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);

-- Стробы 54.6 КГц.
signal Strobe54K6 : STD_LOGIC;
-- Стробы 12.5 Гц.
signal Strobe12p5 : STD_LOGIC;

-- ==================== Всё, что касается вспомогательного счётчика получения частот пользовательского интерфейса. ====================
-- Вспомогательный счётчик получения частот пользовательского интерфейса.
signal UserInterfaceAuxiliaryCounter : STD_LOGIC_VECTOR ((UserInterfaceAuxCtrWidth - 1) downto 0) := (others => '0'); -- Для удобства симуляции.
-- Признак достижения максимума для счётчика со сдвигом на такт.
signal UserInterfaceAuxiliaryCounterOverflow : STD_LOGIC;

-- ==================== Всё, что касается обработки сигналов от кнопок и переключателей. ====================
-- Буферы для сигналов переключателей.
signal SW0_Buf, SW1_Buf : STD_LOGIC;
-- Буферы для сигналов кнопок.
signal BTN0_Buf, BTN1_Buf : STD_LOGIC;
-- Счётчик формирователь задержки при обработке сигналов кнопки.
signal ButtonTimeOutCounter : STD_LOGIC_VECTOR ((ButtonTimeOutCtrWidth - 1) downto 0) := (others => '0'); -- Для удобства симуляции.

-- ==================== Всё, что касается регистров установки статических амплитуд 1300 Гц и 2100 Гц. ====================
-- Регистр амплитуды сигнала 1300 Гц для статического режима.
signal StaticAmp1300Reg : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
-- Регистр амплитуды сигнала 2100 Гц для статического режима.
signal StaticAmp2100Reg : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);

-- ==================== Входные управляющие сигналы компонента DynamicPRMG. ====================
-- Сигнал включения динамического режима.
signal DynamicMode : STD_LOGIC;
-- Сигнал включения удвоенной скорости в динамическом режиме.
signal DynamicSpeed2X : STD_LOGIC;

-- ==================== Сигналы для работы с семисегментным индикатором. ====================
-- Сигналы выбора отображаемого разряда.
signal Sel_0, Sel_1 : STD_LOGIC;
-- Отображаемый в текущий момент 16-ричный символ.
signal BinBus : STD_LOGIC_VECTOR (3 downto 0);

-- ==================== Всё, относящееся к формирователю синхросигнала. ====================
-- Счётчик формирователь сигнала синхронизации.
signal SyncCounter : STD_LOGIC_VECTOR ((SyncCounterWidth - 1) downto 0);
-- Сформированный сигнал синхронизации.
signal SyncSignal : STD_LOGIC;

-- ==================== Всё, относящееся к передатчику данных в ЦАПы по шине SPI. ====================
-- Счётчик передачи данных.
signal SPITransmissionStateCounter : STD_LOGIC_VECTOR ((SPITransmissionStateCounterWidth - 1) downto 0);
-- Буфер для значения, подаваемого на ЦАП.
signal DACBuffer : STD_LOGIC_VECTOR ((SignalBits - 1) downto 0);
-- Тактовый сигнал шины SPI.
signal SPI_CLK : STD_LOGIC;
-- Сигнал nCS шины SPI.
signal SPI_nCS : STD_LOGIC;
-- Передаваемый бит данных с упреждением на 1 такт работы схемы.
signal TXBit_m1T : STD_LOGIC;
-- Сигнал данных шины SPI, он же передаваемый бит данных без упреждения.
signal SPI_DATA : STD_LOGIC;

-- Максимальное разрешённое значение счётчика передачи данных.
constant SPITransmissionStateCounterMax : Integer := 34 - 1;

-- Признак несовпадения данных на выходе генератора и в буфере передачи.
signal Comparator : STD_LOGIC;
-- Признак нуля счётчика передачи.
signal SPITransmissionStateCounterZero : STD_LOGIC;
-- Передаваемый бит данных с упреждением на 2 такта работы схемы.
signal TXBit_m2T : STD_LOGIC;

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
end component;

begin

  -- Сброс всегда неактивен.
  Res <= '0';

-- ==================== Подключение компонента DynamicPRMG. ====================

  Lab0_DynamicPRMG : DynamicPRMG
    generic map (
      FracDividerWidth => FracDividerWidth,
      AuxDividerWidth => AuxDividerWidth,
      AuxDividerRatio => AuxDividerRatio,
      DualModeDividerWidth => DualModeDividerWidth,
      CycleCounterWidth => CycleCounterWidth,
      --
      SignalBits => SignalBits)
    port map (
      Res => Res,
      Clk => Clk,
      DynamicMode => DynamicMode,
      DynamicSpeed2X => DynamicSpeed2X,
      StaticAmp1300Bus => StaticAmp1300Reg,
      StaticAmp2100Bus => StaticAmp2100Reg,
      DACBus => DACBus, -- Сам сигнал ПРМГ. Все сигналы ниже - вспомогательные, для непосредственно генерации не нужны.
      DynamicAmp1300Bus => DynamicAmp1300Bus, -- Амплитуда используется только в схеме индикации.
      DynamicAmp2100Bus => DynamicAmp2100Bus, -- Амплитуда используется только в схеме индикации.
      Strobe54K6Out => Strobe54K6, -- Эти стробы используются только ради экономии ресурсов, не нужно создавать ещё один делитель тактовой частоты.
      Strobe25Out => open, -- Эти стробы не используются.
      Strobe12p5Out => Strobe12p5); -- Эти стробы используются только для удобства синхронизации осциллографа при просмотре генерируемого сигнала.

-- ==================== Подраздел интерфейса пользователя. ====================

  -- Вспомогательный счётчик получения частот пользовательского интерфейса, входной частотой счётчика является частота 54.6 кГц.
  -- Признак достижения максимума для счётчика со сдвигом на такт.
  -- Частоты используются для регенерации индикатора и опроса кнопок и переключателей.
  process (Clk)
  begin
    if (Clk'event) then
      if (Strobe54K6 = '1') then
        UserInterfaceAuxiliaryCounter <= UserInterfaceAuxiliaryCounter + 1;
        if (UserInterfaceAuxiliaryCounter = (UserInterfaceAuxiliaryCounter'Range => '1')) then
          UserInterfaceAuxiliaryCounterOverflow <= '1';
        else
          UserInterfaceAuxiliaryCounterOverflow <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Буферизация сигналов переключателей и кнопок,
  -- счётчик формирователь задержки при обработке сигналов кнопки,
  -- регистры установки статической амплитуды 1300 Гц и 2100 Гц.
  process (Clk)
  begin
    if (Clk'event) then
      if (Strobe54K6 = '1') and (UserInterfaceAuxiliaryCounterOverflow = '1') then
        --
        SW0_Buf <= SW0; -- Буферизация.
        SW1_Buf <= SW1; -- ->>-
        BTN0_Buf <= BTN0; -- ->>-
        BTN1_Buf <= BTN1; -- ->>-
        --
        if (BTN0_Buf = '0') and (BTN1_Buf = '1') then -- Обработка случая: правая кнопка (BTN0) нажата, левая (BTN1) отжата.
          --
          if (SW1_Buf = '1') then -- Обработка случая: левый переключатель (SW1) в левом положении - включен статический режим.
            ButtonTimeOutCounter <= ButtonTimeOutCounter + 1; -- В статическом режиме кнопки используются для установки амплитуд.
            if (ButtonTimeOutCounter = (ButtonTimeOutCounter'Range => '1')) then
              if (SW0_Buf = '1') then -- Обработка случая: правый переключатель (SW0) в левом положении - инкремент амплитуды сигнала 1300 Гц с насыщением.
                if (StaticAmp1300Reg /= (StaticAmp1300Reg'Range => '1')) then
                  StaticAmp1300Reg <= StaticAmp1300Reg + 1;
                end if;
              else -- Обработка случая: правый переключатель (SW0) в правом положении - инкремент амплитуды сигнала 2100 Гц с насыщением.
                if (StaticAmp2100Reg /= (StaticAmp2100Reg'Range => '1')) then
                  StaticAmp2100Reg <= StaticAmp2100Reg + 1;
                end if;
              end if;
            end if;
          else -- Обработка случая: левый переключатель (SW1) в правом положении - включен динамический режим.
            ButtonTimeOutCounter <= (others => '0'); -- В динамическом режиме кнопки не используются, установленные амплитуды статического режима сохраняются.
          end if;
          --
        elsif (BTN0_Buf = '1') and (BTN1_Buf = '0') then -- Обработка случая: правая кнопка (BTN0) отжата, левая (BTN1) нажата.
          --
          if (SW1_Buf = '1') then -- Обработка случая: левый переключатель (SW1) в левом положении - включен статический режим.
            ButtonTimeOutCounter <= ButtonTimeOutCounter + 1; -- В статическом режиме кнопки используются для установки амплитуд.
            if (ButtonTimeOutCounter = (ButtonTimeOutCounter'Range => '1')) then
              if (SW0_Buf = '1') then -- Обработка случая: правый переключатель (SW0) в левом положении - декремент амплитуды сигнала 1300 Гц с насыщением.
                if (StaticAmp1300Reg /= (StaticAmp1300Reg'Range => '0')) then
                  StaticAmp1300Reg <= StaticAmp1300Reg - 1;
                end if;
              else -- Обработка случая: правый переключатель (SW0) в правом положении - декремент амплитуды сигнала 2100 Гц с насыщением.
                if (StaticAmp2100Reg /= (StaticAmp2100Reg'Range => '0')) then
                  StaticAmp2100Reg <= StaticAmp2100Reg - 1;
                end if;
              end if;
            end if;
          else -- Обработка случая: левый переключатель (SW1) в правом положении - включен динамический режим.
            ButtonTimeOutCounter <= (others => '0'); -- В динамическом режиме кнопки не используются, установленные амплитуды статического режима сохраняются.
          end if;
          --
        else -- Обработка случая: правая (BTN0) и левая (BTN1) кнопки одновременно отжаты или нажаты.
          ButtonTimeOutCounter <= (others => '0'); -- Никакие действия не выполняются.
        end if;
        --
      end if;
    end if;
  end process;

  -- Управление компонентом DynamicPRMG.

  -- Сигнал включения динамического режима.
  DynamicMode <= (not SW1_Buf); -- Левый переключатель (SW1) в правом положении.
  -- Сигнал включения удвоенной скорости в динамическом режиме.
  DynamicSpeed2X <= (not SW0_Buf) and (not SW1_Buf); -- Правый (SW0) левый (SW1) переключатели в правом положении.

  -- Индикация на светодиодах.
  LD0 <= not (ButtonTimeOutCounter (ButtonTimeOutCtrWidth - 1) and ButtonTimeOutCounter (ButtonTimeOutCtrWidth - 2) and (not BTN0_Buf) and BTN1_Buf);
  LD1 <= not (ButtonTimeOutCounter (ButtonTimeOutCtrWidth - 1) and ButtonTimeOutCounter (ButtonTimeOutCtrWidth - 2) and BTN0_Buf and (not BTN1_Buf));
  LD2 <= SW0_Buf;
  LD3 <= SW1_Buf;

  -- Индикация на семисегментном индикаторе.

  -- Сигналы выбора активного разряда индикатора.
  Sel_0 <= UserInterfaceAuxiliaryCounter (UserInterfaceAuxCtrWidth - 2);
  Sel_1 <= UserInterfaceAuxiliaryCounter (UserInterfaceAuxCtrWidth - 1);

  -- Последовательный перебор анодов индикатора.
  Digit0 <= '0' when (Sel_1 = '0') and (Sel_0 = '0') else '1';
  Digit1 <= '0' when (Sel_1 = '0') and (Sel_0 = '1') else '1';
  Digit2 <= '0' when (Sel_1 = '1') and (Sel_0 = '0') else '1';
  Digit3 <= '0' when (Sel_1 = '1') and (Sel_0 = '1') else '1';

  -- Выбор одного 16-ричного символа для отображения на индикаторе.
  BinBus <= DynamicAmp2100Bus (3 downto 0)                                                                when (Sel_1 = '0') and (Sel_0 = '0') else
            DynamicAmp2100Bus (7) & DynamicAmp2100Bus (6) & DynamicAmp2100Bus (5) & DynamicAmp2100Bus (4) when (Sel_1 = '0') and (Sel_0 = '1') else
            DynamicAmp1300Bus (3 downto 0)                                                                when (Sel_1 = '1') and (Sel_0 = '0') else
            DynamicAmp1300Bus (7) & DynamicAmp1300Bus (6) & DynamicAmp1300Bus (5) & DynamicAmp1300Bus (4);

  -- Получение кодов для семисегментного индикатора.
  process (BinBus)
  begin
    case (BinBus) is
      when "0000" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '1'; DP <= '1';
      when "0001" => CA <= '1'; CB <= '0'; CC <= '0'; CD <= '1'; CE <= '1'; CF <= '1'; CG <= '1'; DP <= '1';
      when "0010" => CA <= '0'; CB <= '0'; CC <= '1'; CD <= '0'; CE <= '0'; CF <= '1'; CG <= '0'; DP <= '1';
      when "0011" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '0'; CE <= '1'; CF <= '1'; CG <= '0'; DP <= '1';
      when "0100" => CA <= '1'; CB <= '0'; CC <= '0'; CD <= '1'; CE <= '1'; CF <= '0'; CG <= '0'; DP <= '1';
      when "0101" => CA <= '0'; CB <= '1'; CC <= '0'; CD <= '0'; CE <= '1'; CF <= '0'; CG <= '0'; DP <= '1';
      when "0110" => CA <= '0'; CB <= '1'; CC <= '0'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
      when "0111" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '1'; CE <= '1'; CF <= '1'; CG <= '1'; DP <= '1';
      when "1000" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
      when "1001" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '0'; CE <= '1'; CF <= '0'; CG <= '0'; DP <= '1';
      when "1010" => CA <= '0'; CB <= '0'; CC <= '0'; CD <= '1'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
      when "1011" => CA <= '1'; CB <= '1'; CC <= '0'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
      when "1100" => CA <= '0'; CB <= '1'; CC <= '1'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '1'; DP <= '1';
      when "1101" => CA <= '1'; CB <= '0'; CC <= '0'; CD <= '0'; CE <= '0'; CF <= '1'; CG <= '0'; DP <= '1';
      when "1110" => CA <= '0'; CB <= '1'; CC <= '1'; CD <= '0'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
      when others => CA <= '0'; CB <= '1'; CC <= '1'; CD <= '1'; CE <= '0'; CF <= '0'; CG <= '0'; DP <= '1';
    end case;
  end process;

-- ==================== Подраздел генератора сигнала. ====================

  -- Формирование синхросигнала. Синхросигнал сделан просто для удобства просмотра генерируемого сигнала на осциллографе.

  -- Счётчик формирователь сигнала синхронизации.
  -- Сформированный сигнал синхронизации.
  process (Clk)
  begin
    if (Clk'event) then
      if (Strobe12p5 = '1') then
        SyncCounter <= (others => '1');
        SyncSignal <= '1';
      else
        if (SyncCounter /= (SyncCounter'Range => '0')) then
          SyncCounter <= SyncCounter - 1;
        else
          SyncSignal <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Подача сигнала синхронизации на выводы.
  OSC_Sync <= SyncSignal;
  OSC_Sync_Copy <= SyncSignal;

  -- Преобразование данных из параллельного представления в последовательное и передача их по интерфейсу SPI в ЦАП.

  -- Схема передачи работает следующим образом.
  -- Два раза за такт на схему поступает новое значение по параллельной шине, это значение сравнивается с последним переданным по интерфейсу SPI в ЦАП.
  -- Если значения совпали, то ничего не происходит, никаких данных в ЦАП не передаётся, т.к. в ЦАП находится актуальное значение.
  -- Если же новое значение отличается от предыдущего, то оно записывается в буфер передачи и начинается передача новых данных по интерфейсу SPI.
  -- Полный цикл передачи занимает 17 тактов, соответственно через 17 тактов схема вернётся в исходное состояние.
  -- Таким образом, если на входе схемы обновление данных происходит чаще, чем за 17 тактов, данные будут теряться.
  -- Но с другой стороны в сигнале ПРМГ данные изменяются относительно редко, не чаще, чем с частотой 4100 Гц, соответственно потерь данных не будет.
  -- Максимальная частота обновления данных может достигать величины 8 МГц / 17 = 470.588 КГц.

  -- Счётчик передачи данных.
  -- Буфер для значения, подаваемого на ЦАП.
  -- Тактовый сигнал шины SPI.
  -- Сигнал nCS шины SPI.
  -- Сигнал данных шины SPI с упреждением на 1 такт.
  -- Сигнал данных шины SPI без упреждения.
  process (Clk)
  begin
    if (Clk'event) then
      if (Comparator = '0') and (SPITransmissionStateCounterZero = '1') then -- Цикл ожидания новых данных.
        SPITransmissionStateCounter <= (others => '0');
        DACBuffer <= DACBuffer;
        SPI_CLK <= '0';
        SPI_nCS <= '1';
      elsif (Comparator = '1') and (SPITransmissionStateCounterZero = '1') then -- Перезапись новых данных в регистр передачи и запуск счётчика.
        SPITransmissionStateCounter <= SPITransmissionStateCounter + 1;
        DACBuffer <= DACBus;
        SPI_CLK <= '0';
        SPI_nCS <= '1';
      else
        if (SPITransmissionStateCounter = SPITransmissionStateCounterMax) then -- Как только счётчик достиг максимума, переход в исходное состояние.
          SPITransmissionStateCounter <= (others => '0');
          DACBuffer <= DACBuffer;
          SPI_CLK <= not SPITransmissionStateCounter (0);
          SPI_nCS <= '1';
        else -- Передача данных.
          SPITransmissionStateCounter <= SPITransmissionStateCounter + 1;
          DACBuffer <= DACBuffer;
          SPI_CLK <= not SPITransmissionStateCounter (0);
          SPI_nCS <= '0';
        end if;
      end if;
      TXBit_m1T <= TXBit_m2T; -- Сигнал с индексом _m1T с упреждением 1 такт.
      SPI_DATA <= TXBit_m1T; -- Сигнал SPI_DATA без упреждения.
    end if;
  end process;

  -- Признак несовпадения данных на выходе генератора и в буфере передачи.
  Comparator <= '1' when (DACBus /= DACBuffer) else '0';

  -- Признак нуля счётчика передачи.
  SPITransmissionStateCounterZero <= '1' when (SPITransmissionStateCounter = (SPITransmissionStateCounter'Range => '0')) else '0';

  -- Передаваемый бит данных, ниже он задаётся с упреждением на 2 такта работы схемы.
  TXBit_m2T <= '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  32) or (CONV_INTEGER (SPITransmissionStateCounter) = 33)) else -- В то время, когда nCS = 1, TXBit = 0.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  0) or (CONV_INTEGER (SPITransmissionStateCounter) =  1)) else -- Бит nINT/EXT ЦАП. Этот же бит передаётся во время ожидания.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  2) or (CONV_INTEGER (SPITransmissionStateCounter) =  3)) else -- ЦАП не использует этот бит.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  4) or (CONV_INTEGER (SPITransmissionStateCounter) =  5)) else -- Бит LDAC ЦАП.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  6) or (CONV_INTEGER (SPITransmissionStateCounter) =  7)) else -- Бит PDB ЦАП.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) =  8) or (CONV_INTEGER (SPITransmissionStateCounter) =  9)) else -- Бит PDA ЦАП.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) = 10) or (CONV_INTEGER (SPITransmissionStateCounter) = 11)) else -- Бит nA/B ЦАП.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) = 12) or (CONV_INTEGER (SPITransmissionStateCounter) = 13)) else -- Бит CR1 ЦАП.
               '0' when ((CONV_INTEGER (SPITransmissionStateCounter) = 14) or (CONV_INTEGER (SPITransmissionStateCounter) = 15)) else -- Бит CR0 ЦАП.
               DACBuffer (7) when ((CONV_INTEGER (SPITransmissionStateCounter) = 16) or (CONV_INTEGER (SPITransmissionStateCounter) = 17)) else
               DACBuffer (6) when ((CONV_INTEGER (SPITransmissionStateCounter) = 18) or (CONV_INTEGER (SPITransmissionStateCounter) = 19)) else
               DACBuffer (5) when ((CONV_INTEGER (SPITransmissionStateCounter) = 20) or (CONV_INTEGER (SPITransmissionStateCounter) = 21)) else
               DACBuffer (4) when ((CONV_INTEGER (SPITransmissionStateCounter) = 22) or (CONV_INTEGER (SPITransmissionStateCounter) = 23)) else
               DACBuffer (3) when ((CONV_INTEGER (SPITransmissionStateCounter) = 24) or (CONV_INTEGER (SPITransmissionStateCounter) = 25)) else
               DACBuffer (2) when ((CONV_INTEGER (SPITransmissionStateCounter) = 26) or (CONV_INTEGER (SPITransmissionStateCounter) = 27)) else
               DACBuffer (1) when ((CONV_INTEGER (SPITransmissionStateCounter) = 28) or (CONV_INTEGER (SPITransmissionStateCounter) = 29)) else
               DACBuffer (0) when ((CONV_INTEGER (SPITransmissionStateCounter) = 30) or (CONV_INTEGER (SPITransmissionStateCounter) = 31)) else '0';

  -- Сигналы для модулей ЦАП. Во все ЦАПы всех модулей передаются одни и те же данные.

  -- Сигналы к первому модулю ЦАП.
  DAC12_Clk <= SPI_CLK;
  DAC12_nLE_R <= SPI_nCS;
  DAC1_Data_R <= SPI_DATA;
  DAC2_Data_R <= SPI_DATA;
  -- Их копия.
  DAC12_Clk_Copy <= SPI_CLK;
  DAC12_nLE_R_Copy <= SPI_nCS;
  DAC1_Data_R_Copy <= SPI_DATA;
  DAC2_Data_R_Copy <= SPI_DATA;

  -- Сигналы ко второму модулю ЦАП.
  DAC34_Clk <= SPI_CLK;
  DAC34_nLE_R <= SPI_nCS;
  DAC3_Data_R <= SPI_DATA;
  DAC4_Data_R <= SPI_DATA;
  -- Их копия.
  DAC34_Clk_Copy <= SPI_CLK;
  DAC34_nLE_R_Copy <= SPI_nCS;
  DAC3_Data_R_Copy <= SPI_DATA;
  DAC4_Data_R_Copy <= SPI_DATA;

  -- Сигналы к третьему модулю ЦАП.
  DAC56_Clk <= SPI_CLK;
  DAC56_nLE_R <= SPI_nCS;
  DAC5_Data_R <= SPI_DATA;
  DAC6_Data_R <= SPI_DATA;
  -- Их копия.
  DAC56_Clk_Copy <= SPI_CLK;
  DAC56_nLE_R_Copy <= SPI_nCS;
  DAC5_Data_R_Copy <= SPI_DATA;
  DAC6_Data_R_Copy <= SPI_DATA;

  -- Сигналы к четвёртому модулю ЦАП.
  DAC78_Clk <= SPI_CLK;
  DAC78_nLE_R <= SPI_nCS;
  DAC7_Data_R <= SPI_DATA;
  DAC8_Data_R <= SPI_DATA;
  -- Их копия.
  DAC78_Clk_Copy <= SPI_CLK;
  DAC78_nLE_R_Copy <= SPI_nCS;
  DAC7_Data_R_Copy <= SPI_DATA;
  DAC8_Data_R_Copy <= SPI_DATA;

end ArcPRMG_SigGen;
