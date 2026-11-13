# Proteus v2

Сюда — проект симуляции v2 (`.pdsprj`).

**Дисплей в схеме:** SSD1306 (128×64, I2C), адрес **0x3C**. INA219 — **0x40**, SDA/SCL **A4/A5**.

**Прошивка (Program File у Arduino Nano, 16 MHz, ATmega328 old bootloader):**

| Режим ШИМ на D11 | Файл |
|------------------|------|
| **~976 Hz** (основная v2) | `firmware/mast_control_v2.ino.hex` |
| **~244 Hz** (эксперимент CED252) | `firmware/mast_control_v2_PWM244hz.ino.hex` |

Схема одна и та же — в Proteus только укажите нужный `.hex` и перезапустите симуляцию.

**Пересборка PWM244** (из корня репозитория, Arduino CLI из Arduino IDE):

```text
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old ^
  --libraries lib_V2 --output-dir lib_V2/proteus_V2/firmware ^
  mast_control_v2_PWM244hz
```

## Кнопка обнуления петли (D12)

| Схема | Значение |
|--------|-----------|
| Пин | **D12** (Arduino Nano) |
| Нажатие | замыкание **D12 → GND** |
| Подтяжка | **10 kΩ: D12 → +5V** (во многих схемах Proteus внутренний `INPUT_PULLUP` слабый) |
| Удержание | **3 с** (как на железе) → **ZERO SET** |
| Если D12 не разведён | **Virtual Terminal** на **9600**, символ **`Z`** или **`z`** — тот же tare |

На OLED: при нажатии **TARE…** → **ZERO SET** или **TARE FAIL**.

**TARE FAIL:** ток &lt; 3,6 mA, OPEN/SHORT, I2C с INA219.

## Петля 4–20 mA (ИВЭ-50) в симуляции

Пример: **+24 V → Rограничения → RV ≈ 5,2 kΩ** → INA219 (Vin+/Vin−) → GND.

Переменный резистор **~5,2 kΩ** даёт более плавные шаги тока, чем 10 kΩ.

Пины джойстиков/реле — сверять с `mast_control_v2.ino`.

## Если Proteus «тормозит» или сыпет SPICE (GMIN stepping, timestep too small)

Так бывает на **тяжёлой смешанной** схеме (Arduino + OLED I2C + петля 4–20 mA + силовая часть). Сообщение *«Simulation is not running in real time»* само по себе не ошибка — симуляция просто медленнее 1:1.

**Сначала проверьте hex:**

| Проверка | Действие |
|----------|----------|
| Основная v2 ещё стартует? | Верните `mast_control_v2.ino.hex` — сравните с PWM244 |
| Bootloader | **Program File** = `*.ino.hex`, **не** `*.with_bootloader.hex` |
| Частота | Nano **16 MHz**, старый bootloader (старый) |

**Прошивка PWM244 (~244 Hz):** в `mast_control_v2_PWM244hz` ШИМ на D11 идёт **напрямую через Timer2 (OCR2A)**, без `analogWrite()` в каждом `loop()` — так меньше нагрузки на VSM Arduino. После правки пересоберите hex (команда выше).

**Схема / SPICE** (часто виновата петля тока, не код):

- Сообщения **`transient GMIN stepping`** у **~6–7 s** — типично, когда «оживает» цепь **4–20 mA** (RV, INA219, вольтметр).
- **`Timestep too small` / node #…** — жёсткая цепь: упростите на время теста ШИМ (реле/клапан на D11 — **резистор 1 kΩ на GND**, без MOSFET/RC-фильтра) и смотрите **осциллограф Proteus на D11**.
- Потенциометр джойстика: не оставляйте в зоне **между порогами**, где каждый loop то ШИМ, то стоп — для первого пуска держите **нейтраль** или **явный ВВЕРХ/ВНИЗ**.
- **System → Reset Dynamics** после смены hex; при необходимости отключите *Real time simulation* на время отладки.

Сравнение **976 Hz vs 244 Hz** в Proteus: логика GUI/джойстиков одинаковая; разница только на **D11** (период ~1 ms vs ~4,1 ms).
