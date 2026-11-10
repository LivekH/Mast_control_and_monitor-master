# Mast control and monitor

**v1** — TM1637, тестовая петля 4–20 mA. **v2** (в работе) — OLED SSD1309 2,42", ИВЭ-50, кнопка нуля.

## Структура

| Путь | Назначение |
|------|------------|
| `mast_control_v1/mast_control_v1.ino` | **Основной скетч** — Arduino IDE |
| `src/mast_control_v1.cpp` | Та же прошивка для PlatformIO |
| `lib_V1/` | **Wire**, INA219_WE, GyverSegment; `proteus_V1/` — проект Proteus v1 (`.pdsprj`) |
| `docs/` | Описание системы (MD, HTML, PDF, схемы) |
| `platformio.ini` | `pio run -e mast_v1` / `mast_v2` |
| `mast_control_v2/mast_control_v2.ino` | **v2:** логика = v1, меняется только OLED/данные на экране |
| `src_V2/` | Зеркало v2 для PlatformIO |
| `lib_V2/`, `lib_V2/proteus_V2/` | Библиотеки v2; в Proteus OLED ≈ **SSD1306** |
| `docs/План_версии_v2.md` | План v2, ИВЭ-50, нуль, дисплей |
| `docs/Резюме_работ_v2.md` | Итоги разработки v2 |
| `docs/Пины_Arduino_Nano_v2.md` | Распиновка Nano (v2) |
| `docs/Руководство_пользователя_v2.md` / `.html` | Руководство оператора: GUI и действия |

## Arduino IDE

1. Открыть `mast_control_v1/mast_control_v1.ino`.
2. Библиотеки: `lib_V1/libraries_v1.txt`.
3. Плата: Arduino Nano, порт COM.

**v2:** папка `mast_control_v2`, библиотеки — `lib_V2/`.

## PlatformIO

v1: `pio run -e mast_v1` — `lib_V1`, `src/`.  
v2: `pio run -e mast_v2` — `lib_V2`, `src_V2/` (после появления `mast_control_v2.cpp`).

После правок синхронизируйте `mast_control_v1.ino` и `src/mast_control_v1.cpp` (блок `#include <Arduino.h>` — только для PIO).

## Документация

`docs/Описание_системы_управления.html` — печать/PDF в браузере.
