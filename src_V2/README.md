# src_V2 — прошивка версии 2 (PlatformIO)

Зеркало скетча из **`mast_control_v2/mast_control_v2.ino`** (создаётся в Arduino IDE).

- Платформа: Arduino Nano (как v1).
- Сборка PlatformIO: окружение **`mast_v2`** в `platformio.ini` (`src_dir = src_V2`, `lib_dir = lib_V2`).
- После правок в `.ino` копируйте в `mast_control_v2.cpp` и включайте `#include <Arduino.h>` только для PIO (как в v1).

Файл: `mast_control_v2.cpp` — синхронизируйте с `mast_control_v2/mast_control_v2.ino`.
