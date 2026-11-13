# lib_V2 — зависимости версии 2

| Папка | `#include` | Назначение |
|-------|------------|------------|
| `Wire` | `<Wire.h>` | I2C (INA219 + OLED) |
| `INA219_WE` | `<INA219_WE.h>` | Петля тока (ИВЭ-50 в разрыве) |
| `U8g2` | `<U8g2lib.h>` | OLED **SSD1309** 2,42" I2C (A4/A5); в Proteus — **SSD1306** |

**Arduino IDE:** ZIP из каждой папки или один доп. путь на `lib_V2` (как в v1).

**Конструктор дисплея (подстроить под ваш модуль — I2C/SPI, reset):**

```cpp
// Пример I2C 128x64 SSD1309 (full buffer):
// U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
```

**PlatformIO:** `pio run -e mast_v2`, `lib_dir = lib_V2`.

| `proteus_V2/` | Симуляция v2 (`.pdsprj`) |

Подробнее: `docs/План_версии_v2.md`, `libraries_v2.txt`.

**ШИМ ~244 Hz:** скетч `mast_control_v2_PWM244hz/`, те же библиотеки; `docs/ШИМ_D11_CED252.md`.
