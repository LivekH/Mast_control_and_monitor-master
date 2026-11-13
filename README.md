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
| `mast_control_v1_PWM244hz/` | **v1**, ШИМ D11 ~**244 Hz** (эксперимент, CED252) |
| `mast_control_v2_PWM244hz/` | **v2**, то же по ШИМ |
| `src_V2/` | Зеркало v2 для PlatformIO |
| `lib_V2/`, `lib_V2/proteus_V2/` | Библиотеки v2; в Proteus OLED ≈ **SSD1306** |
| `docs/План_версии_v2.md` | План v2, ИВЭ-50, нуль, дисплей |
| `docs/Резюме_работ_v2.md` | Итоги разработки v2 |
| `docs/Пины_Arduino_Nano_v2.md` | Распиновка Nano (v2) |
| `docs/Руководство_пользователя_v2.md` / `.html` | Руководство оператора: GUI и действия |
| `docs/ШИМ_D11_CED252.md` | Частота ШИМ D11, CED252, прошивки *_PWM244hz |

## Arduino IDE

1. Открыть `mast_control_v1/mast_control_v1.ino`.
2. Библиотеки: `lib_V1/libraries_v1.txt`.
3. Плата: Arduino Nano, порт COM.

**v2:** папка `mast_control_v2`, библиотеки — `lib_V2/`.

### v2: Proteus (симуляция) и реальная плата

В `mast_control_v2/mast_control_v2.ino` флаг **`MAST_OLED_PROTEUS`** выбирает сборку под симулятор или под стенд:

```cpp
// 1 = Proteus. 0 = реальный OLED SSD1309 на щите.
#define MAST_OLED_PROTEUS 1
```

**В репозитории для Proteus обычно стоит `1`** — под это собирается  
`lib_V2/proteus_V2/firmware/mast_control_v2.ino.hex`.  
**На живой Arduino Nano перед Upload ставьте `0`** и прошивайте из IDE, **не** подставляйте hex из `proteus_V2/firmware/` без пересборки с `0`.

#### SSD1306 (Proteus) vs SSD1309 (железо)

| | Proteus | Модуль 2,42″ на щите |
|---|---------|----------------------|
| Контроллер | **SSD1306** | **SSD1309** |
| Разрешение | 128×64 I2C | 128×64 I2C |
| Драйвер U8g2 при `1` / `0` | `SSD1306_128X64_NONAME` | `SSD1309_128X64_NONAME0` |

Оба 128×64 по I2C, но **инициализация чипа разная**. С прошивкой под 1306 реальный 1309 часто даёт **пустой экран, полосы или «мусор»**.

При `MAST_OLED_PROTEUS 0` в коде автоматически подключается **SSD1309 NONAME0** (типично для модулей 2,42″).  
Если картинка смещена или перевёрнута — уточните даташит модуля; иногда нужен вариант **NONAME2** в конструкторе U8g2.

#### Что ещё меняется при `MAST_OLED_PROTEUS 0`

| Параметр | `1` (Proteus) | `0` (железо) |
|----------|---------------|--------------|
| Частота I2C OLED | 100 kHz | 400 kHz |
| «I2C FAILED» на OLED | ~0,8 с | ~5 с |
| Длительность ZERO SET | ~1,2 с | ~0,6 с |
| Tare по Serial `Z` / `z` | да | нет |
| `digitalWrite(D12, HIGH)` в `setup` | да (симулятор) | нет |

Джойстики, реле, D11, давление, D12 (удержание **3 с**) — **одинаково**.

#### Прошивка на реальную Nano (краткий чеклист)

1. `#define MAST_OLED_PROTEUS 0` в `mast_control_v2.ino`.
2. Arduino IDE: плата **Arduino Nano**, библиотеки **Wire**, **INA219_WE**, **U8g2** (`lib_V2` или Менеджер библиотек).
3. **Upload** по USB — не hex из Proteus.
4. I2C на **A4/A5**: INA219 **0x40**, OLED **0x3C** (`OLED_I2C_ADDR`; при перемычке **0x3D** на модуле — поменять в коде).
5. После включения: заставки → **READY v2** → главный экран.

#### Если OLED молчит при `0`

- Убедиться, что на модуле **SSD1309**, не 1306-only плата без совместимости.
- Пробный **I2C scanner** (`lib_V2/Wire/examples/i2c_scanner`) — видны **0x3C** (или 0x3D) и **0x40**.
- Общий GND, питание модуля, подтяжки SDA/SCL.
- При странной геометрии картинки — другой профиль U8g2 SSD1309 (NONAME0 / NONAME2).

Подробнее по железу/плану: `docs/План_версии_v2.md`.

### ШИМ пропорционального клапана (D11) и CED252

Оригинальный блок **Walvoil [CED252](https://www.walvoil.com/allegati/catalogo/CED252_US.pdf)** (5× HSD PWM): в каталоге указаны **100–150–220 Hz**, до **2 A** на пропорциональный выход. На **Arduino Nano**, пин **D11** (Timer2), стандартный `analogWrite()` даёт **~976 Hz**. Клапан на макете обычно работает и так; для испытаний «ближе к CED252» без переноса ШИМ на другой пин (например D9 у оригинала) используйте:

| Папка | Дисплей | ШИМ D11 |
|-------|---------|---------|
| `mast_control_v1` / `mast_control_v2` | TM1637 / OLED | **~976 Hz** (по умолчанию) |
| `mast_control_v1_PWM244hz` / `mast_control_v2_PWM244hz` | то же | **~244 Hz** (делитель Timer2 /256) |

Подробно: **`docs/ШИМ_D11_CED252.md`**, README внутри папок `*_PWM244hz`.

## PlatformIO

v1: `pio run -e mast_v1` — `lib_V1`, `src/`.  
v2: `pio run -e mast_v2` — `lib_V2`, `src_V2/` (после появления `mast_control_v2.cpp`).

После правок синхронизируйте `mast_control_v1.ino` и `src/mast_control_v1.cpp` (блок `#include <Arduino.h>` — только для PIO).

## Документация

`docs/Описание_системы_управления.html` — печать/PDF в браузере.
