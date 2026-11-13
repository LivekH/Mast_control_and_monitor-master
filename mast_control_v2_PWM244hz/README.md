# mast_control_v2_PWM244hz

Экспериментальная **v2** (OLED GUI): ШИМ на **D11** **~244 Hz** (ориентир [Walvoil CED252](https://www.walvoil.com/allegati/catalogo/CED252_US.pdf): **100–150–220 Hz**).

Логика совпадает с `mast_control_v2`, отличие только в блоке Timer2 — см. комментарии у `analogWriteOutputPowerPwm244` в `.ino`.

### Proteus

- В скетче по умолчанию **`#define MAST_OLED_PROTEUS 1`** (SSD1306, I2C 100 kHz) — как `mast_control_v2`.
- Готовый hex: **`lib_V2/proteus_V2/firmware/mast_control_v2_PWM244hz.ino.hex`** — в схеме v2 вместо `mast_control_v2.ino.hex`.
- На **D11** в симуляции смотрите период ШИМ (~4,1 ms vs ~1 ms у 976 Hz).

## Документация

- **[`docs/ШИМ_D11_CED252.md`](../docs/ШИМ_D11_CED252.md)** — частоты, CED252, D11 vs D9  
- **[`docs/Руководство_пользователя_v2.md`](../docs/Руководство_пользователя_v2.md)** — GUI (одинаков с v2)

## Прошивка

1. Arduino IDE, Nano, `lib_V2/`.  
2. На **реальном щите:** `#define MAST_OLED_PROTEUS 0` (SSD1309).  
3. Не путать с hex из `lib_V2/proteus_V2/firmware/` (основная v2, ~976 Hz).

Основная v2: `../mast_control_v2/mast_control_v2.ino`.
