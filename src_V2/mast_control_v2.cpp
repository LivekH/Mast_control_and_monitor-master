/*
Mast control — ВЕРСИЯ 2 (mast_control_v2)

ПРАВИЛО СОПРОВОЖДЕНИЯ:
  • Логика v1 (джойстики, концевики, защёлка осей/реле, ШИМ D11, INA219, пороги) —
    ДОЛЖНА совпадать с mast_control_v1.ino. Меняем только при явной договорённости
    и тогда же правим v1.
  • В v2 развиваем ТОЛЬКО дисплей и формат выводимых данных (OLED U8g2, ИВЭ-50,
    кнопка нуля, GUI). Не трогаем applyMotionOutputs / updateActiveJoystickAxis без нужды.

Разработано Олегом "Salty NUggeTZ" Величко и Игорем "Terrano"
*/

//==================ДОБАВЛЕНИЕ НУЖНЫХ БИБЛИОТЕК==================
//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) СЛЕДУЮЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<
//#include <Arduino.h> //библиотека Arduino для использования в VSCode PlatformIO (НЕ НУЖНА ЕСЛИ ПИСАТЬ И/ИЛИ ПРОШИВАТЬ ИЗ ARDUINO IDE!!!)
//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) ПРЕДЫДУЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<

#include <Wire.h> //lib_V2/Wire
#include <INA219_WE.h> //lib_V2/INA219_WE
#include <U8g2lib.h> //OLED SSD1309 2,42" 128x64, I2C (общая шина с INA219)

#define I2C_ADDRESS 0x40 //адрес I2C модуля INA219
#define OLED_I2C_ADDR 0x3C //SSD1309/1306 I2C (U8g2: адрес << 1)

//==================ОПРЕДЕЛЕНИЯ ПИНОВ==================
//входы
//датчик давления подключён к модулю INA219 который будет инициализирован ниже, пины А4 и А5

int valve_test_pin = A7; //вход A7 — тест напряжения на клапанах (блок проверки катушек отключён)

int axis_1 = A0; //потенциометр джойстика оси 1
int axis_2 = A1; //потенциометр джойстика оси 2
int axis_fingers = A2;  //потенциометр джойстика оси "пальцы"

int neutral_switch_1 = 8; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_2 = 9; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_3 = 10; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ

//выходы


//определить 8 выходов (7 цифровых выходов (6 управляющих реле + 1 реле для тестов), 1 ШИМ выход для мосфета)

/*----------------------------------------------------------------
out1-D2 (Выход на реле1) реле 1-2 подъемспуск первой секции мачты
out2-D3(выход на реле2)
out3-D4(выход на реле3) реле 3-4 подъемспуск второй секции мачты
out4-D5(выход на реле4)
out5-D6(выход на реле5) реле 5-6 выдвинутьзадвинуть пальцы
out6-D7(выход на реле6)
D11-выход ШИМ на управление мосфетом
*/
int test_relay = A3; //реле замыкается только для проверки сопротивления (напряжения) на клапанах, совместно с реле управления

//реле секции 1 (активны при LOW). Если вверх/вниз перепутаны на плате — поменяйте два числа местами
const uint8_t mast1_relay_up_pin = 2; //D2 — реле «подъём» первой секции мачты
const uint8_t mast1_relay_down_pin = 3; //D3 — реле «спуск» первой секции мачты

int mast2_up_relay = 4; //D4 — реле «подъём» второй секции мачты
int mast2_down_relay = 5; //D5 — реле «спуск» второй секции мачты

const uint8_t fingers_relay_close_pin = 6; //D6 — реле «закрыть» пальцы
const uint8_t fingers_relay_open_pin = 7; //D7 — реле «открыть» пальцы

int output_power = 11; //D11 — общий ШИМ на силовой ключ (нагрузка)

/*
УДАЛИТЬ ПОСЛЕ ПРОВЕРКИ КОДА
int up_state = 2; //статус движения "вверх", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВВЕРХ" от одной из осей (в том числе "пальцы" ОТКРЫТИЕ)
int down_state = 4; ///статус движения "вниз", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВНИЗ" от одной из осей (в том числе "пальцы" ЗАКРЫТИЕ)

int out_power_1 = 6; //ШИМ выход оси 1, пропорционален углу отклонения джойстика
int out_power_2 = 5; //ШИМ выход оси 2, пропорционален углу отклонения джойстика
int out_power_fingers = 3; //ШИМ выход оси "пальцы", пропорционален углу отклонения джойстика
*/

//================== v2: ДИСПЛЕЙ и ПЕТЛЯ (кнопка нуля, OLED — менять здесь) ==================
//OLED SSD1309 — I2C SDA A4, SCL A5
const uint8_t loop_zero_button_pin = 12; //кнопка «установить ноль» петли (INPUT_PULLUP, нажат = GND)
// loop_zero_mA — после нажатия текущий ток = 0 бар; шкала loop_zero_mA…20 mA → 0…250

//==================ПЕРЕМЕННЫЕ И НАСТРОЙКИ (главное — править ниже) ==================
//--- Давление (INA219, петля 4–20 mA) ---
// ina_oc_threshold_mA     — ниже: надпись «-OC-» на дисплее
// loop_zero_mA + ina_zero_display_band_mA — зона «0» на дисплее после кнопки D12
// ina_sc_threshold_mA     — выше: «-SC-»
// ina_loop_* + map         — 4 mA→0, 20 mA→250 на дисплее (калибровать шкалу при необходимости)
// display_interval_ms      — период усреднения и обновления OLED (меньше = быстрее, больше мерцание)
// display_pressure_step + display_pressure_deadband — шаг и порог обновления на OLED (бар)
// pressure_display_zero_band — показывать 0 если среднее ниже порога
// i2c_display_fault_ms     — сбой I2C дольше N мс → «I2c-»
//--- Джойстики ---
// joyProf1/2/3             — {center, relay−, relay+, pwm−, pwm+}; реле до нейтрали концевика
// joy_adc_down/up_travel    — АЦП на упорах (полный ШИМ)
// neutral_switch_debounce_ms — дребезг концевиков (больше = позже старт хода)
// joy_adc_avg_samples / joy_pwm_change_deadband — сглаживание ШИМ от шума АЦП
//--- Отладка ---
// serial_debug_enabled       — false на столе; true на ноуте (замедляет loop на 9600)
//==================ПЕРЕМЕННЫЕ==================
int delay_test_disp = 2000; //пауза на время вывода tESt на дисплей
int delay_u_disp = 2000; //пауза на время вывода номера клапана на дисплей
int delay_err_disp = 2000; //пауза на время вывода результата проверки на дисплей
int delay_done_disp = 2000; //пауза на время вывода dOnE на дисплей
int delay_redy_disp = 3000; //пауза на время вывода rEdY на дисплей
int delay_boot = 1000; //пауза при старте (зарезерв)
int delay_short = 500; //короткая пауза на дисплее при самотесте

//если напряжение ниже 0.857 - U1SC (короткое замыкание), если выше 0.888 то U1OC (обрыв цепи)
float valve_sc_limit = 0.860; //нижний предел напряжения на тестируемом клапане
float valve_oc_limit = 0.880; //верхний предел напряжения на тестируемом клапане

float current_mA = 0.0; //float для считывания данных с модуля INA219 (после constrain)
float current_mA_raw = 0.0; //сырой ток INA219 для Serial / порогов OC/SC

//датчик давления по токовой петле 4…20 mA (INA219 измеряет ток в mA)
const float ina_loop_max_mA = 20.0; //номинал петли 20 mA (верх шкалы)
const float ina_oc_threshold_mA = 3.5; //ниже — обрыв (_OC_); 3,8 mA — шум INA у нуля, не OC
const float ina_sc_threshold_mA = 20.2; //выше — короткое замыкание / перегруз (_SC_), запас выше 20 mA
const float ina_current_ceil_mA = 20.0; //верхний предел для map давления
const float ina_zero_capture_min_mA = 3.6; //ниже — не принимать «ноль» (обрыв / мусор)
const float ina_zero_capture_max_mA = 19.0; //выше — не ставить ноль (перегруз)
const float ina_zero_display_band_mA = 0.08; //ток ≤ loop_zero_mA + band → 0 на дисплее (дрожь)

float loop_zero_mA = 4.0; //опорный «ноль» (по умолчанию 4,0 до кнопки; в RAM, без EEPROM)
bool loop_zero_user_set = false; //true после успешного нажатия «ноль»
const unsigned long loop_zero_button_debounce_ms = 40;
const unsigned long loop_zero_button_hold_ms = 3000; //D12 удерживать 3 с → ZERO SET (после дребезга 40 ms)
int loop_zero_button_reading = HIGH;
int loop_zero_button_state = HIGH; //1 = отпущена
unsigned long loop_zero_button_debounce_at = 0;
unsigned long loop_zero_hold_started_ms = 0; //0 = кнопка отпущена
bool loop_zero_hold_accepted = false; //один захват нуля за одно удержание

const uint32_t i2c_wire_timeout_us = 25000; //таймаут Wire (мкс); без него TWI может зависнуть навсегда
const unsigned long i2c_recover_min_interval_ms = 200; //не чаще — повторная инициализация шины + INA219
bool ina219_comm_ok = false; //последний успешный обмен по I2C с INA219
float current_mA_last_good = 4.0; //последнее удачное значение тока (если шина сорвалась)
unsigned long i2c_last_recover_millis = 0; //время последней попытки восстановить I2C
const unsigned long i2c_display_fault_ms = 5000; //дольше — показать «I2c» (короткие сбои не мигают)
unsigned long i2c_fault_since_millis = 0; //0 = сейчас I2C в порядке

float valve_test_value = 0.0; //float для проверки сопротивления (напряжения) на клапанах

bool axis_1_neutral = true; //флаг нейтрали оси 1
bool axis_1_up = false; //флаг ВВЕРХ оси 1
bool axis_1_down = false; //флаг ВНИЗ оси 1

bool axis_2_neutral = true; //флаг нейтрали оси 2
bool axis_2_up = false; //флаг ВВЕРХ оси 2
bool axis_2_down = false; //флаг ВНИЗ оси 2

bool axis_fingers_neutral = true; //флаг нейтрали оси "пальцы"
bool axis_fingers_close = false; //флаг ЗАКРЫТЬ оси "пальцы"
bool axis_fingers_open = false; //флаг ОТКРЫТЬ оси "пальцы"

int active_joystick_axis = 0; //0=все в нейтрали; 1/2/3 — единственная разрешённая ось (жёсткая блокировка)
bool lock_1 = false; //для Serial: true если ось 1 заблокирована
bool lock_2 = false; //для Serial: true если ось 2 заблокирована
bool lock_3 = false; //для Serial: true если ось «пальцы» заблокирована

int neutral_switch_1_state = 1; //устойчивое состояние концевика оси 1 (1 = нейтраль)
int neutral_switch_2_state = 1; //устойчивое состояние концевика оси 2
int neutral_switch_3_state = 1; //устойчивое состояние концевика оси "пальцы"

const unsigned long neutral_switch_debounce_ms = 25; //дребезг концевиков (больше = дольше задержка старта хода)
const bool serial_debug_enabled = false; //true — строка в Monitor каждый loop; на 9600 даёт заметную задержку отклика
int neutral_switch_1_reading = 1; //сырое чтение концевика оси 1 (дребезг)
int neutral_switch_2_reading = 1; //сырое чтение концевика оси 2
int neutral_switch_3_reading = 1; //сырое чтение концевика оси «пальцы»
unsigned long neutral_switch_1_debounce_at = 0; //метка времени последнего изменения оси 1
unsigned long neutral_switch_2_debounce_at = 0; //метка времени последнего изменения оси 2
unsigned long neutral_switch_3_debounce_at = 0; //метка времени последнего изменения оси «пальцы»

//ADC джойстиков: реле до концевика «нейтраль»; ШИМ — отдельные пороги «−» / «+» у каждой оси
const int joy_adc_substitute_neutral = 510; //подставка в Serial, когда ось не читается (универсальный center)
const int joy_adc_down_travel = 256; //физический упор вниз (нижняя граница ШИМ, подстройка по пульту)
const int joy_adc_up_travel = 758;   //физический упор вверх (среднее ~760)

const int joy_pwm_max = 255; //максимум ШИМ analogWrite 0…255
const int joy_pwm_floor_percent = 20; //мин. ШИМ при выходе за порог ШИМ
const int joy_pwm_floor = (joy_pwm_max * joy_pwm_floor_percent) / 100; //51 при 20%

const uint8_t joy_adc_avg_samples = 4; //подряд analogRead и среднее (гасит шум линии АЦП)
const int joy_pwm_change_deadband = 5; //не менять ШИМ если |Δ| меньше (при ненулевом ходе)
int joy_adc_filtered[3] = {-1, -1, -1}; //EMA после среднего; −1 = ещё не инициализировано
int joy_pwm_smoothed[3] = {0, 0, 0}; //последний выданный ШИМ по осям 0=мачта1, 1=мачта2, 2=пальцы

//калибровка по осям: подстроить center и пороги по Serial (A1V / A2V / FV в нейтрали)
struct JoyAxisProfile {
  int center;     //центр АЦП в нейтрали
  int relayMinus; //реле «−»: ADC ≤ center − relayMinus (один раз за ход)
  int relayPlus;  //реле «+»: ADC ≥ center + relayPlus
  int pwmMinus;   //ШИМ «−»: ADC ≤ center − pwmMinus
  int pwmPlus;    //ШИМ «+»: ADC ≥ center + pwmPlus
};

// Универсальный профиль: реле при ADC≤480 / ≥540, ШИМ при ≤450 / ≥570 (center 510).
// ВВЕРХ позже → увеличивай relayPlus и pwmPlus; pwmPlus ≥ relayPlus, pwmMinus ≥ relayMinus.
const JoyAxisProfile joyProf1 = {510, 30, 30, 60, 60}; //мачта 1
const JoyAxisProfile joyProf2 = {510, 30, 30, 60, 60}; //мачта 2
const JoyAxisProfile joyProf3 = {510, 30, 30, 60, 60}; //«пальцы»: + = закрыть, − = открыть

uint8_t joy_relay_latch_axis_1 = 0; //0 нет, 1 «+»/вверх/закрыть, 2 «−»; сброс только концевиком
uint8_t joy_relay_latch_axis_2 = 0; //защёлка направления реле оси 2
uint8_t joy_relay_latch_axis_3 = 0; //защёлка реле «пальцы»

int pressure_val = 0; //переформатированное значение float с модуля INA219 в int для последующей обработки
int pressure_actual = 0; //фактическое давление в bar (атмосфер)

int axis_1_val = 0; //сырое значение состояния оси 1
int axis_1_out = 0; //переформатированное значение оси 1 для вывода ШИМ

int axis_2_val = 0; //сырое значение состояния оси 2
int axis_2_out = 0; //переформатированное значение оси 2 для вывода ШИМ

int axis_fingers_val = 0; //сырое значение состояния оси "пальцы"
int axis_fingers_out = 0; //переформатированное значение оси "пальцы" для вывода ШИМ

//ВНИМАНИЕ!!! ВОЗМОЖНО БУДУТ КОСЯКИ С ДИСПЛЕЕМ ПОСЛЕ 50 ДНЕЙ БЕЗ ПЕРЕЗАГРУЗКИ!!!
unsigned long previousmillis = 0; //когда было последнее обновление дисплея
unsigned long currentmillis = 0; //текущее время в миллисекундах
const long display_interval_ms = 100; //период усреднения и обновления OLED (мс)
const int display_pressure_step = 2; //округление давления на дисплее: 0, 2, 4, … бар
const int display_pressure_deadband = 2; //перерисовка, если шаг на экране ≥ этого (обычно = step)
const int pressure_display_zero_band = 8; //среднее 0…8 на дисплее → показывать 0

int pressure_display_filtered = 0; //среднее давление за период — только для дисплея
int pressure_display_last_shown = -1; //что уже выведено на OLED
long pressure_sample_sum = 0; //сумма pressure_actual между обновлениями дисплея
int pressure_sample_count = 0; //число замеров в текущем периоде

//--- v2 GUI (латиница) ---
const unsigned long gui_zero_msg_ms = 600;
unsigned long gui_zero_msg_until = 0;
bool gui_zero_msg_ok = true;
int gui_last_pressure = -1;
int gui_last_axis = -1;
int gui_last_dir = -99;
int gui_last_pwm_pct = -1;
bool gui_last_oc = false;
bool gui_last_sc = false;
bool gui_last_i2c = false;
const int gui_joy_cx[3] = {21, 64, 107}; //слева→направо оси 3, 2, 1

//Флажки ошибок по току
bool open_circuit = false; //обрыв цепи
bool short_circuit = false; //короткое замыкание

//==================СОЗДАНИЕ ОБЪЕКТОВ БИБЛИОТЕК==================
INA219_WE ina219 = INA219_WE(I2C_ADDRESS); //создание объекта "ina219" библиотеки INA219_WE, инициализация датчика давления
// _1_ = page buffer (меньше RAM); NONAME0/2 или SSD1306 в Proteus при пустом экране
U8G2_SSD1309_128X64_NONAME0_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//обновление состояния концевика с подавлением дребезга (stableState меняется после quiet period)
void updateNeutralSwitchDebounced(int pin, int &reading, unsigned long &debounceAt, int &stableState) {
  int sample = digitalRead(pin); //текущий уровень на пине концевика
  if (sample != reading) { //если отличается от предыдущего сэмпла — дребезг
    reading = sample; //запомнить новый уровень
    debounceAt = millis(); //сбросить таймер стабилизации
  }
  if (millis() - debounceAt >= neutral_switch_debounce_ms) { //пауза прошла
    stableState = reading; //зафиксировать устойчивое состояние
  }
}

void guiCenterStr(int y, const char *text) {
  int w = u8g2.getStrWidth(text);
  u8g2.drawStr((128 - w) / 2, y, text);
}

void showBootScreen(const char *line1, const char *line2) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tf);
    guiCenterStr(24, line1);
    guiCenterStr(42, line2);
  } while (u8g2.nextPage());
}

int guiActivePwmPercent() {
  int pwm = 0;
  if (active_joystick_axis == 1) { pwm = axis_1_out; }
  else if (active_joystick_axis == 2) { pwm = axis_2_out; }
  else if (active_joystick_axis == 3) { pwm = axis_fingers_out; }
  return constrain((pwm * 100) / 255, 0, 100);
}

int guiAxisDirection(uint8_t axis) {
  if (axis == 1) {
    if (axis_1_up) { return 1; }
    if (axis_1_down) { return -1; }
  } else if (axis == 2) {
    if (axis_2_up) { return 1; }
    if (axis_2_down) { return -1; }
  } else if (axis == 3) {
    if (axis_fingers_close) { return 1; }
    if (axis_fingers_open) { return -1; }
  }
  return 0;
}

void drawJoyArrows(int cx, int dir) {
  const int y_up = 46;
  const int y_dot = 55;
  const int y_dn = 63;
  if (dir > 0) {
    u8g2.drawTriangle(cx, y_up - 1, cx - 5, y_up + 6, cx + 5, y_up + 6);
  } else {
    u8g2.drawLine(cx, y_up, cx - 4, y_up + 6);
    u8g2.drawLine(cx, y_up, cx + 4, y_up + 6);
    u8g2.drawLine(cx - 4, y_up + 6, cx + 4, y_up + 6);
  }
  u8g2.drawDisc(cx, y_dot, 3);
  if (dir < 0) {
    u8g2.drawTriangle(cx, y_dn, cx - 5, y_dn - 7, cx + 5, y_dn - 7);
  } else {
    u8g2.drawLine(cx, y_dn, cx - 4, y_dn - 6);
    u8g2.drawLine(cx, y_dn, cx + 4, y_dn - 6);
    u8g2.drawLine(cx - 4, y_dn - 6, cx + 4, y_dn - 6);
  }
}

// подпись 3/2/1; стрелки и точка — только активная ось
void drawJoySlot(int cx, const char *label, bool active, int dir) {
  if (!active) {
    return;
  }
  drawJoyArrows(cx, dir);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(cx - 2, 63, label);
}

const int gui_pwm_label100_dx = -2;

void drawPwmBarLabelCentered(int barCx, int yBaseline, const char *label, int dx) {
  const int w = u8g2.getStrWidth(label);
  int tx = barCx - w / 2 + dx;
  if (tx < 0) {
    tx = 0;
  }
  u8g2.drawStr(tx, yBaseline, label);
}

void drawPwmBar(int pct) {
  pct = constrain(pct, 0, 100);
  const int x = 117;
  const int barW = 8;
  const int barCx = x + barW / 2;
  const int y0 = 16;
  const int h = 38;
  u8g2.setFont(u8g2_font_6x10_tf);
  drawPwmBarLabelCentered(barCx, 11, "100", gui_pwm_label100_dx);
  drawPwmBarLabelCentered(barCx, y0 + h + 9, "0", 0);
  u8g2.drawFrame(x, y0, barW, h);
  if (pct <= 0) {
    return;
  }
  int fill = (h * pct) / 100;
  if (fill < 1) { fill = 1; }
  u8g2.drawBox(x + 1, y0 + h - fill, barW - 2, fill);
}

int quantizePressureDisplayBar(int p) { //шаг display_pressure_step бар (округление)
  if (p <= 0) {
    return 0;
  }
  p = constrain(p, 0, 250);
  return ((p + display_pressure_step / 2) / display_pressure_step) * display_pressure_step;
}

void drawMainScreen(bool i2c_fault) {
  u8g2.firstPage();
  do {
    bool main_view = (millis() >= gui_zero_msg_until) && !i2c_fault
        && !open_circuit && !short_circuit;

    if (millis() < gui_zero_msg_until) {
      u8g2.setFont(u8g2_font_6x12_tf);
      if (gui_zero_msg_ok) {
        guiCenterStr(26, "ZERO SET");
      } else {
        guiCenterStr(26, "TARE FAIL");
      }
    } else if (i2c_fault) {
      u8g2.setFont(u8g2_font_6x12_tf);
      guiCenterStr(26, "I2C ERR");
    } else if (open_circuit) {
      u8g2.setFont(u8g2_font_6x12_tf);
      guiCenterStr(26, "LOOP OPEN");
    } else if (short_circuit) {
      u8g2.setFont(u8g2_font_6x12_tf);
      guiCenterStr(26, "LOOP SHORT");
    } else {
      int p = constrain(pressure_display_last_shown < 0 ? 0 : pressure_display_last_shown, 0, 250);
      char buf[8];
      snprintf(buf, sizeof(buf), "%d", p);
      u8g2.setFont(u8g2_font_logisoso26_tn);
      u8g2.drawStr(4, 30, buf);
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.drawStr(4 + u8g2.getStrWidth(buf) + 4, 30, "bar");
    }

    if (main_view) {
      u8g2.drawHLine(4, 43, 110);
      drawJoySlot(gui_joy_cx[0], "3", active_joystick_axis == 3, guiAxisDirection(3));
      drawJoySlot(gui_joy_cx[1], "2", active_joystick_axis == 2, guiAxisDirection(2));
      drawJoySlot(gui_joy_cx[2], "1", active_joystick_axis == 1, guiAxisDirection(1));
      if (active_joystick_axis != 0) {
        drawPwmBar(guiActivePwmPercent());
      }
    }
  } while (u8g2.nextPage());
}

void displayGuiUpdate(bool i2c_fault) {
  int dir = active_joystick_axis ? guiAxisDirection(active_joystick_axis) : 0;
  int pwmPct = active_joystick_axis ? guiActivePwmPercent() : 0;
  bool need = (pressure_display_last_shown != gui_last_pressure)
      || (active_joystick_axis != gui_last_axis)
      || (dir != gui_last_dir)
      || (pwmPct != gui_last_pwm_pct)
      || (open_circuit != gui_last_oc)
      || (short_circuit != gui_last_sc)
      || (i2c_fault != gui_last_i2c)
      || (millis() < gui_zero_msg_until);
  if (!need) {
    return;
  }
  drawMainScreen(i2c_fault);
  gui_last_pressure = pressure_display_last_shown;
  gui_last_axis = active_joystick_axis;
  gui_last_dir = dir;
  gui_last_pwm_pct = pwmPct;
  gui_last_oc = open_circuit;
  gui_last_sc = short_circuit;
  gui_last_i2c = i2c_fault;
}

//v2: сброс накопителя дисплея после tare — иначе 1–2 периода display_interval старые «бар» висят
void resetPressureDisplayAfterTare() {
  pressure_sample_sum = 0;
  pressure_sample_count = 0;
  pressure_display_filtered = 0;
  pressure_display_last_shown = -1;
  pressure_actual = 0;
}

//v2: нуль петли — только после удержания кнопки loop_zero_button_hold_ms
void pollLoopZeroButton() {
  int sample = digitalRead(loop_zero_button_pin);
  if (sample != loop_zero_button_reading) {
    loop_zero_button_reading = sample;
    loop_zero_button_debounce_at = millis();
  }
  if (millis() - loop_zero_button_debounce_at < loop_zero_button_debounce_ms) {
    return;
  }
  loop_zero_button_state = loop_zero_button_reading;

  if (loop_zero_button_state == LOW) { //удерживают
    if (loop_zero_hold_started_ms == 0) {
      loop_zero_hold_started_ms = millis();
    } else if (!loop_zero_hold_accepted
        && (millis() - loop_zero_hold_started_ms >= loop_zero_button_hold_ms)) {
      if (ina219_comm_ok && !open_circuit && !short_circuit
          && current_mA_raw >= ina_zero_capture_min_mA
          && current_mA_raw <= ina_zero_capture_max_mA) {
        loop_zero_mA = current_mA_raw;
        loop_zero_user_set = true;
        loop_zero_hold_accepted = true;
        resetPressureDisplayAfterTare();
        gui_zero_msg_ok = true;
#if MAST_OLED_PROTEUS
        gui_zero_msg_until = millis() + 1200; //дольше — успеть увидеть ZERO SET в симуляции
#else
        gui_zero_msg_until = millis() + gui_zero_msg_ms;
#endif
        gui_last_pressure = -1;
        if (serial_debug_enabled) {
          Serial.print(" LoopZero_mA:");
          Serial.println(loop_zero_mA, 3);
        }
      } else {
        loop_zero_hold_accepted = true;
        gui_zero_msg_ok = false;
        gui_zero_msg_until = millis() + gui_zero_msg_ms;
        gui_last_pressure = -1;
      }
    }
  } else { //отпустили
    loop_zero_hold_started_ms = 0;
    loop_zero_hold_accepted = false;
  }
}

int pressureFromLoop_mA(float mA_raw) {
  if (mA_raw <= loop_zero_mA + ina_zero_display_band_mA) {
    return 0;
  }
  const int raw100 = (int)(mA_raw * 100.0f + 0.5f);
  const int z100 = (int)(loop_zero_mA * 100.0f + 0.5f);
  const int top100 = (int)(ina_loop_max_mA * 100.0f);
  if (raw100 >= top100) {
    return 250;
  }
  return map(raw100, z100, top100, 0, 250);
}

//один раз за ход: выбрать реле «+» или «−» по АЦП; сброс при нейтрали концевика
void tryCommitRelayDirection(int adcVal, const JoyAxisProfile &prof, uint8_t &relayLatch) {
  if (relayLatch != 0) { return; } //направление реле уже зафиксировано до нейтрали концевика
  if (adcVal <= prof.center - prof.relayMinus) { relayLatch = 2; } //«−» / вниз / открыть
  else if (adcVal >= prof.center + prof.relayPlus) { relayLatch = 1; } //«+» / вверх / закрыть
}

//ШИМ только если защёлка реле есть и АЦП глубже порога pwmMinus/pwmPlus
void computeJoystickPwm(int adcVal, uint8_t relayLatch, const JoyAxisProfile &prof, int &pwmOut) {
  pwmOut = 0; //по умолчанию выкл. (реле может быть уже вкл.)
  if (relayLatch == 1 && adcVal >= prof.center + prof.pwmPlus) {
    const int pwmHi = prof.center + prof.pwmPlus;
    pwmOut = map(constrain(adcVal, pwmHi, joy_adc_up_travel), pwmHi, joy_adc_up_travel, joy_pwm_floor, joy_pwm_max);
  } else if (relayLatch == 2 && adcVal <= prof.center - prof.pwmMinus) {
    const int pwmLo = prof.center - prof.pwmMinus;
    pwmOut = map(constrain(adcVal, joy_adc_down_travel, pwmLo), joy_adc_down_travel, pwmLo, joy_pwm_max, joy_pwm_floor);
  }
}

//чтение АЦП, защёлка реле, расчёт ШИМ и флагов positive/negative для applyMotionOutputs
//axisIdx: 0=мачта1, 1=мачта2, 2=пальцы (сброс фильтров при нейтрали концевика)
void processJoystickAxisWithProfile(uint8_t analogPin, uint8_t axisIdx, int &adcVal, const JoyAxisProfile &prof,
    uint8_t &relayLatch, int &pwmOut, bool &positive, bool &negative) {
  if (axisIdx > 2) { axisIdx = 0; }
  long adcSum = 0;
  for (uint8_t n = 0; n < joy_adc_avg_samples; n++) {
    adcSum += analogRead(analogPin);
  }
  adcVal = (int)(adcSum / joy_adc_avg_samples);
  if (joy_adc_filtered[axisIdx] < 0) {
    joy_adc_filtered[axisIdx] = adcVal;
  } else {
    joy_adc_filtered[axisIdx] = (joy_adc_filtered[axisIdx] * 3 + adcVal) / 4; //лёгкое сглаживание
  }
  adcVal = joy_adc_filtered[axisIdx];

  pwmOut = 0;
  positive = false;
  negative = false;
  tryCommitRelayDirection(adcVal, prof, relayLatch); //реле до отпуска концевика
  if (relayLatch == 1) { positive = true; }
  else if (relayLatch == 2) { negative = true; }
  computeJoystickPwm(adcVal, relayLatch, prof, pwmOut);
  if (pwmOut == 0) {
    joy_pwm_smoothed[axisIdx] = 0;
  } else if (abs(pwmOut - joy_pwm_smoothed[axisIdx]) < joy_pwm_change_deadband) {
    pwmOut = joy_pwm_smoothed[axisIdx]; //дрожь на месте — держим прошлый ШИМ
  } else {
    joy_pwm_smoothed[axisIdx] = pwmOut;
  }
}

void stopAllMotionOutputs() {
  digitalWrite(output_power, HIGH); //выключить ШИМ (логика «HIGH = выкл»)
  digitalWrite(mast1_relay_up_pin, HIGH); //реле мачты 1 вверх — разомкнуто
  digitalWrite(mast1_relay_down_pin, HIGH); //реле мачты 1 вниз — разомкнуто
  digitalWrite(mast2_up_relay, HIGH); //реле мачты 2 вверх — разомкнуто
  digitalWrite(mast2_down_relay, HIGH); //реле мачты 2 вниз — разомкнуто
  digitalWrite(fingers_relay_open_pin, HIGH); //реле открыть пальцы — разомкнуто
  digitalWrite(fingers_relay_close_pin, HIGH); //реле закрыть пальцы — разомкнуто
}

void clearJoystickMotionFlags() {
  axis_1_out = 0; //сброс ШИМ оси 1
  axis_2_out = 0; //сброс ШИМ оси 2
  axis_fingers_out = 0; //сброс ШИМ «пальцы»
  axis_1_up = false; //нет команды вверх ось 1
  axis_1_down = false; //нет команды вниз ось 1
  axis_2_up = false; //нет команды вверх ось 2
  axis_2_down = false; //нет команды вниз ось 2
  axis_fingers_open = false; //нет открытия пальцев
  axis_fingers_close = false; //нет закрытия пальцев
}

void updateActiveJoystickAxis() {
  // Защёлка активной оси: пока оператор держит ось 2 (или 3) в работе, ось 1 не перехватывает
  // управление при случайном нажатии другого джойстика. Сброс — только когда активная ось
  // вернулась в нейтраль по концевику.
  if (active_joystick_axis == 1) {
    if (axis_1_neutral) { active_joystick_axis = 0; }
  } else if (active_joystick_axis == 2) {
    if (axis_2_neutral) { active_joystick_axis = 0; }
  } else if (active_joystick_axis == 3) {
    if (axis_fingers_neutral) { active_joystick_axis = 0; }
  }

  if (active_joystick_axis == 0) { //никто не занят — выбираем одну не-нейтральную
    if (!axis_1_neutral) {
      active_joystick_axis = 1;
    } else if (!axis_2_neutral) {
      active_joystick_axis = 2;
    } else if (!axis_fingers_neutral) {
      active_joystick_axis = 3;
    }
  }

  lock_1 = (active_joystick_axis != 0 && active_joystick_axis != 1); //блок остальных осей
  lock_2 = (active_joystick_axis != 0 && active_joystick_axis != 2);
  lock_3 = (active_joystick_axis != 0 && active_joystick_axis != 3);
}

void processJoystickAxis1() { //мачта секция 1
  processJoystickAxisWithProfile(axis_1, 0, axis_1_val, joyProf1, joy_relay_latch_axis_1, axis_1_out, axis_1_up, axis_1_down);
}

void applyMast1Relays(bool up, bool down) {
  if (up && !down) { //только подъём
    digitalWrite(mast1_relay_up_pin, LOW); //замкнуть реле вверх
    digitalWrite(mast1_relay_down_pin, HIGH); //реле вниз разомкнуто
  } else if (down && !up) { //только спуск
    digitalWrite(mast1_relay_up_pin, HIGH); //реле вверх разомкнуто
    digitalWrite(mast1_relay_down_pin, LOW); //замкнуть реле вниз
  } else { //неопределённость или стоп
    digitalWrite(mast1_relay_up_pin, HIGH); //оба реле разомкнуты
    digitalWrite(mast1_relay_down_pin, HIGH); //реле вниз разомкнуто
  }
}

void processJoystickAxis2() { //мачта секция 2
  processJoystickAxisWithProfile(axis_2, 1, axis_2_val, joyProf2, joy_relay_latch_axis_2, axis_2_out, axis_2_up, axis_2_down);
}

//ось «пальцы»: «+» ADC → ЗАКРЫТЬ (D6), «−» → ОТКРЫТЬ (D7)
void processJoystickAxisFingers() {
  processJoystickAxisWithProfile(axis_fingers, 2, axis_fingers_val, joyProf3, joy_relay_latch_axis_3, axis_fingers_out, axis_fingers_close, axis_fingers_open);
}

void applyFingersRelays(bool closeActive, bool openActive) {
  if (closeActive && !openActive) { //закрыть пальцы
    digitalWrite(fingers_relay_close_pin, LOW); //реле «закрыть» вкл
    digitalWrite(fingers_relay_open_pin, HIGH); //«открыть» выкл
  } else if (openActive && !closeActive) { //открыть пальцы
    digitalWrite(fingers_relay_close_pin, HIGH); //«закрыть» выкл
    digitalWrite(fingers_relay_open_pin, LOW); //«открыть» вкл
  } else { //стоп
    digitalWrite(fingers_relay_close_pin, HIGH); //оба реле выкл
    digitalWrite(fingers_relay_open_pin, HIGH); //оба реле пальцев разомкнуты
  }
}

//единственная активная ось: один ШИМ D11 + её реле (остальное гасится в stopAllMotionOutputs)
void applyMotionOutputs() {
  stopAllMotionOutputs(); //сначала всё выключить
  if (active_joystick_axis == 0) { //нет активной оси
    return; //выход
  }
  if (active_joystick_axis == 1) { //мачта секция 1
    if (axis_1_up || axis_1_down) { //есть команда движения
      analogWrite(output_power, axis_1_out); //ШИМ по оси 1
      applyMast1Relays(axis_1_up, axis_1_down); //реле вверх/вниз
    }
  } else if (active_joystick_axis == 2) { //мачта секция 2
    if (axis_2_up) { //подъём
      analogWrite(output_power, axis_2_out); //ШИМ по оси 2
      digitalWrite(mast2_up_relay, LOW); //реле вверх вкл
      digitalWrite(mast2_down_relay, HIGH); //реле вниз выкл
    } else if (axis_2_down) { //спуск
      analogWrite(output_power, axis_2_out); //ШИМ по оси 2
      digitalWrite(mast2_up_relay, HIGH); //реле вверх выкл
      digitalWrite(mast2_down_relay, LOW); //реле вниз вкл
    }
  } else if (active_joystick_axis == 3) { //пальцы
    if (axis_fingers_close || axis_fingers_open) { //есть команда
      analogWrite(output_power, axis_fingers_out); //ШИМ
      applyFingersRelays(axis_fingers_close, axis_fingers_open); //реле пальцев
    }
  }
}

void i2cBusBegin() {
  Wire.begin(); //старт I2C (мастер)
  Wire.setClock(100000); //100 kHz — устойчивее на длинной шине
  Wire.setWireTimeout(i2c_wire_timeout_us, true); //таймаут + сброс TWI при зависании
  Wire.clearWireTimeoutFlag(); //сброс флага таймаута
}

bool i2cIna219Present() {
  Wire.beginTransmission(I2C_ADDRESS); //старт передачи на INA219
  uint8_t err = Wire.endTransmission(); //0 = ACK
  if (Wire.getWireTimeoutFlag()) { //таймаут шины
    return false; //нет устройства
  }
  return err == 0; //ответ получен
}

void i2cRecoverBusAndIna219() {
  unsigned long now = millis(); //текущее время
  if (now - i2c_last_recover_millis < i2c_recover_min_interval_ms) { //не чаще лимита
    return; //пропуск попытки
  }
  i2c_last_recover_millis = now; //зафиксировать попытку
  Wire.clearWireTimeoutFlag(); //сброс флага
  Wire.end(); //отключить TWI
  delay(2); //короткая пауза на линии
  i2cBusBegin(); //снова поднять I2C
  ina219_comm_ok = ina219.init(); //переинициализация INA219
}

//true — прочитан новый ток; false — шина/датчик не ответили, вызывающий код использует last_good
bool readIna219Current_mA(float &out_mA) {
  Wire.clearWireTimeoutFlag(); //очистить флаг перед обменом
  if (!i2cIna219Present()) { //нет ответа на адресе
    i2cRecoverBusAndIna219(); //восстановить шину
    return false; //данные невалидны
  }
  out_mA = ina219.getCurrent_mA(); //чтение тока в mA
  if (Wire.getWireTimeoutFlag()) { //обрыв при чтении
    i2cRecoverBusAndIna219();
    return false;
  }
  return true; //успех
}

//==================НАСТРОЙКИ, выполняется разово при включении МК==================
void setup() {
Serial.begin(9600); //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы
i2cBusBegin(); //старт I2C шины и таймаутов

  ina219_comm_ok = ina219.init(); //проверка подключения модуля ina219
  if (!ina219_comm_ok) { //датчик не ответил
    Serial.println("INA219 not connected!"); //сообщение в Serial Monitor
  }

u8g2.setI2CAddress(OLED_I2C_ADDR * 2);
u8g2.begin();
u8g2.setBusClock(400000);
u8g2.setFontMode(1);
u8g2.setContrast(200);

//определения режимов работы пинов входов
pinMode(axis_1, INPUT); //вертикальная ось 1, потенциометр, вход
pinMode(neutral_switch_1, INPUT_PULLUP); //концевик нейтрали вертикальной оси 1, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_2, INPUT); //вертикальная ось 2, потенциометр, вход
pinMode(neutral_switch_2, INPUT_PULLUP); //концевик нейтрали вертикальной оси 2, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_fingers, INPUT); //ось "пальцы", потенциометр, вход
pinMode(neutral_switch_3, INPUT_PULLUP); //концевик нейтрали оси "пальцы", !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(loop_zero_button_pin, INPUT_PULLUP); //кнопка нуля петли 4–20 mA (v2)
loop_zero_button_reading = digitalRead(loop_zero_button_pin);
loop_zero_button_state = loop_zero_button_reading;
loop_zero_button_debounce_at = millis();

neutral_switch_1_reading = digitalRead(neutral_switch_1); //начальное чтение концевика 1
neutral_switch_2_reading = digitalRead(neutral_switch_2); //начальное чтение концевика 2
neutral_switch_3_reading = digitalRead(neutral_switch_3); //начальное чтение концевика 3
neutral_switch_1_state = neutral_switch_1_reading; //устойчивое = начальное
neutral_switch_2_state = neutral_switch_2_reading; //устойчивое = начальное ось 2
neutral_switch_3_state = neutral_switch_3_reading; //устойчивое = начальное ось «пальцы»
neutral_switch_1_debounce_at = millis(); //старт таймера дребезга 1
neutral_switch_2_debounce_at = millis(); //старт таймера дребезга 2
neutral_switch_3_debounce_at = millis(); //старт таймера дребезга 3

//определения режимов работы пинов выходов
//pinMode(test_relay, OUTPUT); //реле тестирования клапанов (не используется — отказ от проверки катушек)
pinMode(output_power, OUTPUT); //общий ШИМ на мосфет
pinMode(mast1_relay_up_pin, OUTPUT); //мачта элемент 1 выход на реле ВВЕРХ
pinMode(mast1_relay_down_pin, OUTPUT); //мачта элемент 1 выход на реле ВНИЗ
pinMode(mast2_up_relay, OUTPUT); //мачта элемент 2 выход на реле ВВЕРХ
pinMode(mast2_down_relay, OUTPUT); //мачта элемент 2 выход на реле ВНИЗ
pinMode(fingers_relay_open_pin, OUTPUT); //выход реле «открыть» пальцы
pinMode(fingers_relay_close_pin, OUTPUT); //выход реле «закрыть» пальцы

//размыкание всех реле
//реле управления клапанами РАЗМЫКАЮТСЯ НА ВЫСОКОМ УРОВНЕ И ЗАМЫКАЮТСЯ НА НИЗКОМ!!!
//digitalWrite(test_relay, LOW);
digitalWrite(output_power, HIGH); //ШИМ выкл. (логика как в рабочем цикле)
digitalWrite(mast1_relay_up_pin, HIGH); //реле мачты 1 — выкл
digitalWrite(mast1_relay_down_pin, HIGH); //реле мачты 1 вниз — выкл
digitalWrite(mast2_up_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(mast2_down_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(fingers_relay_open_pin, HIGH); //реле пальцев — выкл
digitalWrite(fingers_relay_close_pin, HIGH); //реле пальцев — выкл

//сброс статуса блокировки на ЛОЖЬ для всех трёх осей, скорее всего удалить строки когда будут использоваться концевики
active_joystick_axis = 0; //ни одна ось не занята
lock_1 = false; //ось 1 разблокирована
lock_2 = false; //ось 2 разблокирована
lock_3 = false; //ось «пальцы» разблокирована

showBootScreen("LOOP OPEN", "TEST");
delay(delay_short);
showBootScreen("LOOP SHORT", "TEST");
delay(delay_short);
showBootScreen("READY", "v2");
delay(delay_done_disp);
pressure_display_last_shown = -1;
pressure_display_filtered = 0;
gui_last_pressure = -1;

/*
//--- проверка целостности катушек клапанов (U1–U6 через A7 / test_relay) — отключено ---
//проверить исправвность клапанов путём подачи 1.1В на aref и на входе А7 ...
tm1637.displayByte(_d, _i, _S, _P);
delay(delay_test_disp);
tm1637.displayByte(_t, _E, _S, _t);
delay(delay_test_disp);
...
analogReference(INTERNAL);
digitalWrite(test_relay, HIGH);
digitalWrite(mast1_up_relay, LOW);
valve_test_value = map(analogRead(valve_test_pin), 0, 1023, 0, 1.100);
...
analogReference(DEFAULT);
tm1637.displayByte(_r, _E, _d, _Y);
delay(delay_redy_disp);
//--- конец отключённой проверки катушек ---
*/
}


//Архив заметок с макета: универсально в коде center 510, упоры 248/758, joyProf см. выше.
//Ось «пальцы»: отклонение «+» по АЦП = ЗАКРЫТЬ (D6), «−» = ОТКРЫТЬ (D7).

//==================ОСНОВНОЙ ЦИКЛ, выполняется пока работает МК==================
void loop() {
//=====================================================================================================

currentmillis = millis(); //текущее время (мс) для дисплея и таймеров

//==================ПОЛУЧЕНИЕ СЫРЫХ ДАННЫХ С ДАТЧИКА ДАВЛЕНИЯ==================
open_circuit = false; //сброс флага обрыва 4–20 mA на этот цикл
short_circuit = false; //сброс флага КЗ/перегруза
if (readIna219Current_mA(current_mA_raw)) { //успешное чтение INA219
  ina219_comm_ok = true; //I2C в норме
  current_mA_last_good = current_mA_raw; //запомнить для fallback
  i2c_fault_since_millis = 0; //сброс таймера «I2c» на дисплее
} else {
  ina219_comm_ok = false; //обмен не удался
  //I2C сорвался — цифры на дисплее до порога i2c_display_fault_ms, затем «I2c»
  if (i2c_fault_since_millis == 0) { //первый момент сбоя
    i2c_fault_since_millis = currentmillis; //начать отсчёт до «I2c»
  }
  current_mA_raw = current_mA_last_good; //не портить расчёты нулём
}
if (ina219_comm_ok && current_mA_raw < ina_oc_threshold_mA){open_circuit = true;} //обрыв — проверка до constrain
if (ina219_comm_ok && current_mA_raw > ina_sc_threshold_mA){short_circuit = true;} //короткое замыкание
current_mA = constrain(current_mA_raw, loop_zero_mA, ina_current_ceil_mA);

pollLoopZeroButton(); //v2: установка нуля петли по кнопке D12

//==================КОНВЕРТИРОВАНИЕ ДАННЫХ С ФОРМАТА float В ФОРМАТ int==================
if (ina219_comm_ok) { //считаем давление только при живом I2C
  pressure_val = (int)(current_mA * 100.0f);
  //v2: шкала от loop_zero_mA (кнопка) до 20 mA → 0…250
  pressure_actual = pressureFromLoop_mA(current_mA_raw);

  pressure_sample_sum += pressure_actual; //накапливаем для среднего на дисплей
  pressure_sample_count++; //счётчик замеров за период
}

//==================СЧИТЫВАНИЕ КОНЦЕВИКОВ (с подавлением дребезга)==================
updateNeutralSwitchDebounced(neutral_switch_1, neutral_switch_1_reading, neutral_switch_1_debounce_at, neutral_switch_1_state); //ось 1
updateNeutralSwitchDebounced(neutral_switch_2, neutral_switch_2_reading, neutral_switch_2_debounce_at, neutral_switch_2_state); //ось 2
updateNeutralSwitchDebounced(neutral_switch_3, neutral_switch_3_reading, neutral_switch_3_debounce_at, neutral_switch_3_state); //«пальцы»

if (neutral_switch_1_state == 1){axis_1_neutral = true;} //концевик разомкнут = нейтраль ось 1
else {axis_1_neutral = false;} //джойстик активен
if (neutral_switch_2_state == 1){axis_2_neutral = true;} //нейтраль ось 2
else {axis_2_neutral = false;} //джойстик ось 2 активен
if (neutral_switch_3_state == 1){axis_fingers_neutral = true;} //нейтраль «пальцы»
else {axis_fingers_neutral = false;} //джойстик «пальцы» активен

if (axis_1_neutral) { //нейтраль концевика — реле и фильтры оси 1
  joy_relay_latch_axis_1 = 0;
  joy_adc_filtered[0] = -1;
  joy_pwm_smoothed[0] = 0;
}
if (axis_2_neutral) {
  joy_relay_latch_axis_2 = 0;
  joy_adc_filtered[1] = -1;
  joy_pwm_smoothed[1] = 0;
}
if (axis_fingers_neutral) {
  joy_relay_latch_axis_3 = 0;
  joy_adc_filtered[2] = -1;
  joy_pwm_smoothed[2] = 0;
}

//==================ЕДИНСТВЕННАЯ АКТИВНАЯ ОСЬ + ДжОЙСТИК==================
updateActiveJoystickAxis(); //кто один управляет
clearJoystickMotionFlags(); //сброс ШИМ/флагов перед новым расчётом

if (active_joystick_axis == 0) { //все в нейтрали
  axis_1_val = joy_adc_substitute_neutral; //подставка «центр» для Serial
  axis_2_val = joy_adc_substitute_neutral; //подставка ось 2
  axis_fingers_val = joy_adc_substitute_neutral; //подставка «пальцы»
} else if (active_joystick_axis == 1) { //только ось 1
  axis_2_val = joy_adc_substitute_neutral; //остальные не читаем
  axis_fingers_val = joy_adc_substitute_neutral;
  processJoystickAxis1(); //АЦП + map ось 1
} else if (active_joystick_axis == 2) { //только ось 2
  axis_1_val = joy_adc_substitute_neutral; //ось 1 не читаем
  axis_fingers_val = joy_adc_substitute_neutral;
  processJoystickAxis2(); //АЦП + map ось 2
} else if (active_joystick_axis == 3) { //только «пальцы»
  axis_1_val = joy_adc_substitute_neutral;
  axis_2_val = joy_adc_substitute_neutral; //ось 2 не читаем
  processJoystickAxisFingers(); //АЦП + map «пальцы»
}

applyMotionOutputs(); //реле + один ШИМ на активную ось

if (serial_debug_enabled) { //в отладке — реальный ADC всех осей (иначе неактивные = подставка joy_adc_substitute_neutral)
  axis_1_val = analogRead(axis_1);
  axis_2_val = analogRead(axis_2);
  axis_fingers_val = analogRead(axis_fingers);
}

//v2 OLED GUI
    if (currentmillis - previousmillis >= display_interval_ms) {
        previousmillis = currentmillis;
        if (pressure_sample_count > 0) {
            int avg = (int)((pressure_sample_sum + pressure_sample_count / 2) / pressure_sample_count);
            if (avg < pressure_display_zero_band) {
              pressure_display_filtered = 0;
            } else {
              pressure_display_filtered = quantizePressureDisplayBar(avg);
            }
            if (pressure_display_last_shown < 0
                || abs(pressure_display_filtered - pressure_display_last_shown) >= display_pressure_deadband) {
              pressure_display_last_shown = pressure_display_filtered;
            }
            pressure_sample_sum = 0;
            pressure_sample_count = 0;
        }
        bool i2c_display_fault = (!ina219_comm_ok && i2c_fault_since_millis != 0
            && (currentmillis - i2c_fault_since_millis >= i2c_display_fault_ms));
        displayGuiUpdate(i2c_display_fault);
    }
    if (millis() < gui_zero_msg_until) {
        bool i2c_display_fault = (!ina219_comm_ok && i2c_fault_since_millis != 0
            && (currentmillis - i2c_fault_since_millis >= i2c_display_fault_ms));
        displayGuiUpdate(i2c_display_fault);
    }

//===============================================================================================



//ВЫВОД ДАННЫХ НА ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ (9600) — только если serial_debug_enabled
if (serial_debug_enabled) {
Serial.print("A1V:"); Serial.print(axis_1_val); Serial.print(" A1O:"); Serial.print(axis_1_out); //ось 1 АЦП и ШИМ
Serial.print(" A1N:"); Serial.print(axis_1_neutral); //нейтраль ось 1
Serial.print(" A1U:"); Serial.print(axis_1_up); //флаг вверх
Serial.print(" A1D:"); Serial.print(axis_1_down); //флаг вниз
Serial.print(" R1up:"); Serial.print(digitalRead(mast1_relay_up_pin) == LOW); //реле вверх замкнуто?
Serial.print(" R1dn:"); Serial.print(digitalRead(mast1_relay_down_pin) == LOW); //реле вниз замкнуто?
Serial.print(" A1Lock:"); Serial.print(lock_1); //блокировка оси 1
Serial.print(" ActAx:"); Serial.print(active_joystick_axis); //активная ось 0…3
Serial.print(" A2V:"); Serial.print(axis_2_val); Serial.print(" A2O:"); Serial.print(axis_2_out); //ось 2
Serial.print(" A2N:"); Serial.print(axis_2_neutral); //нейтраль ось 2
Serial.print(" A2U:"); Serial.print(axis_2_up); //вверх ось 2
Serial.print(" A2D:"); Serial.print(axis_2_down); //вниз ось 2
Serial.print(" A2Lock:"); Serial.print(lock_2); //блокировка оси 2
Serial.print(" FV:"); Serial.print(axis_fingers_val); Serial.print(" FO:"); Serial.print(axis_fingers_out); //«пальцы»
Serial.print(" AFN:"); Serial.print(axis_fingers_neutral); //нейтраль «пальцы»
Serial.print(" AFclose:"); Serial.print(axis_fingers_close); //закрыть
Serial.print(" AFopen:"); Serial.print(axis_fingers_open); //открыть
Serial.print(" R3cl:"); Serial.print(digitalRead(fingers_relay_close_pin) == LOW); //реле закрыть
Serial.print(" R3op:"); Serial.print(digitalRead(fingers_relay_open_pin) == LOW); //реле открыть
Serial.print(" AFLock:"); Serial.print(lock_3); //блокировка «пальцы»
Serial.print(" I2C:"); Serial.print(ina219_comm_ok); //связь с INA219
Serial.print(" Iraw[mA]:"); Serial.print(current_mA_raw); //сырой ток
Serial.print(" Iuse[mA]:"); Serial.print(current_mA); //после constrain
Serial.print(" OC:"); Serial.print(open_circuit); //обрыв
Serial.print(" SC:"); Serial.print(short_circuit); //КЗ
Serial.print(" PressVal:"); Serial.print(pressure_val); //промежуточное
Serial.print(" PressAct:"); Serial.print(pressure_actual); //давление расчёт
Serial.print(" PressDisp:"); Serial.print(pressure_display_filtered); //на дисплей
Serial.println(); //конец строки CSV в Monitor
}
}
