/*
  DHT22 + ёмкостный датчик почвы + помпа на реле + RTC DS3231 + LCD 16x2 + энкодер
  --------------------------------------------------------------------------------
  Экранов три, переключаются энкодером по кругу
  (вправо — следующий, влево — предыдущий):

   1) AIR   — температура и влажность воздуха (DHT22), опрос раз в 5 мин
   2) SOIL  — влажность почвы, опрос раз в 10 с
   3) CLOCK — время и дата с DS3231

  Полив:
   * влажность <= 20 %  → реле включает помпу (пин 27);
   * влажность >= 30 %  → реле выключает помпу (гистерезис, чтобы не
     «дребезжать» контактами на границе 20 %);
   * помпа не выключается раньше PUMP_MIN_ON_MS — защита реле от частых
     срабатываний;
   * защита по времени: не дольше PUMP_MAX_MS подряд, затем пауза
     PUMP_COOLDOWN_MS;
   * обрыв/КЗ датчика почвы — полив блокируется;
   * почва опрашивается всегда, даже когда на экране время или воздух;
   * при старте помпа гарантированно выключена.

  Часы:
   * DS3231 висит на той же шине I2C (SDA 21, SCL 22), адрес 0x68;
   * если при старте RTC сообщает о потере питания (села батарейка CR2032),
     время выставляется по времени компиляции скетча — потом поправьте
     кнопкой/через NTP (см. комментарий в setup).

  ВАЖНО: пороги SOIL_DRY_MV / SOIL_WET_MV — в милливольтах и зависят от
  экземпляра датчика. Как откалибровать — в комментарии под ними.

  Оптимизации:
   * analogReadMilliVolts() вместо сырого analogRead(): АЦП ESP32
     откалиброван по eFuse, шкала линейная и мВ не зависят от партии чипа.
   * Убран delay(10) в опросе АЦП: 15 быстрых отсчётов, по 3 крайних
     отбрасываются — без блокировки loop(), энкодер не «залипает».
   * lcd.clear() в цикле убран: пишется только изменившаяся строка ровно
     на 16 символов — нет мерцания и «хвостов» от старых значений.
   * Пока данные не обновились, шина I2C не используется вообще.
   * Часы читаются 4 раза в секунду, но экран перерисовывается только при
     смене секунды.
   * Ошибка чтения DHT22 — повтор через 2 с, а не через 5 минут.
   * Нет delay(2000) в setup() — прогрев датчика не блокирует энкодер.
   * Время в uint32_t, сравнение разностей — переполнение millis() не страшно.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <EncButton.h>
#include <RTClib.h>

// ---------------- Настройки ----------------
constexpr uint8_t DHT_PIN  = 4;
constexpr uint8_t DHT_TYPE = DHT22;

constexpr uint8_t ENC_CLK = 25;
constexpr uint8_t ENC_DT  = 26;
constexpr uint8_t ENC_SW  = 14;

constexpr uint8_t SOIL_PIN = 32;               // ADC1 — работает вместе с WiFi
constexpr uint8_t PUMP_PIN = 27;

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_ROWS = 2;

// ---------------- Калибровка датчика почвы (мВ) ----------------
// Датчик ёмкостный, питание 3.3 В.
// Калибровка: откройте монитор порта (115200), опустите щуп в СУХУЮ землю/
// на воздух — запишите показанное значение, это SOIL_DRY_MV; затем в стакан
// с водой — это SOIL_WET_MV. Значения по умолчанию — типовые, их надо
// заменить на свои (у разных экземпляров разброс до ±300 мВ).
constexpr uint16_t SOIL_DRY_MV = 2500;         // сухо  (0 %)
constexpr uint16_t SOIL_WET_MV = 700;          // вода  (100 %)

// Диагностика линии: с резистором 100 кОм от GPIO32 к GND оборванный провод
// читается как ~0 мВ, а КЗ на 3.3 В — как насыщение АЦП.
constexpr uint16_t SOIL_FAULT_MV_LOW  = 100;   // ниже — обрыв/нет датчика
constexpr uint16_t SOIL_FAULT_MV_HIGH = 3050;  // выше — КЗ/насыщение

// Сколько отсчётов АЦП берём и сколько отбрасываем с каждого края
constexpr uint8_t SOIL_SAMPLES = 15;
constexpr uint8_t SOIL_TRIM    = 3;

// Пороги полива, %
constexpr int SOIL_PUMP_ON_PCT  = 20;          // суше — включаем помпу
constexpr int SOIL_PUMP_OFF_PCT = 30;          // влажнее — выключаем

// Релейный модуль с опторазвязкой — вход активный НИЗКИЙ.
constexpr uint8_t PUMP_ON_LEVEL = LOW;

constexpr uint32_t PUMP_MIN_ON_MS   = 5UL * 1000UL;    // минимум работы за раз
constexpr uint32_t PUMP_MAX_MS      = 30UL * 1000UL;   // максимум непрерывной работы
constexpr uint32_t PUMP_COOLDOWN_MS = 60UL * 1000UL;   // пауза после аварии

constexpr uint32_t INTERVAL_AIR   = 5UL * 60UL * 1000UL;  // воздух — 5 минут
constexpr uint32_t INTERVAL_SOIL  = 10UL * 1000UL;        // почва — 10 секунд
constexpr uint32_t CLOCK_REFRESH  = 250;                  // опрос DS3231
constexpr uint32_t RETRY_MS       = 2000;                 // повтор после ошибки DHT22
constexpr uint32_t WARMUP_MS      = 2000;                 // прогрев DHT22 после старта

constexpr char DEGREE = '\xDF';                // знак градуса в HD44780

// ---------------- Объекты ----------------
EncButton enc(ENC_CLK, ENC_DT, ENC_SW);
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);
RTC_DS3231 rtc;

enum class View : uint8_t { AIR, SOIL, CLOCK };
constexpr uint8_t VIEW_COUNT = 3;

constexpr const char* const WEEKDAYS[7] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

// ---------------- Состояние ----------------
View     view = View::AIR;
char     screen[LCD_ROWS][LCD_COLS + 1];       // то, что уже на экране

uint32_t nextAir   = 0;
uint32_t nextSoil  = 0;
uint32_t nextClock = 0;
uint32_t pumpStartedAt = 0;
uint32_t pumpLockUntil = 0;

float    temperature = 0;
float    humidity    = 0;
uint16_t soilMv      = 0;
int      soilPercent = 0;
DateTime clockNow;

bool airOk       = false;
bool soilOk      = false;
bool rtcOk       = false;
bool clockOk     = false;
bool pumpOn      = false;
bool screenDirty = false;

// ---------------- Помпа ----------------
void pumpSet(bool on) {
  if (on == pumpOn) return;
  pumpOn = on;
  digitalWrite(PUMP_PIN, on ? PUMP_ON_LEVEL : !PUMP_ON_LEVEL);
  screenDirty = true;
}

// Вызывается в каждом проходе loop(): и гистерезис, и защита по времени.
void updatePump(uint32_t now) {
  if (!pumpOn) {
    const bool ready = (int32_t)(now - pumpLockUntil) >= 0;
    if (ready && soilOk && soilPercent <= SOIL_PUMP_ON_PCT) {
      pumpStartedAt = now;
      pumpSet(true);
      Serial.printf("Pump: ON (soil %d %%)\n", soilPercent);
    }
    return;
  }

  const char* reason = nullptr;

  if (!soilOk) {                                          // датчик отвалился
    reason = "sensor error";
    pumpLockUntil = now + PUMP_COOLDOWN_MS;
  } else if (now - pumpStartedAt >= PUMP_MAX_MS) {        // слишком долго льём
    reason = "timeout";
    pumpLockUntil = now + PUMP_COOLDOWN_MS;
  } else if (now - pumpStartedAt >= PUMP_MIN_ON_MS &&
             soilPercent >= SOIL_PUMP_OFF_PCT) {          // уже достаточно мокро
    reason = "soil wet";
  }

  if (reason) {
    pumpSet(false);
    Serial.printf("Pump: OFF (%s), soil %d %%\n", reason, soilPercent);
  }
}

// ---------------- Вывод на LCD ----------------
// Пишем строку только если она отличается от показанной ранее.
void drawRow(uint8_t row, const char* text) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, text);   // добить пробелами до 16

  if (memcmp(buf, screen[row], LCD_COLS) == 0) return;  // изменений нет — не тратим I2C

  memcpy(screen[row], buf, LCD_COLS + 1);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void render() {
  char row0[LCD_COLS + 1];
  char row1[LCD_COLS + 1];
  row1[0] = '\0';

  switch (view) {
    case View::SOIL:
      if (soilOk) snprintf(row0, sizeof(row0), "Soil: %3d %%", soilPercent);
      else        snprintf(row0, sizeof(row0), "Soil: ---");

      snprintf(row1, sizeof(row1), "%4u mV %-4s", soilMv, pumpOn ? "PUMP" : "");
      break;

    case View::CLOCK:
      if (!clockOk) {
        snprintf(row0, sizeof(row0), "RTC not found");
      } else {
        snprintf(row0, sizeof(row0), "Time: %02d:%02d:%02d",
                 clockNow.hour(), clockNow.minute(), clockNow.second());
        snprintf(row1, sizeof(row1), "%02d.%02d.%04d %s",
                 clockNow.day(), clockNow.month(), clockNow.year(),
                 WEEKDAYS[clockNow.dayOfTheWeek()]);
      }
      break;

    case View::AIR:
    default:
      if (!airOk) {
        snprintf(row0, sizeof(row0), "DHT22 error");
      } else {
        snprintf(row0, sizeof(row0), "Temp: %4.1f%cC", temperature, DEGREE);
        snprintf(row1, sizeof(row1), "Hum:  %4.1f %%", humidity);
      }
      break;
  }

  drawRow(0, row0);
  drawRow(1, row1);
}

// ---------------- Датчик почвы ----------------
// Медианное (усечённое) среднее: 15 отсчётов, по 3 худших с каждого края
// отбрасываются. Без delay() — весь опрос занимает единицы миллисекунд.
uint16_t readSoilTrimmedMv() {
  uint16_t v[SOIL_SAMPLES];
  for (uint8_t i = 0; i < SOIL_SAMPLES; ++i) {
    v[i] = (uint16_t)analogReadMilliVolts(SOIL_PIN);
  }

  // сортировка вставками: 15 элементов — быстрее любого общего вызова
  for (uint8_t i = 1; i < SOIL_SAMPLES; ++i) {
    const uint16_t x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      --j;
    }
    v[j + 1] = x;
  }

  uint32_t sum = 0;
  for (uint8_t i = SOIL_TRIM; i < SOIL_SAMPLES - SOIL_TRIM; ++i) sum += v[i];

  return sum / (SOIL_SAMPLES - 2 * SOIL_TRIM);
}

void readSoil() {
  soilMv  = readSoilTrimmedMv();
  soilOk  = soilMv > SOIL_FAULT_MV_LOW && soilMv < SOIL_FAULT_MV_HIGH;

  soilPercent = soilOk
                  ? constrain(map(soilMv, SOIL_DRY_MV, SOIL_WET_MV, 0, 100), 0, 100)
                  : 0;

  screenDirty = true;

  if (view == View::SOIL || !soilOk) {
    Serial.printf("Soil: %d %%, %u mV%s\n", soilPercent, soilMv,
                  soilOk ? "" : " (sensor error)");
  }
}

// ---------------- Датчик воздуха ----------------
bool readAir() {
  // Второй вызов не читает шину повторно: Adafruit DHT кэширует замер ~2 с.
  const float t = dht.readTemperature();
  const float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    airOk       = false;
    screenDirty = true;
    Serial.println("DHT22 read error");
    return false;
  }

  temperature = t;
  humidity    = h;
  airOk       = true;
  screenDirty = true;

  Serial.printf("Temp: %.1f C, Hum: %.1f %%\n", t, h);
  return true;
}

// ---------------- Часы ----------------
void readClock() {
  if (!rtcOk) {
    clockOk = false;
    return;
  }

  const DateTime t = rtc.now();

  if (clockOk && t.unixtime() == clockNow.unixtime()) return;  // секунда не сменилась

  clockNow    = t;
  clockOk     = true;
  screenDirty = true;
}

// ---------------- Экраны ----------------
const char* viewName(View v) {
  switch (v) {
    case View::SOIL:  return "SOIL";
    case View::CLOCK: return "CLOCK";
    default:          return "AIR";
  }
}

void switchView(View v) {
  if (v == view) return;

  view        = v;
  screenDirty = true;

  const uint32_t now = millis();
  if (v == View::AIR)   nextAir   = now;   // свежие данные сразу
  if (v == View::SOIL)  nextSoil  = now;
  if (v == View::CLOCK) nextClock = now;

  Serial.printf("Screen: %s\n", viewName(v));
}

// ---------------- setup ----------------
void setup() {
  // Реле выключаем в первую очередь — до всего остального.
  digitalWrite(PUMP_PIN, !PUMP_ON_LEVEL);
  pinMode(PUMP_PIN, OUTPUT);

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);   // 0…~3.1 В

  lcd.init();
  lcd.backlight();
  lcd.clear();
  memset(screen, 0, sizeof(screen));

  dht.begin();

  rtcOk = rtc.begin();
  if (!rtcOk) {
    Serial.println("DS3231 not found (check SDA/SCL, 3.3V, address 0x68)");
  } else if (rtc.lostPower()) {
    // Батарейка села / часы не ходили — ставим время компиляции.
    // Вместо этого можно один раз выставить точное время вручную:
    //   rtc.adjust(DateTime(2026, 10, 21, 14, 30, 0));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("DS3231: time lost, set to build time");
  }

  nextAir   = millis() + WARMUP_MS;   // вместо delay(2000) — не блокируем loop()
  nextSoil  = millis();
  nextClock = millis();

  drawRow(0, "Init...");
}

// ---------------- loop ----------------
void loop() {
  enc.tick();                                   // энкодер опрашивается всегда

  if (enc.right()) {                            // вправо — следующий экран
    switchView(static_cast<View>((static_cast<uint8_t>(view) + 1) % VIEW_COUNT));
  } else if (enc.left()) {                      // влево — предыдущий
    switchView(static_cast<View>(
        (static_cast<uint8_t>(view) + VIEW_COUNT - 1) % VIEW_COUNT));
  }

  const uint32_t now = millis();

  // Почва опрашивается в любом режиме: от неё зависит полив.
  if ((int32_t)(now - nextSoil) >= 0) {
    nextSoil = now + INTERVAL_SOIL;
    readSoil();
  }

  if (view == View::AIR && (int32_t)(now - nextAir) >= 0) {
    nextAir = now + (readAir() ? INTERVAL_AIR : RETRY_MS);
  }

  if (view == View::CLOCK && (int32_t)(now - nextClock) >= 0) {
    nextClock = now + CLOCK_REFRESH;
    readClock();
  }

  updatePump(now);

  if (screenDirty) {
    render();
    screenDirty = false;
  }
}
