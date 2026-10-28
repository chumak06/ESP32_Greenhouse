/*
  ESP32: DHT22 + ёмкостный датчик почвы + помпа на реле + RTC DS3231
         + LCD 16x2 (I2C) + энкодер + веб-сервер

  Экранов три, переключаются энкодером по кругу
  (вправо — следующий, влево — предыдущий):

   1) AIR   — температура и влажность воздуха (DHT22), опрос раз в 5 мин
   2) SOIL  — влажность почвы, опрос раз в 10 с
   3) CLOCK — время и дата с DS3231

   Воздух и почва опрашиваются в любом режиме: данные нужны веб-серверу
   и логике полива, а не только открытому экрану.

  Web:
   * http://<ip-адрес>        — страница с показаниями (обновление раз в 2 с)
   * http://<ip-адрес>/api    — те же данные в JSON
   * http://dht22.local       — то же самое по mDNS (если сеть умеет)
   * POST /pump (state=on|off|auto) — ручное управление помпой;
     запрос защищён одноразовым токеном, страница получает его из /api.

  Полив (циклами, порциями):
   * влажность <= 20 % (SOIL_START_PCT) → начинаем сессию полива;
   * цикл: помпа 15 с (WATER_ON_MS), затем пауза 20 с (WATER_PAUSE_MS);
   * в паузе делаются 3 замера, берётся медиана: если она ниже 70 %
     (SOIL_TARGET_PCT) — ещё один цикл, и так до цели;
   * предохранители: не больше WATER_MAX_CYCLES (12) циклов = 3 мин воды
     на сессию, затем 10 минут паузы; после успеха — минута на впитывание;
   * помпа никогда не работает дольше PUMP_MAX_MS подряд (страховка);
   * обрыв/КЗ датчика почвы — полив блокируется;
   * одиночный выброс АЦП выключает помпу, но не даёт паузу 60 с —
     только серия из SOIL_FAULT_LOCK_AFTER ошибок подряд;
   * почва опрашивается всегда, при старте помпа выключена:
     уровень реле задаётся до pinMode(), иначе core 3.x его игнорирует;
   * ручной режим из web: ВКЛ — максимум MANUAL_ON_MS (датчик игнорируется),
     ВЫКЛ — автоматика на паузе MANUAL_OFF_MS, затем снова AUTO;
     ручная команда прерывает текущую сессию полива.

  Часы:
   * DS3231 на шине I2C (SDA 21, SCL 22), адрес 0x68 (LCD — 0x27);
   * при потере питания время ставится по времени компиляции.

  ВАЖНО: пороги SOIL_DRY_MV / SOIL_WET_MV — в милливольтах и зависят от
  экземпляра датчика. Как откалибровать — в комментарии под ними.

  Оптимизации:
   * analogReadMilliVolts() вместо сырого analogRead(): АЦП ESP32
     откалиброван по eFuse, шкала линейная и мВ не зависят от партии чипа.
   * Убран delay(10) в опросе АЦП: 15 быстрых отсчётов, по 3 крайних
     отбрасываются — без блокировки loop(), энкодер не «залипает».
   * lcd.clear() в цикле убран: пишется только изменившаяся строка ровно
     на 16 символов — нет мерцания и «хвостов» от старых значений.
   * Пока строка на LCD не изменилась, запись по I2C не выполняется.
   * WiFi подключается неблокирующе: loop() и энкодер работают во время
     соединения, при обрыве — повтор каждые 15 с.
   * Часы читаются 4 раза в секунду, LCD перерисовывается по смене секунды.
   * Ошибка чтения DHT22 — повтор через 2 с, а не через 5 минут.
   * Нет delay(2000) в setup() — прогрев датчика не блокирует энкодер.
   * Время в uint32_t, сравнение разностей — переполнение millis() не страшно.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <EncButton.h>
#include <RTClib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_random.h>

// ---------------- WiFi ----------------
// Логин и пароль — в secrets.h (он в .gitignore, на GitHub не попадает).
// Перед сборкой: скопируйте secrets.example.h в secrets.h и впишите свои данные.
// ESP32 работает только с сетями 2.4 ГГц.
#include "secrets.h"

constexpr char MDNS_NAME[] = "dht22";          // http://dht22.local

constexpr uint32_t WIFI_RETRY_MS  = 15000;     // пауза между попытками подключения
constexpr uint32_t WIFI_SCREEN_MS = 3000;      // сколько показывать IP на LCD

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
// ВАЖНО: 3050 мВ — это уже почти насыщение АЦП (11 дБ ≈ 3.1 В). Если у вашего
// датчика сухой выход выше ~2900 мВ, поднимите порог к 3080–3100, иначе
// исправный датчик будет считаться аварийным и полив заблокируется.

// Сколько подряд ошибочных чтений считать аварией датчика.
// Помпа выключается сразу при любом сбое, но пауза 60 с включается только
// после серии: одиночный выброс АЦП не должен останавливать полив на минуту.
constexpr uint8_t SOIL_FAULT_LOCK_AFTER = 3;

// Сколько отсчётов АЦП берём и сколько отбрасываем с каждого края
constexpr uint8_t SOIL_SAMPLES = 15;
constexpr uint8_t SOIL_TRIM    = 3;

// Пороги полива, %
constexpr int SOIL_START_PCT  = 20;   // суше — начинаем сессию полива
constexpr int SOIL_TARGET_PCT = 70;   // влажность в паузе достигла — сессия окончена

// Полив порциями: 15 с льём, 20 с ждём, пока вода впитается, и за это время
// делаем несколько замеров. Если медиана замеров всё ещё ниже цели — ещё цикл.
constexpr uint32_t WATER_ON_MS    = 15UL * 1000UL;   // помпа работает
constexpr uint32_t WATER_PAUSE_MS = 20UL * 1000UL;   // пауза на впитывание и замеры
constexpr uint8_t  WATER_MEASURES = 3;               // замеров за паузу (берём медиану)
constexpr uint32_t WATER_MEASURE_STEP = WATER_PAUSE_MS / (WATER_MEASURES + 1);  // 5 с
constexpr uint8_t  WATER_MAX_CYCLES = 12;            // предохранитель: 12 × 15 с = 3 мин

// Пауза автоматики после сессии: даём воде разойтись по грунту и не лить
// снова сразу. После исчерпания циклов пауза гораздо длиннее.
constexpr uint32_t WATER_SETTLE_MS = 60UL * 1000UL;        // 1 минута после успеха
constexpr uint32_t WATER_RETRY_MS  = 10UL * 60UL * 1000UL;  // 10 минут после лимита

// Релейный модуль с опторазвязкой — вход активный НИЗКИЙ.
constexpr uint8_t PUMP_ON_LEVEL = LOW;

constexpr uint32_t PUMP_MAX_MS      = 30UL * 1000UL;   // максимум непрерывной работы
constexpr uint32_t PUMP_COOLDOWN_MS = 60UL * 1000UL;   // пауза после аварии

// Ручное управление из веб-интерфейса.
// ВКЛ — принудительно, без проверки датчика, но не дольше этого времени.
// ВЫКЛ — запрет автоматики на это время (потом снова AUTO).
constexpr uint32_t MANUAL_ON_MS  = 60UL * 1000UL;          // 60 секунд
constexpr uint32_t MANUAL_OFF_MS = 30UL * 60UL * 1000UL;   // 30 минут

constexpr uint32_t INTERVAL_AIR  = 5UL * 60UL * 1000UL;   // воздух — 5 минут
constexpr uint32_t INTERVAL_SOIL = 10UL * 1000UL;         // почва — 10 секунд
constexpr uint32_t CLOCK_REFRESH = 250;                   // опрос DS3231
constexpr uint32_t RETRY_MS      = 2000;                  // повтор после ошибки DHT22
constexpr uint32_t WARMUP_MS     = 2000;                  // прогрев DHT22 после старта

constexpr char DEGREE = '\xDF';                // знак градуса в HD44780

// ---------------- Объекты ----------------
EncButton enc(ENC_CLK, ENC_DT, ENC_SW);
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);
RTC_DS3231 rtc;
WebServer server(80);

// Токен от CSRF для POST /pump: генерируется при каждом старте и вшивается
// в HTML. Чужой сайт не может прочитать страницу из-за same-origin policy,
// поэтому не сможет отправить команду помпе.
char webToken[9] = "00000000";

enum class View : uint8_t { AIR, SOIL, CLOCK };
constexpr uint8_t VIEW_COUNT = 3;

enum class PumpMode : uint8_t { AUTO, MANUAL_ON, MANUAL_OFF };

// Фаза автоматического полива: льём / ждём и меряем / ждём следующего раза.
enum class WaterState : uint8_t { IDLE, RUN, PAUSE };

constexpr const char* const WEEKDAYS[7] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

// Дата и время в виде строк — общие для экрана и для /api.
// Объявлено до первой функции: Arduino вставляет сюда же автопрототипы.
struct ClockText {
  char        time[9];      // "14:35:07"
  char        date[11];     // "27.10.2026"
  const char* wd;           // "Tue"
};

// ---------------- Состояние ----------------
View     view = View::AIR;
char     screen[LCD_ROWS][LCD_COLS + 1];       // то, что уже на экране

uint32_t nextAir     = 0;
uint32_t nextSoil    = 0;
uint32_t nextClock   = 0;
uint32_t nextWifiTry = 0;
uint32_t pumpStartedAt = 0;
uint32_t pumpLockUntil = 0;
uint32_t manualEndsAt  = 0;
uint32_t waterPauseUntil = 0;
uint32_t nextMeasureAt   = 0;

PumpMode pumpMode = PumpMode::AUTO;

WaterState waterState    = WaterState::IDLE;
uint8_t    waterCycle    = 0;                              // циклов в текущей сессии
uint8_t    waterSampleN  = 0;                              // валидных замеров в паузе
uint8_t    waterSamplePct[WATER_MEASURES] = {0};

float    temperature = 0;
float    humidity    = 0;
uint16_t soilMv      = 0;
int      soilPercent = 0;
uint8_t  soilFaultCount = 0;   // подряд ошибочных чтений почвы
DateTime clockNow;

bool airOk       = false;
bool soilOk      = false;
bool rtcOk       = false;
bool clockOk     = false;
bool pumpOn      = false;
bool netUp       = false;
bool mdnsStarted = false;
bool screenDirty = false;

// ---------------- Помпа ----------------
void pumpSet(bool on) {
  if (on == pumpOn) return;
  pumpOn = on;
  digitalWrite(PUMP_PIN, on ? PUMP_ON_LEVEL : !PUMP_ON_LEVEL);
  screenDirty = true;
}

const char* pumpModeName(PumpMode m) {
  switch (m) {
    case PumpMode::MANUAL_ON:  return "on";
    case PumpMode::MANUAL_OFF: return "off";
    default:                   return "auto";
  }
}

// Сколько секунд осталось до возврата в AUTO (0 — уже AUTO).
uint32_t manualLeftS(uint32_t now) {
  if (pumpMode == PumpMode::AUTO) return 0;
  const int32_t left = (int32_t)(manualEndsAt - now);
  return left > 0 ? (uint32_t)left / 1000UL : 0;
}

// Сбросить автоматическую сессию полива (ручная команда или авария).
void waterReset() {
  waterState   = WaterState::IDLE;
  waterCycle   = 0;
  waterSampleN = 0;
}

void pumpManualOn(uint32_t now) {
  waterReset();                     // ручная команда прерывает авто-сессию
  pumpMode      = PumpMode::MANUAL_ON;
  manualEndsAt  = now + MANUAL_ON_MS;
  pumpStartedAt = now;
  pumpSet(true);
  screenDirty = true;
  Serial.println("Pump: ON (manual)");
}

void pumpManualOff(uint32_t now) {
  waterReset();
  pumpMode      = PumpMode::MANUAL_OFF;
  manualEndsAt  = now + MANUAL_OFF_MS;
  pumpSet(false);
  pumpLockUntil = manualEndsAt;   // авто не включит, пока действует ручной ВЫКЛ
  screenDirty   = true;
  Serial.println("Pump: OFF (manual)");
}

void pumpAuto(uint32_t now) {
  waterReset();
  if (pumpMode == PumpMode::MANUAL_ON) {
    // Ручной полив уже шёл — останавливаем и даём почве впитать воду.
    pumpSet(false);
    pumpLockUntil = now + PUMP_COOLDOWN_MS;
  } else if (pumpMode == PumpMode::MANUAL_OFF) {
    pumpLockUntil = now;          // снять запрет автоматики
  }
  pumpMode    = PumpMode::AUTO;
  screenDirty = true;
  Serial.println("Pump: AUTO");
}

// Медиана замеров паузы: одиночный выброс АЦП не должен ни продлить полив,
// ни остановить его раньше времени.
uint8_t waterMedianPct() {
  uint8_t v[WATER_MEASURES];
  const uint8_t n = waterSampleN;
  for (uint8_t i = 0; i < n; ++i) v[i] = waterSamplePct[i];

  for (uint8_t i = 1; i < n; ++i) {          // вставками: n ≤ 3
    const uint8_t x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; --j; }
    v[j + 1] = x;
  }
  return n ? v[n / 2] : 0;
}

const char* waterStateName() {
  switch (waterState) {
    case WaterState::RUN:   return "on";
    case WaterState::PAUSE: return "pause";
    default:                return "idle";
  }
}

// Сколько секунд осталось до конца текущей фазы (0 — фазы нет).
uint32_t waterLeftS(uint32_t now) {
  int32_t left = 0;
  if (waterState == WaterState::RUN)        left = (int32_t)(pumpStartedAt + WATER_ON_MS - now);
  else if (waterState == WaterState::PAUSE) left = (int32_t)(waterPauseUntil - now);
  return left > 0 ? (uint32_t)left / 1000UL : 0;
}

// Очередная порция: помпа на WATER_ON_MS, дальше пауза с замерами.
void waterStartCycle(uint32_t now) {
  ++waterCycle;
  waterState    = WaterState::RUN;
  pumpStartedAt = now;
  waterSampleN  = 0;
  pumpSet(true);
  screenDirty = true;
  Serial.printf("Water: cycle %u/%u, on %lu s\n", waterCycle, WATER_MAX_CYCLES,
                (unsigned long)(WATER_ON_MS / 1000UL));
}

// Вызывается в каждом проходе loop(): ручные режимы, циклы полива, защита.
void updatePump(uint32_t now) {
  // Ручной режим имеет приоритет над автоматикой, но ограничен по времени.
  if (pumpMode == PumpMode::MANUAL_ON) {
    if ((int32_t)(now - manualEndsAt) >= 0) {
      pumpSet(false);
      pumpLockUntil = now + PUMP_COOLDOWN_MS;
      pumpMode = PumpMode::AUTO;
      screenDirty = true;
      Serial.println("Pump: OFF (manual timeout)");
      return;
    }
    if (!pumpOn) {                // страховка, если выход почему-то сбросился
      pumpStartedAt = now;
      pumpSet(true);
    }
    return;
  }

  if (pumpMode == PumpMode::MANUAL_OFF) {
    if (pumpOn) pumpSet(false);
    if ((int32_t)(now - manualEndsAt) >= 0) {
      pumpLockUntil = now;
      pumpMode = PumpMode::AUTO;
      screenDirty = true;
      Serial.println("Pump: AUTO (manual off timeout)");
    }
    return;
  }

  // --- Автоматика: циклы «15 с полив / 20 с пауза с замерами» ---
  if (waterState == WaterState::RUN) {
    if (!soilOk) {                       // датчик отвалился — стоп немедленно
      pumpSet(false);
      if (soilFaultCount >= SOIL_FAULT_LOCK_AFTER) {
        pumpLockUntil = now + PUMP_COOLDOWN_MS;
      }
      waterReset();
      Serial.println("Water: aborted (sensor)");
      return;
    }

    if (now - pumpStartedAt >= WATER_ON_MS) {
      pumpSet(false);
      waterState      = WaterState::PAUSE;
      waterPauseUntil = now + WATER_PAUSE_MS;
      nextMeasureAt   = now + WATER_MEASURE_STEP;
      waterSampleN    = 0;
      screenDirty     = true;
      Serial.println("Water: pause");
      return;
    }

    if (now - pumpStartedAt >= PUMP_MAX_MS) {   // страховка, сработать не должна
      pumpSet(false);
      waterReset();
      pumpLockUntil = now + PUMP_COOLDOWN_MS;
      Serial.println("Water: aborted (timeout)");
    }
    return;
  }

  if (waterState == WaterState::PAUSE) {
    // Замеры разнесены по паузе: вода успевает впитаться, а не остаётся
    // на поверхности. Неудачное чтение просто не попадает в выборку.
    if (waterSampleN < WATER_MEASURES && (int32_t)(now - nextMeasureAt) >= 0) {
      readSoil();
      if (soilOk) waterSamplePct[waterSampleN++] = (uint8_t)soilPercent;
      nextMeasureAt += WATER_MEASURE_STEP;   // от расписания, а не от «сейчас»
    }

    if ((int32_t)(now - waterPauseUntil) < 0) return;

    if (waterSampleN == 0) {             // датчик так и не ответил
      waterReset();
      Serial.println("Water: aborted (no samples)");
      return;
    }

    const uint8_t pct = waterMedianPct();

    if (pct >= SOIL_TARGET_PCT) {
      waterReset();
      pumpLockUntil = now + WATER_SETTLE_MS;
      Serial.printf("Water: done, soil %u %%\n", pct);
    } else if (waterCycle >= WATER_MAX_CYCLES) {
      waterReset();
      pumpLockUntil = now + WATER_RETRY_MS;
      Serial.printf("Water: cycle limit, soil %u %%\n", pct);
    } else {
      waterStartCycle(now);
    }
    return;
  }

  // IDLE: ждём, пока почва подсохнет ниже порога.
  const bool ready = (int32_t)(now - pumpLockUntil) >= 0;
  if (ready && soilOk && soilPercent < SOIL_START_PCT) {
    waterCycle = 0;
    waterStartCycle(now);
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

// ---------------- Часы ----------------
void clockText(ClockText& out) {
  if (!clockOk) {
    strcpy(out.time, "--:--:--");
    strcpy(out.date, "--.--.----");
    out.wd = "---";
    return;
  }

  snprintf(out.time, sizeof(out.time), "%02d:%02d:%02d",
           clockNow.hour(), clockNow.minute(), clockNow.second());
  snprintf(out.date, sizeof(out.date), "%02d.%02d.%04d",
           clockNow.day(), clockNow.month(), clockNow.year());
  out.wd = WEEKDAYS[clockNow.dayOfTheWeek()];
}

void render() {
  char row0[LCD_COLS + 1];
  char row1[LCD_COLS + 1];
  row1[0] = '\0';

  switch (view) {
    case View::SOIL: {
      if (soilOk) snprintf(row0, sizeof(row0), "Soil: %3d %%", soilPercent);
      else        snprintf(row0, sizeof(row0), "Soil: ---");

      const char* pumpTag = pumpOn
                              ? (pumpMode == PumpMode::MANUAL_ON ? "MAN" : "PUMP")
                              : (pumpMode == PumpMode::MANUAL_OFF
                                   ? "HOLD"
                                   : (waterState == WaterState::PAUSE ? "WAIT" : ""));

      snprintf(row1, sizeof(row1), "%4u mV %-4s", soilMv, pumpTag);
      break;
    }

    case View::CLOCK: {
      ClockText ct;
      clockText(ct);
      if (!clockOk) {
        snprintf(row0, sizeof(row0), "RTC not found");
      } else {
        snprintf(row0, sizeof(row0), "Time: %s", ct.time);
        snprintf(row1, sizeof(row1), "%s %s", ct.date, ct.wd);
      }
      break;
    }

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
  soilMv = readSoilTrimmedMv();
  soilOk = soilMv > SOIL_FAULT_MV_LOW && soilMv < SOIL_FAULT_MV_HIGH;

  if (soilOk) {
    soilFaultCount = 0;
  } else if (soilFaultCount < 255) {
    ++soilFaultCount;
  }

  soilPercent = soilOk
                  ? constrain(map(soilMv, SOIL_DRY_MV, SOIL_WET_MV, 0, 100), 0, 100)
                  : 0;

  screenDirty = true;

  if (view == View::SOIL || !soilOk) {
    const char* note = "";
    if (!soilOk) {
      note = (soilFaultCount >= SOIL_FAULT_LOCK_AFTER) ? " (sensor error)"
                                                       : " (glitch)";
    }
    Serial.printf("Soil: %d %%, %u mV%s\n", soilPercent, soilMv, note);
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

// ---------------- Часы (опрос) ----------------
void readClock() {
  if (!rtcOk) {
    clockOk = false;
    return;
  }

  const DateTime t = rtc.now();

  if (clockOk && t.unixtime() == clockNow.unixtime()) return;  // секунда не сменилась

  clockNow = t;
  clockOk  = true;

  // Экран перерисовываем только если он сейчас показан.
  if (view == View::CLOCK) screenDirty = true;
}

// ---------------- WiFi ----------------
void serviceWiFi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (!netUp) {
      netUp = true;
      Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());

      if (!mdnsStarted) {
        mdnsStarted = MDNS.begin(MDNS_NAME);
        if (mdnsStarted) {
          MDNS.addService("http", "tcp", 80);
          Serial.printf("mDNS: http://%s.local\n", MDNS_NAME);
        }
      }

      // Показать IP на экране на WIFI_SCREEN_MS — без delay():
      // просто откладываем первый замер почвы, который перерисует экран.
      drawRow(0, "WiFi OK");
      drawRow(1, WiFi.localIP().toString().c_str());
      nextSoil = now + WIFI_SCREEN_MS;
    }
    return;
  }

  if (netUp) {
    netUp = false;
    Serial.println("WiFi: connection lost");
  }

  if ((int32_t)(now - nextWifiTry) < 0) return;
  nextWifiTry = now + WIFI_RETRY_MS;

  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi: connecting...");
}

// ---------------- Web ----------------
const char INDEX_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 · датчики</title>
<style>
:root{--bg:#0f1419;--card:#1a2129;--fg:#e6edf3;--dim:#8b98a5;--ok:#3fb950;--bad:#f85149}
*{box-sizing:border-box}
body{margin:0;padding:20px;background:var(--bg);color:var(--fg);font:16px/1.45 system-ui,"Segoe UI",Arial,sans-serif}
h1{margin:0 0 16px;font-size:17px;font-weight:600;color:var(--dim)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;max-width:760px}
.card{background:var(--card);border-radius:12px;padding:14px 16px}
.k{font-size:12px;letter-spacing:.04em;text-transform:uppercase;color:var(--dim)}
.v{margin-top:4px;font-size:30px;font-weight:600;line-height:1.15}
.v small{font-size:15px;font-weight:400;color:var(--dim)}
.on{color:var(--ok)}.off{color:var(--dim)}.err{color:var(--bad)}
.btns{display:flex;gap:6px;margin-top:12px}
.btn{flex:1;padding:8px 4px;border:1px solid #2d3843;border-radius:8px;background:#222b34;
     color:var(--fg);font:14px system-ui,Arial,sans-serif;cursor:pointer}
.btn:hover{background:#2b3541}
.btn.on{border-color:var(--ok);color:var(--ok)}
.btn.off{border-color:var(--bad);color:var(--bad)}
footer{margin-top:16px;color:var(--dim);font-size:13px}
</style></head><body>
<h1>ESP32 · датчики и климат</h1>
<div class="grid">
<div class="card"><div class="k">Температура</div><div class="v" id="t">—</div></div>
<div class="card"><div class="k">Влажность воздуха</div><div class="v" id="h">—</div></div>
<div class="card"><div class="k">Влажность почвы</div><div class="v" id="s">—</div></div>
<div class="card"><div class="k">Помпа</div><div class="v" id="p">—</div>
<div class="btns">
<button class="btn on" onclick="setPump('on')" title="Включить вручную на 60 секунд">ВКЛ</button>
<button class="btn off" onclick="setPump('off')" title="Выключить и не поливать 30 минут">ВЫКЛ</button>
<button class="btn" onclick="setPump('auto')" title="Вернуть автоматический режим">АВТО</button>
</div></div>
<div class="card"><div class="k">Время</div><div class="v" id="tm">—</div></div>
<div class="card"><div class="k">Дата</div><div class="v" id="dt">—</div></div>
</div>
<footer id="st">подключение…</footer>
<script>
let TOKEN = "%TOKEN%";
const $ = id => document.getElementById(id);
const fmtUp = s => s < 3600 ? Math.floor(s / 60) + " мин"
                            : Math.floor(s / 3600) + " ч " + Math.floor(s % 3600 / 60) + " мин";
const fmtLeft = s => s >= 60 ? Math.floor(s / 60) + " мин" : s + " с";
function pumpHtml(d) {
  let s = d.pump ? '<span class="on">ВКЛ</span>' : '<span class="off">выкл</span>';
  let sub = "авто";
  if (d.mode === "on")
    sub = "ручной · " + d.left + " с";
  else if (d.mode === "off")
    sub = "ручной ВЫКЛ · " + fmtLeft(d.left);
  else if (d.ws === "on")
    sub = "полив · " + d.wleft + " с · цикл " + d.cyc;
  else if (d.ws === "pause")
    sub = "пауза · " + d.wleft + " с · замер";
  s += '<div class="k" style="margin-top:6px">' + sub + "</div>";
  return s;
}
async function setPump(state) {
  if (state === "on" && !confirm("Включить помпу вручную на 60 секунд?")) return;
  try {
    const r = await fetch("/pump", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: "state=" + encodeURIComponent(state) + "&token=" + encodeURIComponent(TOKEN)
    });
    if (!r.ok) throw new Error(r.status + " " + (await r.text()));
    await tick();
  } catch (e) {
    $("st").textContent = "помпа: " + e.message + " — обновите страницу";
  }
}
async function tick() {
  try {
    const r = await fetch("/api", { cache: "no-store" });
    const d = await r.json();
    // Токен приходит заново при каждом опросе: если ESP32 перезагрузилась
    // после открытия страницы, команда всё равно пройдёт.
    if (d.token) TOKEN = d.token;
    $("t").innerHTML = d.t_ok ? d.t.toFixed(1) + "<small> °C</small>" : '<span class="err">ошибка</span>';
    $("h").innerHTML = d.t_ok ? d.h.toFixed(1) + "<small> %</small>" : '<span class="err">ошибка</span>';
    $("s").innerHTML = d.soil_ok
      ? d.soil + "<small> %</small><div class=\"k\" style=\"margin-top:6px\">" + d.mv + " мВ</div>"
      : '<span class="err">датчик</span>';
    $("p").innerHTML = pumpHtml(d);
    $("tm").textContent = d.time;
    $("dt").textContent = d.date + " " + d.wd;
    $("st").textContent = "IP " + d.ip + " · RSSI " + d.rssi + " dBm · работает " + fmtUp(d.up);
  } catch (e) {
    $("st").textContent = "нет связи с ESP32";
  }
}
tick(); setInterval(tick, 2000);
</script></body></html>)html";

void handleRoot() {
  String page = INDEX_HTML;
  page.replace("%TOKEN%", webToken);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", page);
}

void handleApi() {
  ClockText ct;
  clockText(ct);

  char json[512];
  snprintf(json, sizeof(json),
           "{\"t\":%.1f,\"t_ok\":%s,\"h\":%.1f,\"soil\":%d,\"mv\":%u,"
           "\"soil_ok\":%s,\"pump\":%s,\"mode\":\"%s\",\"left\":%lu,"
           "\"ws\":\"%s\",\"cyc\":%u,\"wleft\":%lu,"
           "\"time\":\"%s\",\"date\":\"%s\","
           "\"wd\":\"%s\",\"up\":%lu,\"rssi\":%d,\"ip\":\"%s\",\"token\":\"%s\"}",
           airOk ? temperature : 0.0f,
           airOk ? "true" : "false",
           airOk ? humidity : 0.0f,
           soilOk ? soilPercent : 0,
           (unsigned)soilMv,
           soilOk ? "true" : "false",
           pumpOn ? "true" : "false",
           pumpModeName(pumpMode),
           (unsigned long)manualLeftS(millis()),
           waterStateName(),
           (unsigned)waterCycle,
           (unsigned long)waterLeftS(millis()),
           ct.time, ct.date, ct.wd,
           (unsigned long)(millis() / 1000UL),
           WiFi.RSSI(),
           WiFi.localIP().toString().c_str(),
           webToken);

  server.sendHeader("Cache-Control", "no-store, max-age=0");
  server.send(200, "application/json", json);
}

// POST /pump, body: state=on|off|auto + token=<токен>.
// Токен принимается и в заголовке X-Pump-Token (для ручных запросов).
// GET намеренно не поддерживаем: команду нельзя послать простой ссылкой
// или картинкой с чужого сайта.
void handlePump() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    server.send(405, "text/plain", "POST only");
    return;
  }

  // Токен ищем сначала в теле запроса: разбор POST-аргументов не зависит
  // от того, собирает ли WebServer заголовки.
  const String bodyTok = server.arg(String("token"));
  const String headTok = server.header(String("X-Pump-Token"));

  if (!bodyTok.equals(webToken) && !headTok.equals(webToken)) {
    // В ответе только длины, а не значения: так видно причину, но токен не утекает.
    char msg[64];
    snprintf(msg, sizeof(msg), "forbidden body=%u hdr=%u",
             (unsigned)bodyTok.length(), (unsigned)headTok.length());
    server.sendHeader("Cache-Control", "no-store");
    server.send(403, "text/plain", msg);
    return;
  }

  const String state = server.arg(String("state"));
  const uint32_t now = millis();

  if (state == "on")        pumpManualOn(now);
  else if (state == "off")  pumpManualOff(now);
  else if (state == "auto") pumpAuto(now);
  else {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"state\"}");
    return;
  }

  char resp[96];
  snprintf(resp, sizeof(resp),
           "{\"ok\":true,\"mode\":\"%s\",\"pump\":%s}",
           pumpModeName(pumpMode), pumpOn ? "true" : "false");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", resp);
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
  // В ESP32 core 3.x digitalWrite() до pinMode() игнорируется:
  // __digitalWrite() пишет только в пин, уже зарегистрированный как GPIO.
  // Поэтому сначала задаём уровень защёлки напрямую, потом включаем выход
  // (иначе после pinMode(OUTPUT) на активном низком реле помпа включится).
  gpio_set_level((gpio_num_t)PUMP_PIN, !PUMP_ON_LEVEL);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, !PUMP_ON_LEVEL);   // страховка: точно выключено
  pumpOn = false;

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
    // Точное время можно выставить один раз так:
    //   rtc.adjust(DateTime(2026, 10, 27, 14, 30, 0));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("DS3231: time lost, set to build time");
  }

  // WiFi: подключение неблокирующее, дальше им занимается serviceWiFi().
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // меньше задержки и помех АЦП (чуть больше ток)
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  nextWifiTry = millis() + WIFI_RETRY_MS;

  // CSRF-токен для ручного управления помпой, новый на каждый старт.
  snprintf(webToken, sizeof(webToken), "%08lx", (unsigned long)esp_random());

  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/pump", HTTP_POST, handlePump);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();

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

  serviceWiFi(now);
  server.handleClient();

  // Почва опрашивается в любом режиме: от неё зависит полив.
  if ((int32_t)(now - nextSoil) >= 0) {
    nextSoil = now + INTERVAL_SOIL;
    readSoil();
  }

  // Воздух тоже опрашивается в любом режиме: данные нужны не только экрану,
  // но и /api, иначе на SOIL/CLOCK они «замерзают» на последнем замере.
  if ((int32_t)(now - nextAir) >= 0) {
    nextAir = now + (readAir() ? INTERVAL_AIR : RETRY_MS);
  }

  // Часы читаем всегда — они нужны и на экране, и в /api.
  if ((int32_t)(now - nextClock) >= 0) {
    nextClock = now + CLOCK_REFRESH;
    readClock();
  }

  updatePump(now);

  if (screenDirty) {
    render();
    screenDirty = false;
  }
}
