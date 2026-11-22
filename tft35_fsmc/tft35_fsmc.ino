/*
 * ===========================================================================
 *  TFT35 V2.0 (BIGTREETECH) — монитор сервера
 *  STM32F103VCT6, экран ILI9488 480x320 на параллельной шине FSMC (16 бит)
 * ===========================================================================
 *  Плата: Generic STM32F1 series / Generic F103VCTx / Upload: STM32CubeProgrammer (SWD)
 *  Библиотека: Adafruit GFX Library
 *
 *  Распиновка (из исходников BTT, pin_TFT35_V2_0.h):
 *    LCD:   FSMC 16 бит  D0-D15: PD14 PD15 PD0 PD1 PE7..PE15 PD8 PD9 PD10
 *                        RD=PD4  WR=PD5  CS=PD7  RS=PE2
 *    Тач:   XPT2046 (программный SPI): CS=PC0 SCK=PB3 MISO=PB4 MOSI=PB5, касание = PC1 в LOW
 *    Зуммер PB2.  Подсветка: отдельного пина нет, горит всегда.
 *    ESP-01 (разъём WIFI): USART2, MCU RX=PA3, MCU TX=PA2, EN=PC4
 *    Отладка: разъём USART3, TX платы = PB10, 115200 (печатает координаты тача)
 *
 *  UI: на главном экране кнопка Menu (справа вверху). В меню:
 *    Touch Cal — калибровка тача (4 крестика по углам), сохраняется во flash;
 *    ESP-01    — состояние ESP, ввод адреса сервера Prometheus (экранная клавиатура),
 *                перезапуск ESP.
 *  Если своей калибровки нет, берётся калибровка родной прошивки BTT (из flash),
 *  если и её нет — калибровка запускается сама. Держи палец на экране при подаче
 *  питания — калибровка запустится принудительно.
 *
 *  Главный экран: температуры CPU/GPU1/SSD/GPU2 + полоски нагрузки (и RAM) с иконками; справа
 *  внизу плашка Docker (ок/всего) - нажми, откроется список контейнеров (также в Menu -> Docker).
 *
 *  Протокол от ESP-01 (строки с \n):  "CPU:63.4"  "GPU1:.." "SSD:.." "GPU2:.."
 *                                      "LCPU:23.5" "LGPU1:.." "LSSD:.." "LGPU2:.." "LRAM:.."  (нагрузка, %)
 *                                      "DK:всего;ок;внимание;плохо"  "DKI:<код>|<сек>|<рестарты>|<exit>|<имя>"  "DKEND:n"
 *                                      "ERR:DOCKER:<код>"
 *                                      "ALERT:CPU_OVERHEAT:95.0"
 *  Часы и 2-я страница:   "TIME:<секунд локального времени>"  "XUPT:<с>" "XDSKP:<% свободно>" "XDSKG:<ГБ>" "XNRX:<Мбит/с>" "XNTX:<Мбит/с>" "XTGT:<n>"
 *  Команды дисплея -> ESP:  "GETCFG"  "SETSRV:192.168.1.10:9090"  "SETDK:192.168.1.72:8099"  "DKNOW"  "SCAN"  "SETWIFI:<ssid>\t<пароль>"
 *  Ответы ESP -> дисплею:   "CFG:srv=ip:port;wifi=1;ip=x.x.x.x;rssi=-60;dk=ip:port;ssid=<имя>"  "ACK:SETSRV"  "ERR:SETSRV"
 *                           "NET:<rssi>;<1=с паролем>;<ssid>" (по строке на сеть)  "NETEND:<n>"
 *                           "ACK:WIFIOK" / "ERR:WIFI" (итог подключения)
 */
#include <Adafruit_GFX.h>
#include <EEPROM.h>

// ---------- экспорт истории на SD-карту (по умолчанию выключен) ----------
// 1 = на экране Graph появляется кнопка SD. Для сборки нужны ДВЕ библиотеки (Library Manager):
//     "STM32duino STM32SD" и "STM32duino FatFs". Слот SD на плате - это SDIO (PC8-PC12, PD2).
// Это самая непроверенная часть: я не мог запустить её на железе.
#define SD_EXPORT 0
#if SD_EXPORT
#include <STM32SD.h>
#endif

// ---------- пины ----------
#define BUZZER_PIN PB2
#define TOUCH_PEN  PC1
#define ESP_EN     PC4
#define T_CS       PC0
#define T_SCK      PB3
#define T_MISO     PB4
#define T_MOSI     PB5

// ======================= КОНФИГ GPU (настройки отображения) =======================
// При замене видеокарт меняется только этот блок. Подписи на экране (страницы 1 и 3,
// кнопки графиков), температурные пороги (>= WARN - жёлтая цифра, >= CRIT - красная),
// лимиты мощности (красная подсветка при превышении и шкала % / микрографиков).
// PWR_LIM = 0 отключает проверку. PSUM_LIM - ориентир для суммы мощности (БП).
static const char* GPU1_NAME = "GPU0";      // подпись 1-й видеокарты
static const char* GPU2_NAME = "GPU1";      // подпись 2-й видеокарты
static const float GPU1_TEMP_WARN = 75.0;   // температура Ti 1: жёлтая от
static const float GPU1_TEMP_CRIT = 85.0;   //                 красная от
static const float GPU2_TEMP_WARN = 75.0;   // температура Ti 2: жёлтая от
static const float GPU2_TEMP_CRIT = 85.0;   //                 красная от
static const float GPU1_PWR_LIM   = 260.0;  // лимит мощности Ti 1, Вт
static const float GPU2_PWR_LIM   = 260.0;  // лимит мощности Ti 2, Вт
static const float PSUM_LIM       = 850.0;  // лимит блока питания: потолок шкалы мощности и база для %, Вт
static const float DEF_TEMP_WARN  = 65.0;   // пороги остальных датчиков (CPU, SSD)
static const float DEF_TEMP_CRIT  = 80.0;

// ---------- батарея (Li-ion через делитель на АЦП) ----------
#define BAT_ENABLED     0
#define BAT_ADC_PIN     PA0        // разъём ENCODER, делитель: + батареи -> 47к -> PA0 -> 47к -> GND, 100 нФ между PA0 и GND
#define BAT_CHRG_PIN    PB8        // CHRG зарядника (TP4056 и т.п.), активный уровень LOW
#define BAT_STDBY_PIN   PB9        // STDBY зарядника, активный уровень LOW
#define BAT_DIV_X1000   2000       // коэффициент делителя x1000 ((R1+R2)/R2 = 2.000). Подгони по мультиметру
#define BAT_MIN_MV      2500       // ниже этого - считаем, что батареи нет (иконка скрыта)
#define BAT_MAX_MV      4600       // выше этого - тоже нет

// ---------- режим сна (экран гаснет при бездействии) ----------
#define SLEEP_DEFAULT_S 60                      // сон по умолчанию: через сколько секунд гасить экран (чёрная заливка). Настраивается в Menu -> Sleep (вкл/выкл и секунды), хранится во flash. Подсветка не гаснет, энергию это не экономит
#define SLEEP_LCD_CMD   0      // 1 = ещё и команды сна контроллера ILI9488; на некоторых панелях экран после них становится БЕЛЫМ
#define BL_ENABLED      0      // 1 = есть управляемая подсветка (нужна доработка платы, см. описание)
#define BL_PIN          PA10   // пин управления подсветкой через транзистор (HIGH = горит)
static bool sleeping = false;
static bool wakeReq = false;   // ESP прислал тревогу - разбудить экран
static unsigned long lastActMs = 0;

// настройка сна (Menu -> Sleep): хранится во flash рядом с калибровками (смещение 48)
struct SleepCfg { uint32_t magic; uint16_t secs; uint8_t enabled; uint8_t pad; };
static const uint32_t SLEEPCFG_MAGIC = 0x51EE0001;
static const uint16_t SLEEPCFG_OFFSET = 48;
static bool sleepEnabled = true;
static uint16_t sleepSecs = SLEEP_DEFAULT_S;

static void sleepLoad() {
  SleepCfg c;
  EEPROM.get(SLEEPCFG_OFFSET, c);
  if (c.magic == SLEEPCFG_MAGIC && c.secs >= 5 && c.secs <= 3600 && c.enabled <= 1) { sleepEnabled = c.enabled; sleepSecs = c.secs; }
}
static void sleepSave() {
  SleepCfg c; c.magic = SLEEPCFG_MAGIC; c.secs = sleepSecs; c.enabled = sleepEnabled ? 1 : 0; c.pad = 0;
  eeprom_buffer_fill();
  const uint8_t* p = (const uint8_t*)&c;
  for (uint16_t i = 0; i < sizeof(c); i++) eeprom_buffered_write_byte(SLEEPCFG_OFFSET + i, p[i]);
  eeprom_buffer_flush();
}

Uart Dbg(PB11, PB10);        // USART3: (RX, TX)
Uart EspSerial(PA3, PA2);    // USART2: (RX, TX)

// ---------- FSMC ----------
static volatile uint16_t* const LCD_REG = (volatile uint16_t*)0x60FFFFFE;  // RS=0 (команда)
static volatile uint16_t* const LCD_RAM = (volatile uint16_t*)0x61000000;  // RS=1 (данные)

static void gpioAfPp(GPIO_TypeDef* g, uint8_t pin) {
  volatile uint32_t* cr = (pin < 8) ? &g->CRL : &g->CRH;
  uint8_t sh = (pin & 7) * 4;
  *cr = (*cr & ~(0xFu << sh)) | (0xBu << sh);   // AF push-pull, 50 МГц
}

static void fsmcInit() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN;
  RCC->AHBENR  |= RCC_AHBENR_FSMCEN;

  const uint8_t pd[] = {14, 15, 0, 1, 8, 9, 10, 4, 5, 7, 11};
  const uint8_t pe[] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 2};
  for (uint8_t p : pd) gpioAfPp(GPIOD, p);
  for (uint8_t p : pe) gpioAfPp(GPIOE, p);

  // BCR1: MBKEN | тип NOR | ширина 16 бит | FACCEN | WREN | EXTMOD
  FSMC_Bank1->BTCR[0] = (1u << 0) | (2u << 2) | (1u << 4) | (1u << 6) | (1u << 12) | (1u << 14);
  // Тайминги (проверено, работает). Можно ускорять: уменьшай DATAST (число после << 8).
  FSMC_Bank1->BTCR[1]  = (3u << 0) | (0x0Fu << 8);   // чтение
  FSMC_Bank1E->BWTR[0] = (3u << 0) | (0x0Fu << 8);   // запись
}

static inline void wrReg(uint8_t r)   { *LCD_REG = r; }
static inline void wrData(uint16_t d) { *LCD_RAM = d; }
static inline uint16_t rdData()       { return *LCD_RAM; }

static uint16_t readId(uint8_t cmd) {
  wrReg(cmd);
  rdData();  rdData();
  uint16_t id = rdData();
  id <<= 8;
  id |= rdData();
  return id;
}

// ---------- драйвер экрана ----------
class FsmcLcd : public Adafruit_GFX {
 public:
  FsmcLcd() : Adafruit_GFX(480, 320) {}

  void begin() {
    fsmcInit();
    delay(50);
    uint8_t madctl = 0x28;             // ILI9488, ландшафт. Если вверх ногами: 0xE8
    if (readId(0xD3) != 0x9488 && readId(0xD4) == 0x5310) madctl = 0x60;  // NT35310

    wrReg(0xC0); wrData(0x0C); wrData(0x02);
    wrReg(0xC1); wrData(0x44);
    wrReg(0xC5); wrData(0x00); wrData(0x16); wrData(0x80);
    wrReg(0x36); wrData(madctl);
    wrReg(0x3A); wrData(0x55);         // 16 бит на пиксель (RGB565)
    wrReg(0xB0); wrData(0x00);
    wrReg(0xB1); wrData(0xB0);
    wrReg(0xB4); wrData(0x02);
    wrReg(0xB6); wrData(0x02); wrData(0x02);
    wrReg(0xE9); wrData(0x00);
    wrReg(0xF7); wrData(0xA9); wrData(0x51); wrData(0x2C); wrData(0x82);
    wrReg(0x11); delay(120);
    wrReg(0x29);
  }

  void sleepIn()  { wrReg(0x28); wrReg(0x10); delay(10); }
  void sleepOut() { wrReg(0x11); delay(120); wrReg(0x29); }

  void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    wrReg(0x2A); wrData(x0 >> 8); wrData(x0 & 0xFF); wrData(x1 >> 8); wrData(x1 & 0xFF);
    wrReg(0x2B); wrData(y0 >> 8); wrData(y0 & 0xFF); wrData(y1 >> 8); wrData(y1 & 0xFF);
    wrReg(0x2C);
  }

  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    setWindow(x, y, x, y);
    wrData(c);
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _width)  w = _width - x;
    if (y + h > _height) h = _height - y;
    if (w <= 0 || h <= 0) return;
    setWindow(x, y, x + w - 1, y + h - 1);
    uint32_t n = (uint32_t)w * h;
    while (n--) wrData(c);
  }

  void fillScreen(uint16_t c) override { fillRect(0, 0, _width, _height, c); }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { fillRect(x, y, w, 1, c); }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override { fillRect(x, y, 1, h, c); }
};

FsmcLcd tft;

#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_RED    0xF800
#define C_YELLOW 0xFFE0
#define C_GREY   0x7BEF
#define C_CYAN   0x07FF
#define C_ORANGE 0xFD20

// ---------- тач (XPT2046, программный SPI, как в прошивке BTT) ----------
static uint16_t xptRead(uint8_t cmd) {
  digitalWrite(T_CS, LOW);
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(T_MOSI, (cmd >> i) & 1);
    delayMicroseconds(1);
    digitalWrite(T_SCK, HIGH); delayMicroseconds(1);
    digitalWrite(T_SCK, LOW);  delayMicroseconds(1);
  }
  uint16_t v = 0;
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(T_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(T_MISO) ? 1 : 0);
    digitalWrite(T_SCK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(T_CS, HIGH);
  return v >> 4;                       // 12 бит
}

static uint16_t xptAvg(uint8_t cmd) {  // среднее из 5 чтений без min/max
  uint16_t b[5];
  for (uint8_t i = 0; i < 5; i++) b[i] = xptRead(cmd);
  uint16_t mn = b[0], mx = b[0]; uint32_t sum = 0;
  for (uint8_t i = 0; i < 5; i++) { if (b[i] < mn) mn = b[i]; if (b[i] > mx) mx = b[i]; sum += b[i]; }
  return (sum - mn - mx) / 3;
}

static bool touchPressed() { return digitalRead(TOUCH_PEN) == LOW; }

static void click() { digitalWrite(BUZZER_PIN, HIGH); delay(15); digitalWrite(BUZZER_PIN, LOW); }

// ---------- калибровка (хранится во flash через EEPROM-эмуляцию) ----------
struct TouchCal {
  uint32_t magic;
  int16_t  swap;            // 1: канал B (0x90) = ось X экрана
  int16_t  x0, x1, y0, y1;  // сырые значения на X=CAL_L / X=CAL_R и Y=CAL_T / Y=CAL_B
};
static const uint32_t CAL_MAGIC = 0xCA1B0002;
static TouchCal cal;
static bool calValid = false;

#define CAL_L 25
#define CAL_R 454
#define CAL_T 25
#define CAL_B 294

// (без параметра-структуры: автогенерация прототипов Arduino ставит их до объявления struct)
static bool calSane(uint32_t m, int x0, int x1, int y0, int y1) {
  return m == CAL_MAGIC && abs(x1 - x0) > 600 && abs(y1 - y0) > 600;
}

static void loadCal() {
  EEPROM.get(0, cal);
  calValid = calSane(cal.magic, cal.x0, cal.x1, cal.y0, cal.y1);
}

// Калибровка родной прошивки BTT лежит в последней странице flash (0x0803F800):
// слово 1 = метка 0x20200512, затем A,B,C,D,E,F,K (32 бит, старший байт первым).
// x = (A*tpX + B*tpY + C) / K,  y = (D*tpX + E*tpY + F) / K  (tpX = канал 0xD0, tpY = канал 0x90)
static int32_t bttPara[7];
static bool bttCalValid = false;

static int32_t beWord(const volatile uint8_t* p) {
  return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}

static void loadBttCal() {
  const volatile uint8_t* base = (const volatile uint8_t*)0x0803F800;
  if (beWord(base + 4) != (int32_t)0x20200512) return;
  for (uint8_t i = 0; i < 7; i++) bttPara[i] = beWord(base + 8 + 4 * i);
  bttCalValid = (bttPara[6] != 0);
}

static void saveCal() {
  eeprom_buffer_fill();
  const uint8_t* p = (const uint8_t*)&cal;
  for (uint16_t i = 0; i < sizeof(cal); i++) eeprom_buffered_write_byte(i, p[i]);
  eeprom_buffer_flush();
}

static bool touchRaw(uint16_t &a, uint16_t &b) {
  if (!touchPressed()) return false;
  a = xptAvg(0xD0);                    // как CMD_RDX в BTT
  b = xptAvg(0x90);                    // как CMD_RDY в BTT
  return touchPressed();               // палец убрали во время чтения
}

static bool touchRead(int16_t &sx, int16_t &sy, uint16_t &rawA, uint16_t &rawB) {
  if (!touchRaw(rawA, rawB)) return false;
  if (!calValid && bttCalValid) {            // калибровка из родной прошивки BTT
    int32_t tx = rawA, ty = rawB;
    int32_t lx = (bttPara[0] * tx + bttPara[1] * ty + bttPara[2]) / bttPara[6];
    int32_t ly = (bttPara[3] * tx + bttPara[4] * ty + bttPara[5]) / bttPara[6];
    sx = (int16_t)constrain(lx, 0, 479);
    sy = (int16_t)constrain(ly, 0, 319);
    return true;
  }
  if (!calValid) { sx = sy = 0; return true; }
  long x = cal.swap ? rawB : rawA;
  long y = cal.swap ? rawA : rawB;
  long lx = CAL_L + (x - cal.x0) * (CAL_R - CAL_L) / (cal.x1 - cal.x0);
  long ly = CAL_T + (y - cal.y0) * (CAL_B - CAL_T) / (cal.y1 - cal.y0);
  sx = (int16_t)constrain(lx, 0, 479);
  sy = (int16_t)constrain(ly, 0, 319);
  return true;
}

static void drawCross(int16_t x, int16_t y, uint16_t c) {
  tft.drawFastHLine(x - 14, y, 29, c);
  tft.drawFastVLine(x, y - 14, 29, c);
  tft.drawCircle(x, y, 8, c);
}

// ждём касания и усредняем сырые значения, пока палец прижат.
// Возвращает false, если касание было слишком коротким (мало замеров)
static bool sampleTouch(uint16_t &ra, uint16_t &rb) {
  while (!touchPressed()) delay(10);
  delay(40);                           // дать пальцу устояться
  uint32_t sa = 0, sb = 0; uint8_t n = 0;
  uint16_t a, b;
  while (n < 20 && touchRaw(a, b)) { sa += a; sb += b; n++; delay(5); }
  while (touchPressed()) delay(10);    // ждём отпускания
  delay(150);
  if (n < 3) { ra = rb = 0; return false; }
  ra = sa / n; rb = sb / n;
  return true;
}

static void runCalibration() {
  const int16_t tx[4] = {CAL_L, CAL_R, CAL_R, CAL_L};   // левый-верх, правый-верх, правый-низ, левый-низ
  const int16_t ty[4] = {CAL_T, CAL_T, CAL_B, CAL_B};
  uint16_t A[4], B[4];
  while (true) {
    for (uint8_t i = 0; i < 4; i++) {
      while (touchPressed()) delay(10);      // палец с прошлого касания/кнопки не считается
      delay(200);
      tft.fillScreen(C_BLACK);
      tft.setTextColor(C_WHITE, C_BLACK);
      tft.setTextSize(2);
      tft.setCursor(110, 140); tft.print("Calibration "); tft.print(i + 1); tft.print("/4");
      tft.setCursor(80, 170);  tft.print("Touch the center of the cross");
      drawCross(tx[i], ty[i], C_RED);
      while (!sampleTouch(A[i], B[i])) {
        tft.setTextColor(C_YELLOW, C_BLACK);
        tft.setCursor(60, 200); tft.print("Too short - press and hold");
      }
      click();
    }
    // определяем, какой канал отвечает за горизонталь: сравниваем левый-верх и правый-верх
    TouchCal c; c.magic = CAL_MAGIC;
    int dA = abs((int)A[1] - (int)A[0]), dB = abs((int)B[1] - (int)B[0]);
    c.swap = (dB > dA) ? 1 : 0;
    const uint16_t* X = c.swap ? B : A;     // канал по горизонтали
    const uint16_t* Y = c.swap ? A : B;     // канал по вертикали
    c.x0 = (X[0] + X[3]) / 2;  c.x1 = (X[1] + X[2]) / 2;
    c.y0 = (Y[0] + Y[1]) / 2;  c.y1 = (Y[2] + Y[3]) / 2;
    // оси должны быть перпендикулярны: при движении вдоль X «чужой» канал почти не меняется
    int crossH = abs((int)Y[1] - (int)Y[0]), crossV = abs((int)X[3] - (int)X[0]);
    int mainH  = abs((int)X[1] - (int)X[0]), mainV  = abs((int)Y[3] - (int)Y[0]);
    bool orth = (crossH * 3 < mainH) && (crossV * 3 < mainV);
    if (orth && calSane(c.magic, c.x0, c.x1, c.y0, c.y1)) {
      cal = c; calValid = true; saveCal();
      tft.fillScreen(C_BLACK);
      tft.setTextColor(C_GREEN, C_BLACK); tft.setTextSize(3);
      tft.setCursor(100, 20); tft.print("Calibration OK");
      tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
      const char* nm[4] = {"TL", "TR", "BR", "BL"};
      for (uint8_t i = 0; i < 4; i++) {
        tft.setCursor(30, 80 + i * 28);
        tft.print(nm[i]); tft.print(" A="); tft.print(A[i]); tft.print(" B="); tft.print(B[i]);
      }
      tft.setCursor(30, 200); tft.print("swap="); tft.print(cal.swap);
      tft.setCursor(30, 228); tft.print("x0="); tft.print(cal.x0); tft.print(" x1="); tft.print(cal.x1);
      tft.setCursor(30, 256); tft.print("y0="); tft.print(cal.y0); tft.print(" y1="); tft.print(cal.y1);
      tft.setTextColor(C_GREY, C_BLACK);
      tft.setCursor(30, 292); tft.print("Touch to continue");
      Dbg.print("cal swap="); Dbg.print(cal.swap);
      Dbg.print(" x0="); Dbg.print(cal.x0); Dbg.print(" x1="); Dbg.print(cal.x1);
      Dbg.print(" y0="); Dbg.print(cal.y0); Dbg.print(" y1="); Dbg.println(cal.y1);
      delay(500);
      while (!touchPressed()) delay(10);
      while (touchPressed()) delay(10);
      delay(200);
      return;
    }
    // показать, почему отвергнуто
    tft.fillScreen(C_BLACK);
    tft.setTextColor(C_RED, C_BLACK); tft.setTextSize(2);
    tft.setCursor(30, 20); tft.print(orth ? "Bad points: span too small" : "Bad points: axes not orthogonal");
    tft.setTextColor(C_WHITE, C_BLACK);
    const char* nm[4] = {"TL", "TR", "BR", "BL"};
    for (uint8_t i = 0; i < 4; i++) {
      tft.setCursor(30, 60 + i * 28);
      tft.print(nm[i]); tft.print(" A="); tft.print(A[i]); tft.print(" B="); tft.print(B[i]);
    }
    tft.setCursor(30, 180); tft.print("swap="); tft.print(c.swap);
    tft.setCursor(30, 208); tft.print("mainH="); tft.print(mainH); tft.print(" crossH="); tft.print(crossH);
    tft.setCursor(30, 236); tft.print("mainV="); tft.print(mainV); tft.print(" crossV="); tft.print(crossV);
    tft.setTextColor(C_GREY, C_BLACK);
    tft.setCursor(30, 280); tft.print("Touch to retry");
    delay(500);
    while (!touchPressed()) delay(10);
    while (touchPressed()) delay(10);
    delay(200);
  }
}

// ---------- данные ----------
float cpuTemp = 0, gpuTemp = 0, ssdTemp = 0, p102Temp = 0;
static float loadV[5] = {-1, -1, -1, -1, -1};   // нагрузка, %: CPU, GPU, SSD, P102, RAM (-1 = нет данных)
#define VAL_STALE_MS 20000UL                     // значение старше этого считается устаревшим и показывается как "--"
static unsigned long tempAt[4] = {0, 0, 0, 0};   // когда последний раз пришло каждое значение (0 = ещё не приходило)
static unsigned long loadAt[5] = {0, 0, 0, 0, 0};

// ---- 2-я страница: XUPT аптайм (с), XDSKP свободно (%), XDSKG свободно (ГБ), XNRX / XNTX сеть (Мбит/с), XTGT упавших targets
static float xV[6] = {-1, -1, -1, -1, -1, -1};
static unsigned long xAt[6] = {0, 0, 0, 0, 0, 0};
// ---- 3-я страница, мощность в Вт: PSUM (оценка суммы, не из розетки), PCPU, PRAM (память CPU), P1060, PP102
static float pV[5] = {-1, -1, -1, -1, -1};
static float pMax[5] = {0, 0, 0, 0, 0};          // максимум с момента включения дисплея
static unsigned long pAt[5] = {0, 0, 0, 0, 0};
#define X_STALE_MS 60000UL                      // доп. метрики приходят раз в ~15 с

// ---- часы: ESP присылает "TIME:<локальные секунды>", дальше идём по millis()
static long clkBase = 0;
static unsigned long clkMs = 0;
static bool clkValid() { return clkBase != 0; }
static long nowLocal() { return clkBase + (long)((millis() - clkMs) / 1000UL); }

// секунды с 1970 (локальные) -> дата и время; wd: 0 = воскресенье
static void epochToDate(long t, int &yr, int &mo, int &d, int &hh, int &mi, int &ss, int &wd) {
  if (t < 0) t = 0;
  long days = t / 86400L, rem = t % 86400L;
  hh = (int)(rem / 3600L); mi = (int)(rem % 3600L / 60L); ss = (int)(rem % 60L);
  wd = (int)((days + 4) % 7);
  long z = days + 719468L;
  long era = z / 146097L;
  long doe = z - era * 146097L;
  long yoe = (doe - doe / 1460L + doe / 36524L - doe / 146096L) / 365L;
  long y = yoe + era * 400L;
  long doy = doe - (365L * yoe + yoe / 4L - yoe / 100L);
  long mp = (5L * doy + 2L) / 153L;
  d = (int)(doy - (153L * mp + 2L) / 5L + 1L);
  long m = (mp < 10) ? mp + 3 : mp - 9;
  yr = (int)(y + (m <= 2 ? 1 : 0));
  mo = (int)m;
}
static const char* const WD_NAME[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const char* const MO_NAME[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static String two(int v) { return (v < 10) ? (String("0") + String(v)) : String(v); }
bool alertActive = false, alertDismissed = false;
String alertName = "";
unsigned long alertMs = 0;
#define ALERT_SLOTS 4                           // какие тревоги уже озвучены (иначе две тревоги подряд пищали бы в каждом цикле)
static String alertSeen[ALERT_SLOTS];
static unsigned long alertSeenMs[ALERT_SLOTS];
const unsigned long ALERT_HOLD_MS = 10000;   // тревога гаснет, если ESP 10 с не повторяет ALERT
String alertText = "";

// ---- журнал тревог (последние JRN_N, новые первыми) и отложенные тревоги (snooze)
#define JRN_N 8
static char jrnTxt[JRN_N][28];
static long jrnTs[JRN_N];                       // локальное время (0 = часов не было)
static unsigned long jrnMs[JRN_N];
static uint8_t jrnHead = 0, jrnCnt = 0;         // jrnHead - куда писать следующую
static uint16_t jrnRev = 0;
static unsigned long snoozeUntil = 0;           // 0 = не отложено

static bool snoozeActive() {
  if (snoozeUntil == 0) return false;
  if ((long)(snoozeUntil - millis()) > 0) return true;
  snoozeUntil = 0;
  return false;
}
static void jrnAdd(const String &t) {
  uint8_t i = jrnHead;
  t.substring(0, 27).toCharArray(jrnTxt[i], sizeof(jrnTxt[i]));
  jrnTs[i] = clkValid() ? nowLocal() : 0;
  jrnMs[i] = millis();
  jrnHead = (jrnHead + 1) % JRN_N;
  if (jrnCnt < JRN_N) jrnCnt++;
  jrnRev++;
}

unsigned long lastDataMs = 0, lastDrawMs = 0;
const unsigned long DATA_STALE_MS = 15000;
String serialLine = "";
bool dirty = true;

// ---- состояние ESP-01 (ответы на GETCFG) ----
static String espSrv = "", espIp = "", espSsid = "", espDk = "";
static int espWifi = -1, espRssi = 0;
static unsigned long espCfgMs = 0;

// результат поиска сетей (ответы NET:/NETEND: на команду SCAN)
#define MAX_NETS 12
static String netSsid[MAX_NETS];
static int netRssi[MAX_NETS];
static uint8_t netEnc[MAX_NETS];
static uint8_t netCount = 0;
static uint8_t scanState = 0;          // 0 - нет, 1 - идёт, 2 - готово, 3 - ESP не ответил
static unsigned long scanReqMs = 0;
static String espErrText = "";         // ошибка опроса от ESP (ERR:PROM / ERR:QUERY / ERR:NOWIFI)
static unsigned long espErrMs = 0;
static const unsigned long ERR_HOLD_MS = 10000;
static int wifiRes = 0;                // 0 - ждём, 1 - подключено, -1 - ошибка

// ---- Docker (данные от docker-status через ESP) ----
#define DK_MAX 24
static char     dkCode[DK_MAX];
static uint32_t dkSecs[DK_MAX];
static uint16_t dkRst[DK_MAX];
static int16_t  dkExit[DK_MAX];
static char     dkName[DK_MAX][24];
static uint8_t  dkN = 0, dkBuildN = 0;
static int      dkTotal = 0, dkOk = 0, dkWarn = 0, dkBad = 0;
static bool     dkValid = false;
static unsigned long dkMs = 0;                 // когда пришёл последний полный кадр
static uint16_t dkRev = 0;
static String   dkErrText = "";
static unsigned long dkErrMs = 0;
static const unsigned long DK_FRESH_MS = 45000;   // дольше без обновлений - данные считаются устаревшими

static void dkParseSum(const String &s) {         // "14;12;0;2"
  int v[4] = {0, 0, 0, 0};
  int pos = 0;
  for (uint8_t i = 0; i < 4; i++) {
    int e = s.indexOf(';', pos);
    String f = (e < 0) ? s.substring(pos) : s.substring(pos, e);
    v[i] = f.toInt();
    if (e < 0) break;
    pos = e + 1;
  }
  dkTotal = v[0]; dkOk = v[1]; dkWarn = v[2]; dkBad = v[3];
  dkBuildN = 0;                                   // начало нового кадра
}

static void dkParseItem(const String &s) {       // "U|1800|3|0|bs-api"
  if (dkBuildN >= DK_MAX || s.length() < 9 || s.length() > 70) return;
  int p[4]; int pos = -1;
  for (uint8_t i = 0; i < 4; i++) {
    p[i] = s.indexOf('|', pos + 1);
    if (p[i] < 0) return;
    pos = p[i];
  }
  char c = s[0];
  if (p[0] != 1 || !strchr("OSPURX", c)) return;
  uint8_t k = dkBuildN;
  long rs = s.substring(p[1] + 1, p[2]).toInt();
  dkCode[k] = c;
  dkSecs[k] = (uint32_t)s.substring(2, p[1]).toInt();
  dkRst[k]  = (uint16_t)(rs < 0 ? 0 : (rs > 9999 ? 9999 : rs));
  dkExit[k] = (int16_t)s.substring(p[2] + 1, p[3]).toInt();
  s.substring(p[3] + 1).toCharArray(dkName[k], sizeof(dkName[k]));
  dkBuildN++;
}

// "-60;1;MyNetwork"
static void parseNet(String s) {
  int a = s.indexOf(';'), b = s.indexOf(';', a + 1);
  if (a < 0 || b < 0 || netCount >= MAX_NETS) return;
  netRssi[netCount] = s.substring(0, a).toInt();
  netEnc[netCount]  = s.substring(a + 1, b).toInt();
  netSsid[netCount] = s.substring(b + 1);
  netCount++;
}

// "srv=192.168.1.10:9090;wifi=1;ip=192.168.1.55;rssi=-60"
static void parseCfg(String s) {
  // принимаем только целую, правдоподобную строку; повреждённую (потерянные байты) молча отбрасываем
  if (s.length() > 190 || !s.startsWith("srv=")) return;
  for (unsigned i = 0; i < s.length(); i++) if ((uint8_t)s[i] < 32) return;
  String tSrv, tIp, tSsid, tDk; int tWifi = -1, tRssi = 0; uint8_t got = 0;
  int si = s.indexOf(";ssid=");                       // ssid всегда последний (в нём могут быть ';')
  if (si >= 0) { tSsid = s.substring(si + 6); s = s.substring(0, si); got |= 16; }
  int pos = 0;
  while (pos < (int)s.length()) {
    int e = s.indexOf(';', pos); if (e < 0) e = s.length();
    String kv = s.substring(pos, e); pos = e + 1;
    int q = kv.indexOf('='); if (q < 0) continue;
    String k = kv.substring(0, q), v = kv.substring(q + 1);
    if (k == "srv")       { tSrv = v; got |= 1; }
    else if (k == "wifi") { tWifi = v.toInt(); got |= 2; }
    else if (k == "ip")   { tIp = v; got |= 4; }
    else if (k == "rssi") { tRssi = v.toInt(); got |= 8; }
    else if (k == "dk")   { tDk = v; }
  }
  if ((got & 15) != 15 || tSrv.length() > 27 || tIp.length() > 15 || tSsid.length() > 32 || tDk.length() > 27) return;
  espSrv = tSrv; espWifi = tWifi; espIp = tIp; espRssi = tRssi; espDk = tDk;
  if (got & 16) espSsid = tSsid;
  espCfgMs = millis();
}

void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.startsWith("CFG:")) { parseCfg(line.substring(4)); return; }
  if (line.startsWith("NET:"))    { parseNet(line.substring(4)); return; }
  if (line.startsWith("NETEND:")) { scanState = 2; return; }
  if (line == "ACK:WIFIOK")       { wifiRes = 1;  return; }
  if (line == "ERR:WIFI")         { wifiRes = -1; return; }
  if (line.startsWith("ERR:PROM")) {
    espErrText = "PROMETHEUS UNREACHABLE";
    int c = line.indexOf(':', 8);                       // "ERR:PROM:-11" -> код ошибки HTTP
    if (c > 0) espErrText += " (" + line.substring(c + 1) + ")";
    espErrMs = millis(); return;
  }
  if (line == "ERR:NOWIFI")       { espErrText = "ESP: NO WI-FI";          espErrMs = millis(); return; }
  if (line.startsWith("ERR:QUERY:")) { espErrText = "NO DATA: " + line.substring(10); espErrMs = millis(); return; }
  if (line.startsWith("ERR:DOCKER")) {
    dkErrText = "docker-status error";
    int c = line.indexOf(':', 10);                      // "ERR:DOCKER:-1"
    if (c > 0) dkErrText += " " + line.substring(c + 1);
    dkErrMs = millis(); return;
  }
  if (line.startsWith("TIME:")) {
    long v = line.substring(5).toInt();
    if (v > 1600000000L && v < 2100000000L) { clkBase = v; clkMs = millis(); }
    return;
  }
  if (line.startsWith("DKI:"))  { dkParseItem(line.substring(4)); return; }
  if (line.startsWith("DKEND:")) {
    if (dkBuildN == line.substring(6).toInt()) {        // кадр целый - принимаем; иначе оставляем прежние данные
      dkN = dkBuildN; dkValid = true; dkMs = millis(); dkErrMs = 0; dkRev++;
    }
    return;
  }
  if (line.startsWith("DK:"))   { dkParseSum(line.substring(3)); return; }
  if (line.startsWith("ACK:") || line.startsWith("ERR:") || line.startsWith("STATUS:")) { Dbg.println(line); return; }
  lastDataMs = millis();
  dirty = true;
  if (line.startsWith("ALERT:")) {
    // ESP повторяет ALERT в каждом цикле, пока значение выше порога.
    // Сигналим только на НОВУЮ тревогу (другое имя) или если прошлая уже погасла.
    String t = line.substring(6);
    int lc = t.lastIndexOf(':');
    String name = (lc > 0) ? t.substring(0, lc) : t;     // "CPU_OVERHEAT" без значения; "DOWN bs-db"
    unsigned long now = millis();
    int slot = -1, freeSlot = -1;
    for (uint8_t i = 0; i < ALERT_SLOTS; i++) {
      if (alertSeen[i] == name) slot = i;
      else if (freeSlot < 0 && (alertSeen[i].length() == 0 || now - alertSeenMs[i] > ALERT_HOLD_MS)) freeSlot = i;
    }
    bool freshAlert = slot >= 0 && now - alertSeenMs[slot] <= ALERT_HOLD_MS;
    bool snoozed = snoozeActive();
    if (!freshAlert) {                                   // новая (или давно погасшая) тревога - в журнал и сигналим
      alertDismissed = false;
      jrnAdd(t);
      if (!snoozed) { digitalWrite(BUZZER_PIN, HIGH); delay(80); digitalWrite(BUZZER_PIN, LOW); }
      if (slot < 0) slot = (freeSlot >= 0) ? freeSlot : 0;
    }
    alertSeen[slot] = name; alertSeenMs[slot] = now;
    alertName = name; alertText = t; alertMs = now;
    alertActive = !alertDismissed && !snoozed;
    if (alertActive) wakeReq = true;
    return;
  }
  int sep = line.indexOf(':');
  if (sep < 0) return;
  String key = line.substring(0, sep);
  float val = line.substring(sep + 1).toFloat();
  unsigned long now = millis();
  if (key == "CPU")       { cpuTemp = val;  tempAt[0] = now; }
  else if (key == "GPU1")  { gpuTemp = val;  tempAt[1] = now; }
  else if (key == "SSD")  { ssdTemp = val;  tempAt[2] = now; }
  else if (key == "GPU2") { p102Temp = val; tempAt[3] = now; }
  else if (key == "LCPU") { loadV[0] = val; loadAt[0] = now; }
  else if (key == "LGPU1") { loadV[1] = val; loadAt[1] = now; }
  else if (key == "LSSD") { loadV[2] = val; loadAt[2] = now; }
  else if (key == "LGPU2") { loadV[3] = val; loadAt[3] = now; }
  else if (key == "LRAM") { loadV[4] = val; loadAt[4] = now; }
  else if (key == "XUPT") { xV[0] = val; xAt[0] = now; }
  else if (key == "XDSKP") { xV[1] = val; xAt[1] = now; }
  else if (key == "XDSKG") { xV[2] = val; xAt[2] = now; }
  else if (key == "XNRX") { xV[3] = val; xAt[3] = now; }
  else if (key == "XNTX") { xV[4] = val; xAt[4] = now; }
  else if (key == "XTGT") { xV[5] = val; xAt[5] = now; }
  else if (key == "PSUM") { pV[0] = val; pAt[0] = now; if (val > pMax[0]) pMax[0] = val; }
  else if (key == "PCPU") { pV[1] = val; pAt[1] = now; if (val > pMax[1]) pMax[1] = val; }
  else if (key == "PRAM") { pV[2] = val; pAt[2] = now; if (val > pMax[2]) pMax[2] = val; }
  else if (key == "PGU1") { pV[3] = val; pAt[3] = now; if (val > pMax[3]) pMax[3] = val; }
  else if (key == "PGU2") { pV[4] = val; pAt[4] = now; if (val > pMax[4]) pMax[4] = val; }
}

static bool forceRedraw = true;              // true = перерисовать все поля (после drawStatic)
static String lastVal[4];
static String lastStatus;

#define C_BTN 0x2104   // тёмно-серый фон кнопок

static void drawButton(int16_t x, int16_t y, int16_t w, int16_t h, const char* label, uint16_t col) {
  tft.fillRect(x, y, w, h, C_BTN);
  tft.drawRect(x, y, w, h, col);
  tft.setTextSize(2); tft.setTextColor(col, C_BTN);
  int16_t tw = strlen(label) * 12;
  tft.setCursor(x + (w - tw) / 2, y + (h - 16) / 2);
  tft.print(label);
}

static bool inRect(int16_t px, int16_t py, int16_t x, int16_t y, int16_t w, int16_t h) {
  return px >= x && px < x + w && py >= y && py < y + h;
}

// одно событие на одно касание (координаты уточняются через 30 мс после касания)
static bool getTap(int16_t &x, int16_t &y) {
  static bool wasDown = false;
  static unsigned long lastMs = 0;
  int16_t sx, sy; uint16_t ra, rb;
  if (touchRead(sx, sy, ra, rb)) {
    lastMs = millis();
    lastActMs = millis();
    if (!wasDown) {
      wasDown = true;
      delay(30);
      touchRead(sx, sy, ra, rb);               // повторный замер; если не удался - остаются прежние
      x = sx; y = sy;
      click();
      return true;
    }
  } else if (wasDown && millis() - lastMs > 150) {
    wasDown = false;
  }
  return false;
}

// ======================= ИСТОРИЯ (ОЗУ) =======================
// Запись идёт всегда, на любом экране, в трёх разрешениях: 15 с x 240 = 1 ч, 3 мин x 240 = 12 ч, 18 мин x 240 = 72 ч.
// Ряды: 0..3 температуры (CPU, GPU0, SSD, GPU1), 4..8 нагрузка (CPU, GPU0, SSD, GPU1, RAM),
// 9..13 мощность (PSUM, PCPU, PRAM, PGU1, PGU2) - в октets хранится Вт * POW_HIST_SCALE.
// Значение хранится как градусы/проценты x 2 (0..254), 255 = нет данных. После перезагрузки история пустая.
#define H_N 240
#define H_SER 14
#define POW_HIST_SCALE 0.5f                     // мощность в истории: байт = Вт * 0.5 (потолок 508 Вт, шаг 2 Вт)
#define POW_HIST_FIRST 9                        // первый ряд мощности в истории
#define HIST_STEP_MS 15000UL
static uint8_t  hist[3][H_SER][H_N];
static uint16_t hHead[3] = {0, 0, 0};            // куда писать следующий отсчёт
static uint16_t hCnt[3]  = {0, 0, 0};            // сколько отсчётов накоплено
static uint16_t hRev = 0;                         // растёт при каждом новом отсчёте (экран графика перерисуется)
static unsigned long histMs = 0;
static uint8_t  histN0 = 0, histN1 = 0;
static uint16_t acc1[H_SER], acc2[H_SER];
static uint8_t  cnt1[H_SER], cnt2[H_SER];

static uint8_t histEnc(float v) {
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  return (uint8_t)(v * 2.0f + 0.5f);
}

// мощность: байт = Вт * POW_HIST_SCALE (0..508 Вт при 0.5), 255 = нет данных
static uint8_t histEncPow(float w) {
  if (w < 0) return 255;
  if (w > 254.0f / POW_HIST_SCALE) w = 254.0f / POW_HIST_SCALE;
  return (uint8_t)(w * POW_HIST_SCALE + 0.5f);
}
static float histDecPow(uint8_t e) { return (e == 255) ? -1.0f : (e / POW_HIST_SCALE); }

static void histPush(uint8_t lv, const uint8_t* e) {
  for (uint8_t i = 0; i < H_SER; i++) hist[lv][i][hHead[lv]] = e[i];
  hHead[lv] = (hHead[lv] + 1) % H_N;
  if (hCnt[lv] < H_N) hCnt[lv]++;
}

static uint8_t histGet(uint8_t lv, uint16_t k, uint8_t sr) {     // k-й отсчёт с конца (0 = самый новый)
  if (k >= hCnt[lv]) return 255;
  return hist[lv][sr][(hHead[lv] + H_N - 1 - k) % H_N];
}

static void histTick() {
  unsigned long now = millis();
  if (now - histMs < HIST_STEP_MS) return;
  histMs = (now - histMs < 2 * HIST_STEP_MS) ? histMs + HIST_STEP_MS : now;     // после долгой паузы - без догоняющей очереди
  uint8_t e[H_SER];
  float tv[4] = {cpuTemp, gpuTemp, ssdTemp, p102Temp};
  for (uint8_t i = 0; i < 4; i++) {
    bool ok = tempAt[i] != 0 && now - tempAt[i] <= VAL_STALE_MS;
    e[i] = ok ? histEnc(tv[i]) : 255;
  }
  for (uint8_t i = 0; i < 5; i++) {
    bool ok = loadAt[i] != 0 && now - loadAt[i] <= VAL_STALE_MS;
    e[4 + i] = ok ? histEnc(loadV[i]) : 255;
  }
  for (uint8_t i = 0; i < 5; i++) {
    bool ok = pAt[i] != 0 && now - pAt[i] <= X_STALE_MS;
    e[POW_HIST_FIRST + i] = ok ? histEncPow(pV[i]) : 255;
  }
  histPush(0, e);
  for (uint8_t i = 0; i < H_SER; i++) if (e[i] != 255) { acc1[i] += e[i]; cnt1[i]++; }
  if (++histN0 >= 12) {                                          // 12 x 15 с = 3 мин
    histN0 = 0;
    uint8_t e1[H_SER];
    for (uint8_t i = 0; i < H_SER; i++) {
      e1[i] = cnt1[i] ? (uint8_t)((acc1[i] + cnt1[i] / 2) / cnt1[i]) : 255;
      acc1[i] = 0; cnt1[i] = 0;
    }
    histPush(1, e1);
    for (uint8_t i = 0; i < H_SER; i++) if (e1[i] != 255) { acc2[i] += e1[i]; cnt2[i]++; }
    if (++histN1 >= 6) {                                         // 6 x 3 мин = 18 мин
      histN1 = 0;
      uint8_t e2[H_SER];
      for (uint8_t i = 0; i < H_SER; i++) {
        e2[i] = cnt2[i] ? (uint8_t)((acc2[i] + cnt2[i] / 2) / cnt2[i]) : 255;
        acc2[i] = 0; cnt2[i] = 0;
      }
      histPush(2, e2);
    }
  }
  hRev++;
}

static void pollEsp() {
  while (EspSerial.available()) {
    char c = EspSerial.read();
    if (c == '\n') { parseLine(serialLine); serialLine = ""; }
    else if (c != '\r' && serialLine.length() < 200) serialLine += c;
  }
  histTick();
}

// ======================= БАТАРЕЯ =======================
static const uint16_t BAT_TAB_MV[11] = {3300, 3600, 3700, 3750, 3790, 3820, 3870, 3920, 3980, 4060, 4200};
static const uint8_t  BAT_TAB_PC[11] = {   0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100};
static long batRaw = 0;            // сглаженное напряжение батареи БЕЗ калибровки, мВ (0 - ещё не измеряли)
static long batMv = 0;             // то же с калибровкой, мВ
static long batSig = -1;           // что нарисовано сейчас (чтобы не перерисовывать без нужды)
static int  batShown = -1;         // показанный процент (с гистерезисом)
static unsigned long batMs = 0;

// калибровка: поправочный коэффициент в промилле (1000 = без поправки), хранится во flash рядом с калибровкой тача
struct BatCal { uint32_t magic; uint16_t permille; uint16_t pad; };
static const uint32_t BATCAL_MAGIC = 0xBA7C0001;
static const uint16_t BATCAL_OFFSET = 32;
static uint16_t batPermille = 1000;

#if BAT_ENABLED
static void batLoadCal() {
  BatCal b;
  EEPROM.get(BATCAL_OFFSET, b);
  if (b.magic == BATCAL_MAGIC && b.permille >= 800 && b.permille <= 1200) batPermille = b.permille;
}
static void batSaveCal() {
  BatCal b; b.magic = BATCAL_MAGIC; b.permille = batPermille; b.pad = 0;
  eeprom_buffer_fill();
  const uint8_t* p = (const uint8_t*)&b;
  for (uint16_t i = 0; i < sizeof(b); i++) eeprom_buffered_write_byte(BATCAL_OFFSET + i, p[i]);
  eeprom_buffer_flush();
}

// одно измерение: среднее по n отсчётам АЦП, напряжение батареи в мВ (без калибровки)
static long batMeasureRawMv(uint8_t n) {
  long sum = 0;
  for (uint8_t i = 0; i < n; i++) { sum += analogRead(BAT_ADC_PIN); delayMicroseconds(200); }
  long raw = sum / n;                                           // 12 бит
  long pinMv = raw * 3300L / 4095L;
  return pinMv * BAT_DIV_X1000 / 1000;
}
#endif

static int batPercent(long mv) {   // кусочно-линейная кривая разряда Li-ion
  if (mv <= BAT_TAB_MV[0]) return 0;
  for (int i = 1; i < 11; i++) {
    if (mv <= BAT_TAB_MV[i]) {
      long a = BAT_TAB_MV[i - 1], b = BAT_TAB_MV[i];
      return BAT_TAB_PC[i - 1] + (int)((mv - a) * (BAT_TAB_PC[i] - BAT_TAB_PC[i - 1]) / (b - a));
    }
  }
  return 100;
}

static bool batShowPct = false;    // false - иконка, true - проценты (переключается нажатием, не запоминается)

static void batDraw() {
#if BAT_ENABLED
  if (sleeping) return;
  bool present = batMv >= BAT_MIN_MV && batMv <= BAT_MAX_MV;
  bool chg = present && digitalRead(BAT_CHRG_PIN) == LOW;
  bool full = present && !chg && digitalRead(BAT_STDBY_PIN) == LOW;
  int pc = present ? batPercent(batMv) : 0;
  if (present) {                   // гистерезис: показание меняется, только если сдвинулось хотя бы на 2%
    if (batShown < 0 || abs(pc - batShown) >= 2) batShown = pc;
    pc = batShown;
  } else batShown = -1;
  if (full) pc = 100;
  long sig = present ? (pc * 8 + (chg ? 1 : 0) + (full ? 2 : 0) + (batShowPct ? 4 : 0)) : -2;
  if (sig == batSig) return;
  batSig = sig;
  tft.fillRect(286, 6, 80, 49, C_BLACK);
  if (!present) return;
  uint16_t col = (pc >= 30) ? C_GREEN : (pc >= 15 ? C_YELLOW : C_RED);
  if (chg) col = C_YELLOW;
  if (batShowPct) {
    String t = String(pc) + "%";
    tft.setTextSize(3); tft.setTextColor(col, C_BLACK);
    tft.setCursor(286 + (80 - (int16_t)t.length() * 18) / 2, 10);
    tft.print(t);
  } else {
    int16_t x = 296, y = 10, w = 48, h = 24;
    tft.drawRect(x, y, w, h, C_WHITE);
    tft.fillRect(x + w, y + 7, 4, h - 14, C_WHITE);            // "носик"
    int16_t fw = (int16_t)((w - 6) * (long)pc / 100);
    tft.fillRect(x + 3, y + 3, w - 6, h - 6, C_BLACK);
    if (fw > 0) tft.fillRect(x + 3, y + 3, fw, h - 6, col);
  }
  if (chg || full) {               // под иконкой / процентами - состояние зарядки
    const char* st = chg ? "CHG" : "FULL";
    tft.setTextSize(2); tft.setTextColor(chg ? C_YELLOW : C_GREEN, C_BLACK);
    tft.setCursor(286 + (80 - (int16_t)strlen(st) * 12) / 2, 38);
    tft.print(st);
  }
#endif
}

static void batTick() {            // вызывать из loop() на главном экране
#if BAT_ENABLED
  if (millis() - batMs < 2000) return;
  batMs = millis();
  long mv = batMeasureRawMv(32);
  batRaw = (batRaw == 0) ? mv : (batRaw * 7 + mv) / 8;          // плавное сглаживание
  batMv = batRaw * batPermille / 1000;
  batDraw();
#endif
}

// ======================= ИКОНКИ (рисуются линиями, 28x28) =======================
static void icoCpu(int16_t x, int16_t y, uint16_t c) {
  tft.drawRect(x + 6, y + 6, 16, 16, c);
  tft.fillRect(x + 10, y + 10, 8, 8, c);
  for (int8_t i = 0; i < 3; i++) {                              // ножки со всех сторон
    int16_t o = 9 + i * 5;
    tft.drawFastVLine(x + o, y + 1, 5, c);  tft.drawFastVLine(x + o, y + 22, 5, c);
    tft.drawFastHLine(x + 1, y + o, 5, c);  tft.drawFastHLine(x + 22, y + o, 5, c);
  }
}

static void icoGpu(int16_t x, int16_t y, uint16_t c, bool twin) {   // видеокарта; twin = два вентилятора
  tft.drawRect(x + 1, y + 6, 26, 15, c);
  tft.fillRect(x + 1, y + 3, 3, 21, c);                         // планка-кронштейн
  if (twin) { tft.drawCircle(x + 12, y + 13, 4, c); tft.drawCircle(x + 21, y + 13, 4, c); }
  else      { tft.drawCircle(x + 15, y + 13, 5, c); tft.fillRect(x + 14, y + 12, 3, 3, c); }
  tft.fillRect(x + 8, y + 22, 12, 3, c);                        // контакты
}

static void icoSsd(int16_t x, int16_t y, uint16_t c) {
  tft.drawRect(x + 2, y + 5, 24, 18, c);
  tft.fillRect(x + 6, y + 9, 12, 4, c);                         // наклейка
  tft.fillRect(x + 6, y + 16, 5, 2, c);
  tft.fillCircle(x + 21, y + 18, 2, c);                         // светодиод
}

static void icoRam(int16_t x, int16_t y, uint16_t c) {
  tft.drawRect(x + 1, y + 7, 26, 12, c);
  for (int8_t i = 0; i < 4; i++) tft.fillRect(x + 4 + i * 6, y + 10, 4, 6, c);   // микросхемы
  for (int8_t i = 0; i < 6; i++) tft.fillRect(x + 3 + i * 4, y + 20, 2, 4, c);   // контакты
}

static void icoDocker(int16_t x, int16_t y, uint16_t c) {       // стопка контейнеров на волне
  for (int8_t i = 0; i < 4; i++) tft.fillRect(x + 2 + i * 6, y + 15, 5, 5, c);
  for (int8_t i = 0; i < 3; i++) tft.fillRect(x + 5 + i * 6, y + 9, 5, 5, c);
  for (int8_t i = 0; i < 2; i++) tft.fillRect(x + 8 + i * 6, y + 3, 5, 5, c);
  tft.drawFastHLine(x + 1, y + 22, 26, c);
  tft.drawFastHLine(x + 5, y + 24, 18, c);
}

// ======================= ГЛАВНЫЙ ЭКРАН =======================
static const int16_t ROW_Y0 = 64, ROW_DY = 45;                 // 5 строк сверху вниз: CPU, RAM, GPU1, GPU2, SSD
static const char* const ROW_LBL[5] = {"CPU", "RAM", GPU1_NAME, GPU2_NAME, "SSD"};    // видеокарты - из конфига GPU
static const char* const ROW_LBL2[5] = {"UPT", "DISK", "RX", "TX", "TGT"};            // 2-я страница
static const char* const ROW_LBL3[5] = {"SUM", "CPU", "DRAM", GPU1_NAME, GPU2_NAME};  // 3-я страница (мощность)
static const int8_t ROW_TEMP[5] = {0, -1, 1, 3, 2};            // какая температура в строке (индекс в tv[]: CPU, GPU, SSD, P102), -1 = нет
static const int8_t ROW_LOAD[5] = {0, 4, 1, 3, 2};             // какая нагрузка (индекс в loadV[]: CPU, GPU, SSD, P102, RAM)
static int16_t lastLoad[5];
static String lastDkW;
static String fmtDur(uint32_t t);                       // определены ниже
static String padTo(String s, uint8_t n);

static void rowIcon(uint8_t i, int16_t y) {
  int16_t x = 10;
  y -= 2;
  switch (i) {
    case 0: icoCpu(x, y, C_CYAN); break;
    case 1: icoRam(x, y, C_CYAN); break;
    case 2: icoGpu(x, y, C_CYAN, false); break;
    case 3: icoGpu(x, y, C_CYAN, true); break;
    default: icoSsd(x, y, C_CYAN); break;
  }
}

static uint16_t tempColLim(float v, float warn, float crit) { return v >= crit ? C_RED : (v >= warn ? C_YELLOW : C_GREEN); }
// пороги по индексу датчика: 0 = CPU, 1 = GPU1, 2 = SSD, 3 = GPU2 (см. tv[] в drawValues)
static uint16_t tempColIdx(uint8_t idx, float v) {
  if (idx == 1) return tempColLim(v, GPU1_TEMP_WARN, GPU1_TEMP_CRIT);
  if (idx == 3) return tempColLim(v, GPU2_TEMP_WARN, GPU2_TEMP_CRIT);
  return tempColLim(v, DEF_TEMP_WARN, DEF_TEMP_CRIT);
}
static uint16_t loadCol(int16_t pct) { return pct >= 90 ? C_RED : (pct >= 65 ? C_YELLOW : C_CYAN); }

// плашка Docker внизу справа: иконка + "ок/всего"; цвет = худшее состояние
static void dkWidgetDraw() {
  bool fresh = dkValid && (millis() - dkMs < DK_FRESH_MS);
  String txt = fresh ? (String(dkOk) + "/" + String(dkTotal)) : String("--");
  uint16_t col = !fresh ? C_GREY : (dkBad > 0 ? C_RED : (dkWarn > 0 ? C_YELLOW : C_GREEN));
  String sig = txt + "|" + String((int)col);
  if (!forceRedraw && sig == lastDkW) return;
  lastDkW = sig;
  tft.drawRect(344, 282, 132, 34, C_GREY);
  tft.fillRect(345, 283, 130, 32, C_BLACK);
  icoDocker(350, 285, col);
  tft.setTextSize(2); tft.setTextColor(col, C_BLACK);
  tft.setCursor(386, 292);
  tft.print(txt);
}

// ---- иконки 2-й страницы
static void icoClock(int16_t x, int16_t y, uint16_t c) {
  tft.drawCircle(x + 14, y + 14, 11, c);
  tft.drawCircle(x + 14, y + 14, 10, c);
  tft.drawFastVLine(x + 14, y + 7, 8, c);
  tft.drawFastHLine(x + 14, y + 14, 6, c);
}

static void icoArrow(int16_t x, int16_t y, uint16_t c, bool down) {
  if (down) {
    tft.fillRect(x + 11, y + 3, 6, 12, c);
    tft.fillTriangle(x + 4, y + 14, x + 24, y + 14, x + 14, y + 25, c);
  } else {
    tft.fillRect(x + 11, y + 13, 6, 12, c);
    tft.fillTriangle(x + 4, y + 14, x + 24, y + 14, x + 14, y + 3, c);
  }
}

static void icoTarget(int16_t x, int16_t y, uint16_t c) {
  tft.drawCircle(x + 14, y + 14, 11, c);
  tft.drawCircle(x + 14, y + 14, 6, c);
  tft.fillCircle(x + 14, y + 14, 2, c);
}

static void icoBolt(int16_t x, int16_t y, uint16_t c) {         // молния
  tft.fillTriangle(x + 17, y + 2, x + 6, y + 15, x + 15, y + 15, c);
  tft.fillTriangle(x + 11, y + 13, x + 22, y + 13, x + 12, y + 26, c);
}

static void rowIcon3(uint8_t i, int16_t y) {
  int16_t x = 10;
  y -= 2;
  switch (i) {
    case 0: icoBolt(x, y, C_CYAN); break;
    case 1: icoCpu(x, y, C_CYAN); break;
    case 2: icoRam(x, y, C_CYAN); break;
    case 3: icoGpu(x, y, C_CYAN, false); break;
    default: icoGpu(x, y, C_CYAN, true); break;
  }
}

static void rowIcon2(uint8_t i, int16_t y) {
  int16_t x = 10;
  y -= 2;
  switch (i) {
    case 0: icoClock(x, y, C_CYAN); break;
    case 1: icoSsd(x, y, C_CYAN); break;
    case 2: icoArrow(x, y, C_CYAN, true); break;
    case 3: icoArrow(x, y, C_CYAN, false); break;
    default: icoTarget(x, y, C_CYAN); break;
  }
}

static uint8_t curPage = 0;                    // 0 - температуры и нагрузка, 1 - аптайм / диск / сеть / targets
static String lastX[5];
static String lastP[5];
static int16_t lastDiskPct = -2;
static String clockSig;

// центральная часть экрана (строки): иконки и подписи выбранной страницы
static void drawBody() {
  tft.fillRect(1, 56, 478, 224, C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  for (uint8_t i = 0; i < 5; i++) {
    int16_t ry = ROW_Y0 + i * ROW_DY;
    if (curPage == 0) rowIcon(i, ry); else if (curPage == 1) rowIcon2(i, ry); else rowIcon3(i, ry);
    tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
    tft.setCursor(44, ry + 2); tft.print(curPage == 0 ? ROW_LBL[i] : (curPage == 1 ? ROW_LBL2[i] : ROW_LBL3[i]));
  }
  for (uint8_t i = 0; i < 4; i++) lastVal[i] = "";
  for (uint8_t i = 0; i < 5; i++) { lastLoad[i] = -2; lastX[i] = ""; }
  lastDiskPct = -2;
  for (uint8_t i = 0; i < 5; i++) lastP[i] = "";
  forceRedraw = true;
}

static void drawPageDots() {                   // точки страниц слева от строки статуса
  for (uint8_t i = 0; i < 3; i++) {
    int16_t cx = 8 + i * 8;
    tft.fillCircle(cx, 298, 4, C_BLACK);
    if (i == curPage) tft.fillCircle(cx, 298, 3, C_WHITE); else tft.drawCircle(cx, 298, 3, C_GREY);
  }
}

// ---- иконка Wi-Fi в шапке (по RSSI из ответов GETCFG на ESP; дисплей спрашивает CFG раз в 10 с)
static uint8_t wifiIcoSig = 0xFF;              // 0xFF = перерисовать в любом случае
static void wifiIconDraw() {
  bool alive = espCfgMs != 0 && millis() - espCfgMs < 15000 && espWifi == 1;
  uint8_t bars = 0; uint16_t col = C_GREY;
  if (alive) {
    bars = (espRssi >= -55) ? 4 : (espRssi >= -65) ? 3 : (espRssi >= -75) ? 2 : 1;
    col = (bars >= 3) ? C_GREEN : (bars == 2 ? C_YELLOW : C_RED);
  }
  uint8_t sig = bars | (col == C_GREEN ? 0x10 : col == C_YELLOW ? 0x20 : col == C_RED ? 0x30 : (uint8_t)0x00);
  if (!forceRedraw && sig == wifiIcoSig) return;
  wifiIcoSig = sig;
  const int16_t ix = 246, iy = 28;             // 4 столбика вверх от iy, высотой 4/8/12/16
  tft.fillRect(ix - 2, iy - 18, 26, 20, C_BLACK);
  for (uint8_t b = 0; b < 4; b++) {
    int16_t h = 4 + b * 4;
    int16_t x = ix + b * 5;
    if (b < bars) tft.fillRect(x, iy - h, 3, h, col);
    else tft.drawRect(x, iy - h, 3, h, C_GREY);
  }
}

static void drawClock() {                      // время и дата в шапке (вместо надписи Server Monitor)
  int yr = 0, mo = 1, d = 1, hh = 0, mi = 0, ss = 0, wd = 0;
  String sig = "--";
  if (clkValid()) {
    epochToDate(nowLocal(), yr, mo, d, hh, mi, ss, wd);
    sig = two(hh) + ":" + two(mi) + " " + String(wd) + "," + String(d) + "," + String(mo);
  }
  if (!forceRedraw && sig == clockSig) { wifiIconDraw(); return; }
  clockSig = sig;
  tft.fillRect(10, 6, 272, 46, C_BLACK);
  wifiIcoSig = 0xFF;                           // заливка стёрла иконку - перерисовать
  tft.setTextSize(4);
  tft.setCursor(16, 12);
  if (!clkValid()) { tft.setTextColor(C_GREY, C_BLACK); tft.print("--:--"); wifiIconDraw(); return; }
  tft.setTextColor(C_WHITE, C_BLACK); tft.print(two(hh) + ":" + two(mi));
  tft.setTextSize(2); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(156, 12); tft.print(WD_NAME[wd]);
  tft.setCursor(156, 32); tft.print(String(d) + " " + MO_NAME[mo - 1]);
  wifiIconDraw();
}

void drawStatic() {
  forceRedraw = true;
  batSig = -1;
  tft.fillScreen(C_BLACK);
  tft.drawFastHLine(10, 55, 460, C_GREY);
  tft.drawRect(0, 0, 480, 320, C_GREY);       // рамка: видно ли экран целиком
  clockSig = "";
  drawBody();
  drawPageDots();
  drawButton(372, 8, 100, 40, "Menu", C_WHITE);
#if !BAT_ENABLED
  drawButton(286, 8, 80, 40, "Graph", C_WHITE);
#endif
}

static void printVal(uint8_t idx, int16_t rowY, float v) {   // printf("%f") в Newlib Nano не работает
  bool fresh = tempAt[idx] != 0 && millis() - tempAt[idx] <= VAL_STALE_MS;   // нет свежих данных - "--" серым
  uint16_t col = fresh ? tempColIdx(idx, v) : C_GREY;
  String t = fresh ? (String(v, 1) + " C") : String("--");
  while (t.length() < 7) t += ' ';
  String key = t + String((int)col);
  if (!forceRedraw && key == lastVal[idx]) return;           // не менялось - не трогаем (не стираем точки)
  lastVal[idx] = key;
  tft.setTextSize(3); tft.setTextColor(col, C_BLACK);
  tft.setCursor(122, rowY + 2);
  tft.print(t);
}

// нагрузка на 1-й странице: проценты числом (полоски убраны, справа микрографик - см. sparkDraw)
static void printLoad(uint8_t idx, int16_t rowY, float v) {  // v < 0 = нет данных
  if (loadAt[idx] == 0 || millis() - loadAt[idx] > VAL_STALE_MS) v = -1;
  int16_t pct = (v < 0) ? -1 : (int16_t)(v + 0.5f);
  if (pct > 100) pct = 100;
  if (!forceRedraw && pct == lastLoad[idx]) return;
  lastLoad[idx] = pct;
  uint16_t col = (pct < 0) ? C_GREY : loadCol(pct);
  String t = (pct < 0) ? String("--") : (String((int)pct) + "%");
  while (t.length() < 4) t += ' ';
  tft.setTextSize(2); tft.setTextColor(col, C_BLACK);
  tft.setCursor(252, rowY + 6);
  tft.print(t);
}

// микрографик из истории (уровень 0, последние ~7 мин): автомасштаб по максимуму окна
#define SPARK_N 28
static uint16_t sparkRev = 0;                    // по какому тику истории рисовали последний раз
static void sparkDraw(int16_t rowY, int16_t sx, int16_t sw, uint8_t sr, uint16_t col) {
  const int16_t sy = rowY + 4, sh = 18;
  tft.fillRect(sx, sy, sw, sh, C_BLACK);
  tft.drawRect(sx, sy, sw, sh, 0x2104);
  uint8_t buf[SPARK_N], mx = 0, cnt = 0;
  for (uint8_t i = 0; i < SPARK_N; i++) {
    uint8_t e = histGet(0, SPARK_N - 1 - i, sr);
    buf[i] = e;
    if (e != 255) { if (e > mx) mx = e; cnt++; }
  }
  if (cnt < 2 || mx == 0) return;
  int16_t px = 0, py = 0;
  bool have = false;
  for (uint8_t i = 0; i < SPARK_N; i++) {
    if (buf[i] == 255) { have = false; continue; }
    int16_t x = sx + 2 + (int16_t)((long)i * (sw - 5) / (SPARK_N - 1));
    int16_t y = sy + sh - 2 - (int16_t)((long)buf[i] * (sh - 5) / mx);
    if (have) tft.drawLine(px, py, x, y, col); else tft.drawPixel(x, y, col);
    px = x; py = y; have = true;
  }
}

// ---- 2-я страница: значения
static bool xFresh(uint8_t k) { return xAt[k] != 0 && millis() - xAt[k] <= X_STALE_MS; }

static String fmtGb(float gb) {
  if (gb >= 1000.0f) return String(gb / 1000.0f, 1) + " TB";
  if (gb >= 100.0f)  return String((int)(gb + 0.5f)) + " GB";
  return String(gb, 1) + " GB";
}

static String fmtMbit(float m) {
  if (m >= 100.0f) return String((int)(m + 0.5f)) + " Mb/s";
  if (m >= 10.0f)  return String(m, 1) + " Mb/s";
  return String(m, 2) + " Mb/s";
}

static void putX(uint8_t row, const String &txt, uint16_t col, uint8_t width) {
  String key = txt + "|" + String((int)col);
  if (!forceRedraw && key == lastX[row]) return;
  lastX[row] = key;
  tft.setTextSize(3); tft.setTextColor(col, C_BLACK);
  tft.setCursor(122, ROW_Y0 + row * ROW_DY + 2);
  tft.print(padTo(txt, width));
}

static void drawValues2() {
  // 0: аптайм
  if (xFresh(0)) putX(0, fmtDur((uint32_t)xV[0]), C_CYAN, 12); else putX(0, "--", C_GREY, 12);
  pollEsp();
  // 1: диск - свободно ГБ + полоска и проценты свободного места
  int16_t pct = xFresh(1) ? (int16_t)(xV[1] + 0.5f) : -1;
  if (pct > 100) pct = 100;
  uint16_t col = (pct < 0) ? (xFresh(2) ? C_CYAN : C_GREY) : (pct < 10 ? C_RED : (pct < 20 ? C_YELLOW : C_CYAN));
  putX(1, xFresh(2) ? fmtGb(xV[2]) : String("--"), col, 7);
  pollEsp();
  if (forceRedraw || pct != lastDiskPct) {
    lastDiskPct = pct;
    const int16_t ry = ROW_Y0 + ROW_DY, bx = 256, bw = 120, by = ry + 4, bh = 18;
    uint16_t bc = (pct < 0) ? C_GREY : col;
    int16_t fw = (pct < 0) ? 0 : (int16_t)((long)(bw - 4) * pct / 100);
    tft.drawRect(bx, by, bw, bh, C_GREY);
    if (fw > 0) tft.fillRect(bx + 2, by + 2, fw, bh - 4, bc);
    if (fw < bw - 4) tft.fillRect(bx + 2 + fw, by + 2, bw - 4 - fw, bh - 4, C_BLACK);
    String t = (pct < 0) ? String("--") : (String((int)pct) + "%");
    tft.setTextSize(2); tft.setTextColor(bc, C_BLACK);
    tft.setCursor(384, ry + 6);
    tft.print(padTo(t, 4));
  }
  // 2, 3: сеть вниз / вверх
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t idx = 3 + k;
    if (xFresh(idx)) putX(2 + k, fmtMbit(xV[idx]), C_CYAN, 12); else putX(2 + k, "--", C_GREY, 12);
    pollEsp();
  }
  // 4: недоступные targets Prometheus
  if (!xFresh(5)) putX(4, "--", C_GREY, 12);
  else {
    int n = (int)(xV[5] + 0.5f);
    if (n <= 0) putX(4, "all up", C_GREEN, 12); else putX(4, String(n) + " down", C_RED, 12);
  }
  pollEsp();
}

// ---- 3-я страница: мощность. Значение, % от бюджета БП (PSUM_LIM) для всех строк, справа микрографик.
//      Красный = превышен свой лимит (SUM - PSUM_LIM, GPU - свои PWR_LIM).
static const uint16_t POW_SPARK_COL[5] = {C_RED, C_YELLOW, C_GREY, C_CYAN, C_ORANGE};

static bool powOver(uint8_t k, float w) {
  if (k == 0) return PSUM_LIM > 0 && w > PSUM_LIM;
  if (k == 3) return GPU1_PWR_LIM > 0 && w > GPU1_PWR_LIM;
  if (k == 4) return GPU2_PWR_LIM > 0 && w > GPU2_PWR_LIM;
  return false;
}

static void drawValues3() {
  for (uint8_t k = 0; k < 5; k++) {
    bool fresh = pAt[k] != 0 && millis() - pAt[k] <= X_STALE_MS;
    String v = "--", mid = "--";
    if (fresh) {
      v = (pV[k] < 10.0f) ? (String(pV[k], 1) + " W") : (String((int)(pV[k] + 0.5f)) + " W");
      int mp = (PSUM_LIM > 0) ? (int)(pV[k] * 100.0f / PSUM_LIM + 0.5f) : -1;
      if (mp > 999) mp = 999;
      if (mp >= 0) mid = String(mp) + "%";
    }
    bool over = fresh && powOver(k, pV[k]);
    uint16_t col = fresh ? (over ? C_RED : C_CYAN) : C_GREY;
    uint16_t mcol = C_GREY;
    if (fresh) {
      int mp = (PSUM_LIM > 0) ? (int)(pV[k] * 100.0f / PSUM_LIM + 0.5f) : 0;
      mcol = over ? C_RED : (mp >= 90 ? C_YELLOW : C_CYAN);
    }
    String key = v + "|" + mid + "|" + String((int)col) + String((int)mcol);
    if (forceRedraw || key != lastP[k]) {
      lastP[k] = key;
      int16_t ry = ROW_Y0 + k * ROW_DY;
      tft.setTextSize(3); tft.setTextColor(col, C_BLACK);
      tft.setCursor(122, ry + 2); tft.print(padTo(v, 6));
      tft.setTextSize(2); tft.setTextColor(mcol, C_BLACK);
      tft.setCursor(246, ry + 6); tft.print(padTo(mid, 7));
    }
    pollEsp();
  }
}

// ---- статусная строка; при наличии упавших контейнеров - бегущая строка их имен
static char badBuf[128];                          // имена контейнеров со статусами U/R/X через "; "
static uint8_t badLen = 0;
static uint8_t badOff = 0;
static unsigned long badMs = 0;
static uint16_t badRev = 0xFFFF;                  // пересобираем список при каждом новом кадре Docker

static void badRebuild() {
  badLen = 0; badBuf[0] = 0; badOff = 0;
  if (!dkValid || dkBad <= 0 || millis() - dkMs > DK_FRESH_MS) return;
  for (uint8_t i = 0; i < dkN && badLen < sizeof(badBuf) - 26; i++) {
    char c = dkCode[i];
    if (c != 'U' && c != 'R' && c != 'X') continue;
    uint8_t n = strlen(dkName[i]);
    if (badLen) { badBuf[badLen++] = ';'; badBuf[badLen++] = ' '; }
    memcpy(badBuf + badLen, dkName[i], n);
    badLen += n; badBuf[badLen] = 0;
  }
}

static void statusDraw() {
  bool stale = (lastDataMs != 0) && (millis() - lastDataMs > DATA_STALE_MS);
  bool errRecent = (espErrMs != 0) && (millis() - espErrMs < ERR_HOLD_MS);
  if (badRev != dkRev) { badRev = dkRev; badRebuild(); }
  if (badLen && !alertActive && !errRecent && !stale && !snoozeActive()) {
    if (millis() - badMs <= 350) return;          // тик прокрутки 350 мс
    badMs = millis();
    uint16_t cyc = badLen + 6;                    // 6 пробелов в конце - пауза перед повтором
    char w[27];
    for (uint8_t i = 0; i < 26; i++) {
      uint16_t p = (badOff + i) % cyc;
      w[i] = (p < badLen) ? badBuf[p] : ' ';
    }
    w[26] = 0;
    badOff = (uint8_t)((badOff + 1) % cyc);
    tft.setTextSize(2); tft.setTextColor(C_ORANGE, C_BLACK);
    tft.setCursor(30, 290); tft.print(w);
    lastStatus = String("!") + w;                 // маркер: обычная ветка не считает это совпадением
    return;
  }
  String st = alertActive ? ("ALERT: " + alertText) : (errRecent ? espErrText : (stale ? String("NO DATA FROM ESP") : (snoozeActive() ? (String("OK, alerts off ") + String((snoozeUntil - millis()) / 60000UL + 1UL) + "m") : String("OK"))));
  if (forceRedraw || st != lastStatus) {
    lastStatus = st;
    String t = st;
    if (t.length() > 26) t = t.substring(0, 26);            // правее - плашка Docker
    while (t.length() < 26) t += ' ';
    tft.setTextSize(2);
    tft.setCursor(30, 290);
    tft.setTextColor(alertActive ? C_RED : ((errRecent || stale) ? C_YELLOW : C_GREY), C_BLACK);
    tft.print(t);
  }
}

void drawValues() {
  drawClock();
  if (curPage == 0) {
    float tv[4] = {cpuTemp, gpuTemp, ssdTemp, p102Temp};
    for (uint8_t i = 0; i < 5; i++) {
      int16_t ry = ROW_Y0 + i * ROW_DY;
      if (ROW_TEMP[i] >= 0) { printVal(ROW_TEMP[i], ry, tv[ROW_TEMP[i]]); pollEsp(); }
      printLoad(ROW_LOAD[i], ry, loadV[ROW_LOAD[i]]); pollEsp();
    }
    if (forceRedraw || sparkRev != hRev) {                // микрографики нагрузки (~7 мин из истории)
      sparkRev = hRev;
      for (uint8_t i = 0; i < 5; i++) { sparkDraw(ROW_Y0 + i * ROW_DY, 306, 164, 4 + ROW_LOAD[i], C_CYAN); pollEsp(); }
    }
  } else if (curPage == 1) {
    drawValues2();
  } else {
    drawValues3();
    if (forceRedraw || sparkRev != hRev) {                // микрографики мощности
      sparkRev = hRev;
      for (uint8_t k = 0; k < 5; k++) { sparkDraw(ROW_Y0 + k * ROW_DY, 336, 134, POW_HIST_FIRST + k, POW_SPARK_COL[k]); pollEsp(); }
    }
  }

  statusDraw();
  dkWidgetDraw();
  forceRedraw = false;
}

// ======================= ЭКРАНЫ МЕНЮ =======================

static void sendEsp(const String &cmd) {
  EspSerial.print(cmd); EspSerial.print("\n");
  Dbg.print("-> ESP: "); Dbg.println(cmd);
}

// экранная клавиатура для ввода "адрес[:порт]"; true = OK
static bool editServer(String &val) {
  const int16_t X0 = 20, Y0 = 65, DX = 115, DY = 60, BW = 105, BH = 52;
  const char* lbl[4][4] = {
    {"1", "2", "3", "DEL"},
    {"4", "5", "6", "."},
    {"7", "8", "9", ":"},
    {"0", "Cancel", "OK", "OK"}
  };
  tft.fillScreen(C_BLACK);
  for (uint8_t r = 0; r < 4; r++) {
    for (uint8_t c = 0; c < 4; c++) {
      if (r == 3 && c == 3) continue;                 // OK занимает две клетки
      int16_t w = (r == 3 && c == 2) ? 220 : BW;
      uint16_t col = C_WHITE;
      if (r == 3 && c == 1) col = C_YELLOW;
      if (r == 3 && c == 2) col = C_GREEN;
      drawButton(X0 + c * DX, Y0 + r * DY, w, BH, lbl[r][c], col);
    }
  }
  String last = "#";
  int16_t x, y;
  while (true) {
    pollEsp();
    if (val != last) {
      last = val;
      tft.drawRect(20, 8, 450, 44, C_WHITE);
      tft.setTextSize(3); tft.setTextColor(C_GREEN, C_BLACK);
      tft.setCursor(28, 17);
      String t = val + "_";
      while (t.length() < 24) t += ' ';
      tft.print(t);
    }
    if (!getTap(x, y)) continue;
    int16_t c = (x - X0) / DX, r = (y - Y0) / DY;
    if (x < X0 || y < Y0 || r > 3 || c > 3) continue;
    if ((x - X0) % DX >= BW || (y - Y0) % DY >= BH) continue;
    const char* k = lbl[r][c];
    if (!strcmp(k, "OK"))          { if (val.length() > 0) return true; }
    else if (!strcmp(k, "Cancel")) return false;
    else if (!strcmp(k, "DEL"))    { if (val.length()) val.remove(val.length() - 1); }
    else if (val.length() < 23)    val += k;
  }
}

// ---------- полная экранная клавиатура (пароль Wi-Fi) ----------
static const char* const KB_L[3][4] = {
  {"1234567890", "qwertyuiop", "asdfghjkl.", "zxcvbnm,-_"},
  {"1234567890", "QWERTYUIOP", "ASDFGHJKL.", "ZXCVBNM,-_"},
  {"!@#$%^&*()", "=+[]{}<>/?", "\\|;:'\"~`", ""}
};
static const char* const KB_B[6] = {"Shift", "123", "Space", "DEL", "Cancel", "OK"};
static const uint8_t KB_U[6] = {3, 3, 4, 3, 4, 3};      // ширина клавиш нижнего ряда (в 1/2 клетки)

static void kbDraw(uint8_t layer) {
  for (uint8_t r = 0; r < 4; r++) {
    int n = strlen(KB_L[layer][r]);
    for (int i = 0; i < 10; i++) {
      int16_t x = 10 + i * 46, y = 62 + r * 44;
      if (i < n) { char t[2] = {KB_L[layer][r][i], 0}; drawButton(x, y, 42, 40, t, C_WHITE); }
      else tft.fillRect(x, y, 42, 40, C_BLACK);
    }
  }
  int16_t cum = 0;
  for (uint8_t k = 0; k < 6; k++) {
    const char* lb = KB_B[k];
    if (k == 1) lb = (layer == 2) ? "abc" : "123";
    uint16_t col = C_WHITE;
    if (k == 0 && layer == 1) col = C_GREEN;
    if (k == 4) col = C_YELLOW;
    if (k == 5) col = C_GREEN;
    drawButton(10 + cum * 23, 62 + 4 * 44, KB_U[k] * 23 - 4, 40, lb, col);
    cum += KB_U[k];
  }
}

static bool editText(const String &title, String &val, uint8_t maxLen) {
  uint8_t layer = 0;
  tft.fillScreen(C_BLACK);
  tft.setTextSize(1); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(10, 4); tft.print(title);
  kbDraw(layer);
  String last = "\x01";
  int16_t x, y;
  while (true) {
    pollEsp();
    if (val != last) {
      last = val;
      tft.drawRect(10, 16, 460, 40, C_WHITE);
      tft.setTextSize(2); tft.setTextColor(C_GREEN, C_BLACK);
      tft.setCursor(16, 27);
      String t = val + "_";
      if (t.length() > 36) t = t.substring(t.length() - 36);
      while (t.length() < 36) t += ' ';
      tft.print(t);
    }
    if (!getTap(x, y)) continue;
    if (y < 62 || x < 10) continue;
    int16_t r = (y - 62) / 44;
    if (r > 4 || (y - 62) % 44 >= 40) continue;
    if (r < 4) {
      int16_t i = (x - 10) / 46;
      if (i > 9 || (x - 10) % 46 >= 42) continue;
      if (i >= (int16_t)strlen(KB_L[layer][r])) continue;
      if (val.length() < maxLen) {
        val += KB_L[layer][r][i];
        if (layer == 1) { layer = 0; kbDraw(layer); }       // Shift действует на один символ
      }
    } else {
      int16_t cum = 0;
      for (uint8_t k = 0; k < 6; k++) {
        int16_t x0 = 10 + cum * 23, w = KB_U[k] * 23 - 4;
        cum += KB_U[k];
        if (x < x0 || x >= x0 + w) continue;
        if (k == 0)      { layer = (layer == 1) ? 0 : 1; kbDraw(layer); }
        else if (k == 1) { layer = (layer == 2) ? 0 : 2; kbDraw(layer); }
        else if (k == 2) { if (val.length() < maxLen) val += ' '; }
        else if (k == 3) { if (val.length()) val.remove(val.length() - 1); }
        else if (k == 4) return false;
        else             return true;
        break;
      }
    }
  }
}

// ---------- выбор Wi-Fi сети ----------
static String padTo(String s, uint8_t n) {
  if (s.length() > n) s = s.substring(0, n);
  while (s.length() < n) s += ' ';
  return s;
}

static bool connectScreen(const String &ssid, const String &pass) {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(2); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 40); tft.print("Connecting to:");
  tft.setTextColor(C_GREEN, C_BLACK);
  tft.setCursor(20, 80); tft.print(padTo(ssid, 36));
  wifiRes = 0;
  EspSerial.print("SETWIFI:"); EspSerial.print(ssid); EspSerial.print('\t'); EspSerial.print(pass); EspSerial.print("\n");
  unsigned long t0 = millis(), lastShow = 0;
  while (wifiRes == 0 && millis() - t0 < 30000) {
    pollEsp();
    if (millis() - lastShow > 500) {
      lastShow = millis();
      tft.setTextColor(C_YELLOW, C_BLACK);
      tft.setCursor(20, 140); tft.print("Please wait "); tft.print((millis() - t0) / 1000); tft.print(" s   ");
    }
  }
  tft.setTextSize(3);
  tft.fillRect(0, 130, 480, 60, C_BLACK);
  tft.setCursor(20, 140);
  if (wifiRes == 1) {
    tft.setTextColor(C_GREEN, C_BLACK); tft.print("Connected!");
    sendEsp("GETCFG");
  } else {
    tft.setTextColor(C_RED, C_BLACK);
    tft.print(wifiRes == -1 ? "Failed" : "No answer");
  }
  delay(1800);
  return wifiRes == 1;
}

static void wifiDrawStatic() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("Wi-Fi");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  drawButton(10, 274, 130, 40, "Rescan", C_WHITE);
  drawButton(150, 274, 80, 40, "Up", C_WHITE);
  drawButton(240, 274, 80, 40, "Down", C_WHITE);
  drawButton(330, 274, 140, 40, "Back", C_YELLOW);
}

static void wifiDrawRow(uint8_t slot, int idx) {
  int16_t y = 62 + slot * 42;
  if (idx < 0 || idx >= netCount) { tft.fillRect(10, y, 460, 38, C_BLACK); return; }
  uint16_t col = (espWifi == 1 && netSsid[idx] == espSsid) ? C_GREEN : C_WHITE;
  tft.fillRect(10, y, 460, 38, C_BTN);
  tft.drawRect(10, y, 460, 38, col);
  tft.setTextSize(2); tft.setTextColor(col, C_BTN);
  tft.setCursor(18, y + 11);
  tft.print(padTo(netSsid[idx], 26));
  tft.print(padTo(String(netRssi[idx]), 5));
  tft.print(netEnc[idx] ? "pwd" : "open");
}

static void wifiStartScan() {
  netCount = 0; scanState = 1; scanReqMs = millis();
  sendEsp("SCAN");
}

static void wifiScreen() {
  wifiDrawStatic();
  wifiStartScan();
  uint8_t top = 0;
  long sig = -1;
  String lastSt = "#";
  int16_t x, y;
  while (true) {
    pollEsp();
    if (scanState == 1 && millis() - scanReqMs > 12000) scanState = 3;

    long sg = netCount * 1000L + scanState * 100L + top;
    if (sg != sig) {
      sig = sg;
      for (uint8_t i = 0; i < 5; i++) wifiDrawRow(i, top + i);
    }
    String st = (scanState == 1) ? "Scanning..." : (scanState == 3) ? "ESP not answering" : (netCount == 0 ? "No networks found" : "Tap a network");
    if (st != lastSt) {
      lastSt = st;
      tft.setTextSize(2); tft.setTextColor(scanState == 3 ? C_YELLOW : C_GREY, C_BLACK);
      tft.setCursor(140, 22); tft.print(padTo(st, 22));
    }

    if (!getTap(x, y)) continue;
    if (inRect(x, y, 10, 274, 130, 40)) {
      if (scanState != 1) { wifiStartScan(); top = 0; }
    } else if (inRect(x, y, 150, 274, 80, 40)) {
      top = (top >= 3) ? top - 3 : 0;
    } else if (inRect(x, y, 240, 274, 80, 40)) {
      uint8_t mx = (netCount > 5) ? netCount - 5 : 0;
      top = (top + 3 > mx) ? mx : top + 3;
    } else if (inRect(x, y, 330, 274, 140, 40)) {
      return;
    } else if (y >= 62 && y < 62 + 5 * 42 && (y - 62) % 42 < 38 && x >= 10 && x < 470) {
      int idx = top + (y - 62) / 42;
      if (idx < netCount) {
        String ssid = netSsid[idx], pass = "";
        bool ok = true;
        if (netEnc[idx]) ok = editText("Password for: " + ssid, pass, 63);
        if (ok) connectScreen(ssid, pass);
        wifiDrawStatic();
        sig = -1; lastSt = "#";
      }
    }
  }
}

#if BAT_ENABLED
static String fmtV(long mv) {      // 4041 -> "4.041 V"
  String f = String(mv % 1000);
  while (f.length() < 3) f = "0" + f;
  return String(mv / 1000) + "." + f + " V";
}

static void batCalScreen() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("Battery cal");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  tft.setTextSize(2); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(20, 66);  tft.print("1. Measure the battery with a multimeter");
  tft.setCursor(20, 90);  tft.print("2. Press Enter real V and type it");
  drawButton(20, 190, 440, 48, "Enter real V", C_GREEN);
  drawButton(20, 248, 215, 52, "Reset", C_WHITE);
  drawButton(245, 248, 215, 52, "Back", C_YELLOW);
  String last = "#";
  unsigned long lastMs = 0;
  int16_t x, y;
  while (true) {
    pollEsp();
    if (millis() - lastMs > 600) {
      lastMs = millis();
      long raw = batMeasureRawMv(32);
      String t = "Reads: " + fmtV(raw * batPermille / 1000) + "  x" + String(batPermille / 1000) + "." + String(batPermille % 1000 / 100) + String(batPermille % 100 / 10) + String(batPermille % 10);
      if (t != last) {
        last = t;
        tft.setTextColor(C_GREEN, C_BLACK);
        tft.setCursor(20, 130);
        tft.print(padTo(t, 36));
      }
    }
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 20, 190, 440, 48)) {
      String s = "";
      if (editServer(s) && s.length()) {
        float v = s.toFloat();
        long real = (v > 100.0f) ? (long)(v + 0.5f) : (long)(v * 1000.0f + 0.5f);   // "4.041" или "4041"
        long raw = batMeasureRawMv(64);
        if (real >= 2500 && real <= 4600 && raw > 0) {
          long pm = real * 1000 / raw;
          if (pm < 800) pm = 800;
          if (pm > 1200) pm = 1200;
          batPermille = (uint16_t)pm;
          batSaveCal();
          batRaw = 0; batShown = -1;                              // перемерить заново
        }
      }
      tft.fillScreen(C_BLACK);
      tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
      tft.setCursor(20, 15); tft.print("Battery cal");
      tft.drawFastHLine(10, 55, 460, C_GREY);
      tft.setTextSize(2); tft.setTextColor(C_GREY, C_BLACK);
      tft.setCursor(20, 66);  tft.print("1. Measure the battery with a multimeter");
      tft.setCursor(20, 90);  tft.print("2. Press Enter real V and type it");
      drawButton(20, 190, 440, 48, "Enter real V", C_GREEN);
      drawButton(20, 248, 215, 52, "Reset", C_WHITE);
      drawButton(245, 248, 215, 52, "Back", C_YELLOW);
      last = "#";
    } else if (inRect(x, y, 20, 248, 215, 52)) {
      batPermille = 1000; batSaveCal(); batRaw = 0; batShown = -1;
    } else if (inRect(x, y, 245, 248, 215, 52)) {
      return;
    }
  }
}

#endif   // BAT_ENABLED (калибровка батареи)

// ======================= ЭКРАН DOCKER =======================
#define DK_ROWS 8
static String dkSig[DK_ROWS];
static String dkHdrSig;

static const char* dkLabel(char c) {
  switch (c) {
    case 'O': return "UP";
    case 'S': return "START";
    case 'P': return "PAUSE";
    case 'U': return "UNHLTH";
    case 'R': return "RSTRT";
    default:  return "DOWN";
  }
}

static uint16_t dkColor(char c) {
  switch (c) {
    case 'O': return C_GREEN;
    case 'S': case 'P': return C_YELLOW;
    case 'U': case 'R': return C_ORANGE;
    default:  return C_RED;
  }
}

static String fmtDur(uint32_t t) {                 // 259200 -> "3d 0h", 4500 -> "1h 15m", 720 -> "12m"
  unsigned long s = t;
  if (s >= 86400UL) return String(s / 86400UL) + "d " + String((s % 86400UL) / 3600UL) + "h";
  if (s >= 3600UL)  return String(s / 3600UL) + "h " + String((s % 3600UL) / 60UL) + "m";
  if (s >= 60UL)    return String(s / 60UL) + "m";
  return String(s) + "s";
}

static void dkDrawAll() {                          // каркас экрана; значения дорисует dockerScreen()
  tft.fillScreen(C_BLACK);
  tft.drawFastHLine(10, 55, 460, C_GREY);
  drawButton(10, 278, 70, 36, "<", C_WHITE);
  drawButton(90, 278, 70, 36, ">", C_WHITE);
  drawButton(170, 278, 140, 36, "Server", C_WHITE);
  drawButton(320, 278, 150, 36, "Back", C_YELLOW);
  for (uint8_t i = 0; i < DK_ROWS; i++) dkSig[i] = "#";
  dkHdrSig = "#";
}

static void dkDrawHeader(int page, int pages, bool fresh, bool errRecent) {
  tft.fillRect(0, 0, 480, 54, C_BLACK);
  icoDocker(14, 14, C_CYAN);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(50, 15); tft.print("Docker");
  tft.setTextSize(2); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(168, 22); tft.print(String(page + 1) + "/" + String(pages));
  tft.setCursor(222, 22);
  if (errRecent) {
    String t = dkErrText;
    if (t.length() > 21) t = t.substring(0, 21);
    tft.setTextColor(C_YELLOW, C_BLACK); tft.print(t);
  } else if (!dkValid) {
    tft.setTextColor(espDk.length() ? C_GREY : C_YELLOW, C_BLACK);
    tft.print(espDk.length() ? "waiting for data..." : "server not set");
  } else if (!fresh) {
    tft.setTextColor(C_YELLOW, C_BLACK); tft.print("no updates");
  } else {
    tft.setTextColor(C_GREEN, C_BLACK);  tft.print(String(dkOk) + " ok");
    if (dkWarn > 0) { tft.setTextColor(C_YELLOW, C_BLACK); tft.print(" " + String(dkWarn) + " warn"); }
    if (dkBad > 0)  { tft.setTextColor(C_RED, C_BLACK);    tft.print(" " + String(dkBad) + " bad"); }
  }
}

// содержимое строки списка (слот 0..7 страницы); пустой слот = ""
static String dkRowSig(int idx, int slot) {
  if (idx < 0 || idx >= dkN) {
    if (slot == 0 && dkN == 0) return dkValid ? "msg:no containers" : (espDk.length() ? "msg:waiting for docker data" : "msg:tap Server to set address");
    return "";
  }
  return String(dkCode[idx]) + "|" + dkName[idx] + "|" + fmtDur(dkSecs[idx]) + "|" + String(dkRst[idx]) + "|" + String(dkExit[idx]);
}

static void dkDrawRow(int slot, int idx) {
  int16_t y = 62 + slot * 26;
  if (idx < 0 || idx >= dkN) {
    tft.fillRect(1, y, 478, 26, C_BLACK);
    if (slot == 0 && dkN == 0) {
      tft.setTextSize(2); tft.setTextColor(dkValid ? C_GREY : C_YELLOW, C_BLACK);
      tft.setCursor(30, y + 6);
      tft.print(dkValid ? "No containers" : (espDk.length() ? "Waiting for docker data..." : "Tap Server to set address"));
    }
    return;
  }
  char c = dkCode[idx];
  uint16_t col = dkColor(c);
  tft.fillCircle(17, y + 9, 5, col);
  tft.setTextSize(2);
  tft.setTextColor(c == 'O' ? C_WHITE : col, C_BLACK);
  tft.setCursor(30, y + 1);  tft.print(padTo(String(dkName[idx]), 18));
  pollEsp();
  tft.setTextColor(col, C_BLACK);
  tft.setCursor(252, y + 1); tft.print(padTo(dkLabel(c), 6));
  tft.setTextColor(c == 'O' ? C_GREY : C_WHITE, C_BLACK);
  tft.setCursor(330, y + 1); tft.print(padTo(fmtDur(dkSecs[idx]), 7));
  String extra = (c == 'X') ? ("e" + String((int)dkExit[idx])) : (dkRst[idx] > 0 ? ("x" + String((int)dkRst[idx])) : String(""));
  tft.setTextColor(c == 'X' ? C_RED : C_ORANGE, C_BLACK);
  tft.setCursor(420, y + 1); tft.print(padTo(extra, 4));
  pollEsp();
}

static void dockerScreen() {
  int page = 0;
  dkDrawAll();
  sendEsp("GETCFG");
  sendEsp("DKNOW");                                // свежие данные сразу
  int16_t x, y;
  while (true) {
    pollEsp();
    if (millis() - lastActMs > 60000UL) return;          // забыли экран открытым - вернуться
    int pages = dkN ? (dkN + DK_ROWS - 1) / DK_ROWS : 1;
    if (page >= pages) page = pages - 1;
    bool fresh = dkValid && (millis() - dkMs < DK_FRESH_MS);
    bool errRecent = dkErrMs != 0 && (millis() - dkErrMs < 25000);
    String hs = String(page) + "/" + String(pages) + "|" + (fresh ? "1" : "0") + (errRecent ? "1" : "0") + (dkValid ? "1" : "0") +
                String(dkOk) + "," + String(dkWarn) + "," + String(dkBad) + "," + String(espDk.length()) + (errRecent ? dkErrText : String(""));
    if (hs != dkHdrSig) { dkHdrSig = hs; dkDrawHeader(page, pages, fresh, errRecent); }
    for (int slot = 0; slot < DK_ROWS; slot++) {
      int idx = page * DK_ROWS + slot;
      String sg = dkRowSig(idx, slot);
      if (sg != dkSig[slot]) { dkSig[slot] = sg; dkDrawRow(slot, idx); }
      pollEsp();
    }
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 10, 278, 70, 36)) {
      if (page > 0) { page--; for (uint8_t i = 0; i < DK_ROWS; i++) dkSig[i] = "#"; dkHdrSig = "#"; }
    } else if (inRect(x, y, 90, 278, 70, 36)) {
      if (page + 1 < pages) { page++; for (uint8_t i = 0; i < DK_ROWS; i++) dkSig[i] = "#"; dkHdrSig = "#"; }
    } else if (inRect(x, y, 170, 278, 140, 36)) {
      String v = espDk;
      if (editServer(v)) {
        sendEsp("SETDK:" + v);
        delay(400);
        sendEsp("GETCFG");
        sendEsp("DKNOW");
      }
      dkDrawAll();
    } else if (inRect(x, y, 320, 278, 150, 36)) {
      return;
    }
  }
}

static String espLast[4];

static void espDrawStatic() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("ESP-01");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  tft.setTextSize(2);
  tft.setCursor(20, 70);  tft.print("Server:");
  tft.setCursor(20, 100); tft.print("WiFi:");
  tft.setCursor(20, 130); tft.print("ESP IP:");
  tft.setCursor(20, 160); tft.print("Signal:");
  drawButton(20, 200, 140, 50, "Wi-Fi", C_WHITE);
  drawButton(170, 200, 140, 50, "Server", C_WHITE);
  drawButton(320, 200, 140, 50, "Reset", C_WHITE);
  drawButton(20, 262, 440, 48, "Back", C_YELLOW);
  for (uint8_t i = 0; i < 4; i++) espLast[i] = "";
}

static void espDrawValues() {
  bool alive = espCfgMs != 0 && millis() - espCfgMs < 6000;
  String v[4]; uint16_t col[4];
  if (!alive) {
    v[0] = "ESP not answering"; col[0] = C_YELLOW;
    v[1] = v[2] = v[3] = "-";  col[1] = col[2] = col[3] = C_GREY;
  } else {
    v[0] = espSrv;                                  col[0] = C_GREEN;
    v[1] = (espWifi == 1) ? espSsid : String("not connected"); col[1] = (espWifi == 1) ? C_GREEN : C_RED;
    v[2] = espIp;                                   col[2] = C_GREEN;
    v[3] = String(espRssi) + " dBm";                col[3] = C_GREEN;
  }
  tft.setTextSize(2);
  for (uint8_t i = 0; i < 4; i++) {
    if (v[i] == espLast[i]) continue;
    espLast[i] = v[i];
    tft.setTextColor(col[i], C_BLACK);
    tft.setCursor(130, 70 + i * 30);
    tft.print(padTo(v[i], 28));                  // ровно 28 символов: длинная строка не залезет на соседние поля
    pollEsp();                                   // отрисовка медленная - не терять байты UART
  }
}

static void espScreen() {
  espDrawStatic();
  sendEsp("GETCFG");
  unsigned long lastReq = millis();
  int16_t x, y;
  while (true) {
    pollEsp();
    if (millis() - lastReq > 2000) { sendEsp("GETCFG"); lastReq = millis(); }
    espDrawValues();
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 20, 200, 140, 50)) {
      wifiScreen();
      espDrawStatic();
      sendEsp("GETCFG"); lastReq = millis();
    } else if (inRect(x, y, 170, 200, 140, 50)) {
      String s = espSrv;
      if (editServer(s)) {
        sendEsp("SETSRV:" + s);
        delay(400);
        sendEsp("GETCFG");
        lastReq = millis();
      }
      espDrawStatic();
    } else if (inRect(x, y, 320, 200, 140, 50)) {
      digitalWrite(ESP_EN, LOW); delay(300); digitalWrite(ESP_EN, HIGH);
      espCfgMs = 0;
    } else if (inRect(x, y, 20, 262, 440, 48)) {
      return;
    }
  }
}

// ======================= ЭКРАН ГРАФИКОВ =======================
// Диапазон = сколько последних минут / часов показать. Запись идёт всегда (см. ИСТОРИЯ), поэтому "длительность записи" - это окно просмотра.
static const uint8_t  GR_LV[7]   = {0, 0, 1, 1, 1, 2, 2};
static const uint16_t GR_N[7]    = {120, 240, 60, 120, 240, 80, 240};
static const char* const GR_LBL[7] = {"30m", "1h", "3h", "6h", "12h", "24h", "72h"};
static const uint16_t GR_STEP[3] = {15, 180, 1080};            // секунд на отсчёт в каждом разрешении
static uint8_t grSel = 0, grRange = 1;                         // grSel - строка (CPU, RAM, GPU1, GPU2, SSD)
static bool grPow = false;                                     // true = все 5 рядов мощности
static String gMsg = "", grLastVals;
static unsigned long gMsgMs = 0;

#define GP_X 38
#define GP_Y 50
#define GP_W 432
#define GP_H 172
#define GR_BT_Y 268
#define GR_RANGE_X 118                                        // кнопка интервала: рядом с Back, внизу

// шкала мощности на графике: авто-подстройка по максимуму окна, потолок - PSUM_LIM (БП)
static uint16_t grTopW = (uint16_t)PSUM_LIM;      // верх шкалы, Вт (пересчитывается в grPlot)

static int16_t grPx(int i, int n) { return GP_X + (int16_t)((long)i * (GP_W - 1) / (n - 1)); }
static int16_t grPy(uint8_t e) {
  if (e > 200) e = 200;
  return GP_Y + GP_H - 1 - (int16_t)((long)e * (GP_H - 1) / 200);
}
static int16_t grPyW(uint8_t e) {                             // мощность из истории (байт = Вт * 0.5)
  return GP_Y + GP_H - 1 - (int16_t)((long)e * 2 * (GP_H - 1) / (long)grTopW);
}

static void grPlot() {
  tft.fillRect(GP_X, GP_Y, GP_W, GP_H, C_BLACK);
  tft.drawRect(GP_X - 1, GP_Y - 1, GP_W + 2, GP_H + 2, C_GREY);
  uint8_t lv = GR_LV[grRange];
  uint16_t n = GR_N[grRange];
  if (grPow) {
    // вертикальная шкала: максимум видимого окна, округлённый вверх до 25 Вт, потолок - PSUM_LIM
    uint8_t mx = 0;
    for (uint16_t i = 0; i < n; i++)
      for (uint8_t sr = 0; sr < 5; sr++) {
        uint8_t e = histGet(lv, n - 1 - i, (uint8_t)(POW_HIST_FIRST + sr));
        if (e != 255 && e > mx) mx = e;
      }
    uint16_t topW = mx * 2;                                   // максимум окна, Вт
    if (topW < 100) topW = 100;
    topW = (uint16_t)((topW + 24) / 25 * 25);
    if (topW > (uint16_t)PSUM_LIM) topW = (uint16_t)PSUM_LIM;
    grTopW = topW;
    for (int g = 1; g <= 3; g++)                              // сетка на четвертях шкалы
      tft.drawFastHLine(GP_X, GP_Y + GP_H - 1 - (int16_t)((long)g * (topW / 4) * (GP_H - 1) / topW), GP_W, 0x2104);
    static const uint16_t PCL[5] = {C_RED, C_YELLOW, C_GREY, C_CYAN, C_ORANGE};
    for (uint8_t sr = 0; sr < 5; sr++) {
      int16_t px = 0, py = 0;
      bool have = false;
      for (uint16_t i = 0; i < n; i++) {
        uint8_t e = histGet(lv, n - 1 - i, (uint8_t)(POW_HIST_FIRST + sr));
        if (e == 255) { have = false; continue; }               // разрыв в данных
        int16_t x = grPx(i, n), y = grPyW(e);
        if (have) tft.drawLine(px, py, x, y, PCL[sr]); else tft.drawPixel(x, y, PCL[sr]);
        px = x; py = y; have = true;
      }
      pollEsp();
    }
    return;
  }
  for (int g = 1; g <= 3; g++) tft.drawFastHLine(GP_X, GP_Y + GP_H - 1 - g * (GP_H - 1) / 4, GP_W, 0x2104);   // сетка 25 / 50 / 75
  int8_t ts = ROW_TEMP[grSel];
  int8_t sr[2] = {(int8_t)(4 + ROW_LOAD[grSel]), ts};           // нагрузка, температура
  uint16_t cl[2] = {C_CYAN, C_ORANGE};
  for (uint8_t pass = 0; pass < 2; pass++) {
    if (sr[pass] < 0) continue;
    int16_t px = 0, py = 0;
    bool have = false;
    for (uint16_t i = 0; i < n; i++) {
      uint8_t e = histGet(lv, n - 1 - i, (uint8_t)sr[pass]);
      if (e == 255) { have = false; continue; }                 // разрыв в данных
      int16_t x = grPx(i, n), y = grPy(e);
      if (have) tft.drawLine(px, py, x, y, cl[pass]); else tft.drawPixel(x, y, cl[pass]);
      px = x; py = y; have = true;
    }
    pollEsp();
  }
}

static String grTimeLbl(long ago, bool day) {
  if (!clkValid()) return (ago == 0) ? String("now") : (String("-") + fmtDur((uint32_t)ago));
  int yr, mo, d, hh, mi, ss, wd;
  epochToDate(nowLocal() - ago, yr, mo, d, hh, mi, ss, wd);
  String t = two(hh) + ":" + two(mi);
  if (day) t = String(WD_NAME[wd]) + " " + t;
  return t;
}

static void grLabels() {
  long range = (long)GR_N[grRange] * GR_STEP[GR_LV[grRange]];
  bool day = range >= 86400L;
  String l = grTimeLbl(range, day), m = grTimeLbl(range / 2, day), r = grTimeLbl(0, false);
  int16_t ly = GP_Y + GP_H + 4;
  tft.fillRect(0, ly, 480, 10, C_BLACK);
  tft.setTextSize(1); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(GP_X, ly); tft.print(l);
  tft.setCursor(GP_X + GP_W / 2 - (int16_t)m.length() * 3, ly); tft.print(m);
  tft.setCursor(GP_X + GP_W - (int16_t)r.length() * 6, ly); tft.print(r);
}

// 6 кнопок в ряд: 5 строк + Pow; текст короткий, ширины хватает
static void grButtons() {
  for (uint8_t i = 0; i < 5; i++) drawButton(6 + i * 79, 6, 74, 38, ROW_LBL[i], !grPow && i == grSel ? C_GREEN : C_WHITE);
  drawButton(6 + 5 * 79, 6, 74, 38, "Pow", grPow ? C_GREEN : C_WHITE);
}

static void grDrawYAxis() {                      // подписи шкалы: %/°C или Вт в режиме Pow
  tft.setTextSize(1); tft.setTextColor(C_GREY, C_BLACK);
  tft.fillRect(0, GP_Y - 10, 36, GP_H + 12, C_BLACK);
  if (grPow) {
    tft.setCursor(4, GP_Y - 2);             tft.print(grTopW);
    tft.setCursor(4, GP_Y + GP_H / 2 - 4);  tft.print(grTopW / 2);
    tft.setCursor(20, GP_Y + GP_H - 8);     tft.print("0");
    tft.setCursor(2, GP_Y + GP_H + 6);      tft.print("W");
  } else {
    tft.setCursor(8, GP_Y - 2);            tft.print("100");
    tft.setCursor(14, GP_Y + GP_H / 2 - 4); tft.print("50");
    tft.setCursor(20, GP_Y + GP_H - 8);    tft.print("0");
  }
}

static void grDrawAll() {
  tft.fillScreen(C_BLACK);
  grDrawYAxis();
  grButtons();
  drawButton(10, GR_BT_Y, 100, 44, "Back", C_YELLOW);
  drawButton(GR_RANGE_X, GR_BT_Y, 100, 44, GR_LBL[grRange], C_WHITE);   // интервал: тап = следующий
#if SD_EXPORT
  drawButton(120, GR_BT_Y, 100, 44, "SD", C_GREEN);
#endif
  grLastVals = "#";
}

static bool pFresh(uint8_t k) { return pAt[k] != 0 && millis() - pAt[k] <= X_STALE_MS; }
static String pFmt(uint8_t k) { return pFresh(k) ? (String((int)(pV[k] + 0.5f)) + "W") : String("--"); }

static void grValues() {                       // справа внизу: текущие значения или сообщение
  String msg = (gMsgMs != 0 && millis() - gMsgMs < 6000UL) ? gMsg : String("");
  String sig = msg + "|";
  if (grPow) {
    // мощность: 5 значений в цветах линий, две строки мелким шрифтом
    static const uint16_t PCL[5] = {C_RED, C_YELLOW, C_GREY, C_CYAN, C_ORANGE};
    for (uint8_t k = 0; k < 5; k++) sig += pFmt(k) + ";";
    if (sig == grLastVals) return;
    grLastVals = sig;
    tft.fillRect(228, GR_BT_Y, 250, 44, C_BLACK);
    if (msg.length()) {
      tft.setTextSize(2); tft.setTextColor(C_YELLOW, C_BLACK);
      tft.setCursor(232, GR_BT_Y + 14); tft.print(msg.substring(0, 19)); return;
    }
    tft.setTextSize(1);
    tft.setCursor(232, GR_BT_Y + 8);
    tft.setTextColor(PCL[0], C_BLACK); tft.print("SUM " + pFmt(0) + "  ");
    tft.setTextColor(PCL[1], C_BLACK); tft.print("CPU " + pFmt(1) + "  ");
    tft.setTextColor(PCL[2], C_BLACK); tft.print("DRAM " + pFmt(2));
    tft.setCursor(232, GR_BT_Y + 24);
    tft.setTextColor(PCL[3], C_BLACK); tft.print(String(GPU1_NAME) + " " + pFmt(3) + "  ");
    tft.setTextColor(PCL[4], C_BLACK); tft.print(String(GPU2_NAME) + " " + pFmt(4));
    return;
  }
  String a = "", b = "";
  int8_t ts = ROW_TEMP[grSel], ld = ROW_LOAD[grSel];
  float tv[4] = {cpuTemp, gpuTemp, ssdTemp, p102Temp};
  if (ts >= 0 && tempAt[ts] != 0 && millis() - tempAt[ts] <= VAL_STALE_MS) a = String(tv[ts], 1) + "C";
  if (loadAt[ld] != 0 && millis() - loadAt[ld] <= VAL_STALE_MS) b = String((int)(loadV[ld] + 0.5f)) + "%";
  sig += a + "|" + b;
  if (sig == grLastVals) return;
  grLastVals = sig;
  tft.fillRect(228, GR_BT_Y, 250, 44, C_BLACK);
  tft.setTextSize(2);
  tft.setCursor(232, GR_BT_Y + 14);
  if (msg.length()) { tft.setTextColor(C_YELLOW, C_BLACK); tft.print(msg.substring(0, 19)); return; }
  tft.setTextColor(C_ORANGE, C_BLACK); tft.print(a);
  tft.setTextColor(C_CYAN, C_BLACK);   tft.print(String("  ") + b);
}

#if SD_EXPORT
// CSV с выбранным диапазоном (все 9 рядов). Время - локальное, если часы синхронизированы, иначе секунды назад.
static String sdExport() {
  if (!SD.begin()) return "No SD card";
  uint8_t lv = GR_LV[grRange];
  uint16_t n = GR_N[grRange];
  char fn[16];
  if (clkValid()) {
    int yr, mo, d, hh, mi, ss, wd;
    epochToDate(nowLocal(), yr, mo, d, hh, mi, ss, wd);
    snprintf(fn, sizeof(fn), "H%02d%02d%02d.CSV", d, hh, mi);
  } else {
    snprintf(fn, sizeof(fn), "HIST.CSV");
  }
  SD.remove(fn);
  File f = SD.open(fn, FILE_WRITE);
  if (!f) return "SD open failed";
  f.println("time,cpu_t,gpu1_t,ssd_t,gpu2_t,cpu_l,gpu1_l,ssd_l,gpu2_l,ram_l,psum_w,pcpu_w,pram_w,pgu1_w,pgu2_w");
  for (uint16_t i = 0; i < n; i++) {                            // от старого к новому
    uint16_t k = n - 1 - i;
    if (k >= hCnt[lv]) continue;
    long ago = (long)k * GR_STEP[lv];
    String line;
    if (clkValid()) {
      int yr, mo, d, hh, mi, ss, wd;
      epochToDate(nowLocal() - ago, yr, mo, d, hh, mi, ss, wd);
      line = String(yr) + "-" + two(mo) + "-" + two(d) + " " + two(hh) + ":" + two(mi) + ":" + two(ss);
    } else {
      line = String("-") + String(ago);
    }
    for (uint8_t sr = 0; sr < H_SER; sr++) {
      line += ',';
      uint8_t e = histGet(lv, k, sr);
      if (sr >= POW_HIST_FIRST) { if (e != 255) line += String((int)(e / POW_HIST_SCALE + 0.5f)); }
      else if (e != 255) line += String(e / 2) + ((e & 1) ? ".5" : ".0");
    }
    f.println(line);
  }
  f.close();
  return String("Saved ") + fn;
}
#endif

static void graphScreen() {
  int16_t x, y;
  grDrawAll();
  int lastRev = -1;
  unsigned long lastDraw = 0;
  bool redraw = true;
  while (true) {
    pollEsp();
    if (millis() - lastActMs > 120000UL) return;                // забыли экран открытым
    if (redraw || (int)hRev != lastRev || millis() - lastDraw > 60000UL) {
      lastRev = (int)hRev; lastDraw = millis(); redraw = false;
      grPlot(); grLabels();
      if (grPow) grDrawYAxis();                 // авто-шкала могла смениться
    }
    grValues();
    if (!getTap(x, y)) continue;
    bool hit = false;
    for (uint8_t i = 0; i < 5; i++)
      if (inRect(x, y, 6 + i * 79, 6, 74, 38)) { grSel = i; grPow = false; hit = true; }
    if (inRect(x, y, 6 + 5 * 79, 6, 74, 38)) { grPow = !grPow; hit = true; }
    if (inRect(x, y, GR_RANGE_X, GR_BT_Y, 100, 44)) { grRange = (grRange + 1) % 7; drawButton(GR_RANGE_X, GR_BT_Y, 100, 44, GR_LBL[grRange], C_WHITE); hit = true; }
    if (hit) { grButtons(); grDrawYAxis(); redraw = true; grLastVals = "#"; continue; }
    if (inRect(x, y, 10, GR_BT_Y, 100, 44)) return;
#if SD_EXPORT
    if (inRect(x, y, 120, GR_BT_Y, 100, 44)) {
      gMsg = "Saving..."; gMsgMs = millis(); grValues();
      gMsg = sdExport(); gMsgMs = millis(); grLastVals = "#";
    }
#endif
  }
}

// ======================= ЭКРАН ТРЕВОГ =======================
static void alertsDrawStatic() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("Alerts");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  drawButton(10, 266, 88, 44, "15 min", C_WHITE);
  drawButton(102, 266, 88, 44, "1 hour", C_WHITE);
  drawButton(194, 266, 88, 44, "Resume", C_GREEN);
  drawButton(286, 266, 88, 44, "Clear", C_WHITE);
  drawButton(378, 266, 92, 44, "Back", C_YELLOW);
}

static void alertsDrawRows() {
  tft.setTextSize(2);
  for (uint8_t i = 0; i < JRN_N; i++) {
    int16_t y = 62 + i * 25;
    tft.setCursor(8, y + 4);
    if (i >= jrnCnt) {
      tft.setTextColor(C_GREY, C_BLACK);
      tft.print(padTo((i == 0) ? String("No alerts yet") : String(""), 39));
      continue;
    }
    uint8_t e = (jrnHead + JRN_N - 1 - i) % JRN_N;                // новые первыми
    String tm;
    if (jrnTs[e] != 0) {
      int yr, mo, d, hh, mi, ss, wd;
      epochToDate(jrnTs[e], yr, mo, d, hh, mi, ss, wd);
      tm = two(d) + " " + two(hh) + ":" + two(mi);
    } else {
      tm = fmtDur((millis() - jrnMs[e]) / 1000UL) + " ago";
    }
    tft.setTextColor(C_GREY, C_BLACK);   tft.print(padTo(tm, 11));
    tft.setTextColor(C_ORANGE, C_BLACK); tft.print(padTo(String(jrnTxt[e]), 28));
    pollEsp();
  }
}

static void setSnooze(unsigned long ms) {
  snoozeUntil = millis() + ms;
  if (snoozeUntil == 0) snoozeUntil = 1;
  alertActive = false;
}

static void alertsScreen() {
  alertsDrawStatic();
  String sigRows = "#", sigHdr = "#";
  int16_t x, y;
  while (true) {
    pollEsp();
    if (millis() - lastActMs > 120000UL) return;
    String sr = String(jrnRev) + "|" + String(millis() / 60000UL) + "|" + String((int)clkValid());
    if (sr != sigRows) { sigRows = sr; alertsDrawRows(); }
    bool sn = snoozeActive();
    String sh = sn ? (String("snoozed ") + String((snoozeUntil - millis()) / 60000UL + 1UL) + " min") : String("alerts on");
    if (sh != sigHdr) {
      sigHdr = sh;
      tft.setTextSize(2); tft.setTextColor(sn ? C_YELLOW : C_GREEN, C_BLACK);
      tft.setCursor(190, 22); tft.print(padTo(sh, 20));
    }
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 10, 266, 88, 44))        setSnooze(15UL * 60000UL);
    else if (inRect(x, y, 102, 266, 88, 44))  setSnooze(60UL * 60000UL);
    else if (inRect(x, y, 194, 266, 88, 44))  snoozeUntil = 0;
    else if (inRect(x, y, 286, 266, 88, 44))  { jrnCnt = 0; jrnHead = 0; jrnRev++; }
    else if (inRect(x, y, 378, 266, 92, 44))  return;
  }
}

// ======================= ЭКРАН СНА =======================
static void sleepDraw() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("Sleep");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  drawButton(20, 70, 440, 52, sleepEnabled ? "Sleep: ON" : "Sleep: OFF", sleepEnabled ? C_GREEN : C_GREY);
  tft.setTextSize(3); tft.setTextColor(sleepEnabled ? C_WHITE : C_GREY, C_BLACK);
  tft.setCursor(20, 146); tft.print(padTo(String("After ") + String((unsigned)sleepSecs) + " s", 20));
  tft.setTextSize(2); tft.setTextColor(C_GREY, C_BLACK);
  tft.setCursor(20, 176); tft.print("Screen goes black, backlight stays on");
  drawButton(20, 200, 440, 46, "Set seconds (5..3600)", C_WHITE);
  drawButton(20, 258, 440, 52, "Back", C_YELLOW);
}

static void sleepScreen() {
  sleepDraw();
  int16_t x, y;
  while (true) {
    pollEsp();
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 20, 70, 440, 52)) {
      sleepEnabled = !sleepEnabled; sleepSave(); sleepDraw();
    } else if (inRect(x, y, 20, 200, 440, 46)) {
      String v = "";
      if (editServer(v) && v.length()) {
        long n = v.toInt();                                   // лишние символы (точка, двоеточие) игнорируются
        if (n < 5) n = 5;
        if (n > 3600) n = 3600;
        sleepSecs = (uint16_t)n; sleepSave();
      }
      sleepDraw();
    } else if (inRect(x, y, 20, 258, 440, 52)) {
      return;
    }
  }
}

// пункты меню; Battery Cal есть только при BAT_ENABLED
enum { M_TOUCH, M_BAT, M_ESP, M_DOCKER, M_ALERTS, M_GRAPHS, M_SLEEP, M_COUNT };
static int8_t menuItems[M_COUNT];
static uint8_t menuN = 0;
static const char* const MENU_LBL[M_COUNT] = {"Touch Cal", "Battery Cal", "ESP-01", "Docker", "Alerts", "Graphs", "Sleep"};

static void menuBuild() {
  menuN = 0;
  for (uint8_t i = 0; i < M_COUNT; i++) {
#if !BAT_ENABLED
    if (i == M_BAT) continue;
#endif
    menuItems[menuN++] = i;
  }
}

static void menuRect(uint8_t n, int16_t &x, int16_t &y) {     // две колонки; при 7 пунктах строки плотнее
  x = (n % 2) ? 245 : 20;
  y = 64 + (n / 2) * (menuN > 6 ? 54 : 60);
}
static int16_t menuBackY() { return menuN > 6 ? 284 : 252; }
static int16_t menuBackH() { return menuN > 6 ? 30 : 52; }

static void drawMenu() {
  tft.fillScreen(C_BLACK);
  tft.setTextSize(3); tft.setTextColor(C_WHITE, C_BLACK);
  tft.setCursor(20, 15); tft.print("Menu");
  tft.drawFastHLine(10, 55, 460, C_GREY);
  for (uint8_t n = 0; n < menuN; n++) {
    int16_t x, y;
    menuRect(n, x, y);
    drawButton(x, y, 215, 48, MENU_LBL[menuItems[n]], C_WHITE);
  }
  drawButton(20, menuBackY(), 440, menuBackH(), "Back", C_YELLOW);
}

static void menuScreen() {
  menuBuild();
  drawMenu();
  int16_t x, y;
  while (true) {
    pollEsp();
    if (!getTap(x, y)) continue;
    if (inRect(x, y, 20, menuBackY(), 440, menuBackH())) return;
    for (uint8_t n = 0; n < menuN; n++) {
      int16_t bx, by;
      menuRect(n, bx, by);
      if (!inRect(x, y, bx, by, 215, 48)) continue;
      switch (menuItems[n]) {
        case M_TOUCH:  runCalibration(); break;
        case M_ESP:    espScreen(); break;
        case M_DOCKER: dockerScreen(); break;
        case M_ALERTS: alertsScreen(); break;
        case M_GRAPHS: graphScreen(); break;
        case M_SLEEP:  sleepScreen(); break;
#if BAT_ENABLED
        case M_BAT:    batCalScreen(); break;
#endif
        default: break;
      }
      drawMenu();
      break;
    }
  }
}

void setup() {
  // освободить PB3/PB4 от JTAG (SWD остаётся рабочим)
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

  Dbg.begin(115200);
  memset(hist, 255, sizeof(hist));           // история: 255 = нет данных
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(ESP_EN, OUTPUT);     digitalWrite(ESP_EN, HIGH);   // включить ESP-01
  EspSerial.begin(115200);

  pinMode(TOUCH_PEN, INPUT_PULLUP);
  pinMode(T_CS, OUTPUT);   digitalWrite(T_CS, HIGH);
  pinMode(T_SCK, OUTPUT);  digitalWrite(T_SCK, LOW);
  pinMode(T_MOSI, OUTPUT); digitalWrite(T_MOSI, LOW);
  pinMode(T_MISO, INPUT);

#if BL_ENABLED
  pinMode(BL_PIN, OUTPUT); digitalWrite(BL_PIN, HIGH);
#endif
  lastActMs = millis();
#if BAT_ENABLED
  analogReadResolution(12);
  pinMode(BAT_CHRG_PIN, INPUT_PULLUP);
  pinMode(BAT_STDBY_PIN, INPUT_PULLUP);
#endif
  tft.begin();
#if BAT_ENABLED
  batLoadCal();                              // поправка измерения батареи
#endif
  sleepLoad();                               // настройка сна
  loadCal();                                 // своя калибровка (приоритет)
  if (!calValid) loadBttCal();               // иначе — калибровка родной прошивки BTT
  Dbg.print("cal: own="); Dbg.print(calValid); Dbg.print(" btt="); Dbg.println(bttCalValid);
  // калибровка: если нет никакой или палец на экране при включении
  if ((!calValid && !bttCalValid) || touchPressed()) {
    runCalibration();
  }
  drawStatic();
  drawValues();
  lastActMs = millis();
}

static void goSleep() {
  sleeping = true;
  tft.fillScreen(C_BLACK);
#if SLEEP_LCD_CMD
  tft.sleepIn();
#endif
#if BL_ENABLED
  digitalWrite(BL_PIN, LOW);
#endif
}

static void wakeUp() {
#if BL_ENABLED
  digitalWrite(BL_PIN, HIGH);
#endif
#if SLEEP_LCD_CMD
  tft.sleepOut();
#endif
  sleeping = false; wakeReq = false; lastActMs = millis();
  drawStatic(); drawValues();
}

void loop() {
  pollEsp();
  batTick();
  static unsigned long lastCfgReq = 0;         // RSSI для иконки Wi-Fi в шапке
  if (millis() - lastCfgReq > 10000) { lastCfgReq = millis(); sendEsp("GETCFG"); }

  if (sleeping) {
    if (wakeReq || touchPressed()) {
      bool byTouch = touchPressed();
      wakeUp();
      if (byTouch) { while (touchPressed()) delay(10); delay(150); }   // первое касание только будит
    }
    return;
  }
  if (sleepEnabled && !alertActive && millis() - lastActMs > (unsigned long)sleepSecs * 1000UL) { goSleep(); return; }

  int16_t x, y;
  if (getTap(x, y)) {
    if (inRect(x, y, 372, 8, 100, 40)) {          // кнопка Menu
      menuScreen();
      drawStatic(); drawValues();
#if !BAT_ENABLED
    } else if (inRect(x, y, 286, 8, 80, 40)) {    // кнопка Graph
      graphScreen();
      drawStatic(); drawValues();
#endif
    } else if (inRect(x, y, 344, 282, 132, 34)) { // плашка Docker
      dockerScreen();
      drawStatic(); drawValues();
#if BAT_ENABLED
    } else if (inRect(x, y, 286, 6, 80, 49)) {    // нажатие на батарею: иконка <-> проценты
      batShowPct = !batShowPct; batSig = -1; batDraw();
#endif
    } else if (alertActive) {                     // тап по экрану гасит тревогу
      alertActive = false; alertDismissed = true; dirty = true;
    } else if (inRect(x, y, 0, 56, 480, 224)) {   // тап по центральной части: другая страница метрик
      curPage = (curPage + 1) % 3;
      drawBody(); drawPageDots(); drawValues();
    }
  }
  if (alertActive && millis() - alertMs > ALERT_HOLD_MS) { alertActive = false; dirty = true; }

  if (dirty || millis() - lastDrawMs > 1000) {
    drawValues();
    dirty = false;
    lastDrawMs = millis();
  }
  if (badLen && millis() - badMs > 350) statusDraw();   // прокрутка имён упавших контейнеров чаще раза в секунду
}