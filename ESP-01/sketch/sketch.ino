// =====================================================================
//  ESP-01 (ESP8266) v2.8: опрос Prometheus + Docker + время (NTP) + мощность -> UART на дисплей TFT35
//
//  ПЛАТА В ARDUINO IDE:  Generic ESP8266 Module
//     Flash Size: 1MB (FS:none OTA:~502KB)     <-- ВАЖНО для OTA
//     Flash Mode: DOUT,  Upload Speed: 115200
//
//  Возможности:
//   - опрос Prometheus: 4 температуры (CPU / GPU1 / SSD / GPU2) -> строки "CPU:63.4" ...
//     и 5 нагрузок в % (LCPU / LGPU1 / LSSD / LGPU2 / LRAM) -> строки "LCPU:23.5" ...
//     тревоги "ALERT:CPU_OVERHEAT:95.4" (после значений)
//   - опрос сервиса docker-status (см. claude_code_prompts.txt) раз в 10 с -> на дисплей
//     "DK:всего;ок;внимание;плохо", "DKI:<код>|<сек>|<рестарты>|<exit>|<имя>", "DKEND:n"
//     и тревога "ALERT:DOWN имя" / "ALERT:UNHEALTHY имя" при упавших контейнерах
//   - время по NTP: строка "TIME:<секунд локального времени>" раз в 10 с; часовой пояс (минуты от UTC)
//     задаётся на веб-странице или командой SETTZ:<минуты> (по умолчанию +180, Москва)
//   - 6 дополнительных метрик для 2-й страницы дисплея (раз в ~15 с): XUPT (аптайм, с), XDSKP (свободно
//     на диске, %), XDSKG (свободно, ГБ), XNRX / XNTX (сеть, Мбит/с), XTGT (число упавших targets)
//   - 3-я страница дисплея, мощность в ваттах (тот же цикл, что и доп. метрики): PSUM (оценка суммы CPU Package + память +
//     видеокарты, это НЕ мощность из розетки), PCPU, PRAM (память CPU), PGU1, PGU2
//   - ОБНОВЛЕНИЕ ПО Wi-Fi (OTA): в Arduino IDE в списке портов появится сетевой порт
//     "esp-monitor", либо загрузка .bin в браузере: http://esp-monitor.local/update
//   - веб-страница настроек http://esp-monitor.local (или по IP): запросы PromQL,
//     пороги, адрес сервера, Wi-Fi; там же таблица состояния датчиков
//   - если сеть не найдена 60 с - поднимает свою точку доступа TFT-Monitor-Setup
//     (пароль AP_PASS), настройки по адресу http://192.168.4.1
//   - статусы ошибок для дисплея: ERR:NOWIFI, ERR:PROM, ERR:QUERY:<датчики>
//   - команды по UART (от дисплея или из Serial Monitor, 115200, Newline):
//       GETCFG | SCAN | STATUS | DKNOW | MODE:FAST | MODE:SLEEP
//       SETSRV:ip[:port] | SETDK:ip[:port] (пусто = выключить) | SETWIFI:<ssid>\t<пароль> | SETTZ:<минуты>
//  Все настройки хранятся в EEPROM и переживают перезагрузку.
// =====================================================================
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <time.h>

#define FW_VERSION "2.8"

// ---- Wi-Fi по умолчанию (нужно только для самого первого запуска; потом сеть выбирается с дисплея / из браузера)
const char* DEF_SSID = "MTS_GPON_4DFE";
const char* DEF_PASS = "CBLyUuTEau";

// ---- Prometheus по умолчанию (потом меняется с дисплея или в браузере)
#define DEF_HOST "192.168.1.72"
#define DEF_PORT 9090

// ---- сервис docker-status по умолчанию (потом меняется с дисплея или в браузере)
#define DEF_DK_HOST "192.168.1.72"
#define DEF_DK_PORT 8099

// ---- Доступ: имя в сети, пароль OTA и веб-страницы, точка доступа. ПОМЕНЯЙ пароли.
#define HOSTNAME  "esp-monitor"
#define WEB_USER  "admin"
#define WEB_PASS  "monitor"          // и для страницы настроек, и для /update, и для OTA из Arduino IDE
#define AP_SSID   "TFT-Monitor-Setup"
#define AP_PASS   "tftmonitor"       // не короче 8 символов

// ---- Датчики. Имена ключей фиксированы (их ждёт дисплей).
//      Запросы и пороги ниже - значения ПО УМОЛЧАНИЮ; дальше их можно менять в браузере.
//      Названия метрик - ПРИМЕРЫ: проверь на http://<сервер>:9090/graph, что у тебя есть на самом деле.
//      Запрос должен возвращать ОДНО число (оберни в max(...) / avg(...)).
#define NS 4
static const char* KEYS[NS] = {"CPU", "GPU1", "SSD", "GPU2"};
static const char* DEF_QUERY[NS] = {
  "max(node_hwmon_temp_celsius{chip=~\".*coretemp.*\"})",
  "max(nvidia_smi_temperature_gpu{uuid=\"f8c2a01d-34ee-3ad7-e426-b61534518069\"})",
  "max(smartctl_device_temperature{temperature_type=\"current\"})",
  "max(nvidia_smi_temperature_gpu{uuid=\"c32ad3ff-bf4a-45a6-dbb9-edcc45b7d3d0\"})"
};
static const float DEF_ALERT[NS] = {90.0, 85.0, 70.0, 85.0};

// ---- Нагрузка, % (0..100). Тоже ПРИМЕРЫ под Linux node_exporter + nvidia_gpu_exporter.
//      Если сервер на Windows (windows_exporter), например:
//        LCPU : 100 - avg(rate(windows_cpu_time_total{mode="idle"}[1m])) * 100
//        LRAM : 100 - 100 * windows_os_physical_memory_free_bytes / windows_cs_physical_memory_bytes
//      Для SSD вместо занятости можно показывать заполнение диска, это тоже "нагрузка".
//      Точные запросы под твои экспортеры подбирает ПРОМПТ 2 из claude_code_prompts.txt.
#define NL 5
static const char* LKEYS[NL] = {"LCPU", "LGPU1", "LSSD", "LGPU2", "LRAM"};
static const char* DEF_LQUERY[NL] = {
  "100 - (avg(rate(node_cpu_seconds_total{mode=\"idle\"}[1m])) * 100)",
  "max(nvidia_smi_utilization_gpu_ratio{uuid=\"f8c2a01d-34ee-3ad7-e426-b61534518069\"}) * 100",
  "max(rate(node_disk_io_time_seconds_total{device!~\"loop.*|ram.*|dm-.*\"}[1m])) * 100",
  "max(nvidia_smi_utilization_gpu_ratio{uuid=\"c32ad3ff-bf4a-45a6-dbb9-edcc45b7d3d0\"}) * 100",
  "100 * (1 - node_memory_MemAvailable_bytes / node_memory_MemTotal_bytes)"
};

// ---- Дополнительные метрики для 2-й страницы дисплея. Опрашиваются реже (раз в ~15 с).
//      Единицы: XUPT - секунды, XDSKP - % свободного места, XDSKG - свободно ГБ,
//      XNRX / XNTX - Мбит/с, XTGT - сколько targets Prometheus сейчас недоступны.
//      Запросы - ПРИМЕРЫ под Linux node_exporter; реальные подбирает ПРОМПТ 3 из claude_code_prompts.txt.
#define NX 6
#define NTL (NS + NL)                        // датчики, которые опрашиваются каждый цикл
#define NP 5                                 // мощность, 3-я страница дисплея
#define NTE (NS + NL + NX)                   // всё, кроме мощности
#define NT (NS + NL + NX + NP)
static const char* XKEYS[NX] = {"XUPT", "XDSKP", "XDSKG", "XNRX", "XNTX", "XTGT"};
static const char* DEF_XQUERY[NX] = {
  "time() - max(node_boot_time_seconds)",
  "max(100 * node_filesystem_avail_bytes{mountpoint=\"/\"} / node_filesystem_size_bytes{mountpoint=\"/\"})",
  "max(node_filesystem_avail_bytes{mountpoint=\"/\"}) / 1e9",
  "sum(rate(node_network_receive_bytes_total{device!~\"lo|docker.*|veth.*|br-.*\"}[1m])) * 8 / 1e6",
  "sum(rate(node_network_transmit_bytes_total{device!~\"lo|docker.*|veth.*|br-.*\"}[1m])) * 8 / 1e6",
  "count(up == 0) or vector(0)"
};
static const char* PKEYS[NP] = {"PSUM", "PCPU", "PRAM", "PGU1", "PGU2"};
static const char* DEF_PQUERY[NP] = {       // Linux: ohm_* (OpenHardwareMonitor) больше нет; ватты CPU/памяти
  "sum(nvidia_smi_power_draw_watts)",          // hwmon не отдает -> PSUM = сумма по обеим 2080 Ti
  "",                                          // PCPU: нечего показывать (пустой запрос = выключен)
  "",                                          // PRAM: нечего показывать
  "max(nvidia_smi_power_draw_watts{uuid=\"f8c2a01d-34ee-3ad7-e426-b61534518069\"})",
  "max(nvidia_smi_power_draw_watts{uuid=\"c32ad3ff-bf4a-45a6-dbb9-edcc45b7d3d0\"})"
};
#define X_EVERY 5                            // дополнительные метрики - раз в 5 циклов

// ---------------------------------------------------------------------
//  Хранилище
// ---------------------------------------------------------------------
#define SRV_ADDR   0
#define WIFI_ADDR  64
#define SENS_ADDR  192
#define LOAD_ADDR  860      // SensBlk занимает 192..851
#define DK_ADDR    1680     // LoadBlk занимает 860..1663
#define TZ_ADDR    1740     // DkCfg занимает 1680..1727
#define X_ADDR     1760     // XBlk занимает 1760..2723 (поэтому EEPROM 4096)
#define P_ADDR     2740     // PBlk занимает 2740..3547
#define EEPROM_SZ  4096
static const uint32_t SRV_MAGIC  = 0x5E7C0001;
static const uint32_t WIFI_MAGIC = 0x5E7C0002;
static const uint32_t SENS_MAGIC = 0x5E7C0003;
static const uint32_t DK_MAGIC   = 0x5E7C0004;
static const uint32_t LOAD_MAGIC = 0x5E7C0005;
static const uint32_t TZ_MAGIC   = 0x5E7C0006;
static const uint32_t X_MAGIC    = 0x5E7C0007;
static const uint32_t P_MAGIC    = 0x5E7C0008;

struct SrvCfg   { uint32_t magic; char host[40]; uint16_t port; };
struct WifiCfg  { uint32_t magic; char ssid[33]; char pass[65]; };
struct SensCfg  { char query[160]; float alertAbove; };
struct SensBlk  { uint32_t magic; SensCfg s[NS]; };
struct LoadBlk  { uint32_t magic; char q[NL][160]; };
struct DkCfg    { uint32_t magic; char host[40]; uint16_t port; uint8_t alert; uint8_t pad; };
struct TzCfg    { uint32_t magic; int16_t minutes; int16_t pad; };
struct XBlk     { uint32_t magic; char q[NX][160]; };
struct PBlk     { uint32_t magic; char q[NP][160]; };

static SrvCfg  srv;
static WifiCfg wifiCfg, wifiNew;
static SensBlk sens;
static LoadBlk loads;
static DkCfg   dk;
static TzCfg   tzc;
static XBlk    xq;
static PBlk    pq;

static unsigned long checkDelay = 3000;     // MODE:FAST = 3 с, MODE:SLEEP = 60 с
static unsigned long lastCheckTime = 0;
static unsigned long backoffUntil = 0;      // после сбоя связи с Prometheus пауза, чтобы страница и OTA оставались отзывчивыми
static int lastLinkCode = 0;
static const unsigned long BACKOFF_MS = 15000;
static bool wifiPending = false;
static unsigned long wifiPendMs = 0;

// состояние последнего опроса (для STATUS и веб-страницы); индексы 0..NS-1 - температуры, NS..NT-1 - нагрузка
static uint8_t sState[NT];                  // 0 не опрашивался, 1 ок, 2 нет данных, 3 нет связи
static int sCode[NT];
static float sVal[NT];
static unsigned long sMs[NT];

// docker-status
static const unsigned long DK_PERIOD_MS = 10000, DK_BACKOFF_MS = 20000;
static unsigned long dkLastPoll = 0, dkBackoffUntil = 0, dkOkMs = 0;
static int dkErrCode = 0;
static uint8_t dkState = 0;                 // 0 не опрашивался, 1 ок, 3 нет связи / ошибка
static int dkTotal = 0, dkOkN = 0, dkWarnN = 0, dkBadN = 0, dkLines = 0;
static String dkAlertText = "";             // пусто = тревоги нет
static unsigned long dkAlertMs = 0;

static inline const char* keyOf(int i)  { return i < NS ? KEYS[i] : (i < NTL ? LKEYS[i - NS] : (i < NTE ? XKEYS[i - NTL] : PKEYS[i - NTE])); }
static inline char* queryOf(int i)      { return i < NS ? sens.s[i].query : (i < NTL ? loads.q[i - NS] : (i < NTE ? xq.q[i - NTL] : pq.q[i - NTE])); }
static const size_t QLEN = 160;

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
static bool svcStarted = false, apOn = false;
static unsigned long offlineSince = 0;
static String urlEncode(const char* s);      // определён ниже, в разделе Prometheus
static void service();                       // определён ниже

static void saveSrv()  { EEPROM.put(SRV_ADDR, srv);  EEPROM.commit(); }
static void saveWifi() { EEPROM.put(WIFI_ADDR, wifiCfg); EEPROM.commit(); }
static void saveSens() { EEPROM.put(SENS_ADDR, sens); EEPROM.put(LOAD_ADDR, loads); EEPROM.put(X_ADDR, xq); EEPROM.put(P_ADDR, pq); EEPROM.commit(); }
static void saveTz()   { EEPROM.put(TZ_ADDR, tzc); EEPROM.commit(); }
static void saveDk()   { EEPROM.put(DK_ADDR, dk); EEPROM.commit(); }

static void cfgLoad() {
  EEPROM.begin(EEPROM_SZ);
  EEPROM.get(SRV_ADDR, srv);
  if (srv.magic != SRV_MAGIC || srv.host[sizeof(srv.host) - 1] != 0) {
    memset(&srv, 0, sizeof(srv));
    srv.magic = SRV_MAGIC;
    strcpy(srv.host, DEF_HOST);
    srv.port = DEF_PORT;
  }
  EEPROM.get(WIFI_ADDR, wifiCfg);
  if (wifiCfg.magic != WIFI_MAGIC || wifiCfg.ssid[sizeof(wifiCfg.ssid) - 1] != 0 || wifiCfg.pass[sizeof(wifiCfg.pass) - 1] != 0)
    memset(&wifiCfg, 0, sizeof(wifiCfg));
  EEPROM.get(SENS_ADDR, sens);
  bool ok = (sens.magic == SENS_MAGIC);
  for (int i = 0; i < NS && ok; i++) if (sens.s[i].query[sizeof(sens.s[i].query) - 1] != 0) ok = false;
  if (!ok) {
    memset(&sens, 0, sizeof(sens));
    sens.magic = SENS_MAGIC;
    for (int i = 0; i < NS; i++) {
      strncpy(sens.s[i].query, DEF_QUERY[i], sizeof(sens.s[i].query) - 1);
      sens.s[i].alertAbove = DEF_ALERT[i];
    }
  }
  EEPROM.get(LOAD_ADDR, loads);
  ok = (loads.magic == LOAD_MAGIC);
  for (int i = 0; i < NL && ok; i++) if (loads.q[i][QLEN - 1] != 0) ok = false;
  if (!ok) {
    memset(&loads, 0, sizeof(loads));
    loads.magic = LOAD_MAGIC;
    for (int i = 0; i < NL; i++) strncpy(loads.q[i], DEF_LQUERY[i], QLEN - 1);
  }
  EEPROM.get(DK_ADDR, dk);
  if (dk.magic != DK_MAGIC || dk.host[sizeof(dk.host) - 1] != 0) {
    memset(&dk, 0, sizeof(dk));
    dk.magic = DK_MAGIC;
    strcpy(dk.host, DEF_DK_HOST);
    dk.port = DEF_DK_PORT;
    dk.alert = 1;
  }
  EEPROM.get(TZ_ADDR, tzc);
  if (tzc.magic != TZ_MAGIC || tzc.minutes < -720 || tzc.minutes > 840) {
    memset(&tzc, 0, sizeof(tzc));
    tzc.magic = TZ_MAGIC;
    tzc.minutes = 180;                       // Москва
  }
  EEPROM.get(X_ADDR, xq);
  ok = (xq.magic == X_MAGIC);
  for (int i = 0; i < NX && ok; i++) if (xq.q[i][QLEN - 1] != 0) ok = false;
  if (!ok) {
    memset(&xq, 0, sizeof(xq));
    xq.magic = X_MAGIC;
    for (int i = 0; i < NX; i++) strncpy(xq.q[i], DEF_XQUERY[i], QLEN - 1);
  }
  EEPROM.get(P_ADDR, pq);
  ok = (pq.magic == P_MAGIC);
  for (int i = 0; i < NP && ok; i++) if (pq.q[i][QLEN - 1] != 0) ok = false;
  if (!ok) {
    memset(&pq, 0, sizeof(pq));
    pq.magic = P_MAGIC;
    for (int i = 0; i < NP; i++) strncpy(pq.q[i], DEF_PQUERY[i], QLEN - 1);
  }
}

// ---------------------------------------------------------------------
//  Сеть
// ---------------------------------------------------------------------
static void sendCfg() {
  String d = dk.host[0] ? (String(dk.host) + ":" + String((unsigned)dk.port)) : String("");
  Serial.printf("CFG:srv=%s:%u;wifi=%d;ip=%s;rssi=%d;dk=%s;ssid=%s\n",
                srv.host, (unsigned)srv.port,
                (int)(WiFi.status() == WL_CONNECTED),
                WiFi.localIP().toString().c_str(),
                (int)WiFi.RSSI(),
                d.c_str(),
                WiFi.SSID().c_str());
}

static void doScan() {
  int n = WiFi.scanNetworks();               // блокирующий, 2-5 секунд
  int idx[32], m = 0;
  for (int i = 0; i < n && m < 32; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;
    bool dup = false;
    for (int j = 0; j < m; j++) if (WiFi.SSID(idx[j]) == s) { dup = true; break; }
    if (!dup) idx[m++] = i;
  }
  for (int a = 0; a < m; a++)
    for (int b = a + 1; b < m; b++)
      if (WiFi.RSSI(idx[b]) > WiFi.RSSI(idx[a])) { int t = idx[a]; idx[a] = idx[b]; idx[b] = t; }
  if (m > 12) m = 12;
  for (int a = 0; a < m; a++) {
    int i = idx[a];
    Serial.printf("NET:%d;%d;%s\n", (int)WiFi.RSSI(i), (int)(WiFi.encryptionType(i) != ENC_TYPE_NONE), WiFi.SSID(i).c_str());
  }
  Serial.printf("NETEND:%d\n", m);
  WiFi.scanDelete();
}

// Смена сети (с дисплея или из браузера). Новая сеть запоминается только после успешного подключения.
static bool startWifiChange(const String &ssid, const String &pass) {
  if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64) return false;
  memset(&wifiNew, 0, sizeof(wifiNew));
  wifiNew.magic = WIFI_MAGIC;
  ssid.toCharArray(wifiNew.ssid, sizeof(wifiNew.ssid));
  pass.toCharArray(wifiNew.pass, sizeof(wifiNew.pass));
  WiFi.disconnect();
  WiFi.begin(wifiNew.ssid, wifiNew.pass);
  wifiPending = true;
  wifiPendMs = millis();
  return true;
}

static void wifiTick() {
  if (!wifiPending) return;
  if (WiFi.status() == WL_CONNECTED) {
    wifiCfg = wifiNew;
    saveWifi();
    wifiPending = false;
    Serial.println("ACK:WIFIOK");
    sendCfg();
  } else if (millis() - wifiPendMs > 20000) {
    wifiPending = false;
    Serial.println("ERR:WIFI");
    if (wifiCfg.magic == WIFI_MAGIC && wifiCfg.ssid[0]) WiFi.begin(wifiCfg.ssid, wifiCfg.pass);   // назад на прежнюю
    else WiFi.begin(DEF_SSID, DEF_PASS);
  }
}

// ---------------------------------------------------------------------
//  Время (NTP). Часовой пояс прибавляется вручную, поэтому configTime идёт с нулевым смещением.
// ---------------------------------------------------------------------
static bool ntpStarted = false;
static unsigned long timeSentMs = 0;

static bool timeValid() { return time(nullptr) > 1600000000UL; }

static void sendTime() {
  if (!timeValid()) return;
  long loc = (long)time(nullptr) + (long)tzc.minutes * 60L;
  Serial.printf("TIME:%ld\n", loc);
  timeSentMs = millis();
}

static void startServices() {                // один раз, после первого подключения
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(WEB_PASS);
  ArduinoOTA.begin();                        // поднимает и mDNS (esp-monitor.local)
  MDNS.addService("http", "tcp", 80);
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  ntpStarted = true;
  svcStarted = true;
}

static void netTick() {
  if (WiFi.status() == WL_CONNECTED) {
    offlineSince = 0;
    if (!svcStarted) startServices();
    if (apOn && !wifiPending) { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); apOn = false; }
  } else {
    if (offlineSince == 0) offlineSince = millis();
    if (!apOn && millis() - offlineSince > 60000) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_SSID, AP_PASS);
      apOn = true;
    }
  }
}

// ---------------------------------------------------------------------
//  Команды по UART
// ---------------------------------------------------------------------
static void sendStatus() {
  Serial.printf("STATUS:ver=%s;up=%lu;heap=%u;wifi=%d;ssid=%s;ip=%s;ap=%d;srv=%s:%u;delay=%lu\n",
                FW_VERSION, millis() / 1000, (unsigned)ESP.getFreeHeap(),
                (int)(WiFi.status() == WL_CONNECTED), WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                (int)apOn, srv.host, (unsigned)srv.port, checkDelay);
  for (int i = 0; i < NT; i++) {
    const char* st = (sState[i] == 1) ? "ok" : (sState[i] == 2) ? "nodata" : (sState[i] == 3) ? "nolink" : "-";
    Serial.printf("STATUS:%s;state=%s;http=%d;val=%s;age=%lus\n", keyOf(i), st, sCode[i],
                  String(sVal[i], 1).c_str(), sMs[i] ? (millis() - sMs[i]) / 1000 : 0);
  }
  Serial.printf("STATUS:TIME;valid=%d;tz=%d;now=%ld\n", (int)timeValid(), (int)tzc.minutes, timeValid() ? (long)time(nullptr) : 0L);
  Serial.printf("STATUS:DOCKER;state=%s;http=%d;total=%d;ok=%d;warn=%d;bad=%d;age=%lus\n",
                dkState == 1 ? "ok" : (dkState == 3 ? "error" : "-"), dkErrCode, dkTotal, dkOkN, dkWarnN, dkBadN,
                dkOkMs ? (millis() - dkOkMs) / 1000 : 0);
}

static void handleCmd(String l) {            // l - строка без \n и \r (пробелы не режем: они могут быть в пароле)
  String t = l; t.trim();
  if (t == "MODE:SLEEP") { checkDelay = 60000; return; }
  if (t == "MODE:FAST")  { checkDelay = 3000;  return; }
  if (t == "GETCFG")     { sendCfg(); return; }
  if (t == "SCAN")       { doScan();  return; }
  if (t == "STATUS")     { sendStatus(); return; }
  if (t == "DKNOW")      { dkLastPoll = 0; dkBackoffUntil = 0; return; }
  if (t.startsWith("SETTZ:")) {
    long m = t.substring(6).toInt();
    if (m < -720 || m > 840) { Serial.println("ERR:SETTZ"); return; }
    tzc.minutes = (int16_t)m;
    saveTz();
    Serial.println("ACK:SETTZ");
    sendTime();
    return;
  }
  if (t.startsWith("SETDK:")) {
    String v = t.substring(6); v.trim();
    int c = v.indexOf(':');
    String h = (c < 0) ? v : v.substring(0, c);
    long p = (c < 0) ? DEF_DK_PORT : v.substring(c + 1).toInt();
    if (h.length() >= sizeof(dk.host) || (h.length() && (p <= 0 || p > 65535))) { Serial.println("ERR:SETDK"); return; }
    memset(dk.host, 0, sizeof(dk.host));
    h.toCharArray(dk.host, sizeof(dk.host));
    if (h.length()) dk.port = (uint16_t)p;
    saveDk();
    Serial.println("ACK:SETDK");
    sendCfg();
    dkLastPoll = 0; dkBackoffUntil = 0;
    if (!h.length()) { dkState = 0; dkAlertText = ""; }
    return;
  }
  if (t.startsWith("SETSRV:")) {
    String v = t.substring(7); v.trim();
    int c = v.indexOf(':');
    String h = (c < 0) ? v : v.substring(0, c);
    long p = (c < 0) ? 9090 : v.substring(c + 1).toInt();
    if (h.length() == 0 || h.length() >= sizeof(srv.host) || p <= 0 || p > 65535) { Serial.println("ERR:SETSRV"); return; }
    memset(srv.host, 0, sizeof(srv.host));
    h.toCharArray(srv.host, sizeof(srv.host));
    srv.port = (uint16_t)p;
    saveSrv();
    Serial.println("ACK:SETSRV");
    sendCfg();
    lastCheckTime = 0;                       // сразу опросить новый сервер
    backoffUntil = 0;
    return;
  }
  if (l.startsWith("SETWIFI:")) {
    String v = l.substring(8);
    int tab = v.indexOf('\t');
    if (tab <= 0 || !startWifiChange(v.substring(0, tab), v.substring(tab + 1))) Serial.println("ERR:WIFI");
  }
}

static void pollCmds() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleCmd(buf); buf = ""; }
    else if (c != '\r' && buf.length() < 160) buf += c;
  }
  wifiTick();
}

// ---------------------------------------------------------------------
//  Веб-страница настроек
// ---------------------------------------------------------------------
static String he(const String &s) {          // экранирование для HTML-атрибутов
  String r;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') r += "&amp;"; else if (c == '<') r += "&lt;"; else if (c == '>') r += "&gt;";
    else if (c == '"') r += "&quot;"; else r += c;
  }
  return r;
}

static bool webAuth() {
  if (server.authenticate(WEB_USER, WEB_PASS)) return true;
  server.requestAuthentication();
  return false;
}

static void handleRoot() {
  if (!webAuth()) return;
  String page;
  page.reserve(13500);                        // страница собирается целиком и уходит ОДНИМ ответом (мелкие куски на ESP8266 тормозят)
  page += ("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
                     "<meta name=color-scheme content='light dark'><title>TFT Monitor</title>"
                     "<style>body{font-family:sans-serif;max-width:780px;margin:1em auto;padding:0 1em;background:#fff;color:#111}"
                     "@media(prefers-color-scheme:dark){body{background:#181818;color:#eee}a{color:#8ab4f8}small{color:#aaa!important}input{background:#2a2a2a;color:#eee;border:1px solid #555}}"
                     "input{width:100%;box-sizing:border-box;padding:6px;margin:2px 0 10px}td,th{padding:4px 8px;text-align:left}"
                     "h3{margin-top:1.4em}.ok{color:#2a2}.bad{color:#d33}small{color:#666}</style><body>");
  page += (String("<h2>TFT Monitor ESP-01 v") + FW_VERSION + "</h2><small>IP: " + WiFi.localIP().toString() +
                     " &middot; сеть: " + he(WiFi.SSID()) + " &middot; сигнал: " + String(WiFi.RSSI()) + " dBm &middot; heap: " +
                     String(ESP.getFreeHeap()) + " &middot; аптайм: " + String(millis() / 1000) + " с &middot; <a href='/update'>обновить прошивку</a>"
                     " &middot; <a href='/export'>скачать настройки</a> &middot; <a href='/import'>загрузить настройки</a></small>");

  page += ("<h3>Состояние датчиков</h3><table><tr><th>Датчик<th>Состояние<th>HTTP<th>Значение<th>Давность</tr>");
  for (int i = 0; i < NT; i++) {
    const char* st = (sState[i] == 1) ? "<span class=ok>ok</span>" : (sState[i] == 2) ? "<span class=bad>нет данных</span>" :
                     (sState[i] == 3) ? "<span class=bad>нет связи</span>" : "-";
    page += (String("<tr><td>") + keyOf(i) + "<td>" + st + "<td>" + String(sCode[i]) + "<td>" + String(sVal[i], 1) +
                       "<td>" + (sMs[i] ? String((millis() - sMs[i]) / 1000) + " с" : String("-")) + "</tr>");
  }
  page += (String("<tr><td>Docker<td>") + (dkState == 1 ? "<span class=ok>ok</span>" : (dkState == 3 ? "<span class=bad>ошибка</span>" : "-")) +
                     "<td>" + String(dkErrCode) + "<td>всего " + String(dkTotal) + ", ок " + String(dkOkN) + ", вним. " + String(dkWarnN) +
                     ", плохо " + String(dkBadN) + "<td>" + (dkOkMs ? String((millis() - dkOkMs) / 1000) + " с" : String("-")) + "</tr>");
  page += ("</table><form method=post action=/save><h3>Сервер Prometheus</h3>");
  page += (String("Адрес<input name=host value=\"") + he(srv.host) + "\">Порт<input name=port value=\"" + String(srv.port) + "\">");
  page += ("<h3>Запросы PromQL и пороги тревоги (°C)</h3>");
  for (int i = 0; i < NS; i++) {
    page += (String("<b>") + KEYS[i] + "</b> (пустой запрос = датчик отключён)<input name=q" + i + " value=\"" + he(sens.s[i].query) +
                       "\">Порог тревоги<input name=t" + i + " value=\"" + String(sens.s[i].alertAbove, 1) + "\">");
  }
  page += ("<h3>Нагрузка, % (запрос должен вернуть число 0..100)</h3>");
  for (int i = NS; i < NTL; i++) {
    page += (String("<b>") + keyOf(i) + "</b> (пустой запрос = не показывать)<input name=q" + i + " value=\"" + he(queryOf(i)) + "\">");
  }
  page += ("<h3>2-я страница дисплея</h3><small>XUPT - аптайм, секунды &middot; XDSKP - свободно на диске, % &middot; XDSKG - свободно, ГБ &middot; "
           "XNRX / XNTX - сеть, Мбит/с &middot; XTGT - число упавших targets. Пустой запрос = строка не показывается.</small><br>");
  for (int i = NTL; i < NTE; i++) {
    page += (String("<b>") + keyOf(i) + "</b><input name=q" + i + " value=\"" + he(queryOf(i)) + "\">");
  }
  page += ("<h3>3-я страница дисплея: мощность, Вт</h3><small>PSUM - оценка суммы CPU Package + память + видеокарты (не мощность из розетки) &middot; "
           "PCPU - CPU Package &middot; PRAM - память CPU &middot; PGU1 / PGU2 - видеокарты. Пустой запрос = строка не показывается.</small><br>");
  for (int i = NTE; i < NT; i++) {
    page += (String("<b>") + keyOf(i) + "</b><input name=q" + i + " value=\"" + he(queryOf(i)) + "\">");
  }
  page += (String("<h3>Время</h3><small>Часы берутся из интернета (NTP). Сейчас: ") +
           (timeValid() ? String("получено") : String("ещё не получено")) +
           ".</small><br>Часовой пояс, минут от UTC (Москва = 180)<input name=tz value=\"" + String((int)tzc.minutes) + "\">");
  page += (String("<h3>Docker (сервис docker-status)</h3><small>Адрес пустой = не опрашивать.</small><br>Адрес<input name=dkhost value=\"") +
                     he(dk.host) + "\">Порт<input name=dkport value=\"" + String(dk.port) + "\">"
                     "<label><input type=checkbox name=dkalert value=1 style='width:auto' " + (dk.alert ? "checked" : "") +
                     "> тревога на дисплее, если контейнер упал / unhealthy</label>");
  page += ("<h3>Wi-Fi</h3><small>Оставь пустым, чтобы не менять. Пароль для текущей сети можно не вводить.</small>"
                     "Сеть (SSID)<input name=ssid>Пароль<input name=pass type=password>"
                     "<button style='padding:8px 20px;font-size:1.1em'>Сохранить</button></form>");
  server.client().setNoDelay(true);
  server.send(200, "text/html; charset=utf-8", page);
}

static void handleSave() {
  if (!webAuth()) return;
  String h = server.arg("host"); h.trim();  long p = server.arg("port").toInt();
  if (h.length() && h.length() < sizeof(srv.host) && p > 0 && p < 65536) {
    memset(srv.host, 0, sizeof(srv.host));
    h.toCharArray(srv.host, sizeof(srv.host));
    srv.port = (uint16_t)p;
    saveSrv();
  }
  for (int i = 0; i < NT; i++) {
    String q = server.arg(String("q") + i); q.trim();
    if (q.length() < QLEN) {
      memset(queryOf(i), 0, QLEN);
      q.toCharArray(queryOf(i), QLEN);
    }
    if (i < NS) {
      String t = server.arg(String("t") + i); t.trim();
      if (t.length()) sens.s[i].alertAbove = t.toFloat();
    }
  }
  saveSens();
  String tzs = server.arg("tz"); tzs.trim();
  if (tzs.length()) {
    long m = tzs.toInt();
    if (m >= -720 && m <= 840) { tzc.minutes = (int16_t)m; saveTz(); sendTime(); }
  }
  String dh = server.arg("dkhost"); dh.trim();
  long dp = server.arg("dkport").toInt();
  if (dh.length() < sizeof(dk.host) && (dh.length() == 0 || (dp > 0 && dp < 65536))) {
    memset(dk.host, 0, sizeof(dk.host));
    dh.toCharArray(dk.host, sizeof(dk.host));
    if (dh.length()) dk.port = (uint16_t)dp;
    dk.alert = server.hasArg("dkalert") ? 1 : 0;
    saveDk();
    dkLastPoll = 0; dkBackoffUntil = 0;
    if (!dh.length()) { dkState = 0; dkAlertText = ""; }
  }
  String ss = server.arg("ssid"), pw = server.arg("pass");
  ss.trim();
  if (ss.length() && (pw.length() || ss != WiFi.SSID())) startWifiChange(ss, pw);
  lastCheckTime = 0;                         // сразу опросить с новыми настройками
  backoffUntil = 0;
  server.sendHeader("Location", "/");
  server.send(303);
}

// ---------------------------------------------------------------------
//  Экспорт / импорт настроек (текстовый файл, значения url-encoded)
// ---------------------------------------------------------------------
static String urlDecode(const String &s) {
  String r;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '%' && i + 2 < s.length()) {
      char h[3] = {s[i + 1], s[i + 2], 0};
      r += (char)strtol(h, nullptr, 16); i += 2;
    } else if (c == '+') r += ' ';
    else r += c;
  }
  return r;
}

static int keyIndex(const String &k) {       // "CPU" / "LGPU1" / ... -> индекс 0..NT-1
  for (int i = 0; i < NT; i++) if (k == keyOf(i)) return i;
  return -1;
}

static void handleExport() {
  if (!webAuth()) return;
  String t;
  t.reserve(6000);
  t += "# TFT-Monitor ESP-01 v" FW_VERSION " - настройки. Файл содержит пароли - храни в безопасности.\n";
  t += "SRV=" + urlEncode((String(srv.host) + ":" + String((unsigned)srv.port)).c_str()) + "\n";
  String dkv = dk.host[0] ? (String(dk.host) + ":" + String((unsigned)dk.port)) : String("");
  t += "DK=" + urlEncode(dkv.c_str()) + "\n";
  t += "DKALERT=" + String((int)dk.alert) + "\n";
  t += "TZ=" + String((int)tzc.minutes) + "\n";
  if (wifiCfg.magic == WIFI_MAGIC && wifiCfg.ssid[0]) {          // иначе сеть из DEF_SSID, пароля в EEPROM нет
    t += "WIFI_SSID=" + urlEncode(wifiCfg.ssid) + "\n";
    t += "WIFI_PASS=" + urlEncode(wifiCfg.pass) + "\n";
  }
  for (int i = 0; i < NT; i++) {
    t += "Q:" + String(keyOf(i)) + "=" + urlEncode(queryOf(i)) + "\n";
    if (i < NS) t += "T:" + String(KEYS[i]) + "=" + String(sens.s[i].alertAbove, 1) + "\n";
  }
  server.sendHeader("Content-Disposition", "attachment; filename=esp-monitor.cfg");
  server.send(200, "text/plain; charset=utf-8", t);
}

static void handleImportForm() {
  if (!webAuth()) return;
  server.send(200, "text/html; charset=utf-8", F(
    "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Импорт настроек</title><body style='font-family:sans-serif;max-width:780px;margin:1em auto;padding:0 1em'>"
    "<h2>Импорт настроек</h2><p>Вставь сюда содержимое файла, скачанного со <a href='/'>страницы настроек</a> "
    "(ссылка «скачать настройки»). Строки с ошибками будут пропущены, остальные применятся сразу и сохранятся в EEPROM. "
    "Wi-Fi применится только если сеть найдена и пароль верный.</p>"
    "<form method=post action=/import><textarea name=cfg rows=24 style='width:100%;box-sizing:border-box;font-family:monospace'></textarea>"
    "<br><br><button style='padding:8px 20px;font-size:1.1em'>Применить</button> &nbsp; <a href='/'>Отмена</a></form>"));
}

static void handleImport() {
  if (!webAuth()) return;
  String body = server.arg("cfg");
  String wifiImpSsid, wifiImpPass;
  bool srvOk = false, dkOk = false, tzOk = false, qOk = false;
  int applied = 0, skipped = 0;
  int pos = 0;
  while (pos <= (int)body.length()) {
    int e = body.indexOf('\n', pos); if (e < 0) e = body.length();
    String line = body.substring(pos, e); pos = e + 1;
    line.trim();
    if (!line.length() || line[0] == '#') continue;
    int eq = line.indexOf('=');
    if (eq < 1) { skipped++; continue; }
    String k = line.substring(0, eq); k.trim();
    String v = urlDecode(line.substring(eq + 1)); v.trim();
    if (k == "SRV") {
      int c = v.indexOf(':');
      String h = (c < 0) ? v : v.substring(0, c);
      long p = (c < 0) ? 9090 : v.substring(c + 1).toInt();
      if (h.length() && h.length() < sizeof(srv.host) && p > 0 && p < 65536) {
        memset(srv.host, 0, sizeof(srv.host)); h.toCharArray(srv.host, sizeof(srv.host));
        srv.port = (uint16_t)p; srvOk = true; applied++;
      } else skipped++;
    } else if (k == "DK") {
      if (v.length() == 0) { memset(dk.host, 0, sizeof(dk.host)); dkOk = true; applied++; }
      else {
        int c = v.indexOf(':');
        String h = (c < 0) ? v : v.substring(0, c);
        long p = (c < 0) ? DEF_DK_PORT : v.substring(c + 1).toInt();
        if (h.length() && h.length() < sizeof(dk.host) && p > 0 && p < 65536) {
          memset(dk.host, 0, sizeof(dk.host)); h.toCharArray(dk.host, sizeof(dk.host));
          dk.port = (uint16_t)p; dkOk = true; applied++;
        } else skipped++;
      }
    } else if (k == "DKALERT") { dk.alert = v.toInt() ? 1 : 0; dkOk = true; applied++; }
    else if (k == "TZ") {
      long m = v.toInt();
      if (m >= -720 && m <= 840) { tzc.minutes = (int16_t)m; tzOk = true; applied++; } else skipped++;
    } else if (k == "WIFI_SSID") {
      if (v.length() && v.length() <= 32) { wifiImpSsid = v; applied++; } else skipped++;
    } else if (k == "WIFI_PASS") {
      if (v.length() <= 64) { wifiImpPass = v; applied++; } else skipped++;
    } else if (k.startsWith("Q:")) {
      int idx = keyIndex(k.substring(2));
      if (idx >= 0 && v.length() < QLEN) { memset(queryOf(idx), 0, QLEN); v.toCharArray(queryOf(idx), QLEN); qOk = true; applied++; }
      else skipped++;
    } else if (k.startsWith("T:")) {
      int idx = keyIndex(k.substring(2));
      if (idx >= 0 && idx < NS) { sens.s[idx].alertAbove = v.toFloat(); qOk = true; applied++; }
      else skipped++;
    } else skipped++;
    service();                                 // длинный файл не должен надолго блокировать опрос
  }
  if (srvOk) saveSrv();
  if (dkOk) { saveDk(); dkLastPoll = 0; dkBackoffUntil = 0; }
  if (tzOk) { saveTz(); sendTime(); }
  if (qOk) { saveSens(); lastCheckTime = 0; backoffUntil = 0; }
  if (wifiImpSsid.length()) startWifiChange(wifiImpSsid, wifiImpPass);
  String p = F("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
               "<title>Импорт</title><body style='font-family:sans-serif;max-width:780px;margin:1em auto;padding:0 1em'>"
               "<h2>Импорт завершён</h2><p>Применено строк: ");
  p += String(applied) + ", пропущено: " + String(skipped) + ".</p>";
  if (wifiImpSsid.length()) p += String("<p>Wi-Fi: переключаюсь на сеть ") + he(wifiImpSsid) + " (подключение займёт до 20 с).</p>";
  p += F("<p><a href='/'>На главную</a></p>");
  server.send(200, "text/html; charset=utf-8", p);
}

// ---------------------------------------------------------------------
//  Prometheus
// ---------------------------------------------------------------------
static String urlEncode(const char* s) {
  String r;
  const char* hex = "0123456789ABCDEF";
  for (; *s; s++) {
    uint8_t c = (uint8_t)*s;
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.' || c == '~') r += (char)c;
    else { r += '%'; r += hex[c >> 4]; r += hex[c & 15]; }
  }
  return r;
}

// возврат: 1 - значение получено, 2 - сервер ответил, но данных нет (или код не 200), 3 - нет связи
static uint8_t queryValue(const char* q, float &out, int &code) {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2000);                     // и на соединение, и на чтение: недоступный сервер не должен надолго блокировать ESP
  String url = String("http://") + srv.host + ":" + String((unsigned)srv.port) + "/api/v1/query?query=" + urlEncode(q);
  if (!http.begin(client, url)) { code = -100; return 3; }
  code = http.GET();
  if (code < 0) { http.end(); return 3; }
  uint8_t st = 2;
  if (code == HTTP_CODE_OK) {
    String payload = http.getString();
    // {"status":"success","data":{"resultType":"vector","result":[{"metric":{},"value":[1700000000.1,"63.4"]}]}}
    int valIdx = payload.indexOf("\"value\":[");
    if (valIdx != -1) {
      int comma = payload.indexOf(',', valIdx);
      int q1 = payload.indexOf('"', comma);
      int q2 = payload.indexOf('"', q1 + 1);
      if (comma != -1 && q1 != -1 && q2 != -1) {
        String v = payload.substring(q1 + 1, q2);
        if (v.indexOf("NaN") < 0 && v.indexOf("Inf") < 0) { out = v.toFloat(); st = 1; }
      }
    }
  }
  http.end();
  return st;
}

static void service() {                      // всё, что должно обслуживаться между долгими операциями
  pollCmds();
  server.handleClient();
  if (svcStarted) ArduinoOTA.handle();
}

static void pollPrometheus() {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("ERR:NOWIFI"); return; }
  static uint8_t cycle = 0;
  const uint8_t last = (cycle++ % X_EVERY == 0) ? NT : NTL;      // доп. метрики - не каждый цикл
  String alerts[NS];
  String bad;
  bool linkDown = false;
  int linkCode = 0;
  for (uint8_t i = 0; i < last; i++) {
    service();
    if (queryOf(i)[0] == 0) { sState[i] = 0; continue; }
    float v = 0; int code = 0;
    uint8_t st = queryValue(queryOf(i), v, code);
    if (st == 3) {                           // разовый сбой связи: одна повторная попытка
      delay(200); service();
      st = queryValue(queryOf(i), v, code);
    }
    sState[i] = st; sCode[i] = code; sMs[i] = millis();
    if (st == 1) {
      if (i >= NS && i < NTL) v = constrain(v, 0.0f, 100.0f);   // нагрузка: 0..100 %
      else if (i == NTL + 1) v = constrain(v, 0.0f, 100.0f);    // XDSKP: свободно, %
      sVal[i] = v;
      Serial.print(keyOf(i)); Serial.print(':'); Serial.println(String(v, 1));
      if (i < NS && v > sens.s[i].alertAbove) alerts[i] = String(KEYS[i]) + "_OVERHEAT:" + String(v, 1);
    } else if (st == 3) {
      linkDown = true;                       // сервер недоступен - остальные не мучаем
      linkCode = code;
      lastLinkCode = code;
      backoffUntil = millis() + BACKOFF_MS;
      break;
    } else {
      if (bad.length()) bad += ',';
      bad += keyOf(i);
    }
  }
  if (linkDown) Serial.println("ERR:PROM:" + String(linkCode));   // код HTTPClient: -1 отказ, -4 нет связи, -5 обрыв, -11 таймаут
  else if (bad.length()) Serial.println("ERR:QUERY:" + bad);
  for (uint8_t i = 0; i < NS; i++) if (alerts[i].length()) Serial.println("ALERT:" + alerts[i]);   // тревоги - после значений
  // тревога по Docker повторяется в каждом цикле (как температурная), пока контейнеры лежат и данные свежие
  if (dk.alert && dkAlertText.length() && dkOkMs && millis() - dkOkMs < 45000) Serial.println("ALERT:" + dkAlertText);
}

// ---------------------------------------------------------------------
//  docker-status: ответ сервиса -> компактные строки для дисплея
//    SUM total=14 ok=12 warn=0 bad=2
//    O|259200|0|0|имя        (код|секунд|рестартов|exit|имя)
// ---------------------------------------------------------------------
#define DK_FWD_MAX 24                        // столько контейнеров максимум уходит на дисплей

static bool dkLineOk(const String &l) {
  if (l.length() < 9 || l.length() > 70) return false;
  if (!strchr("OSPURX", l[0]) || l[1] != '|') return false;
  int bars = 0;
  for (unsigned i = 0; i < l.length(); i++) {
    uint8_t c = (uint8_t)l[i];
    if (c < 32 || c > 126) return false;
    if (c == '|') bars++;
  }
  return bars == 4;
}

static int sumField(const String &l, const char* key) {      // "total=14" -> 14
  int i = l.indexOf(key);
  return i < 0 ? -1 : l.substring(i + strlen(key)).toInt();
}

static void pollDocker() {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2500);
  String url = String("http://") + dk.host + ":" + String((unsigned)dk.port) + "/";
  if (!http.begin(client, url)) { dkState = 3; dkErrCode = -100; dkBackoffUntil = millis() + DK_BACKOFF_MS; Serial.println("ERR:DOCKER:-100"); return; }
  int code = http.GET();
  if (code != HTTP_CODE_OK || http.getSize() > 8000) {
    http.end();
    dkState = 3; dkErrCode = (code == HTTP_CODE_OK) ? -101 : code;
    dkAlertText = "";
    if (code < 0) dkBackoffUntil = millis() + DK_BACKOFF_MS;
    Serial.println("ERR:DOCKER:" + String(dkErrCode));
    return;
  }
  String body = http.getString();
  http.end();
  service();
  int pos = 0, n = 0, bads = 0;
  String firstBad, badWord;
  bool gotSum = false;
  int t = 0, o = 0, w = 0, b = 0;
  String out[DK_FWD_MAX];
  while (pos < (int)body.length()) {
    int e = body.indexOf('\n', pos); if (e < 0) e = body.length();
    String l = body.substring(pos, e); pos = e + 1;
    l.trim();
    if (l.startsWith("SUM ")) {
      t = sumField(l, "total="); o = sumField(l, "ok="); w = sumField(l, "warn="); b = sumField(l, "bad=");
      gotSum = (t >= 0 && o >= 0 && w >= 0 && b >= 0);
    } else if (dkLineOk(l)) {
      if (n < DK_FWD_MAX) out[n++] = l;
      if (l[0] == 'U' || l[0] == 'R' || l[0] == 'X') {
        if (bads == 0) {
          firstBad = l.substring(l.lastIndexOf('|') + 1);
          badWord = (l[0] == 'X') ? "DOWN" : (l[0] == 'U' ? "UNHEALTHY" : "RESTARTING");
        }
        bads++;
      }
    }
  }
  if (!gotSum) { dkState = 3; dkErrCode = -102; Serial.println("ERR:DOCKER:-102"); return; }   // не тот ответ
  dkState = 1; dkErrCode = HTTP_CODE_OK; dkOkMs = millis();
  dkTotal = t; dkOkN = o; dkWarnN = w; dkBadN = b; dkLines = n;
  Serial.printf("DK:%d;%d;%d;%d\n", t, o, w, b);
  for (int i = 0; i < n; i++) {
    Serial.print("DKI:"); Serial.println(out[i]);
    delay(3); service();                     // не заваливать приёмный буфер дисплея
  }
  Serial.printf("DKEND:%d\n", n);
  if (b > 0 && firstBad.length()) {
    dkAlertText = badWord + " " + firstBad;
    if (b > 1) dkAlertText += " +" + String(b - 1);
  } else {
    dkAlertText = "";
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  WiFi.persistent(false);                    // не писать настройки Wi-Fi во flash на каждом begin()
  WiFi.setAutoReconnect(true);
  WiFi.hostname(HOSTNAME);
  cfgLoad();
  WiFi.mode(WIFI_STA);
  if (wifiCfg.magic == WIFI_MAGIC && wifiCfg.ssid[0]) WiFi.begin(wifiCfg.ssid, wifiCfg.pass);
  else WiFi.begin(DEF_SSID, DEF_PASS);
  // ждать подключения здесь нельзя: при неверной сети дисплей не смог бы её сменить

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/export", HTTP_GET, handleExport);
  server.on("/import", HTTP_GET, handleImportForm);
  server.on("/import", HTTP_POST, handleImport);
  httpUpdater.setup(&server, "/update", WEB_USER, WEB_PASS);
  server.begin();
}

void loop() {
  service();
  netTick();
  if (millis() - lastCheckTime > checkDelay) {
    lastCheckTime = millis();
    if (wifiPending) {
      // ждём результата смены сети
    } else if ((long)(millis() - backoffUntil) < 0) {
      Serial.println("ERR:PROM:" + String(lastLinkCode));      // пауза после сбоя: опрос не делаем, но ошибку напоминаем дисплею
    } else {
      pollPrometheus();
    }
  }
  if (ntpStarted && millis() - timeSentMs > 10000 && timeValid()) sendTime();     // часы для дисплея
  if (dk.host[0] && !wifiPending && WiFi.status() == WL_CONNECTED && millis() - dkLastPoll > DK_PERIOD_MS) {
    dkLastPoll = millis();
    if ((long)(millis() - dkBackoffUntil) < 0) Serial.println("ERR:DOCKER:" + String(dkErrCode));
    else pollDocker();
  }
}