/*
  Эмулятор шасси LOOI для ESP32-S3 (BLE-периферия).
  Профиль восстановлен из официального приложения LOOI 2.9.2.

  СБОРКА (Arduino IDE 2.x):
    1. Менеджер плат: "esp32" от Espressif (3.x).
    2. Менеджер библиотек: "NimBLE-Arduino" от h2zero, версия 2.x (например 2.3.x).
    3. Tools -> Board: "ESP32S3 Dev Module"
       Tools -> USB CDC On Boot: "Enabled"      (иначе Serial не появится в USB)
    4. Загрузить, открыть Serial Monitor: 115200, конец строки "Newline".

  КОМАНДЫ В МОНИТОРЕ ПОРТА:
    help | status
    pair            - следующий пресет режима сопряжения (по кругу)
    unpair          - обычная реклама
    adv name=ИМЯ mfg=ID:HEX      - своя реклама, напр.:  adv mfg=ffff:0000000001
    batt N | charge 0/1 | stick 0/1 | touchl 0/1 | touchr 0/1
    cliff 0/1 | click | dbl | long | tof MM
*/
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>
#include <vector>
#include <sstream>

#define U16(h) "0000" #h "-0000-1000-8000-00805f9b34fb"
static const char *SVC_DP  = "000000FF-0000-1000-8000-00805f9b34fb";

// Приложение фильтрует скан по ТОЧНОМУ имени: "LOOI Robot" или "MEOW_MEOW_BOT".
static std::string g_name = "LOOI Robot";
// Manufacturer data целиком: ID компании (little-endian) + данные. Пусто = обычный режим.
static std::vector<uint8_t> g_mfg;

// "Режим сопряжения": по crash.log приложение читает manufacturer data[4];
// пресет 1 (байт 4 == 1) вызвал диалог "Permit LOOI-xxxxxx to connect".
static const std::vector<std::vector<uint8_t>> PAIR_PRESETS = {
  {0xFF, 0xFF, 0, 0, 0, 0, 1},
  {0xFF, 0xFF, 0, 0, 0, 0, 2},
  {0xFF, 0xFF, 0, 0, 0, 0, 0},
  {0xFF, 0xFF, 0, 0, 0, 0, 1, 0, 0, 0},
};
static size_t g_pairIdx = 0;

// Состояние "железа"
static uint8_t st_cliff[4] = {1, 1, 1, 1};   // 0 == обрыв в этом углу
static uint8_t st_evade[2] = {0, 0};
static uint8_t st_batt = 100, st_charge = 0, st_stick = 1;

static NimBLECharacteristic *cMove, *cClamp, *cLight, *cConfig, *cSeq;
static NimBLECharacteristic *cCliff, *cEvade, *cPower, *cStick, *cNotify, *cLog;

static volatile uint32_t bootAt = 0, seqAt = 0;
static volatile uint8_t seqByte = 0;

static void hexdump(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02x ", d[i]);
}

static void refreshReads() {
  cCliff->setValue(st_cliff, 4);
  cEvade->setValue(st_evade, 2);
  uint8_t p[2] = {st_batt, st_charge};
  cPower->setValue(p, 2);
  uint8_t s[1] = {st_stick};
  cStick->setValue(s, 1);
}

// notify на FED9: первый байт - тип события
static void dpNotify(const uint8_t *d, size_t n) {
  cNotify->setValue(d, n);
  bool ok = cNotify->notify();
  Serial.print("NOTIFY ");
  hexdump(d, n);
  Serial.println(ok ? "ok" : "(нет подписчика)");
}

static void applyAdv() {
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->stop();
  NimBLEAdvertisementData ad;
  ad.setFlags(0x06);
  ad.setName(g_name);
  if (!g_mfg.empty()) ad.setManufacturerData(g_mfg);
  size_t used = 3 + (2 + g_name.size()) + (g_mfg.empty() ? 0 : 2 + g_mfg.size());
  if (used + 4 <= 31) ad.addServiceUUID(NimBLEUUID((uint16_t)0x180A));
  adv->setAdvertisementData(ad);
  adv->start();
  Serial.printf("ADV name='%s' mfg=", g_name.c_str());
  if (g_mfg.empty()) Serial.print("-");
  else hexdump(g_mfg.data(), g_mfg.size());
  Serial.println();
}

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    Serial.printf("CONNECTED %s\n", info.getAddress().toString().c_str());
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    Serial.printf("DISCONNECTED reason=%d\n", reason);
    NimBLEDevice::getAdvertising()->start();
  }
};

class CharCb : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    Serial.printf("READ  %s\n", c->getUUID().toString().c_str());
  }
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &info, uint16_t sub) override {
    Serial.printf("SUBSCRIBE %s value=%u\n", c->getUUID().toString().c_str(), sub);
  }
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t *d = v.data();
    size_t n = v.size();
    if (c == cMove && n >= 2) {
      Serial.printf("MOVE  forward=%.2f veer=%.2f\n", (int8_t)d[0] / 127.0, (int8_t)d[1] / 127.0);
    } else if (c == cClamp) {
      if (n == 1) Serial.printf("HEAD  angle=%.2f\n", 1.0 - d[0] / 255.0);
      else { Serial.print("HEAD  speed "); hexdump(d, n); Serial.println(); }
    } else if (c == cLight) {
      Serial.printf("LIGHT %s\n", (n && d[0]) ? "on" : "off");
    } else if (c == cConfig) {
      Serial.print("CONFIG "); hexdump(d, n);
      Serial.println(" (01/03 - рукопожатие, 3/4 - беспроводная зарядка)");
      if (n && d[0] == 3) bootAt = millis() + 100;      // после рукопожатия робот шлёт 11 01 00
    } else if (c == cSeq) {
      Serial.print("SEQ   "); hexdump(d, n); Serial.println();
      if (n) { seqByte = d[0]; seqAt = millis() + 300; } // байт 0 = SEQ, ждём паузу и шлём [7, seq]
    } else if (c == cLog) {
      Serial.printf("LOGW  %.*s\n", (int)n, (const char *)d);
    } else {
      Serial.printf("WRITE %s ", c->getUUID().toString().c_str());
      hexdump(d, n); Serial.println();
    }
  }
};

static CharCb g_charCb;

static NimBLECharacteristic *mk(NimBLEService *s, const char *uuid, uint32_t props) {
  NimBLECharacteristic *c = s->createCharacteristic(uuid, props);
  c->setCallbacks(&g_charCb);
  return c;
}

static std::vector<uint8_t> fromHex(const std::string &h) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < h.size(); i += 2)
    out.push_back((uint8_t)strtol(h.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

static void handleCmd(const std::string &line) {
  std::istringstream is(line);
  std::string c;
  is >> c;
  if (c.empty()) return;
  std::vector<std::string> a;
  for (std::string t; is >> t;) a.push_back(t);
  int v = a.empty() ? 0 : atoi(a[0].c_str());

  if (c == "batt")      { st_batt = v; refreshReads(); uint8_t b[2] = {8, (uint8_t)v}; dpNotify(b, 2); }
  else if (c == "charge") { st_charge = v; refreshReads(); uint8_t b[2] = {11, (uint8_t)v}; dpNotify(b, 2); }
  else if (c == "stick")  { st_stick = v; refreshReads(); uint8_t b[1] = {(uint8_t)(v ? 5 : 6)}; dpNotify(b, 1); }
  else if (c == "touchl") { uint8_t b[2] = {9, (uint8_t)v}; dpNotify(b, 2); }
  else if (c == "touchr") { uint8_t b[2] = {10, (uint8_t)v}; dpNotify(b, 2); }
  else if (c == "cliff")  { uint8_t x = v ? 0 : 1; for (int i = 0; i < 4; i++) st_cliff[i] = x;
                            refreshReads(); uint8_t b[5] = {1, x, x, x, x}; dpNotify(b, 5); }
  else if (c == "click" || c == "dbl" || c == "long") {
    uint8_t b[3] = {18, 0, (uint8_t)(c == "click" ? 0 : c == "dbl" ? 1 : 2)}; dpNotify(b, 3); }
  else if (c == "tof")    { uint8_t b[3] = {14, (uint8_t)(v & 255), (uint8_t)(v >> 8)}; dpNotify(b, 3); } // формат - догадка
  else if (c == "pair")   { g_mfg = PAIR_PRESETS[g_pairIdx % PAIR_PRESETS.size()];
                            Serial.printf("PAIR preset #%u\n", (unsigned)(g_pairIdx % PAIR_PRESETS.size() + 1));
                            g_pairIdx++; applyAdv(); }
  else if (c == "unpair") { g_mfg.clear(); applyAdv(); }
  else if (c == "adv") {
    for (auto &t : a) {
      size_t eq = t.find('=');
      if (eq == std::string::npos) continue;
      std::string k = t.substr(0, eq), val = t.substr(eq + 1);
      if (k == "name") g_name = val;
      else if (k == "mfg") {
        size_t col = val.find(':');
        uint16_t id = (uint16_t)strtol(val.substr(0, col).c_str(), nullptr, 16);
        g_mfg = {(uint8_t)(id & 255), (uint8_t)(id >> 8)};
        if (col != std::string::npos) { auto p = fromHex(val.substr(col + 1)); g_mfg.insert(g_mfg.end(), p.begin(), p.end()); }
      }
    }
    applyAdv();
  }
  else if (c == "status") {
    Serial.printf("batt=%u charge=%u stick=%u name='%s'\n", st_batt, st_charge, st_stick, g_name.c_str());
  }
  else Serial.println("команды: pair unpair adv batt charge stick touchl touchr cliff click dbl long tof status");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("LOOI emulator (ESP32-S3) starting");

  NimBLEDevice::init(g_name);
  NimBLEDevice::setMTU(517);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCb());

  // Device Information (ровно один, дубля BlueZ тут нет)
  NimBLEService *dis = server->createService(U16(180A));
  NimBLECharacteristic *sn = mk(dis, U16(2A29), NIMBLE_PROPERTY::READ);
  const uint8_t snv[] = {'L','O','O','I','0','0','0','0','0','0','0','1', 0};   // строка до первого 0x00
  sn->setValue(snv, sizeof(snv));
  const uint8_t fw[] = {2, 0, 0}, hw[] = {1, 0, 0};                             // нужно >= 3 байт
  mk(dis, U16(2A27), NIMBLE_PROPERTY::READ)->setValue(fw, 3);
  mk(dis, U16(2A28), NIMBLE_PROPERTY::READ)->setValue(hw, 3);

  // Основной сервис 000000FF
  NimBLEService *dp = server->createService(SVC_DP);
  const uint32_t W = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
  cMove   = mk(dp, U16(FED0), W);
  cClamp  = mk(dp, U16(FED1), W);
  cLight  = mk(dp, U16(FED2), W);
  cConfig = mk(dp, U16(FEDA), W);
  cSeq    = mk(dp, U16(FE00), W);
  cCliff  = mk(dp, U16(FED5), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  cEvade  = mk(dp, U16(FED6), NIMBLE_PROPERTY::READ);
  cPower  = mk(dp, U16(FED8), NIMBLE_PROPERTY::READ);
  cStick  = mk(dp, U16(FED4), NIMBLE_PROPERTY::READ);
  cNotify = mk(dp, U16(FED9), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  cLog    = mk(dp, U16(FEF0), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | W);
  mk(dp, U16(FF02), W);                       // "boost", неподтверждённая
  uint8_t zero = 0;
  cNotify->setValue(&zero, 1);
  cLog->setValue(&zero, 1);
  refreshReads();

  // OTA-сервис: заглушка, чтобы профиль совпадал
  NimBLEService *ota = server->createService(U16(8018));
  mk(ota, U16(8020), W | NIMBLE_PROPERTY::INDICATE);
  mk(ota, U16(8021), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  mk(ota, U16(8022), W | NIMBLE_PROPERTY::INDICATE);
  mk(ota, U16(8023), NIMBLE_PROPERTY::READ | W);

  dis->start();
  dp->start();
  ota->start();

  applyAdv();
  Serial.printf("Address: %s\n", NimBLEDevice::getAddress().toString().c_str());
  Serial.println("Готово. Введите 'pair' для режима сопряжения, 'help' - список команд.");
}

void loop() {
  static std::string buf;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCmd(buf); buf.clear(); }
    else buf.push_back((char)ch);
  }
  uint32_t now = millis();
  if (bootAt && now >= bootAt) {
    bootAt = 0;
    const uint8_t b[3] = {0x11, 0x01, 0x00};
    dpNotify(b, 3);
  }
  if (seqAt && now >= seqAt) {
    seqAt = 0;
    const uint8_t b[2] = {7, seqByte};
    dpNotify(b, 2);
  }
  delay(5);
}
