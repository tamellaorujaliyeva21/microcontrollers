#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

#if ESP_ARDUINO_VERSION_MAJOR >= 3
typedef String bytes_t;
#else
typedef std::string bytes_t;
#endif

struct Level { esp_power_level_t lvl; int8_t dbm; };
const Level LEVELS[] = {
  {ESP_PWR_LVL_N12, -12}, {ESP_PWR_LVL_N9, -9}, {ESP_PWR_LVL_N6, -6},
  {ESP_PWR_LVL_N3,   -3}, {ESP_PWR_LVL_N0,  0}, {ESP_PWR_LVL_P3,  3},
  {ESP_PWR_LVL_P6,    6}, {ESP_PWR_LVL_P9,  9}
};
const int N_LEVELS = sizeof(LEVELS) / sizeof(LEVELS[0]);

BLEAdvertising *adv;
int idx = 4;  // start at 0 dBm

void applyLevel() {
  adv->stop();
  BLEDevice::setPower(LEVELS[idx].lvl, ESP_BLE_PWR_TYPE_DEFAULT);
  BLEDevice::setPower(LEVELS[idx].lvl, ESP_BLE_PWR_TYPE_ADV);

  BLEAdvertisementData ad;
  ad.setFlags(0x06);
  ad.setName("LAB5A");

  // Manufacturer data: company id 0xFFFF, then (tx_dbm + 64) so the byte is never 0
  bytes_t md;
  md += (char)0xFF;
  md += (char)0xFF;
  md += (char)(uint8_t)(LEVELS[idx].dbm + 64);
  ad.setManufacturerData(md);

  adv->setAdvertisementData(ad);
  adv->setScanResponse(false);
  adv->setMinInterval(0x20);   // 20 ms
  adv->setMaxInterval(0x20);
  adv->start();

  Serial.printf("TX power level %d = %d dBm (nominal)\n", idx, LEVELS[idx].dbm);
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("LAB5A");
  adv = BLEDevice::getAdvertising();
  applyLevel();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n' && idx < N_LEVELS - 1) idx++;
    else if (c == 'p' && idx > 0) idx--;
    else if (c >= '0' && c < '0' + N_LEVELS) idx = c - '0';
    else return;
    applyLevel();
  }
  delay(10);
}
