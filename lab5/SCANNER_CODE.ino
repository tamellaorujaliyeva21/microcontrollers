#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define UART1_RX_PIN 18
#define UART1_TX_PIN 17
#define SOF_BYTE     0xA5
#define TYPE_RSSI    0x01

struct Reading { int8_t rssi; int8_t tx; };
QueueHandle_t q;
BLEScan *scan;

uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Frame: 0xA5 | LEN(3) | TYPE | RSSI | TX_DBM | SEQ | CRC8 (over LEN..SEQ)
void sendFrame(int8_t rssi, int8_t txdbm, uint8_t seq) {
  uint8_t f[7];
  f[0] = SOF_BYTE;
  f[1] = 3;
  f[2] = TYPE_RSSI;
  f[3] = (uint8_t)rssi;
  f[4] = (uint8_t)txdbm;
  f[5] = seq;
  f[6] = crc8(&f[1], 5);
  Serial1.write(f, sizeof(f));
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveManufacturerData()) return;
    auto md = dev.getManufacturerData();
    if (md.length() != 3) return;
    if ((uint8_t)md[0] != 0xFF || (uint8_t)md[1] != 0xFF) return;
    Reading r;
    r.rssi = (int8_t)dev.getRSSI();
    r.tx   = (int8_t)((uint8_t)md[2] - 64);   // undo the +64 offset
    xQueueSend(q, &r, 0);
  }
};

void senderTask(void *) {
  uint8_t seq = 0;
  Reading r;
  for (;;) {
    if (xQueueReceive(q, &r, portMAX_DELAY) == pdTRUE) {
      sendFrame(r.rssi, r.tx, seq++);
      Serial.printf("rssi=%d dBm  tx=%d dBm  seq=%u\n", r.rssi, r.tx, (unsigned)(uint8_t)(seq - 1));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

  q = xQueueCreate(128, sizeof(Reading));
  xTaskCreate(senderTask, "sender", 4096, nullptr, 1, nullptr);

  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new CB(), true);  // true = report every packet
  scan->setActiveScan(false);                          // passive: only RSSI needed
  scan->setInterval(100);
  scan->setWindow(99);
}

void loop() {
  scan->start(1, false);   // 1 s scan window
  scan->clearResults();
}
