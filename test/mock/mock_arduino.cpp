#include "Arduino.h"
#include "esp_system.h"

MockSerial Serial;

namespace mock {
uint64_t timeUs = 0;
uint8_t pinLevel[PIN_COUNT];
uint32_t pwmDuty[PIN_COUNT];
uint32_t pwmFreq[PIN_COUNT];
int resetReason = ESP_RST_POWERON;
std::string serialOut;
std::string serialIn;
bool echoSerial = false;

void reset() {
  timeUs = 0;
  for (int i = 0; i < PIN_COUNT; i++) {
    pinLevel[i] = LOW;
    pwmDuty[i] = 0;
    pwmFreq[i] = 0;
  }
  pinLevel[0] = HIGH;                   // BOOT button is pulled up: not pressed
  resetReason = ESP_RST_POWERON;
  serialOut.clear();
  serialIn.clear();
}
void advanceUs(uint32_t us) { timeUs += us; }
}  // namespace mock

void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t pin) { return pin < mock::PIN_COUNT ? mock::pinLevel[pin] : LOW; }
void digitalWrite(uint8_t pin, uint8_t value) { if (pin < mock::PIN_COUNT) mock::pinLevel[pin] = value ? HIGH : LOW; }
int analogRead(uint8_t) { return 0; }
uint32_t millis() { return (uint32_t)(mock::timeUs / 1000ULL); }
uint32_t micros() { return (uint32_t)mock::timeUs; }

bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t) {
  if (pin >= mock::PIN_COUNT) return false;
  mock::pwmFreq[pin] = freq;
  return true;
}
bool ledcWrite(uint8_t pin, uint32_t duty) {
  if (pin >= mock::PIN_COUNT) return false;
  mock::pwmDuty[pin] = duty;
  return true;
}

esp_reset_reason_t esp_reset_reason(void) { return (esp_reset_reason_t)mock::resetReason; }

int MockSerial::availableForWrite() { return 4096; }
size_t MockSerial::write(const uint8_t* data, size_t len) {
  mock::serialOut.append((const char*)data, len);
  if (mock::echoSerial) fwrite(data, 1, len, stdout);
  return len;
}
size_t MockSerial::print(const char* s) { return write((const uint8_t*)s, strlen(s)); }
size_t MockSerial::println(const char* s) {
  size_t n = print(s);
  return n + write((const uint8_t*)"\n", 1);
}
int MockSerial::available() { return (int)mock::serialIn.size(); }
int MockSerial::read() {
  if (mock::serialIn.empty()) return -1;
  const int c = (unsigned char)mock::serialIn[0];
  mock::serialIn.erase(0, 1);
  return c;
}
