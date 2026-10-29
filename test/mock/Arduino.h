// Minimal stand-in for the Arduino-ESP32 core, so the real firmware sources (line_follower_robot/*.cpp) compile and run on a PC.
// Only the calls the firmware makes are provided. Time, pins, PWM and the serial port are fully controlled by the test harness.
#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
int analogRead(uint8_t pin);
uint32_t millis();
uint32_t micros();
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(uint8_t pin, uint32_t duty);

class MockSerial {
 public:
  void setTxBufferSize(size_t) {}
  void begin(unsigned long) {}
  int availableForWrite();
  size_t write(const uint8_t* data, size_t len);
  size_t print(const char* s);
  size_t println(const char* s = "");
  int available();
  int read();
};
extern MockSerial Serial;

// ---- test-harness controls (not part of the real Arduino API) ----------------------------------------------------------------------------------
namespace mock {
const int PIN_COUNT = 48;
extern uint64_t timeUs;                 // simulated time
extern uint8_t pinLevel[PIN_COUNT];     // inputs: set by the harness; outputs: written by the firmware
extern uint32_t pwmDuty[PIN_COUNT];     // last ledcWrite() duty per pin
extern uint32_t pwmFreq[PIN_COUNT];
extern int resetReason;                 // value returned by esp_reset_reason()
extern std::string serialOut;           // everything the firmware printed
extern std::string serialIn;            // characters waiting for Serial.read()
extern bool echoSerial;                 // also print firmware output to stdout
void reset();                           // power-cycle the simulated board (pins, PWM, time, serial)
void advanceUs(uint32_t us);
}

#endif
