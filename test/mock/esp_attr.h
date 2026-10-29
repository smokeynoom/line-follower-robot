#ifndef MOCK_ESP_ATTR_H
#define MOCK_ESP_ATTR_H

// On the ESP32 this places a variable in RTC memory that survives resets. On the PC a plain global does the same job for the tests:
// it keeps its value when the test calls robotSetup() again to simulate a reset.
#define RTC_NOINIT_ATTR

#endif
