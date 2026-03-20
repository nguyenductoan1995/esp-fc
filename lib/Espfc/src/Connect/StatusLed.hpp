#pragma once
#include <cstdint>
#include <cstddef>

namespace Espfc::Connect
{

enum LedType
{
  LED_SIMPLE,
  LED_STRIP,
};

enum LedStatus
{
  LED_OFF,
  LED_OK,           // disarmed, ready to arm      — blue slow blink
  LED_ERROR,        // arming disabled              — red triple blink
  LED_ON,           // legacy / init                — blue-white solid
  LED_ARMED,        // armed, flying normally       — green solid
  LED_ALTHOLD,      // altitude hold active         — cyan fast blink
  LED_GPS,          // GPS position/rescue active   — purple double blink
  LED_FAILSAFE,     // failsafe triggered           — red rapid blink
  LED_CALIBRATING,  // gyro/sensor calibrating      — yellow solid
  LED_LOW_BATTERY,  // low battery warning          — orange rapid blink
};

class StatusLed
{

public:
  StatusLed();
  void begin(int8_t pin, uint8_t type, uint8_t invert);
  void update();
  void setStatus(LedStatus newStatus, bool force = false);

private:
  void _write(uint8_t val);
  int8_t _pin;
  uint8_t _type;
  uint8_t _invert;
  LedStatus _status;
  uint32_t _next;
  bool _state;
  size_t _step;
  int * _pattern;
  uint8_t _r, _g, _b;  // active color for WS2812 (GRB order stored as RGB for readability)
};

}
