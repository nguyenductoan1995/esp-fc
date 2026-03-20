#include "StatusLed.hpp"
#include "Target/Target.h"
#include <Arduino.h>

#ifdef ESPFC_LED_WS2812
#include "driver/i2s.h"

// https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/api-reference/peripherals/i2s.html
// https://github.com/vunam/esp32-i2s-ws2812/blob/master/ws2812.c

static constexpr size_t LED_NUMBER = 1;
static constexpr size_t PIXEL_SIZE = 12; // each colour takes 4 bytes in buffer
static constexpr size_t ZERO_BUFFER = 32;
static constexpr size_t SIZE_BUFFER = LED_NUMBER * PIXEL_SIZE + ZERO_BUFFER;
static constexpr uint32_t SAMPLE_RATE = 93750;
static constexpr i2s_port_t I2S_NUM = I2S_NUM_0;

typedef struct {
  uint8_t g;
  uint8_t r;
  uint8_t b;
} ws2812_pixel_t;

static i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_MSB,
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = SIZE_BUFFER / 2,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0,
  .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
  .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
};

static i2s_pin_config_t pin_config = {
  .bck_io_num = -1,
  .ws_io_num = -1,
  .data_out_num = -1,
  .data_in_num = -1
};

static uint8_t out_buffer[SIZE_BUFFER] = {0};

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

static void ws2812_init(int8_t pin)
{
  pin_config.data_out_num = pin;
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  std::fill_n(out_buffer, SIZE_BUFFER, 0);
}

static void ws2812_write_pixel(uint8_t * buffer, const ws2812_pixel_t& pixel)
{
  *buffer++ = bitpatterns[pixel.g >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.r >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.b >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 0 & 0x03];
}

static void ws2812_update(const ws2812_pixel_t * pixels)
{
  size_t bytes_written = 0;
  for (size_t i = 0; i < LED_NUMBER; i++)
  {
    size_t loc = i * PIXEL_SIZE;
    ws2812_write_pixel(out_buffer + loc, pixels[i]);
  }
  i2s_zero_dma_buffer(I2S_NUM);
  i2s_write(I2S_NUM, out_buffer, SIZE_BUFFER, &bytes_written, portMAX_DELAY);
}

static const ws2812_pixel_t PIXEL_OFF[] = {{0, 0, 0}};

#endif // ESPFC_LED_WS2812

namespace Espfc::Connect
{

/**
 * Pattern nhấp nháy: mảng thời gian ON/OFF xen kẽ (ms), kết thúc bằng 0.
 * Chỉ số chẵn (0, 2, ...) = thời gian LED BẬT (ON)
 * Chỉ số lẻ (1, 3, ...) = thời gian LED TẮT (OFF)
 * Giá trị 0 = kết thúc mảng, bắt đầu lại từ đầu.
 */
static int LED_OFF_PATTERN[]    = {0};                                  ///< LED tắt hoàn toàn
static int LED_SOLID_PATTERN[]  = {100, 0};                             ///< Sáng liên tục (solid on)
static int LED_SLOW_BLINK[]     = {200, 800, 0};                        ///< 1Hz, nhấp nháy chậm
static int LED_FAST_BLINK[]     = {150, 150, 0};                        ///< 3Hz, nhấp nháy nhanh
static int LED_DOUBLE_BLINK[]   = {120, 120, 120, 640, 0};              ///< Double flash ~1Hz
static int LED_RAPID_BLINK[]    = {80, 80, 0};                          ///< 6Hz, nhấp nháy rất nhanh
static int LED_TRIPLE_BLINK[]   = {100, 100, 100, 100, 100, 1500, 0};   ///< 3 nhịp nhanh + nghỉ dài

StatusLed::StatusLed() : _pin(-1), _invert(0), _status(LED_OFF), _next(0), _state(LOW), _step(0), _pattern(LED_OFF_PATTERN), _r(0), _g(0), _b(0) {}

void StatusLed::begin(int8_t pin, uint8_t type, uint8_t invert)
{
  if(pin == -1) return;

  _pin = pin;
  _type = type;
  _invert = invert;

#ifdef ESPFC_LED_WS2812
  if(_type == LED_STRIP) ws2812_init(_pin);
  if(_type == LED_SIMPLE) pinMode(_pin, OUTPUT);
#else
  pinMode(_pin, OUTPUT);
#endif
  setStatus(LED_ON, true);
}

void StatusLed::setStatus(LedStatus newStatus, bool force)
{
  if(_pin == -1) return;
  if(!force && newStatus == _status) return;

  _status = newStatus;
  _state = LOW;
  _step = 0;
  _next = millis();

  switch (_status)
  {
    case LED_OK:
      _pattern = LED_SLOW_BLINK;
      _r = 0x00; _g = 0x00; _b = 0x30; // blue
      break;
    case LED_ERROR:
      _pattern = LED_TRIPLE_BLINK;
      _r = 0x60; _g = 0x00; _b = 0x00; // red
      break;
    case LED_ARMED:
      _pattern = LED_SOLID_PATTERN;
      _state = HIGH;
      _r = 0x00; _g = 0x40; _b = 0x00; // green
      break;
    case LED_ALTHOLD:
      _pattern = LED_FAST_BLINK;
      _r = 0x00; _g = 0x30; _b = 0x30; // cyan
      break;
    case LED_GPS:
      _pattern = LED_DOUBLE_BLINK;
      _r = 0x20; _g = 0x00; _b = 0x40; // purple
      break;
    case LED_FAILSAFE:
      _pattern = LED_RAPID_BLINK;
      _r = 0x80; _g = 0x00; _b = 0x00; // red bright
      break;
    case LED_CALIBRATING:
      _pattern = LED_SOLID_PATTERN;
      _state = HIGH;
      _r = 0x40; _g = 0x40; _b = 0x00; // yellow
      break;
    case LED_LOW_BATTERY:
      _pattern = LED_RAPID_BLINK;
      _r = 0x60; _g = 0x20; _b = 0x00; // orange
      break;
    case LED_ON:
      _pattern = LED_SOLID_PATTERN;
      _state = HIGH;
      _r = 0x20; _g = 0x20; _b = 0x40; // blue-white
      break;
    case LED_OFF:
    default:
      _pattern = LED_OFF_PATTERN;
      _r = 0x00; _g = 0x00; _b = 0x00;
      break;
  }
  _write(_state);
}

void StatusLed::update()
{
  if(_pin == -1 || !_pattern) return;

  uint32_t now = millis();

  if(now < _next) return;

  if (!_pattern[_step])
  {
    _step = 0;
    _next = now + 20;
    return;
  }

  _state = !(_step & 1);
  _write(_state);

  _next = now + _pattern[_step];
  _step++;
}

void StatusLed::_write(uint8_t val)
{
#ifdef ESPFC_LED_WS2812
  if(_type == LED_STRIP)
  {
    // Build pixel from current color (stored as RGB, WS2812 is GRB)
    ws2812_pixel_t pixel = {_g, _r, _b};
    ws2812_update(val ? &pixel : PIXEL_OFF);
  }
  if(_type == LED_SIMPLE) digitalWrite(_pin, val ^ _invert);
#else
  digitalWrite(_pin, val ^ _invert);
#endif
}

}
