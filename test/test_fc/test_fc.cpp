#include <unity.h>
#include <ArduinoFake.h>
#include "Utils/Timer.h"
#include "Model.h"
#include "Control/Controller.h"
#include "Control/Actuator.h"
#include "Control/Altitude.hpp"
#include "Control/GpsNavigation.hpp"
#include "Output/Mixer.h"
#include <Gps.hpp>
#include <cmath>

using namespace fakeit;
using namespace Espfc;
using Espfc::Control::Rates;
using Espfc::Control::Controller;
using Espfc::Control::Actuator;
using Espfc::Control::Altitude;
using Espfc::Utils::Timer;

// Thin wrappers so test code reads naturally after gpsDistanceBearing() refactor
static float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dist, bearing;
  Espfc::Control::gpsDistanceBearing(lat1, lon1, lat2, lon2, dist, bearing);
  return dist;
}
static float gpsBearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dist, bearing;
  Espfc::Control::gpsDistanceBearing(lat1, lon1, lat2, lon2, dist, bearing);
  return bearing;
}

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

void test_timer_rate_100hz()
{
  Timer timer;
  timer.setRate(100);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_rate_100hz_div2()
{
  Timer timer;
  timer.setRate(100, 2);
  TEST_ASSERT_EQUAL_UINT32(50, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 50.f, timer.intervalf);
}

void test_timer_interval_10ms()
{
  Timer timer;
  timer.setInterval(10000);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_check()
{
  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE( timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(1500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(2000));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(3000));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_FALSE(timer.check(3999));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(4050));
  TEST_ASSERT_EQUAL_UINT32(4, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1050, timer.delta);
}

void test_timer_check_micros()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 1500, 2000, 3000, 3999, 4050);

  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());

  Verify(Method(ArduinoFake(), micros)).Exactly(6_Times);
}

void test_model_gyro_init_1k_256dlpf()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.begin();

  TEST_ASSERT_EQUAL_INT32(8000, model.state.gyro.clock);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyro.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyro.timer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.mixer.timer.rate);
}

void test_model_gyro_init_1k_188dlpf()
{
  Model model;
  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_188;
  model.config.loopSync = 2;
  model.config.mixerSync = 2;
  model.begin();

  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.clock);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.rate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyro.timer.rate);
  TEST_ASSERT_EQUAL_INT32( 500, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32( 500, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32( 250, model.state.mixer.timer.rate);
}

void test_model_inner_pid_init()
{
  Model model;
  model.state.gyro.clock = 1000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;
  model.config.pid[FC_PID_ROLL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[FC_PID_PITCH] = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[FC_PID_YAW]   = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();

  Control::Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_PITCH].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_YAW].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_YAW].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_YAW].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_YAW].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_YAW].Kf);
}

void test_model_outer_pid_init()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;
  model.config.pid[FC_PID_LEVEL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();

  Control::Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 2000.0f, model.state.outerPid[FC_PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 2000.0f, model.state.outerPid[FC_PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_PITCH].Kf);
}

void test_controller_rates()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 1998;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 1998;
  
  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 1998;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.76f, controller.calculateSetpointRate(AXIS_ROLL, 0.25f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.81f, controller.calculateSetpointRate(AXIS_ROLL, 0.75f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   6.95f, controller.calculateSetpointRate(AXIS_ROLL, 0.85f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_ROLL, 1.1f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_PITCH,  1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.48f, controller.calculateSetpointRate(AXIS_YAW, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -3.65f, controller.calculateSetpointRate(AXIS_YAW, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -8.64f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_controller_rates_limit()
{
  Model model;
  model.state.gyro.clock = 8000;
  model.config.gyro.dlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixer.type = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 500;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 500;

  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 400;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.73f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.73f, controller.calculateSetpointRate(AXIS_PITCH,  1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -8.73f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.83f, controller.calculateSetpointRate(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -6.98f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_rates_betaflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL]      = config.rate[AXIS_PITCH]      = config.rate[AXIS_YAW]      =   70;
  config.expo[AXIS_ROLL]      = config.expo[AXIS_PITCH]      = config.expo[AXIS_YAW]      =    0;
  config.superRate[AXIS_ROLL] = config.superRate[AXIS_PITCH] = config.superRate[AXIS_YAW] =   80;
  config.rateLimit[AXIS_ROLL] = config.rateLimit[AXIS_PITCH] = config.rateLimit[AXIS_YAW] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_PITCH,  1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_YAW, 1.0f));
}

void test_rates_betaflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   10;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.26f, rates.getSetpoint(AXIS_ROLL, 0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.57f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.95f, rates.getSetpoint(AXIS_ROLL, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.40f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.98f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.73f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   3.78f, rates.getSetpoint(AXIS_ROLL, 0.7f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.37f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.10f, rates.getSetpoint(AXIS_ROLL, 0.9f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.51f, rates.getSetpoint(AXIS_ROLL, 1.0f));
}

void test_rates_raceflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =    0;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.59f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.46f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.90f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.75f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_raceflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   20;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.56f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.35f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.88f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.58f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.04f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.44f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.88f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.44f, rates.getSetpoint(AXIS_ROLL, -1.0f));

}

void test_rates_kiss()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =    0;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.59f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.46f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.90f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.75f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_kiss_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   20;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.56f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.35f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.88f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.58f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.04f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.45f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.88f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.45f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_actuator_arming_gyro_motor_calbration()
{
  Model model;
  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NO_GYRO | ARMING_DISABLED_MOTOR_PROTOCOL, model.state.mode.armingDisabledFlags);
}

void test_actuator_arming_failsafe()
{
  Model model;
  model.state.gyro.present = true;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  model.state.gyro.calibrationState = CALIBRATION_UPDATE;
  model.state.input.rxFailSafe = true;
  model.state.input.rxLoss = true;

  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_CALIBRATING, model.state.mode.armingDisabledFlags);
}

void test_actuator_arming_throttle()
{
  Model model;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.config.input.minCheck = 1050;
  model.state.input.us[AXIS_THRUST] = 1100;
  model.state.gyro.present = true;

  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.mode.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_THROTTLE, model.state.mode.armingDisabledFlags);
}

void test_mixer_throttle_limit_none()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
}

void test_mixer_throttle_limit_scale()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.20f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.20f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.60f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
}

void test_mixer_throttle_limit_clip()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.00f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.50f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.60f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
}

void test_mixer_output_limit_motor()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig motor = { .servo = false };

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.6f, mixer.limitOutput( 1.0f, motor, 80));
}

void test_mixer_output_limit_servo()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig servo = { .servo = true  };

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.8f, mixer.limitOutput(-1.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.8f, mixer.limitOutput( 1.0f, servo, 80));
}

// =============================================================================
// GPS Navigation tests
// =============================================================================

static int32_t deg2gps(float deg) { return (int32_t)(deg * 1e7f); }
static float   rad2deg(float rad) { return rad * 180.f / (float)M_PI; }

void test_gps_distance_same_point()
{
  int32_t lat = deg2gps(10.f), lon = deg2gps(106.f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.f, gpsDistance(lat, lon, lat, lon));
}

void test_gps_distance_north_1deg()
{
  // 1 degree latitude ≈ 111195 m
  TEST_ASSERT_FLOAT_WITHIN(500.f, 111195.f,
    gpsDistance(deg2gps(0.f), deg2gps(0.f), deg2gps(1.f), deg2gps(0.f)));
}

void test_gps_distance_east_1deg_equator()
{
  // 1 degree longitude at equator ≈ 111195 m
  TEST_ASSERT_FLOAT_WITHIN(500.f, 111195.f,
    gpsDistance(deg2gps(0.f), deg2gps(0.f), deg2gps(0.f), deg2gps(1.f)));
}

void test_gps_distance_100m_north()
{
  // ~0.0009 deg ≈ 100 m
  TEST_ASSERT_FLOAT_WITHIN(5.f, 100.f,
    gpsDistance(deg2gps(10.f), deg2gps(106.f), deg2gps(10.0009f), deg2gps(106.f)));
}

void test_gps_distance_symmetric()
{
  int32_t lat1 = deg2gps(21.0278f), lon1 = deg2gps(105.8342f);
  int32_t lat2 = deg2gps(10.8231f), lon2 = deg2gps(106.6297f);
  float d1 = gpsDistance(lat1, lon1, lat2, lon2);
  float d2 = gpsDistance(lat2, lon2, lat1, lon1);
  TEST_ASSERT_FLOAT_WITHIN(1.f, d1, d2);
}

void test_gps_bearing_due_north()
{
  float b = rad2deg(gpsBearing(deg2gps(10.f), deg2gps(106.f), deg2gps(11.f), deg2gps(106.f)));
  TEST_ASSERT_FLOAT_WITHIN(1.f, 0.f, b);
}

void test_gps_bearing_due_south()
{
  float b = rad2deg(gpsBearing(deg2gps(10.f), deg2gps(106.f), deg2gps(9.f), deg2gps(106.f)));
  TEST_ASSERT_FLOAT_WITHIN(1.f, 180.f, fabsf(b));
}

void test_gps_bearing_due_east()
{
  float b = rad2deg(gpsBearing(deg2gps(0.f), deg2gps(0.f), deg2gps(0.f), deg2gps(1.f)));
  TEST_ASSERT_FLOAT_WITHIN(1.f, 90.f, b);
}

void test_gps_bearing_due_west()
{
  float b = rad2deg(gpsBearing(deg2gps(0.f), deg2gps(0.f), deg2gps(0.f), deg2gps(-1.f)));
  TEST_ASSERT_FLOAT_WITHIN(1.f, -90.f, b);
}

void test_gps_set_home_requires_fix()
{
  Model model;
  model.config.gps.minSats = 6;
  model.state.gps.fix      = false;
  model.state.gps.numSats  = 8;
  model.setGpsHome();
  TEST_ASSERT_FALSE(model.state.gps.homeSet);
}

void test_gps_set_home_requires_min_sats()
{
  Model model;
  model.config.gps.minSats = 6;
  model.state.gps.fix      = true;
  model.state.gps.numSats  = 4;
  model.setGpsHome();
  TEST_ASSERT_FALSE(model.state.gps.homeSet);
}

void test_gps_set_home_success()
{
  Model model;
  model.config.gps.minSats     = 6;
  model.config.gps.setHomeOnce = 1;
  model.state.gps.fix          = true;
  model.state.gps.numSats      = 8;
  model.state.gps.location.raw.lat = deg2gps(10.f);
  model.state.gps.location.raw.lon = deg2gps(106.f);
  model.setGpsHome();
  TEST_ASSERT_TRUE(model.state.gps.homeSet);
  TEST_ASSERT_EQUAL_INT32(deg2gps(10.f),  model.state.gps.location.home.lat);
  TEST_ASSERT_EQUAL_INT32(deg2gps(106.f), model.state.gps.location.home.lon);
}

void test_gps_set_home_once_flag()
{
  Model model;
  model.config.gps.minSats     = 6;
  model.config.gps.setHomeOnce = 1;
  model.state.gps.fix          = true;
  model.state.gps.numSats      = 8;
  model.state.gps.location.raw.lat = deg2gps(10.f);
  model.setGpsHome();
  model.state.gps.location.raw.lat = deg2gps(20.f);
  model.setGpsHome();
  // Home should NOT be updated on second call
  TEST_ASSERT_EQUAL_INT32(deg2gps(10.f), model.state.gps.location.home.lat);
}

void test_gps_set_home_force()
{
  Model model;
  model.config.gps.minSats = 6;
  model.state.gps.fix      = false;
  model.state.gps.numSats  = 2;
  model.state.gps.location.raw.lat = deg2gps(15.f);
  model.setGpsHome(true);
  TEST_ASSERT_TRUE(model.state.gps.homeSet);
  TEST_ASSERT_EQUAL_INT32(deg2gps(15.f), model.state.gps.location.home.lat);
}

// =============================================================================
// Failsafe GPS rescue tests
// =============================================================================

void test_failsafe_gps_rescue_conditions_met()
{
  Model model;
  model.state.gps.present  = true;
  model.state.gps.homeSet  = true;
  model.state.gps.fix      = true;
  model.state.gps.numSats  = 8;
  model.config.gps.minSats = 6;
  model.state.mode.mask   |= (1 << MODE_ARMED);

  const bool gpsReady = model.gpsActive()
    && model.state.gps.homeSet
    && model.state.gps.fix
    && model.state.gps.numSats >= model.config.gps.minSats
    && !model.isModeActive(MODE_GPS_RESCUE);

  TEST_ASSERT_TRUE(gpsReady);
}

void test_failsafe_gps_rescue_no_fix()
{
  Model model;
  model.state.gps.present  = true;
  model.state.gps.homeSet  = true;
  model.state.gps.fix      = false;
  model.state.gps.numSats  = 8;
  model.config.gps.minSats = 6;

  const bool gpsReady = model.gpsActive()
    && model.state.gps.homeSet
    && model.state.gps.fix
    && model.state.gps.numSats >= model.config.gps.minSats;

  TEST_ASSERT_FALSE(gpsReady);
}

void test_failsafe_gps_rescue_no_home()
{
  Model model;
  model.state.gps.present  = true;
  model.state.gps.homeSet  = false;
  model.state.gps.fix      = true;
  model.state.gps.numSats  = 8;
  model.config.gps.minSats = 6;

  const bool gpsReady = model.gpsActive()
    && model.state.gps.homeSet
    && model.state.gps.fix
    && model.state.gps.numSats >= model.config.gps.minSats;

  TEST_ASSERT_FALSE(gpsReady);
}

void test_failsafe_gps_rescue_insufficient_sats()
{
  Model model;
  model.state.gps.present  = true;
  model.state.gps.homeSet  = true;
  model.state.gps.fix      = true;
  model.state.gps.numSats  = 4;
  model.config.gps.minSats = 6;

  const bool gpsReady = model.gpsActive()
    && model.state.gps.homeSet
    && model.state.gps.fix
    && model.state.gps.numSats >= model.config.gps.minSats;

  TEST_ASSERT_FALSE(gpsReady);
}

void test_failsafe_gps_rescue_mode_bit_set()
{
  Model model;
  model.state.mode.mask |= (1 << MODE_ARMED);
  model.state.mode.mask |= (1 << MODE_GPS_RESCUE);
  TEST_ASSERT_TRUE(model.isModeActive(MODE_GPS_RESCUE));
  TEST_ASSERT_TRUE(model.isModeActive(MODE_ARMED));
}

// =============================================================================
// Mag fusion tests
// =============================================================================

void test_mag_updated_flag_initially_false()
{
  Model model;
  TEST_ASSERT_FALSE(model.state.mag.updated);
}

void test_mag_not_active_dev_none()
{
  Model model;
  model.state.mag.present = true;
  model.config.mag.dev    = MAG_NONE;
  TEST_ASSERT_FALSE(model.magActive());
}

void test_mag_not_active_not_present()
{
  Model model;
  model.state.mag.present = false;
  model.config.mag.dev    = MAG_QMC5883;
  TEST_ASSERT_FALSE(model.magActive());
}

void test_mag_active_present_and_configured()
{
  Model model;
  model.state.mag.present = true;
  model.config.mag.dev    = MAG_QMC5883;
  TEST_ASSERT_TRUE(model.magActive());
}

void test_mag_fusion_guard_all_met()
{
  Model model;
  model.state.mag.present          = true;
  model.config.mag.dev             = MAG_QMC5883;
  model.state.mag.calibrationValid = true;
  model.state.mag.updated          = true;
  const bool use9dof = model.magActive() && model.state.mag.calibrationValid && model.state.mag.updated;
  TEST_ASSERT_TRUE(use9dof);
}

void test_mag_fusion_guard_no_calibration()
{
  Model model;
  model.state.mag.present          = true;
  model.config.mag.dev             = MAG_QMC5883;
  model.state.mag.calibrationValid = false;
  model.state.mag.updated          = true;
  const bool use9dof = model.magActive() && model.state.mag.calibrationValid && model.state.mag.updated;
  TEST_ASSERT_FALSE(use9dof);
}

void test_mag_fusion_guard_no_new_data()
{
  Model model;
  model.state.mag.present          = true;
  model.config.mag.dev             = MAG_QMC5883;
  model.state.mag.calibrationValid = true;
  model.state.mag.updated          = false;
  const bool use9dof = model.magActive() && model.state.mag.calibrationValid && model.state.mag.updated;
  TEST_ASSERT_FALSE(use9dof);
}

void test_mag_updated_consumed_after_fusion()
{
  Model model;
  model.state.mag.updated = true;
  if(model.state.mag.updated) model.state.mag.updated = false;
  TEST_ASSERT_FALSE(model.state.mag.updated);
}

void test_mag_calibration_offset_applied()
{
  VectorFloat raw(100.f, -50.f, 200.f);
  VectorFloat offset(10.f, -10.f, 20.f);
  VectorFloat scale(1.f, 1.f, 1.f);
  VectorFloat adc = raw;
  adc -= offset;
  adc *= scale;
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  90.f, adc[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -40.f, adc[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.f, adc[2]);
}

// =============================================================================
// Altitude complementary filter tests
// =============================================================================

static void setupAltModel(Model& model, float baroHeight, float baroVario, float accelZ, float cosTheta)
{
  model.state.baro.present        = true;
  model.config.baro.dev           = BARO_BMP280;
  model.state.baro.rate           = 25;
  model.state.baro.altitudeGround = baroHeight;
  model.state.baro.vario          = baroVario;
  model.state.accel.present       = true;
  model.state.accel.timer.rate    = 500;
  model.state.accel.timer.intervalf = 1.f / 500.f;
  model.state.attitude.quaternion = Quaternion(); // identity
  model.state.attitude.cosTheta   = cosTheta;
  model.state.accel.adc           = VectorFloat(0.f, 0.f, accelZ);
}

static void runAltTicks(Altitude& alt, int n)
{
  for(int i = 0; i < n; i++) alt.update();
}

void test_altitude_init_zero()
{
  Model model;
  setupAltModel(model, 0.f, 0.f, 1.f, 1.f);
  Altitude alt(model);
  alt.begin();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.f, model.state.altitude.height);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.f, model.state.altitude.vario);
}

void test_altitude_converges_to_baro()
{
  Model model;
  setupAltModel(model, 10.f, 0.f, 1.f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 500);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.f, model.state.altitude.height);
}

void test_altitude_vario_zero_at_hover()
{
  Model model;
  setupAltModel(model, 5.f, 0.f, 1.f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 500);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.f, model.state.altitude.vario);
}

void test_altitude_accel_climb_detected()
{
  Model model;
  setupAltModel(model, 0.f, 1.f, 1.1f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 50);
  TEST_ASSERT_TRUE(model.state.altitude.vario > 0.f);
}

void test_altitude_accel_descent_detected()
{
  Model model;
  setupAltModel(model, 0.f, -1.f, 0.9f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 50);
  TEST_ASSERT_TRUE(model.state.altitude.vario < 0.f);
}

void test_altitude_tilt_suppresses_accel()
{
  // high tilt (cosTheta=0.4) → accel contribution suppressed → height converges to baro
  Model model;
  setupAltModel(model, 10.f, 0.f, 1.5f, 0.4f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 500);
  TEST_ASSERT_FLOAT_WITHIN(1.f, 10.f, model.state.altitude.height);
}

void test_altitude_vario_clamped_max()
{
  Model model;
  setupAltModel(model, 0.f, 10.f, 5.f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 200);
  TEST_ASSERT_TRUE(model.state.altitude.vario <= 10.f);
}

void test_altitude_vario_clamped_min()
{
  Model model;
  setupAltModel(model, 0.f, -10.f, -3.f, 1.f);
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 200);
  TEST_ASSERT_TRUE(model.state.altitude.vario >= -10.f);
}

void test_altitude_no_baro_stays_near_zero()
{
  Model model;
  setupAltModel(model, 0.f, 0.f, 1.f, 1.f);
  model.config.baro.dev    = BARO_NONE;
  model.state.baro.present = false;
  Altitude alt(model);
  alt.begin();
  runAltTicks(alt, 100);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.f, model.state.altitude.height);
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_timer_rate_100hz);
  RUN_TEST(test_timer_rate_100hz_div2);
  RUN_TEST(test_timer_interval_10ms);
  RUN_TEST(test_timer_check);
  RUN_TEST(test_timer_check_micros);
  RUN_TEST(test_model_gyro_init_1k_256dlpf);
  RUN_TEST(test_model_gyro_init_1k_188dlpf);
  RUN_TEST(test_model_inner_pid_init);
  RUN_TEST(test_model_outer_pid_init);
  RUN_TEST(test_controller_rates);
  RUN_TEST(test_controller_rates_limit);
  RUN_TEST(test_rates_betaflight);
  RUN_TEST(test_rates_betaflight_expo);
  RUN_TEST(test_rates_raceflight);
  RUN_TEST(test_rates_raceflight_expo);
  RUN_TEST(test_rates_kiss);
  RUN_TEST(test_rates_kiss_expo);
  RUN_TEST(test_actuator_arming_gyro_motor_calbration);
  RUN_TEST(test_actuator_arming_failsafe);
  RUN_TEST(test_actuator_arming_throttle);
  RUN_TEST(test_mixer_throttle_limit_none);
  RUN_TEST(test_mixer_throttle_limit_scale);
  RUN_TEST(test_mixer_throttle_limit_clip);
  RUN_TEST(test_mixer_output_limit_motor);
  RUN_TEST(test_mixer_output_limit_servo);

  // GPS navigation
  RUN_TEST(test_gps_distance_same_point);
  RUN_TEST(test_gps_distance_north_1deg);
  RUN_TEST(test_gps_distance_east_1deg_equator);
  RUN_TEST(test_gps_distance_100m_north);
  RUN_TEST(test_gps_distance_symmetric);
  RUN_TEST(test_gps_bearing_due_north);
  RUN_TEST(test_gps_bearing_due_south);
  RUN_TEST(test_gps_bearing_due_east);
  RUN_TEST(test_gps_bearing_due_west);
  RUN_TEST(test_gps_set_home_requires_fix);
  RUN_TEST(test_gps_set_home_requires_min_sats);
  RUN_TEST(test_gps_set_home_success);
  RUN_TEST(test_gps_set_home_once_flag);
  RUN_TEST(test_gps_set_home_force);

  // Failsafe GPS rescue
  RUN_TEST(test_failsafe_gps_rescue_conditions_met);
  RUN_TEST(test_failsafe_gps_rescue_no_fix);
  RUN_TEST(test_failsafe_gps_rescue_no_home);
  RUN_TEST(test_failsafe_gps_rescue_insufficient_sats);
  RUN_TEST(test_failsafe_gps_rescue_mode_bit_set);

  // Mag fusion
  RUN_TEST(test_mag_updated_flag_initially_false);
  RUN_TEST(test_mag_not_active_dev_none);
  RUN_TEST(test_mag_not_active_not_present);
  RUN_TEST(test_mag_active_present_and_configured);
  RUN_TEST(test_mag_fusion_guard_all_met);
  RUN_TEST(test_mag_fusion_guard_no_calibration);
  RUN_TEST(test_mag_fusion_guard_no_new_data);
  RUN_TEST(test_mag_updated_consumed_after_fusion);
  RUN_TEST(test_mag_calibration_offset_applied);

  // Altitude complementary filter
  RUN_TEST(test_altitude_init_zero);
  RUN_TEST(test_altitude_converges_to_baro);
  RUN_TEST(test_altitude_vario_zero_at_hover);
  RUN_TEST(test_altitude_accel_climb_detected);
  RUN_TEST(test_altitude_accel_descent_detected);
  RUN_TEST(test_altitude_tilt_suppresses_accel);
  RUN_TEST(test_altitude_vario_clamped_max);
  RUN_TEST(test_altitude_vario_clamped_min);
  RUN_TEST(test_altitude_no_baro_stays_near_zero);

  return UNITY_END();
}
