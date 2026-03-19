#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include "Utils/Math.hpp"
#include <cmath>
#include <algorithm>

namespace Espfc::Control {

/**
 * Altitude estimator using complementary filter: barometer + accelerometer.
 *
 * Baro provides slow but absolute height reference.
 * Accel provides fast vertical dynamics (after removing gravity and tilt compensation).
 *
 * Two-state complementary filter:
 *   predict:  height += vel * dt
 *             vel    += accel_z * dt
 *   correct:  height += k_h * (baro_height - height)
 *             vel    += k_v * (baro_vario  - vel)
 *
 * k_h / k_v control how much baro pulls the estimate — tune smaller for smoother
 * (more accel-driven) or larger for faster baro tracking.
 */
class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.height = 0.0f;
    _model.state.altitude.vario  = 0.0f;
    _height = 0.0f;
    _vel    = 0.0f;
    _initialized = false;

    // Baro pre-filter: smooth raw baro before feeding into complementary filter
    // Higher cutoff than before because accel handles fast noise rejection
    _baroFilter.begin(FilterConfig(FILTER_PT2, 10), _model.state.accel.timer.rate);
    _baroVarioFilter.begin(FilterConfig(FILTER_PT1, 10), _model.state.accel.timer.rate);

    // Accel vertical noise filter (remove vibration above 10Hz)
    _accelZFilter.begin(FilterConfig(FILTER_PT2, 10), _model.state.accel.timer.rate);

    return 1;
  }

  int update()
  {
    const float dt = _model.state.accel.timer.intervalf;

    // --- Baro measurement ---
    const float baroHeight = _baroFilter.update(_model.state.baro.altitudeGround);
    const float baroVario  = _baroVarioFilter.update(_model.state.baro.vario);

    // --- Initialize state on first run ---
    if(!_initialized)
    {
      _height = baroHeight;
      _vel    = 0.0f;
      _initialized = true;
      _model.state.altitude.height = _height;
      _model.state.altitude.vario  = _vel;
      return 1;
    }

    // --- Vertical linear acceleration from IMU ---
    // accel.adc is in body frame (units: g).
    // Rotate to world frame using current attitude quaternion.
    // Subtract 1g (gravity) to get linear vertical acceleration.
    const VectorFloat accelWorld = _model.state.accel.adc.getRotated(_model.state.attitude.quaternion);
    const float accelZ = _accelZFilter.update((accelWorld.z - 1.0f) * GRAVITY_MSS); // m/s²

    // Tilt compensation: when leaned, vertical thrust component is reduced.
    // cosTheta = cos(lean angle) is already computed in Fusion.
    // Only trust accel when not excessively tilted (cosTheta > 0.7 ~ 45 deg)
    const float cosTheta = _model.state.attitude.cosTheta;
    const float accelTrust = std::max(0.f, (cosTheta - 0.5f) * 2.f); // 0 at 60deg, 1 at 60deg+ lean=0

    // --- Complementary filter predict step ---
    _vel    += accelZ * accelTrust * dt;
    _height += _vel * dt;

    // --- Complementary filter correct step (baro pulls estimate) ---
    // Only correct when baro data is fresh (baro runs much slower than accel)
    const bool baroFresh = _model.baroActive() && (_model.state.baro.rate > 0);
    if(baroFresh)
    {
      _height += BARO_GAIN_HEIGHT * (baroHeight - _height);
      _vel    += BARO_GAIN_VARIO  * (baroVario  - _vel);
    }

    // Clamp velocity to prevent runaway
    _vel = std::clamp(_vel, -MAX_VARIO_MS, MAX_VARIO_MS);

    _model.state.altitude.height = _height;
    _model.state.altitude.vario  = _vel;

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] = std::clamp(lrintf(baroHeight * 100.0f),    -32000l, 32000l); // baro height cm
      _model.state.debug[1] = std::clamp(lrintf(baroVario  * 100.0f),    -32000l, 32000l); // baro vario cm/s
      _model.state.debug[2] = std::clamp(lrintf(_height * 100.0f),        -32000l, 32000l); // fused height cm
      _model.state.debug[3] = std::clamp(lrintf(_vel    * 100.0f),        -32000l, 32000l); // fused vario cm/s
      _model.state.debug[4] = std::clamp(lrintf(accelZ  * 100.0f),        -32000l, 32000l); // vertical accel cm/s²
      _model.state.debug[5] = std::clamp(lrintf(accelTrust * 1000.0f),    -32000l, 32000l); // tilt trust
    }

    return 1;
  }

private:
  // Complementary filter gains — tune these:
  // Higher BARO_GAIN_HEIGHT = more baro trust (faster but noisier height)
  // Lower  BARO_GAIN_HEIGHT = more accel trust (smoother but slower to correct)
  static constexpr float BARO_GAIN_HEIGHT = 0.02f;  // height correction per update
  static constexpr float BARO_GAIN_VARIO  = 0.04f;  // velocity correction per update
  static constexpr float GRAVITY_MSS      = 9.80665f;
  static constexpr float MAX_VARIO_MS     = 10.0f;  // m/s max vertical speed clamp

  Model& _model;

  Utils::Filter _baroFilter;
  Utils::Filter _baroVarioFilter;
  Utils::Filter _accelZFilter;

  float _height = 0.f;
  float _vel    = 0.f;
  bool  _initialized = false;
};

}
