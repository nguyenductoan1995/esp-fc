#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include <cmath>

namespace Espfc::Control {

// Convert deg*1e7 (int32) to radians
static inline float gpsToRad(int32_t deg1e7)
{
  return (float)deg1e7 * 1e-7f * M_PI / 180.f;
}

// Compute haversine distance (meters) and bearing (radians) in one pass,
// sharing all trig so coordinates are only converted once.
static void gpsDistanceBearing(int32_t lat1e7, int32_t lon1e7, int32_t lat2e7, int32_t lon2e7,
                                float& distOut, float& bearingOut)
{
  static constexpr float R = 6371000.f; // Earth radius in meters
  const float lat1 = gpsToRad(lat1e7);
  const float lat2 = gpsToRad(lat2e7);
  const float dlat = lat2 - lat1;
  const float dlon = gpsToRad(lon2e7) - gpsToRad(lon1e7);

  const float cosLat1 = cosf(lat1), sinLat1 = sinf(lat1);
  const float cosLat2 = cosf(lat2), sinLat2 = sinf(lat2);
  const float sinDlon  = sinf(dlon), cosDlon = cosf(dlon);
  const float sinHDlat = sinf(dlat * 0.5f);
  const float sinHDlon = sinf(dlon * 0.5f);

  // Haversine distance
  const float a = sinHDlat * sinHDlat + cosLat1 * cosLat2 * sinHDlon * sinHDlon;
  distOut = R * 2.f * atan2f(sqrtf(a), sqrtf(1.f - a));

  // Forward bearing (0 = North, clockwise positive)
  bearingOut = atan2f(sinDlon * cosLat2, cosLat1 * sinLat2 - sinLat1 * cosLat2 * cosDlon);
}

/**
 * Cascade PID position hold controller:
 *
 *  Outer loop (position, ~10Hz GPS rate):
 *    distance error (m) --> velocity setpoint (m/s)   [P only]
 *
 *  Inner loop (velocity, ~10Hz GPS rate):
 *    velocity error (m/s) --> lean angle setpoint (rad)  [PID]
 *      P  : immediate response to velocity error
 *      I  : eliminates steady-state drift (wind, imbalance)
 *      D  : damps overshoot when approaching setpoint
 *
 *  PID gains come from FC_PID_POS (outer) and FC_PID_POSR (inner).
 *  I/D terms are only updated when fresh GPS data arrives (~10Hz).
 *  All I terms are reset when mode is deactivated.
 */
class GpsNavigation
{
public:
  GpsNavigation(Model& model): _model(model) {}

  int begin()
  {
    _velNorthFilter.begin(FilterConfig(FILTER_PT1, 2), 50); // 50Hz GPS rate max
    _velEastFilter.begin(FilterConfig(FILTER_PT1, 2), 50);
    resetPid();
    return 1;
  }

  int update()
  {
    if(!_model.gpsActive()) return 0;

    // Always update distance/bearing for OSD/telemetry regardless of mode
    const bool gpsFresh = (_model.state.gps.lastMsgTs != _lastGpsMsgTs);
    if(gpsFresh)
    {
      _lastGpsMsgTs = _model.state.gps.lastMsgTs;
      updateDistanceBearing();
    }

    const bool modeActive = _model.isModeActive(MODE_POSHOLD) || _model.isModeActive(MODE_GPS_RESCUE);

    if(!modeActive)
    {
      // Reset PID state when leaving mode so stale integral doesn't kick on re-entry
      if(_wasActive) resetPid();
      _wasActive = false;
      return 1;
    }

    // First frame after activation: seed prevErr to current error to avoid D spike
    if(!_wasActive)
    {
      _prevErrNorth = 0.f;
      _prevErrEast  = 0.f;
      _firstFrame   = true;
    }
    _wasActive = true;

    updatePositionSetpoint(gpsFresh);

    return 1;
  }

  // GPS velocity (mm/s) to m/s, filtered
  float velNorth() const { return _velNorth; }
  float velEast()  const { return _velEast; }

private:
  void resetPid()
  {
    _iTermNorth   = 0.f;
    _iTermEast    = 0.f;
    _prevErrNorth = 0.f;
    _prevErrEast  = 0.f;
    _dTermNorth   = 0.f;
    _dTermEast    = 0.f;
    _firstFrame   = true;
  }

  void updateDistanceBearing()
  {
    if(!_model.state.gps.homeSet || !_model.state.gps.fix) return;

    const auto& loc  = _model.state.gps.location.raw;
    const auto& home = _model.state.gps.location.home;

    float dist, bearing;
    gpsDistanceBearing(home.lat, home.lon, loc.lat, loc.lon, dist, bearing);

    _model.state.gps.distanceToHome = (uint16_t)std::min(dist, 65535.f);
    _model.state.gps.bearingToHome  = bearing;

    // directionToHome: angle to fly home relative to craft heading (deg, -180..180)
    const float craftHeading = (float)_model.state.gps.velocity.raw.heading * 1e-5f * M_PI / 180.f;
    float dir = bearing - craftHeading + M_PI; // reverse: home direction
    // normalize to [-pi, pi]
    while(dir >  M_PI) dir -= 2.f * M_PI;
    while(dir < -M_PI) dir += 2.f * M_PI;
    _model.state.gps.directionToHome = (int16_t)(dir * 180.f / M_PI);

    // filter GPS velocity
    _velNorth = _velNorthFilter.update(_model.state.gps.velocity.raw.north * 0.001f); // mm/s -> m/s
    _velEast  = _velEastFilter.update(_model.state.gps.velocity.raw.east  * 0.001f);
  }

  void updatePositionSetpoint(bool gpsFresh)
  {
    if(!_model.state.gps.fix || _model.state.gps.numSats < _model.config.gps.minSats) return;

    const float dist    = (float)_model.state.gps.distanceToHome;
    const float bearing = _model.state.gps.bearingToHome;
    const float maxLean = (float)_model.config.gps.maxLeanAngle * M_PI / 180.f;

    // --- Outer loop: position P → velocity setpoint ---
    // FC_PID_POS.P: 0-255 scale → 0..2.55 m/s per meter
    const float posP = (float)_model.config.pid[FC_PID_POS].P * 0.01f;
    const float velSetpoint = std::min(dist * posP, GPS_MAX_VELOCITY);

    // Decompose into North/East velocity setpoints (fly toward hold point)
    const float targetBearing = bearing + M_PI;
    const float velNorthSp = velSetpoint * cosf(targetBearing);
    const float velEastSp  = velSetpoint * sinf(targetBearing);

    // --- Inner loop: velocity PID → lean angle setpoint ---
    // FC_PID_POSR: P * 0.001, I * 0.0001, D * 0.00001
    const float velP = (float)_model.config.pid[FC_PID_POSR].P * 0.001f;
    const float velI = (float)_model.config.pid[FC_PID_POSR].I * 0.0001f;
    const float velD = (float)_model.config.pid[FC_PID_POSR].D * 0.00001f;

    const float errNorth = velNorthSp - _velNorth;
    const float errEast  = velEastSp  - _velEast;

    // I and D terms only advance on fresh GPS data to avoid dt=0 spikes
    if(gpsFresh)
    {
      // dt from GPS interval (clamped to 50–500ms to guard against stale data)
      const float dt = std::clamp((float)_model.state.gps.interval * 0.001f, 0.05f, 0.5f);

      // Integrate error (wind / steady-state drift compensation)
      _iTermNorth += errNorth * dt * velI;
      _iTermEast  += errEast  * dt * velI;

      // Clamp I terms to half maxLean to leave room for P
      const float iLimit = maxLean * 0.5f;
      _iTermNorth = std::clamp(_iTermNorth, -iLimit, iLimit);
      _iTermEast  = std::clamp(_iTermEast,  -iLimit, iLimit);

      // D term on error derivative (damps overshoot).
      // Skipped on first frame to avoid spike from prevErr=0.
      if(!_firstFrame)
      {
        _dTermNorth = (errNorth - _prevErrNorth) / dt * velD;
        _dTermEast  = (errEast  - _prevErrEast)  / dt * velD;
      }
      _prevErrNorth = errNorth;
      _prevErrEast  = errEast;
      _firstFrame   = false;
    }

    const float pitchSp = errNorth * velP + _iTermNorth + _dTermNorth; // North → Pitch
    const float rollSp  = errEast  * velP + _iTermEast  + _dTermEast;  // East  → Roll

    _model.state.gps.posSetpoint[0] = std::clamp(rollSp,  -maxLean, maxLean);
    _model.state.gps.posSetpoint[1] = std::clamp(pitchSp, -maxLean, maxLean);
  }

  static constexpr float GPS_MAX_VELOCITY = 5.f; // m/s max horizontal speed

  Model& _model;
  Utils::Filter _velNorthFilter;
  Utils::Filter _velEastFilter;
  float    _velNorth     = 0.f;
  float    _velEast      = 0.f;
  uint32_t _lastGpsMsgTs = 0;
  bool     _wasActive    = false;
  bool     _firstFrame   = true;

  // Velocity PID state (North = Pitch axis, East = Roll axis)
  float _iTermNorth   = 0.f;
  float _iTermEast    = 0.f;
  float _prevErrNorth = 0.f;
  float _prevErrEast  = 0.f;
  float _dTermNorth   = 0.f;
  float _dTermEast    = 0.f;
};

}
