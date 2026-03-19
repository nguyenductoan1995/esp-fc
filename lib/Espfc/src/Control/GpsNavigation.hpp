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

// Haversine distance between two GPS coordinates (deg*1e7)
// Returns distance in meters
static float gpsDistance(int32_t lat1e7, int32_t lon1e7, int32_t lat2e7, int32_t lon2e7)
{
  static constexpr float R = 6371000.f; // Earth radius in meters
  const float lat1 = gpsToRad(lat1e7);
  const float lat2 = gpsToRad(lat2e7);
  const float dlat = lat2 - lat1;
  const float dlon = gpsToRad(lon2e7) - gpsToRad(lon1e7);
  const float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f)
                + cosf(lat1) * cosf(lat2) * sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
  return R * 2.f * atan2f(sqrtf(a), sqrtf(1.f - a));
}

// Bearing from point1 to point2 in radians [-pi, pi], 0 = North, clockwise
static float gpsBearing(int32_t lat1e7, int32_t lon1e7, int32_t lat2e7, int32_t lon2e7)
{
  const float lat1 = gpsToRad(lat1e7);
  const float lat2 = gpsToRad(lat2e7);
  const float dlon = gpsToRad(lon2e7) - gpsToRad(lon1e7);
  const float y = sinf(dlon) * cosf(lat2);
  const float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
  return atan2f(y, x);
}

class GpsNavigation
{
public:
  GpsNavigation(Model& model): _model(model) {}

  int begin()
  {
    _velNorthFilter.begin(FilterConfig(FILTER_PT1, 2), 50); // 50Hz GPS rate max
    _velEastFilter.begin(FilterConfig(FILTER_PT1, 2), 50);
    return 1;
  }

  int update()
  {
    if(!_model.gpsActive()) return 0;

    updateDistanceBearing();

    if(_model.isModeActive(MODE_POSHOLD) || _model.isModeActive(MODE_GPS_RESCUE))
    {
      updatePositionSetpoint();
    }

    return 1;
  }

  // GPS velocity (mm/s) to m/s, filtered
  float velNorth() const { return _velNorth; }
  float velEast()  const { return _velEast; }

private:
  void updateDistanceBearing()
  {
    if(!_model.state.gps.homeSet || !_model.state.gps.fix) return;

    const auto& loc  = _model.state.gps.location.raw;
    const auto& home = _model.state.gps.location.home;

    const float dist = gpsDistance(home.lat, home.lon, loc.lat, loc.lon);
    const float bearing = gpsBearing(home.lat, home.lon, loc.lat, loc.lon);

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

  void updatePositionSetpoint()
  {
    if(!_model.state.gps.fix || _model.state.gps.numSats < _model.config.gps.minSats) return;

    const float dist  = (float)_model.state.gps.distanceToHome;
    const float bearing = _model.state.gps.bearingToHome;

    // Position P controller: distance error -> velocity setpoint (m/s)
    // FC_PID_POS.P used (0-255 scale -> 0..2.55 m/s per meter)
    const float posP = (float)_model.config.pid[FC_PID_POS].P * 0.01f;
    float velSetpoint = dist * posP;
    velSetpoint = std::min(velSetpoint, GPS_MAX_VELOCITY);

    // decompose into North/East velocity setpoints
    // for RTH: fly toward home (reverse of bearingToHome)
    const float targetBearing = bearing + M_PI; // toward home
    const float velNorthSp = velSetpoint * cosf(targetBearing);
    const float velEastSp  = velSetpoint * sinf(targetBearing);

    // Velocity P controller: velocity error -> lean angle setpoint (radians)
    // FC_PID_POSR.P used
    const float velP = (float)_model.config.pid[FC_PID_POSR].P * 0.001f;
    const float maxLean = (float)_model.config.gps.maxLeanAngle * M_PI / 180.f;

    float rollSp  = std::clamp((velEastSp  - _velEast)  * velP, -maxLean, maxLean);
    float pitchSp = std::clamp((velNorthSp - _velNorth) * velP, -maxLean, maxLean);

    // write setpoints (angle mode inner loop will handle them)
    _model.state.gps.posSetpoint[0] = rollSp;
    _model.state.gps.posSetpoint[1] = pitchSp;
  }

  static constexpr float GPS_MAX_VELOCITY = 5.f; // m/s max horizontal speed

  Model& _model;
  Utils::Filter _velNorthFilter;
  Utils::Filter _velEastFilter;
  float _velNorth = 0.f;
  float _velEast  = 0.f;
};

}
