#include "matrix/math.hpp"
using matrix::Vector2f;

class Nav_Relative {
public:
  int32_t home_lat = 0;
  int32_t home_lon = 0;
  const float lat_to_m = 1852 * 60 * 1e-7;
  float lon_to_m = 0;
  
  void set_home(int32_t lat, int32_t lon) {
    home_lat = lat;
    home_lon = lon;
    lon_to_m = lat_to_m * cos(fabs(lat * 1e-7));
  }

/*
  void to_m(int32_t lat, int32_t lon, Vector2f &vec) {
    vec(0) = (lon - home_lon) * lon_to_m;
    vec(1) = (lat - home_lat) * lat_to_m;
  }
*/

  Vector2f to_m(int32_t lat, int32_t lon) {
    Vector2f vec;
    vec(0) = (lon - home_lon) * lon_to_m;
    vec(1) = (lat - home_lat) * lat_to_m;
    return vec;
  }
};