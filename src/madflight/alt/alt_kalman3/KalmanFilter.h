#pragma once

class KalmanFilter {

public :

  KalmanFilter();

  void setup(float altCov, float accCov, float biasCov = 0);

  /**
  * Propagate the state.
  * @param acceleration vertical acceleration in Earth frame (positive in the zenith direction) [m/s^2].
  * @param dt update/sampling period [s].
  */
  void propagate(float acceleration, const float dt);

  /**
   * State correction update.
   * @param altitude measurement of altitude in Earth frame (positive in the zenith direction) [m].
   */
  void update(float altitude);

  /**
   * State correction update. Use this method if you use multiple sensors measuring the altitude.
   * @param altitude measurement of altitude in Earth frame (positive in the zenith direction) [m].
   * @param altCov covariance of the altitude measurement (σ^2).
   */
  void update(float altitude, float altCov) { 
    this->altCov = altCov; 
    update(altitude);
  };

  // State being tracked
  float h;  // Estimated vertical height or altitude in Earth frame (positive in the zenith direction) [m].
  float v;  // Estimated vertical velocity (positive in the zenith direction) [m/s].
  float bias;  // Estimated acceleration bias [m/s^2]

  float accCov;  // dynamic acceleration variance
  float altCov; //  z measurement noise variance fixed
  float biasCov; // assumed fixed.

private :

  // 3x3 State Covariance matrix
  float Pzz_;
  float Pzv_;
  float Pza_;
  float Pvz_;
  float Pvv_;
  float Pva_;
  float Paz_;
  float Pav_;
  float Paa_;

};
