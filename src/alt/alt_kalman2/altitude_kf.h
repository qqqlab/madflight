/**
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <robin.lilja@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return. - Robin Lilja
 *
 * @file altitude_kf.h
 * @author Robin Lilja
 * @date 23 Jul 2015
 */

#pragma once

/**
 * A linear Kalman filter estimator of altitude and vertical velocity.
 */
class Altitude_KF {

public:
  Altitude_KF() {
    setup(1.0, 0.5); //default covariance: 1.0m altitude, 0.5m/s^2 accel
  }

  /**
   * Setup.
   * @param accCov covariance of acceleration input signal (σ^2).
   * @param altCov covariance of the altitude measurement (σ^2).
   */

  void setup(float altCov, float accCov) {
    this->altCov = altCov;
    this->accCov = accCov;
    h = 0.0f;
    v = 0.0f;
  }

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

  float h; //Estimated vertical height or altitude in Earth frame (positive in the zenith direction) [m].
  float v; //Estimated vertical velocity (positive in the zenith direction) [m/s].
  float accCov; //Accelerometer covariance
  float altCov; //Altitude measurement covariance

  private:
    //Predicted covariance matrix 'P'
    float P00 = 1;
    float P01 = 0;
    float P10 = 0;
    float P11 = 1;
};
