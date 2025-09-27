/**
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <robin.lilja@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return. - Robin Lilja
 *
 * @file altitude_kf.cpp
 * @author Robin Lilja
 * @date 23 Jul 2015
 */

#include "altitude_kf.h"

void Altitude_KF::propagate(float acceleration, const float dt) {
  // Repeated arithmetics
  float _dtdt = dt * dt;

  // The state vector is defined as x = [h v]' where  'h' is altitude above ground and 'v' velocity, both
  // aligned with the vertical direction of the Earth NED frame, but positive direction being upwards to zenith.

  // State-space system model 'x_k = A*x_k-1 + B*u_k is given by:
  //
  //	x_k = [ h_k ] = [ 1 dT ] * [ h_k-1 ] + [ 1/2*dT^2 ] * u_k
  //  	      [ v_k ]   [ 0  1 ]   [ v_k-1 ]   [ dT       ]
  //
  //			   A			     B
  //
  // where 'u_k' is our acceleration input signal.

  // Propagation of the state (equation of motion) by Euler integration
  h = h + v*dt + 0.5f*acceleration*_dtdt;
  v = v + acceleration*dt;

  // The "a priori" state estimate error covariance 'P_k|k-1 = A * P_k-1 * A' + Q_k' is calculated as follows:
  //
  // P_k|k-1 = [ 1 dT ] * P_k-1 * [  1 0 ] + Q_k
  //	     [ 0  1 ]	        [ dT 1 ]

  // The process noise covariance matrix 'Q' is a bit trickier to derive, but consider some additive noise 'w_k' perturbing the
  // true acceleration 'a_k', thus the input signal is 'u_k = a_k + w_k'. The affect of 'w_k' on the state estimate is by linearity
  // described by [1/2*dT^2 dT]' i.e. the input matrix 'B'. We call this new matrix 'G'.
  //
  // Then, by definition* 'Q' equals 'G * G' * σ^2', which in our case translates into:
  //
  // Q_k = G_k * G'_k * σ_accelerometer^2 = [(dT^4)/4 (dT^3)/2] * σ_accelerometer^2
  //					  [(dT^3)/2     dT^2]
  //
  // * I only get half of the math showing 'Q = G * G' * σ^2', so I hide myself behind 'by definition'.

  // Calculate the state estimate covariance
  //
  // Repeated arithmetics
  float _Q_accel_dtdt = accCov * _dtdt;
  //
  P00 = P00 + (P10 + P01 + (P11 + 0.25f*_Q_accel_dtdt) * dt) * dt;
  P01 = P01 + (P11 + 0.5f*_Q_accel_dtdt) * dt;
  P10 = P10 + (P11 + 0.5f*_Q_accel_dtdt) * dt;
  P11 = P11 + _Q_accel_dtdt;
}

void Altitude_KF::update(float altitude) {
  // Observation vector 'zhat' from the current state estimate:
  //
  // zhat_k = [ 1 0 ] * [ h_k ]
  //                    [ v_k ]
  //             H

  // 'H' is constant, so its time instance I'm using below is a bit ambitious.

  // The innovation (or residual) is given by 'y = z - zhat', where 'z' is the actual observation i.e. measured state.

  // Calculate innovation, in this particular case we observe the altitude state directly by an altitude measurement
  float y = altitude - h;

  // The innovation covariance is defined as 'S_k = H_k * P_k|k-1 * H'_k + R_k', for this particular case
  // 'H_k * P_k|k-1 * H'_k' is equal to the first row first column element of 'P_k|k-1' i.e. P_00.

  // The Kalman gain equals 'K_k = P_k|k-1 * H'_k * S_k^-1', where
  //
  // P_k|k-1 * H'_k = [ P_00 ]
  //                  [ P_10 ]
  //
  // and 'S_k^-1' equals '1/S_k' since 'S_k^-1' is being a scalar (that is a good thing!).

  // Calculate the inverse of the innovation covariance
  float Sinv = 1.0f / (P00 + altCov);

  // Calculate the Kalman gain
  float K0 =  P00 * Sinv;
  float K1 =  P10 * Sinv;

  // Update the state estimate
  h += K0 * y;
  v += K1 * y;

  // The "a posteriori" state estimate error covariance equals 'P_k|k = (I - K_k * H_k) * P_k|k-1', where
  //
  //  (I - K_k * H_k) = ( [ 1 0 ] - [ K_0 ] * [ 1 0 ] ) = [ (1-K_0) 0  ] , thus
  //                    ( [ 0 1 ]   [ K_1 ]           )   [ -K_1    1  ]
  //
  //  P_k|k = (I - K_k * H_k) * P_k+1|k = [ (1-K_0) 0 ] * [ P_00 P_01 ] = [ (1-K_0)*P_00       (1-K_0)*P_01       ]
  //					[ -K_1    1 ]   [ P_10 P_11 ]   [ (-K_1*P_00 + P_10) (-K_1*P_01 + P_11) ]

  // Calculate the state estimate covariance
  P00 = P00 - K0 * P00;
  P01 = P01 - K0 * P01;
  P10 = P10 - K1 * P00;
  P11 = P11 - K1 * P01;
}
