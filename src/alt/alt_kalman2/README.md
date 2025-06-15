Modified for madflight, original source: https://github.com/rblilja/AltitudeKF

# AltitudeKF
A linear Kalman filter estimating altitude and vertical velocity by doing sensor fusion of acceleration and any altitude measurement sensor such as e.g. barometer or SONAR.

Instantiate by calling:

Altitude_KF alt_estimator(ALT_KF_Q_ACCEL, ALT_KF_R_PT);

Where ALT_KF_Q_ACCEL is the covariance of your accelerometer measurement, and ALT_KF_R_PT is your altitude measurement (in this case PT refers to pressure transducer). I have played around with values around 0.1 for both parameters, but a higher value for the acceleration covariance is probably needed if you have a lot of vibrations in your application.

Then, generally speaking, accelerometers can be sampled at a much higher frequency than e.g. a pressure sensor. For every accelerometer measurement you call:

alt_estimator.propagate(accel_vertical - EARTH_GRAVITY, dt);

Where accel_vertical is the vertical acceleration in the Earth frame (positive direction towards zenith). Earth gravity is whatever acceleration mother Earth pulls your system with (e.g. 9.80665 m/s^2). Last parameter is the sampling interval.

Whenever you have an altitude measurement you call:

alt_estimator.update(altitude);

Another note on vibrations and the accelerometer measurement. I often pre-filter the measurements with a low-pass filter, some would argue that you shall tune your KF accordingly. However, I find it very hard to see what the covariance parameters exactly does in the frequency domain. I personally find it much easier to just put my measurements through an IIR filter if there are any frequencies that needs to be rejected. Engineering and theory don't always play well together.
