//==============================================================================================================
//  Mahony Sensor Fusion
//==============================================================================================================
//source: https://github.com/PaulStoffregen/MahonyAHRS

class Mahony {
public:
  float config_2Kp = 2 * 0.5;       //2 * proportional gain (Kp)
  float config_2Ki = 2 * 0.0;       //2 * integral gain (Ki)
  float config_alen2_min = 0;       //default 0, betaflight uses 0.81
  float config_alen2_max = 1e10;    //default 1e10, betaflight uses 1.21
  
  //NED reference frame
  //gyro in rad/sec
  //acc in g
  //mag any unit of measurement

  // quaternion of sensor frame relative to auxiliary frame
  float q0 = 1.0f; //Initialize quaternion for Madgwick filter (shared between ahrs_Madgwick6DOF and ahrs_Madgwick9DOF)
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  float integralFBx = 0, integralFBy = 0, integralFBz = 0;  // integral error terms scaled by Ki

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
  void update9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
  void update6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
};
