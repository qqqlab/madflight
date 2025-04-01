//==============================================================================================================
//  Madgwick Sensor Fusion
//==============================================================================================================

class Madgwick {
public:
  float ahrs_MadgwickB =  0.041;     //Madgwick filter parameter

  //NED reference frame
  //gyro in rad/sec
  //acc in g
  //mag any unit of measurement

  //Initialize quaternion for Madgwick filter
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

  void update9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
  void update6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
};
