//==============================================================================================================
//  Mahony
//==============================================================================================================
//source: https://github.com/PaulStoffregen/MahonyAHRS

static const float ahrs_Mahony2KP = 2 * 0.5;		//Mahony: 2 * proportional gain (Kp)
static const float ahrs_Mahony2KI = 2 * 0.0;		//Mahony: 2 * integral gain (Ki)

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

void _ahrs_Mahony9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void _ahrs_Mahony6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);

void ahrs_Mahony(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
  //Use 6DOF algorithm if magnetometer measurement invalid or unavailable (avoids NaN in magnetometer normalisation)
  if( (mx == 0.0f) && (my == 0.0f) && (mz == 0.0f) ) {
    _ahrs_Mahony6DOF(gx, gy, gz, ax, ay, az, dt);
  }else{
    _ahrs_Mahony9DOF(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
  }
}

void _ahrs_Mahony9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

  //Compute feedback only if accelerometer measurement is in range 0.9g - 1.1g
  float alen2 = ax * ax + ay * ay + az * az;
  
#if AHRS_USE == AHRS_USE_MAHONY_BF
  if (0.81 <= alen2 && alen2 <= 1.21) {
#else
  if (alen2>0) {
#endif

    //Normalise accelerometer measurement
    recipNorm = 1.0/sqrtf(alen2);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = 1.0f/sqrtf(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(ahrs_Mahony2KI > 0.0f) {
			// integral error scaled by Ki
			integralFBx += ahrs_Mahony2KI * halfex * dt;
			integralFBy += ahrs_Mahony2KI * halfey * dt;
			integralFBz += ahrs_Mahony2KI * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += ahrs_Mahony2KP * halfex;
		gy += ahrs_Mahony2KP * halfey;
		gz += ahrs_Mahony2KP * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= 0.5f * dt;		// pre-multiply common factors
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1.0f/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void _ahrs_Mahony6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

  //Compute feedback only if accelerometer measurement is in range 0.9g - 1.1g
  float alen2 = ax * ax + ay * ay + az * az;
  
#if AHRS_USE == AHRS_USE_MAHONY_BF
  if (0.81 <= alen2 && alen2 <= 1.21) {
#else
  if (alen2>0) {
#endif
    
    //Normalise accelerometer measurement
    recipNorm = 1.0/sqrtf(alen2);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(ahrs_Mahony2KI > 0.0f) {
			// integral error scaled by Ki
			integralFBx += ahrs_Mahony2KI * halfex * dt;
			integralFBy += ahrs_Mahony2KI * halfey * dt;
			integralFBz += ahrs_Mahony2KI * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += ahrs_Mahony2KP * halfex;
		gy += ahrs_Mahony2KP * halfey;
		gz += ahrs_Mahony2KP * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= 0.5f * dt;		// pre-multiply common factors
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1.0f/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
