//TODO: Add ACCEL LPF



#include "ICM_20948.h"
#include <Servo.h>

// ===================[ Constants & Config ]===================
#define GYRO_SCALE ((M_PI / 180.0f) * 0.06106870229f) // For 2000dps setting
#define ACCEL_SCALE (0.00119710083f) // For 4G RANGE
#define SPI_PORT SPI
#define CS_PIN 5

// ===================[ Global Objects ]=======================
ICM_20948_SPI imu;
Servo output1, output2, output3, output4;

// ===================[ IMU Calibration Data ]=================
float KP = 5.0f;
float headingTrim = 0.0f;
const float G_OFFSET[3] = {-10.0f, -10.0f, -5.0f};
const float M_B[3] = {52.41f, -84.35f, 191.43f};
const float M_AINV[3][3] = {
  {1.43027, 0.04169, 0.04760},
  {0.04169, 1.46787, 0.00241},
  {0.04760, 0.00241, 1.30143}
};

// ===================[ Orientation Data ]=====================
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float yaw, pitch, roll;
float filteredPitch, filteredRoll;
const float tilt_lpf_alpha = 0.1f;
const float accel_lpf_alpha= 0.1f;

// ===================[ Timing Variables ]=====================
uint32_t now = 0, last_update = 0, last_print = 0;
uint16_t looptimer; 
float delta_t = 0.0f;
uint32_t loop_start = 0, loop_end = 0;
float loop_duration_ms, loop_frequency_hz = 0.0f;
int LOOP_HZ = 1500;
int LOOP_US = 1000000 / LOOP_HZ;

// ===================[ IMU Sensor Data ]======================
float Gxyz[3], Axyz[3], Mxyz[3];
float Axyz_filtered[3];  
float Araw[3];
float Araw_filtered[3];  
float magFieldStrength, totalAcceleration = 0.0f;

// ===================[ FLAGS ]======================
int errorManager[10] = {-1};
// 
// ===================[ Function Declarations ]================
void init_IMU();
void init_General();
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
inline void vector_normalize(float v[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void DebugPrint();
void handleFlags(); 

// ===================[ Setup ]===============================
void setup() {
  Serial.begin(921600);
  while (!Serial); // Wait for connection

  init_IMU();
  init_General();
  

  last_update = last_print = micros();
}

// ===================[ Main Loop ]============================

void loop() {
  // ===================[ Timing Start ]===================
  uint32_t loopStartMicros = micros();

  // ===================[ Main Processing ]================
  getAttitude();  // Reads IMU, applies filters, prints data

  // ===================[ Loop Rate Enforcement ]==========
  while ((micros() - loopStartMicros) < LOOP_US);  // Busy-wait to maintain loop rate

  // ===================[ Diagnostics ]====================
  // Measure total loop time including enforced delay
  uint32_t totalLoopTime = micros() - loopStartMicros;
  loop_frequency_hz = 1e6f / (float)totalLoopTime;
}


// ===================[ Debug Print Function ]=================
void DebugPrint() {
  //Serial.print(Araw_filtered[0]); Serial.print(F(", "));
  //Serial.print(Araw_filtered[1]); Serial.print(F(", "));
  //Serial.print(Araw_filtered[2]); Serial.print(F(", "));

  Serial.print(filteredPitch,0);      Serial.print(F(", "));
  Serial.print(filteredRoll,0);       Serial.print(F(", "));
  Serial.print(yaw,0);                Serial.print(F(", "));
  Serial.print(loop_frequency_hz,0);  Serial.print(F(", "));
  Serial.println(totalAcceleration,1);
}

// ===================[ Handle ze Flags Function ]=================
void handleFlags() {
  // Check magnetometer strength
  if (magFieldStrength > 60.0f) {
    errorManager[1] = 1;
  } else {
    errorManager[1] = 0;
  }

  // Check acceleration and set KP accordingly
  if (totalAcceleration < 10.0f) {
    KP = 1.0f;
    errorManager[2] = 1;
  } else if (totalAcceleration < 12.0f) {
    KP = 1.0f;
    errorManager[2] = 1;
  } else {
    KP = 0.2f;
    errorManager[2] = 0;
  }
}


// ===================[ getATTI ]===================
void getAttitude() {
  imu.getAGMT();

  now = micros();
  delta_t = (now - last_update) * 1.0e-6f;
  last_update = now;

  get_scaled_IMU(Gxyz, Axyz, Mxyz);
  handleFlags();

  MahonyQuaternionUpdate(
    Axyz_filtered[0], Axyz_filtered[1], Axyz_filtered[2],
    Gxyz[0], Gxyz[1], Gxyz[2],
    Mxyz[0], Mxyz[1], Mxyz[2],
    delta_t
  );

  looptimer = (uint16_t)(1.0f / delta_t);
  DebugPrint();
}
// ===================[ IMU Initialization ]===================
void init_IMU() {
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.begin();

  imu.begin(CS_PIN, SPI);
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;
  myFSS.g = dps2000;
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  imu.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous);
}

// ===================[ General Initialization ]===================
void init_General() {
  output1.attach(9, 500, 2400);
  output2.attach(10, 500, 2400);
  output3.attach(11, 500, 2400);
  output4.attach(12, 500, 2400);

  output1.write(0);
  output2.write(45);
  output3.write(90);
  output4.write(170);
}

// ===================[ IMU Scaling & Calibration ]============
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  Gxyz[0] = GYRO_SCALE * (imu.agmt.gyr.axes.x - G_OFFSET[0]);
  Gxyz[1] = GYRO_SCALE * (imu.agmt.gyr.axes.y - G_OFFSET[1]);
  Gxyz[2] = GYRO_SCALE * (imu.agmt.gyr.axes.z - G_OFFSET[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;

  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  Araw[0] = Axyz[0]; Araw[1] = Axyz[1]; Araw[2] = Axyz[2];
  
  for (int i = 0; i < 3; i++) 
  {
    Araw[i] *= ACCEL_SCALE;
    Araw_filtered[i] = accel_lpf_alpha * Araw[i] + (1.0f - accel_lpf_alpha) * Araw_filtered[i];
    Axyz_filtered[i] = accel_lpf_alpha * Axyz[i] + (1.0f - accel_lpf_alpha) * Axyz_filtered[i];
  }

  vector_normalize(Axyz_filtered);

  float temp[3];
  for (byte i = 0; i < 3; i++) temp[i] = Mxyz[i] - M_B[i];

  Mxyz[0] = M_AINV[0][0] * temp[0] + M_AINV[0][1] * temp[1] + M_AINV[0][2] * temp[2];
  Mxyz[1] = M_AINV[1][0] * temp[0] + M_AINV[1][1] * temp[1] + M_AINV[1][2] * temp[2];
  Mxyz[2] = M_AINV[2][0] * temp[0] + M_AINV[2][1] * temp[1] + M_AINV[2][2] * temp[2];
  
  magFieldStrength = sqrtf(Mxyz[0]*Mxyz[0] + Mxyz[1]*Mxyz[1] + Mxyz[2]*Mxyz[2]);
  magFieldStrength = magFieldStrength*0.15;
  totalAcceleration = sqrtf(Araw_filtered[0] * Araw_filtered[0] + Araw_filtered[1] * Araw_filtered[1] + Araw_filtered[2] * Araw_filtered[2]
);

  vector_normalize(Mxyz);

  Mxyz[1] = -Mxyz[1];
  Mxyz[2] = -Mxyz[2];
}

// ===================[ Vector Normalization ]=================
inline void vector_normalize(float v[3]) {
  float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (norm < 1e-6f) return;
  float recip = 1.0f / norm;
  v[0] *= recip; v[1] *= recip; v[2] *= recip;
}

// ===================[ Mahony Filter ]========================
void MahonyQuaternionUpdate(
  float ax, float ay, float az,
  float gx, float gy, float gz,
  float mx, float my, float mz,
  float dt
) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm, hx, hy, hz, ex, ey, ez;

  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
  float q3q3 = q3 * q3, q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;

  norm = sqrtf(hx*hx + hy*hy + hz*hz);
  if (norm < 1e-6f) return;
  float recip_norm = 1.0f / norm;
  hx *= recip_norm; hy *= recip_norm; hz *= recip_norm;

  float ux = 2.0f * (q2q4 - q1q3);
  float uy = 2.0f * (q1q2 + q3q4);
  float uz = q1q1 - q2q2 - q3q3 + q4q4;

  float wx = 2.0f * (q2q3 + q1q4);
  float wy = q1q1 - q2q2 + q3q3 - q4q4;
  float wz = 2.0f * (q3q4 - q1q2);

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  gx += KP * ex; gy += KP * ey; gz += KP * ez;

  float half_dt = 0.5f * dt;
  gx *= half_dt; gy *= half_dt; gz *= half_dt;

  float qa = q1, qb = q2, qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  if (norm < 1e-6f) return;
  recip_norm = 1.0f / norm;
  q[0] = q1 * recip_norm; q[1] = q2 * recip_norm; q[2] = q3 * recip_norm; q[3] = q4 * recip_norm;

  roll  = atan2((q[0]*q[1] + q[2]*q[3]), 0.5f - (q[1]*q[1] + q[2]*q[2]));
  pitch = asin(2.0f * (q[0]*q[2] - q[1]*q[3]));
  yaw   = atan2((q[1]*q[2] + q[0]*q[3]), 0.5f - (q[2]*q[2] + q[3]*q[3]));

  yaw   *= 180.0f / M_PI;
  pitch *= -180.0f / M_PI;
  roll  *= 180.0f / M_PI;

  yaw = -(yaw + headingTrim);
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;

  filteredPitch += tilt_lpf_alpha * (pitch - filteredPitch);
  filteredRoll  += tilt_lpf_alpha * (roll  - filteredRoll);
}
