//AXES: ROLL: RIGHTPOS ; PITCH: UPPOS
//V0.4 | Early Access | Now with fun goodies: Checksum, high-speed UART, extensive testing (still not safety-rated!)
//TODO | Add Error out via UART (for AHRS health)
#include <Wire.h>
#include "ICM_20948.h"
#include <Servo.h>
#include "SparkFun_External_EEPROM.h"

// ===================[ Constants & Config ]===================
#define GYRO_SCALE ((M_PI / 180.0f) * 0.06106870229f) // For 2000dps setting
#define ACCEL_SCALE (0.00119710083f) // For 4G RANGE
#define SPI_PORT SPI
#define CS_PIN 5

// ===================[ Global Objects ]=======================
ICM_20948_SPI imu;
ExternalEEPROM mem1;
Servo output1, output2, output3, output4;

// ===================[ EEPROM Data ]=================
int loc1;

// ===================[ IMU Calibration Data ]=================
float KP = 5.0f;
float headingTrim = -10.0f;
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
const float tilt_lpf_alpha = 0.5f;
const float accel_lpf_alpha= 0.5f;

// ===================[ Timing Variables ]=====================
uint32_t now = 0, last_AHRSUpdate = 0, last_RPIDUpdate, last_PPIDUpdate = 0;
uint16_t looptimer; 
float delta_t = 0.0f;
uint32_t loop_start = 0, loop_end = 0;
float loop_duration_ms, loop_frequency_hz = 0.0f;
int LOOP_HZ = 1500;
int LOOP_US = 1000000 / LOOP_HZ;
int boot_Seconds = 0;

// ===================[ PID Gains ]===================
float Kp_roll = 1.0f, Ki_roll = 0.5f, Kd_roll = 0.1f;
float Kp_pitch = 1.0f, Ki_pitch = 0.5f, Kd_pitch = 0.1f;

// ===================[ Internal PID State ]===================
float integral_roll = 0.0f, lastError_roll = 0.0f;
float integral_pitch = 0.0f, lastError_pitch = 0.0f;
float rollPIDOutput, pitchPIDOutput, yawPIDOutput = 0;
float hz_roll_debug = 0.0f;
float hz_pitch_debug = 0.0f;
float thrust = 1;
float motor1, motor2, motor3, motor4 = 0;

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
void handle_AHRS_Flags(); 
void handle_GEN_OPSFlags(); 
void getAttitude();
void getPIDUpdate();
void sendControlOut();
void sendUARTOut();


// ===================[ Setup ]===============================
void setup() {
  init_IMU();
  init_General();
}

// ===================[ Main Loop ]============================
void loop() {
  uint32_t loopStartMicros = micros();
  boot_Seconds = millis() / 1000;

  static uint8_t state = 0;
  static uint8_t uartCounter = 0;

  switch (state) {
    case 0:
      getAttitude();
      break;

    case 1:
      getPIDUpdate();
      break;

    case 2:
      sendControlOut();
      break;

    case 3:
      if (++uartCounter >= 5) {
        sendUARTOut();
        uartCounter = 0;
      }
      break;

    default:
      break; 
  }

  state = (state + 1) % 4; // Only 0 to 3 now

  while ((micros() - loopStartMicros) < LOOP_US);
  uint32_t totalLoopTime = micros() - loopStartMicros;
  loop_frequency_hz = 1e6f / (float)totalLoopTime;
  rp2040.wdt_reset();
}



// ===================[ Handle ze Flags Function ]=================
void handle_AHRS_Flags() {
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


void handle_GEN_OPSFlags()
{

}



// ===================[ getATTI ]===================
void getAttitude() {
  imu.getAGMT();

  now = micros();
  delta_t = (now - last_AHRSUpdate) * 1.0e-6f;
  last_AHRSUpdate = now;

  get_scaled_IMU(Gxyz, Axyz, Mxyz);
  handle_AHRS_Flags();

  MahonyQuaternionUpdate(Axyz_filtered[0], Axyz_filtered[1], Axyz_filtered[2],Gxyz[0], Gxyz[1], Gxyz[2],Mxyz[0], Mxyz[1], Mxyz[2], delta_t);

  looptimer = (uint16_t)(1.0f / delta_t);
}


void getPIDUpdate() {
  rollPIDOutput  = computeRollPID(filteredRoll);
  pitchPIDOutput = computePitchPID(filteredPitch);
  yawPIDOutput   = 0;
}

void sendControlOut() { 
  motor1 = 180.0f*((thrust*0.7) - (rollPIDOutput*0.1) + (pitchPIDOutput*0.1)); //FR
  motor2 = 180.0f*((thrust*0.7) - (rollPIDOutput*0.1) - (pitchPIDOutput*0.1)); //RR
  motor3 = 180.0f*((thrust*0.7) + (rollPIDOutput*0.1) - (pitchPIDOutput*0.1)); //RL
  motor4 = 180.0f*((thrust*0.7) + (rollPIDOutput*0.1) + (pitchPIDOutput*0.1)); //FL

  output1.write(motor1);
  output2.write(motor2);
  output3.write(motor3);
  output4.write(motor4);
}

void sendUARTOut() {
  
int P_int = round(filteredPitch);  
int R_int = round(filteredRoll);  
int Y_int = round(yaw);  
int LF_int= round(loop_frequency_hz);
int CS_int = P_int + R_int + Y_int + LF_int;

  Serial1.print("$");
  Serial1.print(P_int);      Serial1.print(",");
  Serial1.print(R_int);      Serial1.print(",");
  Serial1.print(Y_int);      Serial1.print(",");
  Serial1.print(LF_int);     Serial1.print(",");
  Serial1.print(CS_int);     Serial1.print('\n'); 
  

}

// ===================[ PID Function ]===================
float computeRollPID(float input_roll) {
  unsigned long now = micros();
  float dt = (now - last_RPIDUpdate) / 1e6f;
  hz_roll_debug   = 1/dt;
  last_RPIDUpdate = now;

  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  float error = -input_roll;  // Setpoint = 0
  float derivative = (error - lastError_roll) / dt;

  // Provisional output (before updating integral)
  float provisional_output = Kp_roll * error + Ki_roll * integral_roll + Kd_roll * derivative;

  // Only integrate if output is not saturated
  if (provisional_output > -1.0f && provisional_output < 1.0f) {
    integral_roll += error * dt;
  }

  // Final output with updated integral
  float output = Kp_roll * error + Ki_roll * integral_roll + Kd_roll * derivative;
  lastError_roll = error;

  // Clamp to [-1, 1]
  if (output > 1.0f) output = 1.0f;
  else if (output < -1.0f) output = -1.0f;

  return output;
}

float computePitchPID(float input_pitch) {
  unsigned long now = micros();
  float dt = (now - last_PPIDUpdate) / 1e6f;
  hz_pitch_debug  = 1/dt;
  last_PPIDUpdate = now;

  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  float error = -input_pitch;  // Setpoint = 0
  float derivative = (error - lastError_pitch) / dt;

  // Provisional output (before updating integral)
  float provisional_output = Kp_pitch * error + Ki_pitch * integral_pitch + Kd_pitch * derivative;

  // Only integrate if output is not saturated
  if (provisional_output > -1.0f && provisional_output < 1.0f) {
    integral_pitch += error * dt;
  }

  // Final output with updated integral
  float output = Kp_pitch * error + Ki_pitch * integral_pitch + Kd_pitch * derivative;
  lastError_pitch = error;

  // Clamp to [-1, 1]
  if (output > 1.0f) output = 1.0f;
  else if (output < -1.0f) output = -1.0f;

  return output;
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
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.setClock(400000); 
  //Wire1.begin();
  mem1.setMemoryType(32);
  //mem1.begin(0b1010111 , Wire1);
    loc1 = 0;
  //mem1.put(10, 3444466);
  //mem1.get(10, loc1);
  output1.attach(9,  500, 2400);
  output2.attach(10, 500, 2400);
  //output3.attach(11, 500, 2400);
  //output4.attach(12, 500, 2400);

  output1.write(0);
  output2.write(0);
  //output3.write(0);
  //output4.write(0);

  Serial1.setTX(12);
  Serial1.begin(921600);
  rp2040.wdt_begin(100);
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
