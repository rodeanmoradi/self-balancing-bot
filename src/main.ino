#include <ArduinoEigenDense.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

using namespace Eigen;

MPU6050 mpu;

static unsigned long lastMicros;
static const int ena = 9;
static const int enb = 10;
static const int in1 = 7;
static const int in2 = 8;
static const int in3 = 12;
static const int in4 = 13;
static const float kP = 37.0;
static const float kI = 117.0;
static const float kD = 2.2;

// Hold robot at real balance point, read Kalman output from Serial
// Monitor, set this to that value. 0.0 is a starting guess
static const float DESIRED_ANGLE = 0.0;

float prevError = 0.0;
float integral = 0.0;
float dt = 0.01;

// State is [theta, b], where theta is the tilt angle and b is the gyro bias
Eigen::Vector2f x;
Eigen::Vector2f B;
float y;
float z;
Eigen::Matrix<float, 2, 2> phi;
Eigen::Matrix<float, 2, 1> K;
Eigen::Matrix<float, 1, 2> H;
Eigen::Matrix<float, 2, 2> P;
Eigen::Matrix<float, 2, 2> I;
Eigen::Matrix<float, 2, 2> Q;
float R;
float S;

float kalman(float accel, float gyro);
float pidController(float tiltAngle, float kP, float kI, float kD);

void setup() {

  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  lastMicros = micros();
  x(0) = 0.0;
  x(1) = 0.0;
  B(0) = dt;
  B(1) = 0.0;
  y = 0.0;
  z = 0.0;
  phi << 1.0, -dt,
         0,   1.0;
  K.setZero();
  H(0) = 1.0;
  H(1) = 0.0;
  P << 4.0, 0.0,
       0.0, 100.0;
  I.setIdentity();
  Q.setZero();
  Q << 0.001, 0.0,
       0.0,   0.03;
  R = 0.5;
  S = 0.0;

}

void loop() {

  unsigned long now = micros();
  dt = (now - lastMicros) * 1e-6;
  lastMicros = now;
  phi(0,1) = -dt;
  B(0) = dt;

  float aYRaw = mpu.getAccelerationY();
  float aZRaw = mpu.getAccelerationZ();
  float ay = aYRaw / 16384.0;
  float az = aZRaw / 16384.0;
  float accel = atan2(ay, az) * 180.0 / M_PI;
  float omegaRaw = mpu.getRotationX();
  float gyro = omegaRaw / 131.0;

  float tiltAngle = kalman(accel, gyro);
  float u_controller = pidController(tiltAngle, kP, kI, kD);
  //if(abs(u_controller) < 70.0) {
    //if(u_controller < 0.0) {
      //u_controller = -70.0;
    //}
    //else if(u_controller > 0.0) {
      //u_controller = 70.0;
    //}
  //}
  float uPWM = constrain(u_controller, -255.0, 255.0);

  //Serial.print(accel);
  //Serial.print(",");
  //Serial.println(tiltAngle);

  // 45 may be too high
  if (abs(tiltAngle) > 30) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enb, 0);
    integral = 0.0;
    prevError = 0.0;
  }
  else {
    if (u_controller > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, int(abs(uPWM)));
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, int(abs(uPWM)));
    }
    else if (u_controller < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, int(abs(uPWM)));
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, int(abs(uPWM)));
    }
  }

}

float kalman(float accel, float gyro) {

  z = accel;

  // Predict FIRST, then compute innovation.
  // Predict
  x = phi * x + B * gyro;
  P = phi * P * phi.transpose() + Q;

  // Innovation (after predict)
  y = z - (H * x)(0, 0);

  // Kalman gain
  S = (H * P * H.transpose())(0, 0) + R;
  K = (P * H.transpose()) / S;

  // Correct
  x = x + K * y;
  P = (I - K * H) * P;

  return x(0);

}

float pidController(float tiltAngle, float kP, float kI, float kD) {

  float e = DESIRED_ANGLE - tiltAngle;

  integral += e * dt;
  integral = constrain(integral, -75.0, 75.0);
  float derivative = (e - prevError) / dt;
  float u_controller = kP * e + kI * integral + kD * derivative;

  prevError = e;
  
  return u_controller;

}
