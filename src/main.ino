#include <ArduinoEigenDense.h>
#include <Wire.h>
#include <MPU6050.h>

using namespace Eigen;

MPU6050 mpu;

static unsigned long lastMicros;
static const int ena = 10;
static const int enb = 9;
static const int in1 = 8;
static const int in2 = 7;
static const int in3 = 13;
static const int in4 = 12;
static const float pi = 3.14159;
static const float kP = 8.1;
static const float kI = 0.0137;
static const float kD = 0.1;
float prevError = 0.0;
float integral = 0.0;
float dt = 0.01;
Eigen::Vector2f x;
Eigen::Vector2f y;
Eigen::Vector2f z;
Eigen::Matrix<float, 2, 2> phi;
Eigen::Matrix<float, 2, 2> K;
Eigen::Matrix<float, 2, 2> H;
Eigen::Matrix<float, 2, 2> P;
Eigen::Matrix<float, 2, 2> I;
Eigen::Matrix<float, 2, 2> Q;
Eigen::Matrix<float, 2, 2> R;
Eigen::Matrix<float, 2, 2> S;

float kalman(float accel, float gyro);
float pidController(float tiltAngle, float kP, float kI, float kD);


void setup() {

  Serial.begin(9600);
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
  // Angle
  x(0) = 0.0;
  // Angular velocity
  x(1) = 0.0;
  y(0) = 0.0;
  y(1) = 0.0;
  z(0) = 0.0;
  z(1) = 0.0;
  phi << 1, dt,
         0, 1;
  K.setZero();
  H << 1, 0,
       0, 1;
  P << 4, 0,
       0, 100;
  I.setIdentity();
  Q.setZero();
  Q << 0.001, 0,
       0,     0.03;
  R.setZero();
  R << 0.03, 0,
       0,    0.01;
  S.setZero();

}

void loop() {

  unsigned long now = micros();
  dt = (now - lastMicros) * 1e-6;
  lastMicros = now;
  phi(0,1) = dt;
  
  float aYRaw = mpu.getAccelerationY();
  float aZRaw = mpu.getAccelerationZ();
  float ay = aYRaw / 16384.0;
  float az = aZRaw / 16384.0;
  float accel = atan2(ay, az) * 180.0 / pi;
  float omegaRaw = mpu.getRotationX();
  float gyro = omegaRaw / 131.0;

  float tiltAngle = kalman(accel, gyro);
  float u = pidController(tiltAngle, kP, kI, kD);
  float uPWM = constrain(u, -255.0, 255.0);

  // Visualize Kalman
  Serial.print(accel);
  Serial.print(",");
  Serial.println(tiltAngle);

  // Visualize PID
  // Serial.println(uPWM);


  if (tiltAngle > 0) {

    // Move forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, abs(uPWM));
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enb, abs(uPWM));

  } 
  
  else if (tiltAngle < 0) {

    // Move backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, abs(uPWM));
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enb, abs(uPWM));

  }

}

// TODO: Add gyro bias
float kalman(float accel, float gyro) {

  z(0) = accel;
  z(1) = gyro;

  y = z - H * x;

  // Predict
  x = phi * x;
  P = phi * P * phi.transpose() + Q;

  // Compute Kalman gain
  S = H * P * H.transpose() + R;
  K = (P * H.transpose()) * S.inverse();

  // Correct
  x = x + K * y;

  // Update predicted error covariance
  P = (I - K * H) * P;

  float tiltAngle = x(0);
  return tiltAngle;

}

float pidController(float tiltAngle, float kP, float kI, float kD) {

  float desiredAngle = 0.0;

  // Calculate error
  float e = desiredAngle - tiltAngle;

  // Calculate integral term
  integral += e * dt;

  // Calculate derivative term
  float derivative = (e - prevError) / dt;

  // Calculate control input
  float u = kP * e + kI * integral + kD * derivative;

  prevError = e;

  return u;

}
