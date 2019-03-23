#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>

#define MAX_THROTTLE 50
#define MAX_STEERING 150
#define MAX_TARGET_ANGLE 12

// Default control terms
#define KP 0.19
#define KD 28
#define KP_THROTTLE 0.07
#define KI_THROTTLE 0.04

#define MAX_CONTROL_OUTPUT 50

#define ZERO_SPEED 65535
#define MAX_ACCEL 7

#define MICROSTEPPING 8

#define I2C_SPEED 400000L
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ITERM_MAX_ERROR 25 // Iterm windup constants for PI control //40
#define ITERM_MAX 8000

#define CALIBRATION_TIME 5

int MOTOR1_STEP_PIN = 6;
int MOTOR2_STEP_PIN = 12;
int MOTOR1_DIR_PIN = 8;
int MOTOR2_DIR_PIN = 13;

// Components
MPU6050 mpu;

int16_t motor1;
int16_t motor2;

//Control System vars
uint8_t mode;

long timer_old;
long timer_value;
int debug_counter;
float debugVariable;
float dt;

//Robot Control vars
int16_t speed_M1, speed_M2; // Actual speed of motors
int8_t dir_M1, dir_M2;      // Actual direction of steppers motors
uint16_t period_M1, period_M2;
int16_t actual_robot_speed; // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

float angle_adjusted;
float angle_adjusted_Old;
float Kp = KP;
float Kd = KD;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;

float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;

// DMP FUNCTIONS
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

float dmpGetPhi()
{
  mpu.getFIFOBytes(fifoBuffer, 16);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();
  return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
}

// COMPENSATOR
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;
  Serial.println("Stability Control output: " + String(Kd * (error - PID_errorOld)));

  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;
  setPointOld = setPoint;
  return (output);
}

float speedPIControl(float DT, float input, float setPoint, float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  Serial.println("PID Error Output: " + String(PID_errorSum));

  output = Kp * error + Ki * PID_errorSum * DT * 0.001;
  return (output);
}

void delay_1us()
{
  __asm__ __volatile__(
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop");
}

ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0)
    return;
  if (dir_M1 > 0)
  {
    digitalWrite(MOTOR1_DIR_PIN, HIGH); // DIR Motors (Forward for Motor1)
  }
  else
  {
    digitalWrite(MOTOR1_DIR_PIN, LOW); // DIR Motors (Revers for Motor1)
  }
  OCR1A = OCR1A + period_M1;
  __asm__ __volatile__(
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop");
  digitalWrite(MOTOR1_STEP_PIN, HIGH); // STEP MOTOR 1
  delay_1us();
  digitalWrite(MOTOR1_STEP_PIN, LOW);
}

ISR(TIMER1_COMPB_vect)
{
  if (dir_M2 == 0)
    return;
  if (dir_M2 > 0)
  {
    digitalWrite(MOTOR2_DIR_PIN, LOW); // DIR Motors (Revers for Motor2)
  }
  else
  {
    digitalWrite(MOTOR2_DIR_PIN, HIGH); // DIR Motors (Forward for Motor2)
  }
  OCR1B = OCR1B + period_M2;
  __asm__ __volatile__(
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop"
      "\n\t"
      "nop");
  digitalWrite(MOTOR2_STEP_PIN, HIGH); // STEP MOTOR 2
  delay_1us();
  digitalWrite(MOTOR2_STEP_PIN, LOW);
}

void init_motors()
{
  dir_M1 = 0;
  dir_M2 = 0;
  TCCR1A = 0;
  TCCR1B = B00000010;
  TCCR1C = 0;
  OCR1A = ZERO_SPEED;
  OCR1B = ZERO_SPEED;
  TCNT1 = 0;
  TIMSK1 = B00000110;
}

void setMotorSpeed(int16_t tspeed_M1, int16_t tspeed_M2)
{
  long timer_period_1, timer_period_2;
  int16_t speed_1, speed_2;
  int8_t dir_1, dir_2;

  if ((speed_M1 - tspeed_M1) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed_M1) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed_M1;

  if ((speed_M2 - tspeed_M2) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed_M2) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed_M2;

#if MICROSTEPPING == 16
  speed_1 = speed_M1 * 46; // Adjust factor from control output speed to real motor speed in steps/second
  speed_2 = speed_M2 * 46;
#else
  speed_1 = speed_M1 * 23; // 1/8 Microstepping
  speed_2 = speed_M2 * 23;
#endif
  if (speed_1 == 0)
  {
    timer_period_1 = ZERO_SPEED;
    dir_1 = 0;
  }
  else if (speed_1 > 0)
  {
    timer_period_1 = 2000000 / speed_1; // 2Mhz timer
    dir_1 = 1;
  }
  else if (speed_1 < 0)
  {
    timer_period_1 = 2000000 / -speed_1;
    dir_1 = -1;
  }
  if (timer_period_1 > 65535)
    timer_period_1 = ZERO_SPEED;
  if (speed_2 == 0)
  {
    timer_period_2 = ZERO_SPEED;
    dir_2 = 0;
  }
  else if (speed_2 > 0)
  {
    timer_period_2 = 2000000 / speed_2; // 2Mhz timer
    dir_2 = 1;
  }
  else if (speed_2 < 0)
  {
    timer_period_2 = 2000000 / -speed_2;
    dir_2 = -1;
  }
  if (timer_period_2 > 65535)
    timer_period_2 = ZERO_SPEED;
  dir_M1 = dir_1;
  dir_M2 = dir_2;
  period_M1 = timer_period_1;
  period_M2 = timer_period_2;
}

void SIG_SETUP_START()
{
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void SIG_INIT_ERROR()
{
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void setup()
{
  pinMode(MOTOR1_STEP_PIN, OUTPUT); // STEP MOTOR 1
  pinMode(MOTOR1_DIR_PIN, OUTPUT);  // DIR MOTOR 1
  pinMode(MOTOR2_STEP_PIN, OUTPUT); // STEP MOTOR 2
  pinMode(MOTOR2_DIR_PIN, OUTPUT);  // DIR MOTOR 2

  Serial.begin(115200); // Serial output to console

  SIG_SETUP_START();

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();
  // I2C 400Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  TWCR = 1 << TWEN;

  delay(1000);

  Serial.println("I2C INITIALIZATION");
  Serial.println("\tBEGIN");
  // Manual MPU initialization... accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10); //10,20,42,98,188  // Default factor for BROBOT:10
  mpu.setRate(4);                      // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  Serial.println("\tDONE");

  delay(500);

  Serial.println("DMP INITIALIZATION");
  Serial.println("\tBEGIN");
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    Serial.println("DMP STARTED");
  }
  else
  {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.println("DMP ERROR (" + String(devStatus) + ")");

    SIG_INIT_ERROR();
  }
  Serial.println("\tDONE");

  delay(500);

  Serial.println("GYRO CALIBRATION");
  Serial.println("\tBEGIN");
  for (int count = 0; count < CALIBRATION_TIME; count++)
    ;
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  Serial.println("\tDONE");

  for (int i = 0; i < 2; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }

  // Verify connection
  Serial.println("CONNECTION TEST");
  if (mpu.testConnection())
  {
    Serial.println("\tMPU6050: SUCCESS");
  }
  else
  {
    Serial.println("\tMPU6050: ERROR");
  }

  timer_old = millis();

  Serial.println("STEPPER MOTOR INITIALIZATION");
  Serial.println("\tBEGIN");
  init_motors();
  Serial.println("\tDONE");

  dmpSetSensorFusionAccelGain(0x20);
  delay(200);

  Serial.println("STEPPER MOTOR TEST");
  Serial.println("\tBEGIN");
  for (int k = 0; k < 5; k++)
  {
    setMotorSpeed(5, -5);
    delay(200);
    setMotorSpeed(-5, 5);
    delay(200);
  }
  Serial.println("\tDONE");

  Serial.println("START");
  mpu.resetFIFO();
  timer_old = millis();
  mode = 0;
}

// MAIN LOOP
void loop()
{
  digitalWrite(LED_BUILTIN, LOW);

  debug_counter++;
  timer_value = millis();

  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 18)
  {
    if (fifoCount > 18)
    {
      mpu.resetFIFO();
      return;
    }

    dt = (timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    angle_adjusted = dmpGetPhi();

    Serial.print("Throttle: " + String(throttle) + " \n");
    Serial.print("Angle: " + String(angle_adjusted) + " \n");

    mpu.resetFIFO();

    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2;

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0;                    // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;                       // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05; // low pass filter on estimated speed

    // SPEED CONTROL: This is a PI controller.
    //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

    Serial.print("Estimate filtered speed: " + String(estimated_speed_filtered) + " \n");
    Serial.print("Target Angle: " + String(target_angle) + " \n");

    // Stability control: This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly on the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    Serial.print("Motor 1: " + String(motor1) + " \n");
    Serial.print("Motor 2: " + String(motor1) + " \n");

    if ((angle_adjusted < 76) && (angle_adjusted > -76)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      setMotorSpeed(motor1, motor2);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      Serial.println("--------------- ROBOT OUT OF BOUNDS ---------------");
      setMotorSpeed(0, 0);
      digitalWrite(LED_BUILTIN, HIGH);
      PID_errorSum = 0; // Reset PID I term
    }
  }
}