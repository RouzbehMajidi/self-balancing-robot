#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>

// Constants
#define ZERO_SPEED 65535
#define I2C_SPEED 400000L
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489
#define GYRO_CALIBRATION_TIME 20000 //ms

// Control
#define TARGET_ANGLE 0.0
#define ANGLE_TOLERANCE 0.1
#define MAX_ANGLE 50
#define L1_ANGLE_RANGE 10

#define MICROSTEPPING 16

#define MAX_ACCEL 7
#define MAX_CONTROL_OUTPUT_L1 100
#define MAX_CONTROL_OUTPUT_L2 200

// Compensator
#define ALPHA 0.99
#define KP 170
#define KI 1
#define KD 2800

#define ITERM_MAX_ERROR 100
#define ITERM_MAX 8000

// Motors
#define MOTOR_EN_PIN 4

#define MOTOR1_STEP_PIN 6
#define MOTOR1_DIR_PIN 8

#define MOTOR2_STEP_PIN 12
#define MOTOR2_DIR_PIN 11

// Components
MPU6050 mpu;

// TIMER
long timer_old;
long timer_value;
float dt;

// MOTORS
int16_t speed_M1, speed_M2; // Actual speed of motors
int8_t dir_M1, dir_M2;      // Actual direction of steppers motors
uint16_t period_M1, period_M2;

// IMU 
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

// PID
float KP_current = KP;
float KI_current = KI;
float KD_current = KD;

float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;

// CONTROL
float angle_adjusted;
float angle_adjusted_Old;

float target_angle;
float control_output;


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
  float current_angle = (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
  float filtered_angle = ALPHA * current_angle + (1 - ALPHA) * angle_adjusted_Old;
  return filtered_angle;
}

float pid(float DT, float input, float setPoint)
{
  float error;
  float output;

  if(input < ANGLE_TOLERANCE && input > -ANGLE_TOLERANCE){
    error = 0;
  }else{
    error = setPoint - input;
  }

  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  output = KP_current * error + ((KD_current * (setPoint - setPointOld) - KD_current * (input - PID_errorOld2)) / DT) + KI_current * PID_errorSum * DT * 0.001 ;

  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;
  setPointOld = setPoint;

  return output;
}

void pid_reset(){
  PID_errorSum = 0;
  PID_errorOld = 0;
  PID_errorOld2 = 0;
}


void pid_update(){
  if (Serial.available() > 0){
    digitalWrite(MOTOR_EN_PIN, HIGH);
    Serial.println("PID UPDATE RECEIVED");
    KP_current = Serial.parseFloat();
    KI_current = Serial.parseFloat();
    KD_current = Serial.parseFloat();
    pid_reset();
    Serial.print("\tKP:");
    Serial.println(KP_current);
    Serial.print("\tKI:");
    Serial.println(KI_current);
    Serial.print("\tKD:");
    Serial.println(KD_current);
    digitalWrite(MOTOR_EN_PIN, LOW);
  }
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

void setMotorSpeed(float controlInput)
{
  long timer_period_1, timer_period_2;
  int16_t speed_1, speed_2;
  int8_t dir_1, dir_2;

  if ((speed_M1 - controlInput) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - controlInput) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = controlInput;

  if ((speed_M2 - controlInput) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - controlInput) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = controlInput;

#if MICROSTEPPING == 16 // 1/16
  speed_1 = speed_M1 * 46; // Adjust factor from control output speed to real motor speed in steps/second
  speed_2 = speed_M2 * 46;
#elif MICROSTEPPING == 8 // 1/8
  speed_1 = speed_M1 * 23;
  speed_2 = speed_M2 * 23;
#elif MICROSTEPPING == 1 // Full
  speed_1 = speed_M1 * 23;
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
  pinMode(MOTOR_EN_PIN, OUTPUT); // EN MOTOR 1/2
  
  digitalWrite(MOTOR_EN_PIN, HIGH);

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
    Serial.print("DMP ERROR (");
    Serial.print(devStatus);
    Serial.println(")");

    SIG_INIT_ERROR();
  }
  Serial.println("\tDONE");

  delay(500);

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

  Serial.println("GYRO CALIBRATION");
  Serial.println("\tSTART");
  delay(GYRO_CALIBRATION_TIME);
  Serial.println("\tDONE");

  timer_old = millis();

  Serial.println("STEPPER MOTOR INITIALIZATION");
  Serial.println("\tBEGIN");
  init_motors();
  Serial.println("\tDONE");

  dmpSetSensorFusionAccelGain(0x20);
  delay(200);

  Serial.println("START");
  mpu.resetFIFO();
  timer_old = millis();
  digitalWrite(MOTOR_EN_PIN, LOW);
}

// MAIN LOOP
void loop()
{
  digitalWrite(LED_BUILTIN, LOW);

  pid_update();

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

    mpu.resetFIFO();

    control_output += pid(dt, angle_adjusted, TARGET_ANGLE);
    if(angle_adjusted < L1_ANGLE_RANGE && angle_adjusted > -L1_ANGLE_RANGE){
      control_output = constrain(control_output, -MAX_CONTROL_OUTPUT_L1, MAX_CONTROL_OUTPUT_L1);
    }else{
      control_output = constrain(control_output, -MAX_CONTROL_OUTPUT_L2, MAX_CONTROL_OUTPUT_L2);
    }


    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(control_output);

    if ((angle_adjusted < MAX_ANGLE) && (angle_adjusted > -MAX_ANGLE))
    {
      digitalWrite(MOTOR_EN_PIN, LOW);
      setMotorSpeed(control_output);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else 
    {
      Serial.println("--------------- ROBOT OUT OF BOUNDS ---------------");
      digitalWrite(MOTOR_EN_PIN, HIGH);
      setMotorSpeed(0);
      digitalWrite(LED_BUILTIN, HIGH);
      PID_errorSum = 0; // Reset PID I term
    }
  }
}

