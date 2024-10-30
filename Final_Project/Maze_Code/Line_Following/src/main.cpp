#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define DT 100
// IMU
Adafruit_MPU6050 mpu;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

// const unsigned int ADC_1_CS = 2;
// const unsigned int ADC_2_CS = 17;

int adc1_buf[8];
int adc2_buf[8];

uint8_t lineArray[13]; 

// Encoders
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motors
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const unsigned int PWM_MAX = 255;
const int freq = 5000;
const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

// LED
const int ledChannel = 0;

// PID
const int base_pid = 80; // Base speed for robot
const float mid = 6;

float e;
float p_e;
float d_e;
float total_e;

// Assign values to the following feedback constants:
float Kp = 1;
float Kd = 1;
float Ki = 1;


/*
 *  Line sensor functions
 */
// void readADC() {
//   for (int i = 0; i < 8; i++) {
//     adc1_buf[i] = adc1.readADC(i);
//     adc2_buf[i] = adc2.readADC(i);
//   }
// }

void readADC() {
  for (int i = 0; i < 8; i++) {
    if (adc1.readADC(i) > 700){
      adc1_buf[i] = 0;
    }
    else{
      adc1_buf[i] = 1;
    }

    if (adc2.readADC(i) > 700){
      adc2_buf[i] = 0;
    }
    else{
      adc2_buf[i] = 1;
    }
  }
}

int32_t all_same(){
  if(adc1_buf[0] == 1){
    for(int i = 0; i < 8; i++) {
      if((i < 7 && adc1_buf[i] != 1) || (i < 6 && adc2_buf[i] != 1)){
        return 0;
      }
    }

    return 1;
  }

  else{
    for(int i = 0; i < 8; i++) {
      if((i < 7 && adc1_buf[i] != 0) || (i < 6 && adc2_buf[i] != 0)){
        return 0;
      }
    }

    return 2;
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot) 
void digitalConvert() {
  int threshold = 700;
  for (int i = 0; i < 7; i++) {
    if (adc1.readADC(i)>threshold) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i<6) {
      if (adc2.readADC(i)>threshold){
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }

    // print line sensor position
    for(int i = 0; i < 13; i++) {
      Serial.print(lineArray[i]); Serial.print(" ");
    }
  }
}

float getPosition(uint8_t lineArray[13]) { //passing lineArray values (13 bool values)
    int count = 0;
    float sum = 0;
    for (int i = 0; i < 13; i++) {
        if (lineArray[i] == 1) {
          sum += i;  
          count++;         
        }
    }
    if (count == 0) {
        return 6.0;
    }
    return sum/count;
}


/*
 *  Movement functions
 */

void move(int left, int right) {
  M1_forward(left);
  M2_forward(right);
}

int constrain(int value) {
  if (value > 255) {
    return 255;
  } else if (value < -255) {
    return -255;
  }
}

void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void M1_backward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_backward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void turnCorner(bool clockwise, int rotation_pwm) {
  /* 
   * Use the encoder readings to turn the robot 90 degrees clockwise or 
   * counterclockwise depending on the argument. You can calculate when the 
   * robot has turned 90 degrees using either the IMU or the encoders + wheel measurements
   */
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angle_traveled = 0;
  int rotation_speed = rotation_pwm;
  float DEG_90 = 1.25; // 90 degrees in radians
  

  if (clockwise) {
    M1_forward(rotation_speed);
    M2_backward(rotation_speed);
  } else {
    M1_backward(rotation_speed);
    M2_forward(rotation_speed);
  }

  while ((clockwise && angle_traveled > -DEG_90) || (!clockwise && angle_traveled < DEG_90)) {
    mpu.getEvent(&a, &g, &temp);
    angle_traveled += g.gyro.z * 0.01; // 0.01 is the time interval in seconds
    Serial.print("Angle traveled: ");
    Serial.println(angle_traveled);
    delay(10);
  }

  // Stop the robot
  M1_stop();
  M2_stop();

  delay(10000);
}

void printADC(){
  for (int i = 0; i < 8; i++) {
    if (i<7) {
      Serial.print(adc1_buf[i]); Serial.print("\t");
    }

    if (i<6) {
      Serial.print(adc2_buf[i]); Serial.print("\t");
    }
  }
  Serial.println("");
}

/*
 *  setup and loop
 */
void setup() {
  Serial.begin(115200);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

  M1_stop();
  M2_stop();

  // IMU Stop

  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  pinMode(ADC_1_CS, OUTPUT);
  pinMode(ADC_2_CS, OUTPUT);

  digitalWrite(ADC_1_CS, HIGH); // Without this the ADC's write
  digitalWrite(ADC_2_CS, HIGH); // to the SPI bus while the nRF24 is!!!!

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  delay(100);
}

void loop() {

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  while(true) {
    int u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    // printADC();
    digitalConvert();
    Serial.println("forward");

    pos = getPosition(lineArray); //passing lineArray to function which contains 13 boolean values
    Serial.println("Pos is "); Serial.println(pos);
    delay(1000);
    
    // Define the PID errors
    e = 6 - pos;
    d_e = (e - p_e) / DT;
    total_e += e*DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    int base_pwm = 110;
    u = Kp * e + Kd * d_e + Ki * 1; //need to integrate e

    leftWheelPWM = constrain(base_pwm - u);
    rightWheelPWM = constrain(base_pwm + u);
    
    move(leftWheelPWM, rightWheelPWM);


    // Check for corners
    int same = all_same();
    Serial.print("same: ");
    Serial.println(same);
    if(same > 0) {
      /* if all same indicates all white, then turn right
       * else back up until it detects both black and white
       * then if there is white on the right, turn right
       * else if there is white on the left, turn left
      */
      M1_stop();
      M2_stop();

      if(same == 1){
        M1_forward(base_pwm);
        M2_forward(base_pwm);
        delay(500);
        M1_stop();
        M2_stop();
        delay(3000);
        turnCorner(1,base_pwm);
        Serial.println("right");
        delay(1000);
        M1_stop();
        M2_stop();
        delay(3000);
        M1_forward(base_pwm);
        M2_forward(base_pwm);
        delay(1000);
        M1_stop();
        M2_stop();
        delay(3000);
      }

      else{
        while(all_same() != 0){
          Serial.println("back");
          readADC();
          printADC();
          M1_backward(base_pwm);
          M2_backward(base_pwm);
          delay(1000);
          M1_stop();
          M2_stop();
          delay(3000);
        }

        Serial.println("turn");
        readADC();
        printADC();

        int turn = 1;
        for(int i = 0; i < 3; i++){
          if(adc1_buf[i] == 1 || adc2_buf[i] == 1){
            turn = 0;
          }
        }
        Serial.print("turn: ");
        Serial.println(turn);

        if(turn == 0){
          turnCorner(0,base_pwm);
          Serial.println("right");
          M1_backward(rightWheelPWM);
          M2_forward(leftWheelPWM);
          delay(1000);
          M1_stop();
          M2_stop();
          delay(3000);
        }

        else{
          turnCorner(1,base_pwm);
          Serial.println("left");
          rightWheelPWM = base_pwm + u;
          leftWheelPWM = base_pwm - u;
          M1_forward(rightWheelPWM);
          M2_backward(leftWheelPWM);
          delay(1000);
          M1_stop();
          M2_stop();
          delay(3000);
        }
      }
    }
    delay(DT);
  }
}
