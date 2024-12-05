#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define RIGHT true
#define LEFT false
#define ALL_WHITE 1
#define ALL_BLACK 2
#define NOT_ALL_SAME 0

#define DEFAULT_PWM 100

// States
#define DEFAULT_STATE 0
#define SQUARE_STATE 1
#define DOTTED_STATE 2
#define GRID_STATE 3

#define DT 0.05

// IMU
Adafruit_MPU6050 mpu;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

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
float e;
float p_e;
float d_e;
float total_e;

// Assign values to the following feedback constants:
float Kp = 1;
float Kd = 0.35;
float Ki = 0;

const float mid = 6;

int u;

int pidRight;
int pidLeft;

float pos;
float first_pos;
int same; 
int turn;
int STATE;

/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {
    if (adc1.readADC(i) > 690){
      adc1_buf[i] = 0;
    }
    else{
      adc1_buf[i] = 1;
    }

    if (adc2.readADC(i) > 690){
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
        return NOT_ALL_SAME;
      }
    }

    return ALL_WHITE;
  }

  else{
    for(int i = 0; i < 8; i++) {
      if((i < 7 && adc1_buf[i] != 0) || (i < 6 && adc2_buf[i] != 0)){
        return NOT_ALL_SAME;
      }
    }

    return ALL_BLACK;
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot) 
void digitalConvert() {
  for (int i = 0; i < 7; i++) {
    if (adc1_buf[i] == 0) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i < 6) {
      if(adc2_buf[i] == 0){
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }

    // // print line sensor position
    // for(int i = 0; i < 13; i++) {
    //   Serial.print(lineArray[i]); Serial.print(" ");
    // }
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
void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
  //Serial.println("I got to M1FWD ");

}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
  //Serial.println("I got to M2FWD ");
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

int constrain_pwm(int pwm) {
  if (pwm > PWM_MAX) {
    return PWM_MAX;
  } else if (pwm < 0) {
    return 0;
  } else {
    return pwm;
  }
}

void inch_forward() {
  M1_forward(DEFAULT_PWM);
  M2_forward(DEFAULT_PWM);
  delay(30);
  M1_stop();
  M2_stop();
}

void turnCorner(bool right, int right_wheel, int left_wheel) {
  if (right) {
    M1_forward(left_wheel);
    M2_backward(right_wheel);
    delay(300);
  } else {
    M1_backward(left_wheel);
    M2_forward(right_wheel);
    delay(300);
  }

  // Stop the robot
  M1_stop();
  M2_stop();

  delay(100);
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

  turn = LEFT;
  STATE = DEFAULT_STATE;

  delay(100);
}

void loop() {

  // Encoder enc1(M1_ENC_A, M1_ENC_B);
  // Encoder enc2(M2_ENC_A, M2_ENC_B);

  readADC();
  digitalConvert();
  Serial.println("forward");

  pos = getPosition(lineArray); //passing lineArray to function which contains 13 boolean values
  first_pos = pos;
  Serial.println("Pos is "); Serial.println(pos);

  if (STATE == DEFAULT_STATE) {
    // Define the PID errors
    e = 6 - pos;
    d_e = (e - p_e) / DT;
    total_e += e*DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e; //need to integrate e

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    pidLeft = constrain_pwm(90 + u);
    pidRight = constrain_pwm(90 - u);

    M1_forward(pidLeft);
    M2_forward(pidRight);

    // TURNING LOGIC
    if (lineArray[0] == 1) {
      turn = RIGHT;
    } else if (lineArray[12] == 1) {
      turn = LEFT;
    }
  } else if (STATE == SQUARE_STATE) {
    // Define the PID errors
    e = 9 - pos;
    d_e = (e - p_e) / DT;
    total_e += e*DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e; //need to integrate e

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    pidLeft = constrain_pwm(90 + u);
    pidRight = constrain_pwm(90 - u);

    M1_forward(pidLeft);
    M2_forward(pidRight);
  }

  same = all_same();
  // Serial.print("same: ");
  // Serial.println(same);
  if (STATE == DEFAULT_STATE) {
    if (same == ALL_WHITE) { // AT SQUARE
      inch_forward();

      // turn right to trace square
      turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

      STATE = SQUARE_STATE;

      delay(100);
    } else if (same == ALL_BLACK) { // TURN CORNER
      Serial.print("turn: ");
      Serial.println(turn == RIGHT ? "right" : "left");

      delay(100);

      turnCorner(turn, DEFAULT_PWM, DEFAULT_PWM);
    }
  } else if (STATE == SQUARE_STATE) {
    if (same == ALL_WHITE) { // time to exit square
      inch_forward();

      // turn right to leave square
      turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

      STATE = DEFAULT_STATE;
    } else if (same == ALL_BLACK) { // trace corner in square
      inch_forward();

      // turn left to trace square
      delay(100);
      turnCorner(LEFT, DEFAULT_PWM, DEFAULT_PWM);
    }
  }

  delay(50);



}
