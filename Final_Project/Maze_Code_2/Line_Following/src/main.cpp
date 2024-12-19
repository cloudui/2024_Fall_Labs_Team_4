#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <ESP32Encoder.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>

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
const float Kp = 1.75;
const float Kd = 0.3;
const float Ki = 0.05;

const float mid = 6;

int u;

int pidRight;
int pidLeft;

float pos;
float first_pos;
int same;
int turn;
int STATE;
int SQUARE_COUNT;

int BLUE_COUNT;
int RED_COUNT;
int GREEN_COUNT;

int left_count = 0;
int right_count = 0;
int straight_count = 0;

String maxColor;

bool audioFlag = true;

ESP32Encoder enc1;
ESP32Encoder enc2;

/*
 *  WiFi code integration
 */
struct __attribute__((packed)) Data {
  int16_t seq;   // sequence number
  char text[50]; // text
};

// WiFi network credentials
// const char* ssid = "ParksideUnit521";
// const char* password = "BCQLMMDN";

// Server IP and port
// const char* host = "10.5.21.112";  // Replace with the IP address of server
const uint16_t port = 9500;

// Emily's House Settings
const char *host = "172.20.10.12"; // Replace with the IP address of server
const char *ssid = "iPhone (18)";
const char *password = "lillit12";

// Create a client
WiFiClient client;

int i_wifi = 1;

bool findGap() {
  // find gap of 2 or 3 black consecutive lines inbetween white ones
  int count = 0;
  for (int i = 1; i < 12; i++) {
    if (lineArray[i] == 0) {
      count++;
    } else {
      count = 0;
    }

    if (count >= 2) {
      return true;
    }
  }

  return false;
}

String commWithServer(const String &message) {
  if (client.connect(host, port)) {
    Serial.println("Connected to server");

    // Send the message
    client.println(message);
    Serial.println("Message sent: " + message);

    // Wait for a response
    String response = "";
    while (client.connected()) {
      if (client.available()) {
        response = client.readStringUntil('\n');
        break;
      }
    }
    // Close the connection
    client.stop();
    return response;
  } else {
    Serial.println("Connection to server failed");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i < 5) {
      Serial.println("Wi-Fi disconnected. Reconnecting...");
      WiFi.reconnect();
      i++;
      delay(500);
    }
    return "";
  }
}

/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {

    adc1_buf[i] = adc1.readADC(i) < 690 ? 1 : 0;
    adc2_buf[i] = adc2.readADC(i) < 690 ? 1 : 0;
  }
}

int32_t all_same() {
  if (adc1_buf[0] == 1) {
    for (int i = 0; i < 8; i++) {
      if ((i < 7 && adc1_buf[i] != 1) || (i < 6 && adc2_buf[i] != 1)) {
        return NOT_ALL_SAME;
      }
    }

    return ALL_WHITE;
  }

  else {
    for (int i = 0; i < 8; i++) {
      if ((i < 7 && adc1_buf[i] != 0) || (i < 6 && adc2_buf[i] != 0)) {
        return NOT_ALL_SAME;
      }
    }

    return ALL_BLACK;
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your
// robot)
void digitalConvert() {
  for (int i = 0; i < 7; i++) {
    lineArray[2 * i] = adc1_buf[i];
    // Serial.println(adc1_buf[i]);

    if (i < 6) {
      // Serial.println(adc2_buf[i]);
      lineArray[2 * i + 1] = adc2_buf[i];
    }

    // // print line sensor position
    // for(int i = 0; i < 13; i++) {
    //   Serial.print(lineArray[i]); Serial.print(" ");
    // }
  }
}

float getPosition(
    uint8_t lineArray[13]) { // passing lineArray values (13 bool values)
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
  return sum / count;
}

/*
 *  Movement functions
 */
void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
  // Serial.println("I got to M1FWD ");
}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
  // Serial.println("I got to M2FWD ");
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

void stop() {
  M1_stop();
  M2_stop();
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

void inch(int time, bool direction) {
  int pwm1 = DEFAULT_PWM; // Initial PWM for motor 1
  int pwm2 = DEFAULT_PWM; // Initial PWM for motor 2
  int error = 0;
  int lastError = 0;
  int integral = 0;
  int derivative = 0;
  float Kp = 1.5, Ki = 0.1, Kd = 0.05; // PID gains

  enc1.clearCount();
  enc2.clearCount();

  stop();
  delay(300);
  // while time elapsed < time
  int start_time = millis();

  while (millis() - start_time < time) {
    // Calculate errors
    error = abs(enc1.getCount()) - abs(enc2.getCount());

    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Adjust PWM values based on PID output
    int adjustment = Kp * error + Ki * integral + Kd * derivative;

    pwm1 = DEFAULT_PWM - adjustment; // Adjust motor 1
    pwm2 = DEFAULT_PWM + adjustment; // Adjust motor 2

    // Clamp PWM values to prevent overflow
    pwm1 = constrain(pwm1, 0, 255);
    pwm2 = constrain(pwm2, 0, 255);

    if (direction) {
      M1_forward(pwm1);
      M2_forward(pwm2);
    } else {
      M1_backward(pwm1);
      M2_backward(pwm2);
    }
  }

  stop();
}


void inch_forward(int time) {
  M1_forward(DEFAULT_PWM);
  M2_forward(DEFAULT_PWM);
  delay(time);
  M1_stop();
  M2_stop();
}

void inch_backward(int time) {
  M1_forward(DEFAULT_PWM);
  M2_forward(DEFAULT_PWM);
  delay(time);
  M1_stop();
  M2_stop();
}


void turnCorner(bool right) {
  int counts = 95;        // Target encoder count
  int pwm1 = DEFAULT_PWM; // Initial PWM for motor 1
  int pwm2 = DEFAULT_PWM; // Initial PWM for motor 2
  int error = 0;
  int lastError = 0;
  int integral = 0;
  int derivative = 0;
  float Kp = 1.5, Ki = 0.1, Kd = 0.05; // PID gains

  stop();
  delay(500);

  enc1.clearCount();
  enc2.clearCount();

  while (abs(enc1.getCount()) < counts && abs(enc2.getCount()) < counts) {
    // Calculate errors
    error = abs(enc1.getCount()) - abs(enc2.getCount());

    integral += error;
    derivative = error - lastError;
    lastError = error;

    // Adjust PWM values based on PID output
    int adjustment = Kp * error + Ki * integral + Kd * derivative;

    pwm1 = DEFAULT_PWM - adjustment; // Adjust motor 1
    pwm2 = DEFAULT_PWM + adjustment; // Adjust motor 2

    // Clamp PWM values to prevent overflow
    pwm1 = constrain(pwm1, 0, 255);
    pwm2 = constrain(pwm2, 0, 255);

    // Apply motor commands based on direction
    if (right) {
      M1_forward(pwm1);
      M2_backward(pwm2);
    } else {
      M1_backward(pwm1);
      M2_forward(pwm2);
    }

    delay(10); // Small delay to stabilize the loop
  }

  Serial.print("Enc1: ");
  Serial.println(enc1.getCount());
  Serial.print("Enc2: ");
  Serial.println(enc2.getCount());

  // Stop the robot
  stop();

  delay(100);
}

void printADC() {
  for (int i = 0; i < 8; i++) {
    if (i < 7) {
      Serial.print(adc1_buf[i]);
      Serial.print("\t");
    }

    if (i < 6) {
      Serial.print(adc2_buf[i]);
      Serial.print("\t");
    }
  }
  Serial.println("");
}

/*
 *  setup and loop
 */
void setup() {
  Serial.begin(115200);

  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

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

  stop();

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

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  ESP32Encoder::useInternalWeakPullResistors =
      puType::up;                          // Enable pull-up resistors
  enc1.attachHalfQuad(M1_ENC_A, M1_ENC_B); // Attach pins
  enc1.clearCount();                       // Reset encoder count

  enc2.attachHalfQuad(M2_ENC_B, M2_ENC_A); // Attach pins
  enc2.clearCount();                       // Reset encoder count

  turn = LEFT;
  STATE = GRID_STATE;

  SQUARE_COUNT = 5;

  BLUE_COUNT = 0;
  RED_COUNT = 100;
  GREEN_COUNT = 0;

  maxColor = "red";

  delay(100);
}

void loop() {
  readADC();
  digitalConvert();

  pos = getPosition(lineArray); // passing lineArray to function which contains
                                // 13 boolean values
  first_pos = pos;

  // for (int i = 0; i < 13; i++)
  // {
  //   Serial.print(lineArray[i]);
  //   Serial.print(" ");
  // }

  // Serial.println("");
  // delay(2000);
  // return;
  // Serial.println("Pos is "); Serial.println(pos);
  // return;
  // turnCorner(RIGHT);

  // delay(1000);
  // turnCorner(LEFT);

  // delay(1000);
  // return;

  // String response = commWithServer("direction");
  // if (!response.isEmpty() && response != "invalid") {
  //   Serial.println("Server says: " + response);
  //   // turnCorner(response == "right", DEFAULT_PWM, DEFAULT_PWM);
  // }

  // delay(500);
  // return;

  same = all_same();

  if (STATE == DEFAULT_STATE) {
    // Define the PID errors
    e = 6 - pos;
    d_e = (e - p_e) / DT;
    total_e += e * DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e; // need to integrate e

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    pidLeft = constrain_pwm(85 + u);
    pidRight = constrain_pwm(85 - u);

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
    total_e += e * DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e; // need to integrate e

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    pidLeft = constrain_pwm(85 + u);
    pidRight = constrain_pwm(85 - u);

    M1_forward(pidLeft);
    M2_forward(pidRight);
  } else if (STATE == DOTTED_STATE) {
    const float DKp = 4.0;
    const float DKd = 0.5;
    const float DKi = 0.1;

    if (same == ALL_BLACK) {
      M1_forward(80);
      M2_forward(80);
      p_e = 0;
    } else {
      // Define the PID errors
      e = 6 - pos;
      d_e = (e - p_e) / DT;
      total_e += e * DT;

      // Update the previous error
      p_e = e;

      // Implement PID control (include safeguards for when the PWM values go
      // below 0 or exceed maximum)
      u = DKp * e + DKd * d_e + DKi * total_e; // need to integrate e

      // Implement PID control (include safeguards for when the PWM values go
      // below 0 or exceed maximum)
      pidLeft = constrain_pwm(80 + u);
      pidRight = constrain_pwm(80 - u);

      M1_forward(pidLeft);
      M2_forward(pidRight);
    }
  } else if (STATE == GRID_STATE) {
    // Define the PID errors
    e = 6 - pos;
    d_e = (e - p_e) / DT;
    total_e += e * DT;

    // Update the previous error
    p_e = e;

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    u = Kp * e + Kd * d_e + Ki * total_e; // need to integrate e

    // Implement PID control (include safeguards for when the PWM values go
    // below 0 or exceed maximum)
    pidLeft = constrain_pwm(90 + u);
    pidRight = constrain_pwm(90 - u);

    M1_forward(pidLeft);
    M2_forward(pidRight);
  }

  // STATE TRANSITIONS
  // Serial.print("same: ");
  // Serial.println(same);
  if (STATE == DEFAULT_STATE) {
    if (!audioFlag && same == ALL_WHITE && SQUARE_COUNT == 2) { // WAIT FOR AUDIO
      stop();
      String message = "left";
      for (int i = 0; i < 10; i++) {
        message = commWithServer("direction");
        if (!message.isEmpty() && message != "invalid") {
          break;
        }
        delay(1000);
      }
      bool audioTurn = message == "right" ? RIGHT : LEFT;

      inch_forward(70);
      turnCorner(audioTurn);

      STATE = DEFAULT_STATE;
      audioFlag = true;
    } else if (same == ALL_WHITE) { // AT SQUARE
      stop();

      String message = "red";
      for (int i = 0; i < 10; i++) {
        message = commWithServer("circle");
        if (!message.isEmpty() && message != "invalid") {
          break;
        }
        delay(1000);
      }

      if (message == "red") {
        RED_COUNT++;
      } else if (message == "blue") {
        BLUE_COUNT++;
      } else if (message == "green") {
        GREEN_COUNT++;
      }

      inch_forward(100);

      // turn right to trace square
      turnCorner(RIGHT);

      STATE = SQUARE_STATE;
      SQUARE_COUNT++;

      delay(100);
    } else if (same == ALL_BLACK) { // TURN CORNER
      Serial.print("turn: ");
      Serial.println(turn == RIGHT ? "right" : "left");

      turnCorner(turn);
    } else if (SQUARE_COUNT == 2) {
      if (lineArray[0] == 1 && lineArray[1] == 1) {
        turnCorner(RIGHT);
      } else if (lineArray[11] == 1 && lineArray[12] == 1) {
        turnCorner(LEFT);
      }
    }
  } else if (STATE == SQUARE_STATE) {
    if (same == ALL_WHITE) { // time to exit square
      inch_forward(70);

      // turn right to leave square
      turnCorner(RIGHT);

      if (SQUARE_COUNT == 4) {
        STATE = DOTTED_STATE;
      } else if (SQUARE_COUNT == 5) {
        STATE = GRID_STATE;
        if (BLUE_COUNT > RED_COUNT && BLUE_COUNT > GREEN_COUNT) {
          maxColor = "blue";
        } else if (RED_COUNT > BLUE_COUNT && RED_COUNT > GREEN_COUNT) {
          maxColor = "red";
        } else {
          maxColor = "green";
        }
      } else {
        STATE = DEFAULT_STATE;
      }
    } else if (same == ALL_BLACK) { // trace corner in square
      inch_forward(50);

      // turn left to trace square
      delay(100);
      turnCorner(LEFT);
    }
  } else if (STATE == DOTTED_STATE) {
    if (same == ALL_WHITE) { // enter square
      inch_forward(30);

      // turn right to trace square
      turnCorner(RIGHT);

      STATE = SQUARE_STATE;
      SQUARE_COUNT = 5;

      delay(100);
    }
  } else if (STATE == GRID_STATE) {
    bool gap = findGap();
    if (same == ALL_BLACK) {
      turnCorner(LEFT);
    } else if (same == ALL_WHITE) {
      inch_forward(100);
      readADC();
      digitalConvert();

      inch_backward(30);
      if (all_same() == ALL_BLACK) {
        turnCorner(LEFT);
        return;
      }
      // turn right to trace square
      turnCorner(RIGHT);

      STATE = SQUARE_STATE;
      SQUARE_COUNT = 6;

      delay(100);
    } else if (gap || (lineArray[0] == 1 && lineArray[1] == 1)) {
      stop();
      inch_backward(75);
      String message = "straight";
      for (int i = 0; i < 5; i++) {
        message = commWithServer(maxColor);
        if (!message.isEmpty() && message != "invalid") {
          break;
        }
        delay(1000);
      }

      inch_forward(150);

      if (message == "left") {
        turnCorner(LEFT);
        left_count++;
      } else if (message == "right") {
        turnCorner(RIGHT);
        right_count++;
      } else {
        inch_forward(100);
        STATE = GRID_STATE;
        straight_count++;
      }
    }
  }

  delay(50);
}