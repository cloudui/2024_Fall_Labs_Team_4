#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

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
const float Kd = 0.5;
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

int blue;
int purple;
int green;


/*
 *  WiFi code integration
 */
struct __attribute__((packed)) Data {
    int16_t seq;     // sequence number
    char text[50];   // text
};

// WiFi network credentials
// const char* ssid = "ParksideUnit521";
// const char* password = "BCQLMMDN";

// Server IP and port
// const char* host = "10.5.21.112";  // Replace with the IP address of server
const uint16_t port = 9500;

// Emily's House Settings
const char* host = "10.0.0.79";  // Replace with the IP address of server
const char* ssid = "luckytheratdog";
const char* password = "Luckyblueberrymuffin382!";

// Create a client
WiFiClient client;

int i_wifi = 1;
char finalText[50];  // Store the final text after 500 loops

String commWithServer(const String& message) {
  WiFiClient client;
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
    return "Empty";
  }
}

String runWiFiExchange() {
  // Connect to server
  if (!client.connect(host, port)) {
    Serial.println("Connection to server failed.");
    return "left";
  }

  Serial.println("Connected to server!");
  int i_wifi = 1;

  for (int attempt = 0; attempt < 10; attempt++) {
    if (client.connected()) {
      Serial.printf("Attempt %d: Listening for server messages...\n", attempt + 1);

      // Wait for a message from the server
      Data data;
      strncpy(data.text, "direction", sizeof(data.text));
      client.write((uint8_t*)&data, sizeof(data));

      while (client.available()) {
        char message[64] = {0}; // Initialize buffer to zero
        Data response;
        client.readBytes((char*)&response, sizeof(response));
        Serial.printf("seq %d text %s\n", (int)response.seq, response.text);
        // Store the latest response text
        strncpy(finalText, response.text, sizeof(finalText) - 1);


        Serial.printf("Received: %s\n", message);

        // Check for "left" or "right"
        if (strcmp(message, "left") == 0 || strcmp(message, "right") == 0) {
          Serial.printf("Direction: %s\n", message);
          return String(message); // Explicitly return a String object
        }
      }

      // Add a small delay between retries
      delay(500);
    } else {
      Serial.println("Disconnected from server. Reconnecting...");
      if (client.connect(host, port)) {
        Serial.println("Reconnected to server.");
      } else {
        Serial.println("Connection to server failed.");
        return "left";
      }
    }
  }

  Serial.println("Failed to receive a valid message after multiple attempts.");
  return "left";
}


/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i) > 690 ? 0 : 1;
    adc2_buf[i] = adc2.readADC(i) > 690 ? 0 : 1;
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
    lineArray[2*i] = adc1_buf[i];

    if (i < 6) {
      lineArray[2*i + 1] = adc2_buf[i];
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

void inch_forward(int time) {
  M1_forward(DEFAULT_PWM);
  M2_forward(DEFAULT_PWM);
  delay(time);
  M1_stop();
  M2_stop();
}

void stop() {
  M1_stop();
  M2_stop();
}

void turnCorner(bool right, int right_wheel, int left_wheel) {
  stop();
  delay(500);

  if (right) {
    M1_forward(left_wheel);
    M2_backward(right_wheel);
    delay(270);
  } else {
    M1_backward(left_wheel);
    M2_forward(right_wheel);
    delay(270);
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

  turn = LEFT;
  STATE = DEFAULT_STATE;

  SQUARE_COUNT = 0;

  delay(100);
}


void loop() {
  
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  enc1.write(0);
  enc2.write(0);

  while(true) {

    readADC();
    digitalConvert();

    pos = getPosition(lineArray); //passing lineArray to function which contains 13 boolean values
    first_pos = pos;
    // Serial.println("Pos is "); Serial.println(pos);

    String response = commWithServer("direction");
    if (!response.isEmpty()) {
      Serial.println("Server says: " + response);
    }

    delay(1000);
    continue;
    

    same = all_same();

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
      } else if (lineArray[12] == 1 && pos > 6) {
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
        total_e += e*DT;

        // Update the previous error
        p_e = e;

        // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
        u = DKp * e + DKd * d_e + DKi * total_e; //need to integrate e

        // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
        pidLeft = constrain_pwm(80 + u);
        pidRight = constrain_pwm(80 - u);

        M1_forward(pidLeft);
        M2_forward(pidRight);
      }
    } else if (STATE == GRID_STATE) {

    }


    // STATE TRANSITIONS
    // Serial.print("same: ");
    // Serial.println(same);
    if (STATE == DEFAULT_STATE) {
      if (same == ALL_WHITE && SQUARE_COUNT == 2) { // WAIT FOR AUDIO
        stop();
        delay(1000);

        String message = runWiFiExchange();
        bool audioTurn = message == "right";

        turnCorner(audioTurn, DEFAULT_PWM, DEFAULT_PWM);
        
        STATE = DEFAULT_STATE;

      } else if (same == ALL_WHITE) { // AT SQUARE
        inch_forward(30);

        // turn right to trace square
        turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

        STATE = SQUARE_STATE;
        SQUARE_COUNT++;
        
        delay(100);
      } else if (same == ALL_BLACK) { // TURN CORNER
        Serial.print("turn: ");
        Serial.println(turn == RIGHT ? "right" : "left");

        turnCorner(turn, DEFAULT_PWM, DEFAULT_PWM);
      } else if (SQUARE_COUNT == 2) {
        if (lineArray[0] == 1 && lineArray[1] == 1 && lineArray[2] == 1) {
          turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);
        } else if (lineArray[11] == 1 && lineArray[12] == 1 && lineArray[10] == 1) {
          turnCorner(LEFT, DEFAULT_PWM, DEFAULT_PWM);
        }
      }
    } else if (STATE == SQUARE_STATE) {
      if (same == ALL_WHITE) { // time to exit square
        inch_forward(100);

        // turn right to leave square
        turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

        if (SQUARE_COUNT == 4) {
          STATE = DOTTED_STATE;
        } else if (SQUARE_COUNT == 5) {
          STATE = GRID_STATE;
        } else {
          STATE = DEFAULT_STATE;
        }
      } else if (same == ALL_BLACK) { // trace corner in square
        inch_forward(50);

        // turn left to trace square
        delay(100);
        turnCorner(LEFT, DEFAULT_PWM, DEFAULT_PWM);
      }
    } else if (STATE == DOTTED_STATE) {
      if (same == ALL_WHITE) { // enter square
        inch_forward(30);

        // turn right to trace square
        turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

        STATE = SQUARE_STATE;
        SQUARE_COUNT = 5;
        
        delay(100);
      } 
    } else if (STATE == GRID_STATE) {
      if (same == ALL_WHITE) {
        inch_forward(30);

        // turn right to trace square
        turnCorner(RIGHT, DEFAULT_PWM, DEFAULT_PWM);

        STATE = SQUARE_STATE;
        SQUARE_COUNT = 6;
        
        delay(100);
      }
    }

    delay(50);
  }
}