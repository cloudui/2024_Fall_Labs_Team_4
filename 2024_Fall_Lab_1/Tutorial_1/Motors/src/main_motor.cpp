#include <Arduino.h>

const unsigned int M1_IN_1 = 13; // GPIO13
const unsigned int M1_IN_2 = 12; // GPIO12
const unsigned int M2_IN_1 = 25; // GPIO25
const unsigned int M2_IN_2 = 14; // GPIO14
const unsigned int BUZZER_PIN = 26; // GPIO26

void setup() {
  Serial.begin(115200);
  
  // Set motor control pins as outputs
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  // Set buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

  // Ensure all pins are LOW initially
  digitalWrite(M1_IN_1, LOW);
  digitalWrite(M1_IN_2, LOW);
  digitalWrite(M2_IN_1, LOW);
  digitalWrite(M2_IN_2, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  //Move forward
  // digitalWrite(M1_IN_1, HIGH);
  // digitalWrite(M1_IN_2, LOW);
  // digitalWrite(M2_IN_1, HIGH);
  // digitalWrite(M2_IN_2, LOW);
  // delay(2000); // Move forward for 2 seconds

  // Stop
  digitalWrite(M1_IN_1, LOW);
  digitalWrite(M1_IN_2, LOW);
  digitalWrite(M2_IN_1, LOW);
  digitalWrite(M2_IN_2, LOW);
  delay(500); // Stop for half a second

  //Move backward
  // digitalWrite(M1_IN_1, LOW);
  // digitalWrite(M1_IN_2, HIGH);
  // digitalWrite(M2_IN_1, LOW);
  // digitalWrite(M2_IN_2, HIGH);
  // delay(2000); // Move backward for 2 seconds

  // Stop
  digitalWrite(M1_IN_1, LOW);
  digitalWrite(M1_IN_2, LOW);
  digitalWrite(M2_IN_1, LOW);
  digitalWrite(M2_IN_2, LOW);
  delay(500); // Stop for half a second
}





// #include <Arduino.h>

// const unsigned int M1_IN_1 = 13;
// const unsigned int M1_IN_2 = 12;
// const unsigned int M2_IN_1 = 25;
// const unsigned int M2_IN_2 = 14;

// const unsigned int M1_IN_1_CHANNEL = 8;
// const unsigned int M1_IN_2_CHANNEL = 9;
// const unsigned int M2_IN_1_CHANNEL = 10;
// const unsigned int M2_IN_2_CHANNEL = 11;

// const unsigned int M1_I_SENSE = 35;
// const unsigned int M2_I_SENSE = 34;

// const unsigned int PWM_VALUE = 512; // Do not give max PWM. Robot will move fast

// const int freq = 5000;
// const int resolution = 10;


// void setup() {
//   Serial.begin(115200);
  
//   ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
//   ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
//   ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
//   ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

//   ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
//   ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
//   ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
//   ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

//   pinMode(M1_I_SENSE, INPUT);
//   pinMode(M2_I_SENSE, INPUT);
// }

// void loop() {

// }
