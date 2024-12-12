#include <Arduino.h>
#include <ESP32Encoder.h>

const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

ESP32Encoder enc1;
ESP32Encoder enc2;



void setup() {
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Enable pull-up resistors
  enc1.attachHalfQuad(M1_ENC_A, M1_ENC_B); // Attach pins
  enc1.clearCount(); // Reset encoder count

  enc2.attachHalfQuad(M2_ENC_B, M2_ENC_A); // Attach pins
  enc2.clearCount(); // Reset encoder count

  // set to low
  pinMode(M2_IN_1, OUTPUT);
  digitalWrite(M2_IN_1, LOW);
  pinMode(M2_IN_2, OUTPUT);
  digitalWrite(M2_IN_2, LOW);

}

void loop() {
  long position = enc1.getCount(); // Read position
  Serial.print("Encoder 1: ");
  Serial.println(position);

  position = enc2.getCount(); // Read position
  Serial.print("Encoder 2: ");
  Serial.println(position);

  delay(100);
}
