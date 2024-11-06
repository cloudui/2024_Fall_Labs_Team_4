#include <Arduino.h>
#include <Encoder.h>
#include <cmath>

const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

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

const unsigned int PWM_VALUE = 150; // Max PWM given 8 bit resolution

const int freq = 5000;
const int resolution = 8;

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

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

}

void loop() {
  // Create the encoder objects after the motor has
  // stopped, else some sort exception is triggered
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  long enc1_value = enc1.read();
  long enc2_value = enc2.read();
  int m1_pwm = PWM_VALUE;
  int m2_pwm = PWM_VALUE;
  int m1 = 0;
  int m2 = 0;

  while(true) {
    Serial.println(enc1_value);
    Serial.println(enc2_value);
    Serial.println("");

    ledcWrite(M1_IN_1_CHANNEL, m1_pwm);
    ledcWrite(M1_IN_2_CHANNEL, 0);
    ledcWrite(M2_IN_1_CHANNEL, m2_pwm);
    ledcWrite(M2_IN_2_CHANNEL, 0);

    delay(1000);

    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, 0);
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, 0);

    delay(1000);

    long diff_one = abs(enc1.read() - enc1_value);
    long diff_two = abs(enc2.read() - enc2_value);

    Serial.println("Diff: ");
    Serial.println(diff_one);
    Serial.println(diff_two);
    Serial.println("");
    Serial.println("Motors: ");
    Serial.println(m1_pwm);
    Serial.println(m2_pwm);
    Serial.println("");

    if (diff_one > diff_two + 100){
      if(m2 == 1){
        m2_pwm += 2;
      }
      else{
        m1_pwm -= 10;
        m1 = 1;
      }
    }
    else if (diff_one + 100 < diff_two){
      if(m1 == 1){
        m1_pwm += 2;
      }
      else{
        m2_pwm -= 10;
        m2 = 1;
      }
    }

    enc1_value = enc1.read();
    enc2_value = enc2.read();

    delay(1000);
  }
}
