#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVOMIN  150 
#define SERVOMAX  600 
#define USMIN  600 
#define USMAX  2400 
#define SERVO_FREQ 50 

// our servo 
uint8_t FRS = 0;
uint8_t servonum1 = 1;
uint8_t servonum2 = 2;
uint8_t FRK = 3;
uint8_t BRS = 4;
uint8_t servonum5 =5;
uint8_t servonum6 = 6;
uint8_t BRK = 7;
uint8_t BLA = 8;
uint8_t servonum9 = 9;
uint8_t servonum10 = 10;
uint8_t BLK = 11;
uint8_t FLS = 12;
uint8_t servonum = 13;
uint8_t servonum14 = 14;
uint8_t FLK = 15;
void setup() {
  Serial.begin(9600);
  Serial.println("15 channel Servo test!");

  pwm.begin();
 
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);
}


void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;  
  pulselength /= SERVO_FREQ;   
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  
 
  
  Serial.println(FLS);
delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(FLS, 0, pulselen);
  }
  Serial.println(FLK);
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(FLK, 0, pulselen);
  }
  Serial.println(BLA);
   delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(BLA, 0, pulselen);
  }
    Serial.println(BLK);
   delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(BLK, 0, pulselen);
    delay (1000);
  }
  Serial.println(FRS);
   delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(FRS, 0, pulselen);
  }
Serial.println(FRK);
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(FRK, 5, pulselen);
  }
  Serial.println(BRS);
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(BRS, 0, pulselen);
  }
    Serial.println(BRK);
   delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(BRK, 0, pulselen);
  }
}
