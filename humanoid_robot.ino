#include "constants.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDRESS 0x40 // You may need to adjust the address based on your board

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
// Writes the angles to each joint of leg obtained from function pos()
void updateServoPos(int h, int k, int f, char leg) {
  if (leg == 'l') {
    pwm.setPWM(5, 0, HLOffset - h); // Hip Left
    pwm.setPWM(3, 0, KLOffset - k); // Knee Left
    pwm.setPWM(1, 0, 2 * ALOffset - f); // Ankle Left
  } else if (leg == 'r') {
    pwm.setPWM(4, 0, HROffset + h); // Hip Right
    pwm.setPWM(2, 0, KROffset + k); // Knee Right
    pwm.setPWM(0, 0, f); // Ankle Right
  }
}


// Does reverse kinematics and calculate angles in degress of each joint based on coordinate of end effector
void pos(float HF_x, float HF_y, char leg){
  float h_a = atan(HF_x/HF_y);
  float h_a_d = h_a * (180/PI); // -> Converted to degree
  float HF = HF_y/cos(h_a);

  float h_b = acos((sq(l1) + sq(HF) - sq(l2))/(2*l1*HF));
  float h_b_d = h_b * (180/PI); // -> Converted to Degree
  
  float k_b = PI - acos((sq(l1) + sq(l2) - sq(HF))/(2*l1*l2));

  float f_r = PI/2 + h_a - acos((sq(l2) + sq(HF) - sq(l1))/(2*l2*HF));
  
  float h = h_a_d + h_b_d;
  float k = k_b * (180/PI);
  float f = f_r * (180/PI);

  Serial.print(h);
  Serial.print("\t");
  Serial.print(k);
  Serial.print("\t");
  Serial.println(f);

  updateServoPos(h, k, f, leg);  
}

void takeStep(float stepLength, int stepVelocity){
  // Calculate the number of steps based on the desired distance and step size
  int numSteps = int(stepLength / 0.5); // Assuming each step is 0.5 meters in length
  // Loop for the forward step
  for (int step = 0; step <= numSteps; step++) {
    float stepDistance = step * 0.5;
    pos(stepDistance, stepHeight, 'r');
    pos(-stepDistance, stepHeight - stepClearance, 'l');
    delay(stepVelocity);
  }

  // Loop for the backward step
  for (int step = numSteps - 1; step >= 0; step--) {
    float stepDistance = step * 0.5;
    pos(stepDistance, stepHeight, 'r');
    pos(-stepDistance, stepHeight - stepClearance, 'l');
    delay(stepVelocity);
  }

}

void initialize(){
  for (float i = 10.7; i >= stepHeight; i-=0.1){
    pos(0, i, 'l');
    pos(0, i, 'r');
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C communication
  pwm.begin();  // Initialize the PCA9685
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // Set the frequency of the PWM signal (50 Hz for most servos)
  pwm.setPWM(0, 0, SERVOMAX);
  Serial.println("done");
  // You may need to calibrate the PWM frequency based on your servos.
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("stepping");
  takeStep(2, 1000);
}
