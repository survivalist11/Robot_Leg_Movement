-/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMINSTOP 50
#define SERVOMIN  (SERVOMINSTOP+10) // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMID  (SERVOMAX-SERVOMIN)/2
#define SERVOMAX  (SERVOMAXSTOP-250) // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAXSTOP 500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

struct servo {
  int id;
   int minPos;
   int midPos;
   int maxPos;
};
//void servoUp(struct servo servoID) {
//  Serial.println(servoID.id);
//    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos; pulselen++) {
//    pwm.setPWM(servoID.id, 0, pulselen);
//    }
//    Serial.println("Done Up");
//}
//
//void servoDown(struct servo servoID) {
//  Serial.println(servoID.id);
//    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.minPos; pulselen--) {
//    pwm.setPWM(servoID.id, 0, pulselen);
//    }
//    Serial.println("Done Down");
//}
// our servo # counter
//uint8_t servonum = 0;
struct servo servo0; 
struct servo servo1; 
uint8_t loop_counter = 4;

void setup() {
  servo0.id = 0;
  servo0.minPos = SERVOMIN;
  servo0.midPos = SERVOMID;
  servo0.maxPos = SERVOMAX;
  servo1.id = 1;
  servo1.minPos = SERVOMIN;
  servo1.midPos = SERVOMID;
  servo1.maxPos = SERVOMAX;
  Serial.begin(9600);
  pwm.begin();

  
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  servoMinUpToMid (servo1);
  servoMinUpToMid (servo0);
  delay(10);
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void servoMinUpToMid(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.midPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MinUpToMid");
}

void servoMaxDownToMid(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.midPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MaxDownToMid");
}

void servoMidUpToMax(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.midPos; pulselen < servoID.maxPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MidUpToMax");
}

void servoMidDownToMin(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.midPos; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MidDownToMin");
}

void servoMaxDownToMin(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.maxPos; pulselen > servoID.minPos; pulselen--) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("Done MaxDownToMin");

}

void servoMinUpToMax(struct servo servoID) {
  Serial.println(servoID.id);
    for (uint16_t pulselen = servoID.minPos; pulselen < servoID.maxPos; pulselen++) {
    pwm.setPWM(servoID.id, 0, pulselen);
    }
    Serial.println("MinUpToMax");
}

void loop() {
  if (loop_counter <= 0) return;

  loop_counter--;

  servoMidDownToMin(servo0);

  delay(500);

  servoMinUpToMax(servo1);

  delay(500);
  
  servoMinUpToMax(servo0);
  
  delay(500);

  servoMaxDownToMid(servo1);

  delay(500);


}
