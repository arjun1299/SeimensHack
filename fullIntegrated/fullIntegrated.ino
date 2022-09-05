/*************************************************** 
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
int PROXY= 9;
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
int CAP_PROXY = 3; 
float GLASS_THRESHOLD = 0.7;
float OBJ_THRESHOLD=1; 
int DEPTH_PIN = 4;
int lidMotor=0;
int plasticMotor=1;
int glassMotor=2;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  pinMode(CAP_PROXY, INPUT);  
  pinMode(DEPTH_PIN, INPUT_PULLUP);
  pinMode(PROXY,INPUT);

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

/*
      pwm.setPWM(lidMotor,0,SERVOMIN);
      delay(500);
      pwm.setPWM(lidMotor,0,SERVOMIN+150);
      delay(500);

      pwm.setPWM(plasticMotor,0,SERVOMIN-35);
      delay(500);
      pwm.setPWM(plasticMotor,0,SERVOMIN+150);
      delay(500);

      pwm.setPWM(glassMotor,0,SERVOMIN);
      delay(500);
      pwm.setPWM(glassMotor,0,SERVOMIN+150);
      delay(500);
*/
      
  delay(10);

  
  pwm.setPWM(lidMotor,0,SERVOMIN);
  pwm.setPWM(glassMotor,0,SERVOMIN+10);
  pwm.setPWM(plasticMotor,0,SERVOMIN-35);  
}

void loop() {

if(!digitalRead(PROXY))
{
    int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
/*
  Serial.print("MP8511 output: ");
  Serial.print(uvLevel);

  Serial.print(" MP8511 voltage: ");
  Serial.print(outputVoltage);
*/
int uvFlag=0;
int capFlag=0;
int depthFlag=0;
int materialFlag=2;//0 - Plastic; 1- Glass; 2- misc
int glassFlag =0;
  if(uvIntensity>GLASS_THRESHOLD)// && uvIntensity<OBJ_THRESHOLD)
  {
      Serial.print("Glass detected");
      uvFlag = 1;
      }

  else if(uvIntensity<GLASS_THRESHOLD)
  {
      Serial.print("Plastic detected");
      uvFlag=0;
      
  }

  capFlag=digitalRead(CAP_PROXY);
  depthFlag = !digitalRead(DEPTH_PIN);


  
  if(uvFlag==0 && capFlag == 0 && depthFlag == 0)
    materialFlag=0;
  else if(uvFlag==0 && capFlag == 1 && depthFlag == 0)
    materialFlag=2;
  else if(uvFlag==0 && capFlag == 1 && depthFlag == 1)
    materialFlag=0;
  else if(uvFlag==1 && capFlag == 1 && depthFlag == 0)
    materialFlag=1;
  else if(uvFlag==1 && capFlag == 1 && depthFlag == 1)
    materialFlag=1;

  Serial.println(materialFlag);
  
  switch (materialFlag)
  {
    case 1:
      pwm.setPWM(plasticMotor,0,SERVOMIN-35);
      delay(500);
      pwm.setPWM(plasticMotor,0,SERVOMIN-100);
      delay(500);
      break;
   case 2:
      pwm.setPWM(plasticMotor,0,SERVOMIN-35);
      delay(500);
      pwm.setPWM(plasticMotor,0,SERVOMIN-100);
      delay(500);

      pwm.setPWM(glassMotor,0,SERVOMIN+10);
      delay(500);
      pwm.setPWM(glassMotor,0,SERVOMIN-75);
      delay(500);
      
      break; 
    }
  
  
  /*
  Serial.print(" UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);

  Serial.print("Capacitive sensor:");
  Serial.print(digitalRead(CAP_PROXY));
  */

  //Serial.print("Depth sensor:");
  //Serial.print(digitalRead(DEPTH_PIN));
  //Serial.println();

  pwm.setPWM(lidMotor,0,SERVOMIN+150);
  delay(5000);

}
  pwm.setPWM(lidMotor,0,SERVOMIN);
  delay(500);
  pwm.setPWM(glassMotor,0,SERVOMIN+10);
  delay(500);
  pwm.setPWM(plasticMotor,0,SERVOMIN-35);
  delay(500); 
  // Drive each servo one at a time using setPWM()
  /*Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(500);
  /*
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(2000);

/*
pwm.setPWM(0,0,2*SERVOMIN);
delay(500);

pwm.setPWM(0,0,SERVOMIN);
delay(500);
/*pwm.setPWM(1,0,SERVOMIN);
delay(500);
pwm.setPWM(1,0,SERVOMIN+100);
delay(500);
pwm.setPWM(2,0,SERVOMIN);
delay(500);
pwm.setPWM(2,0,SERVOMIN+100);
delay(500);
*/
/*
  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
  for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);
  for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

  servonum++;
  if (servonum > 7) servonum = 0; // Testing the first 8 servo channels*/
}


//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
