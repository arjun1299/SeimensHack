int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
int CAP_PROXY = 3; 
float GLASS_THRESHOLD = 0.6;
float OBJ_THRESHOLD=1; 
int DEPTH_PIN = 4;

void setup()
{
  Serial.begin(9600);

  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  pinMode(CAP_PROXY, INPUT);
  pinMode(DEPTH_PIN, INPUT_PULLUP);
  //Serial.println("MP8511 example");
}

void loop()
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
  if(uvIntensity>GLASS_THRESHOLD && uvIntensity<OBJ_THRESHOLD)
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
  depthFlag = digitalRead(DEPTH_PIN);


  
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
          
  
  
  
  Serial.print(" UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);

  Serial.print("Capacitive sensor:");
  Serial.print(digitalRead(CAP_PROXY));

  Serial.print("Depth sensor:");
  Serial.print(digitalRead(DEPTH_PIN));
  Serial.println();
  
  delay(100);
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
