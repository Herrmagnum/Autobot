# include <Servo.h>
Servo FrontServo;
int ServoPinFront = 2;
Servo BackServo;
int ServoPinBack = 3;

//front angle sensor
int FrontValue1 = 507;
int FrontValue2 = 511;
int FrontValue3 = 516; //low zero
int FrontValue4 = 523; //high zero
int FrontValue5 = 568;
int FrontValue6 = 850;
int sensorPinFront = A0;    // select the input pin for the potentiometer
float  angleFront;

//back angle sensor
int BackValue1 = 522;
int BackValue2 = 539;
int BackValue3 = 568; //low zero
int BackValue4 = 593; //high zero
int BackValue5 = 681;
int BackValue6 = 1021;
int sensorPinBack = A1;    // select the input pin for the potentiometer
float  angleBack;

//common variables
int sensorValue = 0;
float  angle;
float time;
int inputAngle;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(9600);
  FrontServo.attach(ServoPinFront); 
  BackServo.attach(ServoPinBack);
  pinMode(ServoPinFront,OUTPUT);
  pinMode(ServoPinBack,OUTPUT);
}

void loop() {
  inputAngle = random(-30,30);
  FrontServo.write(90+inputAngle);
  BackServo.write(90-inputAngle);
  
  time = millis();
  angleFront = GetAngle(FrontValue1, FrontValue2, FrontValue3, FrontValue4, FrontValue5, FrontValue6, sensorPinFront);
  angleBack = GetAngle(BackValue1, BackValue2, BackValue3, BackValue4, BackValue5, BackValue6, sensorPinBack);
  
  Serial.print(90+inputAngle); //input angle
  Serial.print(",");
  Serial.print(90+angleFront); //angle front
  Serial.print(",");
  Serial.print(90-angleBack); //angle back
  Serial.print(",");
  Serial.print(time/1000);
  Serial.println();
  delay(2000);
}

float GetAngle(int Value1, int Value2, int Value3, int Value4, int Value5, int Value6, int sensorPin)
{
  sensorValue = analogRead(sensorPin);
  if (sensorValue<Value2)
  {
    angle = map(sensorValue, Value1, Value2, 3000,1500);
  }
  
  else if (sensorValue>=Value2 && sensorValue<Value3)
  {
    angle = map(sensorValue, Value2, Value3,1500,0);
  }

  else if (sensorValue>=Value3 && sensorValue<Value4)
  {
    angle = 0;
  }
  
  else if (sensorValue>=Value4 && sensorValue<Value5)
  {
    angle = map(sensorValue, Value4, Value5, 0, -1500);
  }
  
  else
  {
    angle = map(sensorValue, Value5, Value6, -1500,-3000);
  }
  angle = angle/100;
  //angle = sensorValue;
  return angle;
}
