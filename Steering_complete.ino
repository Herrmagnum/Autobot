/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/

#include <PID_v1.h>
#include <Servo.h>
/*Defines output pin for PID(steering)*/
#define PIN_OUTPUT 3
Servo Dcmotor, FrontSteering,BackSteering;

double Dc = 7;
double servo = 6;
double servo2 = 5;

/*Define sensor pins*/
const int sensor8 = 13;
const int sensor9 = 12;
const int sensor10 = 11;
const int sensor11 = 10;
const int sensor12 = 9;
const int sensor13 = 8;

/*Constants for the car*/
double Ls=0.079 , Lc=0.264 ;
double SensorDistances[]={0.0818,0.0533,0.0265,-0.0265,-0.0533,-0.0818};

/*Ints for sensor reading array*/
int val1, val2, val3, val4, val5, val6;
int sensors[6];

void ReadSensors()
{
  val1 = digitalRead(sensor8);
  val2 = digitalRead(sensor9);
  val3 = digitalRead(sensor10);
  val4 = digitalRead(sensor11);
  val5 = digitalRead(sensor12);
  val6 = digitalRead(sensor13);

  /*Updates sensor array*/
   sensors[0]= val1;
   sensors[1]= val2;
   sensors[2]= val3;
   sensors[3]= val4;
   sensors[4]= val5;
   sensors[5]= val6;
 
}

double DstoAlpha(int Sensors[])
/*Converts sensorreadings to angle alpha, to send into PID for steering*/
{
  double count =0;
  double sumDs =0;
  double SUMDs =0;
  double Ds=0;
 for( int i = 0; i<3; i++)
{
  double sumDs = SensorDistances[i]*Sensors[i] + SensorDistances[5-i]*Sensors[5-i];
  if(sumDs!=0)
  {
    count ++;
  }
  SUMDs += sumDs;
}
if(count!=0) 
{
  Ds = SUMDs/count;
}
  double Alpha = (180/3.14)*atan(2*Ds*Lc/(Ds*Ds+(Lc+Ls)*(Lc+Ls))); 
  //Serial.println(Alpha);
 return Alpha; 
}

//Define Variables we'll be connecting to
double Setpoint, Input, Output,ServoWrite, ServoWrite2;
int SampleTime = 50;
 
//Define the aggressive and conservative Tuning Parameters
double aggKp=1.5, aggKi=0.15, aggKd=0.025;
double consKp=1.5, consKi=0.15, consKd=0.025;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = 90;
  Setpoint = 0;

  Dcmotor.attach(Dc);
  FrontSteering.attach(servo);
  BackSteering.attach(servo2);
  pinMode(Dc,OUTPUT);
  pinMode(servo,OUTPUT);
  pinMode(servo2,OUTPUT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(SampleTime);
  myPID.SetOutputLimits(-20,20);

/*Assigning sensor input pins*/
   for(int i=sensor8; i<=sensor13; i++)
  {
     pinMode(i,INPUT);
  }  
  Serial.begin(57600);
}

void loop()
{
  Input = DstoAlpha(sensors);

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 30)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  ServoWrite= 91.5-1.242*Output;
  ServoWrite2= 91.5+1.242*Output;
  myPID.Compute();
  //analogWrite(PIN_OUTPUT, ServoWrite);
  FrontSteering.write(ServoWrite);
  BackSteering.write(ServoWrite2);
  Dcmotor.write(102.6);
  /*Temporary solution to get sensor updates*/
//start
  val1 = digitalRead(sensor8);
  val2 = digitalRead(sensor9);
  val3 = digitalRead(sensor10);
  val4 = digitalRead(sensor11);
  val5 = digitalRead(sensor12);
  val6 = digitalRead(sensor13);

  /*Updates sensor array*/
   sensors[0]= val1;
   sensors[1]= val2;
   sensors[2]= val3;
   sensors[3]= val4;
   sensors[4]= val5;
   sensors[5]= val6;
   //end
  Serial.println(ServoWrite);
}




