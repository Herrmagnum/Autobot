# include <Servo.h>
#include <PID_v1.h>

int HALL=3;                               //pin för hall
int state=HIGH;                              //signal från hall
int oldstate=HIGH;
double v=0;                               //Hastighet cm/s
double vold=0;
double Omkrets=20.4;                         //Omkrets på hjulet cm
int passeringar;
unsigned long lasttime=0; 
unsigned long currenttime=0;
unsigned long lastangletime=0;
unsigned long inputintervall=220000;      //Microsekunder mellan PID beräkningar

//regulator 1
double Sampletime=0.2;
double P=0.00856;
double I=0.0856;
long integral =0;

//Regulator 2
//double Sampletime=0.2;
//double P=0.00618;
//double I=0.618;

Servo motor;
int Mymotor = 12;
float angle = 90;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=0.00944, aggKi= 0.123, aggKd= 0.000169;
double consKp=0.00944, consKi=0.123, consKd= 0.000169;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600); 
    pinMode(Mymotor,OUTPUT);            //sätter dc motor som output
  motor.attach(Mymotor); 
  
  passeringar=0;
//  Input = 0;
  Setpoint = 140;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
//  SetSampleTime(20);
  angle=103.7;
  motor.write(angle); 
  delay(2000);
}

void loop() {

//Setpoint=Serial.read();
currenttime=micros();


   

    state=digitalRead(HALL);              //Reads the hallsignal


    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=micros();
      if (currenttime==0){
      }
      else{
        v=(2*Omkrets*1000000)/((currenttime-lasttime)*17.53);    //multiply with tousend to get seconds instead of mS 17.53 magnets per wheel rotation
        lasttime=currenttime;    
        passeringar++;

        if(v < 0.6*vold){
          v=vold;
        }
        Serial.print(v);                      //Only for testing
        Serial.print('\n');                   //Only for testing

        vold=v;
      }
    }

    oldstate=state;

if((currenttime-lastangletime) > inputintervall) {   

    integral=integral+(Setpoint-v)*Sampletime;
    angle=90+P*(Setpoint-v)+integral*I;
    

    motor.write(angle);
  //  Serial.print(angle);
  //  Serial.print('\n');
   //delayMicroseconds(ts);
   lastangletime=currenttime;
  }
}
