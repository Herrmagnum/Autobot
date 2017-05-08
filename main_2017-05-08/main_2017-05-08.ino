# include <Servo.h>
#include <PID_v1.h>
Servo FrontSteering,BackSteering,motor;

float angle;

// TIMERS All in microseconds
unsigned long lastSampletime=0; 
int Ts= 200;                         //Microsecond between the samplings of velocity
unsigned long currenttime=0;
unsigned long lastSpeedPIDtime=0;
unsigned long lasPrinttime=0;
int printtime=1000000;
int inputintervall=220000;           //Microsekunder mellan PID beräkningar


//Hastighetsmätningen
int state=HIGH;                              //signal från hall
int oldstate=HIGH;
float v;                               //Hastighet cm/s
float vold=0;
int passeringar=0;
float vsum=0;


//CrouseControll Parametrar Home made PID
double SetSpeed;
float oldError=0;          //Gamla felet i hastighet regulatorn
float integral =0;               //ca 10/Ki
int nomagnet=0;


// PID LIBRARY CROUSECONTROLL PARAMETERS
 double P=0.00856;
  double I=0.0856;
  double D=0;
 double Speedinput, speedOutput;
PID speedpid(&Speedinput, &speedOutput, &SetSpeed, P, I, D, DIRECT);


// Obsticleavoidance
int avoid=LOW;


                  /* DIGITAL PINS */ 
int HALL=2;                               //pin för hall
int distsensor = 5;
int Mymotor = 3;
int servo = 4;
int servo2 = 0;

/*Define sensor pins*/
const int sensor8 = 13;
const int sensor9 = 12;
const int sensor10 = 11;
const int sensor11 = 10;
const int sensor12 = 9;
const int sensor13 = 8;
const int sensor14 = 7;
const int sensor15 = 6;

/*Constants for the car*/
double Ls=0.079 , Lc=0.264 ;
double SensorDistances[]={0.099,0.076,0.053,0.030,-0.030,-0.053,-0.076, -0.099};


//SensorArray
//int sensor8,sensor9,sensor10,sensor11,sensor12,sensor13;
int val1, val2, val3, val4, val5, val6, val7, val8;
int sensors[8];

//Define Variables we'll be connecting to
double Setpoint, Input, Output,ServoWrite, ServoWrite2;
float STime = 50;

//Define the aggressive and conservative Tuning Parameters
double aggKp=1.5, aggKi=0, aggKd=0;
double consKp=0.889, consKi=0, consKd=0.234;


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

 double LastReadings =0;             // inititsiering föregående utslag av sensorarray 
 int LostLine = 0; // parameter for controll of steering when car lost the line (0= on target, 1= lost line LEFT, -1= lost line RIGTH)
void setup()
{
  for(int i=sensor8; i<=sensor15; i++)
  {
     pinMode(i,INPUT);
  } 
 
//  pinMode(interruptPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
 
  pinMode(distsensor,INPUT);                //sätter hallsensor som input
  pinMode(HALL,INPUT); 
  pinMode(Mymotor,OUTPUT);           //sätter dc motor som output

  motor.attach(Mymotor);             //kopplat främre servo till pin Mymotor
  SetSpeed=100;
  v=0;
  

  // Styrning
  FrontSteering.attach(servo);
  BackSteering.attach(servo2);
  pinMode(servo,OUTPUT);
  pinMode(servo2,OUTPUT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  speedpid.SetMode(AUTOMATIC);
  myPID.SetSampleTime(STime);
  speedpid.SetSampleTime(200);
  speedpid.SetOutputLimits(90,130);
  myPID.SetOutputLimits(-20,20);
  Serial.begin(9600); 
  
  
  // KAN BEHÖVAS EN UPPSTARTS SEKVENS FÖR REGULATORERNA
integral=0;
  angle=102.6;
  motor.write(angle); 
  delay(2000);

}

                                          void loop() {
                                            currenttime=micros();
//                                            avoid=digitalRead(5);
                                            SetSteering();
                                             speedpid.Compute();
                                              motor.write(speedOutput);
//                                            if(avoid=HIGH){
//                                                OBsticleavoidance();
//                                            }
                                            if((currenttime-lastSampletime)>Ts){
                                               ReadSpeed();
                                            }
//                                             if((currenttime-lastSpeedPIDtime) > inputintervall){   //Home made PIDfor crousecontroll
//                                               SPEEDcontroler();
//                                            }
                                             if((currenttime-lasPrinttime) > printtime){
                                              serialPrint();
                                              lasPrinttime=currenttime;
                                             }
                                          }


void SetSteering(){
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
  ServoWrite2= 91.5+2.2*Output;
  myPID.Compute();
  //analogWrite(PIN_OUTPUT, ServoWrite);
  FrontSteering.write(ServoWrite);
  BackSteering.write(ServoWrite2);
  /*Temporary solution to get sensor updates*/
//start
  val1 = digitalRead(sensor8);
  val2 = digitalRead(sensor9);
  val3 = digitalRead(sensor10);
  val4 = digitalRead(sensor11);
  val5 = digitalRead(sensor12);
  val6 = digitalRead(sensor13);
  val7 = digitalRead(sensor14);
  val8 = digitalRead(sensor15);

  /*Updates sensor array*/
   sensors[0]= val1;
   sensors[1]= val2;
   sensors[2]= val3;
   sensors[3]= val4;
   sensors[4]= val5;
   sensors[5]= val6;
   sensors[6]= val7;
   sensors[7]= val8;
}

        /* SPEEDcontroller uses the velocity V, Vold and the setpoint SetSpeed to adjust the input signal to the motor */
                                        void SPEEDcontroler(){          //Euler backward PID assuming constant samplingtime
                                            double Smpltime=0.22;     //Converts the sampletime to seconds from microseconds
                                            float P=0.00856;
                                            float I=0.0856;
                                            float D=0;

                                              if(passeringar>0){
                                                v=vsum/passeringar;
                                              }
                                              else {
                                                v=0;
                                              }
                                           
                                            
                                            integral=integral+(SetSpeed-v)*Smpltime; //Updaterar integralen med antagandet att sampletiden är konstant
                                            angle=90+P*(SetSpeed-v)+integral*I+D*((SetSpeed-v-oldError)/Smpltime);        //Beräknar vinkeln, regulatorn går från 90.
                                            if (angle<90){
                                              angle=90;
                                            }
                                            else if (angle>180){
                                              integral=800;
                                              angle=150;
                                            }
                                            motor.write(angle);
                                            //Serial.println(Smpltime);
                                            oldError=(SetSpeed-v);
                                            lastSpeedPIDtime=currenttime;                 //lastPIDtime är tiden då vinkeln till motorn senast ändrats
                                        }
/* ReadSpeed looks at the state of the hall sensor to calculate the velocity v and stors the previus velocity in vold */
void ReadSpeed(){    //Could be adjusted to use interupt instead but not sure if it is bad for other parts of the script
    float Omkrets=20.4;                   //Omkrets på hjulet 
    state=digitalRead(HALL);              //Reads the hallsignal
    if (state==LOW && oldstate==HIGH){    //Cheaks if the magnet is there
      currenttime=micros();
      if (currenttime==0){                //kan kanske tas bort nu, bör testas på bilen
        v=0;
      }
      else{
        v=(2*Omkrets*1000000)/((currenttime-lastSampletime)*17.53);    //multiply with e6 to get seconds, 17.53 magnets per wheel rotation men vi tog bort hälften av magneterna
        lastSampletime=currenttime;    
                passeringar++;
        if(v < 0.6*vold){             //För att slippa dipparna som sker en gång per varv av hjulen
          v=vold;
        }
        vold=v;
        Speedinput=v;
        //vsum=vsum+v;
        nomagnet=0;
      }
    }
    else {
      nomagnet++;
      if(nomagnet > 2000){
        v=0;
      }
    }
    oldstate=state;
 }
                                       
                                      void OBsticleavoidance(){         //SAfety break for testign
                                            angle=90;
                                            motor.write(angle);
                                            
                                            v=0;
                                       }

  double DstoAlpha(int Sensors[])
/*Converts sensorreadings to angle alpha, to send into PID for steering*/
{
  float count =0;
  double sumDs =0;
  double SUMDs =0;
  double Ds=0;

 for( int i = 0; i<4; i++)
{
  double sumDs = SensorDistances[i]*Sensors[i] + SensorDistances[7-i]*Sensors[7-i];
  
  if(sumDs!=0)
  {
    count ++;
  }
  
  SUMDs += sumDs;
}

  if((LastReadings == SensorDistances[0] && SUMDs == 0)||(LostLine == 1 && SUMDs != SensorDistances[0]))
  {
   LostLine = 1;  // Lost the line and last reading was on the left
   return 22;     // Max Angle for turning left?
   
  }
   if((LastReadings == SensorDistances[7] && SUMDs == 0)||(LostLine == -1 && SUMDs != SensorDistances[7]))
  {
   LostLine = -1;  // Lost the line and last reading was on the right
   return -22;     // Max Angle for turning right?
  }
  else
  {
   LostLine = 0;
  }

if(count!=0) 
{
  Ds = SUMDs/count;
  LastReadings = Ds; 
}

  double Alpha = (180/3.14)*atan(2*Ds*Lc/(Ds*Ds+(Lc+Ls)*(Lc+Ls))); 
  //Serial.println(Alpha);
 return Alpha; 
}



void serialPrint(){
  Serial.print(Input);
  Serial.print(',');
  Serial.print(Output);
  Serial.print(',');
  Serial.print(SetSpeed);
  Serial.print(',');
  Serial.print(v);
  Serial.print(',');
  Serial.print(speedOutput);
  Serial.print('\n');
}

