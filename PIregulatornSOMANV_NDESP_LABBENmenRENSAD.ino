# include <Servo.h>

int HALL=3;                               //pin för hall
int state=HIGH;                              //signal från hall
int oldstate=HIGH;
double v=0;                               //Hastighet cm/s
double vold=0;
double Omkrets=20.4;                         //Omkrets på hjulet cm
unsigned long lasttime=0; 
unsigned long currenttime=0;
unsigned long lastangletime=0;
unsigned long inputintervall=200000;      //Microsekunder mellan PID beräkningar

//regulator 1
double Sampletime=0.2;
double P=0.00856;
double I=0.0856;
double d=0;
long integral =0;

//Regulator 2
//double Sampletime=0.2;
//double P=0.00618;
//double I=0.618;

Servo motor;
int Mymotor = 12;
float angle = 90;

double Setpoint;

void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600); 
    pinMode(Mymotor,OUTPUT);            //sätter dc motor som output
    motor.attach(Mymotor); 
    Setpoint = 90;                       // Börvärde till refulatorn
    angle=103.7;                 //Uppstart
    motor.write(angle);         //Uppstart
    delay(2000);                //Uppstart
}

void loop() {

    currenttime=micros();
    state=digitalRead(HALL);              //Reads the hallsignal

    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=micros();
      if (currenttime==0){
      }
      else{
        v=(2*Omkrets*1000000)/((currenttime-lasttime)*17.53);    //multiply with e6 17.53 magnets per wheel rotation men det halverades när vi tog bort varannan magnet
        lasttime=currenttime;    

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
      lastangletime=currenttime;
    }
}
