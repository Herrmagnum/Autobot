# include <Servo.h>

int HALL=2;                               //pin för hall
int state=HIGH;                              //signal från hall
int oldstate=HIGH;
double v=0;                               //Hastighet cm/s
double vold=0;
double Omkrets=20.4;                         //Omkrets på hjulet cm
int ts=2;                                //sampletime  microseconds
unsigned long lasttime=0; 
unsigned long currenttime=0;
unsigned long lastangletime=0;

Servo motor;
int Mymotor = 12;
float angle = 90;
unsigned long inputintervall=2000000;   //intervall för slumpningen

void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600); 
    pinMode(Mymotor,OUTPUT);            //sätter dc motor som output
    motor.attach(Mymotor); 
    angle=104.2;
    motor.write(angle);  
}

void loop() {

  if((currenttime-lastangletime) > inputintervall) {    //Slumpar input till motorn
    angle=random(103.3, 108);
    motor.write(angle);
    lastangletime=currenttime;
  }

    state=digitalRead(HALL);              //Reads the hallsignal

    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=micros();
      if (currenttime==0){
      }
      else{
        v=(2*Omkrets*1000000)/((currenttime-lasttime)*17.53);    //multiply with e6 to get seconds, 17.53 magnets per wheel rotation men vi tog bort hälften av magneterna
        lasttime=currenttime;    

        if(v < 0.6*vold){             //För att slippa dipparna som sker en gång per varv av hjulen
          v=vold;
        }
        
        Serial.print(v);              //CSV format som kan kopieras i serialmonitor och sparas i ett textdokument.          
        Serial.print(',');
        Serial.print(currenttime);
        Serial.print(',');
        Serial.print(angle);
        Serial.print('\n');
        vold=v;
      }
    }
    oldstate=state;
    delayMicroseconds(ts);
}
