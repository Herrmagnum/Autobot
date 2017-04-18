# include <Servo.h>

int HALL=4;                               //pin för hall
int state=0;                              //signal från hall
double v=0;                               //Hastighet cm/s
double Omkrets=20.4;                         //Omkrets på hjulet cm
int ts=10;                                //sampletime  microseconds
unsigned long lastsampletime=0; 
unsigned long lasttime=0;
unsigned long currenttime=0;
//int passeringar=0;            //For testing
int state2=HIGH;
int oldstate=HIGH;
//double lowestspeed=1; // mät till denna hastighet, om lägre hastighet så aproximeras den tilll 0
//double timeout= 1000*lowestspeed/(20.4/17.5) ;    //tiden som leder till hastigheten 0
int inputintervall=1000;
 Servo motor;
int Mymotor = 12;
float angle = 90;
void setup() {
  pinMode(Mymotor,OUTPUT);            //sätter dc motor som output
  motor.attach(Mymotor);             
  Serial.begin(9600);
}

void loop() {
 currenttime=millis();
 
if((currenttime-lasttime) > inputintervall) {
 // angle=random(75, 90);
 angle=102;
  motor.write(angle);
  lasttime=currenttime;
}

else if((currenttime-lastsampletime) > ts){
  state=digitalRead(HALL);              //Reads the hallsignal

    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=millis();
      if (currenttime==0){
      }
      else{
        v=(Omkrets*1000)/((currenttime-lastsampletime)*17.53);    //multiply with tousend to get seconds instead of mS 17.53 magnets per wheel rotation
        lastsampletime=currenttime;    
   //     passeringar++;
   //     Serial.print(v);                      //Only for testing
  //     Serial.print(passeringar);                      //Only for testing
  //    Serial.print('\n');                   //Only for testing
      }
    }
//    else if (millis > (timeout+lasttime)){
//      v=0;
//    }
    oldstate=state;
}

Serial.print(v);
Serial.print(',');
Serial.print(currenttime);
Serial.print(',');
Serial.print(angle);
Serial.print(',');
  Serial.print('\n');

}

//void ReadSpeed{
//    
//    //delayMicroseconds(ts);
//    return (v);
//}



