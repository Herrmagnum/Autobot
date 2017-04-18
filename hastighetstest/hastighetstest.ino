# include <Servo.h>

int MotorPin = 12;
int HallPin = 4;
int NumberOfMeasurements = 20;
double VelocityArray[20];


int HALL=4;                               //pin för hall
int state=0;                              //signal från hall
double v;                               //Hastighet cm/s
double Omkrets=20.4;                         //Omkrets på hjulet cm
int ts=20;                                //sampletime  microseconds
int lasttime=0; 
int currenttime=0;
int passeringar=0;            //For testing
int state2=HIGH;
int oldstate=HIGH;
double lowestspeed=1; // mät till denna hastighet, om lägre hastighet så aproximeras den tilll 0
double timeout= 1000*lowestspeed/(20.4/17.5) ;    //tiden som leder till hastigheten 0
Servo motor;
int Mymotor = 12;
float angle = 90;
double speed2;

void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600); 
    pinMode(Mymotor,OUTPUT);            //sätter dc motor som output
  motor.attach(Mymotor); 
  angle=104;
  motor.write(angle);  
}

void loop() {
  // put your main code here, to run repeatedly:
    state=digitalRead(HALL);              //Reads the hallsignal
//    Serial.print(v);                      //Only for testing
//    Serial.print("_____");                //Only for testing
//    Serial.print(state);                  //Only for testing
//    Serial.print('\n');                   //Only for testing

    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=millis();
      if (currenttime==0){
      }
      else{
        v=v+(Omkrets*1000)/((currenttime-lasttime)*17.53);    //multiply with tousend to get seconds instead of mS 17.53 magnets per wheel rotation
        VelocityArray[passeringar]=v;
        lasttime=currenttime;    
        passeringar++;
        
  
      }
    }
    oldstate=state;

    if (passeringar >= NumberOfMeasurements){
      sort(VelocityArray,NumberOfMeasurements);
      speed2 = 0;
      for(int i = 4; i < (NumberOfMeasurements-4); i++){
        speed2 = speed2 + VelocityArray[i];
      }
      speed2=speed2/(NumberOfMeasurements-8);
            Serial.print(speed2);                      //Only for testing
  //     Serial.print(passeringar);                      //Only for testing
      Serial.print("\tcm/s");
      Serial.print('\n');                   //Only for testing
      v=0;
      passeringar=0;
    }
}

void sort(double a[], int Sz) {
    for(int i=0; i<(Sz-1); i++) {
        for(int o=0; o<(Sz-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    int t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}
