int HALL=4;                               //pin för hall
int state=0;                              //signal från hall
double v=0;                               //Hastighet cm/s
double Omkrets=20.4;                         //Omkrets på hjulet cm
int ts=200;                                //sampletime  microseconds
int lasttime=0; 
int currenttime=0;
int passeringar=0;            //For testing
int state2=HIGH;
int oldstate=HIGH;
double lowestspeed=1; // mät till denna hastighet, om lägre hastighet så aproximeras den tilll 0
double timeout= 1000*lowestspeed/(20.4/17.5) ;    //tiden som leder till hastigheten 0

void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600); 
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
        v=(Omkrets*1000)/((currenttime-lasttime)*17.53);    //multiply with tousend to get seconds instead of mS 17.53 magnets per wheel rotation
        lasttime=currenttime;    
        passeringar++;
  //      Serial.print(v);                      //Only for testing
  //     Serial.print(passeringar);                      //Only for testing
  //    Serial.print('\n');                   //Only for testing
      }
    }
    else if (millis > timeout+lasttime){
      v=0;
    }
    oldstate=state;
    delayMicroseconds(ts);
}
