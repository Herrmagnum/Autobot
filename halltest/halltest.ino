int HALL=4;                               //pin för hall
int state=0;                              //signal från hall
double v=0;                               //Hastighet cm/s
double Omkrets=7;                         //Omkrets på hjulet cm
double utvxlig=3.1;                       //Utväxling mellan drivaxel och hjul
int ts=20;                                //sampletime  miliseconds
int antalMagnet=2;                        //Amount of magnets
int lasttime=0; 
int currenttime=0;
int state2=HIGH;
int oldstate=HIGH;
double d=Omkrets/antalMagnet;         //avstånd mellan magneterna
void setup() {
    pinMode(HALL, INPUT);
    Serial.begin(9600);

  
}

void loop() {
  // put your main code here, to run repeatedly:
    state=digitalRead(HALL);              //Reads the hallsignal
    Serial.print(v);                      //Only for testing
    Serial.print("_____");                //Only for testing
    Serial.print(state);                  //Only for testing
    Serial.print('\n');                   //Only for testing
   

    if (state==LOW && oldstate==HIGH){                        //Cheaks if the magnet is there
      currenttime=millis();
      v=(utvxlig*d*1000)/(currenttime-lasttime);    //multiply with tousend to get seconds instead of mS
      lasttime=currenttime;    
    }
    oldstate=state;
    delay(ts);
}
