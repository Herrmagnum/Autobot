

#define TP 0      //Trig_pin
#define EP 1      //Echo_pin
#define OP 4         //Outputpin
#define Tune A3       //pin for potensiometer
#define led  2       
 
int obsticleavoid= 20; //avstond att börja undvika hindret
int sampletime = 30; // Microseconds OBS LJUDVÅGENS TID ÄR INTE MEDRÄKNAD
int x=0;                                                                           
void setup(){
  pinMode(TP,OUTPUT);       // set TP output for trigger  
  pinMode(EP,INPUT);        // set EP input for echo
  pinMode(Tune,INPUT);        // set EP input for echo
  pinMode(led, OUTPUT);  
  pinMode(OP, OUTPUT);
  //Serial.begin(9600);      // init serial 9600
  
}

void loop(){  
  digitalWrite(led, LOW);                                                                           //ONLY FOR TESTING
  long microseconds = TP_init();
  long distacne_cm = Distance(microseconds);
  
 x=analogRead(Tune);
  obsticleavoid = map(x, 0, 1023, 15, 150);
  if (distacne_cm < 1){  //DEBug sequence, denna verkar snabbast hittils men kanske transistor är att föredra
      pinMode(EP,OUTPUT);   
      digitalWrite(EP, LOW);
      pinMode(EP,INPUT);   
  }
  else if (distacne_cm < obsticleavoid){    // start avoidance 
   // Obsticle avoidance code
    digitalWrite(OP, HIGH);                                                                        
    digitalWrite(led, HIGH); 
                                                                             
  }

  else{
  digitalWrite(OP, LOW); 
  digitalWrite(led, LOW);                                                                   
  }
  delay(sampletime);
}

long Distance(long time)
{
  long distacne;
    distacne = time /29 / 2  ;     // Distance_CM  = ((Duration of high level)*(Sonic :340m/s))/2
  return distacne;
}
long TP_init()
{             
  digitalWrite(TP, LOW);                    
  delayMicroseconds(2);
  digitalWrite(TP, HIGH);                 // pull the Trig pin to high level for more than 10us impulse 
  delayMicroseconds(10);
  digitalWrite(TP, LOW);
  long microseconds = pulseIn(EP,HIGH);   // waits for the pin to go HIGH, and returns the length of the pulse in microseconds
  return microseconds;   // return microseconds
}
