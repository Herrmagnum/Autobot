

#define TP 7      //Trig_pin
#define EP 10      //Echo_pin
 
int obsticleavoid= 20; //avstond att börja undvika hindret
int sampletime = 30; // Microseconds OBS LJUDVÅGENS TID ÄR INTE MEDRÄKNAD
int led=4;                                                                             //ONLY FOR TESTING
void setup(){
  pinMode(TP,OUTPUT);       // set TP output for trigger  
  pinMode(EP,INPUT);        // set EP input for echo
  Serial.begin(9600);      // init serial 9600
  pinMode(led, OUTPUT);                                                                           //ONLY FOR TESTING
}

void loop(){  
  digitalWrite(led, LOW);                                                                           //ONLY FOR TESTING
  long microseconds = TP_init();
  long distacne_cm = Distance(microseconds);
  Serial.print("Distacne_CM = ");
  Serial.println(distacne_cm);
  
  if (distacne_cm < 1){  //DEBug sequence, denna verkar snabbast hittils men kanske transistor är att föredra
      pinMode(EP,OUTPUT);   
      digitalWrite(EP, LOW);
      pinMode(EP,INPUT);   
  }
  else if (distacne_cm < obsticleavoid){    // start avoidance 
   // Obsticle avoidance code
    Serial.print("Avoid ");                                                                           //ONLY FOR TESTING
    digitalWrite(led, HIGH);                                                                           //ONLY FOR TESTING
  }

  else{
  digitalWrite(led, LOW);                                                                           //ONLY FOR TESTING
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
