void setup() {
  // put your setup code here, to run once:
int OP=9;
pinMode(OP, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:



  digitalWrite(OP, HIGH); 
  delay(1) ;
  digitalWrite(OP, LOW);
  delay(1000);

  
  digitalWrite(OP, HIGH); 
  delay(1.5) ;
  digitalWrite(OP, LOW);
  delay(1000);
}
