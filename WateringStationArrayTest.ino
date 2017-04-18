int SensorPin = A2;
int PumpPin = 1;
int On = 0;
int Off = 255;
int NumberOfMeasurements = 10;
int SensorValueArray[10];
int SensorValue;
int PumpOnSuccessive = 0;
int PumpOnSuccessiveMax = 20;
int PumpOnValue = 100;

void setup() {
  /*Because the circuit uses inverted logic for the pump, the 
  PumpPin needs to be turned HIGH immediately. So that no unwanted
  watering will happen upon startup.*/
  analogWrite(PumpPin,Off);
  pinMode(PumpPin, OUTPUT);
  //Serial.begin(9600);
}

void loop () {
  /*Take the mean value of the moisturesensor to avoid noice*/
  SensorValueArray[NumberOfMeasurements];
  for (int i = 0; i <NumberOfMeasurements; i++){
    SensorValueArray[i] = analogRead(SensorPin);
    delay(500);
  }

  sort(SensorValueArray,NumberOfMeasurements);

  SensorValue = 0;
  for(int i = 2; i < (NumberOfMeasurements-2); i++){
    SensorValue = SensorValue + SensorValueArray[i];
  }
  SensorValue=SensorValue/(NumberOfMeasurements-4);
  //Serial.println(SensorValue);
  /*if the moisturesensor detects that the earth is to dry, then 
  squert water for 1,2 s.*/
  if (SensorValue <= PumpOnValue) {
    analogWrite(PumpPin,On);
    //Serial.println("Pump On!");
    delay(1500);
    analogWrite(PumpPin,Off);
    //Serial.println("Pump Off!");
    PumpOnSuccessive = PumpOnSuccessive + 1;
  }
  
  /*if the moisturesensor detects that the earth is moist enough, reset 
  number of times the pump have been on in a row (if it has been on).*/
  else {
    PumpOnSuccessive = 0;
  }

  /*if the pump have pumped water (int PumpOnSuccessiveMax) numbers
  in a row, then keep the program in a constant loop to awoid burning
  upp the pump (incase that the waterreservoar is empty).*/
  if (PumpOnSuccessive >= PumpOnSuccessiveMax) {
    //Serial.println("Pump on successive max reached!");
    while(1)
     ;
  }
  delay(20000);
}

void sort(int a[], int Sz) {
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
