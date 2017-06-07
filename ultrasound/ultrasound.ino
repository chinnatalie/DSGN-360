
int echoPin2 =10;
int initPin2 =3;
int distance2 =0;
int detectPin = 11;

void setup() {

  pinMode(detectPin, OUTPUT);
  pinMode(initPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(9600);
  
}

void loop() {

  distance2 = getDistance(initPin2, echoPin2);
  printDistance(2, distance2);

  if(distance2 > 10) {
    digitalWrite(detectPin, LOW);
    digitalWrite(13, LOW);
    }
  else {
    digitalWrite(detectPin, HIGH);
    digitalWrite(13, HIGH);
    }
  delay(10);
  
}

int getDistance (int initPin, int echoPin){

 digitalWrite(initPin, HIGH);
 delayMicroseconds(10); 
 digitalWrite(initPin, LOW); 
 unsigned long pulseTime = pulseIn(echoPin, HIGH); 
 int distance = pulseTime/58;
 return distance;
 
}
 
 void printDistance(int id, int dist){
  
     Serial.print(id);
    if (dist >= 120 || dist <= 0 ){
      Serial.println(" Out of range");
    }else
    for (int i = 0; i <= dist; i++) { 
         Serial.print("-");
    }
    Serial.print(dist, DEC);
    Serial.println(" cm");}
