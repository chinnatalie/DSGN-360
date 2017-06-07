#define WALLPIN_CLOSE 8
#define WALLPIN_FRONT 10

//Sonar 1
int echoPin1 =2;
int initPin1 =3;
int distance1 =0;

//Sonar 2
int echoPin2 =6;
int initPin2 =7;
int distance2 =0;

//Sonar 3
int echoPin3 =4;
int initPin3 =5;
int distance3 =0;

////Sonar 4
int echoPin4 =12;
int initPin4 =13;
int distance4 =0;

//Sonar 5
int echoPin5 = 9;
int initPin5 = 11;
int distance5 =0;

void setup() {
  
  pinMode(initPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(initPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(initPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(initPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(initPin5, OUTPUT);
  pinMode(echoPin5, INPUT);


  //output pins
  pinMode(WALLPIN_CLOSE, OUTPUT);
  pinMode(WALLPIN_FRONT, OUTPUT);
//  pinMode(WALLPIN_BACK, OUTPUT);  
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  
}

void loop() {
  
  distance1 = getDistance(initPin1, echoPin1);
  printDistance(1, distance1);
//  delay(10);
  
  distance2 = getDistance(initPin2, echoPin2);
  printDistance(2, distance2);
//  delay(10);
  
  distance3 = getDistance(initPin3, echoPin3);
  printDistance(3, distance3);
//  delay(150);
  
  distance4 = getDistance(initPin4, echoPin4);
  printDistance(4, distance4);
//  delay(10);

  distance5 = getDistance(initPin5, echoPin5);
  printDistance(5, distance5);
 
    if(distance1 < 10) {
      digitalWrite(13, HIGH);
      digitalWrite(WALLPIN_CLOSE, HIGH);
      Serial.println("Sonar 1 is close! ");
    }
    else {
      digitalWrite(13, LOW);
      digitalWrite(WALLPIN_CLOSE, LOW);
      Serial.println("Sonar 1 is far! ");
    }

    if(distance2 < 10) {
      digitalWrite(13, HIGH);
      digitalWrite(WALLPIN_FRONT, HIGH);
      Serial.println("Sonar 2 is close!");
    }
    else {
      digitalWrite(13, LOW);
      digitalWrite(WALLPIN_FRONT, LOW);
    }

    if(distance3 < 10) {
      digitalWrite(13, HIGH);
      digitalWrite(WALLPIN_FRONT, HIGH);
      Serial.println("Sonar 3 is close!");
    }
    else {
      digitalWrite(13, LOW);
      digitalWrite(WALLPIN_FRONT, LOW);
    }
        if(distance4 < 10) {
      digitalWrite(13, HIGH);
      digitalWrite(WALLPIN_FRONT, HIGH);
      Serial.println("Sonar 4 is close!");
    }
    else {
      digitalWrite(13, LOW);
      digitalWrite(WALLPIN_FRONT, LOW);
    }
        if(distance5 < 10) {
      digitalWrite(13, HIGH);
      digitalWrite(WALLPIN_FRONT, HIGH);
      Serial.println("Sonar 5 is close!");
    }
    else {
      digitalWrite(13, LOW);
      digitalWrite(WALLPIN_FRONT, LOW);
    }

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
    Serial.println(" cm");
    }
