/*************************************
 * code for the hunted robot
 * as often as possible, read the opponent position from the xbee
 * the UART4 RX pin is 31 (second from the bottom left)
 * the xbee needs 3.3v, gnd, and dout
 * 
 * and read the vive position sensor on pin 24
 * 
*************************************/
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define V1PIN 24 // the signal from the vive sensor 1
#define V2PIN 10 // the signal from the vive sensor 2

#define DEG_PER_US 0.0216 // equal to (180 deg) / (8333 us)
#define LIGHTHOUSEHEIGHT 6.0 // in feet

#define MOVEPIN 12 // tell the robot to move if hunter is close
#define CWPIN 11 // tell robot which direction to move: HIGH for CW, CCW otherwise
 
// structure to store the sensor data
typedef struct {
  unsigned long changeTime[11];
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;

// variables for the sensor data and filter
volatile viveSensor V1;
volatile viveSensor V2;
int state = 0;

double xPos, yPos;
double xOld = 0, yOld = 0, xFilt = 0, yFilt = 0;

// variables for the xbee data
char msg[100];
int msg_index = 0;
float xOpponent, yOpponent;

//variables for inter-robot distance
double distance;
double x_1, y_1; //coordinates for corners to initialize
double x_2, y_2;
double x_3, y_3;
double x_4, y_4;

double bx_1, by_1; //coordinates for bucket points to initialize
double bx_2, by_2;
double bx_3, by_3;
double bx_4, by_4;
double bx_5, by_5;
double bx_6, by_6;
double bx_7, by_7;
double bx_8, by_8;

double b_distance1;
double b_distance2;
double b_distance3;
double b_distance4;
double b_distance5;
double b_distance6;
double b_distance7;
double b_distance8;

double xHistory[10];
double yHistory[10];
int historyIndex;
//double diagonal;

//double x1_2, y1_2; //midway point between 1 and 2
//double x2_3, y2_3;
//double x3_4, y3_4;
//double x4_1, y4_1;

//int currentQuarter;
//int temp_prevHunterQuarter;
//int prevHunterQuarter;
//int currHunterQuarter;
//bool escapeDirection = false; //true for CW, CCW otherwise

void setup() {
  x_1 = -5.6; //initialize 4 corners of map
  y_1 = -6.5;
  x_2 = 4.4;
  y_2 = -5.6;
  x_3 = 4;
  y_3 = 4;
  x_4 = -6.4;
  y_4 = 3.2;

  //TODO: initialize bucket points
  bx_1 = -3.8;
  by_1 = -5.7;
  bx_2 = -3.4;
  by_2 = -5.7;
  bx_3 = -3.3;
  by_3 = -4.9;
  bx_4 = -3.2;
  by_4 = -3.5;
  bx_5 = -4.0;
  by_5 = -3.6;
  bx_6 = -4.7;
  by_6 = -3.9;
  bx_7 = -5.3;
  by_7 = -4.6;
  bx_8 = -5.1;
  by_8 = -5.3;

  
//  x1_2 = (x1 + x2)/2;
//  y1_2 = (y1 + y2)/2;
//  x2_3 = (x2 + x3)/2;
//  y2_3 = (y2 + y3)/2;
//  x3_4 = (x3 + x4)/2;
//  y3_4 = (y3 + y4)/2;
//  x4_1 = (x4 + x1)/2;
//  y4_1 = (y4 + y1)/2;
  
//  diagonal = getDistance(x_1,y_1,x_3,y_3)/2.5; //compensate so robot doesn't move immediately
  
  Serial.begin(9600); // to talk to the computer
  Serial3.begin(9600); // to listen to the xbee
  pinMode(13, OUTPUT); // to blink the led on pin 13
  pinMode(V1PIN, INPUT); // to read the sensor
  pinMode(V2PIN, INPUT); // to read the sensor
  pinMode(MOVEPIN, OUTPUT);
  pinMode(CWPIN, OUTPUT);

  // initialize the sensor variables
  V1.horzAng = 0;
  V1.vertAng = 0;
  V1.useMe = 0;
  V1.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V1PIN), ISRV1, CHANGE);

  
  // initialize the sensor variables
  V2.horzAng = 0;
  V2.vertAng = 0;
  V2.useMe = 0;
  V2.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V2PIN), ISRV2, CHANGE);

  delay(5000);
}

void loop() {
  // see if the xbee has sent any data
//  temp_prevHunterQuarter = getQuarter(xOpponent, yOpponent);

  if(historyIndex == 9)
    historyIndex = 0;
  xHistory[historyIndex] = xOpponent;
  yHistory[historyIndex] = yOpponent;
  historyIndex++;
  
  
  if (Serial3.available() > 0) {
    msg[msg_index] = Serial3.read();
    // if you get a newline, there is data to read
    if (msg[msg_index] == '\n') {
      msg_index = 0;
      // data is in the format of two floats seperated by spaces
      sscanf(msg, "%f %f", &xOpponent, &yOpponent);

      Serial.print("op: ");
      Serial.print(xOpponent);
      Serial.print(" ");
      Serial.println(yOpponent);
    }
    else {
      // did not get a newline yet, just store for later
      msg_index++;
      if (msg_index == 100) {
        msg_index = 0;
      }
    }
  }

  // if the sensor data is new
  if (V1.useMe == 1) {
    V1.useMe = 0;

    // calculate the position and filter it
    xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    xFilt = xOld * 0.5 + xPos * 0.5;
    yFilt = yOld * 0.5 + yPos * 0.5;
    xOld = xFilt;
    yOld = yFilt;
    Serial.print("V1: ");
    Serial.print(xFilt);
    Serial.print(" ");
    Serial.println(yFilt);

    // blink the led so you can tell if you are getting sensor data
    digitalWrite(13, state);
    if (state == 1) {
      state = 0;
    }
    else {
      state = 1;
    }
  }

 // if the sensor data is new
  if (V2.useMe == 1) {
    V2.useMe = 0;

    // calculate the position and filter it
    xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
    xFilt = xOld * 0.5 + xPos * 0.5;
    yFilt = yOld * 0.5 + yPos * 0.5;
    xOld = xFilt;
    yOld = yFilt;
    Serial.print("V2: ");
    Serial.print(xFilt);
    Serial.print(" ");
    Serial.println(yFilt);

    // blink the led so you can tell if you are getting sensor data
    digitalWrite(13, state);
    if (state == 1) {
      state = 0;
    }
    else {
      state = 1;
    }
  }

    distance = getDistance(xFilt, yFilt, xOpponent, yOpponent); //get current distance to hunter
    b_distance1 = getDistance(bx_1, by_1, xOpponent, yOpponent); //get current distance between bucket point 1 and hunter
    b_distance2 = getDistance(bx_2, by_2, xOpponent, yOpponent); //get current distance between bucket point 2 and hunter
    b_distance3 = getDistance(bx_3, by_3, xOpponent, yOpponent); //get current distance between bucket point 3 and hunter
    b_distance4 = getDistance(bx_4, by_4, xOpponent, yOpponent); //get current distance between bucket point 4 and hunter
    b_distance5 = getDistance(bx_5, by_5, xOpponent, yOpponent); //get current distance between bucket point 5 and hunter
    b_distance6 = getDistance(bx_6, by_6, xOpponent, yOpponent); //get current distance between bucket point 6 and hunter
    b_distance7 = getDistance(bx_7, by_7, xOpponent, yOpponent); //get current distance between bucket point 7 and hunter
    b_distance8 = getDistance(bx_8, by_8, xOpponent, yOpponent); //get current distance between bucket point 8 and hunter

    double bucketdist[8];
    double maxdistance;
    bucketdist[0] = b_distance1;
    bucketdist[1] = b_distance2;
    bucketdist[2] = b_distance3;
    bucketdist[3] = b_distance4;
    bucketdist[4] = b_distance5;
    bucketdist[5] = b_distance6;
    bucketdist[6] = b_distance7;
    bucketdist[7] = b_distance8;

    for(int i=0; i<7; i++) {
        if(bucketdist[i]<bucketdist[i+1]){
            maxdistance = bucketdist[i+1];
          }
        else {
            maxdistance = bucketdist[i];
          }
      }

    if(distance > maxdistance) {
        digitalWrite(MOVEPIN, LOW);
        Serial.println("DON'T MOVE!");
      }
    else {
        digitalWrite(MOVEPIN, HIGH);
        if(xOpponent < xFilt && yOpponent < yFilt) {
          if(yOpponent > yHistory[0]){
            Serial.println("MOVE COUNTERCLOCKWISE!");
            digitalWrite(CWPIN, LOW);
            }
          else{
            Serial.println("MOVE CLOCKWISE!");
            digitalWrite(CWPIN, HIGH);
            }
          }
        else if(xOpponent < xFilt && yOpponent > yFilt) {
          if(yOpponent > yHistory[0]){
            Serial.println("MOVE COUNTERCLOCKWISE!");
            digitalWrite(CWPIN, LOW);
            }
          else{
            Serial.println("MOVE CLOCKWISE!");
            digitalWrite(CWPIN, HIGH);
            }
          }
        else if(xOpponent > xFilt && yOpponent < yFilt) {
          if(yOpponent > yHistory[0]){
            Serial.println("MOVE CLOCKWISE!");
            digitalWrite(CWPIN, HIGH);
            }

          else{
            Serial.println("MOVE COUNTERCLOCKWISE!");
            digitalWrite(CWPIN, LOW);
            }
          }
        else if(xOpponent > xFilt && yOpponent > yFilt) {
          if(yOpponent > yHistory[0]){
            Serial.println("MOVE CLOCKWISE!");
            digitalWrite(CWPIN, HIGH);
            }

          else{
            Serial.println("MOVE COUNTERCLOCKWISE!");
            digitalWrite(CWPIN, LOW);
            }

          }

      }

//  currentQuarter = getQuarter(xFilt, yFilt);
//  currHunterQuarter = getQuarter(xOpponent, yOpponent);
//
//  if(distance != 0) {
//    if(fabs(currentQuarter-currHunterQuarter) < 1 && distance < diagonal) {
//      prevHunterQuarter = temp_prevHunterQuarter;
////          Serial.print("Nathan Q: ");
////          Serial.print(currentQuarter);
////          Serial.print(" Hunter Q: ");
////          Serial.println(currHunterQuarter);
////          Serial.print("Distance between: ");
////          Serial.print(distance);
////          Serial.print(" Diagonal: ");
////          Serial.println(diagonal);
//      if((prevHunterQuarter-currentQuarter) < 0) {
//          digitalWrite(CWPIN, LOW);
//          escapeDirection = false;
//          Serial.println("ESCAPE IN CCW DIRECTION!");
//        }
//      else if ((prevHunterQuarter-currentQuarter) > 0) {
//          digitalWrite(CWPIN, HIGH);
//          escapeDirection = true;
//          Serial.println("ESCAPE IN CW DIRECTION!");
//        }
//      else {
////          Serial.println("SAME QUADRANT, RUN!");
//          if(escapeDirection)
//            digitalWrite(CWPIN, HIGH);
//          else if(!escapeDirection)
//            digitalWrite(CWPIN, LOW);
//          else {
//            Serial.println("Direction has something wrong! Just run CCW!");
//            digitalWrite(CWPIN, LOW);
//            }
//        }
//        
//      digitalWrite(MOVEPIN, HIGH);
//    }
//    else {
////      Serial.println("Hunter robot far away, stay put!");
//      digitalWrite(MOVEPIN, LOW);
//      }
//  }
}

int getQuarter(double x,double y) {
    int quarter = 0;
    double distance1, distance2, distance3, distance4;
    distance1 = getDistance(x,y,x_1,y_1); //distance to corner 1
    distance2 = getDistance(x,y,x_2,y_2);
    distance3 = getDistance(x,y,x_3,y_3);
    distance4 = getDistance(x,y,x_4,y_4);
    
    double distarray[4];
    distarray[0] = distance1;
    distarray[1] = distance2;
    distarray[2] = distance3;
    distarray[3] = distance4;
    double mindistance;
    
    for(int i = 0; i<3; i++) {
      if(distarray[i]<distarray[i+1]){
          mindistance = distarray[i];
        }
      else {
          mindistance = distarray[i+1];
        }
      }

    if(mindistance == distance1) {
      quarter = 1; //currently in quarter 1
      }
    else if(mindistance == distance2) {
      quarter = 2;
      }
    else if(mindistance == distance3) {
      quarter = 3;
      }
    else if(mindistance == distance4) {
      quarter = 4;
      }
     else {
      Serial.println("Something's not right!");
      }

      return quarter;
  }

double getDistance(double x1, double y1, double x2, double y2) {
    double out;
    out = sqrt (pow ( (x1 - x2), 2.0) + pow ( (y1 - y2) , 2.0) );
    return out;
  }

// the sensor interrupt
void ISRV1() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V1.changeTime[i] = V1.changeTime[i + 1];
  }
  V1.changeTime[10] = mic;

  // if the buffer is full
  if (V1.collected < 11) {
    V1.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V1.changeTime[1] - V1.changeTime[0] > 7000) && (V1.changeTime[3] - V1.changeTime[2] > 7000) && (V1.changeTime[6] - V1.changeTime[5] < 50) && (V1.changeTime[10] - V1.changeTime[9] < 50)) {
      V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_US;
      V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_US;
      V1.useMe = 1;
    }
  }
}

// the sensor interrupt
void ISRV2() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V2.changeTime[i] = V2.changeTime[i + 1];
  }
  V2.changeTime[10] = mic;

  // if the buffer is full
  if (V2.collected < 11) {
    V2.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V2.changeTime[1] - V2.changeTime[0] > 7000) && (V2.changeTime[3] - V2.changeTime[2] > 7000) && (V2.changeTime[6] - V2.changeTime[5] < 50) && (V2.changeTime[10] - V2.changeTime[9] < 50)) {
      V2.horzAng = (V2.changeTime[5] - V2.changeTime[4]) * DEG_PER_US;
      V2.vertAng = (V2.changeTime[9] - V2.changeTime[8]) * DEG_PER_US;
      V2.useMe = 1;
    }
  }
}




