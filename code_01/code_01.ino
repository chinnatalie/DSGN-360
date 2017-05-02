#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanDigital.h>
#include <OrangutanBuzzer.h>
#include <OrangutanLCD.h>
#include <OrangutanLEDs.h>
#include <OrangutanMotors.h>

const int maxSpeed = 100;
const int turnSpeed = 70;
long randomNumber;

OrangutanLCD lcd;
OrangutanBuzzer buzzer;
OrangutanMotors motors;
OrangutanPushbuttons buttons;
Pololu3pi robot;
bool nearObstacle = false;

void rotateLeft(int spd, int duration) {
  motors.setSpeeds(-spd, spd);
  delay(duration);
}
void rotateRight(int spd, int duration) {
  motors.setSpeeds(spd, -spd);
  delay(duration);
}

void setup() {
  // put your setup code here, to run once:
  buzzer.playNote(NOTE_C(5), 200, 15);
  lcd.print("Hi. I'm");
  lcd.gotoXY(0, 1);
  lcd.print("lil' Jon!");
  delay(2000);
  buzzer.playNote(NOTE_G(6), 300, 15);
  robot.init();
  OrangutanDigital::setInput(IO_D0, PULL_UP_ENABLED);
}

void loop() {
  nearObstacle = false;
  // put your main code here, to run repeatedly:
  if (OrangutanDigital::isInputHigh(IO_D0)) {
    OrangutanDigital::setOutput(IO_D1, HIGH);
    nearObstacle = true;
  } else {
    OrangutanDigital::setOutput(IO_D1, LOW);
    
  }
  unsigned char button = OrangutanPushbuttons::getSingleDebouncedPress(ANY_BUTTON);
  if (button) {
    lcd.clear();
    lcd.print("STOP");
    motors.setSpeeds(0,0);
    delay(100000);
  }
  if (!nearObstacle) {
    lcd.clear();
    lcd.print("CLEAR");
    motors.setSpeeds(100, 100);
    delay(200);
  } else {
    randomNumber = random(2);
    lcd.clear();
    if (randomNumber == 1){
      lcd.print("LOOK R");
      rotateLeft(turnSpeed, 600);
    } else {
      lcd.print("LOOK L");
      rotateRight(turnSpeed, 600);
    }
    nearObstacle = false;
  }
}
