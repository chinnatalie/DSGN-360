#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>
#include <OrangutanLCD.h>
#include <OrangutanLEDs.h>
#include <OrangutanAnalog.h>
#include <OrangutanMotors.h>

OrangutanLCD lcd;
OrangutanBuzzer buzzer;
Pololu3pi robot;
unsigned int sensors(5);

void display_readings(const unsigned int *calibrated_values) {
  unsigned char i;
  for (i = 0; i < 5; i++) {
    const char display_characters[10] = {' ', 0, 1, 2, 3, 4, 5, 6, 255};
    char c = display_characters[calibrated_values[i]/101];
    OrangutanLCD::print(c);
  }
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

  unsigned int counter;

  for (counter = 0; counter < 80; counter++) {
    if (counter < 20 || counter >= 60) {
      OrangutanMotors::setSpeeds(40, -40);
    } else {
      OrangutanMotors::setSpeeds(-40, 40);
    }
    robot.calibrateLineSensors(IR_EMITTERS_ON);
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
  while (!OrangutanPushbuttons::isPressed(BUTTON_B)) {
    robot.readLine(&sensors, IR_EMITTERS_ON);
    lcd.clear();
    //lcd.print(position);
    lcd.gotoXY(0, 1);
    display_readings(&sensors);
    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /*unsigned int sensors[5];
  lcd.clear();
  lcd.print("reset");
  delay(500);

  unsigned char i;

  for (int i = 0; i < 5; i++) {
    lcd.clear();
    lcd.print(i);
    lcd.print("-");
    lcd.print(sensors[i]);
    delay(500);
  }*/
  //motors.setSpeeds(200, 200);
}
