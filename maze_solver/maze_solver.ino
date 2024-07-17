//Pololu 3pi Robot
//3Pi 內建函數
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
unsigned int sensors[5]; // an array to hold sensor values
unsigned int whiteValue = 400;  //設定 white area value
unsigned int blackValue = 600;  //設定 black area value

char path[100] = "";  //設定線迷宮路徑的儲存空間,可調整參數***
unsigned char path_length = 0; //the length of the path
unsigned int rule_check = 0; // "0" Left Turn first, "1" Right Turn first

#include <avr/pgmspace.h>
//#include "C:\Users\d218\Documents\Arduino\_3piMazeSolver_Speed120\3pi_maze-solve_fun.h"  //For Win 7 system
#include "C:\Users\User\Desktop\_3piMazeSolver_SpeedABC_Loop_Ex\3pi_maze-solve_fun.h"  //For Win 7 system
//#include "C:\Documents and Settings\user\My Documents\Arduino\_3piMazeSolver_SpeedABC\3pi_maze-solve_fun.h" //For Win XP system
//3pi Maze Solve Functions file 3pi_maze-solve_fun.h for Win7, the path is different from Win XP
//=================================================================
void setup()
{
  unsigned int counter; // used as a simple timer

  robot.init(2000);
  pinMode(0, INPUT_PULLUP);

  load_custom_characters(); // load the custom characters

    // Play welcome music and display a message
  OrangutanLCD::printFromProgramSpace(welcome_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(welcome_line2);
  OrangutanBuzzer::playFromProgramSpace(welcome);
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::printFromProgramSpace(demo_name_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(demo_name_line2);
  delay(1000);

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }

  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);

  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    robot.calibrateLineSensors(IR_EMITTERS_ON);

    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(ANY_BUTTON))
  {
	unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    OrangutanLCD::clear();
    OrangutanLCD::print(position);
    OrangutanLCD::gotoXY(0, 1);
    display_readings(sensors);

    delay(100);
  }
  
  while(OrangutanPushbuttons::isPressed(BUTTON_A)){
	  rule_check = 0;
	  delay(500);
  }
  while(OrangutanPushbuttons::isPressed(BUTTON_C)){
	  rule_check = 1;
	  delay(500);
  }

  OrangutanPushbuttons::waitForRelease(ANY_BUTTON);

  OrangutanLCD::clear();

  OrangutanLCD::print("Go!");		

  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());
}
//===============================================================================================
void loop()
{
  while (1)
  {
    follow_segment();

    OrangutanMotors::setSpeeds(100, 100);
    delay(10);

    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // Now read the sensors and check the intersection type.
    //unsigned int sensors[5];
    robot.readLine(sensors, IR_EMITTERS_ON, 1);

    // Check for left and right exits.
    if (sensors[0] < whiteValue)
      found_left = 1;
    if (sensors[4] < whiteValue)
      found_right = 1;

    OrangutanMotors::setSpeeds(100, 100);
    delay(40);

    // Check for a straight exit.
    robot.readLine(sensors, IR_EMITTERS_ON, 1);
    if (sensors[1] < whiteValue || sensors[2] < whiteValue || sensors[3] < whiteValue)
      found_straight = 1;
    
    //finish
    if (sensors[0] < whiteValue && sensors[1] < whiteValue && sensors[2] < whiteValue && sensors[3] < whiteValue && sensors[4] < whiteValue)
      {
        OrangutanMotors::setSpeeds(120, 120);
        delay(90);
        break;
      }
      
    unsigned char dir = select_turn(found_left, found_straight, found_right, rule_check);

    turn(dir);

    path[path_length] = dir;
    path_length++;

    simplify_path();
//loop path's special solutions
  if ((path_length >= 3)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L'))
	rule_check = 1;//path = L L L change Rule to Right First
  if ((path_length >= 3)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R'))
	rule_check = 0;//path = R R R change Rule to Left First
  if ((path_length >= 3)&&(path[path_length - 1]=='S')&&(path[path_length - 2]=='S')&&(path[path_length - 3]=='L'))
	rule_check = 1;//path = S S L change Rule to Right First
  if ((path_length >= 3)&&(path[path_length - 1]=='S')&&(path[path_length - 2]=='S')&&(path[path_length - 3]=='R'))
	rule_check = 0;//path = S S R change Rule to Left First
  if ((path_length >= 4)&&(path[path_length - 1]=='B')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='L'))
	rule_check = 0;//path = B L L L change Rule to Left First
  if ((path_length >= 4)&&(path[path_length - 1]=='B')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='R'))
	rule_check = 1;//path = B R R R change Rule to Right First
  if ((path_length >= 4)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='R'))
	rule_check = 1;//path = L L L R change Rule to Right First
  if ((path_length >= 4)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='L'))
	rule_check = 0;//path = R R R L change Rule to Left First
  if ((path_length >= 4)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='L'))
	rule_check = 0;//path = L L L L change Rule to Left First
  if ((path_length >= 4)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='R'))
	rule_check = 1;//path = R R R R change Rule to Right First
  if ((path_length >= 5)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='L')&&(path[path_length - 5]=='L'))
	rule_check = 0;//path = R R L L L change Rule to Left First
  if ((path_length >= 5)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='R')&&(path[path_length - 5]=='R'))
	rule_check = 1;//path = L L R R R change Rule to Right First
  if ((path_length >= 5)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='L')&&(path[path_length - 5]=='L'))
	rule_check = 1;//path = L L L L L change Rule to Right First
  if ((path_length >= 5)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='R')&&(path[path_length - 5]=='R'))
	rule_check = 0;//path = R R R R R change Rule to Left First
  if ((path_length >= 5)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='S')&&(path[path_length - 4]=='L')&&(path[path_length - 5]=='R'))
	rule_check = 0;//path = L L S L R change Rule to Left First
  if ((path_length >= 5)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='S')&&(path[path_length - 4]=='R')&&(path[path_length - 5]=='L'))
	rule_check = 1;//path = R R S R L change Rule to Right First
  if ((path_length >= 5)&&(path[path_length - 1]=='L')&&(path[path_length - 2]=='L')&&(path[path_length - 3]=='L')&&(path[path_length - 4]=='R')&&(path[path_length - 5]=='L'))
	rule_check = 1;//path = L L L R L change Rule to Right First
  if ((path_length >= 5)&&(path[path_length - 1]=='R')&&(path[path_length - 2]=='R')&&(path[path_length - 3]=='R')&&(path[path_length - 4]=='L')&&(path[path_length - 5]=='R'))
	rule_check = 0;//path = R R R L R change Rule to Left First


  //Display the path on the LCD.
  //display_path();
  }
  // Solved the maze!
  while (1)
  {
    // Beep to show that we solved the maze.
    OrangutanMotors::setSpeeds(0, 0);
    OrangutanBuzzer::play(">>a32");

    while (!OrangutanPushbuttons::isPressed(ANY_BUTTON))
    {
      if (millis() % 2000 < 1000)
      {
        OrangutanLCD::clear();
        OrangutanLCD::print("Solved!");
        OrangutanLCD::gotoXY(0, 1);
        OrangutanLCD::print("PressBCA");
      }
      else
        display_path();
      delay(30);
    }
    while (OrangutanPushbuttons::isPressed(BUTTON_A)) {
    delay(1000);
    for (int i = 0; i < path_length; i++)
    {
      follow_segment_Hispeed();  //speed 180
      //     
      turn_Hispeed(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_segment_Hispeed();
    OrangutanMotors::setSpeeds(120, 120);
    delay(180);
    // Now we should be at the finish!  Restart the loop...
    }
//==========================================================================================
    while (OrangutanPushbuttons::isPressed(BUTTON_B)) {
    delay(1000);
    for (int i = 0; i < path_length; i++)
    {
      follow_segment_Hispeed_2();  //speed 200
      //     
      turn_Hispeed_2(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_segment_Hispeed_2();
    OrangutanMotors::setSpeeds(120, 120);
    delay(100);  // Now we should be at the finish!  Restart the loop...
    }
//====================================================================================================    
    while (OrangutanPushbuttons::isPressed(BUTTON_C)) {
    delay(1000);
    for (int i = 0; i < path_length; i++)
    {
      follow_segment_Hispeed_3();  //speed 220
      //     
      turn_Hispeed_3(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_segment_Hispeed_3();
    OrangutanMotors::setSpeeds(120, 120);
    delay(80);  // Now we should be at the finish!  Restart the loop...
    }
  }
}


