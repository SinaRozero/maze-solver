//3pi Maze Solve Functions
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PROGMEM means the program space.
const char welcome_line1[] PROGMEM = "star sky";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Maze";
const char demo_name_line2[] PROGMEM = "solver";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";

const char levels[] PROGMEM = {
  0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000,
  0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
//=======================================================================
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}
// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  for (unsigned char i=0;i<5;i++) {    
    const char display_characters[10] = { 
      ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255     };

    char c = display_characters[calibrated_values[i] / 101];

    OrangutanLCD::print(c);
  }
}
//========================================================================
void display_path()  //LCD顯示線迷宮路徑,"LSR..."
{
  path[path_length] = 0;

  OrangutanLCD::clear();
  OrangutanLCD::print(path);

  if (path_length > 8)
  {
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print(path + 8);
  }
}
//========================================================================
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)	//Left or Right path  first rule
{
  if (rule_check == 0) {
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B'; }
else {
if (found_right)
    return 'R';
  else if (found_straight)
    return 'S';
  else if (found_left)
    return 'L';
  else
    return 'B';
  }
}
//========================================================================
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
 
  for (int i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}
//========================================================================
void follow_segment()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    int power_difference = proportional/8 + integral/10000 + derivative*2;

    const int maximum = 120; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] > blackValue && sensors[1] > blackValue && sensors[2] > blackValue && sensors[3] > blackValue && sensors[4] > blackValue)
    {
	// 感應器皆感應到黑色，表示死路，跳出此副程式//.
	return;
    }
    else if(sensors[0] < whiteValue || sensors[4] < whiteValue)
    {
        // 最左或最右的感應器感應到白色，表示要轉彎，跳出此副程式//.
	return;
    }
  }
}
//==============================================================================
void turn_follow_line()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    int proportional = ((int)position) - 2000;

    int derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    int power_difference = proportional/8 + integral/10000 + derivative*2;

    const int maximum = 120; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] > blackValue && sensors[1] < whiteValue && sensors[2] < whiteValue && sensors[3] < whiteValue && sensors[4] > blackValue)
    {
	//若最左和最右的感應器感應到黑色，且中間3個感應器感應到白色，表示回到正常軌道，則跳出此副程式
	return;
    }
  }
}
//===========================================================================================
void turn(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-80, 80);
    delay(180);
    turn_follow_line();
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(80, -80);
    delay(180);
    turn_follow_line();
    break;
  case 'B':
    // Turn around.
    OrangutanMotors::setSpeeds(80, -80);
    delay(380);
    turn_follow_line();
    break;
  case 'S':
    // Don't do anything!
    turn_follow_line();
    break;
  }
}
//---------------------------------------------------------------------------------------------------
void follow_segment_Hispeed()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    int power_difference = proportional/6 + integral/10000 + derivative*6;

    const int maximum = 200; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(digitalRead(0) == 0)
    {
        //延伸感應器感應到白色，表示要轉彎，跳出此副程式//.
	return;
    }
  }
}
//==============================================================================
void turn_follow_line_Hispeed()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    int proportional = ((int)position) - 2000;

    int derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    int power_difference = proportional/6 + integral/8000 + derivative*6;

    const int maximum = 195; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] > blackValue && sensors[1] < whiteValue && sensors[2] < whiteValue && sensors[3] < whiteValue && sensors[4] > blackValue)
    {
	//若最左和最右的感應器感應到黑色，且中間3個感應器感應到白色，表示回到正常軌道，則跳出此副程式
	return;
    }
  }
}
//===========================================================================================
void turn_Hispeed(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-20,160);
    delay(140);
    turn_follow_line_Hispeed();
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(160,-20);
    delay(140);
    turn_follow_line_Hispeed();
    break;
  case 'S':
    // Don't do anything!
    OrangutanMotors::setSpeeds(200, 200);
    delay(40);
    turn_follow_line_Hispeed();
    break;
  }
}
//---------------------------------------------------------------------------------------------------
void follow_segment_Hispeed_2()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    int power_difference = proportional/4 + integral/8000 + derivative*6;

    const int maximum = 190; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] < whiteValue || sensors[4] < whiteValue)
    {
        // 最左或最右的感應器感應到白色，表示要轉彎，跳出此副程式//.
	return;
    }
  }
}
//==============================================================================
void turn_follow_line_Hispeed_2()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    int proportional = ((int)position) - 2000;

    int derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    int power_difference = proportional/4 + integral/8000 + derivative*6;

    const int maximum = 185; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] > blackValue && sensors[1] < whiteValue && sensors[2] < whiteValue && sensors[3] < whiteValue && sensors[4] > blackValue)
    {
	//若最左和最右的感應器感應到黑色，且中間3個感應器感應到白色，表示回到正常軌道，則跳出此副程式
	return;
    }
  }
}
//===========================================================================================
void turn_Hispeed_2(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-120, 70);
    delay(170);
    turn_follow_line_Hispeed_2();
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(70, -120);
    delay(170);
    turn_follow_line_Hispeed_2();
    break;
  case 'S':
    // Don't do anything!
    OrangutanMotors::setSpeeds(180, 180);
    delay(20);
    turn_follow_line_Hispeed_2();
    break;
  }
}
//------------------------------------------------------------------------------------------
void follow_segment_Hispeed_3()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // Get the position of the line.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    int power_difference = proportional/3 + integral/8000 + derivative*6;

    const int maximum = 200; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] < whiteValue || sensors[4] < whiteValue)
    {
        // 最左或最右的感應器感應到白色，表示要轉彎，跳出此副程式//.
	return;
    }
  }
}
//==============================================================================
void turn_follow_line_Hispeed_3()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON, 1);

    int proportional = ((int)position) - 2000;

    int derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    int power_difference = proportional/3 + integral/8000 + derivative*6;

    const int maximum = 195; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

    if(sensors[0] > blackValue && sensors[1] < whiteValue && sensors[2] < whiteValue && sensors[3] < whiteValue && sensors[4] > blackValue)
    {
	//若最左和最右的感應器感應到黑色，且中間3個感應器感應到白色，表示回到正常軌道，則跳出此副程式
	return;
    }
  }
}
//===========================================================================================
void turn_Hispeed_3(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-80, 100);
    delay(165);
    turn_follow_line_Hispeed_3();
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(100, -80);
    delay(165);
    turn_follow_line_Hispeed_3();
    break;
  case 'S':
    // Don't do anything!
    OrangutanMotors::setSpeeds(200, 200);
    delay(40);
    turn_follow_line_Hispeed_3();
    break;
  }
}
//===============================================================================