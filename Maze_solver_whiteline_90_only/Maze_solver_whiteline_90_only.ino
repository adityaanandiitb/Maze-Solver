#include <SparkFun_TB6612.h>

#define PWMB 3
#define BIN2 4
#define BIN1 5
#define STBY 6
#define AIN1 7
#define AIN2 9
#define PWMA 10



// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


  //  Motor Right Dir- 7
  //  Motor Right PWM- 9
  //  Motor Left Dir- 8
  //  Motor Left PWM- 10
  //  RGB- 6,5,3

  //  IR sensor on A0,A1,A2,A3,A4
  //  A0-leftmost sensor & A4 - right most sensor

// const int startButton = 11;

bool l = 0;
bool r = 0;
bool s = 0;
bool u = 0;
int e = 0;
int paths = 0;

bool endFound = 0;

// int blackValue = 900;    // will modify later accordng to surface and can also make a calibration function which will detect the minimum and maximum value by itself and then find the mean
// int whiteValue = 100;
// int threshold = 660;
//int threshold = (blackValue + whiteValue) * 0.5;
int FT = 20;
int P, I, D, previousError, PIDvalue, error;
int lsp = 0;  // max 255 tak kar sakte hai
int rsp = 0;
int lfspeed = 120;  // line following speed
int turnspeed = lfspeed * 0.6;
float Kp = 0;
float Kd = 0;
float Ki = 0;
int minValues[5], maxValues[5], threshold[5];
String str;

void setup() {
  Serial.begin(9600);
  pinMode(STBY,OUTPUT);
  pinMode(11, INPUT_PULLUP);   //after pressing this pin calibration will start
  pinMode(12, INPUT_PULLUP);   //LSR
  pinMode(13, INPUT_PULLUP);   //RSL

  pinMode (4, OUTPUT);      // AIN1/2
  pinMode (5, OUTPUT);      // BIN1/2
  pinMode (7, OUTPUT);  
  pinMode (9, OUTPUT);

  pinMode (3, OUTPUT);      //PWMB
  pinMode (10, OUTPUT);     //PWMA

  pinMode (2, OUTPUT);       //red
   //turn speed ko thoda kam rakhna hai compared to line following speed
  
}

void loop() {

  while (digitalRead(11) == 0)
  { //wait for button press 
  }  
  delay(1000);

  calibrate();

  while (digitalRead(12) == 0)
  { //wait for button press
  }
  delay(1000);


// this part of code is for dry run
  while (endFound == 0)   
  {
    linefollow();
    checknode();
    botstop();
    delay(100);
    reposition ();
  }
// From here the Actual run code starts 
  for (int x = 0; x < 4; x++)  
  {
    str.replace("LULUS", "U");
    str.replace("LUSUL", "U");
    str.replace("LUL", "S");
    str.replace("SUL", "R");
    str.replace("LUS", "R");
    str.replace("RUL", "U");
  }
  int endpos = str.indexOf('E');

  while (digitalRead(13) == 0)
  { //wait for button press
  }
  delay(1000);

  for (int i = 0; i <= endpos; i++)
  {
    char node = str.charAt(i);
    paths = 0;
    while (paths < 2)
    {
      linefollow();
      checknode();
      if (paths == 1)
      {
        reposition();
      }
    }
    switch (node)
    {
      case 'L':
        botstop();
        delay(50);
        botleft();
        break;

      case 'S':
        break;

      case 'R':
        botstop();
        delay(50);
        botright();
        break;

      case 'E':
        for (int i = 0; i < 50; i++)
        {
          botinchforward ();
        }
        red();
        botstop();
        delay(1000);
        break;
    }//_________end of switch
  }//_________end of for loop

}

void showSensorData()
{
  Serial.print("  Sensor 0-  ");
  Serial.print(analogRead(0));
  Serial.print("  Sensor 1-  ");
  Serial.print(analogRead(1));
  Serial.print("  Sensor 2-  ");
  Serial.print(analogRead(2));
  Serial.print("  Sensor 3-  ");
  Serial.print(analogRead(3));
  Serial.print("  Sensor 4-  ");
  Serial.println(analogRead(4));
}

void calibrate()
{
  for ( int i = 0; i < 5; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 5000; i++)
  {
    motor1.drive(70);     //rotate the bot at a fix point
    motor2.drive(-70);

    for ( int i = 0; i < 5; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 0; i < 5; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0); //stop the motors
  motor2.drive(0);
}


void linefollow()
{ //green () ;
  paths = 0;
  // pid is in loop so the bot keeps on moving and the value keeps on overwriting
  while ((analogRead(0) > threshold[0] ) && (analogRead(4) > threshold[4] ) && (analogRead(2) < threshold[2]))
  {
    Kp = 0.000004 * (1000 - analogRead(2));  //Kp=0.000009 + Ki=0.0005
    Kd = 10 * Kp;
    Ki = 0.005;
    PID();
  }
  lightsoff();
}




void PID()
{
  int error = analogRead(1) - analogRead(3);

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 200) {
    lsp = 200;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 200) {
    rsp = 200;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);
  // analogWrite(9, lsp);
  // analogWrite(10, rsp);
}


void checknode ()
{
  // yellow();
  l = 0;
  r = 0;
  s = 0;
  u = 0;
  e = 0;
  paths = 0;

  // checks whether bot is on node and the number of directions possible


  if (analogRead(4) < threshold[4]) r = 1;
  if (analogRead(0) < threshold[0]) l = 1;
  if ((analogRead(0) > threshold[0] && (analogRead(4) > threshold[4]) && (analogRead(2) > threshold[2]))) {
    u = 1;
  }
  if ((analogRead(2) < threshold[2]) && (analogRead(3) < threshold[3]) && (analogRead(4) < threshold[4]) && (analogRead(0) < threshold[0]) && (analogRead(1) < threshold[1])) {
    e = 1; // all on white
  }

  if (u == 0)         //if uturn not possible then left + right + straight
  {
    for (int i = 0; i < FT; i++)   // if after moving a "bit" forward we detect no white line
    {
      // botinchforward ();
      PID();
      if (analogRead(4) < threshold[4]) r = 1;
      if (analogRead(0) < threshold[0]) l = 1;
    }

    for (int i = 0; i < FT; i++)
    { 
      // cyan();
      // botinchforward ();
      PID();
      if (analogRead(2) < threshold[2]) s = 1; 
      if ((e == 1) && (analogRead(3) < threshold[3]) && (analogRead(4) < threshold[4]) && (analogRead(2) < threshold[2]) && (analogRead(0) < threshold[0]) && (analogRead(1) < threshold[1])) e = 2;  // if after moving a bit forward still e=1 then end has occured

    }
  }
  if (u == 1)
  {
    for (int i = 0; i < 20; i++)
    {
      botinchforward ();
    }
  }

  paths = l + s + r;

}





void reposition()
{
  // blue();
  if (e == 2)
  {
    str += 'E';
    endFound = 1;
    red();
    botstop();
    delay(2000);
  }


//if more than one path
  

//turn left if possible
  else if (l == 1)        
  {
    if (paths > 1) str += 'L';
    botleft(); 
  }


//else take straight if possible, no repositioning req.
  else if (s == 1)
  {
    if (paths > 1) str += 'S';
  }


//else turn right
  else if (r == 1)
  {
    if (paths > 1) str += 'R';
    botright(); //take right
  }

  else if (u == 1)
  {
    str += 'U';
    botuturn(); //take left u turn
  }
  lightsoff();

}



void botleft ()
{
    motor1.drive(0);
    motor2.drive(lfspeed);
  // motor1.drive()
  // digitalWrite(8, HIGH);
  // digitalWrite(8, LOW);
  // analogWrite(9, lfspeed);
  // analogWrite(10, lfspeed);
  delay(200);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(0);
    motor2.drive(lfspeed);
  }
  analogWrite(3, 0);   /// Doubt
  analogWrite(10, 0);
  delay(50);
}

void botright ()
{
    motor1.drive(lfspeed);
    motor2.drive(0);
  // digitalWrite(7, LOW);
  // digitalWrite(8, HIGH);
  // analogWrite(9, lfspeed );
  // analogWrite(10, lfspeed);
  delay(200);
  while (analogRead(2) > threshold[2])
  {
      motor1.drive(lfspeed);
      motor2.drive(0);
    // digitalWrite(7, LOW);
    // digitalWrite(8, HIGH);
    // analogWrite(9, lfspeed );
    // analogWrite(10, lfspeed);

  }
  analogWrite(3, 0);
  analogWrite(10, 0);
  delay(50);
}

void botstraight ()
{
  motor1.drive(lfspeed);
  motor2.drive(lfspeed);
  // digitalWrite(7, HIGH);
  // digitalWrite(8, HIGH);
  // analogWrite(9, lfspeed);
  // analogWrite(10, lfspeed);
}

void botinchforward ()
{
  motor1.drive(turnspeed);
  motor2.drive(turnspeed);
  // digitalWrite(7, HIGH);
  // digitalWrite(8, HIGH);
  // analogWrite(9, turnspeed);
  // analogWrite(10, turnspeed);
  delay(10);
}
void botstop ()
{
  motor1.drive(0);
  motor2.drive(0);

//   digitalWrite(7, HIGH);
//   digitalWrite(8, HIGH);
//   analogWrite(9, 0);
//   analogWrite(10, 0);
}
void botuturn ()
{
  motor1.drive(lfspeed);
  motor2.drive(lfspeed * 0.8);
  // digitalWrite(7, HIGH);
  // digitalWrite(8, LOW);
  // analogWrite(9, lfspeed * 0.8 );
  // analogWrite(10, lfspeed);
  delay(400);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(lfspeed);
    motor2.drive(lfspeed * 0.8);

    // digitalWrite(7, HIGH);
    // digitalWrite(8, LOW);
    // analogWrite(3, lfspeed * 0.8);
    // analogWrite(10, lfspeed);
  }
  analogWrite(3, 0);
  analogWrite(10, 0);
  delay(50);
}

void red ()
{
  digitalWrite (2, HIGH);
}

void lightsoff()
{
  digitalWrite (2, LOW);

}