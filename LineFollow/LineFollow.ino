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


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 120;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;


int minValues[5], maxValues[5], threshold[5];

void setup()
{
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}


void loop()
{
  while (digitalRead(11)==0) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)==0) {}
  delay(1000);

  while (1)
  {
    if (analogRead(0) < threshold[0] && analogRead(4) > threshold[4] )
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }

    else if (analogRead(4) < threshold[4] && analogRead(0) > threshold[0])
    { lsp = lfspeed; rsp = 0;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }
    else if (analogRead(2) < threshold[2])
    {
      Kp = 0.000008 * (1000 - analogRead(2));  //Kp=0.000009 + Ki=0.0005
      Kd = 10 * Kp;
      // Ki = 0.0005;
      linefollow();
    }
  }
}

void linefollow()
{
  int error = (analogRead(1) - analogRead(3));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);

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
    motor1.drive(60);
    motor2.drive(-60);

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
  
  motor1.drive(0);
  motor2.drive(0);
}