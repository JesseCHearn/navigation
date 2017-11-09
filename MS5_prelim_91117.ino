#define PI 3.1415926535 // pi
#include <dfr_tank.h>
#include <SoftwareSerial.h>
#include "Enes100.h"

// Global variables
float theta; // tank angle in rad
float targetTheta; // Enter the target angle in the space here
int minMotorValue = 160; // minimum input value which will produce a turning movement in the tank
float k = (255 - minMotorValue) / PI; // proportional controller for turning speed (“gain”)
float errTheta; // variable to hold the error in theta

float errX; // variable to hold the error in x
float errY; // variable to hold the error in y

int motorRate = 0;



int dist; // resistance value provided by distance sensor

float xMission; // variable to hold x location of mission site
float yMission; // variable to hold y location of mission site


float xPos; // tank x coordinate in m
float yPos; // tank y coordinate in m


boolean early = true;

/*
   establish motor control ports
*/
int leftD = 4;
int leftS = 5;
int rightS = 6;
int rightD = 7;


//IGNORE missionState
int missionState; // value which sets the OSV operational mode
/*
   missionState values
   0 in start orientation
   1 moving up and down in start
   2 moving left and right
   3 navigated
   4


   3 move forward until a specific y coord
   4 move forward until an obstacle is detected
   5 perform mission elements
*/

DFRTank tank;
Enes100 enes("JediArms", WATER, 114, 8, 9); //REPLACE INPUT3 WITH MARKER ID

void setup()
{
  Serial.begin(9600);
  //enes.println("RF WORKS");
  //  SoftwareSerial mySerial(8, 9);
  tank.init();

  enes.retrieveDestination(); // get site location
  xMission = enes.destination.x;
  yMission = enes.destination.y;

  enes.print("Mission y = ");
  enes.println(yMission);

  enes.updateLocation(); // get tank position
  theta = enes.location.theta;
  xPos = enes.location.x;
  yPos = enes.location.y;

  missionState = 5;
}
void loop()
{
  enes.updateLocation();
  theta = enes.location.theta;
  xPos = enes.location.x;
  yPos = enes.location.y;

  if (missionState == 5)
  {
    while (calcDistance() > .05)
    {
      if (dist > 300)
      {
        enes.println("OBSTACLE DETECTED");
        enes.updateLocation();
        xPos = enes.location.x;
        yPos = enes.location.y;
        if (yPos < 1) //tank below centerline
        {
          if (yMission < 1) //mission site below centerline
          {
            turnTo(3 * PI / 2);
            digitalWrite(leftD, HIGH);
            digitalWrite(rightD, HIGH);
            analogWrite(leftS, 255);
            analogWrite(rightS, 255);
            delay(300);
            analogWrite(leftS, 0);
            analogWrite(rightS, 0);
            delay(100);

          }
          else //mission site above centerline
          {
            turnTo(PI / 2);
            digitalWrite(leftD, HIGH);
            digitalWrite(rightD, HIGH);
            analogWrite(leftS, 255);
            analogWrite(rightS, 255);
            delay(300);
            analogWrite(leftS, 0);
            analogWrite(rightS, 0);
            delay(100);
          }
        }
        else //tank above centerline
        {
          if (yMission < 1) //mission site below centerline
          {
            turnTo(3 * PI / 2);
            digitalWrite(leftD, HIGH);
            digitalWrite(rightD, HIGH);
            analogWrite(leftS, 255);
            analogWrite(rightS, 255);
            delay(300);
            analogWrite(leftS, 0);
            analogWrite(rightS, 0);
            delay(100);
          }
          else //mission site above centerline
          {
            turnTo(PI / 2);
            digitalWrite(leftD, HIGH);
            digitalWrite(rightD, HIGH);
            analogWrite(leftS, 255);
            analogWrite(rightS, 255);
            delay(300);
            analogWrite(leftS, 0);
            analogWrite(rightS, 0);
            delay(100);
          }
        }
      }
      else
      {
        turnTo(calcHeading());
        digitalWrite(leftD, HIGH);
        digitalWrite(rightD, HIGH);
        analogWrite(leftS, 255);
        analogWrite(rightS, 255);
        delay(100);
      }


    }
    enes.navigated();
    analogWrite(leftS, 0);
    analogWrite(rightS, 0);
    missionState = -1;
  }




}








float calcHeading()
{
  float desiredHeading;
  enes.updateLocation();
  xPos = enes.location.x;
  yPos = enes.location.y;
  desiredHeading = atan((yMission - yPos) / (xMission - xPos));
  enes.print("Turning to ");
  enes.println(desiredHeading);
  return desiredHeading;
}

float calcDistance()
{
  float distance;
  enes.updateLocation();
  xPos = enes.location.x;
  yPos = enes.location.y;
  distance = sqrt(((yMission - yPos) * (yMission - yPos)) + ((xMission - xPos) * (xMission - xPos)));
  return distance;
}


/*
   turns the OSV to angle theta_desired
   when missionState == 1
*/

void turnTo(float thetaDesired)
{
  boolean turnCompleted = false;
  /*
       calculates the error in theta
  */
  while (!turnCompleted)
  {
    enes.updateLocation();
    theta = enes.location.theta;// update theta
    if ((theta - thetaDesired <= PI) && (theta - thetaDesired >= 0))
    {
      errTheta = (theta - thetaDesired); // compute error TURN RIGHT (POSITIVE)
    }
    else if (theta - thetaDesired > PI)
    {
      errTheta = -((2 * PI - theta) + thetaDesired); // compute error TURN LEFT (NEGATIVE)
    }
    else if ((theta - thetaDesired >= -PI) && (theta - thetaDesired < 0))
    {
      errTheta = (theta - thetaDesired); // compute error TURN LEFT (NEGATIVE)
    }
    else if (theta - thetaDesired < -PI)
    {
      errTheta = ((2 * PI - thetaDesired) + theta); // compute error TURN RIGHT (POSITIVE)
    }
    /*
       turns tank to desired angle theta
    */
    if (errTheta > 0.05 || errTheta < -0.05) // only compute if not converged
    {
      if (abs(errTheta) < .1)
      {
        motorRate = minMotorValue;
      }
      else
      {
        motorRate = minMotorValue + abs(errTheta) * k;

      }

      if (errTheta < 0)
      {
        digitalWrite(leftD, LOW);
        digitalWrite(rightD, HIGH);
      }
      else
      {
        digitalWrite(leftD, HIGH);
        digitalWrite(rightD, LOW);
      }

      analogWrite(leftS, motorRate);
      analogWrite(rightS, motorRate);
      enes.println(errTheta);
    }
    else
    {
      analogWrite(rightS, 0);
      analogWrite(leftS, 0);
      turnCompleted = true;
    }
    delay(250);
    analogWrite(rightS, 0);
    analogWrite(leftS, 0);
    delay(250);

    enes.updateLocation();
    theta = enes.location.theta;// update theta
  }

  enes.print("Reached target angle of ");
  enes.println(thetaDesired);

}

/*
   Steps:
  1 orient and get to proper y pos in starting zone
  2 travel straight in x direction
  A obstacle detection
  B obstacle avoidance
  C obstacle passing

  3 travel to mission site
*/

