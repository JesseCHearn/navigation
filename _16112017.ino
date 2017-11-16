#define PI 3.1415926535 // pi
#include <dfr_tank.h>
#include <SoftwareSerial.h>
#include "Enes100.h"

// Global variables

DFRTank tank;
Enes100 enes("JediArms", WATER, 4, 8, 9); //REPLACE INPUT3 WITH MARKER ID

int minMotorValue = 140; // minimum input value which will produce a turning movement in the tank
float k = (255 - minMotorValue) / PI; // proportional controller for turning speed (“gain”)

int dist; // resistance value provided by distance sensor
int dist1;

float xMission; // variable to hold x location of mission site
float yMission; // variable to hold y location of mission site

//OSV position variables
float xPos; // OSV x coordinate in m
float yPos; // OSV y coordinate in m
float theta; // OSV angle in rad
boolean obstacle=false;
/*
   establish motor control ports
*/
int leftD = 4;
int leftS = 5;
int rightS = 6;
int rightD = 7;

void setup()
{
  Serial.begin(9600);
  tank.init();

  enes.retrieveDestination(); // get site location
  xMission = enes.destination.x;
  yMission = enes.destination.y;

  checkPosition(); //get tank position
}

void loop()
{
  checkPosition();
  if (calcDistance() > .15)
  {
    dist = analogRead(6);
    dist1 = analogRead(1);
    if (dist > 200 || dist1>200)//%%%%%%%%Need to go forward over rocky terrain. No stopping. (Add and to if statement)
    {
      enes.println("OBSTACLE DETECTED");
      stopMoving();
      checkPosition();
      obstacle=true;

      if (yPos < 1) //tank below centerline
      {
        enes.println("yPos<1");
        if (yMission < 1) //mission site below centerline
        {
          enes.println("yMission<1");
          turnTo(3 * PI / 2);
          moveForward(255);
          delay(1900);
          stopMoving();
          delay(100);

        }
        else //mission site above centerline
        {
          enes.println("yMission>1");
          turnTo(PI / 2);
          moveForward(255);
          delay(1900);
          stopMoving();
          delay(100);
        }
      }
      else //tank above centerline
      {
        enes.println("yPos>1");
        if (yMission < 1) //mission site below centerline
        {
          enes.println("yMission<1");
          turnTo(3 * PI / 2);
          moveForward(255);
          delay(1900);
          stopMoving();
          delay(100);
        }
        else //mission site above centerline
        {
          enes.println("yMission>1");
          turnTo(PI / 2);
          moveForward(255);
          delay(1900);
          stopMoving();
          delay(100);
        }
      }
      checkPosition();
      turnTo(0);
    }
    else
    {
      if(obstacle){
        moveForward(255);
        delay(1000);
        stopMoving();
        delay(100);
        obstacle=false;
      }
      turnTo(calcHeading());
      moveForward(150);
      delay(200);
    }
  }
  else
  {
    enes.navigated();
    stopMoving();
  }

}

/*
   takes power input
   where minMotorValue<=power<=255
   moves OSV forward with power
*/

void moveForward(int power)
{
  digitalWrite(leftD, HIGH);
  digitalWrite(rightD, HIGH);
  analogWrite(leftS, power);
  analogWrite(rightS, power);
}

/*
   takes power input
   where minMotorValue<=power<=255
   moves OSV backward with power
*/

/*void moveBack(int power)
  {
  digitalWrite(leftD, LOW);
  digitalWrite(rightD, LOW);
  analogWrite(leftS, power);
  analogWrite(rightS, power);
  }

  /*
   halts OSV drivetrain movement
*/

void stopMoving()
{
  analogWrite(leftS, 0);
  analogWrite(rightS, 0);
}

/*
   gets tank position from the Enes100 vision system library
*/

void checkPosition()
{
  enes.updateLocation();
  theta = enes.location.theta;
  xPos = enes.location.x;
  yPos = enes.location.y;
}

/*
   returns the heading angle that the OSV should take in order to point towards the mission site
*/

float calcHeading()
{
  float desiredHeading;
  enes.updateLocation();
  xPos = enes.location.x;
  yPos = enes.location.y;
  if (xPos > xMission) {
    desiredHeading = (PI) - (atan((yMission - yPos) / (xMission - xPos)));
  }
  else {
    desiredHeading = atan((yMission - yPos) / (xMission - xPos));
  }
  enes.print("Turning to ");
  enes.println(desiredHeading);
  return desiredHeading;
}

/*
   returns distance between the OSV and the destination site using pythagorean theorem
*/

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
   turns the OSV to angle thetaDesired
   uses proportional error correction
*/

void turnTo(float thetaDesired)
{
  float errTheta; // variable to hold the error in theta
  int motorRate = 0; //variable to hold motor turn rate
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
