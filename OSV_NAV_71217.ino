#define PI 3.1415926535 // pi
#include <dfr_tank.h>
#include <SoftwareSerial.h>
#include "Enes100.h"

// Global variables


//USER INPUTS
int markerID = 103;         //INPUT MARKER ID
int minMotorValue = 220;  //INPUT MINIMUM MOTOR VALUE FOR TURNING
//minimum input value which will produce a turning movement in the OSV


Enes100 enes("JediArms", WATER, markerID, 8, 9);

float k = (255 - minMotorValue) / PI; // proportional controller for turning speed (“gain”)
float p = (255 - minMotorValue); // proportional controller for moving speed (“gain”)


int dist; // resistance value provided by left distance sensor
int dist1;// resistance value provided by right distance sensor

float xMission; // variable to hold x location of mission site
float yMission; // variable to hold y location of mission site

//OSV position variables
float xPos; // OSV x coordinate in m
float yPos; // OSV y coordinate in m
float theta; // OSV angle in rad

int powerValue=0;

int missionState = 0;

boolean obstacle = false; //boolean used in obstacle avoidance

/*
   establish motor control ports
*/
int rightBackward = 2;
int rightForward = 3;
int rightPower = 5;
int leftForward = 4;
int leftBackward = 7;
int leftPower = 6;

void setup()
{
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);//left backwards
  pinMode(5, OUTPUT);//left forwards
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);//right forwards
  pinMode(2, OUTPUT);//right backwards

  Serial.begin(9600);

  enes.retrieveDestination(); // get mission site location
  xMission = enes.destination.x; //set x coordinate of mission site
  yMission = enes.destination.y; //set y coordinate of mission site

  checkPosition(); //get tank position
}

void loop()
{
  printValues();
  if (missionState == 0)
  {
    checkPosition();
    if (calcDistance() > .25)
    {
      dist = analogRead(2);
      dist1 = analogRead(3);
      printValues();
      delay(1000);
      if ((dist < 800 || dist1 < 800) && (xPos > 1.0)) //%%%%%%%%Need to go forward over rocky terrain. No stopping. (Add and to if statement)
      {
        enes.println("OBSTACLE DETECTED");
        stopMoving();
        checkPosition();
        driveToX(.20, xPos, false);
        checkPosition();
        obstacle = true;//variable to hold whether an obstacle has been detected

        if (yPos < 1) //tank below centerline
        {
          enes.println("yPos<1");
          if (yMission < 1) //mission site below centerline
          {
            enes.println("yMission<1");
            turnTo(3 * PI / 2);
            /*
              moveForward(255);
              delay(1900);
              stopMoving();
              delay(100);
            */

          }
          else //mission site above centerline
          {
            enes.println("yMission>1");
            turnTo(PI / 2);
            /*
              moveForward(255);
              delay(1900);
              stopMoving();
              delay(100);
            */
          }
        }
        else //tank above centerline
        {
          enes.println("yPos>1");
          if (yMission < 1) //mission site below centerline
          {
            enes.println("yMission<1");
            turnTo(3 * PI / 2);
            /*
              moveForward(255);
              delay(1900);
              stopMoving();
              delay(100);
            */
          }
          else //mission site above centerline
          {
            enes.println("yMission>1");
            turnTo(PI / 2);
            /*
              moveForward(255);
              delay(1900);
              stopMoving();
              delay(100);
            */
          }
        }
        checkPosition();
        driveToY(.2, yPos);
        checkPosition();
        turnTo(0);
        printValues();
      }
      else
      {
        if (obstacle) { //if an obstacle was detected, move forward past it
          checkPosition();
          driveToX(.6,xPos,true);
          //delay(1000);
          //stopMoving();
          //delay(100);
          obstacle = false;
        }
        if ((xPos > 1.0) || (xPos < .45))
        {
          turnTo(calcHeading());
          moveForward(255);
          delay(200);
        }
        else
        {
          moveForward(255);
        }

      }
    }
    else
    {
      enes.navigated();
      stopMoving();
      missionState = 1;
    }
  }
  else if (missionState == 1)
  {
    enes.println("DOING MISSION STUFF");
  }

}

/*
   takes power input
   where minMotorValue<=power<=255
   moves OSV forward with power
*/

void moveForward(int power)
{
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftBackward, LOW);
  analogWrite(leftPower, power);
  analogWrite(rightPower, power);
}

void printValues() {
  enes.print("Left Sensor: ");
  enes.print(analogRead(0));
  enes.print("        Right Sensor: ");
  enes.println(analogRead(1));
}

/*
   takes power input
   where minMotorValue<=power<=255
   moves OSV backward with power
*/



void moveBackward(int power)
{
  digitalWrite(rightForward, LOW);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftBackward, HIGH);
  analogWrite(leftPower, power);
  analogWrite(rightPower, power);
}

/*
  halts OSV drivetrain movement
*/

void stopMoving()
{
  analogWrite(leftPower, 0);
  analogWrite(rightPower, 0);
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

void driveToX(float deltaX, float startingX, boolean BOOLforward)
{
  float errX; // variable to hold the error in theta
  int motorRate = 0; //variable to hold motor turn rate
  boolean driveCompleted = false;
  /*
       calculates the error in theta
  */
  while (!driveCompleted)
  {
    checkPosition();
    enes.print("float(xPos): ");
    enes.println(xPos);

    if (!BOOLforward)
    {
      if (startingX - float(xPos) < deltaX)
      {
        moveBackward(minMotorValue + ((startingX - float(xPos)) / deltaX)*p);
        enes.print("Moving backwards with rate ");
        enes.println(minMotorValue + ((startingX - float(xPos)) / deltaX)*p);
        delay(200);
      }
      else
      {
        stopMoving();
        driveCompleted = true;
        enes.println("Moved back boi");
      }
    }
    else
    {
      if((float(xPos)-startingX) <deltaX)
      {
        powerValue = minMotorValue + ((float(xPos)-startingX) / deltaX)*p;
        moveForward(powerValue);
        enes.print("Moving backwards with rate ");
        enes.println(minMotorValue + ((float(xPos)-startingX) / deltaX)*p);
        delay(200);
      }
      else
      {
        stopMoving();
        driveCompleted = true;
        enes.println("Moved forward boi");
      }
    }

  }
}

void driveToY(float deltaY, float startingY)
{
  float errY; // variable to hold the error in theta
  int motorRate = 0; //variable to hold motor turn rate
  boolean driveCompleted = false;
  /*
       calculates the error in theta
  */
  while (!driveCompleted)
  {
    checkPosition();
    enes.print("float(yPos): ");
    enes.println(yPos);

    if ((float(yPos) - startingY < deltaY) && (sin(theta) > 0))
    {
      moveForward(minMotorValue + ((float(yPos) - startingY) / deltaY)*p);
      enes.print("Moving forwards with rate ");
      enes.println(minMotorValue + ((float(yPos) - startingY) / deltaY)*p);
      delay(200);
    }
    else if ((startingY - float(yPos) < deltaY) && (sin(theta) < 0))
    {
      moveForward(minMotorValue + ((startingY - float(yPos)) / deltaY)*p);
      enes.print("Moving forwards with rate ");
      enes.println(minMotorValue + ((startingY - float(yPos)) / deltaY)*p);
      delay(200);
    }
    else
    {
      stopMoving();
      driveCompleted = true;
      enes.println("Moved forward boi");
    }
  }
}

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
    else
    {
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
        //turn left
        digitalWrite(rightForward, HIGH);
        digitalWrite(rightBackward, LOW);

        digitalWrite(leftBackward, HIGH);
        digitalWrite(leftForward, LOW);
      }
      else
      {
        //turn right
        digitalWrite(rightForward, LOW);
        digitalWrite(rightBackward, HIGH);

        digitalWrite(leftBackward, LOW);
        digitalWrite(leftForward, HIGH);
      }
      analogWrite(leftPower, motorRate);
      analogWrite(rightPower, motorRate);
    }
    else
    {
      analogWrite(leftPower, 0);
      analogWrite(rightPower, 0);
      /*
        analogWrite(rightS, 0);
        analogWrite(leftS, 0);
      */
      turnCompleted = true;
    }
    delay(250);
    analogWrite(leftPower, 0);
    analogWrite(rightPower, 0);
    delay(250);

    enes.updateLocation();
    theta = enes.location.theta;// update theta
  }


}
