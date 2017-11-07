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

  missionState = 0;

  enes.println("SETUP INITIALIZED");
}
void loop()
{
  enes.updateLocation();
  theta = enes.location.theta;
  xPos = enes.location.x;
  yPos = enes.location.y;

  


  if (missionState == 0)
  {
    enes.println("missionState=0");
    if (yPos < yMission) // tank is below destination
    {
      targetTheta = PI / 2;
      missionState = 1;
    }
    else if (yPos > yMission) // tank is above destination
    {
      targetTheta = 3 * PI / 2;
      missionState = 1;
    }
    else // tank is at same y coord as destination
    {
      targetTheta = 0;
      missionState = 2;
    }
    turnTo(targetTheta);
  }
  else if (missionState == 1)
  {
    
    enes.println("missionState=1");
    enes.updateLocation();
    yPos = enes.location.y;
    errY = yPos - yMission;
    enes.print("Error in y: ");
    enes.println(errY);
    

    while (errY > .05 || errY < -.05)
    {
      digitalWrite(leftD, HIGH);
      digitalWrite(rightD, HIGH);
      analogWrite(leftS, 255);
      analogWrite(rightS, 255);
      delay (250);
      analogWrite(leftS, 0);
      analogWrite(rightS, 0);
      delay(250);
      enes.updateLocation();
      yPos = enes.location.y;
      errY = yPos - yMission;
      turnTo(targetTheta);
    }
    enes.print("Reached destination yPos of ");
    enes.println(yPos);
    if(early)
    {
      missionState = 2;
    }
    else
    {
      missionState = 3;
    }
    
  }



  else if (missionState == 2)
  {

    early = false;
    turnTo(0);

    enes.println("missionState=2");
    enes.updateLocation();
    xPos = enes.location.x;
    errX = xMission - xPos;

    //boolean hitObstacle = false;

    while (errX > .05 || errX < -.05)
    {
      dist = analogRead(6);

      if (dist > 200)
      {
        turnTo(3 * PI / 2);
        digitalWrite(leftD, HIGH);
        digitalWrite(rightD, HIGH);
        analogWrite(leftS, 255);
        analogWrite(rightS, 255);
        delay(1000);
        turnTo(0);
        enes.println("Hit obstacle");
      }
      else
      {
        turnTo(calcHeading());
        digitalWrite(leftD, HIGH);
        digitalWrite(rightD, HIGH);
        analogWrite(leftS, 255);
        analogWrite(rightS, 255);
        delay(300);
        analogWrite(leftS, 0);
        analogWrite(rightS, 0);
        delay(100);
        
        /*digitalWrite(leftD, HIGH);
        digitalWrite(rightD, HIGH);
        analogWrite(leftS, 255);
        analogWrite(rightS, 255);
        delay(250);
        analogWrite(leftS, 0);
        analogWrite(rightS, 0);
        delay(250);
        */
      }
      enes.updateLocation();
      xPos = enes.location.x;
      enes.print("XPOS = ");
      enes.println(xPos);
      errX = xPos - xMission;
      delay(100);
      enes.print("Error in x: ");
      enes.println(errX);

      /*if(errX <.05)
      {
         enes.println("Reached x coord of destination");
         missionState = 3;
      }
      */
     
    }
       
    analogWrite(leftS, 0);
    analogWrite(rightS, 0);
    //missionState = 5;
  }

  /*else if(missionState==3)
  {
    enes.navigated();
    missionState= -1;
  }
*/

  else if(missionState==5)
  {
    while(calcDistance()>.05)
    {
      turnTo(calcHeading());
      digitalWrite(leftD, HIGH);
      digitalWrite(rightD, HIGH);
      analogWrite(leftS, 255);
      analogWrite(rightS, 255);
      delay(100);
      
    }
      missionState = 3;
      analogWrite(leftS, 0);
      analogWrite(rightS, 0);
  }




}








float calcHeading()
{
  float desiredHeading;
  enes.updateLocation();
  xPos = enes.location.x;
  yPos = enes.location.y;
  desiredHeading = atan((yMission-yPos)/(xMission-xPos)); 
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
  distance = sqrt(((yMission-yPos)*(yMission-yPos))+((xMission-xPos)*(xMission-xPos)));
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

