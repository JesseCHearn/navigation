#define PI 3.1415926535 // pi
#include <dfr_tank.h>
#include <SoftwareSerial.h>
#include "Enes100.h"

// Global variables


//USER INPUTS
int markerID = 4;         //INPUT MARKER ID
int minMotorValue = 220;  //INPUT MINIMUM MOTOR VALUE FOR TURNING
                          //minimum input value which will produce a turning movement in the OSV


Enes100 enes("JediArms", WATER, markerID, 8, 9);

float k = (255 - minMotorValue) / PI; // proportional controller for turning speed (“gain”)

int dist; // resistance value provided by left distance sensor
int dist1;// resistance value provided by right distance sensor

float xMission; // variable to hold x location of mission site
float yMission; // variable to hold y location of mission site

//OSV position variables
float xPos; // OSV x coordinate in m
float yPos; // OSV y coordinate in m
float theta; // OSV angle in rad


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


/*
  enes.println("HELLO WORLD");

  delay(500);


  if(enes.updateLocation())
  {
    enes.println("LOCATION FOUND");
    checkPosition();
    enes.println("ATTEMPTING TO TURN");
    turnTo(0);
    enes.println("TURN SUCCESSFUL");
  }
  else
  {
    enes.println("ERROR: COULD NOT FIND LOCATION");
  }
*/



  /*
    digitalWrite(rightForward, HIGH);
    digitalWrite(leftForward, LOW);
    digitalWrite(rightBackward, LOW);
    digitalWrite(leftBackward, HIGH);
    analogWrite(leftPower, 200);
    analogWrite(rightPower, 200);
  */


    checkPosition();
    if (calcDistance() > .15)
    {
    dist = analogRead(0);
    dist1 = analogRead(1);
    if ((dist > 200 || dist1 > 200)&&(xPos>1.2))//%%%%%%%%Need to go forward over rocky terrain. No stopping. (Add and to if statement)
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
      if(xPos>1.2||xPos<.34)
      {
      turnTo(calcHeading());
      }
      moveForward(255);
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
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftBackward, LOW);
  analogWrite(leftPower, power);
  analogWrite(rightPower, power);
}



/*
   takes power input
   where minMotorValue<=power<=255
   moves OSV backward with power
*/

void moveBack(int power)
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
      enes.println("errTheta is positive");
    }
    else if (theta - thetaDesired > PI)
    {
      errTheta = -((2 * PI - theta) + thetaDesired); // compute error TURN LEFT (NEGATIVE)
      enes.println("errTheta is negative");
    }
    else if ((theta - thetaDesired >= -PI) && (theta - thetaDesired < 0))
    {
      errTheta = (theta - thetaDesired); // compute error TURN LEFT (NEGATIVE)
      enes.println("errTheta is negative");
    }
    else if (theta - thetaDesired < -PI)
    {
      errTheta = ((2 * PI - thetaDesired) + theta); // compute error TURN RIGHT (POSITIVE)
      enes.println("errTheta is positive");
    }
    else
    {
      enes.println("ERROR: no conds satisfied");
    }
    /*
       turns tank to desired angle theta
    */
    if (errTheta > 0.05 || errTheta < -0.05) // only compute if not converged
    {
      if (abs(errTheta) < .1)
      {
        motorRate = minMotorValue;
        enes.println("setting motorRate to minMotorValue");
      }
      else
      {
        motorRate = minMotorValue + abs(errTheta) * k;
        enes.println("setting motorRate to proportional value");

      }

      if (errTheta < 0)
      {
        //turn left
        digitalWrite(rightForward, HIGH);
        digitalWrite(rightBackward, LOW);

        digitalWrite(leftBackward, HIGH);
        digitalWrite(leftForward, LOW);
        enes.println("turning left");
      }
      else
      {
        //turn right
        digitalWrite(rightForward, LOW);
        digitalWrite(rightBackward, HIGH);

        digitalWrite(leftBackward, LOW);
        digitalWrite(leftForward, HIGH);
        enes.println("turning right");
      }
      analogWrite(leftPower, motorRate);
      analogWrite(rightPower, motorRate);
      enes.println(errTheta);
    }
    else
    {
      analogWrite(leftPower, 0);
      analogWrite(rightPower, 0);
      /*
        analogWrite(rightS, 0);
        analogWrite(leftS, 0);
      */
      enes.println("turn completed");
      turnCompleted = true;
    }
    delay(250);
    analogWrite(leftPower, 0);
    analogWrite(rightPower, 0);
    delay(250);
    enes.println("reached end of if else chain, starting again");

    enes.updateLocation();
    theta = enes.location.theta;// update theta
  }

  enes.print("Reached target angle of ");
  enes.println(thetaDesired);

}
