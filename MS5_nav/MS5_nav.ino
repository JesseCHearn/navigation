#include "Enes100.h"


// Global vars
float theta; // tank angle in rad
float xPos; //tank x coord
float yPos; //tank y coord
//float theta_desired = PI / 12; // Enter the target angle in the space here
int minMotorValue = 140; // minimum input value which will produce a turning movement in the tank

int leftD = 4;
int leftS = 5;
int rightS = 6;
int rightD = 7;

int dist; // read distance to obstacle


/* Create a new Enes100 object
   Parameters:
    string teamName
    int teamType
    int markerId
    int rxPin
    int txPin
*/
#include <dfr_tank.h>
Enes100 enes("JEDIARMS", WATER, 5, 8, 9);
DFRTank tank;
void setup()
{
  tank.init();
  // Retrieve the destination
  while (!enes.retrieveDestination()) {
    enes.println("Unable to retrieve location");
  }

  enes.print("My destination is at ");
  enes.print(enes.destination.x);
  enes.print(",");
  enes.println(enes.destination.y);
}

void loop()
{
  // Update the OSV's current location
  if (enes.updateLocation()) {
    enes.println("Huzzah! Location updated!");
    enes.print("My x coordinate is ");
    enes.println(enes.location.x);
    enes.print("My y coordinate is ");
    enes.println(enes.location.y);
    enes.print("My theta is ");
    enes.println(enes.location.theta);
  } else {
    enes.println("Sad trombone... I couldn't update my location");
  }

//turn towards y direction of destination


if








yCoord();
xCoord();
obstacle();
  


}




void driveUntil






  void yCoord() {
    //Get to correct y coordinate
    if (enes.location.y > enes.destination.y) {
      turnTo(3 * PI / 2); //turn to negative y axis
      while (enes.location.y > enes.destination.y) {
        enes.updateLocation();
        tank.setLeftMotorPWM(255);
        tank.setRightMotorPWM(255);
        delay(500);
        if (enes.location.y <= enes.destination.y) {
          tank.setLeftMotorPWM(0);
          tank.setRightMotorPWM(0);
          turnTo(0);//turn 90 degrees to be parallel to x axis
        }
      }
    }
    else if (enes.location.y < enes.destination.y) {
      turnTo(PI / 2); //turn to positive y axis
      while (enes.location.y < enes.destination.y) {
        enes.updateLocation();
        tank.setLeftMotorPWM(255);
        tank.setRightMotorPWM(255);
        delay(500);
        if (enes.location.y >= enes.destination.y) {
          tank.setLeftMotorPWM(0);
          tank.setRightMotorPWM(0);
          turnTo(0);//turn 90 degrees to be parallel to x axis
        }
      }
    }
    else {
      turnTo(0);//turn 90 degrees to be parallel to x axis
    }
  }

  void xCoord() {
    //Navigate to x coordinate
    
    while (dist < 400) {
      dist = analogRead(6);
      enes.updateLocation();
      while (enes.location.x < enes.destination.x) {
        enes.updateLocation();
        tank.setLeftMotorPWM(255);
        tank.setRightMotorPWM(255);
        delay(500);
        if (enes.location.x >= enes.destination.x) {
          tank.setLeftMotorPWM(0);
          tank.setRightMotorPWM(0);
        }
      }
    }
  }


  //Detect obstacle, stop, and turn upwards
  void obstacle() {
      dist = analogRead(6);
     if (dist >= 400 && enes.location.y < 3) {
      tank.setLeftMotorPWM(0);
      tank.setRightMotorPWM(0);
      turnTo(PI / 2); //turn 90 degrees towards positive y axis
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(255);
      delay(500);
      tank.setLeftMotorPWM(0);
      tank.setRightMotorPWM(0);
      turnTo(0);//rotate back towards positive x axis
      obstacle();
    }
    else if (dist >= 400 && enes.location.y > 3) {
      tank.setLeftMotorPWM(0);
      tank.setRightMotorPWM(0);
      turnTo(3 * PI / 2); //turn 90 degrees towards negative y axis
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(255);
      delay(500);
      tank.setLeftMotorPWM(0);
      tank.setRightMotorPWM(0);
      turnTo(0);//rotate back towards positive x axis
      obstacle();
    }
    else {
      xCoord();
      yCoord();
    }
  }
  /*if (enes.location.x == enes.destination.x && enes.location.y == enes.location.y) {
    enes.navigated();
  }
*/


  /*
     turns tank to desired angle theta
  */

void turnTo(float theta_desired)
{
  float k = (255 - minMotorValue) / PI; // proportional controller for turning speed (“gain”)
  float errTheta; // variable to hold the error
  int motorRate = 0; //variable for turning rate of motor

  if ((theta - theta_desired <= PI) && (theta - theta_desired >= 0))
  {
    errTheta = (theta - theta_desired); // compute error TURN RIGHT (POSITIVE)
  }
  else if (theta - theta_desired > PI)
  {
    errTheta = -((2 * PI - theta) + theta_desired); // compute error TURN LEFT (NEGATIVE)
  }
  else if ((theta - theta_desired >= -PI) && (theta - theta_desired < 0))
  {
    errTheta = (theta - theta_desired); // compute error TURN LEFT (NEGATIVE)
  }
  else if (theta - theta_desired < -PI)
  {
    errTheta = ((2 * PI - theta_desired) + theta); // compute error TURN RIGHT (POSITIVE)
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
  }
  else
  {
    analogWrite(rightS, 0);
    analogWrite(leftS, 0);
  }

  delay(250);
  analogWrite(rightS, 0);
  analogWrite(leftS, 0);
  delay(250);

  enes.updateLocation();
  theta = enes.location.theta;// update theta
}
