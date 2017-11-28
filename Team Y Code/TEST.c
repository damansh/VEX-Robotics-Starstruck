#pragma config(Sensor, in1,    claw_s,         sensorPotentiometer)
#pragma config(Sensor, in3,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  arms_s,         sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  right,          sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  left,           sensorQuadEncoder)
#pragma config(Motor,  port1,           ArmR1,         tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           ArmR2,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           ArmR3,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           ClawA,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           ClawB,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           DriveL,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           DriveR,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           ArmL1,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           ArmL2,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          ArmL3,         tmotorVex393HighSpeed_HBridge, openLoop, reversed)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/


// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

int claw;
int arms;
float target;
float error;
float derivative;
float lastError;
float Kp = 0.0001;
float Ki = 0.002;
float Kd = 0.002;
const short leftButton = 1;
const short centerButton = 2;
int count = 0;
const short rightButton = 4;

task clawControl() {
	while(true) {
		while (vexRT[Btn5U] == 1) {
			claw = 70;
		}
		while (vexRT[Btn5D] == 1) {
			claw = -70;
		}
		claw = 0;
	}
}

task arm() {
	while(true) {
		if (vexRT[Btn7R] == 1) {
			target = 300;
			float integral = 0;
			SensorValue[arms_s] = 0;
			while(vexRT[Btn6D] == 0) {
				motor[ClawA] = 50;
				motor[ClawB] = 50;
				error = target - SensorValue[arms_s];
				integral = integral + error;

				if (error == 0) {
					integral = 0;
				}

				 if (abs(error) > 65) {
				integral = 0;
				}

				derivative = error - lastError;
				lastError = error;
				arms = (Kp*error) + (Ki*integral) + (Kd*derivative);
			}
		}

		while (vexRT[Btn6U] == 1) {
			arms = 127;
		}
		while(vexRT[Btn6D] == 1) {
			arms = -127;
		}
		arms = 0;
	}
}

task drive() {
	while(true) {
		motor[DriveL] = vexRT[Ch2];
		motor[DriveR] = vexRT[Ch3];
	}
}

//Wait for Press--------------------------------------------------
void waitForPress()
{
	while(nLCDButtons == 0){}
	wait1Msec(5);
}
//----------------------------------------------------------------

//Wait for Release------------------------------------------------
void waitForRelease()
{
	while(nLCDButtons != 0){}
	wait1Msec(5);
}
//----------------------------------------------------------------
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

task codeChooser () {
//Declare count variable to keep track of our choice
//Clear LCD

clearLCDLine(0);
clearLCDLine(1);
//Loop while center button is not pressed
while(nLCDButtons != centerButton)
{
	//Switch case that allows the user to choose from 4 different options
	switch(count){
	case 0:
		//Display first choice
		displayLCDCenteredString(0, "Autonomous 1");
		displayLCDCenteredString(1, "<		 Enter		>");
		waitForPress();
		//Increment or decrement "count" based on button press
		if(nLCDButtons == leftButton)
		{
			waitForRelease();
			count = 3;
		}
		else if(nLCDButtons == rightButton)
		{
			waitForRelease();
			count++;
		}
		break;
	case 1:
		//Display second choice
		displayLCDCenteredString(0, "Autonomous 2");
		displayLCDCenteredString(1, "<		 Enter		>");
		waitForPress();
		//Increment or decrement "count" based on button press
		if(nLCDButtons == leftButton)
		{
			waitForRelease();
			count--;
		}
		else if(nLCDButtons == rightButton)
		{
			waitForRelease();
			count++;
		}
		break;
	case 2:
		//Display third choice
		displayLCDCenteredString(0, "Autonomous 3");
		displayLCDCenteredString(1, "<		 Enter		>");
		waitForPress();
		//Increment or decrement "count" based on button press
		if(nLCDButtons == leftButton)
		{
			waitForRelease();
			count--;
		}
		else if(nLCDButtons == rightButton)
		{
			waitForRelease();
			count++;
		}
		break;
	case 3:
		//Display fourth choice
		displayLCDCenteredString(0, "Autonomous 4");
		displayLCDCenteredString(1, "<		 Enter		>");
		waitForPress();
		//Increment or decrement "count" based on button press
		if(nLCDButtons == leftButton)
		{
			waitForRelease();
			count--;
		}
		else if(nLCDButtons == rightButton)
		{
			waitForRelease();
			count = 0;
		}
		break;
	default:
		count = 0;
		break;
	}
}
}

void pre_auton()
{

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
clearLCDLine(0);
clearLCDLine(1);
//Switch Case that actually runs the user choice
switch(count){
case 0:
	//If count = 0, run the code correspoinding with choice 1
	displayLCDCenteredString(0, "Autonomous 1");
	displayLCDCenteredString(1, "is running!");
	wait1Msec(1000);						// Robot waits for 2000 milliseconds

	// INSERT AUTONOMOUS 1 CODE HERE BEFORE BREAK
	// MANITHA AUTONOMOUS
/*	int kill = false;
	int auton_s = 2;
	while(auton_s == 1 && kill == false)
	{
		startTask(clawManitha);
		shootManitha(-100,10,true);
		drive_tManitha(60,127);
		gyro_turnManitha(-750);
		close_claw = false;
		open_claw = true;
		drive_tManitha(100,127);
		close_claw = true;
		open_claw = false;
		gyro_turnManitha(-800);
		drive_tManitha(70,-127);
		arms_return = false;
		shootManitha(-1000,10,false);
		kill = true;
	}
	while(auton_s == 2 && kill == false)
	{
		startTask(clawManitha);
		drive_tManitha(60,127);
		gyro_turnManitha(-750);
		close_claw = false;
		open_claw = true;
		drive_tManitha(100,127);
		close_claw = true;
		open_claw = false;
		gyro_turnManitha(-800);
		drive_tManitha(70,-127);
		arms_return = false;
		shootManitha(-1000,10,true);
		close_claw = false;
		open_claw = true;
		drive_tManitha(150,127);
		close_claw = true;
		open_claw = false;
		drive_tManitha(150,-127);
		shootManitha(-1000,10,true);
		kill = true;
	} */
	//END OF FIRST AUTONOMOUS
	break;
case 1:
	//If count = 1, run the code correspoinding with choice 2
	displayLCDCenteredString(0, "Autonomous 2");
	displayLCDCenteredString(1, "is running!");
	wait1Msec(1000);						// Robot waits for 2000 milliseconds

	// INSERT AUTONOMOUS 2 CODE HERE BEFORE BREAK
	// Just the middle cube

	break;
case 2:
	//If count = 2, run the code correspoinding with choice 3
	displayLCDCenteredString(0, "Autonomous 3");
	displayLCDCenteredString(1, "is running!");
	wait1Msec(1000);						// Robot waits for 2000 milliseconds

	//INSERT AUTONOMOUS 3 CODE HERE BEFORE BREAK
	// Three stars off of the fence

	break;
case 3:
	//If count = 3, run the code correspoinding with choice 4
	displayLCDCenteredString(0, "Autonomous 4");
	displayLCDCenteredString(1, "is running!");
	wait1Msec(1000);						// Robot waits for 2000 milliseconds

	//INSERT AUTONOMOUS 4 CODE HERE BEFORE BREAK
	//Three stars off of the fence and the middle cube

	break;
default:
	displayLCDCenteredString(0, "No valid choice");
	displayLCDCenteredString(1, "was made!");
	break;
}
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	startTask(drive);
	startTask(codeChooser);
	startTask(arm);
	startTask(clawControl);
	while (true)
	{
		motor[ClawA] = claw;
		motor[ClawB] = claw;
		motor[ArmR1] = arms;
		motor[ArmR2] = arms;
		motor[ArmR3] = arms;
		motor[ArmL1] = arms;
		motor[ArmL2] = arms;
		motor[ArmL3] = arms;
	}
}
