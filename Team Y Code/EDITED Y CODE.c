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

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int claw;
int arms;
float target;
float error;
float derivative;
float lastError;
float Kp = 0.0001;
float Ki = 0.002;
float Kd = 0.002;

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
/*
task battery()
{
	bLCDBacklight = true;									// Turn on LCD Backlight
	string mainBattery, backupBattery;

	while(true)														// An infinite loop to keep the program running until you terminate it
	{
		clearLCDLine(0);											// Clear line 1 (0) of the LCD
		clearLCDLine(1);											// Clear line 2 (1) of the LCD

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Backup: ");
		sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');	//Build the value to be displayed
		displayNextLCDString(backupBattery);

		//Short delay for the LCD refresh rate
		wait1Msec(100);
	}
}

void autoDriveTicksShaft(int masterPower, int left_ticks) { //Drive forward based on sensor values.
	int totalTicks = 0;
	int Autoerror = 0;
	int slavePower = masterPower - 5;

	SensorValue[right] = 0;
	SensorValue[left] = 0;

	while(abs(totalTicks) < left_ticks) {
		motor[DriveL] = masterPower;
		motor[DriveR] = slavePower;

		Autoerror = (abs(SensorValue[left]) - SensorValue[right]);
		slavePower += Autoerror * 0.5;
		totalTicks += abs(SensorValue[left]);
		SensorValue[right] = 0;
		SensorValue[left] = 0;
		wait1Msec(100);
	}
	motor[DriveL] = 0;
	motor[DriveR] = 0;
}

void gyroTurnLeft (int degrees, int speedLeft, int speedRight) {
	SensorType[gyro] = sensorNone;
	int acceptableError = 5;
	wait1Msec(1000);
	while(abs(SensorValue[gyro]) < degrees - 100)  {
		//		motor[ClawA] = 70;
		//		motor[ClawB] = 70;
		motor[left] = speedLeft;
		motor[right] = speedRight;
	}
	motor[left] = -5;
	motor[right] = 5;
	wait1Msec(100);
	while(abs(SensorValue[gyro]) > degrees + acceptableError || abs(SensorValue[gyro]) < degrees - acceptableError)
	{
		if(abs(SensorValue[gyro]) > degrees)
		{
			motor[right] = -30;
			motor[left] = 30;
			//		motor[ClawA] = 70;
			//		motor[ClawB] = 70;
		}
		else
		{
			//		motor[ClawA] = 70;
			//		motor[ClawB] = 70;
			motor[right] = 30;
			motor[left] = -30;
		}
		motor[right] = 0;
		motor[left] = 0;
		wait1Msec(250);
	}
}

void clawOpen () {
int claw_;
int target;

}*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
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
	//autoDriveTicksShaft(100,100);
//	gyroTurnLeft(900,-50,50);

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

task usercontrol()
{
	startTask(drive);
//	startTask(battery);
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
