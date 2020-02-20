#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
//#include "JointMotor.h"
#include "JointMotor2.h"
#include "pins.h"
#include "Gripper.h"
#include "Button.h"
#include "scLookUp.h"
#include "Storage.h"
//#include "TimerOne.h"

// ************* Robot Constant ************* //
const int NUM_MOTORS = 3;
const int NUM_GRIPPERS = 2;

const float m1 = 0.281; // 0.09  CAD value: 0.183
const float m2 = 0.495; // with storing mechanism (with block 0.297 kg) Old: 0.357
const float m3 = 0.207; // mass with new screwing mechanism // Old: 0.201

const float L1 = 0.1633; // Old: 0.1633
const float L2 = 0.1633; // Old: 0.1633
const float L3 = 0.1048; // Old: 0.1048
const float Lblock = 0.145;
const float mblock = 0.365; //0.365

const float LCoM1 = 0.055;
const float LCoM2 = 0.082;
const float LCoM3 = 0.064;

const float g = 9.81;


// ************* Controls Constant ************* //
float gc_complimentary_filter = 1.0;

float k1_a = -0.116; // -130 //-0.089 new
float k2_a = -0.129; //-200 //-0.13 new
float k3_a = -0.15;  //-200 //-0.1 new

float k1_d = -0.089; //a link
float k2_d = -0.1325;
float k3_d = -0.037;

// ************* Communication Constants ************* //
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 4; // control packet example: "0131"
const int len = MOTOR_PKT_LEN * 3 + CONTROL_PKT_LEN;
const int PARSE_PKT_LEN = 4;
char serialBuffer[len];
char temp[PARSE_PKT_LEN];

// ************* Motors Variables ************* //
JointMotor2 jointMotor[NUM_MOTORS];
Gripper gripper[NUM_GRIPPERS];
Storage storage;
// int theta[NUM_MOTORS];

// ************* Robot States Variables ************* //
unsigned long lastPubAng = 0;
int storageMotorState = 1;
int useGravityComp = 1;

bool gripperFinished1 = true;
bool gripperFinished2 = true;
int gripperStatus = 0;
int previousGripperState1 = -1;
int previousGripperState2 = -1;
int gripperStatusSerial1;
int gripperStatusSerial2;

int gripperSelect = 0; //idle (No gripper selected)
int gripperState = 0;  //idle (No gripper action)
int currentEngagedGripper = 0;

// ************* Function Prototypes ************* //
int gravityCompensation(JointMotor2 i, int th[], bool select);
void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper); // to controls grippers with buttons. Remember to set grippers current state.
void updateSpeeds();
void pidTunningHelper(int jointSelect, int potP, int potI, int potD);
void debugPrint(char jName[3], char pName[3], char iName[3], char dName[3], double pInput, double iInput, double dInput)

// ************* Debug Variables ************* //
// Buttons have to be pull up
// Pull up: one terminal on GND and the other
//          attached to the analog pin.

//Setup buttons for PID tunning, button gripper is used sometimes for gripper debug
//          A0-A3 are avaibale analog pins
//Button buttonGrip_1 = Button(A0, PULLUP);
bool buttonState = true;
int jointSelectTune = A0;
int pValue = A1;
int iValue = A2;
int dValue = A3;

void setup()
{
	Serial.begin(9600); //Debug Serial
	Wire.begin();		//begin I2C

	Serial.println("Robot fintializing....");
	temp[int(len / 4)] = '\n'; //you need this
	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 9.2, 0.034, 0.5, 8.5, 0.01, 0.5, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 27, 0.05, 0, 27, 0.05, 0, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 8.85, 0.07, 0.7, 8.85, 0.07, 0, 27.81, false, 2); //works

	jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 40., 0.1, 2.4, 8.4, 0.1, 2.4, 27.81, true, 0);
	jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 8.4, 0.1, 3.2, 8.4, 0.1, 3.2, 124.38, true, 1);
	jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 8, 0.1, 2.6, 8, 0.1, 2.6, 27.81, false, 2); //works

	// 0030.00 0084.00 0067.00 0100
	// 0045.00 0090.00 0045.00 0100

	//D link fixed
	// 0067.00 0084.00 0030.00 0100
	storage = Storage(STORAGE_MOTOR_LC);

	/* DEBUG */
	jointMotor[0].setAngle(27.81);
	jointMotor[1].setAngle(124.38);
	jointMotor[2].setAngle(27.81);

	jointMotor[0].debug = true;
	jointMotor[1].debug = true;
	jointMotor[2].debug = true;

	gripper[0] = Gripper(GRIPPER_MOTOR_1, false, false); //yellow gripper
	gripper[1] = Gripper(GRIPPER_MOTOR_2, true, false);  //red gripper
	Serial.println("Done");

	//Timer1 Interupt
	// Timer1.initialize(500000);
	// Timer1.attachInterrupt(updateSpeeds);
	// interrupts();
}

void loop()
{
	if (Serial.available() > 0)
	{
		Serial.println("Message received");
		Serial.readBytesUntil('\n', serialBuffer, len);
		int tempIndex = 0;
		int jointIndex = 0;
		float tempAngle = 0;
		boolean motorPktCompleted = true;

		if (serialBuffer[0] == '-' || serialBuffer[0] == '0')
		{
			for (int i = 0; i < len; i++)
			{
				temp[tempIndex] = serialBuffer[i];
				if (tempIndex < 3)
				{
					tempIndex++;
				}
				else
				{
					tempIndex = 0;
					if (jointIndex > 2)
					{ //Gripper
						if ((temp[2] - '0' == 1) || (temp[2] - '0' == 2) || (temp[2] - '0' == 3))
						{
							gripperSelect = (temp[2] - '0');
							gripperState = (temp[3] - '0');

							// if (gripperSelect == 1)
							// {
							// 	gripperFinished1 = false;
							// }
							// else if (gripperSelect == 2)
							// {
							// 	gripperFinished2 = false;
							// }
							// else
							// { // both gripper selected
							// 	gripperFinished1 = false;
							// 	gripperFinished2 = false;
							// }
						}
						storageMotorState = temp[1] - '0';
					}
					else
					{ //Joint angles
						if (motorPktCompleted)
						{							   // receiving first half of an angle value
							motorPktCompleted = false; // flip the flag to false
							if (temp[0] == '-')
							{
								temp[0] = '0';
								tempAngle = -1 * atoi(temp);
							}
							else
							{
								tempAngle = atoi(temp);
							}
						}
						else
						{							  // receiving second half of an angle value
							motorPktCompleted = true; // flip the flag to true
							temp[0] = '0';
							temp[PARSE_PKT_LEN - 1] = '0';
							tempAngle += (atof(temp) / 1000.0); // divide by 1000 to compensate for the extra 0
							useGravityComp = 1;
							jointMotor[jointIndex].setAngle(tempAngle);
							// jointMotor[jointIndex].sumError = 0.0;
							Serial.print("Setting angle[");
							Serial.print(jointIndex + 1);
							Serial.print("]: ");
							Serial.println(tempAngle);
							jointIndex++;
						}
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < len; i++)
			{
				Serial.read();
			}
		}
	}

	//Block storage control
	if (storageMotorState == 1)
	{
		storage.restPosition();
	}
	else if (storageMotorState == 2)
	{
		storage.loadPosition();
	}

	//Gripper Actuation
	// if (!gripperFinished1 && gripperSelect == 1)
	// {
	// 	gripperFinished1 = gripper[gripperSelect - 1].setGripper(gripperState);
	// }

	// if (!gripperFinished2 && gripperSelect == 2)
	// {
	// 	gripperFinished2 = gripper[gripperSelect - 1].setGripper(gripperState);
	// }

	// if (gripperFinished1 && gripperFinished2)
	// {
	// 	useGravityComp = 1;
	// }
	
	// Grippers Control
	if (gripperSelect == 1)
	{
		gripper[0].setGripper(gripperState);
	}
	else if (gripperSelect == 2)
	{
		gripper[1].setGripper(gripperState);
	}
	else if (gripperSelect == 3) 	// both motors selected
	{
		gripper[0].setGripper(gripperState);
		gripper[1].setGripper(gripperState);
	}
	gripperSelect = 0;	// Resets the gripper selection

	// Switching PID values for joint motors
	if (gripper[0].isEngaged  && currentEngagedGripper != 1) // Gripper 1 just engaged
	{
		currentEngagedGripper = 1;
		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
		{
			jointMotor[i].switchPID(currentEngagedGripper);
			useGravityComp = 0;
		}
	}
	else if (gripper[1].isEngaged && currentEngagedGripper != 2) // Gripper 2 just engaged
	{
		currentEngagedGripper = 2;
		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
		{
			jointMotor[i].switchPID(currentEngagedGripper);
			useGravityComp = 0;
		}
	}
	else
	{
		currentEngagedGripper = 0;
		useGravityComp = 1;
	}
	updateSpeeds();
}

/*
* Prints variable every 2000 millis
*/
void debugPrint(char jName[3], char pName[3], char iName[3], char dName[3], double pInput, double iInput, double dInput)
{
	if (millis() - lastPubAng > 2000)
	{
		Serial.print("--------");
		Serial.println(jName);
		Serial.print(pName);
		Serial.print(": ");
		Serial.println(pInput);
		Serial.print(iName);
		Serial.print(": ");
		Serial.println(iInput);
		Serial.print(dName);
		Serial.print(": ");
		Serial.println(dInput);
		lastPubAng = millis();
	}
}

/*
*	Calculates Gravity Compensation
*/
int gravityCompensation(JointMotor2 i, int th[], bool select)
{
	// int theta0 = th[0];
	// int theta1 = th[1];
	// int theta2 = th[2];

	//Wrap around
	// if (theta0 >= 360)
	// {
	// 	theta0 %= 360;
	// }
	// if (theta1 >= 360)
	// {
	// 	theta1 %= 360;
	// }
	// if (theta2 >= 360)
	// {
	// 	theta2 %= 360;
	// }

	for (int i = 0; i < NUM_MOTORS; i++){
		th[i] %= th[i];
	}

	int theta_0 = th[0];
	int theta_0_1 = th[0] + th[1];
	int theta_0_1_2 = th[0] + th[1] + th[2];

	if (currentEngagedGripper == 2) // TODO change back to 2
	{							   // D link gripper engaged (current link has block)
		if (i.id == 0)
		{
			return k1_d * (g * m3 * (L3 - LCoM3) * sinLut[theta_0_1_2]);
		}
		else if (i.id == 1)
		{
			return k2_d * (g * m3 * (L2 * sinLut[theta_0_1] + (L3 - LCoM3) * sinLut[theta_0_1_2]) + g * (L2 - LCoM2) * m2 * sinLut[theta_0_1] + g * mblock * Lblock * sinLut[theta_0_1]);
		}
		else if (i.id == 2)
		{
			return k3_d * (g * m3 * (L1 * sinLut[theta_0] + L2 * sinLut[theta_0_1] + (L3 - LCoM3) * sinLut[theta_0_1_2]) + g * m2 * (L1 * sinLut[theta_0] + (L2 - (L2 - LCoM2)) * sinLut[theta_0_1]) + g * (L1 - LCoM1) * m1 * sinLut[theta_0] + g * mblock * (L1 * sinLut[theta_0] + Lblock * sinLut[theta_0_1]));
		}
		else
		{
			Serial.print("NO JOINT ID AVAILABLE FOR GRAVITY COMPENSATION");
			return 0;
		}
	}
	else
	{ 								// A link gripper engaged (opposite link has block)
		if (i.id == 0)
		{
			return k1_a * (g * m3 * (L1 * sinLut[theta_0] + L2 * sinLut[theta_0_1] + LCoM3 * sinLut[theta_0_1_2]) + g * m2 * (L1 * sinLut[theta_0] + LCoM2 * sinLut[theta_0_1]) + g * LCoM1 * m1 * sinLut[theta_0] + g * mblock * (L1 * sinLut[theta_0] + Lblock * sinLut[theta_0_1]));
		}
		else if (i.id == 1)
		{
			return k2_a * (g * m3 * (L2 * sinLut[theta_0_1] + LCoM3 * sinLut[theta_0_1_2]) + g * LCoM2 * m2 * sinLut[theta_0_1] + g * mblock * Lblock * sinLut[theta_0_1]);
		}
		else if (i.id == 2)
		{
			return k3_a * (g * m3 * LCoM3 * sinLut[theta_0_1_2]);
		}
		else
		{
			Serial.print("NO JOINT ID AVAILABLE FOR GRAVITY COMPENSATION");
			return 0;
		}
	}
}

/*
* Updates speed of all joint motors for PWM
*/
void updateSpeeds()
{
	int gc, jointAngle;
	double speeds[NUM_MOTORS] = {0, 0, 0};
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		jointAngle = jointMotor[i].getAngleDegrees();
		gc = gravityCompensation(jointMotor[i], jointAngle, true) * gc_complimentary_filter;
		speeds[i] = jointMotor[i].calcSpeed(gc, useGravityComp);
		// Serial.print("\nGravity Comp:");
		// Serial.print(gc);
		// Serial.print("\nPID: ");
		// Serial.println(speeds[i] - gc);
	}
	// set joint motor speeds after all gcs are calculated to
	// minimize delay between joint movements
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		jointMotor[i].setSpeed(speeds[i]);
	}
}

/*
* Enables to interface (engage and disengage) the grippers using buttons.
* Buttons should be plugged into the Analog pins.
*/
// void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper){

// 		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && buttonState){
// 			buttonState = false;
// 			if(currentState == 1){
// 				Serial.println("disengage");
// 				grip.setGripper(2);
// 			}else{
// 				Serial.println("engage");
// 				grip.setGripper(1);
// 			}
// 		}
// 		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && !buttonState){
// 			buttonState = true;
// 			if(currentState == 1){
// 				Serial.println("engage");
// 				grip.setGripper(1);
// 			}else{
// 				Serial.println("disengage");
// 				grip.setGripper(2);
// 			}
// 		}
// }

/*
* Helper function for tunnning of PID values on the three diffrent joints using one button and 3 potentiometers
*/
void pidTunningHelper(int jointSelect, int potP, int potI, int potD)
{
	int js = analogRead(jointSelect);
	int offset = 70;
	double ratio = 0.01056;

	if (js < 341)
	{
		if (js > 170)
		{
			jointMotor[0].kP = (analogRead(potP) - offset) * ratio;
			jointMotor[0].kI = (analogRead(potI) - offset) * ratio;
			jointMotor[0].kD = (analogRead(potD) - offset) * ratio;
		}
		debugPrint("J0", "KP", "KI", "KD", jointMotor[0].kP, jointMotor[0].kI, jointMotor[0].kD);
	}
	else if (js > 341 && js < 682)
	{
		if (js > 511)
		{
			jointMotor[1].kP = (analogRead(potP) - offset) * ratio;
			jointMotor[1].kI = (analogRead(potI) - offset) * ratio;
			jointMotor[1].kD = (analogRead(potD) - offset) * ratio;
		}
		debugPrint("J1", "KP", "KI", "KD", jointMotor[1].kP, jointMotor[1].kI, jointMotor[1].kD);
	}
	else if (js > 682)
	{
		if (js > 852)
		{
			jointMotor[2].kP = (analogRead(potP) - offset) * ratio;
			jointMotor[2].kI = (analogRead(potI) - offset) * ratio;
			jointMotor[2].kD = (analogRead(potD) - offset) * ratio;
		}
		debugPrint("J2", "KP", "KI", "KD", jointMotor[2].kP, jointMotor[2].kI, jointMotor[2].kD);
	}
	// Serial.print("Kp: ");
	// Serial.println(jointMotor[0].kP);
}
