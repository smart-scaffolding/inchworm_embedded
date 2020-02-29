// JointMotor2.h

#ifndef _JOINTMOTOR2_h
#define _JOINTMOTOR2_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "ams_as5048b.h"
#include "RunningAverage.h"
//#include <Servor.h>

class JointMotor2
{
private:
	int pinDirectionA, pinDirectionB, pinPWM;
	AMS_AS5048B encoder;

	//PID
	double kP2, kI2, kD2;
	double kP1, kI1, kD1;

	double last_calibrated_angle; //angle of joint
	double angle_offset;		  // offset of angle in calibration position
	bool enc_clockwise;			  //1 if switch direction

	double lastPubAng;
	double lastPubAng2;
	double lastPubAng3;

	// RunningAverage myRA = RunningAverage(30);
	// RunningAverage myRA(10);

	// const static int num_last_errors = 5;
	// double last_errors[num_last_errors];
	// int error_idx;

	double velocity_term = 0;

	// Joint Velocity Calculation
	unsigned long last_angle_update_time = 0;
	double current_velocity = 0;
	int desired_velocity = 0;
public:
	double sumError, lastError;
	double desiredAngle;

	double kP, kI, kD; //TODO: whn values tunned, put this back into private variables
	bool debug;
	int id;

	JointMotor2();
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset, bool encoder_clockwise, int id_input, double velocity_term_init);
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset, bool encoder_clockwise, int id_input, double velocity_term_init);

	void setSpeed(double speed);
	void changeDirection(double speed);
	void setAngle(double angle);
	void setVelocity(int velocity);
	bool switchPID(int gripperEngagedSelect);
	double calcSpeed(double currentAngle, int gc, int useGravityComp, int velocity_term_scale);
	double getAngleDegrees();
	double getVelocity();
	int getDesiredVelocity();
	void debugPrint(char vName[3], double vInput);
	void debugPrint2(char vName[3], double vInput);
	// float gravityCompensation(int th);
	// double getKP();
	// void setKP(double kpValue)
};

#endif
