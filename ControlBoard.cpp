#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2014.h"
#include <math.h>

ControlBoard::ControlBoard(RobotModel* myRobot){
	robot = myRobot;

	intakeDesiredForward = false;
	intakeDesiredBackward = false;
	intakingDesired = false; 
	holdBallDesired = false;
	intakeStartDesired = false; 
	intakeShootPositionDesired = false;
	popBallPistonInDesired = false;
	popBallPistonOutDesired = false;
	
	shooterDesired = false;
	armingDesired = false;

	lowGearDesired = false;
	highGearDesired = false;
	
	reverseDriveDesired = false;
	
	sharpTurnDesired = false;
	
	autoGearShiftDesired = false;
	manualGearShiftDesired = false;
	
	cameraImageRequested = false;

	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);

	//*****OPERATOR JOY BUTTONS******
	shooterButton = new ButtonReader(operatorJoy, OPERATOR_JOY_SHOOTER_PORT);
	armingButton = new ButtonReader(operatorJoy, OPERATOR_JOY_ARMING_PORT);
	intakeForwardButton = new ButtonReader(operatorJoy, OPERATOR_JOY_INTAKE_FORWARD_PORT);
	intakeBackwardButton = new ButtonReader(operatorJoy, OPERATOR_JOY_INTAKE_BACKWARD_PORT);
	intakingButton = new ButtonReader(operatorJoy, OPERATOR_JOY_INTAKING_PORT);
	holdBallButton = new ButtonReader(operatorJoy, OPERATOR_JOY_HOLD_BALL_PORT);
	intakeStartButton = new ButtonReader(operatorJoy, OPERATOR_JOY_INTAKE_START_PORT);
	intakeShootPositionButton = new ButtonReader(operatorJoy, OPERATOR_JOY_INTAKE_SHOOT_PORT);
	popBallButton = new ButtonReader(operatorJoy, OPERATOR_JOY_POP_BALL_PORT);
	
	reverseDriveButton = new ButtonReader(operatorJoy, OPERATOR_JOY_REVERSE_DRIVE_PORT);

	//*****RIGHT JOY BUTTONS******
	highlowGearButton = new ButtonReader(rightJoy, RIGHT_JOY_HIGH_LOW_GEAR_PORT);
	autoManualGearButton = new ButtonReader(leftJoy, LEFT_JOY_AUTO_MAN_GEAR_PORT);
	quickTurnButton = new ButtonReader(rightJoy, RIGHT_JOY_QUICK_TURN_PORT);

}


void ControlBoard::ReadControls() {
	//read button values here
	if (intakeForwardButton->IsDown()){
		intakeDesiredForward = true;
	}
	else intakeDesiredForward = false;
	
	if(intakeBackwardButton->IsDown()) {
		intakeDesiredBackward = true;
	}
	else intakeDesiredBackward = false;
	
	if(intakingButton->WasJustPressed()) {
		intakingDesired = true;
	} else intakingDesired = false;
	
	if(holdBallButton->WasJustPressed()) {
		holdBallDesired = true;
	} else holdBallDesired = false;

	if(intakeStartButton->WasJustPressed()) {
		intakeStartDesired = true;
	} else intakeStartDesired = false;

	if(intakeShootPositionButton->WasJustPressed()) {
		intakeShootPositionDesired = true;
	} else intakeShootPositionDesired = false;
	
	if(!popBallButton->IsDown()) {
		popBallPistonOutDesired = true;
	} else popBallPistonOutDesired = false;
	
	if(popBallButton->IsDown()) {
		popBallPistonInDesired = true;
	} else popBallPistonInDesired = false;

	if(shooterButton->WasJustPressed()) {
		shooterDesired = true;
	}
	else shooterDesired = false;

	if (armingButton->WasJustPressed()) {
		armingDesired = true;
	}
	else armingDesired = false;

	
	if (highlowGearButton->IsDown()) {
		lowGearDesired = true;
		highGearDesired = false;
	}
	else {
		lowGearDesired = false;
		highGearDesired = true;
	}
	
	if (autoManualGearButton->IsDown()) {
		autoGearShiftDesired = true;
		manualGearShiftDesired = false;
	}
	else {
		autoGearShiftDesired = false;
		manualGearShiftDesired = true;
	}

	if (quickTurnButton->IsDown()) {
		quickTurnDesired = true;
	}

	else quickTurnDesired = false;
	
	if (reverseDriveButton->IsDown()){
		reverseDriveDesired = true;
	}
	else reverseDriveDesired = false;
	
	if (fabs(rightJoy->GetRawAxis(Joystick::kDefaultXAxis)) > 0.3){
		sharpTurnDesired = true;
	}
	else sharpTurnDesired = false;
}

float ControlBoard::ArcadeDrive(Wheels w)
{
	float moveValue = 0.0;
	float rotateValue = 0.0;
	// The drive station joysticks are wired as follows:
	//   - if the left joystick is pushed forward, it produces negative values
	//   - if the right joystick is pushed to the right, it produces positive values
	if (reverseDriveDesired) {
		moveValue = leftJoy->GetRawAxis(Joystick::kDefaultYAxis);
		rotateValue = -(rightJoy->GetRawAxis(Joystick::kDefaultXAxis));
	}
	else {
		// The sign of the left joystick value needs to be negated
		// since we want a positive value to mean "go forward"
		moveValue = -(leftJoy->GetRawAxis(Joystick::kDefaultYAxis));
		rotateValue = rightJoy->GetRawAxis(Joystick::kDefaultXAxis);
	}

	float overPower = 0.0;

	float sensitivity = 0.8;
	float leftMotorOutput;
	float rightMotorOutput;

#define M_PI 3.1415926535
	double wheelNonLinearity;
	if (1) {//need to implement high gear
		wheelNonLinearity = 0.7; // used to be csvReader->TURN_NONLIN_HIGH
		// Apply a sin function that's scaled to make it feel better.
		rotateValue = sin(M_PI / 2.0 * wheelNonLinearity * rotateValue) / sin(M_PI / 2.0 * wheelNonLinearity);
		rotateValue = sin(M_PI / 2.0 * wheelNonLinearity * rotateValue) / sin(M_PI / 2.0 * wheelNonLinearity);
	} else {
		wheelNonLinearity = 0.4; // used to be csvReader->TURN_NONLIN_LOW
		// Apply a sin function that's scaled to make it feel better.
		rotateValue = sin(M_PI / 2.0 * wheelNonLinearity * rotateValue) / sin(M_PI / 2.0 * wheelNonLinearity);
		rotateValue = sin(M_PI / 2.0 * wheelNonLinearity * rotateValue) / sin(M_PI / 2.0 * wheelNonLinearity);
		rotateValue = sin(M_PI / 2.0 * wheelNonLinearity * rotateValue) / sin(M_PI / 2.0 * wheelNonLinearity);
	}
	if (quickTurnDesired) {
		overPower = 1.0;
		sensitivity = 1.0;
		if (reverseDriveDesired) {
			rotateValue = -rotateValue;
		}
	} else {
		overPower = 0.0;
		rotateValue = (moveValue) * rotateValue * sensitivity; // try to get reverse steering to work
	} 

	rightMotorOutput = leftMotorOutput = moveValue;
	leftMotorOutput += rotateValue;
	rightMotorOutput -= rotateValue;

	if (leftMotorOutput > 1.0) {
		rightMotorOutput -= overPower*(leftMotorOutput - 1.0);
		leftMotorOutput = 1.0;
	} else if (rightMotorOutput > 1.0) {
		leftMotorOutput -= overPower*(rightMotorOutput - 1.0);
		rightMotorOutput = 1.0;
	} else if (leftMotorOutput < -1.0) {
		rightMotorOutput += overPower*(-1.0 - leftMotorOutput);
		leftMotorOutput = -1.0;
	} else if (rightMotorOutput < -1.0) {
		leftMotorOutput += overPower*(-1.0 - rightMotorOutput);
		rightMotorOutput = -1.0;
	}

	switch(w) {
	case kLeftWheel:
		return leftMotorOutput;
	case kRightWheel:
		return rightMotorOutput;
	default: 
		return 0.0;
	}
}

ControlBoard::~ControlBoard()
{
}

