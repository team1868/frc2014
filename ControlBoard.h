#ifndef CONTROLBOARD_H_
#define CONTROLBOARD_H_

#include "WPILib.h"
#include "ButtonReader.h"
#include "RemoteControl.h"
#include "RobotModel.h"

/* ******* IMPORTANT LOOK HERE: *********
 * Make sure that when you add new buttons/joysticks, you add the corresponding 
 * state accessor method to the RemoteControl.
 */

class ControlBoard : public RemoteController
{
public:
	ControlBoard(RobotModel* myRobot);
	virtual ~ControlBoard();
	enum Wheels { kLeftWheel, kRightWheel, kBothWheels };	
	
	// inherited from RemoteControl
	virtual float GetLeftWheelMotorValues() { return ArcadeDrive(ControlBoard::kLeftWheel); };
	
	virtual float GetRightWheelMotorValues() { return ArcadeDrive(ControlBoard::kRightWheel); };
		
	virtual bool LowGearDesired() { return lowGearDesired; };
	
	virtual bool HighGearDesired() { return highGearDesired; };
	
	virtual bool AutoGearShiftDesired() { return autoGearShiftDesired; };
	
	virtual bool ManualGearShiftDesired() { return manualGearShiftDesired; };
	
	virtual bool IntakeDesiredForward() { return intakeDesiredForward; };
	
	virtual bool IntakeDesiredBackward() { return intakeDesiredBackward; };
	
	virtual bool IntakingDesired() { return intakingDesired; }
	
	virtual bool HoldBallDesired() { return holdBallDesired; }
	
	virtual bool IntakeStartDesired() { return intakeStartDesired; }
	
	virtual bool IntakeShootPositionDesired() { return intakeShootPositionDesired; }
	
	virtual bool PopBallPistonOutDesired() { return popBallPistonOutDesired; }
	
	virtual bool PopBallPistonInDesired() { return popBallPistonInDesired; }
	
	virtual bool CameraImageRequested() { return cameraImageRequested; } ;
		
	virtual bool ShooterDesired() { return shooterDesired; };	
	
	virtual bool AutoDriveDesired() { return autoDriveDesired; };	
	
	virtual bool ArmingDesired() { return armingDesired; };
	
	virtual bool SharpTurnDesired() { return sharpTurnDesired; };
	
	virtual void ReadControls();
		
private:
	
	float ArcadeDrive(Wheels w);
	
	Joystick* leftJoy;
	Joystick* rightJoy;
	Joystick* operatorJoy;
	RobotModel* robot;
	
	ButtonReader* intakeForwardButton;
	ButtonReader* intakeBackwardButton;
	ButtonReader* intakingButton;
	ButtonReader* holdBallButton;
	ButtonReader* intakeStartButton;
	ButtonReader* intakeShootPositionButton;
	ButtonReader* popBallButton;
	
	ButtonReader* highlowGearButton;
	ButtonReader* autoManualGearButton;
	ButtonReader* quickTurnButton;
	ButtonReader* reverseDriveButton;
	
	ButtonReader* shooterButton;
	ButtonReader* armingButton;
	
	bool lowGearDesired, highGearDesired;
	bool manualGearShiftDesired, autoGearShiftDesired;
	bool intakeDesiredForward, intakeDesiredBackward;
	bool intakingDesired, holdBallDesired, intakeStartDesired, intakeShootPositionDesired;
	bool popBallPistonInDesired, popBallPistonOutDesired;
	bool autoDriveDesired;
	bool quickTurnDesired, sharpTurnDesired;
	bool cameraImageRequested;
	bool reverseDriveDesired;
	bool shooterDesired;
	bool armingDesired;
};

#endif /*CONTROLBOARD_H_*/
