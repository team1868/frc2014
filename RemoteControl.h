#ifndef REMOTECONTROLLER_H_
#define REMOTECONTROLLER_H_

typedef struct {
	double left;
	double right;
} MotorSpeedSet;

/* RemoteController is an abstract base class with virtual methods that provide the interface 
 * for modeling the state of the driver station. We don't instantiate RemoteController objects
 * but ControlBoard is our implementation for this particular game. Methods from ControlBoard 
 * must also be declared here.
 */

class RemoteController {
	
public:
	
	virtual bool LowGearDesired() = 0;
	
	virtual bool HighGearDesired() = 0;
	
	virtual bool ManualGearShiftDesired() = 0;
	
	virtual bool AutoGearShiftDesired() = 0;
	
	virtual bool AutoDriveDesired() = 0;
	
	virtual bool ShooterDesired() = 0;
	
	virtual bool ArmingDesired() = 0;
	
	virtual bool IntakeDesiredForward() = 0;
	
	virtual bool IntakeDesiredBackward() = 0;
	
	virtual bool IntakingDesired() = 0;

	virtual bool HoldBallDesired() = 0;
	
	virtual bool IntakeStartDesired() = 0;
	
	virtual bool IntakeShootPositionDesired() = 0;
	
	virtual bool PopBallPistonInDesired() = 0;
	
	virtual bool PopBallPistonOutDesired() = 0;
	
	virtual bool CameraImageRequested() = 0;
	
	virtual void ReadControls() = 0;

	virtual bool SharpTurnDesired() = 0;
	
	virtual float GetLeftWheelMotorValues() = 0;
	
	virtual float GetRightWheelMotorValues() = 0;
		
	virtual ~RemoteController() {}

	
protected:
	MotorSpeedSet wheelSpeedSet;
	
};

#endif
