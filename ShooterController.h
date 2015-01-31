#ifndef SHOOTERCONTROLLER_H_
#define SHOOTERCONTROLLER_H_

#include "Debugging.h"
#include "RemoteControl.h"
#include "RobotModel.h"

enum intakePosDesired {
	kNothingDesired, kIntakingDesired, kHoldingDesired,
	kStartingDesired, kShootingDesired
};

class ShooterController {
 public:
	ShooterController(RobotModel* myRobot, RemoteController* myHumanControl);
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();
	virtual ~ShooterController();
	
	void RequestAutoShoot();
	void RequestIntakePos(uint32_t intakeState);
	void RequestAutoIntakeRollers(bool on);
	bool AutoShootInProgress();
	
	enum ShooterState {
		kInit, kIdle, kShooting, 
		kWaiting, kArming, kLatching, 
		kArmingDone, kTest
	};
	
	enum IntakePosition {
		kStartingPos, kIntakingPos, kShootingPos, kHoldBallPos
		// starting is safe
	};

 private:
	bool ArmingDone();
	
	RobotModel* robot;
	RemoteController* humanControl;
	
	uint32_t m_stateVal;
	uint32_t nextState;
	uint32_t intakeStateDesired;
	uint32_t intakePosition;
	double shootingMotorSpeed;
	double armingMotorSpeed;
	double intakeMotorSpeed;
	int shootingDuration;
	int shootingCounter;
	int shootDelayDuration;
	int shootDelayCounter;
	int waitingDuration;
	int waitingCounter;
	bool armingInProgress;
	int latchingDuration;
	int latchingCounter;
	int useTestMode;
	double testMotorSpeed;
	int softShotLowGearDuration;
	int softShotLowGearCounter;
	int softShotHighGearDuration;
	int softShotHighGearCounter;
	bool autoShootRequested;
	int debugFrequency;

	bool autoIntakeRollersRequestedOn;
	
};

#endif
