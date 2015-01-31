#ifndef AUTONOMOUSCONTROLLER_H_
#define AUTONOMOUSCONTROLLER_H_

#include "ControlBoard.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "CameraController.h"
#include "ShooterController.h"

#include "RobotModel.h"
#include <vector>
#include <string>
#include "AutoCommand.h"

enum AutoMode {
 kTestMode = 0, kBlankAuto = 1, kSimpleDriveAuto = 2, kSimpleShootAuto = 3, kHotAuto = 4,
	kCameraAuto = 5, kTwoBallAuto = 6
};

class AutonomousController {
public:
	AutonomousController(RobotModel *myRobot, CameraController *myCamera, ShooterController *myShooter);
	void CreateQueue();
	void StartAutonomous();
	virtual ~AutonomousController();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();

private:

	AutoCommand *firstCommand;
	AutoCommand *currentCommand;
	RobotModel *robot;
	CameraController *cameraController;
	ShooterController *shooterController;
	
	// Initialized from ini file
	double initialWait;
	double preShootWait;
	double notHotWait;
	double holdBallWait;
	double pauseBetween;
	float forwardDist;
	float maxAutoSpeed;
	unsigned int autoMode;
	double twoBallIntakingTime;
	double holdBallIntakingTime;
};

#endif /*AUTONOMOUSCONTROLLER_H_*/
