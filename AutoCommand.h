#ifndef AUTOCOMMAND_H
#define AUTOCOMMAND_H

#include "Debugging.h"
#include "PIDControlLoop.h"
#include "ControlBoard.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "CameraController.h"
#include "ShooterController.h"

#include "RobotModel.h"
#include <vector>
#include <string>

double Saturate(double value, double maxAbsValue);

class AutoCommand {
public:
	AutoCommand() {
	}
	virtual ~AutoCommand() {
	}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;
	virtual AutoCommand* GetNextCommand() = 0;
};

class SimpleAutoCommand: public AutoCommand {
public:
	SimpleAutoCommand() {
		nextCommand = NULL;
	}
	virtual ~SimpleAutoCommand() {
	}
	virtual void SetNextCommand(AutoCommand *cmd) {
		nextCommand = cmd;
	}
	virtual AutoCommand* GetNextCommand() {
		return nextCommand;
	}
private:
	AutoCommand* nextCommand;
};

class ConditionalAutoCommand: public AutoCommand {
public:
	ConditionalAutoCommand() {
		trueNextCommand = NULL;
		falseNextCommand = NULL;
		condition = false;
	}
	virtual ~ConditionalAutoCommand() {
	}
	virtual void SetTrueNextCommand(AutoCommand* cmd) {
		trueNextCommand = cmd;
	}
	virtual void SetFalseNextCommand(AutoCommand* cmd) {
		falseNextCommand = cmd;
	}
	virtual AutoCommand* GetNextCommand() {
		if (condition) {
			return trueNextCommand;
		} else {
			return falseNextCommand;
		}
	}
	// Subclasses must call this method to set the condition correctly after IsDone
	void SetCondition(bool c) {
		condition = c;
	}
private:
	AutoCommand* trueNextCommand;
	AutoCommand* falseNextCommand;
	bool condition;
};

class DriveCommand: public SimpleAutoCommand {
public:
	DriveCommand(double myDistance, double maxSpeed, RobotModel* myRobot);
	enum AutoDriveState {
		kAutoDriveStart, kAutoDrive, kAutoPivot, kAutoDriveDone,
	};
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	virtual double DriveStraight();
	// Come from ini file
	static double drivePFac, driveIFac, driveDFac;
	static double gyroPFac, gyroIFac, gyroDFac;

private:
	double DistanceDriven();
	PIDConfig* CreateDriveDistanceConfig();
	PIDConfig* CreateDriveStraightConfig();
	PIDControlLoop* pidDriveDistance;
	PIDControlLoop* pidDriveStraight;
	double desiredAutoDistance;
	double initialAutoDistance;
	double maxAutoSpeed;
	uint32_t nextState;
	unsigned int autoDriveState;
	bool autoDriveDone;
	RobotModel* robot;
	float initialAngle;
	double leftEncoderInitialValue;
	double rightEncoderInitialValue;
};

class PivotCommand: public SimpleAutoCommand {
public:
	PivotCommand(double myAngle, double maxSpeed, RobotModel *myRobot);
	enum AutoPivotState {
		kAutoPivotStart, kAutoPivot, kAutoPivotDone,
	};
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

	// Come from ini file
	static double pFac, iFac, dFac;
	static double desiredAccuracy;
	
private:
	PIDConfig* CreatePivotConfig();
	PIDControlLoop* pidPivot;
	double desiredAngle;
	double maxPivotSpeed;
	RobotModel *robot;
	bool isDone;

};

class ShootingCommand: public SimpleAutoCommand {
public:
	ShootingCommand(ShooterController* myShooterController) : 
		SimpleAutoCommand(),
		shooterController(myShooterController){	
	}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	
private:
	ShooterController* shooterController;
};

class IntakeRollersCommand: public SimpleAutoCommand {
public:
	IntakeRollersCommand(ShooterController* myShooterController, double myWaitTimeSec) {
		shooterController = myShooterController;
		waitTimeSec = myWaitTimeSec;
		timer = new Timer();
		rollersAreDone = false;
	}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	ShooterController* shooterController;
	double waitTimeSec; //specifies whether to turn it on or turn it off.
	Timer *timer;
	bool rollersAreDone;
};

class IntakePositionCommand : public SimpleAutoCommand {
 public:
	IntakePositionCommand(ShooterController* myShooterController, uint32_t intakePosition);
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	
static double intakeWaitTime;
 private:
	uint32_t intakePosDesired;
	ShooterController* shooterController;
	Timer* timer;
};

class WaitingCommand: public SimpleAutoCommand {
public:
	WaitingCommand(double myWaitTimeSec) :
		SimpleAutoCommand(), waitTimeSec(myWaitTimeSec) {
		timer = new Timer();
	}
	virtual void Init() {
		printf("waiting \n");
		timer->Start();
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {
	}
	virtual bool IsDone() {
		return (timer->Get() >= waitTimeSec);
	}

private:
	double waitTimeSec;
	Timer *timer;
};

class OpticalHotShootCommand: public ConditionalAutoCommand {
public:
	OpticalHotShootCommand(RobotModel* myRobot) : 
		ConditionalAutoCommand() {
		robot = myRobot;
		isHot = false;
		isDone = false;
	}
	virtual void Init() {
		isHot = robot->OpticalSensorActivated();
		SetCondition(isHot);
		isDone = true;
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {}
	virtual bool IsDone() {
		return isDone;
	}
private:
	bool isHot;
	bool imageProcessed;
	RobotModel* robot;
	bool isDone;
};

#ifdef USE_CAMERA

class HotShootCommand: public ConditionalAutoCommand {
public:
	HotShootCommand(CameraController* myCameraController) : 
		ConditionalAutoCommand() {
		cameraController = myCameraController;
		isHot = false;
		imageProcessed = false;
	}
	virtual void Init() {
		isHot = cameraController->DistortedTargetFound();
		SetCondition(isHot);
		imageProcessed = true;
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {}
	virtual bool IsDone() {
		return imageProcessed;
	}
private:
	bool isHot;
	bool imageProcessed;
	CameraController* cameraController;
};

class RequestImageCommand: public SimpleAutoCommand {
public:
	RequestImageCommand(CameraController* myCameraController) :
		cameraController(myCameraController) {
		imageAcquired = false;
	}
	;
	virtual void Init() {
		cameraController->RequestCameraImage();
	};
	virtual void Update(double currTimeSec, double deltaTimeSec) {
		cameraController->Update(currTimeSec, deltaTimeSec);
		imageAcquired = cameraController->HasCameraImage();
	};
	virtual bool IsDone() {
		return imageAcquired;
	}
private:
	CameraController* cameraController;
	bool imageAcquired;
};

#endif

#endif
