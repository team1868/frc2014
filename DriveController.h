#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"

// Use the given methods, includes, and variables as hints.

class DriveController{
public:
	DriveController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	
	void RequestAutoDrive(double distance, double speed);
	void Reset();
	
	virtual ~DriveController();
	
	enum DriveState {
			kReset, kInitialize, kTeleopDrive, 
			kAutoDriveStart, kAutoDrive, kAutoDriveDone
		};

private:
	RobotModel *robot;
	RemoteController *humanControl;
	RobotDrive *myRobotDrive;
	
	uint32_t m_stateVal;
	uint32_t nextState;
	
	int autoShiftCounter;
	
		bool autoDriveRequested, autoPivotRequested;
		double desiredAutoDistance;
		double autoSpeed;
		double initialAutoDistance;
		double currentAutoDistance;
		

};

#endif /*DRIVECONTROLLER_H_*/
