#ifndef DSLCDPRINTER_H
#define DSLCDPRINTER_H

#include "WPILib.h"
#include "RobotModel.h"
#include "CameraController.h"
#include "ShooterController.h"
#include <queue>
#include <vector>
#include <string>

class LCDMessage {
	//drive station message
public:
	LCDMessage() {}
	virtual ~LCDMessage() {}
	virtual void AddMessages(vector<string>* printQueue) = 0;
private:
};



class DSLCDPrinter {
public:
	DSLCDPrinter(DriverStationLCD* myDSLCD) {
		dsLCD = myDSLCD;
	}
	virtual ~DSLCDPrinter() {}
	void Update();
	void Clear();
	void AddLCDMessage(LCDMessage* myLCDMessage);
private:
	DriverStationLCD* dsLCD;
	vector<string> printQueue;
	vector<LCDMessage*> lcdMessages;
};

class RobotModelLCDMessage : public LCDMessage {
public:
	RobotModelLCDMessage(RobotModel* myRobot);
	virtual ~RobotModelLCDMessage() {}
	virtual void AddMessages(vector<string>* printQueue);
private:
	RobotModel* robot;
};

class CameraControllerLCDMessage : public LCDMessage {
public:
	CameraControllerLCDMessage(CameraController* myCameraController);
	virtual ~CameraControllerLCDMessage() {}
	virtual void AddMessages(vector<string>* printQueue);
private:
	CameraController* cameraController;
};

#endif  // DSLCDPRINTER
