#include "DSLCDPrinter.h"
#include "Debugging.h"
#include <vector>
#include <string>
#include <queue>
#include <iostream>
#include <stdio.h>
using namespace std;

#include <sstream>
std::string Convert (float number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}

std::string Convert (int number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}

std::string Convert (double number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}

void DSLCDPrinter::Update() {
	// updates window
	printQueue.clear();
	for (int i = 0; i < lcdMessages.size(); i++) {
		LCDMessage* lcdMessage = lcdMessages[i];
		lcdMessage->AddMessages(&printQueue);
	}
	
	for (int i = DriverStationLCD::kUser_Line1; 
			(i <= DriverStationLCD::kUser_Line6) && 
				((i - DriverStationLCD::kUser_Line1) < printQueue.size()); 
			i++) {
		string message = printQueue[i - DriverStationLCD::kUser_Line1];
		
		dsLCD->Printf(static_cast<DriverStationLCD::Line>(i), 1, message.c_str());
	}
	dsLCD->UpdateLCD();
}

void DSLCDPrinter::Clear() {
	printQueue.clear();
	dsLCD->Clear();
}

void DSLCDPrinter::AddLCDMessage(LCDMessage* myLCDMessage) {
	lcdMessages.push_back(myLCDMessage);
}

RobotModelLCDMessage::RobotModelLCDMessage(RobotModel* myRobot) {
	robot = myRobot;
}

void RobotModelLCDMessage::AddMessages(vector<string>* printQueue){
	// set buffer equal to message then push buffer
	
	// the string has a length limit for the screen to print it
	char buffer [100];
	sprintf(buffer, "Gyro angle: %f", robot->gyro->GetAngle());
	//printf("%s\n", buffer);
	printQueue->push_back(buffer);
	sprintf(buffer, "Left enc: %f", robot->GetWheelEncoderDistance(RobotModel::kLeftWheel));
	printQueue->push_back(buffer);
	float rightEncoderValue = (float)(robot->GetWheelEncoderDistance(RobotModel::kRightWheel));
	sprintf(buffer, "Right enc: %f", rightEncoderValue);
	printQueue->push_back(buffer);
	
	
	if (robot->OpticalSensorActivated()) {
		sprintf(buffer, "Optical: %s", "true ");
	} else {
		sprintf(buffer, "Optical: %s", "false");
	}
	printQueue->push_back(buffer);
	
	if (robot->limitSwitch->Get() == 1) {
		sprintf(buffer, "Limit: %s", "true");
	} else {
		sprintf(buffer, "Limit: %s", "false");
	}
	printQueue->push_back(buffer);
}

CameraControllerLCDMessage::CameraControllerLCDMessage(CameraController* myCameraController) {
	cameraController = myCameraController;
}

void CameraControllerLCDMessage::AddMessages(vector<string>* printQueue) {
	char buffer[100];
	if (cameraController->targetFound) {
		sprintf(buffer, "Hot target: %s", "true");
	} else {
		sprintf(buffer, "Hot target: %s", "false");
	}
	//printQueue->push_back(buffer);
}

