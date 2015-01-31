#include "DriveController.h"
#include "ControlBoard.h"
#include "RobotPorts2014.h"
#include "WPILib.h"
#include <math.h>


DriveController::DriveController(RobotModel *myRobot, RemoteController *myHumanControl){
	robot = myRobot;
	humanControl = myHumanControl;

	m_stateVal = kInitialize;

	autoDriveRequested = false;
	autoPivotRequested = false;
	desiredAutoDistance = 5.0;
	autoSpeed = 0.5;
	
	autoShiftCounter = 0;
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	
	if (autoShiftCounter > 0) { // last shift was less than 500 ms ago
		autoShiftCounter++;
		if (autoShiftCounter >= 50){ // last shift is now more than 500 ms ago
			autoShiftCounter = 0; // thus we can shift again
		}
	}
	
	switch (m_stateVal)
	{
	case (kInitialize):
		nextState = kTeleopDrive;
		break;

	case (kReset):
	
			nextState = kTeleopDrive;
			break;
			
	case (kTeleopDrive):
			//ARCADE DRIVE
			
			robot->SetWheelSpeed(RobotModel::kLeftWheel,humanControl->GetLeftWheelMotorValues());
			robot->SetWheelSpeed(RobotModel::kRightWheel,humanControl->GetRightWheelMotorValues());
//			DO_PERIODIC(50, printf("Right"
//					" Wheel Encoder Speed: %f\n", (robot->GetRightWheelEncoderSpeed())));
			
			//AUTOMATIC TRANSMISSION VS MANUAL OVERRIDE
	if(humanControl->ManualGearShiftDesired()) {
		if(humanControl->LowGearDesired()){
			if (!(robot->IsLowGear())){
				printf("Manual override shifting to low gear\n");
				robot->ShiftToLowGear();
			}
		} else if(humanControl->HighGearDesired()) {
			if (robot->IsLowGear()) {
				printf("Manual override shifting to high gear\n");
				robot->ShiftToHighGear();
			}
		}
	}
	
	if (humanControl->AutoGearShiftDesired() && (autoShiftCounter == 0)) {
			//DO_PERIODIC(20, printf("Average encoder speed%f\n", robot->GetWheelEncoderSpeed()));
			if (humanControl->SharpTurnDesired()){
				//no shifting while turning so do nothing
			}
			else if (robot->IsLowGear()){
				if (fabs(robot->GetWheelEncoderSpeed()) > 3.0){
					printf("Gear shift because speed spiked in AUTO SHIFT\n");
					robot->ShiftGear();
					autoShiftCounter = 1; // restart counter
				}
			}
			else if (!(robot->IsLowGear())){
				if (fabs(robot->GetWheelEncoderSpeed()) < 1.0 ){
					printf("Gear shift because speed dropped in AUTO SHIFT\n");
					robot->ShiftGear();
					autoShiftCounter = 1; // restart counter
				}
			}
	}
	
	nextState = kTeleopDrive;
	break;
	
	}				
	m_stateVal = nextState; 

}

void DriveController::Reset() {
	m_stateVal = kInitialize;

	autoDriveRequested = false;
	autoPivotRequested = false;
	desiredAutoDistance = 5.0;
	autoSpeed = 0.5;

}

DriveController::~DriveController() {
}
