#ifndef PIDCONTROLLOOP_H_
#define PIDCONTROLLOOP_H_

#include "RobotModel.h"
#include "WPILib.h"
#include <math.h>

struct PIDConfig {
 public:
	PIDConfig();
	double pFac;
	double iFac;
	double dFac;
	double maxAbsOutput;
	double maxAbsError;
	double maxAbsDiffError;
	double desiredAccuracy;
	double maxAbsITerm;
};

class PIDControlLoop {
 public:
	PIDControlLoop(PIDConfig* myConfig);
	~PIDControlLoop() {}
	void Init(double myInitialSensorValue, double desiredSensorValue);
	double Update(double currentSensorValue); // Returns the actuator value (motor speed, etc.)
	bool ControlLoopDone(double currentSensorValue);

 private:
	double Saturate(double value, double maxAbsValue);
	PIDConfig* pidConfig;
	double initialSensorValue;
	double desiredSensorValue;
	double oldError;
	double sumError;
};

#endif
