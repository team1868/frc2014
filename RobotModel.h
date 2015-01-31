
#include "WPILib.h"
#include "ini.h"
#include "Debugging.h"
#include "RobotPorts2014.h"


#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

#define PI 3.1415927

class RobotModel
{
public:
	
	enum Wheels { kLeftWheel, kRightWheel, kBothWheels };	
	
	Ini *pini;
		
	Encoder *rightWheelEncoder; 
	Encoder *leftWheelEncoder;
	Encoder *catapultEncoder;
#ifdef USE_CAMERA
	AxisCamera *camera;
#endif
	Compressor* compressor;
	
	DriverStationLCD* dsLCD;

	// ADXL345_I2C* accel;
	Gyro* gyro;
	Timer* timer;
	
	DigitalInput* limitSwitch;
	DigitalInput* opticalSensor;
		
	RobotModel();
	
	void ResetGyro();
	float GetGyroAngle();
	
    //	void PrintToLCD(int line, char message);
	double GetLeftWheelEncoderSpeed();
	double GetRightWheelEncoderSpeed();
	double GetWheelEncoderSpeed(); //average
	double GetWheelEncoderDistance(Wheels w);
	void ResetEncoder(Encoder *e);
	
	void EnableCompressor();
	void DisableCompressor();
	bool GetCompressorState();
	
	// void ResetWheelEncoder(RobotModel::Wheels w = RobotModel::kBothWheels);
	
	void SetWheelSpeed(Wheels w, double speed);
	void SetCatapultMotorSpeed(double speed);

	bool IsLowGear();
	void ShiftToHighGear();
	void ShiftToLowGear();
	void ShiftGear();
#ifdef NEWBOT	
	void ShiftCatapultToLowGear();
	void ShiftCatapultToHighGear();
	void EngageLatch();
	void ReleaseLatch();
	
	bool IsLowerIntakeIn();
	void LowerIntakeArmIn(bool in);
	Solenoid* GetLowerIntakePiston();
	void MoveLowerIntakeArmIn();
	void MoveLowerIntakeArmOut();
	bool IsUpperIntakeIn();
	void UpperIntakeArmIn(bool in);
	Solenoid* GetUpperIntakePiston();
	void MoveUpperIntakeArmIn();
	void MoveUpperIntakeArmOut();
	Solenoid* GetPopBallPiston();
	void MovePopBallPistonIn();
	void MovePopBallPistonOut();
	void SetIntakeSpeed(double speed);
#endif	
#ifdef USE_CAMERA
	HSLImage *GetCameraImage();
#endif
	bool OpticalSensorActivated();

	//bool hasCamera();
	
	double GetCurrentTimeInSeconds();
	void ResetTimer();
	
	void RefreshIni();
	
	virtual ~RobotModel();	

#ifdef USE_CAMERA
	bool HasCamera() { return hasCamera; }
	
	bool CheckCameraConnection();
#endif

#ifdef NEWBOT
	Talon *driveLeftA, *driveLeftB, *driveRightA, *driveRightB;
	Talon *catapultMotorA, *catapultMotorB;
	Talon *intakeMotorA, *intakeMotorB;
#else
	Victor *driveLeftA, *driveLeftB, *driveRightA, *driveRightB;
#endif
private:	
		
	bool isLowGear;
	Solenoid* shifterSolenoid;
#ifdef NEWBOT
	Solenoid* catapultShifterSolenoid;
	Solenoid* latchSolenoid;
	Solenoid* popBallSolenoid;
	Solenoid* intakeUpperArmSolenoidA;
	Solenoid* intakeLowerArmSolenoidA;
	Solenoid* intakeUpperArmSolenoidB;
	Solenoid* intakeLowerArmSolenoidB;
	bool isLowerIntakeIn;
	bool isUpperIntakeIn;
#endif
#ifdef USE_CAMERA
	bool hasCamera;
#endif
};

#endif /*ROBOTMODEL_H_*/
