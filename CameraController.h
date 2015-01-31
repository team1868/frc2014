#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "WPILib.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"

class CameraController {
public:
	CameraController(RobotModel *myRobot, RemoteController *myHumanControl);
#ifdef USE_CAMERA
	void RequestCameraImage();
	void Update(double currTimeSec, double deltaTimeSec);	
	void RefreshIni();
	HSLImage* GetCameraImage();
	
	void DumpImage(HSLImage *image, char *src);
#endif
	void Reset();
	void WriteImage(HSLImage *image, char *src);
	~CameraController();
	bool DistortedTargetFound();
	bool HasCameraImage();
	bool targetFound;
	
private:
	bool HeightWidthMatch(MeasureParticlesReport* myParticleReport, int myHTarget, int myVTarget);
	bool TopBottomMatch(MeasureParticlesReport* myParticleReport, int myHTarget, int myVTarget);
	bool IsVerticalTarget(MeasureParticlesReport* myParticleReport, int myTarget);
	bool IsHorizontalTarget(MeasureParticlesReport* myParticleReport, int myTarget);
	RobotModel *robot;
	RemoteController *humanControl;
	bool cameraImageRequested;
	bool hasCamera;
	int colorThresholdRMin;
	int colorThresholdRMax;
	int colorThresholdGMin;
	int colorThresholdGMax;
	int colorThresholdBMin;
	int colorThresholdBMax;
	HSLImage* cameraImage;
	int hMaxHeight, hMinHeight, hMaxWidth, hMinWidth, hMaxArea, hMinArea;
	int vMaxHeight, vMinHeight, vMaxWidth, vMinWidth, vMaxArea, vMinArea;
};


#endif
