#include "CameraController.h"
#include "WPILib.h"
#include "Debugging.h"

#include <math.h>

CameraController::CameraController(RobotModel *myRobot,
		RemoteController *myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
#ifdef USE_CAMERA
	hasCamera = robot->HasCamera();
#endif
	cameraImage = NULL;
	cameraImageRequested = false;
	targetFound = false;
}

#ifdef USE_CAMERA
void CameraController::Update(double currTimeSec, double deltaTimeSec) {
	if (hasCamera == false) {
		return;
	}
	if (humanControl->CameraImageRequested() || cameraImageRequested) {
		printf("Camera Image requested");
		delete cameraImage;
		cameraImage = robot->GetCameraImage();
		WriteImage(cameraImage, "rawimage.bmp");
		printf("Just wrote camera image to the robot\n");
		cameraImageRequested = false;
		robot->dsLCD->UpdateLCD();
	}
}

void CameraController::RequestCameraImage() {
	cameraImageRequested = true;
	printf("Camera Image Requested \n");
}

HSLImage* CameraController::GetCameraImage() {
	printf("Getting Camera Image \n");
	cameraImage = robot->GetCameraImage();
	return cameraImage;
}

bool CameraController::HasCameraImage() {
	if (cameraImage != NULL) {
		return true;
	} else {
		return false;
	}
}

void CameraController::WriteImage(HSLImage *image, char *src) {
	int x = imaqWriteBMPFile(image->GetImaqImage(), src, FALSE, NULL);
	if (x == 0)
		printf("Error writing image\n");
}

bool CameraController::DistortedTargetFound() {
	printf("Looking for distorted targets \n");
	int rc;

	Image *thresholdImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);
	Image *convexHullImage =
			imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);
	Image *particleImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);
	Image *equalizedImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);

	/*
	 set the color thresholding process maximum and minimum values
	 color thresholding is a step used to process the image from the camera
	 */
	Range thresholdRangeR;
	thresholdRangeR.minValue = colorThresholdRMin;
	
	thresholdRangeR.maxValue = colorThresholdRMax;
	Range thresholdRangeG;
	thresholdRangeG.minValue = colorThresholdGMin;
	thresholdRangeG.maxValue = colorThresholdGMax;
	Range thresholdRangeB;
	thresholdRangeB.minValue = colorThresholdBMin;
	thresholdRangeB.maxValue = colorThresholdBMax;
/*
	 for all imaq functions, imaq function returns either 0 or 1
	 0 if there is an error during function
	 1 if fucntion succeeds
	 
	 steps for processing the image to get the particles from the image for analysis:
	 1. color threshold - posterize
	 2. convex hull - fill in the particles for shape
	 3. remove small particles
	 4. equalize - turn it black and white
	 5. find the particles and certain properties of them
*/
	rc = imaqColorThreshold(thresholdImage, cameraImage->GetImaqImage(), 255,
			IMAQ_RGB, &thresholdRangeR, &thresholdRangeG, &thresholdRangeB);

	if (rc == 0)
		printf("Thresholding error %d", imaqGetLastError());
	rc = imaqWriteBMPFile(thresholdImage, "thresholdImage.bmp", FALSE, NULL);
	if (rc == 0)
		printf("Error writing image\n");

	rc = imaqConvexHull(convexHullImage, thresholdImage, 1);
	if (rc == 0)
		printf("Convex hull error # %d", imaqGetLastError());
	rc = imaqDispose(thresholdImage);
	if (rc == 0)
		printf("Delete threshold image error # %d", imaqGetLastError());
	rc = imaqWriteBMPFile(convexHullImage, "convexHullImage.bmp", FALSE, NULL);
	if (rc == 0)
		printf("Error writing image\n");

	rc = imaqSizeFilter(particleImage, convexHullImage, true, 3,
			IMAQ_KEEP_LARGE, NULL);
	if (rc == 0)
		printf("Remove Small Particles Error # %d", imaqGetLastError());
	rc = imaqDispose(convexHullImage);
	if (rc == 0)
		printf("Delete convexHull image error # %d", imaqGetLastError());
	rc = imaqWriteBMPFile(particleImage, "particleImage.bmp", FALSE, NULL);

	rc = imaqEqualize(equalizedImage, particleImage, 0, 255, NULL);
	if (rc == 0)
		printf("Equalizing error # %d", imaqGetLastError());
	rc = imaqDispose(particleImage);
	if (rc == 0)
		printf("Delete convex hull image error # %d", imaqGetLastError());
	rc = imaqWriteBMPFile(equalizedImage, "equalizedImage.bmp", FALSE, NULL);
	if (rc == 0)
		printf("Error writing image\n");
	
	int numParticles;
	rc = imaqCountParticles(equalizedImage, TRUE, &numParticles);
	printf("Particle report size: %d\n", numParticles);
	
	if (rc == 0) {
		printf("Error counting particles \n");
	}
	
	MeasurementType measurements[] = {IMAQ_MT_BOUNDING_RECT_LEFT,
									IMAQ_MT_BOUNDING_RECT_TOP,
									IMAQ_MT_BOUNDING_RECT_WIDTH,
									IMAQ_MT_BOUNDING_RECT_HEIGHT,
									IMAQ_MT_AREA,
									IMAQ_MT_BOUNDING_RECT_BOTTOM};
	MeasureParticlesReport* particleReport = imaqMeasureParticles(equalizedImage, 
													IMAQ_CALIBRATION_MODE_PIXEL,
													measurements, sizeof(measurements)/sizeof(measurements[0]));
	
	for (int i = 0; i < particleReport->numParticles; i++) {
		printf("Particle ID %d : ", i);
		for (int j = 0; j < particleReport->numMeasurements; j++) {
			printf("%f ", particleReport->pixelMeasurements[i][j]);
		}
		printf("\n");
	}
	
	vector<int> horizontalTargets;
	vector<int> verticalTargets;
	/*
	 puts particles that past the horizontal or/and vertical test into a vector
	 */
	for (int i = 0; i < particleReport->numParticles; i++) {
		if (IsHorizontalTarget(particleReport, i)) {
			horizontalTargets.push_back(i);
		} 
		if (IsVerticalTarget(particleReport, i)) {
			verticalTargets.push_back(i);
		}
	}
	
	if (horizontalTargets.empty()) {
		printf("No horizontal targets \n");
		return false;
	}
	if (verticalTargets.empty()) {
		printf("No vertical targets!!! \n");
		return false;
	}
	
	int targetMatchCounter = 0;
	/*
	 for each pair of horizontal and vertical targets in the corresponding vectors
	 put the pair through tests to determind if they are the hot goals
	 */
	for (int i = 0; i < horizontalTargets.size(); i++) {
		for (int j = 0; j < verticalTargets.size(); j++) {
			int hTarget = horizontalTargets[i];
			int vTarget = verticalTargets[j];
			printf("Analyzing particles h: %d v: %d \n", hTarget, vTarget);
			if (hTarget == vTarget) {
				continue;
			}
			bool hwMatch = HeightWidthMatch(particleReport, hTarget, vTarget);
			if (hwMatch) {
				printf("Good Height Width \n");
				bool tbMatch =
						TopBottomMatch(particleReport, hTarget, vTarget);
				if (tbMatch) {
					printf("Top Bottom Match good \n");
					targetMatchCounter = targetMatchCounter + 1;
					continue;
				} else {
					printf("Top Bottom fail \n");
					continue;
				}
			} else {
				printf("Height Width fail \n");
				continue;
			}
			
		}
	}
	
	if (targetMatchCounter == 0) {
		printf("No matches! uh oh? \n");
		targetFound = false;
		return false;	
	} else if (targetMatchCounter > 1) {
		printf("Too many matches! %d", targetMatchCounter);
		targetFound = false;
		return false;
	} else {
		printf("Target found \n");
		targetFound = true;
		return true;
	}
}

bool CameraController::HeightWidthMatch(MeasureParticlesReport* myParticleReport, 
								int myHTarget, int myVTarget){
	MeasureParticlesReport* particleReport = myParticleReport;
	int hTarget = myHTarget;
	int vTarget = myVTarget;
	
	double hHeight = particleReport->pixelMeasurements[hTarget][3];
	double vWidth = particleReport->pixelMeasurements[vTarget][2];
	
	double hwFactor = 10.5;
	/*
	 if the height of the horizontal target is within a certain factor of the width
	 of the vertical target, return true
	 */
	if ((hHeight < vWidth + hwFactor) && (hHeight > vWidth - hwFactor)) {
		printf("%d height: %f close to %d width: %f \n", hTarget, hHeight, vTarget, vWidth);
		return true;
	} else if(hHeight > vWidth + hwFactor) {
		printf("%d height: %f too large to %d width: %f \n", hTarget, hHeight, vTarget, vWidth);
		return false;
	} else {
		printf("%d height: %f too small to %d width: %f \n", hTarget, hHeight, vTarget, vWidth);
		return false;
	}
}

bool CameraController::TopBottomMatch(MeasureParticlesReport* myParticleReport, int myHTarget, int myVTarget) {
	MeasureParticlesReport* particleReport = myParticleReport;
	int horizontalTarget = myHTarget;
	int verticalTarget = myVTarget;
	
	double verticalTop = particleReport->pixelMeasurements[verticalTarget][1];
	double horizontalTop = particleReport->pixelMeasurements[horizontalTarget][1];
	double horizontalBottom = particleReport->pixelMeasurements[horizontalTarget][5];
	/*
	 if the coordinate of the vertical top is within the coordinates of the height of
	 the horizontal targets, return true
	 */
	if ((verticalTop > horizontalTop) && (verticalTop < horizontalBottom)){
		printf("%d top %f within %d top: %f and bottom: %f \n", verticalTarget, verticalTop,
				horizontalTarget, horizontalTop, horizontalBottom);
		return true;
	} else if (verticalTop < horizontalTop) {
		printf("%d top %f above %d top %f \n", verticalTarget, verticalTop, horizontalTarget, horizontalTop);
		return false;
	} else {
		printf("%d top %f below %d bottom %f \n", verticalTarget, verticalTop, horizontalTarget, horizontalBottom);
		return false;
	}
}

bool CameraController::IsVerticalTarget(MeasureParticlesReport* myParticleReport, int myTarget) {
	/*
	 if the height, width and area are within the maximum and minimum in order to be
	 a vertical target, the particle passes
	 */
	MeasureParticlesReport* particleReport = myParticleReport;
	int target = myTarget;

	double height = particleReport->pixelMeasurements[target][3];
	double width = particleReport->pixelMeasurements[target][2];
	double area = particleReport->pixelMeasurements[target][4];
	
	vMaxHeight = robot->pini->geti("CameraControllerValues", "VerticalMaxHeight", 150);
	vMinHeight = robot->pini->geti("CameraControllerValues", "VerticalMinHeight", 90);
	vMinWidth = robot->pini->geti("CameraControllerValues", "VerticalMinWidth", 12);
	vMaxWidth = robot->pini->geti("CameraControllerValues", "VerticalMaxWidth", 25);
	vMaxArea = robot->pini->geti("CameraControllerValues", "VerticalMaxArea", 1900);
	vMinArea = robot->pini->geti("CameraControllerValues", "VerticalMinArea", 900);
	
	if (height > vMaxHeight){
		printf("Target: %d not Vertical b/c height %f greater than %d \n", 
				target, height, vMaxHeight);
		return false;
	} else if (height < vMinHeight) {
		printf("Target: %d not Vertical b/c height %f less than %d \n", 
					target, height, vMinHeight);
		return false;
	} else if (width > vMaxWidth) {
		printf("Target: %d not Vertical b/c width %f greater than %d \n", 
						target, width, vMaxWidth);
		return false;
	} else if (width < vMinWidth) {
		printf("Target: %d not Vertical b/c width %f less than %d \n", 
						target, width, vMinWidth);
		return false;
	} else if (area > vMaxArea) {
		printf("Target: %d not Vertical b/c area %f greater than %d \n", 
								target, area, vMaxArea);
		return false;
	} else if (area < vMinArea) {
		printf("Target: %d not Vertical b/c area %f less than %d \n", 
								target, area, vMinArea);
		return false;
	} else {
		return true;
	}
}

bool CameraController::IsHorizontalTarget(MeasureParticlesReport* myParticleReport, int myTarget) {
	/*
		 if the height, width and area are within the maximum and minimum in order to be
		 a horizontal target, the particle passes
		 */
	MeasureParticlesReport* particleReport = myParticleReport;
	int target = myTarget;

	double height = particleReport->pixelMeasurements[target][3];
	double width = particleReport->pixelMeasurements[target][2];
	double area = particleReport->pixelMeasurements[target][4];
	
	hMaxHeight = robot->pini->geti("CameraControllerValues", "HorizontalMaxHeight", 25);
	hMinHeight = robot->pini->geti("CameraControllerValues", "HorizontalMinHeight", 10);
	hMinWidth = robot->pini->geti("CameraControllerValues", "HorizontalMinWidth", 60);
	hMaxWidth = robot->pini->geti("CameraControllerValues", "HorizontalMaxWidth", 100);
	hMaxArea = robot->pini->geti("CameraControllerValues", "HorizontalMaxArea", 1300);
	hMinArea = robot->pini->geti("CameraControllerValues", "HorizontalMinArea", 500);
	
	if (height > hMaxHeight){
		printf("Target: %d not Horizontal b/c height %f greater than %d \n", 
				target, height, hMaxHeight);
		return false;
	} else if (height < hMinHeight) {
		printf("Target: %d not Horizontal b/c height %f less than %d \n", 
					target, height, hMinHeight);
		return false;
	} else if (width > hMaxWidth) {
		printf("Target: %d not Horizontal b/c width %f greater than %d \n", 
						target, width, hMaxWidth);
		return false;
	} else if (width < hMinWidth) {
		printf("Target: %d not Horizontal b/c width %f less than %d \n", 
						target, width, hMinWidth);
		return false;
	} else if (area > hMaxArea) {
		printf("Target: %d not Horizontal b/c area %f greater than %d \n", 
								target, area, hMaxArea);
		return false;
	} else if (area < hMinArea) {
		printf("Target: %d not Horizontal b/c area %f less than %d \n", 
								target, area, hMinArea);
		return false;
	} else {
		return true;
	}	
}

void CameraController::RefreshIni() {
	colorThresholdRMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_R_MIN", 80);
	colorThresholdRMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_R_MAX", 255);
	colorThresholdGMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_G_MIN", 193);
	colorThresholdGMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_G_MAX", 255);
	colorThresholdBMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_B_MIN", 225);
	colorThresholdBMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_B_MAX", 255);
	hMaxHeight = robot->pini->geti("CameraControllerValues", "HorizontalMaxHeight", 25);
	hMinHeight = robot->pini->geti("CameraControllerValues", "HorizontalMinHeight", 10);
	hMinWidth = robot->pini->geti("CameraControllerValues", "HorizontalMinWidth", 60);
	hMaxWidth = robot->pini->geti("CameraControllerValues", "HorizontalMaxWidth", 100);
	hMaxArea = robot->pini->geti("CameraControllerValues", "HorizontalMaxArea", 1300);
	hMinArea = robot->pini->geti("CameraControllerValues", "HorizontalMinArea", 500);
	vMaxHeight = robot->pini->geti("CameraControllerValues", "VerticalMaxHeight", 150);
	vMinHeight = robot->pini->geti("CameraControllerValues", "VerticalMinHeight", 90);
	vMinWidth = robot->pini->geti("CameraControllerValues", "VerticalMinWidth",	12);
	vMaxWidth = robot->pini->geti("CameraControllerValues", "VerticalMaxWidth", 25);
	vMaxArea = robot->pini->geti("CameraControllerValues", "VerticalMaxArea", 1900);
	vMinArea = robot->pini->geti("CameraControllerValues", "VerticalMinArea", 900);

}

#endif

void CameraController::Reset() {
	targetFound = false;
}

CameraController::~CameraController() {

}
