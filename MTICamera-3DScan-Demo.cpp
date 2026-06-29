// 8mm mindvision lens on BASLER acA720-520um gives FoV 34.5 x 26.2
// 6mm mindvision lens on BASLER acA720-520um gives FoV 45 x 34.5

#include "MTIDefinitions.h"
#include "MTIPylonCamera.h"
#include "MTIDevice.h"
#include <thread>
#include <mutex>
#include <direct.h>
#include <vector>
#include <iostream>

// MTI global objects
MTIDevice *mti;
MTIPylonCamera *camera;
MTIDataGenerator *datagen;
#include <chrono>

// OpenCV windows
constexpr auto windowCamera = "Camera";
constexpr auto windowControl = "Control";

// Device parameters
constexpr char cameraSettingsFile[] = "mticamera.ini";
constexpr char deviceSettingsFile[] = "mtiscanmodule.ini";
constexpr unsigned int spsDefault = 20000;
constexpr unsigned int maxSamplesPerFrame = 100000U;
constexpr unsigned int maxNumLines = 200U;

int exposure_time = 25000;
constexpr int exposure_slider_max = 50000;

unsigned int npts = maxSamplesPerFrame;
unsigned int numScanLines = 200;
float lineDuration = 0.006f; // seconds
unsigned int sps = 20000;
float xData[maxSamplesPerFrame];
float yData[maxSamplesPerFrame];
unsigned char mData[maxSamplesPerFrame];
float MEMSAngles[maxNumLines];
float calMEMSAnglesNominal[maxNumLines];
float calMEMSAnglesActual[maxNumLines];
unsigned int calMEMSLines;
cv::Point2f scanAngleOffsets;
cv::Point2f scanAngleMagnitudes;
bool xAveraging = true;
bool mouseDown = false;
cv::Point pClick, pClickPrevious;
unsigned int windowH = 700, windowW = 800;

int BWThreshold = 250;

//scanning amplitude and offset
float xAmplitude = 0.8f, yAmplitude = 0.75f, yAmplitude_sin = 0.6f,  xOffset = 0.f, yOffset = 0.f;


int g_pixelsPerLine = 6;

struct PixelAngleSample
{
    float pixel_x;
    float pixel_y;
    float mems_x;
    float mems_y;
};

std::vector<PixelAngleSample> g_pixelAngleSamples;

void SavePixelAngleSamplesToCSV(const std::string& filename)
{
    FILE* f = fopen(filename.c_str(), "w");
    if (!f)
    {
        printf("Could not open %s for writing.\n", filename.c_str());
        return;
    }

    fprintf(f, "pixel_x,pixel_y,mems_x,mems_y\n");
    for (const auto& s : g_pixelAngleSamples)
    {
        fprintf(f, "%f,%f,%f,%f\n", s.pixel_x, s.pixel_y, s.mems_x, s.mems_y);
    }

    fclose(f);
    printf("Saved %zu samples to %s\n", g_pixelAngleSamples.size(), filename.c_str());
}

static void OnTrackbarChange(int, void* userData)
{
	if (exposure_time < 100)
	{
		exposure_time = 100;
	}
	camera->SetExposureTime(exposure_time);
	cv::setTrackbarPos("Exposure Time", windowControl, exposure_time);
}

void ScaleAndOffsetArray(cv::Rect limits, unsigned int numPoints) {
	// Scale xData and yData arrays by scanAngleMagnitudes.x and scanAngleMagnitudes.y, respectively.
	// Offset xData and yData arrays post-scaling by scanAngleOffsets.x and scanAngleOffsets.y, respectively.
	// xData cannot exceed limits.x and limits.width/2
	// yData cannot exceed limits.y and limits.height/2
	// Useful for scaling normalized +-1 to desired angle
	for (int i = 0; i < numPoints; i++) {
		xData[i] = (xData[i] * scanAngleMagnitudes.x) + scanAngleOffsets.x;
		// Saturation
		xData[i] = std::max((float)limits.x, std::min((float)limits.width / 2, xData[i]));
		yData[i] = (yData[i] * scanAngleMagnitudes.y) + scanAngleOffsets.y;
		yData[i] = std::max((float)limits.y, std::min((float)limits.height / 2, yData[i]));

	}
}

void CameraAngleToMEMSCommands(unsigned int numPoints) {
	// Translate xData and yData arrays (in camera space angles) to MEMS angle
	cv::Point2d cameraAngle;
	cv::Point2d laser;
	for (int i = 0; i < numPoints; i++) {
		cameraAngle.x = xData[i];
		cameraAngle.y = yData[i];
		camera->BilinearInterpolateILUTForAngle(cameraAngle, laser);
		xData[i] = laser.x;
		yData[i] = laser.y;
	}
}

float LookupMEMSAngleCorrection(float angleNominal) {
	float dividend, divisor;
	for (int i = 0; i < calMEMSLines; i++) {
		if (angleNominal < calMEMSAnglesNominal[i]) {
			dividend = (calMEMSAnglesActual[i] * (angleNominal - calMEMSAnglesNominal[i - 1]) + calMEMSAnglesActual[i - 1] * (-angleNominal + calMEMSAnglesNominal[i]));
			divisor = (calMEMSAnglesNominal[i] - calMEMSAnglesNominal[i - 1]);
			if (divisor == 0.f)
				divisor = 0.001f;
			return dividend / divisor;
		}
	}
	return -1000000.f;
}

unsigned int PrepareScanLineData(unsigned int &sampleRate, int lines, float* angles, bool calibrateOnPlane) {
	// Calculate the raster scan based on variable settings
	int npts = 50000;
	// int pixels = 6;
    int ppRaster = 0;
	int angle = 0;
	int bidirF = 1;
	int dRotate = 0;

	// do not allow less than 2 lines to avoid divide by zero below
	lines = std::max(lines, 2);
	// get limits for scan data size
	MTIDeviceParams params;
	mti->GetDeviceParams(&params);
	unsigned int maxNumSamples = params.DeviceLimits.SamplesPerFrame_Max;
	int sps_max = params.DeviceLimits.SampleRate_Max;
	int sps_min = params.DeviceLimits.SampleRate_Min;

	// call the function which will return npts and update sampleRate by reference
	// if npts == 0, the function could not satisfy request, either too many total_pixels (lines*pixels) or too fast of a scan rate (sps_max exceeded)
	npts = datagen->LinearRasterPattern(xData, yData, mData, 1, 1, lines, g_pixelsPerLine, lineDuration, ppRaster, bidirF, dRotate, float(angle*MTI_DEGTORAD), sampleRate, sps_min, sps_max);
	//npts = datagen->AffineTransformData(xData, yData, npts, 1.f, 0.f, xOffset, yOffset);

	if (npts < 1) {
		printf("\n!! LinearRasterPattern function could not satisfy requested parameters. !!");
		printf("\nLines*Pixels total is too high (especially in point-to-point raster), or \nscan rate exceeds max SampleRate of the Controller.\n");
		printf("\nPress any key to return to the menu...");
		return -1;
	}

	if (npts > maxNumSamples) {
		printf("\nToo many samples to run. Number of points (%d) exceeds Controller buffer size (%d)\n", npts, maxNumSamples);
		_getch();
		return -1;
	}

	// Calculate commands to achieve true MEMS angle using ILUT
	cv::Point2d ILUTMaxAngles = camera->GetILUTMaxAngles();
	cv::Rect	limits( -ILUTMaxAngles.x, -ILUTMaxAngles.y, ILUTMaxAngles.x *2 , ILUTMaxAngles.y *2 );
	ScaleAndOffsetArray(limits, npts);
	CameraAngleToMEMSCommands(npts);
	// // now prepare array of X angles for 3D scanning
	for (int i=0; i<lines; i++) {
		angles[i] = -scanAngleMagnitudes.x + (float)i*(2*scanAngleMagnitudes.x)/((float)(lines-1)) + scanAngleOffsets.x;
		// check limits
		angles[i] = std::max( std::min( angles[i], (float)ILUTMaxAngles.x), (float)-ILUTMaxAngles.x );
		if (calibrateOnPlane)
			calMEMSAnglesNominal[i] = angles[i];
		else
			angles[i] = LookupMEMSAngleCorrection(angles[i]);
	}

	return npts;
}

void ScanAndDetectLineDemo() {
    system(CLEARSCREEN);
    printf("\nScan And Detect Line Demo\n\n");

    // For the laser scan, it uses the hardware trigger of the camera to precisely time the exposure
    // Try to enable hardware trigger on the camera with SetHardwareTrigger(true)
    if (camera->SetHardwareTrigger(true) != 0) {
        printf("This camera doesn't support hardware triggering. Press any key to continue...\n");
        camera->SetHardwareTrigger(false);
        _getch();
        return;
    }

    printf("\tChoose operating mode:\n");
    printf("\t(S)ingle scan\n");
    printf("\t(C)ontinuous scan\n");
    printf("\t(ESC) to return to main menu\n");

    bool continuousMode = false;
    while (int ch = _getch()) {
        if (ch == 27) // ESC to exit function
            return;
        if (ch == 'S' || ch == 's') {
            continuousMode = false;
            break;
        }
        if (ch == 'C' || ch == 'c') {
            continuousMode = true;
            break;
        }
    }

    if (continuousMode) {
        printf("\nRunning in continuous mode... Press any key to quit\n");
    }
    else {
        printf("\nRunning single scan. Press any key to return to the main menu\n");
    }

    cv::Size res(camera->GetCameraWidth(), camera->GetCameraHeight());
	cv::Mat fullScanReconstruction = cv::Mat::zeros(res, CV_8UC1);

    // Configure scan amplitudes and offsets (in camera-angle space)
    scanAngleMagnitudes = camera->GetILUTMaxAngles();
    scanAngleOffsets = cv::Point2f(scanAngleMagnitudes.x * xOffset, scanAngleMagnitudes.y * yOffset);
    scanAngleMagnitudes = cv::Point2f(scanAngleMagnitudes.x * xAmplitude, scanAngleMagnitudes.y * yAmplitude);

    int npts = PrepareScanLineData(sps, numScanLines, MEMSAngles, true);
    if (npts < 1) {
        camera->SetHardwareTrigger(false);
        mti->ResetDevicePosition(); // stop device, then send analog outputs (and device) back to origin in 25ms
        return;
    }

    // Call StartCamera() so that it will start listening to hardware triggers
    // and allow us to grab frames
    camera->StartCamera();

    // Now prepare to run the raster once with data send
    mti->SetDeviceParam(MTIParam::SampleRate, sps); // sample rate running must be same as used to generate waveforms
    mti->StopDataStream();

    auto start_scan = std::chrono::high_resolution_clock::now();

    mti->SendDataStream(xData, yData, mData, npts, 2, false, true);

    auto end_scan = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> scan_duration = end_scan - start_scan;

    printf("\n[TIMING] Physical Raster Scan Time: %.3f ms\n", scan_duration.count());

    // Set up variables for the loop, in case we are in continuous mode
    std::vector<std::vector<cv::Point>> allLines;   // Holds all scan line pixels for a complete scan
    std::vector<cv::Point> locations;               // Holds all pixels for a single scan that exceed the threshold
    std::vector<cv::Point> linePoints;              // For x averaging, the single line for each scan line (pixel width is 0 or 1)
    std::vector<std::vector<float>> yValues(res.height); // For x averaging, holds all of the x values for each y pixel
    cv::Mat frame;

    // Clear any previous samples
    g_pixelAngleSamples.clear();

    do {
        // Start the scan from the MTI device.
        // This will also trigger the camera at the start of each scan line and also
        // trigger the camera off at the end of the scan line
        mti->StartDataStream(1, false);

        // Process each frame, extracting the center of the laser line from each frame if xAveraging is true
        for (int lineIdx = 0; lineIdx < (int)numScanLines; lineIdx++) {

            // Indices of samples in xData/yData for this scan line
            int lineStart = lineIdx * g_pixelsPerLine;
            int lineEnd   = std::min(lineStart + g_pixelsPerLine, (int)npts);

            // Grab the next frame in the sequence
            frame = camera->GetFrame();
            
            // We get an empty frame on error, so skip to the next one
            if (frame.empty())
                continue;

            if (frame.channels() == 3)
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
			
			cv::bitwise_or(fullScanReconstruction, frame, fullScanReconstruction);

            // Threshold the image so that we only get the pixels that we detect as the laser
            cv::inRange(frame, cv::Scalar(BWThreshold), cv::Scalar(255), frame);

            // If nothing detected, go to next scan
            // This might happen if the laser doesn't reflect off of anything,
            // or if the threshold is set too high
            if (cv::countNonZero(frame) <= 0)
                continue;

            // Get all pixels from the laser beam and store them in locations vector
            cv::findNonZero(frame, locations);

            linePoints.clear();

            if (xAveraging) {
                // Iterate over all contours and average all x values for a given y pixel.
                // We need a vector of points for each vertical pixel
                std::fill(yValues.begin(), yValues.end(), std::vector<float>(0.0f));

                for (int k = 0; k < (int)locations.size(); k++) {
                    const cv::Point& p = locations[k];
                    if (p.y >= 0 && p.y < res.height)
                        yValues[p.y].push_back((float)p.x);
                }

                // Loop over all y's and calculate average x
                for (int y = 0; y < res.height; y++) {
                    if (yValues[y].empty())
                        continue;

                    float x_avg = 0.0f;
                    for (int k = 0; k < (int)yValues[y].size(); k++) {
                        x_avg += yValues[y][k];
                    }
                    x_avg /= (float)yValues[y].size();

                    linePoints.push_back(cv::Point((int)x_avg, y));
                }

                // Map this line's pixels to MEMS angles (mid-sample angle for the line)
                if (!linePoints.empty() && lineEnd > lineStart) {
					// Use the calibrated per-line angle in degrees
					float mems_x_deg = MEMSAngles[lineIdx];  // <-- degrees, not xData

					for (const auto& p : linePoints) {
						PixelAngleSample s;
						s.pixel_x = (float)p.x;
						s.pixel_y = (float)p.y;
						s.mems_x  = mems_x_deg;   // store degrees
						s.mems_y  = 0.0f;         // or a future vertical angle if you add it
						g_pixelAngleSamples.push_back(s);
					}
				}

                allLines.push_back(linePoints);
            }
            else {

                float mems_x_deg = MEMSAngles[lineIdx];

                for (const auto& p : locations) {
                    PixelAngleSample s;
                    s.pixel_x = (float)p.x;
                    s.pixel_y = (float)p.y;
                    s.mems_x  = mems_x_deg; // Map to the same line angle
                    s.mems_y  = 0.0f;
                    g_pixelAngleSamples.push_back(s);
                }
                // If not doing x averaging, just add all detected pixels from the scan line
                allLines.push_back(locations);
                // (Optional) similar logging could be added here if needed
            }
        }

		std::string compositeFilename = "raster_reconstructed.png";
		// cv::imwrite(compositeFilename, fullScanReconstruction);
		// printf("Saved combined raster image to %s\n", compositeFilename.c_str());

		// Display for the user
		cv::imshow("Final Raster Reconstruction", fullScanReconstruction);
        cv::imwrite("raster_"+ std::to_string(numScanLines) +".png", fullScanReconstruction);
		cv::waitKey(0);

        // Display the frame of detected scan lines
        // First grab a software-triggered image to draw on top of
        // We have to switch back to a software trigger, and you must
        // call StopCamera() before switching trigger modes
        camera->StopCamera();
        camera->SetHardwareTrigger(false);

        camera->StartCamera();
        frame = camera->GetFrame();
        if (frame.empty())
            frame = cv::Mat::zeros(res, CV_8UC3);
        camera->StopCamera();

        camera->SetHardwareTrigger(true);
        camera->StartCamera();

        // Convert frame to color
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

        // Draw the scanned lines as individual points by looping through each scanned line and each point in the line
        for (int i = 0; i < (int)allLines.size(); i++) {
            for (int j = 0; j < (int)allLines[i].size(); j++) {
                cv::circle(frame, allLines[i][j], 1, cv::Scalar(50, 150, 200));
            }
        }

        cv::putText(frame, "Press any key to close", cv::Point(20, 20),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 1));

        // Draw the frame
        cv::imshow(windowCamera, frame);

        // See if we have a key press in the openCV window, and quit if we do
        // If we are in single scan mode, just wait forever
        if (continuousMode) {
            int wKey = cvWaitKey(1);
            if (_kbhit() || (wKey != -1)) {
                int keyPress = (wKey != -1) ? wKey : _getch();
                (void)keyPress;
                continuousMode = false;
            }
        }
        else { // Single scan mode
            // Wait indefinitely. We check for keyboard presses on the main window as well
            while (true) {
                int wKey = cvWaitKey(30); // No need to process as fast as possible, can run at 30Hz or so
                if (_kbhit() || (wKey != -1)) {
                    int keyPress = (wKey != -1) ? wKey : _getch();
                    (void)keyPress;
                    break; // Exit the infinite while loop and continue to the end of the function
                }
            }
        }

        // Reset variables for next loop, if needed
        allLines.clear();
        linePoints.clear();

        // make sure here that the device scan is fully completed before starting another one
        while (mti->GetSamplesRemaining() != 0) {
            // Loop until no more samples
        }
    } while (continuousMode);

    // Save mapping to CSV
    if (!g_pixelAngleSamples.empty()) {
        SavePixelAngleSamplesToCSV("pixel_angle_mapping.csv");
    }

    // Always call StopCamera() when done grabbing frames
    camera->StopCamera();
    camera->SetHardwareTrigger(false);

    mti->SetDeviceParam(MTIParam::SampleRate, spsDefault);
    mti->StopDataStream();
    mti->ResetDevicePosition(); // stop device, then send analog outputs (and device) back to origin in 25ms

    cv::destroyAllWindows();
}

void ViewScanAreaBoundary() {
	system(CLEARSCREEN);
	printf("\nCheck selected scan area.");
	printf("\nThe area is based on the Scan Module's calibrated Field of Regard (CFOR) and user's amplitude and offset settings.\n");
	printf("\nHorizontal extent of the scan area of Scan Module is: %3.2fdeg to %3.2fdeg", -camera->GetILUTMaxAngles().x*(xAmplitude + xOffset), +camera->GetILUTMaxAngles().x*(xAmplitude + xOffset));
	printf("\nVertical extent of the scan area of Scan Module is: %3.2fdeg to %3.2fdeg", -camera->GetILUTMaxAngles().y*(yAmplitude + yOffset), +camera->GetILUTMaxAngles().y*(yAmplitude + yOffset));
	printf("\n\nPress any key to return to the menu...\n");

	// We draw a square with the laser of the previous ILUT values
	float xKey[6], yKey[6];
	unsigned char mKey[6];
	const unsigned int refreshRate = 40;
	const unsigned int spf = spsDefault / refreshRate;
	float xSample[spf * 2], ySample[spf * 2];
	unsigned char mSample[spf * 2];
	cv::Point2d maxAng = camera->GetILUTMaxAngles();
	cv::Point2d laser;

	if (camera->IsILUTCalibrated()) {
		// show a rectangle extension of scan area based on angle of the Scan Module
		// and amplitude and offset settings provided by user
		xKey[0] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
		xKey[1] = maxAng.x * xAmplitude + maxAng.x * xOffset;
		xKey[2] = maxAng.x * xAmplitude + maxAng.x * xOffset;
		xKey[3] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
		xKey[4] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
		yKey[0] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
		yKey[1] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
		yKey[2] = maxAng.y * yAmplitude + maxAng.y * yOffset;
		yKey[3] = maxAng.y * yAmplitude + maxAng.y * yOffset;
		yKey[4] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
		mKey[0] = 0xFF;
		mKey[1] = 0xFF;
		mKey[2] = 0xFF;
		mKey[3] = 0xFF;
		mKey[4] = 0xFF;
		// Close the polygon if necessary
		int nKey = datagen->CloseCurve(xKey, yKey, mKey, 5, 1, false);
		int numSamples = datagen->InterpolateData(xKey, yKey, mKey, xSample, ySample, mSample, nKey, spf);

		// Now convert the angles to device coordinates through binlinear interpolation
		for (int i = 0; i < numSamples; i++) {
			if (!camera->BilinearInterpolateILUTForAngle(cv::Point2d(xSample[i], ySample[i]), laser)) {
				xSample[i] = 0;
				ySample[i] = 0;
			}
			else {
				xSample[i] = laser.x;
				ySample[i] = laser.y;
			}
		}
		mti->SendDataStream(xSample, ySample, mSample, numSamples);
		mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
	}

	// Show a yellow rectangle for iLUT boundary
	cv::Point pixelTL = camera->CFORAngleToCamPixel(cv::Point2d(-1 * maxAng.x, maxAng.y));
	cv::Point pixelBR = camera->CFORAngleToCamPixel(cv::Point2d(maxAng.x, -1 * maxAng.y));
	cv::Rect rectLUT = cv::Rect(pixelTL, pixelBR);

	// Start the camera and frame triggering
	camera->SetHardwareTrigger(false);
	camera->StartCamera();
	exposure_time = 25000;
	camera->SetExposureTime(exposure_time);
	cv::namedWindow(windowControl);
	cv::createTrackbar("Exposure [us]", windowControl, &exposure_time, exposure_slider_max, &OnTrackbarChange);

	// Show the camera view while waiting
	int keyPress = -1;
	cv::Mat frame;
	do {
		frame = camera->GetFrame();
		cv::rectangle(frame, rectLUT, cv::Scalar(100, 255, 255));
		cv::imshow(windowCamera, frame);
		int wKey = cvWaitKey(1);
		if (_kbhit() || (wKey != -1))
			keyPress = (wKey != -1) ? wKey : _getch();
	} while (keyPress == -1);
	// int c = _getch(); // Consume the key if unix
	cv::destroyAllWindows();

	mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
	mti->ResetDevicePosition();
	return;
}

void SetupTriangulation() {
	// We draw a square with the laser of the previous ILUT values
	float xKey[6], yKey[6];
	unsigned char mKey[6];
	const unsigned int refreshRate = 40;
	const unsigned int spf = spsDefault / refreshRate;
	float xSample[spf * 2], ySample[spf * 2];
	unsigned char mSample[spf * 2];
	cv::Point2d maxAng = camera->GetILUTMaxAngles();
	cv::Point2d laser;

	if (camera->IsILUTCalibrated()) {
		// show a cross in center based on previously available iLUT
		xKey[0] = 0;
		xKey[1] = 0;
		xKey[2] = maxAng.x/4;
		xKey[3] = -maxAng.x/4;
		yKey[0] = -maxAng.y/4;
		yKey[1] = +maxAng.y/4;
		yKey[2] = 0;
		yKey[3] = 0;
		mKey[0] = 0xFF;
		mKey[1] = 0x00;
		mKey[2] = 0xFF;
		mKey[3] = 0xFF;
		// Close the polygon if necessary
		int nKey = datagen->CloseCurve(xKey, yKey, mKey, 4, 1, false);
		int numSamples = datagen->InterpolateData(xKey, yKey, mKey, xSample, ySample, mSample, nKey, spf);

		// Now convert the angles to device coordinates through binlinear interpolation
		for (int i = 0; i < numSamples; i++) {
			if (!camera->BilinearInterpolateILUTForAngle(cv::Point2d(xSample[i], ySample[i]), laser)) {
				xSample[i] = 0;
				ySample[i] = 0;
			}
			else {
				xSample[i] = laser.x;
				ySample[i] = laser.y;
			}
		}
		mti->SendDataStream(xSample, ySample, mSample, numSamples);
		mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
	}

    camera->LoadCameraSettings(cameraSettingsFile);

	// Start the camera and frame triggering
	camera->SetHardwareTrigger(false);
	camera->StartCamera();
	camera->SetExposureTime(exposure_time);
	cv::namedWindow(windowControl);
	cv::createTrackbar("Exposure [us]", windowControl, &exposure_time, exposure_slider_max, &OnTrackbarChange);

	// Show the camera view while waiting
	cv::Mat frame;
	bool looping = true, menuFlag = true;
	float angleCorrectionX = camera->GetCameraAngleCorrectionX();
	float planeDistance = camera->GetPlaneDistance();
	float baseDistance = camera->GetBaseDistance();
	while(looping) {
        if (menuFlag) {
			angleCorrectionX = MTI_RADTODEG * atan(baseDistance/planeDistance);
            system(CLEARSCREEN);
            printf("\nSetup Triangulation initial angle - setup camera center to match MEMS line center\n");
            printf("Camera X-Angle Correction: %3.2f degrees\n", angleCorrectionX);
            printf("Distance to Setup Plane: %3.2f mm\n", planeDistance);
            printf("Distance between Camera and Scan Module: %3.2f mm\n", baseDistance);
            printf(" (B/b) increase/decrease base distance of Camera and Scan Module\n");
            printf(" (D/d) increase/decrease distance to setup plane\n");
            printf(" (S/s)ave camera angle correction\n");
            printf(" (Q/q)uit\n");
            menuFlag = false;
        }
		frame = camera->GetFrame();
		//cv::rectangle(frame, rectLUT, cv::Scalar(100, 255, 255));
		cv::line(frame, cv::Point(camera->GetCameraWidth()/2, camera->GetCameraHeight()),cv::Point(camera->GetCameraWidth()/2, 0),cv::Scalar(100, 255, 255)); 
		cv::line(frame, cv::Point(0, camera->GetCameraHeight()/2),cv::Point(camera->GetCameraWidth(), camera->GetCameraHeight()/2),cv::Scalar(100, 255, 255)); 
		cv::imshow(windowCamera, frame);
		int wKey = cvWaitKey(1);
		if (_kbhit() || (wKey != -1)) {
			int keyPress = (wKey != -1) ? wKey : _getch();
            switch (keyPress) {
                case 'q':
                case 'Q':
                    looping = false;
                    break;
                case 'B':
                    baseDistance += 0.5;
                    menuFlag = true;
                    break;
                case 'b':
                    baseDistance -= 0.5;
                    menuFlag = true;
                    break;
                case 'D':
                    planeDistance += 0.5;
                    menuFlag = true;
                    break;
                case 'd':
                    planeDistance -= 0.5;
                    menuFlag = true;
                    break;
                case 's':
                case 'S':
                    camera->SetCameraAngleCorrectionX( angleCorrectionX );
                    camera->SetBaseDistance( baseDistance );
                    camera->SetPlaneDistance( planeDistance );
                    camera->SaveCameraSettings(cameraSettingsFile);
                    menuFlag = true;
                    break;
            }
        }
	}
	//int c = _getch(); // Consume the key if unix
	cv::destroyAllWindows();

	mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
	mti->ResetDevicePosition();
	return;
}

// This is the function that generates a triangular trajectory for the MEMS mirrors and captures an image of the laser line on the object. It saves the captured image as a PNG file.
void CaptureTriangularTrajectoryPNG(std::string filename) {
    const float fx_local = 167.0f;
    const int   Np_local = 50;
    const float Tr = (float)Np_local / (2.f * fx_local);    // scan duration
    const unsigned int sampleRate = 20000;
    
    // lead‑in gives the mirror a moment to start moving before the camera
    const float leadInTime       = 0.5f;                         // adjust as needed
    unsigned int leadInSamples   = (unsigned int)(sampleRate*leadInTime);
    unsigned int captureSamples  = (unsigned int)(sampleRate*Tr);

    unsigned int totalSamples    = leadInSamples + captureSamples;
    if (totalSamples >= maxSamplesPerFrame) totalSamples = maxSamplesPerFrame-1;

    // generate waveform for the *entire* buffer (lead‑in + capture)
    cv::Point2d maxAng = camera->GetILUTMaxAngles();
    for (unsigned int i = 0; i < totalSamples; ++i) {
        float t = (float)i / sampleRate;   // time from buffer starts
        float fy_local = fx_local * (Np_local - 1.0f) / Np_local;

        float xNorm = (2.0f / (float)CV_PI) * asinf(sinf(2.0f * (float)CV_PI * fx_local * t));
        float yNorm = (2.0f / (float)CV_PI) * asinf(sinf(2.0f * (float)CV_PI * fy_local * t));

        float tX = (xNorm * (maxAng.x * xAmplitude)) + (maxAng.x * xOffset);
        float tY = (yNorm * (maxAng.y * yAmplitude_sin)) + (maxAng.y * yOffset);

        cv::Point2d laserCommand;
        camera->BilinearInterpolateILUTForAngle(cv::Point2d(tX, tY), laserCommand);

        xData[i] = (float)laserCommand.x;
        yData[i] = (float)laserCommand.y;
        
        // --- TRIGGER LOGIC ---
        // Trigger only at first sample for single exposure spanning entire scan
        mData[i] = (i == leadInSamples) ? 0 : 255;  // 242: Trigger+Laser, 241: Laser only
        // mData[i] = (i == leadInSamples) ? 0 : 255;  // For this demo, we will just trigger at the start of the scan and keep it high for the entire duration

    }

    // camera configuration
    camera->StopCamera();
    camera->SetHardwareTrigger(true);
    int exposure_us = (int)(Tr*1e6f);                    // exposure = scan length
    camera->SetExposureTime(exposure_us);

    mti->StopDataStream();
    mti->SetDeviceParam(MTIParam::SampleRate, sampleRate);
    mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);

    camera->StartCamera();
    mti->SendDataStream(xData, yData, mData, totalSamples, 0, false, true);
    mti->StartDataStream(1, false);

    while (mti->GetSamplesRemaining() > 0) Sleep(5);
    Sleep(50);                                  // let camera finish
    cv::Mat frame = camera->GetFrame();          // now grab the full trajectory
    if (!frame.empty()) {
        cv::imwrite(filename, frame);
        cv::imshow("Triangular Capture", frame);
        cv::waitKey(500); 
        printf("Successfully captured and saved to %s\n", filename.c_str());
    } else {
        printf("Error: Frame empty. Check if laser is hitting the object.\n");
    }

    // 5. Cleanup (proper sequence)
    camera->StopCamera();
    camera->SetHardwareTrigger(false);
    mti->StopDataStream();
    mti->ResetDevicePosition();
    cv::destroyAllWindows();
}

// This is the function that generates a sinusoidal trajectory for the MEMS mirrors and captures an image of the laser line on the object. It saves the captured image as a PNG file.
void CaptureSinusoidalTrajectoryPNG(std::string filename) {
    // 1. Setup Parameters
    float fx_local = 100.0f;
    int Np_local = 50;
    float Tr = (float)Np_local / fx_local;  // Actual scan duration
    unsigned int sampleRate = 20000;
    
    // Adaptive lead-in: scale with scan duration
    float leadInTime = 0.5f;
    // leadInTime = std::max(0.05f, std::min(0.3f, leadInTime));  // Clamp between 50-300 ms
    
    unsigned int leadInSamples = (unsigned int)(sampleRate * leadInTime);
    unsigned int captureSamples = (unsigned int)(sampleRate * Tr);
    unsigned int totalSamples = leadInSamples + captureSamples;

    if (totalSamples >= maxSamplesPerFrame) totalSamples = maxSamplesPerFrame - 1;

    // 2. Generate Sinusoidal Trajectory (including lead-in)
    cv::Point2d maxAng = camera->GetILUTMaxAngles();
    float fy_local = fx_local * (Np_local - 1.0f) / Np_local;  // Y frequency adjusted for interleaving
    
    for (unsigned int i = 0; i < totalSamples; ++i) {
        float t = (float)i / (float)sampleRate;  // Time including lead-in
        
        // Sinusoidal Wave formula (pure sine/cosine)
        float xNorm = sinf(2.0f * (float)CV_PI * fx_local * t);
        float yNorm = sinf(2.0f * (float)CV_PI * fy_local * t);

        float tX = (xNorm * (maxAng.x * xAmplitude)) + (maxAng.x * xOffset);
        float tY = (yNorm * (maxAng.y * yAmplitude_sin)) + (maxAng.y * yOffset);

        cv::Point2d laserCommand;
        camera->BilinearInterpolateILUTForAngle(cv::Point2d(tX, tY), laserCommand);

        xData[i] = (float)laserCommand.x;
        yData[i] = (float)laserCommand.y;
        
        // --- TRIGGER LOGIC ---
        // Trigger at END of lead-in (when actual capture window begins)
        mData[i] = (i == leadInSamples) ? 242 : 241;  // 242: Trigger+Laser, 241: Laser only
    }

    // 3. Hardware Configuration
    camera->StopCamera();
    camera->SetHardwareTrigger(true);
    
    // Set exposure to match capture duration (NOT including lead-in)
    int exposure_us = (int)(Tr * 1000000.0f);
    camera->SetExposureTime(exposure_us);
    printf("Sinusoidal Scan: Lead-in: %.3f s, Scan duration: %.4f s, Exposure: %d us, Total samples: %d\n", 
           leadInTime, Tr, exposure_us, totalSamples);
    
    mti->StopDataStream();
    mti->SetDeviceParam(MTIParam::SampleRate, sampleRate);
    mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);

    // 4. Execution
    camera->StartCamera();
    
    auto start_scan = std::chrono::high_resolution_clock::now();

    // Send data with Repeat Count = 1 (Plays the buffer once and stops)
    mti->SendDataStream(xData, yData, mData, totalSamples, 0, false, true);

    auto end_scan = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> scan_duration = end_scan - start_scan;

    printf("\n[TIMING] Physical Sinusoidal Scan Time: %.3f ms\n", scan_duration.count());

    mti->StartDataStream(1, true);

    printf("Capturing sinusoidal trajectory (lead-in %.3f s + scan %.4f s)...\n", leadInTime, Tr);

    // Wait for the entire scan to complete before grabbing frame
    // while (mti->GetSamplesRemaining() > 0) {
    //     Sleep(5);
    // }
    // Sleep(50);  // Extra margin for camera to finish processing

    // Grab the image AFTER scan completes
    cv::Mat frame = camera->GetFrame();

    if (!frame.empty()) {
        cv::imwrite(filename, frame);
        cv::imshow("Sinusoidal Capture", frame);
        cv::waitKey(0); 
        printf("Successfully captured and saved to %s\n", filename.c_str());
    } else {
        printf("Error: Frame empty. Check if laser is hitting the object.\n");
    }

    // 5. Cleanup
    camera->StopCamera();
    camera->SetHardwareTrigger(false);
    mti->StopDataStream();
    mti->ResetDevicePosition();
    cv::destroyAllWindows();
}

// This function captures a series of frames while the MEMS mirrors follow a sinusoidal trajectory. 
// It saves each frame as a PNG file in the specified folder and reconstructs a composite image of the entire scan.
void CaptureSinusoidalFrames(std::string folderName) {
    // 1. Parameters (Strict 500 FPS)
    float fx_local = 100.0f; 
    int Np_local = 10;
    float Tr = (float)Np_local / fx_local; 
    unsigned int sampleRate = 20000;
    
    float targetFPS = 250.0f; 
    unsigned int samplesPerFrame = sampleRate / targetFPS; // 40 samples
    int exposure_us = 2000; // Leaving 800us for sensor readout/reset

    unsigned int captureSamples = (unsigned int)(sampleRate * 2 * Tr);
    int expectedFrames = (int)(captureSamples / samplesPerFrame);

    // 2. Waveform Generation with PHASE OFFSET
    // We shift the trigger by half a frame (20 samples) to catch the center crossing
    int triggerOffset = samplesPerFrame / 2; 

    cv::Point2d maxAng = camera->GetILUTMaxAngles();
    float fy_local = fx_local * (Np_local - 1.0f) / Np_local;

    for (unsigned int i = 0; i < captureSamples; ++i) {
        float t = (float)i / (float)sampleRate;
        float xNorm = sinf(2.0f * (float)CV_PI * fx_local * t);
        float yNorm = sinf(2.0f * (float)CV_PI * fy_local * t);

        cv::Point2d laserCommand;
        camera->BilinearInterpolateILUTForAngle(
            cv::Point2d((xNorm * (maxAng.x * xAmplitude)) + (maxAng.x * xOffset),
                        (yNorm * (maxAng.y * yAmplitude)) + (maxAng.y * yOffset)), 
            laserCommand
        );

        xData[i] = (float)laserCommand.x;
        yData[i] = (float)laserCommand.y;
        
        // --- IMPROVED TRIGGER LOGIC ---
        // Offset the pulse so the camera exposure window centers on the laser movement
        if ((i + triggerOffset) % samplesPerFrame < 2) {
            mData[i] = 242; // Trigger pulse
        } else {
            mData[i] = 241; // Laser ON
        }
    }

    // 3. Hardware Configuration
    camera->StopCamera();
    camera->SetHardwareTrigger(true);
    camera->SetExposureTime(exposure_us);
    
    mti->StopDataStream();
    mti->SetDeviceParam(MTIParam::SampleRate, sampleRate);
    mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);

    // 4. Capture into RAM (High Speed)
    std::vector<cv::Mat> ramBuffer;
    ramBuffer.reserve(expectedFrames); 

    camera->StartCamera();
    mti->SendDataStream(xData, yData, mData, captureSamples, 0, false, true);
    mti->StartDataStream(1, true);

    while (ramBuffer.size() < expectedFrames) {
        cv::Mat frame = camera->GetFrame(); 
        if (!frame.empty()) {
            ramBuffer.push_back(frame.clone()); 
        } else if (mti->GetSamplesRemaining() == 0) {
            break; 
        }
    }

    // 5. Save and Reconstruct (Post-Process)
    _mkdir(folderName.c_str());
    cv::Mat trajectory = cv::Mat::zeros(ramBuffer[0].size(), ramBuffer[0].type());

    for (int i = 0; i < (int)ramBuffer.size(); i++) {
        cv::imwrite(folderName + "/frame_" + std::to_string(i) + "_raw.png", ramBuffer[i]);
        // Bitwise OR is better for preserving the laser intensity than cv::max
        cv::bitwise_or(trajectory, ramBuffer[i], trajectory);
    }

    cv::imshow("Scan Result", trajectory);
    cv::waitKey(0);
    
    // Cleanup
    camera->StopCamera();
    camera->SetHardwareTrigger(false);
    mti->ResetDevicePosition();
}

// This function captures a series of frames while the MEMS mirrors follow a sinusoidal trajectory. 
// It saves each frame as a PNG file in the specified folder and reconstructs a composite image of the entire scan. 
// This version includes a phase offset to better align the camera exposure with the laser movement.
void CaptureSinusoidalFrames_changed(std::string folderName) {

    float fx_local = 100.0f; 
    int Np_local = 10;
    float Tr = (float)Np_local / fx_local; 
    unsigned int sampleRate = 20000;
    
    float targetFPS = 100.0f; 
    unsigned int samplesPerFrame = sampleRate / targetFPS;
    printf("spf=%d\n", samplesPerFrame);


    // unsigned int captureSamples = (unsigned int)(sampleRate * Tr) + samplesPerFrame;
    unsigned int captureSamples = (unsigned int)(sampleRate * Tr);
    printf("captureSamples=%d\n", captureSamples);
    int expectedFrames = (int)(captureSamples / samplesPerFrame);

    cv::Point2d maxAng = camera->GetILUTMaxAngles();
    float fy_local = fx_local * (Np_local - 1.0f) / Np_local;

    for (unsigned int i = 0; i < captureSamples; ++i) {
        float t = (float)i / (float)sampleRate;
        float xNorm = sinf(2.0f * (float)CV_PI * fx_local * t);
        float yNorm = sinf(2.0f * (float)CV_PI * fy_local * t);

        cv::Point2d laserCommand;
        camera->BilinearInterpolateILUTForAngle(
            cv::Point2d((xNorm * (maxAng.x * xAmplitude)) + (maxAng.x * xOffset),
                        (yNorm * (maxAng.y * yAmplitude_sin)) + (maxAng.y * yOffset)), 
            laserCommand
        );

        xData[i] = (float)laserCommand.x;
        yData[i] = (float)laserCommand.y;
        
        if (i % samplesPerFrame > 1) {
            mData[i] = 255;
        } else {
            mData[i] = 0; 
        }
    }

    camera->StopCamera();
    camera->SetHardwareTrigger(true); // Forces camera to wait for mData pulses on Line 4
    
    mti->StopDataStream();
    mti->SetDeviceParam(MTIParam::SampleRate, sampleRate);
    mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1); // Ensure Sync Port is active

    // 4. Capture into 
    std::vector<cv::Mat> ramBuffer;
    ramBuffer.reserve(expectedFrames); 

    camera->StartCamera();
    // Send data stream including our new mData trigger track
    mti->SendDataStream(xData, yData, mData, captureSamples, 0, false, true);
    mti->StartDataStream(1, true);

    bool firstFrameSkipped = false; // Add this flag

    while (ramBuffer.size() < expectedFrames) {
        cv::Mat frame = camera->GetFrame(); 
        if (!frame.empty()) {
            if (!firstFrameSkipped) {
                firstFrameSkipped = true; // Skip the first frame which may be partial
                continue;
            }
            ramBuffer.push_back(frame.clone()); 
        } else if (mti->GetSamplesRemaining() == 0) {
            break; 
        }
    }

    // 5. Save and Reconstruct
    _mkdir(folderName.c_str());
    if (ramBuffer.empty()) return;

    cv::Mat trajectory = cv::Mat::zeros(ramBuffer[0].size(), ramBuffer[0].type());

    for (int i = 0; i < (int)ramBuffer.size(); i++) {
        cv::imwrite(folderName + "/frame_" + std::to_string(i) + "_object.png", ramBuffer[i]);
        // bitwise_or combines all laser traces into one final image
        cv::bitwise_or(trajectory, ramBuffer[i], trajectory);
    }

    cv::imshow("Scan Result", trajectory);
    cv::waitKey(0);
    
    camera->StopCamera();
    mti->ResetDevicePosition();
}

// This function captures a series of vertical raster lines by controlling the MEMS mirrors and camera. 
void CaptureVerticalRasterFrames(std::string folderName, int numLines, float lineLength_ms) {
    unsigned int sampleRate = 10000;
    float flyback_ms = 10.0f; 
    
    unsigned int samplesPerLine = (unsigned int)(sampleRate * (lineLength_ms / 1000.0f));
    unsigned int samplesFlyback = (unsigned int)(sampleRate * (flyback_ms / 1000.0f));
    unsigned int totalSamplesPerCycle = samplesPerLine + samplesFlyback;

    unsigned int captureSamples = totalSamplesPerCycle * numLines;
    
    cv::Point2d maxAng = camera->GetILUTMaxAngles();

    for (int line = 0; line < numLines; line++) {
        for (unsigned int s = 0; s < totalSamplesPerCycle; s++) {
            unsigned int i = (line * totalSamplesPerCycle) + s;
            float xNorm, yNorm;

            // X-Axis is now the "Slow" axis (Left to Right)
            xNorm = -1.0f + (2.0f * (float)line / (numLines > 1 ? numLines - 1 : 1));

            if (s < samplesPerLine) {
                // ACTIVE SCAN: Y-Axis is now the "Fast" axis (Top to Bottom)
                yNorm = 1.0f - (2.0f * (float)s / samplesPerLine);
                
                // Keep the 10% safety margin to avoid "hooks" at top/bottom
                if (s > (samplesPerLine * 0.1) && s < (samplesPerLine * 0.9)) {
                    mData[i] = 255; 
                } else {
                    mData[i] = 0;
                }
            } else {
                // FLYBACK: Y-Axis returns to Top (1.0)
                float fProgress = (float)(s - samplesPerLine) / samplesFlyback;
                yNorm = -1.0f + (2.0f * fProgress); 
                mData[i] = 0; 
            }

            cv::Point2d laserCommand;
            camera->BilinearInterpolateILUTForAngle(
                cv::Point2d(xNorm * maxAng.x * xAmplitude + maxAng.x * xOffset, 
                            yNorm * maxAng.y * yAmplitude + maxAng.y * yOffset), 
                laserCommand
            );
            xData[i] = (float)laserCommand.x;
            yData[i] = (float)laserCommand.y;
        }
    }

    // --- Hardware and Capture Logic (Same as before) ---
    camera->StopCamera();
    camera->SetHardwareTrigger(true);
    mti->StopDataStream();
    mti->SetDeviceParam(MTIParam::SampleRate, sampleRate);
    mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);

    std::vector<cv::Mat> ramBuffer;
    camera->StartCamera();
    mti->SendDataStream(xData, yData, mData, captureSamples, 0, false, true);
    mti->StartDataStream(1, true);

    std::cout << "Capturing " << numLines << " Vertical lines..." << std::endl;

    int timeoutCounter = 0;
    while (ramBuffer.size() < (unsigned int)numLines && timeoutCounter < 500) {
        cv::Mat frame = camera->GetFrame();
        if (!frame.empty()) {
            ramBuffer.push_back(frame.clone());
            std::cout << "Captured vertical line " << ramBuffer.size() << std::endl;
        } else {
            Sleep(10);
            timeoutCounter++;
        }
    }

    if (ramBuffer.empty()) return;
    _mkdir(folderName.c_str());
    cv::Mat finalRaster = cv::Mat::zeros(ramBuffer[0].size(), ramBuffer[0].type());
    for (int i = 0; i < (int)ramBuffer.size(); i++) {
        cv::bitwise_or(finalRaster, ramBuffer[i], finalRaster);
    }
    cv::imshow("Vertical Raster Result", finalRaster);
    cv::waitKey(0);
    camera->StopCamera();
}

int main(int argc, char* argv[]) {
	// Open camera
	camera = new MTIPylonCamera();
	if (int error = camera->InitializeCamera()) {
		printf("Couldn't initialize camera. Exiting...\n");
		_getch();
		return -1;
	}
	bool isColorCamera = camera->IsColorCamera();

	// load device related and camera related ini files or exit if not found
	if (!camera->LoadCameraSettings(cameraSettingsFile))
	{
		printf("\n\nUnable to load settings file %s with important camera parameters.  Press any key to Exit.\n", cameraSettingsFile);
		camera->ShutdownCamera();
		_getch();
		return -1;                                       // leave demo if not successfully connected
	}
	if (!camera->LoadDeviceSettings(deviceSettingsFile))
	{
		printf("\n\nUnable to load settings file %s with important device parameters.  Press any key to Exit.\n", deviceSettingsFile);
		camera->ShutdownCamera();
		_getch();
		return -1;                                       // leave demo if not successfully connected
	}

	// Initialize the MTI Device
	mti = new MTIDevice();

	mti->ConnectDevice();
	MTIError lastError = mti->GetLastError();
	if (lastError != MTIError::MTI_SUCCESS)
	{
		printf("\n\nUnable to connect with any device.  Press any key to Exit.\n");
		mti->SendSerialReset();
		camera->ShutdownCamera();
		_getch();
		return -1;                                       // leave demo if not successfully connected
	}
	MTIDeviceParams params;
	mti->GetDeviceParams(&params);       // get current info and parameters from controller
	if (!strstr(params.DeviceName, "MTI-MZ-"))
	{
		printf("\n\nConnected to incompatible Mirrorcle MEMS Controller (Not MTI-MZ-...).  Press any key to Exit.\n");
		mti->SendSerialReset();
		_getch();
		return -1;                                       // leave demo if not successfully connected
	}

	// Force these settings so that saved LUT is not messed up when ini changes
	mti->SetDeviceParam(MTIParam::HardwareFilterBw, camera->GetDeviceHardwareFilterBw());	// over-ride ini file for EaZy2.0 with faster mirror
	mti->SetDeviceParam(MTIParam::VdifferenceMax, camera->GetDeviceVdifferenceMax());
	mti->SetDeviceParam(MTIParam::Vbias, camera->GetDeviceVbias());
	mti->SetDeviceParam(MTIParam::DataScale, 1.0f);
	mti->SetDeviceParam(MTIParam::DataRotation, 0.f);
	mti->SetDeviceParam(MTIParam::OutputOffsets, 0.f, 0.f);
	mti->SetDeviceParam(MTIParam::SampleRate, spsDefault);
	mti->GetDeviceParams(&params);       // get current info and parameters from controller if debugging

	mti->ResetDevicePosition();                 // send analog outputs (and device) back to origin in 25ms
	mti->SetDeviceParam(MTIParam::MEMSDriverEnable, true);        // turn the MEMS Driver on for all examples below

	// MTIDataGenerator object for text, linear raster, interpolation, etc.
	datagen = new MTIDataGenerator();
	
	// Set the default exposure time when using the software trigger
	// Note that the exposure time during laser scan is set by the MTI controller
	// to match the length of the laser scan, and the software exposure time is ignored
	camera->SetExposureTime(exposure_time);

	bool runFlag = true;
	while (runFlag) {
		system(CLEARSCREEN);
		printf("\n************ MTICamera-3DScan 11.1 - 3D Scanning ***************\n");

		printf("\nLoaded mticamera.ini file successfully with following key values:");
		printf("\nCamera field of view (FoV): %3.2f[deg] x %3.2f[deg]", camera->GetCameraHFOV(), camera->GetCameraVFOV());
		printf("\nScan Module calibrated field of regard (CFoR): %3.2f[deg] x %3.2f[deg]\n\n", 2 * camera->GetILUTMaxAngles().x, 2 * camera->GetILUTMaxAngles().y);

		printf("\t1: Start Detect Line Demo\n");
		printf("\t2: Setup Triangulation Angles and Parameters\n");
		printf("\t3: View Scan Area Boundary\n");
        printf("\t4: Triangular Trajectory\n");
        printf("\t5: Before Sinusoidal Trajectory\n");
        printf("\t6: After sinsusoidal\n");
        printf("\t7: Manual Raster\n");
        printf("\t8: Capture Sinusoidal Frames (Object)\n");
        printf("\t9: Capture Sinusoidal Frames (Raw)\n");
		printf("\n\tV/v: Toggle averaging of points along X: %s", xAveraging ? "true" : "false");
		printf("\n\tE(X/x)it\n");
		printf("\n");
		
		// Get keypress for menu selection
		switch (_getch()) {
			/* Demos */
			case '1':	ScanAndDetectLineDemo(); break;
			case '2':	SetupTriangulation(); break;
			case '3':	ViewScanAreaBoundary(); break;
            case '4':	CaptureTriangularTrajectoryPNG("triangular_50.png"); break;
            case '5':	CaptureSinusoidalTrajectoryPNG("sinusoidal_50.png"); break;
            case '6':   CaptureSinusoidalTrajectoryPNG("after_sinusoidal.png"); break;
            case '8':   CaptureSinusoidalFrames_changed("sinusoidal_frames_object"); break;
            case '9':   CaptureSinusoidalFrames("sinusoidal_frames_raw"); break;
			case 'V':
			case 'v':
				xAveraging = !xAveraging;
				break;
			case 'x':
			case 'X':
				runFlag = false;
				break;
			default:	break;
		}
	}

	// Shut down the camera and any open windows
	camera->ShutdownCamera();
	cv::destroyAllWindows();

	// End the program by returning the device to origin
	mti->ResetDevicePosition(); // send analog outputs (and device) back to origin in 25ms
	mti->SetDeviceParam(MTIParam::MEMSDriverEnable, false);       // turn off the MEMS driver
	mti->DisconnectDevice();
}