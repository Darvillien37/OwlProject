// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse, James Rogers Oct 2016 (c) Plymouth University
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demonstration programs does the following:
 * Implements a version of Itti & Kochs saliency model of saccadic stereo vision
 * 1. camera calibration is read from previous calibration data
 * 2. Main Loop - get stereo pair from camera stream
 * 3.   - correct for distorations using REMAP()
 * 4.   -
 *
 * First start communcations on Pi by running './OWLsocket'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 *
 */

#define PI 3.14159265

#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include "opencv2/calib3d.hpp"



#include <iostream> // for standard I/O
#include <string>   // for strings


#define PX2DEG  0.0911
#define DEG2PWM 10.98
#define IPD 67

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define KPX 0.25 // track rate X
#define KPY 0.25// track rate Y


#define xOffset  -0.3815
#define yOffset -54.6682
#define xScale   26.4842
#define yScale   91.4178

using namespace std;
using namespace cv;

// PFC/JR messy decls, but works under openCV 3.4
string ServoAbs(double DEGRx, double DEGRy, double DEGLx, double DEGLy, double DEGNeck);
string ServoRel(double DEGRx, double DEGRy, double DEGLx, double DEGLy, double DEGNeck);
string TrackCorrelTarget(OwlCorrel OWL);
int OwlCalCapture(cv::VideoCapture &cap, string Folder);


//void ServoAbs(float DEGRx,float DEGRy,float DEGLx,float DEGLy,float DEGNeck);
//void ServoRel(float DEGRx,float DEGRy,float DEGLx,float DEGLy,float DEGNeck);
Mat DoGFilter(Mat src, int k, int g);
Mat SobelFilter(Mat src, int scale, int delta);
Mat CannyFilter(Mat greySrc);
Mat ColourFilter(Mat colourSrc);

static string PiADDR = "10.0.0.10";
static int PORT = 12345;
static SOCKET u_sock = OwlCommsInit(PORT, PiADDR);
static ostringstream CMDstream; // string packet
static string CMD;


//Default feature map weights
static int ColourWeight = 60; //Saturation and Brightness
static int DoGHighWeight = 10; //Groups of edges in a small area
static int DoGLowWeight = 30; //DoG edge detection
static int SobelWeight = 30; //Sobel edge detection
static int CannyWeight = 30; //Canny edge detection.
static int CannyStrength = 20; //Strength of canny detection, defaults to 20.
static int FamiliarWeight = 5; //Familiarity of the target, how much has the owl focused on this before
static int foveaWeight = 50; //Distance from fovea (center)

double lxDifference = 0;
double lyDifference = 0;

int loopCounter = 0;

//Running the main loop of the application.
bool inLoop = true;

int main(int argc, char *argv[])
{
    //==========================================Initialize Variables=================================
    //Frame Size
    Size imageSize = Size(IMAGE_WIDTH, IMAGE_HEIGHT);
    
    //Local Feature Map  - implements FOVEA as a bias to the saliency map to central targets, rather than peripheral targets
    // eg. for a primate vision system
    Mat fovea = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U, double(0));
    //Make a circle in the center of the fovia mat.
    circle(fovea, Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2), 150, 255, -1);
    cv::blur(fovea, fovea, Size(301,301));
    fovea.convertTo(fovea, CV_32FC1);
    fovea *= foveaWeight;
    
    //Initilize Mats
    const Mat OWLresult;// correlation result passed back from matchtemplate
    Mat Frame;
    Mat Left, Right, OWLtempl; // images
    Mat PanView = Mat(1600, 2500, CV_8UC3, Scalar(0, 0, 0)); //init as black - ie. no data
    Mat familiar = Mat(1600, 2500, CV_8U, double(255));
    
    //Video stream source
    string source = "http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
    
    //Set center neck positions
    Rx = RxC;
    Lx = LxC;
    Ry = RyC;
    Ly = LyC;
    Neck = NeckC;
    
    //Variable declorations
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point minLocTarget;
    Point maxLocTarget;
    
    
    //===================================Read left calibration data===================================
    const string LeftCalibrationFile = argc > 1 ? argv[1] : "../../Data/LeftCalibration.xml";
    FileStorage LeftFile(LeftCalibrationFile, FileStorage::READ); // Read the settings
    if (!LeftFile.isOpened())
    {
        cout << "Cannot open LeftCalibration.xml: \"" << LeftCalibrationFile << "\"" << endl;
        return -1;
    }
    Mat cameraMatrixL, distCoeffsL;
    LeftFile["Camera_Matrix"] >> cameraMatrixL;
    LeftFile["Distortion_Coefficients"] >> distCoeffsL;
    LeftFile.release();
    
    Mat map1L, map2L;
    initUndistortRectifyMap(cameraMatrixL, distCoeffsL, Mat(),
                            cv::getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1, imageSize, nullptr),
                            imageSize, CV_16SC2, map1L, map2L);
    
    //===================================Read Right calibration data===================================
    const string RightCalibrationFile = argc > 1 ? argv[1] : "../../Data/RightCalibration.xml";
    FileStorage RightFile(RightCalibrationFile, FileStorage::READ); // Read the settings
    if (!RightFile.isOpened())
    {
        cout << "Cannot open RightCalibration.xml: \"" << RightCalibrationFile << "\"" << endl;
        return -1;
    }
    Mat cameraMatrixR, distCoeffsR;
    RightFile["Camera_Matrix"] >> cameraMatrixR;
    RightFile["Distortion_Coefficients"] >> distCoeffsR;
    RightFile.release();
    
    Mat map1R, map2R;
    initUndistortRectifyMap(cameraMatrixR, distCoeffsR, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1, imageSize, nullptr),
                            imageSize, CV_16SC2, map1R, map2R);
    
    //========================================Initialize Servos========================================
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket = OwlSendPacket(u_sock, CMD.c_str());
    
    //========================================Open Video Steam========================================
    VideoCapture cap(source);
    if (!cap.isOpened()){
        cout << "Could not open the input video: " << source << endl;
        return -1;
    }
    
    while(inLoop)
    {//Main processing loop        
        //==========================================Capture Frame============================================
        if (!cap.read(Frame)) {
            cout << "Could not open the input video: " << source << endl;
        }

        
        Mat FrameFlpd;
        cv::flip(Frame, FrameFlpd, 1);     // Note that Left/Right are reversed now
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left = FrameFlpd(Rect(0, 0, 640, 480));         // Using a rectangle
        remap(Left, Left, map1L, map2L, INTER_LINEAR);  // Apply camera calibration
        Right = FrameFlpd(Rect(640, 0, 640, 480));      // Using a rectangle
        remap(Right, Right, map1R, map2R, INTER_LINEAR);// Apply camera calibration
        Mat LeftGrey;                                   // Make a grey copy of Left
        cvtColor(Left, LeftGrey, COLOR_BGR2GRAY);
        
        
        // ======================================CALCULATE FEATURE MAPS ====================================
        // ===============================DoG low bandpass Map===============================
        //Run the DoG filter against the left greyscaled, and put it in 'DoGLow'
		Mat DoGLow = DoGFilter(LeftGrey, 3, 71); //71 = low pass
        Mat DoGLow8;
		//Normalise values between 0 and 255, and convert to an 8bit channel map
        normalize(DoGLow, DoGLow8, 0, 255, CV_MINMAX, CV_8U);
		//Show the filter
        imshow("DoG Low", DoGLow8);
		//Similar functionality for each filter....

        // ===============================DoG high bandpass Map===============================
        Mat DoGHigh = DoGFilter(LeftGrey, 3, 21);//21 = high pass
        Mat DoGHigh8;
        normalize(DoGHigh, DoGHigh8, 0, 255, CV_MINMAX, CV_8U);
        imshow("DoG High", DoGHigh8);
        
        // ===============================Sobel Map===============================
        Mat SobelMap = SobelFilter(LeftGrey, 1, 0);
        Mat SobelMap8;
        normalize(SobelMap, SobelMap8, 0, 255, CV_MINMAX, CV_8U);
        imshow("SobelMap8", SobelMap8);
        
        // ===============================Canny Map===============================
        Mat CannyMap = CannyFilter(LeftGrey);
        Mat CannyMap8;
        normalize(CannyMap, CannyMap8, 0, 255, CV_MINMAX, CV_8U);
        imshow("CannyMap8", CannyMap8);
        
        // ===============================Colour Map===============================
        // Run the colour filter against the left coloured
		Mat ColourMap = ColourFilter(Left);
        Mat ColourMap8;
        normalize(ColourMap, ColourMap8, 0, 255, CV_MINMAX, CV_8U);
        imshow("ColourMap", ColourMap8);
        
        //=====================================Initialise Global Position====================================        
        Point GlobalPos;    // Position of camera view within the range of movement of the OWL
        GlobalPos.x = static_cast<int>(900 + ((-(Neck - NeckC) + (Lx - LxC)) / DEG2PWM) / PX2DEG);
        GlobalPos.y = static_cast<int>(500 + ((Ly - LyC) / DEG2PWM) / PX2DEG);

        //Bound checking for global position
        if(GlobalPos.x < 0)
        {
            GlobalPos .x = 0;
        }
        if(GlobalPos.y < 0)
        {
            GlobalPos.y = 0;
        }

        if (GlobalPos.x > familiar.size().width - Left.size().width - 1)
        {
            GlobalPos.x = familiar.size().width - Left.size().width - 1;
        }

        if (GlobalPos.y > familiar.size().height - Left.size().height - 1)
        {
            GlobalPos.y = familiar.size().height - Left.size().height - 1;
        }

        Mat familiarLocal = familiar(Rect(GlobalPos.x, GlobalPos.y, Left.cols, Left.rows));

        cout << "X: "<< GlobalPos.x << "\tY: " << GlobalPos.y << endl;

        
        //====================================Combine maps into saliency map=====================================
        
        //Conversions for DoG Low filter
        DoGLow.convertTo(DoGLow, CV_32FC1);
        DoGLow *= DoGLowWeight;

        //Conversions for DoG high filter
        DoGHigh.convertTo(DoGHigh, CV_32FC1);
        DoGHigh *= DoGHighWeight;

        //Conversions for SobelMap
        SobelMap.convertTo(SobelMap, CV_32FC1);
        SobelMap *= SobelWeight;

        //Conversions for CannyMap
        CannyMap.convertTo(CannyMap, CV_32FC1);
        CannyMap *= CannyWeight;

        //Conversions for ColourMap
        ColourMap.convertTo(ColourMap, CV_32FC1);
        ColourMap *= ColourWeight;

        familiarLocal.convertTo(familiarLocal, CV_32FC1);
        
        // Linear combination of feature maps to create a salience map
        Mat Salience = cv::Mat(Left.size(), CV_32FC1, 0.0); // init map

		//Add the maps to the saliency map
        add(Salience, DoGLow, Salience);
        add(Salience, DoGHigh, Salience);
        add(Salience, fovea, Salience);
        add(Salience, SobelMap, Salience);
        add(Salience, CannyMap, Salience);
        add(Salience, ColourMap, Salience);

		//Multiply by the familiarity, making unexplored areas more prominent.
		//Stored as black and white image, where the darker the spot, 
		// the more familiar that location is to the owl,
		//A white value is 255, black is 0, therefore if an area is completely explored,
		// (i.e. black) then the resultant saliency for that location will be 0, 
		// (i.e. not interesting)
        Salience = Salience.mul(familiarLocal);
		//Normalize values between 0 and 255
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_32FC1); 
        
        //=====================================Find & Move to Most Salient Target=========================================        
        //Find the maximum salience value in the Salience array.
        minMaxLoc(Salience, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

        // Calculate relative servo correction and magnitude of correction
        lxDifference = static_cast<double>((maxLoc.x - (IMAGE_WIDTH / 2)) * PX2DEG);
        lyDifference = static_cast<double>((maxLoc.y - (IMAGE_HEIGHT / 2)) * PX2DEG);
  
		//Set the target round the most salient area
        target = Rect( Point(maxLoc.x - 32, maxLoc.y - 32),
                       Point(maxLoc.x + 32, maxLoc.y + 32));

#pragma region Possible solution to inefficient eye movement
		// Unable to test as ran out of time, and already recorded video,
		// however added as commented code.

		// set the template as the Most salient area
		//OWLtempl = Left(target);
		// correlate the right image using the most salient area on the left eye.
		//OwlCorrel OWL = Owl_matchTemplate(Right, OWLtempl);

		// Calculate relative servo correction and magnitude of correction
		//rxDifference = static_cast<double>((OWL.Match.x - (IMAGE_WIDTH / 2)) * PX2DEG);
		//ryDifference = static_cast<double>((OWL.Match.y - (IMAGE_HEIGHT / 2)) * PX2DEG);

		// Move left eye based on salience, but don't move the right eye
		//ServoRel(rxDifference, ryDifference, lxDifference, lyDifference, (Lx - LxC) / 100);	

		//With this, no need to read a second frame in the loop. 
		// plus i believe we can use CalculateDistance() accurately here too
#pragma endregion

        // Move left eye based on salience, but don't move the right eye
        ServoRel(0, 0, lxDifference, lyDifference, (Lx - LxC) / 100);

        //Wait to move before capturing another frame, otherwise we may get a blurred image
        waitKey(20);
        //Capture a frame from the stream
        if (!cap.read(Frame)) {
            cout << "Could not open the input video: " << source << endl;
        }
        cv::flip(Frame, FrameFlpd, 1);     // Note that Left/Right are reversed now
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG image
        Left = FrameFlpd(Rect(0, 0, 640, 480));         // Using a rectangle
        remap(Left, Left, map1L, map2L, INTER_LINEAR);  // Apply camera calibration
        Right = FrameFlpd(Rect(640, 0, 640, 480));      // Using a rectangle
        remap(Right, Right, map1R, map2R, INTER_LINEAR);// Apply camera calibration

        //set the template as the centre point of the left eye.
        OWLtempl = Left(CENTRE_TARGET);

        //correlate the right image using the centre point of the left eye.
        OwlCorrel OWL = Owl_matchTemplate(Right, OWLtempl);

        //Now move the right eye.
        TrackCorrelTarget(OWL);

        //=====================================Update Familiarity Map=====================================
        // Familiar map to inhibit salient targets once observed (this is a global map)
        double longitude = (((Ly - LyC) / DEG2PWM) + maxLoc.y * PX2DEG);//calculate longitude as the global map is a spherical projection
        // ensure dwell time at perimeter of map is similar to that at centre.
        longitude *= 2.5; //amplify the projection correction
        if(longitude > 70) {
            longitude = 70;
        }
        
        Mat familiarNew = familiar.clone();
        circle(familiarNew, GlobalPos + maxLoc, static_cast<int>(60 / cos(longitude * PI / 180)), 0, -1);
        cv::blur(familiarNew, familiarNew, Size(151, 151)); //Blur used to save on processing
        normalize(familiarNew, familiarNew, 0, 255, CV_MINMAX, CV_8U);
        addWeighted(familiarNew,
                    (static_cast<double>(FamiliarWeight) / 100),
                    familiar,
                    (100 - static_cast<double>(FamiliarWeight)) / 100,
                    0, familiar);
        
        Mat familiarSmall;
        resize(familiar, familiarSmall, familiar.size() / 4);
        imshow("Familiar", familiarSmall);

        
        //=================================Convert Saliency into Heat Map=====================================
        
        Mat SalienceHSVnorm;
        Salience.convertTo(Salience, CV_8UC1);
        normalize(Salience, SalienceHSVnorm, 130, 255, CV_MINMAX, CV_8U);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_8U);
        Mat SalienceHSV;
        cvtColor(Left, SalienceHSV, COLOR_BGR2HSV);
        
        for(int y = 0; y < 480; y++) {
            for(int x = 0; x < 640; x++){
                SalienceHSV.at<Vec3b>(y, x) = Vec3b(255 - SalienceHSVnorm.at<uchar>(y, x), 255, 255);
            }
        }
        cvtColor(SalienceHSV, SalienceHSV, COLOR_HSV2BGR);
        
        
        //=======================================Update Global View===========================================
        if(GlobalPos != Point(0, 0)){

            Mat LeftCrop = Left(Rect(220, 140, 200, 200));//image cropped to minimize image stitching artifacts

            LeftCrop.copyTo(PanView(Rect(GlobalPos.x, GlobalPos.y, LeftCrop.cols, LeftCrop.rows)));
            Mat PanViewSmall;
            resize(PanView, PanViewSmall, PanView.size() / 2);
            imshow("PanView", PanViewSmall);
        }

		//=======================================Show windows===========================================
        //Draw rectangle on most salient area
        rectangle(Left,
                  target,
                  Scalar::all(255),
                  2, 8, 0);
		
		//resize to half their size, stops overcrowding of all the opened windows
        resize(Left, Left, Left.size() / 2);
        resize(Right, Right, Right.size() / 2);

        //Draw a circle on the centres of the video windows
        circle(Left, Point(Left.size().width/2,Left.size().height/2), 5, Scalar(0,255,0), 1);
        circle(Right, Point(Right.size().width/2,Right.size().height/2), 5, Scalar(0,255,0), 1);


        //show the eyes and the target
        imshow("Left", Left);
        imshow("target", OWLtempl);
        imshow("Right", Right);

        resize(SalienceHSV, SalienceHSV, SalienceHSV.size() / 2);
        imshow("SalienceHSV", SalienceHSV);
        
        //=======================================Control Window for feature weights=======================================        
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        cvCreateTrackbar("DoG Weight", "Control", &DoGLowWeight, 100);
        cvCreateTrackbar("DoG High", "Control", &DoGHighWeight, 100);
        cvCreateTrackbar("Canny W", "Control", &CannyWeight, 100);
        cvCreateTrackbar("Canny Str", "Control", &CannyStrength, 100);
        cvCreateTrackbar("Sobel W", "Control", &SobelWeight, 100);
        cvCreateTrackbar("Colour", "Control", &ColourWeight, 100);
        cvCreateTrackbar("Familiar", "Control", &FamiliarWeight, 100);
        cvCreateTrackbar("fovea", "Control", &foveaWeight, 100);

		//=====================Decay of familiarity=====================
        //At the end of every loop, add to the familiar map,
		// effectively creating a slow decay to the owls "memory" of interest.
        add(Scalar(1, 1, 1), familiar, familiar);

		//Could the '1' could be stored in a variable, 
		// and placed as a slider on the control window?

		//=====================Wait for user input=====================
		//Wait 20ms for user input, 
		// also gives enough time for cameras to capture a stable frame
        int key = waitKey(20);
        switch (key)
        {
        case 'r'://Reset/clear familiarity map
            familiar.setTo(double(255));
            break;
		case 27: //[ESC] key, Exit the application
		case 'q':
			cout << "Exiting Application" << endl;
			inLoop = false;
			break;

        }
    }//End of inLoop
	
	 //Disconnect from Owl
#ifdef _WIN32
	cout << "Closing Socket" << endl;
	closesocket(u_sock);
#else
	close(clientSock);
#endif
	return 0;
}//End of main

//====================================================================================//
// SERVO FUNCTIONS

//Summary:
//      The absolute values, in degrees, the servos should move to. Also does bound checking.
string ServoAbs(double DEGRx, double DEGRy, double DEGLx, double DEGLy, double DEGNeck){    
    //Convert the params to PWM
    Rx = static_cast<int>(DEGRx * DEG2PWM);
    Ry = static_cast<int>(DEGRy * DEG2PWM);
    Lx = static_cast<int>(DEGLx * DEG2PWM);
    Ly = static_cast<int>(DEGLy * DEG2PWM);
    Neck = static_cast<int>(DEGNeck * DEG2PWM);

    //Do bound checking for eye and neck PWM values
    if(Rx > RxRm) {
        Rx = RxRm;
    } else if(Rx < RxLm) {
        Rx = RxLm;
    }
    
    if(Lx > LxRm) {
        Lx = LxRm;
    } else if(Lx < LxLm) {
        Lx = LxLm;
    }
    
    if(Ry > RyTm) {
        Ry = RyTm;
    } else if(Ry < RyBm) {
        Ry = RyBm;
    }
    
    if(Ly > LyBm) {
        Ly = LyBm;
    } else if(Ly < LyTm) {
        Ly = LyTm;
    }
    
    if(Neck > NeckL) {
        Neck = NeckL;
    } else if(Neck < NeckR) {
        Neck = NeckR;
    }
    
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR = OwlSendPacket (u_sock, CMD.c_str());
    return (retSTR);
}

//Summary:
//      The servos move by the difference, in degrees, passed in. Also does bound checking.
//      i.e. If servo is at 5 degrees, and -3 is passed in, the result will be 2.
string ServoRel(double DEGRx, double DEGRy, double DEGLx, double DEGLy, double DEGNeck){    
    //Convert the params to PWM, and add the difference
    Rx = static_cast<int>(DEGRx * DEG2PWM) + Rx;
    Ry = static_cast<int>(-DEGRy * DEG2PWM) + Ry;
    Lx = static_cast<int>(DEGLx * DEG2PWM) + Lx;
    Ly = static_cast<int>(DEGLy * DEG2PWM) + Ly;
    Neck = static_cast<int>(-DEGNeck * DEG2PWM) + Neck;
    
    //Do bound checking for eye and neck PWM values
    if(Rx > RxRm) {
        Rx = RxRm;
    } else if(Rx<RxLm) {
        Rx = RxLm;
    }
    
    if(Lx > LxRm) {
        Lx = LxRm;
    } else if(Lx < LxLm) {
        Lx = LxLm;
    }
    
    if(Ry > RyTm) {
        Ry = RyTm;
    } else if(Ry < RyBm) {
        Ry = RyBm;
    }
    
    if(Ly > LyBm) {
        Ly = LyBm;
    } else if(Ly < LyTm) {
        Ly = LyTm;
    }
    
    if(Neck > NeckL) {
        Neck = NeckL;
    } else if(Neck < NeckR) {
        Neck = NeckR;
    }
    
    //Send the data
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR = OwlSendPacket (u_sock, CMD.c_str());
    return (retSTR);
}

//Calculates PWM position of right eye, based off OwlCorrel.
string TrackCorrelTarget (OwlCorrel OWL){
    ostringstream CMDstream;
    string CMD, RxPacket;

    //================= calculate PWM position of right eye =================
    double RxScaleV = RxRangeV / static_cast<double>(IMAGE_WIDTH); //PWM range /pixel range
    double RxOff = (OWL.Match.x - (IMAGE_WIDTH / 2)  + OWLtempl.cols / 2) / RxScaleV ; // compare to centre of image
    double RxOld = Rx;
    Rx = static_cast<int>(RxOld + RxOff * KPX); // roughly 300 servo offset = 320 [pixel offset]

    double RyScaleV = RyRangeV / static_cast<double>(IMAGE_HEIGHT); //PWM range /pixel range
    double RyOff = ((OWL.Match.y - (IMAGE_HEIGHT / 2) + OWLtempl.rows / 2) / RyScaleV) * KPY ; // compare to centre of image
    double RyOld = Ry;
    Ry = static_cast<int>(RyOld - RyOff); // roughly 300 servo offset = 320 [pixel offset]

    string retSTR = "";
    const int MAX_CYCLES = 10;
    //Every x amount of cycles round, we reset the right eye to be parelel.
    if(loopCounter >= MAX_CYCLES)
    {
        loopCounter = 0;
        cout << "Resetting right eye parelel." << endl;
        //move right eye to be parallel with left eye
        retSTR = ServoRel(
                    ((Lx - LxC + RxC - Rx) / DEG2PWM) + lxDifference * 1,
                    -((LyC - Ly + RyC - Ry) / DEG2PWM) + lyDifference * 1,
                    0,
                    0,
                    0);


    }
    else
    {
        //** ACTION
        // move to get minimise distance from centre of both images, ie verge in to target
        // move servos to position
        retSTR = ServoAbs(
                    (Rx / DEG2PWM), //Rx
                    (Ry / DEG2PWM), // Ry -- the right eye Y servo has inverted direction compared to left.
                    (Lx / DEG2PWM),// Lx
                    (Ly / DEG2PWM), // Ly
                    Neck / DEG2PWM
                    );
    }
    loopCounter ++;
    return (retSTR);
}

// Create DoG bandpass filter, with g being odd always and
// a lower numbers are a higher pass, and visa versa
// k is normally 3 or 5
Mat DoGFilter(Mat src, int k, int g){
    Mat srcC; //Variable to hold the copied image 
    src.convertTo(srcC, CV_32FC1);//convert the image to 32bit floating point
    Mat g1, g2;
    GaussianBlur(srcC, g1, Size(g, g), 0);//Blur the images
    GaussianBlur(srcC, g2, Size(g * k, g * k), 0);
    srcC = (g1 - g2) * 2;//Difference calculation
    return srcC;
}

Mat SobelFilter(Mat greySrc, int scale, int delta)
{
    //http://docs.opencv.org/3.4.3/d2/d2c/tutorial_sobel_derivatives.html
    Mat result;
    int ddepth = CV_16S;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    //Next calculate derivatives, in x and y direction.
    Sobel(greySrc, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    Sobel(greySrc, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);

    //Convert the partial results back to grayscale.
    convertScaleAbs( grad_x, abs_grad_x );
    convertScaleAbs( grad_y, abs_grad_y );

    //Finaly approximate the 'Gradient' by adding both direcitonal gradients.
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, result);

    return result;
}

Mat CannyFilter(Mat greySrc) {
    Mat result;

    const int ratio = 3;
    const int kernelSize = 3;

    blur(greySrc, result, Size(3,3));
    Canny(result, result, CannyStrength, (CannyStrength * ratio), kernelSize);

    return result;
}

Mat ColourFilter(Mat colourSrc) {

    Mat result;

    //Threshold for the greyscale luminocity.
    int threshold = 130; //130 default.

    //Convert the mat to HSV
    cvtColor(colourSrc, result, CV_BGR2HSV);

    //Loop through the Mat Going through the y pixels
    for( int y = 0; y < colourSrc.rows; y++ ) {
        //Then the x pixels, so we have a pixel at y,x
        for( int x = 0; x < colourSrc.cols; x++ ) {
            //Then once at the desired pixel, we combine 2 of the channels (SV) into 1 to get a better salience map.
            for( int c = 0; c < colourSrc.channels(); c++ ) {
                //HSV Channels are: H[0]S[1]V[2]
                //If in 3rd channel (V),
                if (c == 2) {
                    //Then, we times the S and V channels together to combine them.
                    result.at<Vec3b>(y,x)[c] *= saturate_cast<uchar>(result.at<Vec3b>(y,x)[c - 1]);
                }
            }
        }
    }

    //We then convert the colour map to greyscale for the saliency map.
    cvtColor(result, result, COLOR_BGR2GRAY);

    //Loop through the image once again.
    for( int y = 0; y < result.rows; y++ ) {
        //Then the x pixels, so we have a pixel at y,x
        for( int x = 0; x < result.cols; x++ ) {
            //Then if the luminocity is blow threshold, just set the pixel to 0
            //Essentially ignoring it.
            if (result.at<uchar>(y, x) <= threshold) {
                result.at<uchar>(y, x) = 0;
            }
        }
    }

    //We can then blur the image to reduce noise, not sure if this is a good idea?
    //GaussianBlur(result, result, Size(3, 3), 0);

    return result;
}
