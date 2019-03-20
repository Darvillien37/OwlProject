#include <iostream>
#include <fstream>

#include "owl-comms.h"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

//Moved to global variables.
ostringstream CMDstream; // string packet
string CMD;

//Location of the source video.
string source = "http://10.0.0.10:8080/stream/video.mjpeg";
string PiADDR = "10.0.0.10";

//SETUP TCP COMMS
int PORT=12345;
SOCKET u_sock;

//Made global to use within all functions.
Size img_size = {640,480};

//Size of the target (in pixels)
const uint DEFAULT_TARGET_SIZE = 16;
//Initialise the targetSize to the default, can be changed later on.
uint targetSize = DEFAULT_TARGET_SIZE;

//Rectangles must be global.
Rect target = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);
Rect displayTarget = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);

//Boolean for live targeting
bool liveTargeting = false;

//Boolean for storing and displaying the distance of the target.
string distanceString = "";

//Variables for the cyclic buffer array used for averaging.
const int CYCLIC_BUFFER_SIZE = 10;

//First cyclic buffer to find average of sums.
int cyclicBuffer[CYCLIC_BUFFER_SIZE];
uint cyclicBufferIndex = 0;
int cyclicBufferSum = 0;
int cyclicBufferAverage = 0;

//Second cyclic buffer to average the averages.
int cyclicBuffer2[CYCLIC_BUFFER_SIZE];
uint cyclicBuffer2Index = 0;
int cyclicBuffer2Sum = 0;
int cyclicBuffer2Average = 0;

bool averaging = true;

bool targetSizeChanged = false;

void ConnectAndSend() {
    u_sock = OwlCommsInit ( PORT, PiADDR);

    const int Rx=1520 - 20;
    const int Ry=1450;
    const int Lx=1540 + 20;
    const int Ly=1550;
    const int Neck = 1540;

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    cout <<"Sending Data - "<< Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck << endl;
    CMD = CMDstream.str();
#ifdef _WIN32
    OwlSendPacket (u_sock, CMD.c_str());
#else
    OwlSendPacket (clientSock, CMD.c_str());
#endif
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if ( event == EVENT_LBUTTONDOWN) {
        if (liveTargeting) {
            //Check to see if target is out of bounds, if it is, reset it.
            //This prevents crashing for when the target bounds exceed the window.
            if (x < (targetSize / 2) || x > img_size.width - (targetSize / 2) || y < (targetSize / 2) || y > img_size.height - (targetSize / 2)) {
                //If we are out of bounds, reset the targets to the centre of the screen.
                displayTarget = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);
                target = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);
            } else {
                //If we are not out of bounds, set the targets to the position of the mouse click.
                displayTarget = Rect(x-(targetSize /2), y-(targetSize /2), targetSize, targetSize);
                target = Rect(img_size.width - x-(targetSize /2), img_size.height - y-(targetSize /2), targetSize, targetSize);
            }
        }
        //cout << "Mouse clicked at: " << x << ", " << y << endl;
        //Check for right click
    } else if (event == EVENT_RBUTTONDOWN) {
        //Flip the live targeting boolean.
        liveTargeting = !liveTargeting;
        //If live targeting is turned off, reset the target position.
        if (!liveTargeting) {
            displayTarget = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);
            target = Rect((img_size.width / 2) - (targetSize /2), (img_size.height / 2) - (targetSize /2), targetSize, targetSize);
            //Close the window that displays the live target.
            destroyWindow("liveTarget");
        }
    }
}

int main(int argc, char** argv)
{
    ConnectAndSend();

    std::string intrinsic_filename = "../../Data/intrinsics.xml";
    std::string extrinsic_filename = "../../Data/extrinsics.xml";

    enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
    const int ALGORITHM = STEREO_SGBM; //PFC always do SGBM - colour
    //N-disparities and block size referenced in the lecture.
    const int NO_OF_DISP = 256  ; //256 is default.
    const int SAD_BLOCK_SIZE = 3; //3 is default.
    const bool IS_DISPLAY = false;
    const float SCALE_FACTOR = 1.0;
    const int COLOUR_MODE = ALGORITHM == STEREO_BM ? 0 : -1;

    int colourSum = 0;

    Ptr<StereoBM> bm = StereoBM::create(16, 9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    if ( NO_OF_DISP < 1 || NO_OF_DISP % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        return -1;
    }
    if (SCALE_FACTOR < 0)
    {
        printf("Error: The scale factor variable must be a positive floating-point number\n");
        return -1;
    }
    if (SAD_BLOCK_SIZE < 1 || SAD_BLOCK_SIZE % 2 != 1)
    {
        printf("Error: The block size variable must be a positive odd number\n");
        return -1;
    }

    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }


    Rect roi1, roi2;
    Mat Q;

    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened()){
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= SCALE_FACTOR;
        M2 *= SCALE_FACTOR;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }
        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);


        //VIDEO LOOP PFC March 2017 starts here
        string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name

        VideoCapture cap (source);              // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        //Rect region_of_interest = Rect(x, y, w, h);
        bool inLOOP = true;
        cv::Mat Frame, Left, Right;
        cv::Mat disp, disp8;

        //Creating a material for the target.
        Mat targetArray;
        //A scalar for the grey value of the current target pixel to be stored in.
        Scalar targetColour;

        while (inLOOP){
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                break;
            }
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Right= Frame(Rect(0, 0, img_size.width, img_size.height));
            Left= Frame(Rect(img_size.width, 0, img_size.width, img_size.height));

            //Put into an if statement to reduce performance hit by only updating rectangles
            //Only if the size has been changed.
            if (targetSizeChanged) {
                //Update rectangles.
                target = Rect(target.x, target.y, targetSize, targetSize);
                displayTarget = Rect(displayTarget.x, displayTarget.y, targetSize, targetSize);

                targetSizeChanged = false;
            }

            Mat Leftr, Rightr;
            remap(Left, Leftr, map11, map12, INTER_LINEAR);
            remap(Right, Rightr, map21, map22, INTER_LINEAR);

            Left = Leftr;
            Right = Rightr;

            bm->setROI1(roi1);
            bm->setROI2(roi2);
            bm->setPreFilterCap(31);
            bm->setBlockSize(SAD_BLOCK_SIZE);
            bm->setMinDisparity(0);
            bm->setNumDisparities(NO_OF_DISP);
            bm->setTextureThreshold(10);
            bm->setUniquenessRatio(15);
            bm->setSpeckleWindowSize(100);
            bm->setSpeckleRange(32);
            bm->setDisp12MaxDiff(1);

            sgbm->setPreFilterCap(63);
            int sgbmWinSize = SAD_BLOCK_SIZE > 0 ? SAD_BLOCK_SIZE : 3;
            sgbm->setBlockSize(sgbmWinSize);

            int cn = Left.channels();

            sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
            sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
            sgbm->setMinDisparity(0);
            sgbm->setNumDisparities(NO_OF_DISP);
            sgbm->setUniquenessRatio(10);
            sgbm->setSpeckleWindowSize(100);
            sgbm->setSpeckleRange(32);
            sgbm->setDisp12MaxDiff(1);
            if(ALGORITHM == STEREO_HH)
                sgbm->setMode(StereoSGBM::MODE_HH);
            else if(ALGORITHM == STEREO_SGBM)
                sgbm->setMode(StereoSGBM::MODE_SGBM);
            else if(ALGORITHM == STEREO_3WAY)
                sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);


            // int64 t = getTickCount();
            if( ALGORITHM == STEREO_BM )
                bm->compute(Left, Right, disp);
            else if( ALGORITHM == STEREO_SGBM || ALGORITHM == STEREO_HH || ALGORITHM == STEREO_3WAY )
                sgbm->compute(Left, Right, disp);
            // t = getTickCount() - t;
            //calculates the time elapsed for calculation
            // printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());


            if( ALGORITHM != STEREO_VAR ) {
                disp.convertTo(disp8, CV_8U, 255 / (NO_OF_DISP * 16.0));
            } else {
                disp.convertTo(disp8, CV_8U);
            }

            //----------Displaying windows stuff--------------

            //Flipping all of the frames on the x axis before displaying.
            flip(Left, Left, -1);
            flip(Right, Right, -1);
            imshow("left", Left);
            imshow("right", Right);

            //Check for mouse clicks on disparity window before any calculations.
            setMouseCallback("disparity", CallBackFunc, NULL);

            //Calculations for distance begin here.
            //Grab target here.
            disp(target).copyTo(targetArray);

            //Reset colour sum before use.
            colourSum = 0;

            //Loop through every pixel on target.
            for (int i = 0; i < targetSize; i++) {
                for (int j = 0; j < targetSize; j++) {
                    //Get the pixel at i, j
                    targetColour = targetArray.at<ushort>(Point(i, j));
                    //Add the value of said pixel to colour sum.
                    colourSum += targetColour[0];

                }
            }

            colourSum = colourSum / pow(targetSize, 2);

            //Finds the average of the last 10 colourSums for a more consistent result.
            if (averaging) {
                //Set cyclic buffer[cyclicBufferIndex] to colourSum
                cyclicBuffer[cyclicBufferIndex] = colourSum;
                //Increment the index in a cyclical manner
                cyclicBufferIndex = (cyclicBufferIndex + 1) % CYCLIC_BUFFER_SIZE;

                //Only calculate the average when the buffer has filled with fresh values,
                // which is when the index loops back to '0'
                if(cyclicBufferIndex == 0){
                    cout<< "---------------------------Average Buffer Ready" << endl;

                    //----Calculate the average of the buffer:
                    //Loop through the buffer array to get the average.
                    for (int i = 0; i < CYCLIC_BUFFER_SIZE; i++) {
                        cyclicBufferSum += cyclicBuffer[i];
                    }
                    //Once all of the elements have been summed, divide by the buffer size (10)
                    cyclicBufferAverage = cyclicBufferSum / CYCLIC_BUFFER_SIZE;

                    //Then put that average in the average-average buffer
                    cyclicBuffer2[cyclicBuffer2Index] = cyclicBufferAverage;
                    //Increment the index in a cyclical manner
                    cyclicBuffer2Index = (cyclicBuffer2Index + 1) % CYCLIC_BUFFER_SIZE;

                    if(cyclicBuffer2Index == 0){
                        cout<< "---------------------------Average-Average Buffer Ready---------------------------" << endl;
                    }
                }//end if fresh average

                //Loop through the Average -Average buffer to get the average of the averages.
                for (int i = 0; i < CYCLIC_BUFFER_SIZE; i++) {
                    cyclicBuffer2Sum += cyclicBuffer2[i];
                }

                //Once all of the elements have been summed, divide by the buffer size (10)
                cyclicBuffer2Average = cyclicBuffer2Sum / CYCLIC_BUFFER_SIZE;

                //Reset the buffer sum for use on next loop.
                cyclicBufferSum = 0;
                cyclicBuffer2Sum = 0;

                //Change distanceStrings value to the cyclicBuffer average.
                distanceString = to_string((int)cyclicBuffer2Average) + "mm";
            } else {
                //Put the value into distanceString for printing to the window.
                distanceString = to_string((int)colourSum) + "mm";
            }

            //disp8 needs to be flipped before it can be shown.
            flip(disp8, disp8, -1);

            //Show target on disparity frame.
            rectangle(disp8, displayTarget, Scalar::all(255), 1, 8, 0);
            //Write on disparity window.
            //Writing distances
            putText(disp8, "Distance:", cvPoint(445, 50), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            putText(disp8, distanceString, cvPoint(470, 80), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            //Writing liveTargeting status
            putText(disp8, "Live Targeting:", cvPoint(390, 130), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            putText(disp8, liveTargeting ? "True" : "False", cvPoint(475, 160), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            //Write averaging status to screen.
            putText(disp8, "Averaging:", cvPoint(430, 210), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            putText(disp8, averaging ? "True" : "False", cvPoint(475, 240), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            //Write targetSize to screen
            putText(disp8, "targetSize:", cvPoint(430, 290), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            putText(disp8, to_string(targetSize), cvPoint(495, 320), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);

            //Instructions for enabling live targeting.
            string temp = "Right click to ";
            temp += liveTargeting? "disable" : "enable";
            putText(disp8, temp, cvPoint(410, 400), FONT_HERSHEY_DUPLEX, 0.6, Scalar::all(255), 0, 0, false);
            putText(disp8, "live targeting", cvPoint(455, 420), FONT_HERSHEY_DUPLEX, 0.6, Scalar::all(255), 0, 0, false);
            if (liveTargeting) {
                //Flip the targetArray for correct display orientation
                flip(targetArray, targetArray, -1);
                targetArray.copyTo(disp8(Rect(565, 175, targetArray.cols, targetArray.rows)));
            }
            imshow("disparity", disp8);

            //Exit the disparity loop on a key press of 'q'
            char key = waitKey(30);
            if (key=='q') {
                break;
            } else if (key =='a') { // 'a' hotkey to enable or disable averaging.
                averaging = !averaging;
            } else if (key ==',') {
                targetSizeChanged = true;
                targetSize--;
            } else if (key =='.') {
                targetSizeChanged = true;
                targetSize++;
            } else if (key == 'd') {
                targetSizeChanged = true;
                targetSize = DEFAULT_TARGET_SIZE;
            }

        } // end video loop
    } // end got intrinsics IF
#ifdef _WIN32
    cout << "Closing Socket" << endl;
    closesocket(u_sock);
#else
    close(clientSock);
#endif
    return 0;
}
