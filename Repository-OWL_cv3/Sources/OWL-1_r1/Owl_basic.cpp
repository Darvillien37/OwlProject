// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse Oct 2016 (c) Plymouth UNiversity
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demosntration programs does the following:
 * a) loop 1 - take picture, check arrow keys
 *             move servos +5 pwm units for each loop
 *             draw 64x64 pixel square overlaid on Right image
 *             if 'c' is pressed copy patch into a template for matching with left
 *              exit loop 1;
 * b) loop 2 - perform Normalised Cross Correlation between template and left image
 *             move Left eye to centre on best match with template
 *             (treats Right eye are dominate in this example).
 *             loop
 *             on exit by ESC key
 *                  go back to loop 1
 *
 * First start communcations on Pi by running 'python PFCpacket.py'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 * NOTE: this program is just a demonstrator, the right eye does not track, just the left.
 */

#include <iostream>
#include <fstream>


#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"


#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;

//Moved to global variables.
ostringstream CMDstream; // string packet
string CMD;

string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
string PiADDR = "10.0.0.10";

//SETUP TCP COMMS
int PORT=12345;
SOCKET u_sock;

string RxPacket;
bool trunkateOnSend = true;

void SendData() {

    if(trunkateOnSend){
        //Check if
        if (Rx < RxLm) {
            Rx = RxLm;
        }
        if (Rx > RxRm) {
            Rx = RxRm;
        }
        if (Ry < RyBm) {
            Ry = RyBm;
        }
        if (Ry > RyTm) {
            Ry = RyTm;
        }
        if (Lx < LxLm) {
            Lx = LxLm;
        }
        if (Lx > LxRm) {
            Lx = LxRm;
        }
        if (Ly > LyBm) {
            Ly = LyBm;
        }
        if (Ly < LyTm) {
            Ly = LyTm;
        }
        if (Neck > NeckL) {
            Neck = NeckL;
        }
        if (Neck < NeckR) {
            Neck = NeckR;
        }
    }

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
#ifdef _WIN32
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
#else
    OwlSendPacket (clientSock, CMD.c_str());
#endif
}

void CalculateDistance() {
    //Radians for both eyes, used for distance calculations.
    //Right eye is flipped
    rightRads = (float(RxC - Rx)*M_PI) / (Deg2Pwm * 180);
    leftRads = (float(Lx - LxC)*M_PI) / (Deg2Pwm * 180);
    //cout << rightRads * 180 / M_PI << "\t";
    //cout << leftRads * 180 / M_PI << endl;

    //dL = distanceLeft, p1, p2, p3 = part1, part2 and part3 respectively of distance formula.
    float dL = (IPD * cos(rightRads)) / sin(rightRads + leftRads);

    float p1 = pow(dL, 2);
    float p2 = pow(IPD, 2) / 4;
    float p3 = (dL * IPD * sin(leftRads));

    calcDistance = sqrt((p1 + p2) - p3);

}

int main(int argc, char *argv[])
{
    char receivedStr[1024];
    int N;

    Rx = RxLm; Lx = LxLm;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    /***********************
 * LOOP continuously for testing
 */

    u_sock = OwlCommsInit ( PORT, PiADDR);

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images
    enum MODE {MANUAL, TRACKING, EXITING};
    MODE currentMode = MANUAL;


    while (currentMode != EXITING){
        cout<< "Mode Updated: " << currentMode << endl;
        Rx = RxC;
        Lx = LxC;
        Ry = RyC;
        Ly = LyC;
        Neck = NeckC;
        SendData();

        VideoCapture cap (source);              // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        //Rect region_of_interest = Rect(x, y, w, h);
        while (currentMode == MANUAL){
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                //         break;
            }
            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy);
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
            rectangle( Left, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
            circle(Left,Point(320,240),5,Scalar(0,255,0),1);
            circle(RightCopy,Point(320,240),5,Scalar(0,255,0),1);
            //Write text to the right window.
            putText(RightCopy, "Hello", cvPoint(10,300), FONT_HERSHEY_COMPLEX_SMALL, 10, Scalar::all(255), 1, 8, false);

            imshow("Left",Left);
            imshow("Right", RightCopy);
            waitKey(1);
            int key = waitKey(30); // this is a pause long enough to allow a stable photo to be taken.
            switch (key){
            case 'w': //up
                Ly=Ly-5; // was Ly=+5 Changed BILL
                break;
            case 's'://down
                Ly=Ly+5; // was Ly=-5 BILL
                break;
            case 'a'://left
                Lx=Lx-5;
                break;
            case 'd'://right
                Lx=Lx+5;
                break;
            case 'i': //up
                Ry=Ry+5;
                break;
            case 'k'://down
                Ry=Ry-5;
                break;
            case 'j'://left
                Rx=Rx-5;
                break;
            case 'l'://right
                Rx=Rx+5;
                break;
            case ','://Neck Left '<'
                Neck = Neck + 5;
                break;
            case '.'://Neck Right '>'
                Neck = Neck - 5;
                break;

            case 't':
                trunkateOnSend = !trunkateOnSend;
                cout << "Setting Trunkate on send: " << trunkateOnSend << endl;
                break;
            case 'r':
                Rx = RxC;
                Lx = LxC;
                Ry = RyC;
                Ly = LyC;
                Neck= NeckC;

                break;
            case 'c': // lowercase 'c'
                OWLtempl= Right(target);
                //imshow("templ",OWLtempl);
                waitKey(1);
                currentMode = TRACKING; // quit loop and start tracking target
                break; // left
            case 27://Escape key
                cout << "Exiting Application";
                currentMode = EXITING;
                break;
            default:
                key=key;
                //nothing at present
            }
            SendData();

        } // END cursor control loop
        destroyAllWindows();

        //============= Normalised Cross Correlation ==========================
        // right is the template, just captured manually
        while (currentMode == TRACKING) {            
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                break;
            }


            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle

            //Rect target= Rect(320-32, 240-32, 64, 64); //defined in owl-cv.h
            //Mat OWLtempl(Right, target);
            OwlCorrel OWL;
            OWL = Owl_matchTemplate( Right,  Left, OWLtempl, target);
            /// Show me what you got
            Mat RightCopy;
            Right.copyTo(RightCopy);
            // rectangle(RightCopy, target, Scalar::all(255), 2, 8, 0 );
            rectangle(RightCopy, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(Left, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.Result, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.ResultR, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            circle(Left,Point(320,240),5,Scalar(0,255,0),1);
            circle(RightCopy,Point(320,240),5,Scalar(0,255,0),1);
            imshow("Owl-L", Left);
            imshow("Owl-R", RightCopy);
            imshow("Correl L", OWL.Result);
            imshow("CorrelR", OWL.ResultR);
            waitKey(1);
            int key = waitKey(10);
            switch (key )
            {
            case 'm':// 'm' key
                currentMode = MANUAL;
                break;
//            case '.':// '>' key
//                Deg2Pwm = Deg2Pwm + 0.05;
//                break;
//            case ',':// '<' key
//                Deg2Pwm = Deg2Pwm - 0.05;
//                break;
            }



            // Only for left eye at the moment
            //** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo
            //======== try altering KPx & KPy to see the settling time/overshoot
            double KPx=0.13; // track rate X
            double KPy=0.13; // track rate Y

            double LxScaleV = LxRangeV/static_cast<double>(640); //PWM range /pixel range
            double Xoff= (OWL.Match.x - 320 + OWLtempl.cols/2)/LxScaleV ; // compare to centre of image
            double LxOld=Lx;
            Lx=static_cast<int>(LxOld + Xoff*KPx); // roughly 300 servo offset = 320 [pixel offset]

            double LyScaleV = LyRangeV/static_cast<double>(480); //PWM range /pixel range
            double Yoff= ((OWL.Match.y - 240 + OWLtempl.rows/2)/LyScaleV)*KPy ; // compare to centre of image
            double LyOld=Ly;
            Ly=static_cast<int>(LyOld - Yoff); // roughly 300 servo offset = 320 [pixel offset]



            double RxScaleV = RxRangeV/static_cast<double>(640); //PWM range /pixel range
            double RxOff=  (OWL.MatchR.x - 320  + OWLtempl.cols/2)/RxScaleV ; // compare to centre of image
            double RxOld=Rx;
            Rx=static_cast<int>(RxOld + RxOff*KPx); // roughly 300 servo offset = 320 [pixel offset]

            double RyScaleV = RyRangeV/static_cast<double>(480); //PWM range /pixel range
            double RyOff= ((OWL.MatchR.y - 240 + OWLtempl.rows/2) / RyScaleV)*KPy ; // compare to centre of image
            double RyOld=Ry;
            Ry=static_cast<int>(RyOld - RyOff); // roughly 300 servo offset = 320 [pixel offset]

            if(Rx > RxC)//Looking to the right
            {
                Neck = Neck - 5;
            }
            else if (Lx < LxC) // looking left
            {
                Neck = Neck + 5;
            }

            CalculateDistance();
            cout << "Rx: " << Rx << "   Lx: " << Lx << "   Deg2Pwm: " << Deg2Pwm << "   Distance: " << calcDistance << "mm" << endl;

//          Fix this \/
//          cvPutText(RightCopy,'OpenCV',(10,500), FONT_HERSHEY_SIMPLEX, (255,255,255));

            // move to get minimise distance from centre of both images, ie verge in to targe
            // move servos to position
            SendData();

        } // end of tracking loop

        // close windows down
        destroyAllWindows();

    } // end while outer loop
#ifdef _WIN32
    closesocket(u_sock);
#else
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}
