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

ostringstream CMDstream; // string packet
string CMD; //command string to send

string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
string PiADDR = "10.0.0.10";

//SETUP TCP COMMS
int PORT=12345;
SOCKET u_sock;

//Packet received from owl, after sending.
string RxPacket;
//Flag to clip the requested PWM value to the Min and Max bounds.
bool trunkateOnSend = true;

//Folder to store images in.
string const IMAGES_FOLDER = "../../Data/OurImages/";

//Sent the servo data to the OWL
void SendData()
{
    //If we are clipping the PWM values between their Min and Max bounds
    if(trunkateOnSend){
        // If Rx is smaller than Min bound
        if (Rx < RxLm) {
            Rx = RxLm; // Set Rx to min bound
        }        
        // If Rx is greater than Max bound
        if (Rx > RxRm) {
            Rx = RxRm; // Set Rx to max bound
        }        
        //Same structure as the above two if statements,
        // repeated for all servo values.
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

    //Clear the command stream.
    CMDstream.str("");
    CMDstream.clear();

    //Set the command stream to the OWL command interface.
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    //Get the string of the command
    CMD = CMDstream.str();
#ifdef _WIN32
    //Send the command packet to the OWL.
    RxPacket = OwlSendPacket(u_sock, CMD.c_str());
#else
    OwlSendPacket (clientSock, CMD.c_str());
#endif
}

//Calculate the distance of the object, based on the PWM values for the eyes.
void CalculateDistance() {    
    //Note: Right eye servo is flipped
    //Get the radian angle of the eyes,
    // from the current location to the center of the eyes.
    rightRads = (float(RxC - Rx) * M_PI) / (Deg2Pwm * 180);
    leftRads = (float(Lx - LxC) * M_PI) / (Deg2Pwm * 180);

    //Calculations based on lecture notes

    //dL = distanceLeft, p1, p2, p3 = part1, part2 and part3 respectively of distance formula.
    //distance left = (IPD COS(angle right)) / (SIN(angle left + angle right)
    float dL = (IPD * cos(rightRads)) / sin(rightRads + leftRads);

    float p1 = pow(dL, 2); //(distance left^2)
    float p2 = pow(IPD, 2) / 4; //(IPD^2/4)
    float p3 = (dL * IPD * sin(leftRads)); //(distance left * IPD * SIN(angle left)

    //distance estimate = square root((distance left^2) + (IPD^2/4) - (distance left * IPD * SIN(angle left))
    float estimatedDistance = (sqrt((p1 + p2) - p3));

    //Now correct the estemated distance based off of scatter graph data.
    //y = 0.9926x - 13.624
    calcDistance = (0.9926f * estimatedDistance) - 13.624f;
}


int main(int argc, char *argv[])
{
    char receivedStr[1024];
    int N;

    Rx = RxLm;
    Lx = LxLm;
    Ry = RyC;
    Ly = LyC;
    Neck= NeckC;

    /***********************
 * LOOP continuously for testing
 */

    u_sock = OwlCommsInit ( PORT, PiADDR);

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images

    // Different modes the program can be running in.
    enum MODE {MANUAL, TRACKING, CAPTURING, EXITING};
    // Initialise the starting mode to manual.
    MODE currentMode = MANUAL;

    // While the program has not been signald to exit (program is running)
    while (currentMode != EXITING){
        cout<< "Mode Updated: " << currentMode << endl;
        //Set all servo values to centre points
        Rx = RxC;
        Lx = LxC;
        Ry = RyC;
        Ly = LyC;
        Neck = NeckC;
        SendData();

        VideoCapture cap(source); // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }

        //While the program is in manual mode...
        while (currentMode == MANUAL){
            //Capture an image from the video stream.
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                //         break;
            }

            // Note that Left/Right are reversed now
            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1);

            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left = FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy);
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
            rectangle( Left, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
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
            case 'r':
                Rx = RxC;
                Lx = LxC;
                Ry = RyC;
                Ly = LyC;
                Neck= NeckC;
                break;

            case 'b':
                trunkateOnSend = !trunkateOnSend;
                cout << "Setting Trunkate on send: " << (trunkateOnSend ? "True" : "False") << endl;
                break;


            case 't': // lowercase 't'
                OWLtempl= Right(target);
                //imshow("templ",OWLtempl);
                waitKey(1);
                currentMode = TRACKING; // quit loop and start tracking target
                break; // left

            case 'c'://Start capturing the images
                Rx = RxDisparityToeIn;
                Lx = LxDisparityToeIn;
                SendData();
                cout << "capturing images..." << endl;
                OwlCalCapture(cap, IMAGES_FOLDER);
                Rx = RxC;
                Lx = LxC;
                break;

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
        // While the the application is tracking.
        while (currentMode == TRACKING) {
            //Read a frame from the video feed.
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                break;
            }

            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now            
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left = FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
            Right = FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

            //Mach the template for both eyes
            OwlCorrel OWL = Owl_matchTemplate(Right, Left, OWLtempl);

            Mat RightCopy;
            Right.copyTo(RightCopy);

            //Draw rectangles around the best match.
            rectangle(RightCopy, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(Left, OWL.MatchL, Point( OWL.MatchL.x + OWLtempl.cols , OWL.MatchL.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.ResultL, OWL.MatchL, Point( OWL.MatchL.x + OWLtempl.cols , OWL.MatchL.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.ResultR, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            //Draw a circle at the center point of each eye.
            circle(Left,Point(320,240),5,Scalar(0,255,0),1);
            circle(RightCopy,Point(320,240),5,Scalar(0,255,0),1);

            waitKey(1);
            int key = waitKey(10);
            //if the user has pressed a key,
            switch (key)
            {   // and the user has pressed the 'm' key...
                case 'm':// 'm' key
                    currentMode = MANUAL;//Signal to return back to manual mode.
                    break;
            }

//ToDo: comment from here down.

            // Only for left eye at the moment
            //** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo
            //======== try altering KPx & KPy to see the settling time/overshoot
            double KPx=0.13; // track rate X
            double KPy=0.13; // track rate Y

            double LxScaleV = LxRangeV/static_cast<double>(640); //PWM range /pixel range
            double Xoff= (OWL.MatchL.x - 320 + OWLtempl.cols/2)/LxScaleV ; // compare to centre of image
            double LxOld=Lx;
            Lx=static_cast<int>(LxOld + Xoff*KPx); // roughly 300 servo offset = 320 [pixel offset]

            double LyScaleV = LyRangeV/static_cast<double>(480); //PWM range /pixel range
            double Yoff= ((OWL.MatchL.y - 240 + OWLtempl.rows/2)/LyScaleV)*KPy ; // compare to centre of image
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

            //Create the string
            string distanceString = "Distance: " + to_string((int)calcDistance) + "mm";
            //Draw rectangle for text.
            rectangle( RightCopy, textBox, Scalar::all(0), -1, 8, 0);
            if (Lx < LxC && Rx > RxC) { //If eyes diverge
                distanceString = "Divergent Target lost!";
            }
            else if(Rx > RxC)//Looking to the right
            {
                Neck = Neck - 5;
            }
            else if (Lx < LxC) // looking left
            {
                Neck = Neck + 5;
            }
            putText(RightCopy, distanceString, cvPoint(165, 465), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);

            CalculateDistance();
            cout << "Rx: " << Rx << "   Lx: " << Lx << "   Deg2Pwm: " << Deg2Pwm << "   Distance: " << calcDistance << "mm" << endl;

            imshow("Owl-L", Left);
            imshow("Owl-R", RightCopy);
            imshow("Correl L", OWL.ResultL);
            imshow("CorrelR", OWL.ResultR);

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
