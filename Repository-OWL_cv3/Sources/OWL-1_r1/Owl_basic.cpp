
+#include <iostream>
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

// Counts loops to acquire new target.
int trackingLoopCount = 0;

// Boolean for whether we should re-sample targets.
bool resample = false;


void SendData(){//Send the PWM servo data to the OWL
    //Check if we are clipping the PWM values between their Min and Max bounds
    if(trunkateOnSend){//If we are....      
        if (Rx < RxLm) { // If Rx is smaller than Min bound....
            Rx = RxLm; // Set Rx to min bound
        }               
        if (Rx > RxRm) { // If Rx is greater than Max bound....
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
    //Get the string representation of the command
    CMD = CMDstream.str();
#ifdef _WIN32
    //Send the command packet to the OWL.
    RxPacket = OwlSendPacket(u_sock, CMD.c_str());
#else
    OwlSendPacket (clientSock, CMD.c_str());
#endif
}//End of SendData()

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


    u_sock = OwlCommsInit ( PORT, PiADDR);

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images

    // Different modes the program can be running in.
    enum MODE {MANUAL, TRACKING, CAPTURING, EXITING};
    // Initialise the starting mode to manual.
    MODE currentMode = MANUAL;

    // While the program has not been signalled to exit (program is running)
    while (currentMode != EXITING){
        cout<< "Mode Updated: " << currentMode << endl; //Output which mode the program has been changed to
        //Set all servo values to centre points
        Rx = RxC;  Lx = LxC;//Horizontal axis for right/left servos
        Ry = RyC; Ly = LyC; //Vertical axis for right/left servos
        Neck = NeckC; //Neck servo
        SendData(); //Send the PWM data to the OWL
        VideoCapture cap(source); // Open the Video stream input
        if (!cap.isOpened()) //Only run the program if the video source is available and opened
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1; //exit the program
        }

        //While the program is in manual mode...
        while (currentMode == MANUAL){       
            if (!cap.read(Frame)) //Capture an image from the video stream.
            {//output to the console if the frame could not be captured.
                cout  << "Could not open the input video: " << source << endl;
            }
            
            Mat FrameFlpd; cv::flip(Frame, FrameFlpd, 1); // Note that Left/Right are reversed now
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG image
            Left = FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
            Right = FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy);
			// Draw white rectangle around the centre of each image.
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0); 
            rectangle( Left, target, Scalar::all(255), 2, 8, 0);		
			imshow("Left", Left); imshow("Right", RightCopy); //Show the images
            waitKey(1);
			//This pause is long enough to allow a stable photo to be taken, in the next loop.
            int key = waitKey(30); //Get a key press from the user
            switch (key){// lower-case values, make sure caps-lock is off
            case 'w': //move the left camera up
                Ly = Ly - 5; break;
            case 's': //move the left camera down
                Ly = Ly + 5; break;
            case 'a'://move the left camera left
                Lx = Lx - 5; break;
            case 'd'://move the left camera right
                Lx = Lx + 5; break;
            case 'i': //move the right camera up
                Ry = Ry + 5; break;
            case 'k': //move the right camera down
                Ry = Ry - 5; break;
            case 'j': //move the right camera left
                Rx = Rx - 5; break;
            case 'l': //move the right camera right
                Rx = Rx + 5; break;
            case ',': // '<' key   move the neck left
                Neck = Neck + 5; break;
            case '.': // '<' key   move the neck right
                Neck = Neck - 5; break;
            case 'r'://Reset servos to centre positions
                Rx = RxC; Lx = LxC;
                Ry = RyC; Ly = LyC;
                Neck = NeckC;
                break;
            case 'b': //Toggle truncating the PWM data when sending to the OWL
                trunkateOnSend = !trunkateOnSend;
				//Output to the console if truncating or not.
                cout << "Setting Truncate on send: " << (trunkateOnSend ? "True" : "False") << endl;
                break;
            case 't': //Start tracking the target in the centre of the right image
                OWLtempl = Right(target); //set the template to correlate
                waitKey(1);
                currentMode = TRACKING; //Quit Manual loop and start tracking target
                break;
            case 'c': //Start capturing the images, used for disparity task.
                Rx = RxDisparityToeIn; //Set the toe-in values use for calculating disparity.
                Lx = LxDisparityToeIn;
                SendData(); //send the toe-in data to the OWL
                cout << "capturing images..." << endl; 
                OwlCalCapture(cap, IMAGES_FOLDER); //Start capturing the images.
                Rx = RxC; Lx = LxC; //Reset back to centre positions
                break;
            case 27: //[ESC] key, Exit the application
                cout << "Exiting Application";
                currentMode = EXITING;
                break;
            default: //Nothing if the user did not press a key
                key=key;  
            }
            SendData();
        } // END Manual loop

        destroyAllWindows();

        //============= Normalised Cross Correlation ==========================
        // While the application is tracking.
        while (currentMode == TRACKING) {
            trackingLoopCount++; //Increase the loop counter by 1
            //Read a frame from the video feed.
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                break;
            }

            Mat FrameFlpd; cv::flip(Frame, FrameFlpd, 1); // Note that Left/Right are reversed now            
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG image
            Left = FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
            Right = FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

            //Mach the template for both eyes
            OwlCorrel OWL = Owl_matchTemplate(Right, Left, OWLtempl);

            // This is to capture a new target on nth frame (20 default)
            const int NEW_TARGET_CAPTURE_COUNT = 20;

            //Capture a new target template every NEW_TARGET_CAPTURE_COUNT amount of time round the loop
            if (resample) {
                if(trackingLoopCount >= NEW_TARGET_CAPTURE_COUNT)
                {
                     // Move target to the best match
                     target.x = OWL.MatchR.x;
                     target.y = OWL.MatchR.y;
                     // Get new target
                     OWLtempl= Right(target);
                     // Reset counter
                     trackingLoopCount = 0;
                }
            }
			//create a copy of the right image
            Mat RightCopy;
            Right.copyTo(RightCopy);

            //Draw rectangles around the best match of correlation.
            rectangle(RightCopy, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(Left, OWL.MatchL, Point( OWL.MatchL.x + OWLtempl.cols , OWL.MatchL.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.ResultL, OWL.MatchL, Point( OWL.MatchL.x + OWLtempl.cols , OWL.MatchL.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle(OWL.ResultR, OWL.MatchR, Point( OWL.MatchR.x + OWLtempl.cols , OWL.MatchR.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            //Draw a circle at the centre point of each eye.
            circle(Left,Point(320,240),5,Scalar(0,255,0),1);
            circle(RightCopy,Point(320,240),5,Scalar(0,255,0),1);

            // Tracking rates for both eyes, can't be too high or will overshoot target.
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


            //Then calculate the distance for the next loop.
            CalculateDistance();
            // Create the string, for the distance value, calls calcDistance()
            string distanceString = "Distance: " + to_string((int)calcDistance) + "mm";

            // Check to see if eyes are diverging on a target.
            // If not, move the neck towards the direction the eyes
            // are looking in.
            if (Lx < LxC && Rx > RxC) { // If eyes diverge
                distanceString = "Divergent Target lost!";
            }
            else if(Rx > RxC)// Looking to the right
            {
                // Move the neck to the right
                Neck = Neck - 5;
            }
            else if (Lx < LxC) // Looking to the left
            {
                // Move the neck to theleft.
                Neck = Neck + 5;
            }

            // Draw rectangle for distance text.
            rectangle( RightCopy, textBox, Scalar::all(0), -1, 8, 0);
            // Draw rectangle for resample state text.
            rectangle( RightCopy, resampleBox, Scalar::all(0), -1, 8, 0);

            //Then we write the distance string on to the screen.
            putText(RightCopy, distanceString, cvPoint(165, 465), FONT_HERSHEY_DUPLEX, 1, Scalar::all(255), 0, 0, false);
            // coords 520, 33
            putText(RightCopy, "Resampling: ", cvPoint(70, 45), FONT_HERSHEY_DUPLEX, 0.8, Scalar::all(255), 0, 0, false);
            putText(RightCopy, resample ? "True" : "False", cvPoint(230, 45), FONT_HERSHEY_DUPLEX, 0.8, Scalar::all(255), 0, 0, false);
            putText(RightCopy, "| Press 'r' to toggle.", cvPoint(320, 45), FONT_HERSHEY_DUPLEX, 0.8, Scalar::all(255), 0, 0, false);
            //Output all of the variables to console (For debugging and monitoring)
            cout << "Rx: " << Rx << "   Lx: " << Lx << "   Deg2Pwm: " << Deg2Pwm << "   Distance: " << calcDistance << "mm" << endl;

            //Display all of the important windows,
            imshow("Owl-L", Left);
            imshow("CorrelL", OWL.ResultL);
            imshow("CorrelR", OWL.ResultR);
            imshow("Owl-R", RightCopy);
            imshow("Template", OWLtempl);

            // Send the servo data to the PI.
            SendData();

            waitKey(1);
            int key = waitKey(10);//Wait 10 ms for the user to press a key
            //if the user has pressed a key,
            switch (key)
            {   // and the user has pressed the 'm' key...
                case 'm':// Switch to manual mode
                    target.x = 320-32;//Set the target x/y back to the centre of the image.
                    target.y = 240-32;
                    currentMode = MANUAL;//Signal to return back to manual mode.
                    break;
                case 'r': // Toggle re-sampling the template
                    resample = !resample;
                break;
            }

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
