#ifndef OWLCV_H
#define OWLCV_H

#endif // OWLCV_H

/* Phil Culverhouse
 *
 * Vision Processing for OWL camera system
 *  Currently provides Normalised Cross Correlation for template match
 *  uses opencv, assumes 3.1 or similar
 *  uses the Right eye for template source.
 * (c) Plymouth University, 2016
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

struct OwlCorrel {//Results of correlating  images 
    Point MatchL; //The best match location for the left image
    Point MatchR; //The best match location for the right image
    Mat ResultL; //correlation result matrix for the left image
    Mat ResultR; //correlation result matrix for the right image
};

Mat OWLtempl; // used in correlation
Rect target = Rect(320-32, 240-32, 64, 64); // initialise the target is at the centre of the camera FOV
Rect textBox = Rect(320-155, 433, 310, 40); // The distance text box 
Rect resampleBox = Rect(65, 20, 520, 33); // The re-sampling text box

// Overlays a template over 2 images and finds the best match within the images.
struct OwlCorrel Owl_matchTemplate(Mat Right, Mat Left, Mat templ)
{
    static OwlCorrel OWL;
    // Initialise the result matrix for left eye
    int result_cols =  Left.cols - templ.cols + 1;
    int result_rows = Left.rows - templ.rows + 1;    
    OWL.ResultL.create(result_rows, result_cols,  CV_32FC1 );

    // Initialise the result matrix for right eye
    result_cols = Right.cols - templ.cols + 1;
    result_rows = Right.rows - templ.rows + 1;
    OWL.ResultR.create(result_rows, result_cols, CV_32FC1);

    // Do the Matching and Normalize process
	// Produces a matrix of values based on match accuracy
    int match_method = 5; // CV_TM_CCOEFF_NORMED;
	//Output the results of the correlation to OWL.ResultL and ResultR
    matchTemplate(Left, templ, OWL.ResultL, match_method);
    matchTemplate(Right, templ, OWL.ResultR, match_method);

    // Localizing the best match with minMaxLoc
    double minVal, maxVal;
	Point minLoc, maxLoc;

    //Find the location of the best match for the Left eye,
	// by finding the min and max values of the result Matrix.
    minMaxLoc(OWL.ResultL, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    //For SQDIFF and SQDIFF_NORMED, the best matches are lower values. 
	// For all the other methods, the higher the better
    if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED )
    { OWL.MatchL = minLoc; }
    else
    { OWL.MatchL = maxLoc; }
	
	//Find the location of the best match for the Right eye:
    minMaxLoc(OWL.ResultR, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED )
    { OWL.MatchR = minLoc; }
    else
    { OWL.MatchR = maxLoc; }
	//return the created OwlCorrel object.
    return (OWL);
}

//Capture images from the video source, and save them into a folder location
int OwlCalCapture(cv::VideoCapture &cap, string Folder){
    int const MAX_IMAGES = 20;//Maximum images to capture
    int imageCount = 0;//the current
    cv::Mat Frame; //the frame from the video source
	//Signal to the user to what key to press to capture images.
    cout << "Please press {spacebar} to capture an image... " << endl; 	
    int key = 0;//key which user has pressed.
    while(imageCount < MAX_IMAGES)//while we haven't captured the maximum amount of images.....
    {
        if (!cap.read(Frame))//Read a frame from the video source
        {//If the frame was not read, return out of this function.
            return(-1);
        }
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG image...
        cv::Mat Right = Frame( Rect(0, 0, 640, 480)); // using a rectangle
        cv::Mat Left = Frame( Rect(640, 0, 640, 480)); // using a rectangle
        imshow("Left",Left);//Show the left image
        imshow("Right",Right);//Show the right image

		//Wait 10ms for a user input, also a long enough wait to capture a stable image.
        key = waitKey(10);
        if(key == 32)//32 = {spacebar} key code
        {//If the user pressed the spacebar, then save the images.
            string fnameR=(Folder + "right" + to_string(imageCount) + ".jpg");//create the file path
            string fnameL=(Folder + "left" +  to_string(imageCount) + ".jpg");
            cv::imwrite(fnameL, Left);//save the images
            cv::imwrite(fnameR, Right);
			//Signal to the user which pair of images has been saved
            cout << "Saved " << imageCount << " stereo pair" << Folder <<endl;
			//increase the image count, also so the names of the next images are different.
            imageCount++;
        }
    }

    destroyAllWindows();
	//Signal to the user how many pairs of images where saved
    cout << "Just saved "<< to_string(imageCount) << " stereo pairs to: " << Folder <<endl;
    return(0);
}
