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

struct OwlCorrel {
    Point MatchL;
    Point MatchR;
    Mat ResultL;
    Mat ResultR;
};

Mat OWLtempl; // used in correlation
Rect target = Rect(320-32, 240-32, 64, 64); // target is at the centre of the camera FOV
Rect textBox = Rect(320-155, 433, 310, 40); // target is at the centre of the camera FOV
Rect resampleBox = Rect(65, 20, 520, 33); // target is at the centre of the camera FOV
// drawn over whatever is in the centre of the FOV, to act as a template

// Overlays a template over 2 images and finds the best match for the template to be.
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

    // Do the Matching and Normalize
    int match_method = 5; // CV_TM_CCOEFF_NORMED;
    matchTemplate(Left, templ, OWL.ResultL, match_method);
    matchTemplate(Right, templ, OWL.ResultR, match_method);

    // Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;

    //Find the location of the best match.
    // Left Eye:
    minMaxLoc(OWL.ResultL, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED ) //CV4
    {
        OWL.MatchL = minLoc;
    }
    else
    {
        OWL.MatchL = maxLoc;
    }

    //Find the location of the best match.
    // Right Eye:
    minMaxLoc(OWL.ResultR, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED ) //CV4
    {
        OWL.MatchR = minLoc;
    }
    else
    {
        OWL.MatchR = maxLoc;
    }

    return (OWL);
}

int OwlCalCapture(cv::VideoCapture &cap, string Folder){
    int const MAX_IMAGES = 20;
    int imageCount = 0;
    cv::Mat Frame;
    cout << "Please press {spacebar} to capture an image... " << endl;
    int key = 0;
    while(imageCount < MAX_IMAGES)
    {
        if (!cap.read(Frame))
        {
            return(-1);
        }
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        cv::Mat Right = Frame( Rect(0, 0, 640, 480)); // using a rectangle
        cv::Mat Left = Frame( Rect(640, 0, 640, 480)); // using a rectanglecv::imwrite(Folder + "left" + count + "jpg", Left);
        imshow("Left",Left);
        imshow("Right",Right);
        key = waitKey(10);
        if(key == 32)
        {
            string fnameR(Folder + "right" + to_string(imageCount) + ".jpg");
            string fnameL=(Folder + "left" +  to_string(imageCount) + ".jpg");
            cv::imwrite(fnameL, Left);
            cv::imwrite(fnameR, Right);
            cout << "Saved " << imageCount << " stereo pair" << Folder <<endl;
            imageCount++;
        }


    }

    destroyAllWindows();
    cout << "Just saved 10 stereo pairs" << Folder <<endl;
    return(0);
}
