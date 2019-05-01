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
    Point Match;
    Mat Result;

};

Mat OWLtempl; // used in correlation
Rect target = Rect(320-32, 240-32, 64, 64); // target is at the centre of the camera FOV
                                            // drawn over whatever is in the centre of the FOV, to act as a template
const Rect CENTRE_TARGET = Rect(320-32, 240-32, 64, 64); //Constant rectangle for the center of the image, just incase the target above gets changed
//Mat Left
//, Rect target
struct OwlCorrel Owl_matchTemplate(Mat image, Mat templ)
{
    static OwlCorrel OWL;
    // Create the result matrix
    int result_cols = image.cols - templ.cols + 1;
    int result_rows = image.rows - templ.rows + 1;

    OWL.Result.create(result_rows, result_cols, CV_32FC1);

    // Do the Matching, on the right eye, and Normalize
    int match_method = 5; // CV_TM_CCOEFF_NORMED;
    matchTemplate(image, templ, OWL.Result, match_method);

    //Dont normalise in other one, we may need this later
    //normalize(OWL.Result, OWL.Result);

    // Localizing the best match with minMaxLoc
    //variables needed for matching
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;

    //Do the matching
    minMaxLoc(OWL.Result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    //For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if(match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED ) //CV4
    { OWL.Match = minLoc; }
    else
    { OWL.Match = maxLoc; }

    return (OWL);
}
