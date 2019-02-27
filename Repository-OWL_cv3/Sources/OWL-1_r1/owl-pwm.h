#define _USE_MATH_DEFINES
#include <math.h>

#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP


#define OWL14


// Defines for servo limits
// PFC Owl robot
// (c) Plymouth University
#ifdef OWL5
// OWL eye ranges (max)
static int RyBm = 1120; // (bottom) to
static int RyTm = 2000; //(top)
static int RxRm = 1890; //(right) to
static int RxLm = 1200; //(left)
static int LyBm = 2000; //(bottom) to
static int LyTm = 1180; //(top)
static int LxRm = 1850; // (right) to
static int LxLm = 1180; // (left)
static int NeckR = 1100;
static int NeckL = 1940;
// VGA match ranges
static int RyBv = 1220; // (bottom) to
static int RyTv = 1600; //(top)
static int RxRv = 1775; //(right) to
static int RxLv = 1250; //(left)
static int LyBv = 1800; //(bottom) to
static int LyTv = 1365; //(top)
static int LxRv = 1795; // (right) to
static int LxLv = 1260; // (left)
static int RxC=1525;//1545;
static int RyC=1435;//1460;
static int LxC=1535;//1545;
static int LyC=1565;//560;
static int NeckC = 1540;

static int IPD = 67;//mm, interp pupelary distance
#endif
#ifdef OWL14
// OWL eye ranges (max)
static int RyBm = 1120; // (bottom) to
static int RyTm = 2000; //(top)
static int RxRm = 1890; //(right) to
static int RxLm = 1200; //(left)

static int LyBm = 2000; //(bottom) to
static int LyTm = 1180; //(top)
static int LxRm = 1850; // (right) to
static int LxLm = 1180; // (left)
static int NeckR = 1100;
static int NeckL = 1950;
// VGA match ranges
static int RyBv = 1310; // (bottom) to
static int RyTv = 1710; //(top)
static int RxRv = 1850; //(right) to
static int RxLv = 1285; //(left)

static int LyBv = 1665; //(bottom) to
static int LyTv = 1290; //(top)
static int LxRv = 1780; // (right) to
static int LxLv = 1265; // (left)

static int RxC=1565;//1545;
static int RyC=1525;//1460;
static int LxC=1520;//1545;
static int LyC=1505;//1560;
static int NeckC = 1490;


static int IPD = 65;//mm, inter-pupelary distance
#endif



static int Ry,Rx,Ly,Lx,Neck; // calculate values for position
//MAX servo eye socket ranges
static int RyRangeM=RyTm-RyBm;
static int RxRangeM=RxRm-RxLm;
static int LyRangeM=LyTm-LyBm; // reflected so negative
static int LxRangeM=LxRm-LxLm;
static int NeckRange=NeckL-NeckR;
//vga CAMERA ranges
static int RyRangeV=RyTv-RyBv;
static int RxRangeV=RxRv-RxLv;
static int LyRangeV=LyTv-LyBv; // reflected so negative
static int LxRangeV=LxRv-LxLv;

static int eyeMaxDeg = 160;
static int eyeMinDeg = 0;



static int hFOV = 53; // degrees
static int vFOV = 40; // degrees
static int dFOV = 66; // degrees

static int RxPx2Deg = hFOV / RxRangeV;
static int RyPx2Deg = vFOV / RyRangeV;
static int LxPx2Deg = hFOV / LxRangeV;
static int LyPx2Deg = vFOV / LyRangeV;

//160 degree of servo control
//Place target at known distance away (1m),
//then change this value till the result is the known value (1m)
static float Deg2Pwm = 0.0475;


static float rightRads = 0;
static float leftRads = 0;

static float calcDistance = 0;
