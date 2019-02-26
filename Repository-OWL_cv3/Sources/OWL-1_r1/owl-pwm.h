#define _USE_MATH_DEFINES
#include <math.h>

#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP




// Defines for servo limits
// PFC Owl robot
// (c) Plymouth University

//Owl 5 setup
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

static int IPD = 67;//mm, interp pupelary distance

static int hFOV = 53; // degrees
static int vFOV = 40; // degrees
static int dFOV = 66; // degrees

static int RxPx2Deg = hFOV / RxRangeV;
static int RyPx2Deg = vFOV / RyRangeV;
static int LxPx2Deg = hFOV / LxRangeV;
static int LyPx2Deg = vFOV / LyRangeV;

//160 degree of servo control
static float RxDeg2PWM = RxRangeM / eyeMaxDeg;
static int LxDeg2PWM = LxRangeM / eyeMaxDeg;
