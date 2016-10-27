#pragma once
#include "fisheye_corrector.h"


void findPerspectiveDistortion(cv::Mat& frame, FisheyeCorrector& corrector);