#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "fisheye_corrector\fisheye_corrector.h"


void correctVideo(int argc, char* argv[]);
void correctImage(int argc, char* argv[]);

void testPointMapping(int argc, char* argv[]);
void lableMapping(int argc, char* argv[]);