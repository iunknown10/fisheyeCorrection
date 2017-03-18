#include "correctTools.h"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>


#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
//#include <opencv2\contrib\retina.hpp>


int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "A distortion table file path and a video/image file path needed to correct should include in the input parameters." << std::endl;
		exit(0);
	}
	//correctImage(argc, argv);
	//correctVideo(argc, argv);
	//testPointMapping(argc, argv);
	lableMapping(argc, argv);
}