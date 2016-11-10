#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "fisheye_corrector\fisheye_corrector.h"
#include "find_perspective_transform\find_perspective_transform.h"
#include "correctTools.h"

void correctImage(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "need correction table file and video name for parameters" << std::endl;
	}


	std::string correction_table = argv[1];
	std::string imageName = argv[2];
	
	cv::Mat image = cv::imread(imageName);
	cv::Mat corrected_frame;

	cv::namedWindow("frame", 0);
	cv::namedWindow("corrected", 0);


	std::string filePath;
	std::string correct_file_name = "correct2_";
	int file_name_pos = imageName.find_last_of('\\');
	if (file_name_pos != -1)
	{
		correct_file_name += imageName.substr(file_name_pos + 1);
		filePath += imageName.substr(0, file_name_pos+1);
	}
	else
		correct_file_name += imageName;

	float pixelHeight = 0.0042;
	FisheyeCorrector corrector(correction_table, image.rows, image.cols, pixelHeight);
	//Deal with the perspective distortion
	//This is a perspective distortion correction matrix finded manually. 
	//Please notice that this is for vertical range at 53 degree  and horiental range 70 degree and different range should use different perspective correction
	cv::Mat perspectiveTransform = (cv::Mat_<double>(3, 3) <<
		0.7120342503081317, -0.9080554014444526, 98.72580114592542,
		-0.03621256531992459, 0.7347221335742586, 15.81086229667521,
		-0.0001161970505587349, -0.002209380538794291, 1
		);
	/*corrector.setPerspectiveTransformation(perspectiveTransform);*/

	//Use this function if you want to find perspective distortion yourself
	//findPerspectiveDistortion(frame, corrector);

	
		corrector.correct(image, corrected_frame);

		cv::imshow("frame", image);
		cv::imshow("corrected", corrected_frame);
		cv::imwrite(filePath+correct_file_name, corrected_frame);
		std::cout << "save file to " << filePath + correct_file_name << std::endl;
		cv::waitKey(0);
}

