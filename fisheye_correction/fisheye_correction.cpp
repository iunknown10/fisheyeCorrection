#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "fisheye_corrector.h"
#include "find_perspective_transform.h"

const float focal = 150;//Measured in pixels

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "need correction table file and video name for parameters" << std::endl;
	}
		

	std::string correction_table = argv[1];
	std::string videoName = argv[2];
	cv::VideoCapture capture(videoName);
	cv::namedWindow("frame", 0);
	cv::namedWindow("corrected", 0);


	cv::Mat frame,corrected_frame;

	if (!capture.isOpened())
	{
		std::cout << "Fail to open video" << std::endl;
		system("pause");
		exit(-1);
	}
	capture >> frame;

	FisheyeCorrector corrector(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH));
	//Deal with the perspective distortion
	//This is a perspective distortion correction matrix finded manually. 
	//Please notice that this is for vertical range at 53 degree  and horiental range 70 degree and different range should use different perspective correction
	cv::Mat perspectiveTransform = (cv::Mat_<double>(3, 3) <<
			0.7120342503081317, -0.9080554014444526, 98.72580114592542,
			-0.03621256531992459, 0.7347221335742586, 15.81086229667521,
			-0.0001161970505587349, -0.002209380538794291, 1
			);
	corrector.setPerspectiveTransformation(perspectiveTransform);
	std::cout << "K" << std::endl << corrector.getIntrinsicMatrix() << std::endl;
	//Use this function if you want to find perspective distortion yourself
	//findPerspectiveDistortion(frame, corrector);




	std::string correct_file_name = "correct2_";
	int file_name_pos = videoName.find_last_of('\\');
	if (file_name_pos != -1)
		correct_file_name += videoName.substr(file_name_pos);
	else
		correct_file_name += videoName;


	cv::VideoWriter writer(correct_file_name, capture.get(CV_CAP_PROP_FOURCC), capture.get(CV_CAP_PROP_FPS), corrector.getCorrectedSize());

	int frameCount = 0;
	while (!frame.empty())
	{
		corrector.correct(frame, corrected_frame);

		cv::imshow("frame", frame);
		cv::imshow("corrected", corrected_frame);

		writer << corrected_frame;
		capture >> frame;
		cv::waitKey(10);
		if (frameCount > 1000)
			break;
		frameCount++;
	}
	writer.release();
}

