#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
#include "fisheye_corrector\fisheye_corrector.h"
#include "correctTools.h"

void testPointMapping(int argc, char* argv[])
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


	cv::Mat frame, corrected_frame;

	if (!capture.isOpened())
	{
		std::cout << "Fail to open video" << std::endl;
		system("pause");
		exit(-1);
	}
	capture >> frame;
	float pixel_height = 0.0042;
	float f_image_ = 306.605;
	FisheyeCorrector corrector(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 60);
	corrector.setAxisDirection(30, 35, 0);//30,35,-7
	corrector.updateMap();
	corrector.setClipRegion(cv::Rect(cv::Point(400, 430), cv::Point(corrector.getCorrectedSize().width - 100, corrector.getCorrectedSize().height-300)));
	corrector.setSizeScale(0.5);

	cv::Mat K = corrector.getIntrinsicMatrix();
	std::cout << "K" << std::endl << K << std::endl;
	std::ofstream K_file("K.txt");
	K_file << K << std::endl;
	K_file.close();
	//Use this function if you want to find perspective distortion yourself
	//findPerspectiveDistortion(frame, corrector);



	std::string filePath;
	std::string correct_file_name = "correct_";
	int file_name_pos = videoName.find_last_of('\\');
	if (file_name_pos != -1)
	{
		correct_file_name += videoName.substr(file_name_pos + 1);
		filePath += videoName.substr(0, file_name_pos + 1);
	}
	else
		correct_file_name += videoName;


	cv::VideoWriter writer(filePath + correct_file_name, capture.get(CV_CAP_PROP_FOURCC), capture.get(CV_CAP_PROP_FPS), corrector.getCorrectedSize());
	std::cout << "save file to " << filePath + correct_file_name << std::endl;
	int frameCount = 0;

	cv::ORB orb;
	
	FisheyeCorrector corrector_central(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 60);
	corrector_central.updateMap();
	cv::Mat center_frame;
	K = corrector_central.getIntrinsicMatrix();
	float cx = K.at<double>(0, 2);
	float cy = K.at<double>(1, 2);
	std::cout << "cx " << cx << "  cy " << cy << std::endl;
	while (!frame.empty())
	{
		corrector.correct(frame, corrected_frame);
		corrector_central.correct(frame, center_frame);
		std::vector<cv::KeyPoint> keypoints,keypoints_in_fisheye;
		orb.detect(corrected_frame, keypoints);
		cv::drawKeypoints(corrected_frame, keypoints,corrected_frame);
		corrector.mapFromCorrectedImageToCenterImagePlane(keypoints, keypoints_in_fisheye,cx,cy);
		cv::drawKeypoints(center_frame, keypoints_in_fisheye, center_frame);
		cv::imshow("frame", center_frame);
		cv::imshow("corrected", corrected_frame);
		writer << corrected_frame;
		capture >> frame;
		/*if (frameCount > 3000)
		break;*/
		cv::waitKey(0);
		if (frameCount % 300 == 0)
			std::cout << "frame " << frameCount << std::endl;
		frameCount++;
	}
	writer.release();
}

