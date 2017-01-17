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
	cv::VideoCapture video(videoName);
	cv::namedWindow("frame0", 0);
	cv::namedWindow("frame1", 0);
	cv::namedWindow("frame2", 0);
	cv::namedWindow("corrected", 0);


	cv::Mat frame, corrected_frame;

	if (!video.isOpened())
	{
		std::cout << "Fail to open video" << std::endl;
		system("pause");
		exit(-1);
	}
	video >> frame;
	float pixel_height = 0.0042;
	float f_image_ = 306.605;

	std::cout << "generate corrector" << std::endl;
	std::vector<FisheyeCorrector> correctors(3);
	std::cout << video.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << video.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << pixel_height << std::endl;
	std::cout << f_image_ << std::endl;
	correctors[0] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 40);
	correctors[0].setAxisDirection(0, 40, 0);//30,35,-7
	correctors[0].updateMap();
	correctors[0].setClipRegion(cv::Rect(cv::Point(0, 475), cv::Point(correctors[0].getCorrectedSize().width, correctors[0].getCorrectedSize().height-200)));
	//correctors[0].setSizeScale(0.5);

	correctors[1] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[1].setAxisDirection(80, 40,-15);//30,35,-7
	correctors[1].updateMap();
	correctors[1].setClipRegion(cv::Rect(cv::Point(0, 605), cv::Point(correctors[1].getCorrectedSize().width, correctors[1].getCorrectedSize().height)));
	//correctors[1].setSizeScale(0.5);


	correctors[2] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[2].setAxisDirection(-80, 40,15);//30,35,-7
	correctors[2].updateMap();
	correctors[2].setClipRegion(cv::Rect(cv::Point(0, 605), cv::Point(correctors[2].getCorrectedSize().width, correctors[2].getCorrectedSize().height)));
	//correctors[2].setSizeScale(0.5);



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


	cv::VideoWriter writer(filePath + correct_file_name, video.get(CV_CAP_PROP_FOURCC), video.get(CV_CAP_PROP_FPS), correctors[0].getCorrectedSize());
	std::cout << "save file to " << filePath + correct_file_name << std::endl;
	int frameCount = 0;

	cv::ORB orb;


	//FisheyeCorrector corrector_center = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 80, 80);
	//corrector_center.setAxisDirection(0, 0, 0);//30,35,-7
	//corrector_center.updateMap();
	//cv::Mat K = corrector_center.getIntrinsicMatrix();
	//cv::Mat center_frame;

	while (!frame.empty())
	{
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		
		//corrector_center.correct(frame, center_frame);
		std::vector<cv::KeyPoint> keypoints, keypoints_in_fisheye;
		std::vector<bool> points_validat;
		for (int i = 0; i < 3; i++)
		{
			correctors[i].correct(frame, corrected_frame);
			orb.detect(corrected_frame, keypoints);
			correctors[i].mapToOriginalImage(keypoints, keypoints_in_fisheye);
			//correctors[i].mapFromCorrectedImageToCenterImagePlane(keypoints, keypoints_in_fisheye, points_validat, K.at<double>(0, 2), K.at<double>(0, 2), K.at<double>(0,0));
			cv::drawKeypoints(frame, keypoints_in_fisheye, frame);
			std::stringstream sst;
			sst << "frame" << i;
			cv::imshow(sst.str(), corrected_frame);
			cv::waitKey(10);
		}
		
		//cv::imshow("frame", center_frame);
		cv::imshow("corrected", frame);
		//writer << corrected_frame;
		video >> frame;
		/*if (frameCount > 3000)
		break;*/
		cv::waitKey(0);
		if (frameCount % 300 == 0)
			std::cout << "frame " << frameCount << std::endl;
		frameCount++;
	}
	writer.release();
}

