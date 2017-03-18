#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "fisheye_corrector\fisheye_corrector.h"
#include "correctTools.h"

#define VIEW_NUMBER 3

const char* window_names[VIEW_NUMBER] = { "left", "middle", "right" };

struct on_mouseParameters
{
	cv::Mat* image;
	cv::Mat* frame;
	FisheyeCorrector* corrector;
	std::vector<cv::Point2f>* points;
	std::string window_name;
};
void on_mouse(int event, int x, int y, int flags, void* ustc);
void drawLines(cv::Mat& image, std::vector<std::vector<cv::Point2f>>& line_points);
void mapLines(FisheyeCorrector& correct, std::vector<cv::Point2f>& lines_in_corrected, std::vector<std::vector<cv::Point2f>>& line_points_in_fisheye);

void lableMapping(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "need correction table file and video name for parameters" << std::endl;
	}


	std::string correction_table = argv[1];
	std::string videoName = argv[2];
	cv::VideoCapture capture(videoName);
	cv::namedWindow("frame", 0);


	cv::Mat frame;
	if (!capture.isOpened())
	{
		std::cout << "Fail to open video" << std::endl;
		system("pause");
		exit(-1);
	}

	capture >> frame;
	cv::imshow("frame", frame);
	cv::waitKey(10);
	capture.set(CV_CAP_PROP_FPS, 25);
	float pixel_height = 0.0042;
	float f_image_ = 306.605;
	
	cv::Mat corrected_images[VIEW_NUMBER];
	std::vector<FisheyeCorrector> correctors(3);

	correctors[2] = FisheyeCorrector(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[2].setAxisDirection(80, 40, -15);//30,35,-7
	correctors[2].updateMap();
	correctors[2].setClipRegion(cv::Rect(cv::Point2f(0, 605), cv::Point2f(correctors[2].getCorrectedSize().width, correctors[2].getCorrectedSize().height - 300)));
	//correctors[0].setSizeScale(0.5);


	correctors[1] = FisheyeCorrector(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 40);
	correctors[1].setAxisDirection(0, 40, 0);//30,35,-7
	correctors[1].updateMap();
	correctors[1].setClipRegion(cv::Rect(cv::Point2f(0, 475), cv::Point2f(correctors[1].getCorrectedSize().width, correctors[1].getCorrectedSize().height - 500)));
	//correctors[1].setSizeScale(0.5);

	correctors[0] = FisheyeCorrector(correction_table, capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[0].setAxisDirection(-80, 40, 15);//30,35,-7
	correctors[0].updateMap();
	correctors[0].setClipRegion(cv::Rect(cv::Point2f(0, 605), cv::Point2f(correctors[0].getCorrectedSize().width, correctors[0].getCorrectedSize().height - 300)));
	//correctors[2].setSizeScale(0.5);


	
	std::cout << capture.get(CV_CAP_PROP_FPS) << std::endl;
	int frameCount = 0;

	std::vector<std::vector<cv::Point2f>> selected_points(VIEW_NUMBER);

	std::string window_prefix = "corrected_";
	std::vector<on_mouseParameters*> mouse_parameter(VIEW_NUMBER);
	for (int i = 0; i < VIEW_NUMBER; i++)
	{
		cv::namedWindow(window_prefix+window_names[i], 0);
		mouse_parameter[i] = new on_mouseParameters();
		mouse_parameter[i]->points = &selected_points[i];
		mouse_parameter[i]->image =  &corrected_images[i];
		mouse_parameter[i]->frame = &frame;
		mouse_parameter[i]->corrector = &correctors[i];
		mouse_parameter[i]->window_name = window_prefix + window_names[i];
		cvSetMouseCallback(mouse_parameter[i]->window_name.c_str(), on_mouse, (mouse_parameter[i]));
	}

	std::vector<std::vector<cv::Point2f>> line_points_in_fisheye;

	while (!frame.empty())
	{
		for (int i = 0; i < VIEW_NUMBER; i++)
		{
			correctors[i].correct(frame, corrected_images[i]);
			cv::imshow(window_prefix+window_names[i], corrected_images[i]);
		}

		
		cv::imshow("frame", frame);
		cv::waitKey(0);

		for (int i = 0; i < VIEW_NUMBER; i++)
		{
			mapLines(correctors[i], selected_points[i], line_points_in_fisheye);
		}
		drawLines(frame, line_points_in_fisheye);
		cv::imshow("frame", frame);

		capture >> frame;
		frameCount++;
	}
}

void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	on_mouseParameters parameter = *(on_mouseParameters*)ustc;
	std::vector<cv::Point2f>& points = *parameter.points;
	cv::Mat& image = *parameter.image;
	std::vector<std::vector<cv::Point2f>> points_in_fisheye;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
			cv::Point2f pt = cv::Point2f(x, y);
			points.push_back(pt);
			if (points.size() % 2 == 0)
			{
				cv::circle(image, points.back(), 2, cv::Scalar(255, 0, 0));
				cv::line(image, points[points.size() - 1], points[points.size() - 2], cv::Scalar(255, 0, 0));
				mapLines(*parameter.corrector, points, points_in_fisheye);
				drawLines(*parameter.frame, points_in_fisheye);
			}
			else
			{
				cv::circle(image, points.back(), 2, cv::Scalar(255, 0, 0));
			}
			cv::imshow(parameter.window_name, image);
			cv::imshow("frame", *parameter.frame);
			cv::waitKey(10);
	}

}


void drawLines(cv::Mat& image, std::vector<std::vector<cv::Point2f>>& line_points)
{
	for (int i = 0; i < line_points.size(); i++)
	{
		for (int j = 0; j < line_points[i].size(); j++)
			//cv::line(image, line_points[i][j], line_points[i][j], cv::Scalar(255, 0, 0), 2);
			image.at<cv::Vec3b>(line_points[i][j].y, line_points[i][j].x) = cv::Vec3b(255, 0, 0);
	}
}

void mapLines(FisheyeCorrector& correct, std::vector<cv::Point2f>& lines_in_corrected, std::vector<std::vector<cv::Point2f>>& line_points_in_fisheye)
{
	int line_num = lines_in_corrected.size();
	for (int i = 0; i <line_num - 1; i += 2)
	{
		
		cv::Point2f step = lines_in_corrected[i+1] - lines_in_corrected[i];
		step = step*(1/cv::norm(step));

		std::vector<cv::Point2f> line_points;
		cv::Point2f current_point = lines_in_corrected[0];
		cv::Point2f& end_point = lines_in_corrected.back();
		while (cv::norm(current_point-end_point)>1)
		{
			if (line_points.empty()|| ((int)line_points.back().x - (int)current_point.x) != 0 || ((int)line_points.back().y - (int)current_point.y))
			{
				line_points.push_back(current_point);
			}
			current_point += step;
			
		}
		std::vector<cv::Point2f> line_points_mapped;
		correct.mapToOriginalImage(line_points, line_points_mapped);
		line_points_in_fisheye.push_back(line_points_mapped);
	}
}