#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>




class FisheyeCorrector
{
	std::vector<float> distortion_list_;
	cv::Mat perspectiveTrans_;
	cv::Mat K_;

	cv::Mat map_;
	float f_;
	float VerticalDegeree_;
	float HorizontalDegree_;
	int Width_;
	int Height_;
	float CenterX_;
	float CenterY_;
	float CenterX_fisheye_;
	float CenterY_fisheye_;
	float pixelHeight_;
private:
	void readDistortionList(std::string file_name);

	void generateMap();


public:
	//Correction table and pixelHeight should provided by camera manufactor. focal length will infuence the size of the result image.
	FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f = 150, float VerticalDegeree = 60, float HorizontalDegree = 70);

	void setPerspectiveTransformation(cv::Mat& perspectiveTrans);


	cv::Mat getPerspectiveTransformation()
	{
		return perspectiveTrans_;
	}

	cv::Mat& correct(const cv::Mat& src, cv::Mat& dst)
	{
		cv::remap(src, dst, map_, cv::Mat(), cv::INTER_CUBIC);
		return dst;
	}

	cv::Size getCorrectedSize()
	{
		return cv::Size(Width_, Height_);
	}

	cv::Mat getIntrinsicMatrix()
	{
		return K_;
	}
};
