#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <Eigen\core>
#include <Eigen\geometry>
const float kPI = 3.14159;
inline float degreeToRadian(float d){ return (d / 180.f)* kPI; }
inline float radianToDegree(float r){ return (r / kPI)* 180.f; }



class FisheyeCorrector
{
	std::vector<float> distortion_list_;
	cv::Mat K_;

	cv::Mat original_map_;
	cv::Mat map_;
	float f_camera_;
	float CenterX_fisheye_;
	float CenterY_fisheye_;
	float pixelHeight_;


	float vertical_range_;
	float horizontal_range_;
	int Width_;
	int Height_;
	float CenterX_;
	float CenterY_;

	float axis_vertical_radian_;
	float axis_horizontal_radian_;
	float axis_rotation_radian_;
	float f_image_;

	float size_scale_;
	Eigen::Matrix4f transform_camera_to_world_;
	cv::Rect clip_region_;
private:
	void readDistortionList(std::string file_name);

	void generateMap();

	bool map_need_update = true;

	
public:
	//Correction table and pixelHeight should provided by camera manufactor. focal length will infuence the size of the result image.
	FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f = 150, float VerticalDegeree = 60, float HorizontalDegree = 70);


	cv::Mat& correct(const cv::Mat& src, cv::Mat& dst)
	{
		if (map_need_update)
			updateMap(); 
		cv::remap(src, dst, map_, cv::Mat(), cv::INTER_CUBIC);
		return dst;
	}

	cv::Size getCorrectedSize()
	{
		return cv::Size(map_.cols, map_.rows);
	}

	void setClipRegion(cv::Rect& region)
	{
		clip_region_ = region;
		map_ = original_map_(region);
		std::cout << "Size of corrected imageis  width:" << map_.cols << " height:" << map_.rows << std::endl;
	}

	cv::Mat getIntrinsicMatrix()
	{
		K_ = (cv::Mat_<double>(3, 3) <<
			f_image_, 0, CenterX_ - clip_region_.x,
			0, f_image_, CenterY_ - clip_region_.y,
			0, 0, 1
			);

		return K_;
	}

	Eigen::Matrix4f getExtrinsicMatrix()
	{
		return transform_camera_to_world_.inverse();
	}

	void setAxisDirection(float axis_direction_horizontal, float axis_direction_vertical, float axis_rotation)
	{
		axis_vertical_radian_ = degreeToRadian(axis_direction_vertical);
		axis_horizontal_radian_ = degreeToRadian(axis_direction_horizontal);
		axis_rotation_radian_ = degreeToRadian(axis_rotation);
		map_need_update = true;
	}
	void updateMap()
	{
		generateMap();

		if (clip_region_.area() == 0)
			clip_region_ = cv::Rect(0, 0, Width_, Height_);

		setClipRegion(clip_region_);
		map_need_update = false;
	}
};
