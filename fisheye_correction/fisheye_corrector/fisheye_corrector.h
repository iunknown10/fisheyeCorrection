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


	float vertical_range_radian_;
	float horizontal_range_radian_;
	int Width_;
	int Height_;
	float CenterX_;
	float CenterY_;

	float axis_vertical_radian_;
	float axis_horizontal_radian_;
	float axis_rotation_radian_;
	float f_image_;

	float size_scale_;
	Eigen::Matrix4f transform_camera_to_originalplane_;
	cv::Rect clip_region_;


	Eigen::Vector3f new_camera_plane_center;
	Eigen::Vector3f camera_center;
	Eigen::Vector3f original_axis;
private:
	void readDistortionList(std::string file_name);

	void generateMap();

	bool map_need_update = true;

	
public:
	//Correction table and pixelHeight should provided by camera manufactor. focal length will infuence the size of the result image.
	FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f = 306.605, float VerticalDegeree = 60, float HorizontalDegree = 70);


	cv::Mat& correct(const cv::Mat& src, cv::Mat& dst)
	{
		if (map_need_update)
			updateMap(); 
		cv::remap(src, dst, map_, cv::Mat(), cv::INTER_CUBIC); 
		if (size_scale_!=1)
			cv::resize(dst, dst, cv::Size(dst.cols*size_scale_, dst.rows*size_scale_), size_scale_, size_scale_, cv::INTER_CUBIC);
		return dst;
	}
	template<class  pointType>
	void mapToOriginalImage(const std::vector<pointType>& points, std::vector<pointType>& points_in_fisheye);

	template<class pointType>
	void mapFromCorrectedImageToCenterImagePlane(const std::vector<pointType>& points, std::vector<pointType>& points_in_pinhole, float cx, float cy);

	cv::Size getCorrectedSize()
	{
		return cv::Size(map_.cols*size_scale_, map_.rows*size_scale_);
	}

	void setClipRegion(cv::Rect& region)
	{
		clip_region_ = region;
		map_ = original_map_(region);
		std::cout << "Size of corrected imageis  width:" << map_.cols << " height:" << map_.rows << std::endl;
	}

	void setSizeScale(float scale)
	{
		size_scale_ = scale;
	}

	cv::Mat getIntrinsicMatrix()
	{
		K_ = (cv::Mat_<double>(3, 3) <<
			f_image_, 0, CenterX_ - clip_region_.x,
			0, f_image_, CenterY_ - clip_region_.y,
			0, 0, 1
			);
		K_ *= size_scale_;
		K_.at<double>(2, 2) /= size_scale_;
		return K_;
	}

	Eigen::Matrix4f getExtrinsicMatrix()
	{
		Eigen::Matrix4f image_to_imageplane, originalplane_to_camera;
		image_to_imageplane <<
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, -f_image_,
			0, 0, 0, 1;
		originalplane_to_camera<<
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, f_camera_,
			0, 0, 0, 1;
		return (originalplane_to_camera*transform_camera_to_originalplane_*image_to_imageplane).inverse();
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








template<class pointType>
void FisheyeCorrector::mapToOriginalImage(const std::vector<pointType>& points, std::vector<pointType>& points_in_fisheye)
{
	points_in_fisheye.resize(points.size());
	std::cout<<"CenterX_fisheye_ " << CenterX_fisheye_ << "  CenterY_fisheye_ " << CenterY_fisheye_ << std::endl;
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].pt.y/size_scale_ + clip_region_.tl().y;
		float w = points[i].pt.x/size_scale_ + clip_region_.tl().x;
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);

		point_homo = transform_camera_to_originalplane_*point_homo;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
		//std::cout << "point " << point.transpose() << std::endl;
		//Eigen::Vector3f point_vector = (point - camera_center).normalized();

		float cos_value = original_axis.dot((point - camera_center).normalized());
		float degree = radianToDegree(acos(cos_value));
		if (degree > 100)
			continue;
		float radius_in_project = sqrt(point(0)*point(0) + point(1)*point(1));

		int position_floor = floor(degree * 10);
		int position_ceil = ceil(degree * 10);
		float  radius_in_fisheye_floor = distortion_list_[position_floor];
		float  radius_in_fisheye_ceil = distortion_list_[position_ceil];
		float radius_in_fisheye = radius_in_fisheye_floor + (radius_in_fisheye_ceil - radius_in_fisheye_floor)*((degree * 10 - position_floor) / (position_ceil - position_floor));
		radius_in_fisheye = radius_in_fisheye / pixelHeight_;

		float x = ((point(0)) *(radius_in_fisheye / radius_in_project));
		float y = (point(1))*(radius_in_fisheye / radius_in_project);
		//std::cout << "x " << x << "   y " << y << std::endl;
		//Add the map relationship of Point(h,w)
		points_in_fisheye[i] = points[i];
		points_in_fisheye[i].pt.x = x + CenterX_fisheye_;
		points_in_fisheye[i].pt.y = -y + CenterY_fisheye_;
	}
}


template<class pointType>
void FisheyeCorrector::mapFromCorrectedImageToCenterImagePlane(const std::vector<pointType>& points, std::vector<pointType>& points_in_pinhole,float cx,float cy)
{
	points_in_pinhole.resize(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].pt.y / size_scale_ + clip_region_.tl().y;
		float w = points[i].pt.x / size_scale_ + clip_region_.tl().x;
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);

		point_homo = transform_camera_to_originalplane_*point_homo;
		points_in_pinhole[i] = points[i];
		points_in_pinhole[i].pt.x = point_homo(0) + cx;
		points_in_pinhole[i].pt.y = -point_homo(1) + cy;
	}
}