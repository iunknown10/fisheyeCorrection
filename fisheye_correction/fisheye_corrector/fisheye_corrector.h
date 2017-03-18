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
	Eigen::MatrixX4f transform_camera_to_originalplane_;
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
	FisheyeCorrector()
		:pixelHeight_(0), f_camera_(0), horizontal_range_radian_(degreeToRadian(0)), vertical_range_radian_(degreeToRadian(0))
	{
		size_scale_ = 1;
		CenterX_fisheye_ = 0 / 2.0f;
		CenterY_fisheye_ = 0 / 2.0f;
	
		axis_vertical_radian_ = 0;
		axis_horizontal_radian_ = 0;
		axis_rotation_radian_ = 0;

		clip_region_ = cv::Rect(0, 0, 0, 0);
		map_need_update = true;
		new_camera_plane_center = Eigen::Vector3f(0, 0, 0);
		camera_center = Eigen::Vector3f(0, 0, 0);
		original_axis = Eigen::Vector3f(0, 0, 0);
	};
	FisheyeCorrector(FisheyeCorrector& f)
	{
		distortion_list_.assign(f.distortion_list_.begin(), f.distortion_list_.end());
		f.K_.copyTo(K_);

		f.original_map_.copyTo(original_map_);
		f.map_.copyTo(map_);
		f_camera_ = f.f_camera_;
		CenterX_fisheye_ = f.CenterX_fisheye_;
		CenterY_fisheye_ = f.CenterY_fisheye_;
		pixelHeight_ = f.pixelHeight_;


		vertical_range_radian_ = f.vertical_range_radian_;
		horizontal_range_radian_ = f.horizontal_range_radian_;
		Width_ = f.Width_;
		Height_ = f.Height_;
		CenterX_ = f.CenterX_;
		CenterY_ = f.CenterY_;

		axis_vertical_radian_ = f.axis_vertical_radian_;
		axis_horizontal_radian_ = f.axis_horizontal_radian_;
		axis_rotation_radian_ = f.axis_rotation_radian_;
		f_image_ = f.f_image_;

		size_scale_ = f.size_scale_;
		clip_region_ =  f.clip_region_;

		/*transform_camera_to_originalplane_ << f.transform_camera_to_originalplane_(0, 0), f.transform_camera_to_originalplane_(0, 1), f.transform_camera_to_originalplane_(0, 2), f.transform_camera_to_originalplane_(0,3),
			f.transform_camera_to_originalplane_(1, 0), f.transform_camera_to_originalplane_(1, 1), f.transform_camera_to_originalplane_(1, 2), f.transform_camera_to_originalplane_(1, 3),
			f.transform_camera_to_originalplane_(2, 0), f.transform_camera_to_originalplane_(2, 1), f.transform_camera_to_originalplane_(2, 2), f.transform_camera_to_originalplane_(2, 3),
			f.transform_camera_to_originalplane_(3, 0), f.transform_camera_to_originalplane_(3, 1), f.transform_camera_to_originalplane_(3, 2), f.transform_camera_to_originalplane_(3, 3);*/
		//= Eigen::Matrix4f(f.transform_camera_to_originalplane_);
		new_camera_plane_center = Eigen::Vector3f(f.new_camera_plane_center);
		camera_center = Eigen::Vector3f(f.camera_center);
		original_axis = Eigen::Vector3f(f.original_axis);
	}

	FisheyeCorrector& operator=(FisheyeCorrector& f)
	{
		if (this == &f)
			return *this;
		distortion_list_.assign(f.distortion_list_.begin(), f.distortion_list_.end());
		f.K_.copyTo(K_);

		f.original_map_.copyTo(original_map_);
		f.map_.copyTo(map_);
		f_camera_ = f.f_camera_;
		CenterX_fisheye_ = f.CenterX_fisheye_;
		CenterY_fisheye_ = f.CenterY_fisheye_;
		pixelHeight_ = f.pixelHeight_;


		vertical_range_radian_ = f.vertical_range_radian_;
		horizontal_range_radian_ = f.horizontal_range_radian_;
		Width_ = f.Width_;
		Height_ = f.Height_;
		CenterX_ = f.CenterX_;
		CenterY_ = f.CenterY_;

		axis_vertical_radian_ = f.axis_vertical_radian_;
		axis_horizontal_radian_ = f.axis_horizontal_radian_;
		axis_rotation_radian_ = f.axis_rotation_radian_;
		f_image_ = f.f_image_;

		size_scale_ = f.size_scale_;
		clip_region_ = f.clip_region_;


		transform_camera_to_originalplane_ = f.transform_camera_to_originalplane_;
		/*transform_camera_to_originalplane_ << f.transform_camera_to_originalplane_(0, 0), f.transform_camera_to_originalplane_(0, 1), f.transform_camera_to_originalplane_(0, 2), f.transform_camera_to_originalplane_(0, 3),
			f.transform_camera_to_originalplane_(1, 0), f.transform_camera_to_originalplane_(1, 1), f.transform_camera_to_originalplane_(1, 2), f.transform_camera_to_originalplane_(1, 3),
			f.transform_camera_to_originalplane_(2, 0), f.transform_camera_to_originalplane_(2, 1), f.transform_camera_to_originalplane_(2, 2), f.transform_camera_to_originalplane_(2, 3),
			f.transform_camera_to_originalplane_(3, 0), f.transform_camera_to_originalplane_(3, 1), f.transform_camera_to_originalplane_(3, 2), f.transform_camera_to_originalplane_(3, 3);*/
		new_camera_plane_center = Eigen::Vector3f(f.new_camera_plane_center);
		camera_center = Eigen::Vector3f(f.camera_center);
		original_axis = Eigen::Vector3f(f.original_axis);

		return *this;
	}
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
	void  mapFromCorrectedImageToCenterImagePlane(const std::vector<pointType>& points, std::vector<pointType>& points_in_pinhole, std::vector<bool>& points_validat, float cx, float cy, float f_center_image);

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
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].y/size_scale_ + clip_region_.tl().y;
		float w = points[i].x/size_scale_ + clip_region_.tl().x;
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

		float x = point(0) *(radius_in_fisheye / radius_in_project);
		float y = point(1)*(radius_in_fisheye / radius_in_project);
		//std::cout << "x " << x << "   y " << y << std::endl;
		//Add the map relationship of Point(h,w)
		points_in_fisheye[i] = points[i];
		points_in_fisheye[i].x = x + CenterX_fisheye_;
		points_in_fisheye[i].y = -y + CenterY_fisheye_;
	}
}


template<class pointType>
void FisheyeCorrector::mapFromCorrectedImageToCenterImagePlane(const std::vector<pointType>& points, std::vector<pointType>& points_in_pinhole, std::vector<bool>& points_validat, float cx, float cy, float f_center_image)
{
	const float max_project_angle_cos = 0.15;//about 82 degree


	points_in_pinhole.resize(points.size());
	points_validat.resize(points.size());
	double ratio = f_center_image / f_camera_;
	for (int i = 0; i < points.size(); i++)
	{
		float h = points[i].pt.y / size_scale_ + clip_region_.tl().y;
		float w = points[i].pt.x / size_scale_ + clip_region_.tl().x;
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);

		point_homo = transform_camera_to_originalplane_*point_homo;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
		float radius_in_project = sqrt(point(0)*point(0) + point(1)*point(1));

		float cos_value = original_axis.dot((point - camera_center).normalized());
		float x, y;
		if (point(2) < 0 && cos_value > max_project_angle_cos)
		{
			float radius_in_center_plane = (radius_in_project*point(2))/(-point(2)-f_camera_) + radius_in_project;

			x = point(0) *(radius_in_center_plane / radius_in_project);
			y = point(1)*(radius_in_center_plane / radius_in_project);
		}
		else if (point(2) > 0 && cos_value > max_project_angle_cos)
		{
			float radius_in_center_plane = radius_in_project / (point(2)/f_camera_ + 1);

			x = point(0) *(radius_in_center_plane / radius_in_project);
			y = point(1)*(radius_in_center_plane / radius_in_project);
		}
		else if (point(2) == 0)
		{
			x = point(0);
			y = point(1);
		}
		else
		{
			points_validat[i] = false;
			points_in_pinhole[i] = points[i];
			points_in_pinhole[i].pt.x = 0;
			points_in_pinhole[i].pt.y = 0;
			continue;
		}
		points_in_pinhole[i] = points[i];
		points_in_pinhole[i].pt.x = x*ratio + cx;
		points_in_pinhole[i].pt.y = -y*ratio + cy;
		points_validat[i] = true;
	}
}