#include "fisheye_corrector.h"
#include <Eigen\core>
#include <Eigen\geometry>




void FisheyeCorrector::readDistortionList(std::string file_name)
	{
		std::cout << "read distortion list" << std::endl;
		std::ifstream file(file_name);
		distortion_list_.reserve(1035);
		float current_distortion;
		char skip;
		while (file >> current_distortion)
		{
			file >> skip; file >> skip;
			distortion_list_.push_back(current_distortion);
		}
		file.close();
	}

//void FisheyeCorrector::generateMap()
//	{
//		map_ = cv::Mat(Height_, Width_, CV_32FC2);
//		float  radius_in_fisheye = distortion_list_[600] / pixelHeight_;
//		//float x = ((pers_w - CenterX_) *(radius_in_fisheye / (f_camera_ * tan(30))));/// pow(0.3 / Height_*h + 0.7,2)
//		CenterX_fisheye_ += radius_in_fisheye;
//		for (int h = 0; h < Height_; h++)
//			for (int w = 0; w < Width_; w++)
//			{
//			
//
//			//Transform the points in the corrected image to it's correct position
//			cv::Mat point = (cv::Mat_<double>(3, 1) << w, h, 1);
//			point = perspectiveTrans_.inv()*point;
//			point /= point.at<double>(2, 0);
//			int pers_h = point.at<double>(1, 0);
//			int pers_w = point.at<double>(0, 0);
//
//			//Using perspectively corrected points to find the correspondences in the fisheye image.
//			float radius_in_project = sqrt((pers_h - CenterY_)*(pers_h - CenterY_) + (pers_w - CenterX_)*(pers_w - CenterX_));
//			float degree = radianToDegree(atan(radius_in_project / f_camera_)) * 10;
//
//			int position_in_list = round(degree);
//			float  radius_in_fisheye = distortion_list_[position_in_list] / pixelHeight_;
//			float x = ((pers_w - CenterX_) *(radius_in_fisheye / radius_in_project));/// pow(0.3 / Height_*h + 0.7,2)
//			float y = (pers_h - CenterY_)*(radius_in_fisheye / radius_in_project);
//
//			//Add the map relationship of Point(h,w)
//			map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, y + CenterY_fisheye_);
//			}
//		map_.copyTo(original_map_);
//	}
void FisheyeCorrector::generateMap()
{
	map_ = cv::Mat(Height_, Width_, CV_32FC2);
	float angle_between_original_axis = radianToDegree(asin(sqrt(pow(sin(axis_vertical_radian_), 2) + pow(sin(axis_horizontal_radian_)*cos(axis_vertical_radian_), 2))));
	float  radius_in_fisheye = distortion_list_[angle_between_original_axis*10] / pixelHeight_;
	f_image_ = angle_between_original_axis<0.01 ? f_camera_: radius_in_fisheye / sin(degreeToRadian(angle_between_original_axis));
	float trans_y = sin(axis_vertical_radian_)*f_image_;
	float trans_x = cos(axis_vertical_radian_)*f_image_*sin(axis_horizontal_radian_);
	float trans_z = -f_camera_ + f_image_ / cos(degreeToRadian(angle_between_original_axis));
	Eigen::Vector3f new_camera_plane_center(trans_x, trans_y, trans_z);
	Eigen::Vector3f  original_camera_plane_center(0, 0, 0);
	Eigen::Vector3f camera_center(0, 0, -f_camera_);

	Eigen::Quaternion<float> quaternion;
	Eigen::Vector3f original_axis = (original_camera_plane_center - camera_center).normalized();
	Eigen::Vector3f object_axis = (new_camera_plane_center - camera_center).normalized();
	quaternion.setFromTwoVectors(original_axis, object_axis);
	quaternion.normalize();
	Eigen::Matrix3f rotation = quaternion.toRotationMatrix();

	Eigen::Matrix4f transform;
	transform <<
		rotation(0, 0), rotation(0, 1), rotation(0, 2), new_camera_plane_center(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), new_camera_plane_center(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), new_camera_plane_center(2),
		0,0,0,1;

	std::cout << "transform" << std::endl << transform << std::endl;

	


	for (int h = 0; h < Height_; h++)
		for (int w = 0; w < Width_; w++)
		{

		//std::cout << "-----------------------------------------" << std::endl
			//<< "h w " << h << " " << w << std::endl;
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);
		int pers_h = point_homo(1);
		int pers_w = point_homo(0);
		//std::cout << "point_homo " << point_homo << std::endl;
		point_homo = transform*point_homo;
		//std::cout << "transform*point_homo " << point_homo << std::endl;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
		//std::cout << "point " << point << std::endl;
		//Using perspectively corrected points to find the correspondences in the fisheye image.
		//float radius_in_project = sqrt((pers_h - CenterY_)*(pers_h - CenterY_) + (pers_w - CenterX_)*(pers_w - CenterX_));
		//float degree = radianToDegree(atan(radius_in_project / f_camera_)) * 10;
		float cos_value = original_axis.dot((point-camera_center).normalized());
		float degree = radianToDegree(acos(cos_value));
		//std::cout << "degree " << degree << std::endl;
		float radius_in_project = sqrt(point(0)*point(0) + point(1)*point(1));
		int position_in_list = round(degree*10);
		float  radius_in_fisheye = distortion_list_[position_in_list] / pixelHeight_;
		float x = ((point(0)) *(radius_in_fisheye / radius_in_project));/// pow(0.3 / Height_*h + 0.7,2)
		float y = (point(1))*(radius_in_fisheye / radius_in_project);

		//Add the map relationship of Point(h,w)
		map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, -y + CenterY_fisheye_);
		}
	map_.copyTo(original_map_);
}

	FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float VerticalDegeree, float HorizontalDegree)
		:pixelHeight_(pixelHeight),f_camera_(f), horizontal_range_(degreeToRadian(HorizontalDegree)), vertical_range_(degreeToRadian(VerticalDegeree))
	{
		Width_ = tan(horizontal_range_)*f_camera_ * 2;
		Height_ = tan(vertical_range_)*f_camera_ * 2;
		CenterX_ = (float)Width_ / 2.0f;
		CenterY_ = (float)Height_ / 2.0f;
		CenterX_fisheye_ = input_width / 2.0f;
		CenterY_fisheye_ = input_height / 2.0f;
		readDistortionList(correction_table_file);
		std::cout << distortion_list_.size() << std::endl;
		K_ = (cv::Mat_<double>(3, 3) <<
			f_camera_, 0, CenterX_,
			0, f_camera_, CenterY_,
			0, 0, 1
			);

		axis_vertical_radian_ = degreeToRadian(-30);
		axis_horizontal_radian_ = degreeToRadian(0);



		generateMap();
		clip_region_ = cv::Rect(0, 0, Width_, Height_);
		std::cout << "Full size of corrected imageis  width:" << Width_ << " height:" << Height_ << std::endl;
	}




