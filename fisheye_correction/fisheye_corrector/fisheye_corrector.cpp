#include "fisheye_corrector.h"





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

void FisheyeCorrector::generateMap()
{
	float angle_between_original_axis = radianToDegree(asin(sqrt(pow(sin(axis_vertical_radian_), 2) + pow(sin(axis_horizontal_radian_)*cos(axis_vertical_radian_), 2))));
	float  radius_in_fisheye = distortion_list_[angle_between_original_axis * 10] / pixelHeight_;
	f_image_ = angle_between_original_axis<0.01 ? f_camera_ : radius_in_fisheye / sin(degreeToRadian(angle_between_original_axis));
	Width_ = tan(horizontal_range_)*f_image_ * 2;
	Height_ = tan(vertical_range_)*f_image_ * 2;
	CenterX_ = (float)Width_ / 2.0f;
	CenterY_ = (float)Height_ / 2.0f;
	map_ = cv::Mat(Height_, Width_, CV_32FC2);
	
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
	
	Eigen::Quaternion<float> quaternion_axis(cos(axis_rotation_radian_), 0, 0, sin(axis_rotation_radian_));
	quaternion_axis.normalize();
	Eigen::Matrix3f rotation = quaternion_axis.toRotationMatrix()*quaternion.toRotationMatrix();

	transform_camera_to_world_ <<
		rotation(0, 0), rotation(0, 1), rotation(0, 2), new_camera_plane_center(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), new_camera_plane_center(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), new_camera_plane_center(2),
		0,0,0,1;

	std::cout << "transform_camera_to_world_" << std::endl << transform_camera_to_world_ << std::endl;

	


	for (int h = 0; h < Height_; h++)
		for (int w = 0; w < Width_; w++)
		{
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);
		int pers_h = point_homo(1);
		int pers_w = point_homo(0);

		point_homo = transform_camera_to_world_*point_homo;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
	
		float cos_value = original_axis.dot((point-camera_center).normalized());
		float degree = radianToDegree(acos(cos_value));

		float radius_in_project = sqrt(point(0)*point(0) + point(1)*point(1));
		int position_in_list = round(degree*10);
		float  radius_in_fisheye = distortion_list_[position_in_list] / pixelHeight_;
		float x = ((point(0)) *(radius_in_fisheye / radius_in_project));
		float y = (point(1))*(radius_in_fisheye / radius_in_project);

		//Add the map relationship of Point(h,w)
		map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, -y + CenterY_fisheye_);
		}
	map_.copyTo(original_map_);
}

FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float vertical_range, float horizontal_range)
	:pixelHeight_(pixelHeight), f_camera_(f), horizontal_range_(degreeToRadian(horizontal_range)), vertical_range_(degreeToRadian(vertical_range))
	{

		CenterX_fisheye_ = input_width / 2.0f;
		CenterY_fisheye_ = input_height / 2.0f;
		readDistortionList(correction_table_file);
		std::cout << distortion_list_.size() << std::endl;
		if (f_camera_ < distortion_list_[1] / sin(degreeToRadian(0.1)))
		{
			std::cout << "focal length of camera is too small. Please check if it's correct." << std::endl;
			exit(-1);
		}
		K_ = (cv::Mat_<double>(3, 3) <<
			f_camera_, 0, CenterX_,
			0, f_camera_, CenterY_,
			0, 0, 1
			);

		axis_vertical_radian_ = degreeToRadian(40);
		axis_horizontal_radian_ = degreeToRadian(30);
		axis_rotation_radian_ = degreeToRadian(-5);


		generateMap();
		clip_region_ = cv::Rect(0, 0, Width_, Height_);
		std::cout << "Full size of corrected imageis  width:" << Width_ << " height:" << Height_ << std::endl;
	}




