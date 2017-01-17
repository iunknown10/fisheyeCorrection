#include "fisheye_corrector.h"





void FisheyeCorrector::readDistortionList(std::string file_name)
	{
		std::cout << "read distortion list" << std::endl;
		std::ifstream file(file_name);
		if (!file.is_open())
		{
			std::cout << "open file error";
			exit(-1);
		}
			
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
	Width_ = tan(horizontal_range_radian_)*f_image_ * 2;
	Height_ = tan(vertical_range_radian_)*f_image_ * 2;
	CenterX_ = (float)Width_ / 2.0f;
	CenterY_ = (float)Height_ / 2.0f;
	map_ = cv::Mat::ones(Height_, Width_, CV_32FC2)*(-1);
	
	float trans_y = sin(axis_vertical_radian_)*f_image_;
	float trans_x = cos(axis_vertical_radian_)*f_image_*sin(axis_horizontal_radian_);
	float trans_z = -f_camera_ + f_image_ * cos(degreeToRadian(angle_between_original_axis));
	new_camera_plane_center = Eigen::Vector3f(trans_x, trans_y, trans_z);
	Eigen::Vector3f  original_camera_plane_center(0, 0, 0);
	camera_center = Eigen::Vector3f(0, 0, -f_camera_);

	Eigen::Quaternion<float> quaternion;
	original_axis = Eigen::Vector3f(original_camera_plane_center - camera_center).normalized();
	Eigen::Vector3f object_axis = (new_camera_plane_center - camera_center).normalized();
	quaternion.setFromTwoVectors(original_axis, object_axis);
	quaternion.normalize();
	
	Eigen::Quaternion<float> quaternion_axis(cos(axis_rotation_radian_), 0, 0, sin(axis_rotation_radian_));
	quaternion_axis.normalize();
	Eigen::Matrix3f rotation = quaternion_axis.toRotationMatrix()*quaternion.toRotationMatrix();
	transform_camera_to_originalplane_ = Eigen::Matrix4f();
	transform_camera_to_originalplane_ <<
		rotation(0, 0), rotation(0, 1), rotation(0, 2), new_camera_plane_center(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), new_camera_plane_center(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), new_camera_plane_center(2),
		0,0,0,1;

	//std::cout << "transform_camera_to_originalplane_" << std::endl << transform_camera_to_originalplane_ << std::endl;

	for (int h = 0; h < Height_; h++)
		for (int w = 0; w < Width_; w++)
		{
		//Transform the points in the corrected image to it's correct position
		Eigen::Vector4f point_homo(w - CenterX_, -h + CenterY_, 0, 1);

		point_homo = transform_camera_to_originalplane_*point_homo;
		Eigen::Vector3f  point(point_homo(0), point_homo(1), point_homo(2));
		//std::cout << "point " << point.transpose() << std::endl;
		//Eigen::Vector3f point_vector = (point - camera_center).normalized();

		float cos_value = original_axis.dot((point-camera_center).normalized());
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

		//Add the map relationship of Point(h,w)
		//std::cout << "x " << x << "   y " << y << std::endl;
		map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, -y + CenterY_fisheye_);
		}
	map_.copyTo(original_map_);
}



FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float vertical_range, float horizontal_range)
	:pixelHeight_(pixelHeight), f_camera_(f), horizontal_range_radian_(degreeToRadian(horizontal_range)), vertical_range_radian_(degreeToRadian(vertical_range))
	{
		size_scale_ = 1;
		CenterX_fisheye_ = input_width / 2.0f;
		CenterY_fisheye_ = input_height / 2.0f;
		readDistortionList(correction_table_file);
		std::cout << distortion_list_.size() << std::endl;
		if (f_camera_ < distortion_list_[1] / sin(degreeToRadian(0.1)))
		{
			std::cout << "focal length of camera is too small. Please check if it's correct." << std::endl;
			exit(-1);
		}
		

		axis_vertical_radian_ = 0;
		axis_horizontal_radian_ = 0;
		axis_rotation_radian_ = 0;


		
		clip_region_ = cv::Rect(0, 0, 0, 0);
		map_need_update = true;
		new_camera_plane_center = Eigen::Vector3f(0,0,0);
		camera_center = Eigen::Vector3f(0,0,0);
		original_axis = Eigen::Vector3f(0,0,0);

		transform_camera_to_originalplane_ = Eigen::Matrix4f();

	}




