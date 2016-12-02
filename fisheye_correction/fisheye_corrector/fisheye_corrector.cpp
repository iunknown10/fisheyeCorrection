#include "fisheye_corrector.h"
#include <Eigen\core>
#include <Eigen\geometry>

const float kPI = 3.14159;
inline float degreeToRadian(float d){ return (d / 180.f)* kPI; }
inline float radianToDegree(float r){ return (r / kPI)* 180.f; }



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
//		//float x = ((pers_w - CenterX_) *(radius_in_fisheye / (f_ * tan(30))));/// pow(0.3 / Height_*h + 0.7,2)
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
//			float degree = radianToDegree(atan(radius_in_project / f_)) * 10;
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
	float start_point = 0;
	float  radius_in_fisheye = distortion_list_[start_point*10] / pixelHeight_;
	//float x = ((pers_w - CenterX_) *(radius_in_fisheye / (f_ * tan(30))));/// pow(0.3 / Height_*h + 0.7,2)
	//CenterX_fisheye_ += radius_in_fisheye;
	float trans_value = -f_+(radius_in_fisheye / tan(degreeToRadian(start_point)));
	float focal = f_ - (radius_in_fisheye / sin(degreeToRadian(start_point)));
	Eigen::Quaternion<float> quaternion(degreeToRadian(start_point), 0.0f, -f_, 0.0f);
	Eigen::Vector3f translation(radius_in_fisheye, 0, trans_value);
		Eigen::Matrix3f rotation = quaternion.toRotationMatrix();
	cv::Mat transform = (cv::Mat_<float>(4, 4) <<
		rotation(0, 0), rotation(0, 1), rotation(0, 2), translation(0),
		rotation(1, 0), rotation(1, 1), rotation(1, 2), translation(1),
		rotation(2, 0), rotation(2, 1), rotation(2, 2), translation(2)
		);

	cv::Mat camera_center = (cv::Mat_<float>(3, 1) <<
		0, 0, -focal
		);
	
	cv::Mat old_plane_center = (cv::Mat_<float>(3, 1) <<
		-translation(0), -translation(1), -translation(2)
		);

	for (int h = 0; h < Height_; h++)
		for (int w = 0; w < Width_; w++)
		{


		//Transform the points in the corrected image to it's correct position
		cv::Mat point = (cv::Mat_<float>(3, 1) << w-CenterX_, h-CenterY_, 0);
		int pers_h = point.at<float>(1, 0);
		int pers_w = point.at<float>(0, 0);

		//Using perspectively corrected points to find the correspondences in the fisheye image.
		//float radius_in_project = sqrt((pers_h - CenterY_)*(pers_h - CenterY_) + (pers_w - CenterX_)*(pers_w - CenterX_));
		//float degree = radianToDegree(atan(radius_in_project / f_)) * 10;
		float cos_value = ((old_plane_center - camera_center).dot(point - camera_center)) / (cv::norm(old_plane_center - camera_center)*cv::norm(point - camera_center));
		float degree = acos(cos_value);
		point = (cv::Mat_<float>(4, 1) << w - CenterX_, h - CenterY_, 0,1);
		point = transform*point;
		float radius_in_project = sqrt(point.at<float>(0)*point.at<float>(0) + point.at<float>(1)*point.at<float>(1));
		int position_in_list = round(degree);
		float  radius_in_fisheye = distortion_list_[position_in_list] / pixelHeight_;
		float x = ((point.at<float>(0)) *(radius_in_fisheye / radius_in_project));/// pow(0.3 / Height_*h + 0.7,2)
		float y = (point.at<float>(1))*(radius_in_fisheye / radius_in_project);

		//Add the map relationship of Point(h,w)
		map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, y + CenterY_fisheye_);
		}
	map_.copyTo(original_map_);
}

	FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float VerticalDegeree, float HorizontalDegree)
		:pixelHeight_(pixelHeight),f_(f), HorizontalDegree_(degreeToRadian(HorizontalDegree)), VerticalDegeree_(degreeToRadian(VerticalDegeree))
	{
		Width_ = tan(HorizontalDegree_)*f_ * 2;
		Height_ = tan(VerticalDegeree_)*f_ * 2;
		CenterX_ = (float)Width_ / 2.0f;
		CenterY_ = (float)Height_ / 2.0f;
		CenterX_fisheye_ = input_width / 2.0f;
		CenterY_fisheye_ = input_height / 2.0f;
		readDistortionList(correction_table_file);
		std::cout << distortion_list_.size() << std::endl;
		perspectiveTrans_ = cv::Mat::eye(3, 3, CV_64F);
		K_ = (cv::Mat_<double>(3, 3) <<
			f_, 0, CenterX_,
			0, f_, CenterY_,
			0, 0, 1
			);
		generateMap();
		clip_region_ = cv::Rect(0, 0, Width_, Height_);
		std::cout << "Full size of corrected imageis  width:" << Width_ << " height:" << Height_ << std::endl;
	}

	void FisheyeCorrector::setPerspectiveTransformation(cv::Mat& perspectiveTrans)
	{
		perspectiveTrans_ = perspectiveTrans;
		generateMap();
		K_  = perspectiveTrans_*K_;
	}


