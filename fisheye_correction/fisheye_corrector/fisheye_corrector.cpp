#include "fisheye_corrector.h"


const float kPI = 3.14159;
inline float degereeToRadian(float d){ return (d / 180.f)* kPI; }
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

void FisheyeCorrector::generateMap()
	{
		map_ = cv::Mat(Height_, Width_, CV_32FC2);

		for (int h = 0; h < Height_; h++)
			for (int w = 0; w < Width_; w++)
			{


			//Transform the points in the corrected image to it's correct position
			cv::Mat point = (cv::Mat_<double>(3, 1) << w, h, 1);
			point = perspectiveTrans_.inv()*point;
			point /= point.at<double>(2, 0);
			int pers_h = point.at<double>(1, 0);
			int pers_w = point.at<double>(0, 0);

			//Using perspectively corrected points to find the correspondences in the fisheye image.
			float radius_in_project = sqrt((pers_h - CenterY_)*(pers_h - CenterY_) + (pers_w - CenterX_)*(pers_w - CenterX_));
			float degree = radianToDegree(atan(radius_in_project / f_)) * 10;

			int position_in_list = round(degree);
			float  radius_in_fisheye = distortion_list_[position_in_list] / pixelHeight_;
			float x = ((pers_w - CenterX_) *(radius_in_fisheye / radius_in_project));/// pow(0.3 / Height_*h + 0.7,2)
			float y = (pers_h - CenterY_)*(radius_in_fisheye / radius_in_project);

			//Add the map relationship of Point(h,w)
			map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + CenterX_fisheye_, y + CenterY_fisheye_);
			}
		map_.copyTo(original_map_);
	}


	FisheyeCorrector::FisheyeCorrector(std::string correction_table_file, int input_height, int input_width, float pixelHeight, float f, float VerticalDegeree, float HorizontalDegree)
		:pixelHeight_(pixelHeight),f_(f), HorizontalDegree_(degereeToRadian(HorizontalDegree)), VerticalDegeree_(degereeToRadian(VerticalDegeree))
	{
		Width_ = tan(HorizontalDegree_)*f_ * 2;
		Height_ = tan(VerticalDegeree_)*f_ * 2;
		CenterX_ = Width_ / 2.0f;
		CenterY_ = Height_ / 2.0f;
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
		std::cout << "corrected image size is  width:" << Width_ << " height:" << Height_ << std::endl;
	}

	void FisheyeCorrector::setPerspectiveTransformation(cv::Mat& perspectiveTrans)
	{
		perspectiveTrans_ = perspectiveTrans;
		generateMap();
		K_  = perspectiveTrans_*K_;
	}


