#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

const float kPI = 3.14159;
inline float degereeToRadian(float d){ return (d / 180.f)* kPI; }
inline float radianToDegree(float r){ return (r / kPI)* 180.f; }

const float focal = 150;//Measured in pixels

cv::Mat frame, correct_frame;
std::vector<cv::Point2f> original_points;
std::vector<cv::Point2f> correct_points;
cv::Mat perspectiveTrans;


class FisheyeCorrecter
{
	std::vector<float> distortion_list_;
	cv::Mat map_;
	const float f = focal;
	const float kVerticalDegeree_ = degereeToRadian(60);
	const float kHorizontalDegree_ = degereeToRadian(80);
	const int kWidth_ = tan(kHorizontalDegree_)*f * 2;
	const int kHeight_ = tan(kVerticalDegeree_)*f * 2;
	const float kCenterX_ = kWidth_ / 2.0f;
	const float kCenterY_ = kHeight_ / 2.0f;
	float kCenterX_fisheye;
	float kCenterY_fisheye;

private:
	void readDistortionList(std::string file_name)
	{
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

	void generateMap()
	{
		map_ = cv::Mat(kHeight_, kWidth_, CV_32FC2);

		for (int h = 0; h < kHeight_; h++)
			for (int w = 0; w < kWidth_; w++)
			{
			cv::Mat point(1, 1, CV_64FC2);
			point.at<cv::Vec2d>(0,0) = cv::Vec2d(w,h);
			//std::cout << point << std::endl;
			//point = perspectiveTrans.inv()*point;
			cv::perspectiveTransform(point, point, perspectiveTrans.inv());
			int pers_h = point.at<cv::Vec2d>(0,0)(1);
			int pers_w = point.at<cv::Vec2d>(0, 0)(0);
			/*if (pers_h>kHeight_ || pers_h<0 || pers_w>kWidth_ || pers_w < 0)
				continue;*/
			//std::cout << point << std::endl;
			float radius_in_project = sqrt((pers_h - kCenterY_)*(pers_h - kCenterY_) + (pers_w - kCenterX_)*(pers_w - kCenterX_));
			float degree = radianToDegree(atan(radius_in_project / f)) * 10;
			//int position_in_list = round(degree) + 20 > distortion_list_.size() ? distortion_list_.size() : round(degree) + 20;
			//int position_in_list = round(degree) - 50 > 0 ? round(degree) - 50 : 0;
			int position_in_list = round(degree);
			float  radius_in_fisheye = distortion_list_[position_in_list] / 0.0042;
			float x = ((pers_w - kCenterX_) *(radius_in_fisheye / radius_in_project));/// pow(0.3 / kHeight_*h + 0.7,2)
			float y = (pers_h - kCenterY_)*(radius_in_fisheye / radius_in_project);
			//x = (x*projective_trans.at<float>(0, 0) + y*projective_trans.at<float>(0, 1) + projective_trans.at<float>(0, 2)) / (projective_trans.at<float>(2, 0)*x + projective_trans.at<float>(2, 1)*y + projective_trans.at<float>(2, 2));
			//y = (x*projective_trans.at<float>(1, 0) + y*projective_trans.at<float>(1, 1) + projective_trans.at<float>(1, 2)) / (projective_trans.at<float>(2, 0)*x + projective_trans.at<float>(2, 1)*y + projective_trans.at<float>(2, 2));
			
			map_.at<cv::Vec2f>(h, w) = cv::Vec2f(x + kCenterX_fisheye, y + kCenterY_fisheye);
			}
	}


public:

	FisheyeCorrecter(std::string file_name, int input_height, int input_width)
	{
		kCenterX_fisheye = input_width / 2.0f;
		kCenterY_fisheye = input_height / 2.0f;
		readDistortionList(file_name);
		generateMap();
		std::cout << "width:" << kWidth_ << " height:" << kHeight_ << std::endl;
	}



	cv::Mat& correct(const cv::Mat& src, cv::Mat& dst)
	{
		cv::remap(src, dst, map_, cv::Mat(), cv::INTER_CUBIC);
		return dst;
	}

	cv::Size getCorrectSize()
	{
		return cv::Size(kWidth_, kHeight_);
	}
};



void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		cv::Point2f pt = cv::Point2f(x, y);
		char temp[16];
		sprintf(temp, "(%d,%d)", pt.x, pt.y);
		cv::circle(correct_frame, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cv::imshow("correct", correct_frame);
		original_points.push_back(pt);
		cv::waitKey(1);
	}
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		//*****Manually select the correct rectangular points.
		/*cv::Point2f pt = cv::Point2f(x, y);
		char temp[16];
		sprintf(temp, "(%d,%d)", pt.x, pt.y);
		cv::circle(correct_frame, pt, 2, cvScalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
		cv::imshow("correct", correct_frame);
		correct_points.push_back(pt);
		cv::waitKey(1);*/

		//*****Rectify the original points under assumption that the original points should be a rectangle.
		float left_x = original_points[0].x;
		float right_x = original_points[3].x;
		float upper_y = original_points[1].y;
		float bottom_y = original_points[0].y;
		correct_points = original_points;
		correct_points[1].x = left_x;
		correct_points[2].x = right_x;
		/*correct_points[2].y = upper_y;
		correct_points[3].y = bottom_y;*/
		for (int i = 0; i < 4; i++)
		{
			cv::circle(correct_frame, correct_points[i], 2, cvScalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
			cv::imshow("correct", correct_frame);
		}
		cv::waitKey(1);
	}

}





int main(int argc, char* argv[])
{
	std::string fileName = argv[2];
	cv::VideoCapture capture(fileName);
	perspectiveTrans = cv::Mat::eye(3, 3, CV_64F);


	if (capture.isOpened())
		capture >> frame;

	FisheyeCorrecter correcter_nopers(argv[1], capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH));

	std::string correct_file_name = "correct_";
	int file_name_pos = fileName.find_last_of('\\');
	if (file_name_pos != -1)
		correct_file_name += fileName.substr(file_name_pos);
	else
		correct_file_name += fileName;

	cv::namedWindow("frame", 0);
	cv::namedWindow("correct", 0);
	cvSetMouseCallback("correct", on_mouse, 0);
	cv::imshow("frame", frame);
	correcter_nopers.correct(frame, correct_frame);
	cv::imshow("correct", correct_frame);
	cv::waitKey(0);
	perspectiveTrans = cv::getPerspectiveTransform(original_points, correct_points);
	std::ofstream pers_file("perspectiveTrans.txt");
	std::cout << perspectiveTrans << std::endl;
	pers_file << perspectiveTrans;
	pers_file.flush(); pers_file.close();

	FisheyeCorrecter correcter(argv[1], capture.get(CV_CAP_PROP_FRAME_HEIGHT), capture.get(CV_CAP_PROP_FRAME_WIDTH));
	cv::VideoWriter writer(correct_file_name, capture.get(CV_CAP_PROP_FOURCC), capture.get(CV_CAP_PROP_FPS), correcter.getCorrectSize());
	while (!frame.empty())
	{
		correcter.correct(frame, correct_frame);

		cv::imshow("frame", frame);
		cv::imshow("correct", correct_frame);

		writer << correct_frame;
		//std::cout << "correct"<<std::endl;
		//cv::waitKey(0);
		//
		cv::Mat persImage;
		//cv::perspectiveTransform(correct_frame, persImage, perspectiveTrans);
		//cv::warpPerspective(correct_frame, persImage, perspectiveTrans,correct_frame.size());
		//cv::imshow("pers", persImage);
		//std::cout << "perspective" << std::endl << perspectiveTrans << std::endl;
		cv::waitKey(10);
		capture >> frame;
	}
	writer.release();
}

