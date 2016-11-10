#include "correctTools.h"
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\contrib\retina.hpp>

int ORBtest(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "need correction table file and video name for parameters" << std::endl;
	}
	cv::namedWindow("image_front_c", 0);
	cv::namedWindow("image_right_c", 0);

	std::string correction_table = argv[1];
	std::string path = argv[2];

	cv::VideoCapture capture_front("front_outdoor.avi");
	cv::VideoCapture capture_right("right_outdoor.avi");

	
	cv::Mat image_front;
	cv::Mat image_right;


	FisheyeCorrector corrector(correction_table, capture_front.get(CV_CAP_PROP_FRAME_HEIGHT), capture_front.get(CV_CAP_PROP_FRAME_WIDTH));
	cv::Mat perspectiveTransform = (cv::Mat_<double>(3, 3) <<
		0.7120342503081317, -0.9080554014444526, 98.72580114592542,
		-0.03621256531992459, 0.7347221335742586, 15.81086229667521,
		-0.0001161970505587349, -0.002209380538794291, 1
		);
	corrector.setPerspectiveTransformation(perspectiveTransform);

	while (1)
	{
		capture_front >> image_front;
		capture_right >> image_right;
		cv::imshow("image_front_c", image_front);
		cv::imshow("image_right_c", image_right);
	cv::Mat image_front_c, image_right_c;
	cv::Mat front_corrected, right_corrected;
	corrector.correct(image_front, front_corrected);
	corrector.correct(image_right, right_corrected);



	//front_corrected(cv::Rect(front_corrected.cols / 2, 0, front_corrected.cols / 2, front_corrected.rows)).copyTo(image_front_c);
	//right_corrected(cv::Rect(0, 0, front_corrected.cols / 2, front_corrected.rows)).copyTo(image_right_c);
	image_front_c = front_corrected;
	image_right_c = right_corrected;
	cv::imshow("image_front_c", image_front_c);
	cv::imshow("image_right_c", image_right_c);
	cv::waitKey(10);

	cv::ORB orb(100);
	std::vector<cv::KeyPoint> kp_front, kp_right;
	cv::Mat des_front, des_right;
	orb.detect(image_front_c, kp_front);
	orb.detect(image_right_c, kp_right);
	orb.compute(image_front_c, kp_front, des_front);
	orb.compute(image_right_c, kp_right, des_right);

	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector<cv::DMatch> matches;
	matcher.match(des_front, des_right, matches);

	// drawing the results
	cv::namedWindow("matches", 0);
	cv::Mat img_matches;
	drawMatches(image_front_c, kp_front, image_right_c, kp_right, matches, img_matches);
	imshow("matches", img_matches);
	cv::waitKey(0);
	}



}