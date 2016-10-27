#include "find_perspective_transform.h"

std::vector<cv::Point2f> original_points;
std::vector<cv::Point2f> corrected_points;
cv::Mat corrected_frame;

void on_mouse(int event, int x, int y, int flags, void* ustc);

void findPerspectiveDistortion(cv::Mat& frame, FisheyeCorrector& corrector)
{

	cv::namedWindow("frame", 0);
	cv::namedWindow("corrected", 0);
	
	cvSetMouseCallback("corrected", on_mouse, 0);

	
	corrector.correct(frame, corrected_frame);

	cv::imshow("frame", frame);
	cv::imshow("corrected", corrected_frame);

	std::cout << "Please select a rectangle from corrected image in clockwise direction begin with the left bottom corner " << std::endl;
	
	cv::waitKey(0);
	
	corrector.setPerspectiveTransformation(cv::getPerspectiveTransform(original_points, corrected_points));
	
	
	corrector.correct(frame, corrected_frame);
	cv::namedWindow("corrected image without perspective distortion", 0);
	cv::imshow("corrected image without perspective distortion", corrected_frame);


	std::cout << "save transformation to perspectiveTrans.txt" << std::endl;
	std::ofstream pers_file("perspectiveTrans.txt");
	std::cout << corrector.getPerspectiveTransformation() << std::endl;
	pers_file << corrector.getPerspectiveTransformation();
	pers_file.flush(); pers_file.close();
	std::cout << "Press any key to continue" << std::endl;
	cv::waitKey(0);
	cv::destroyWindow("corrected image without perspective distortion");
}





void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
	static int count = 0;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		if (count < 4)//Get the corners of the rectangle
		{
			cv::Point2f pt = cv::Point2f(x, y);
			char temp[16];
			sprintf(temp, "(%d,%d)", pt.x, pt.y);
			cv::circle(corrected_frame, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
			cv::imshow("corrected", corrected_frame);
			original_points.push_back(pt);
			cv::waitKey(1);
		}


		if (count == 3)//Generate the undistorted rectangle
		{
			float left_x = original_points[0].x;
			float right_x = original_points[3].x;
			float upper_y = original_points[1].y;
			float bottom_y = original_points[0].y;
			corrected_points = original_points;
			corrected_points[1].x = left_x;
			corrected_points[2].x = right_x;
			corrected_points[2].y = upper_y;
			corrected_points[3].y = bottom_y;
			for (int i = 0; i < 4; i++)
			{
				cv::circle(corrected_frame, corrected_points[i], 2, cvScalar(0, 0, 255, 0), CV_FILLED, CV_AA, 0);
				cv::imshow("corrected", corrected_frame);
			}
			cv::waitKey(1);
			std::cout << "press any key to continue" << std::endl;
		}
		count++;
	}
	
}
