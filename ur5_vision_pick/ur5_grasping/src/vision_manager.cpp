/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "ur5_grasping/vision_manager.h"

// VisionManager::VisionManager(float length, float breadth)
// {
// 	this->table_length = length;
// 	this->table_breadth = breadth;
// }

void VisionManager::get2DLocation(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	// cv::Rect tablePos;

	// detectTable(tablePos);

	convertToMM(x, y);
}

void VisionManager::detectTable(cv::Rect &tablePos)
{
	// Extract Table from the image and assign values to pixel_per_mm fields
	cv::Mat BGR[3];
	cv::Mat image = curr_img.clone();
	split(image, BGR);
	cv::Mat gray_image_red = BGR[2];
	cv::Mat gray_image_green = BGR[1];
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			int editValue = binaryImage.at<uchar>(i, j);
			int editValue2 = gray_image_green.at<uchar>(i, j);

			if ((editValue >= 0) && (editValue < 20) && (editValue2 >= 0) && (editValue2 < 20))
			{ // check whether value is within range.
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);

	// Update pixels_per_mm fields
	pixels_permm_y = 1.3415;
	pixels_permm_x = 1.3275;

    tablePos = bbox;

	// Test the conversion values
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;

	// Draw Contours - For Debugging
	std::vector< std::vector <cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	// cv::namedWindow("Table Detection", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Table Detection", image);
	// cv::waitKey(100);
}

void VisionManager::detect2DObject(float &pixel_x, float &pixel_y)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, hsv_image;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR);

	cvCvtColor(image,hsv_image,CV_BGR2HSV);

	int width = hsv_image->width;
	int height = hsv_image->height;
		for (i = 0; i < height; i++)
	  	for (j = 0; j < width; j++)
	        {
	            CvScalar s_hsv_image = cvGet2D(hsv_image, i, j);//获取像素点为（j, i）点的HSV的值
	            /*
	                opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
	                S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
	                V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。
	            */
	            CvScalar s;
	            if (!(v_hsv_image.val[2]>0)&&(v_hsv_image.val[2]<200))
	            {
	                s.val[0] =0;
	                s.val[1]=0;
	                s.val[2]=0;
	                cvSet2D(hsv_image, i ,j, s);
	            }
	        }

	std::vector<vector<Point>>contours;
	std::vector<Vec4i>hierarchy;
	
	cv::findContours(hsv_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::vector<Moments> mu(contours.size() );
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments( contours[i], false );
		if (mu[i].m00>1000)
		cv::drawContours(SrcImage, contours,i,Scalar(255,0,0), CV_FILLED);
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}
}

void VisionManager::convertToMM(float &x, float &y)
{
	// Convert from pixel to world co-ordinates in the camera frame
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}
