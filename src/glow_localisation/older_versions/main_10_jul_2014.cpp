#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#define DEBUG

//#define TEST_IMAGE
#define LAB

#define POSE_IP
//#define POSE_MAGNETO

#define NO_BOTS 3

#include <iostream>
#include <stdio.h>
#include "PID.h"


using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	
	ros::Publisher bot_pose_pub[NO_BOTS];

	cv::Mat image;
	
	//Variables for thresholding
	int min_hue_bot[NO_BOTS];
	int max_hue_bot[NO_BOTS];
	int min_sat_bot[NO_BOTS];
	int max_sat_bot[NO_BOTS];
	int min_val_bot[NO_BOTS];
	int max_val_bot[NO_BOTS];
	
	string bot_window[NO_BOTS];
	string bot_pose_topic[NO_BOTS];	
	
	geometry_msgs::Pose2D bot_pose[NO_BOTS];
	
	int min_contour_area;
	#ifdef POSE_IP
		int pose_ip_threshold;
	#endif
	public:
	ImageConverter()
		: it_(nh_)
	{
		
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		char temp[15];
		for(int i=0;i<NO_BOTS;i++)
		{	
			sprintf(temp,"/nxt%d/pose",i);
			bot_pose_topic[i]=temp;
			bot_pose_pub[i]=nh_.advertise<geometry_msgs::Pose2D>(bot_pose_topic[i],100);
		}
		cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("source", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("bot0_detect", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("bot1_detect", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("bot2_detect", CV_WINDOW_AUTOSIZE);
		
		#ifdef LAB
			//Bot1: red
			min_hue_bot[0]=151;
			max_hue_bot[0]=180;
			min_sat_bot[0]=61;
			max_sat_bot[0]=218;
			min_val_bot[0]=0;
			max_val_bot[0]=255;
		
			//Bot2: blue
			min_hue_bot[1]=103;
			max_hue_bot[1]=144;
			min_sat_bot[1]=123;
			max_sat_bot[1]=208;
			min_val_bot[1]=130;
			max_val_bot[1]=255;
		
			//Bot3: yellow
			min_hue_bot[2]=29;
			max_hue_bot[2]=74;
			min_sat_bot[2]=71;
			max_sat_bot[2]=217;
			min_val_bot[2]=0;
			max_val_bot[2]=255;
		#endif
		
		#ifdef TEST_IMAGE
			//Bot1: red
			min_hue_bot[0]=151;
			max_hue_bot[0]=180;
			min_sat_bot[0]=160;
			max_sat_bot[0]=218;
			min_val_bot[0]=0;
			max_val_bot[0]=255;
	
			//Bot2: blue
			min_hue_bot[1]=91;
			max_hue_bot[1]=110;
			min_sat_bot[1]=149;
			max_sat_bot[1]=237;
			min_val_bot[1]=0;
			max_val_bot[1]=255;
	
			//Bot3: yellow
			min_hue_bot[2]=14;
			max_hue_bot[2]=43;
			min_sat_bot[2]=149;
			max_sat_bot[2]=201;
			min_val_bot[2]=0;
			max_val_bot[2]=255;
		#endif	
		
		for(int i=0;i<NO_BOTS;i++)
		{
			sprintf(temp,"bot%d_detect",i);
			bot_window[i]=temp;
		}
		for(int i=0;i<NO_BOTS;i++)
		{			
			cv::createTrackbar( "Min Hue", bot_window[i], &min_hue_bot[i], 180, 0);	//Hue varies between 0 and 180
			cv::createTrackbar( "Max Hue", bot_window[i], &max_hue_bot[i], 180, 0);
			cv::createTrackbar( "Min Saturation", bot_window[i], &min_sat_bot[i], 255, 0);
			cv::createTrackbar( "Max Saturation", bot_window[i], &max_sat_bot[i], 255, 0);
			cv::createTrackbar( "Min Value", bot_window[i], &min_val_bot[i], 255, 0);
			cv::createTrackbar( "Max Value", bot_window[i], &max_val_bot[i], 255, 0);
		}
		
		min_contour_area=832;
		cv::createTrackbar( "Minimum contour area", "result", &min_contour_area, 3000, 0);
		
		#ifdef POSE_IP
			pose_ip_threshold=1554;
			cv::createTrackbar( "Pose IP threshold", "result", &pose_ip_threshold, 6000, 0);
		#endif
	}

	~ImageConverter()
	{
		cv::destroyWindow("result");
		cv::destroyWindow("source");
		cv::destroyWindow("bot0_detect");
		cv::destroyWindow("bot1_detect");
		cv::destroyWindow("bot2_detect");
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//Get a pointer to the image
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		#ifdef LAB
			image=cv_ptr->image;
		#endif
		#ifdef TEST_IMAGE 	
			image=imread("/home/sunny/Pictures/swarm_edit.jpg");
		#endif
		
		Mat result=image.clone();
		imshow("source", image);
		GaussianBlur( image, image, Size( 3, 3 ), 0, 0 );
		cv::cvtColor(image, image, CV_BGR2HSV);		

		Mat bot_mask[3];
		for(int i=0;i<NO_BOTS;i++)
		{
			bot_mask[i] = Mat::zeros(image.size(), CV_8UC3);
			inRange(image, cv::Scalar(min_hue_bot[i], min_sat_bot[i], min_val_bot[i]), cv::Scalar(max_hue_bot[i], max_sat_bot[i], max_val_bot[i]), bot_mask[i]);
			dilate(bot_mask[i], bot_mask[i], Mat(Size(8, 8), CV_8UC1)); // Erode with a 30 x 30 kernel
			erode(bot_mask[i], bot_mask[i], Mat(Size(8, 8), CV_8UC1)); // Erode with a 30 x 30 kernel
			imshow(bot_window[i], bot_mask[i]);
			
			vector<vector<Point> > contours;
			findContours(bot_mask[i], contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
			#ifdef POSE_IP
				Point2f bot_front, bot_rear;
			#endif
			for(size_t j = 0; j < contours.size(); j++)
			{
				size_t count = contours[j].size();
				if( count < 6 )
					continue;
				//Reject contours with area lower than threshold
				if(contourArea(contours[j])<min_contour_area)
					continue;
				Mat pointsf,matellipse;
				Mat(contours[j]).convertTo(pointsf, CV_32F);
				RotatedRect box = fitEllipse(pointsf);
				Point2f vtx[4];
				box.points(vtx);
				for(int k=0;k<4;k++)
					line(result, vtx[(k%4)], vtx[((k+1)%4)], Scalar(0,255,0), 4, CV_AA);
				#ifdef POSE_IP
					if (contourArea(contours[j])<pose_ip_threshold)
					{
						bot_front.x=(float)box.center.x;
						bot_front.y=(float)box.center.y;
					}
					else
					{
						bot_rear.x=(float)box.center.x;
						bot_rear.y=(float)box.center.y;						
					}
				#endif
				#ifdef POSE_MAGNETO
				{	
					bot_pose[i].x=(float)box.center.x;
					bot_pose[i].y=(float)box.center.y;
					bot_pose[i].theta=0.0f;
				}
				#endif
				circle(result, box.center, 2, Scalar(0,255,255), 2, CV_AA);
			}
			#ifdef POSE_IP
				bot_pose[i].x=(float)((bot_front.x+bot_rear.x)/2.0f);
				bot_pose[i].y=(float)((bot_front.y+bot_rear.y)/2.0f);		
				bot_pose[i].theta=atan2f((bot_front.y-bot_rear.y),(bot_front.x-bot_rear.x));
//				cout<<i<<" "<<(bot_pose[i].theta*57.29f)<<endl;
				circle(result, Point2f(bot_pose[i].x,bot_pose[i].y), 2, Scalar(0,0,255), 2, CV_AA);
			#endif
			bot_pose_pub[i].publish(bot_pose[i]);
				
		}
	
		imshow("result", result);
		cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "glow_localisation");
	ImageConverter ic;
	ros::spin();
	return 0;
}
