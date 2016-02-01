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

#define TEST_IMAGE
//#define LAB

#define NO_BOTS 3

#define THRESHOLD 5

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
	
	ros::Subscriber bot_pose_sub[NO_BOTS];

	cv::Mat image;
	
	string bot_pose_topic[NO_BOTS];
	geometry_msgs::Pose2D bot_pose[NO_BOTS];
	
	int target_intensity;
	int min_delta, min_delta_bot;
	
	int text_size_height;
	int iter_count;
	
	Point2f source_point;
	Point2f goal_point;

	#ifdef TEST_IMAGE
		int light_intensity[NO_BOTS];
	#endif
	#ifdef LAB
		ros::Subsriber bot_intensity_sub[NO_BOTS];
	#endif
	bool phase[3];
	bool goal;
	
	public:
	ImageConverter()
		: it_(nh_)
	{
		
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		char temp[25];
		for(int i=0;i<NO_BOTS;i++)
		{	
			sprintf(temp,"/nxt%d/pose",i);
			bot_pose_topic[i]=temp;			
		}
		bot_pose_sub[0]=nh_.subscribe(bot_pose_topic[0],1,&ImageConverter::bot0PoseCb,this);
		bot_pose_sub[1]=nh_.subscribe(bot_pose_topic[1],1,&ImageConverter::bot1PoseCb,this);
		bot_pose_sub[2]=nh_.subscribe(bot_pose_topic[2],1,&ImageConverter::bot2PoseCb,this);
		cv::namedWindow("result", CV_WINDOW_AUTOSIZE);		
		
		#ifdef TEST_IMAGE
			for(int i=0;i<NO_BOTS;i++)
			{	
				sprintf(temp,"bot%d_intensity",i);
				string label=temp;		
				light_intensity[i]=i*10;
				cv::createTrackbar(label, "result", &light_intensity[i], 255, 0);	//Hue varies between 0 and 180
			}
		#endif
		
		#ifdef LAB
			bot_intensity_sub[0]=nh_.subscribe(bot_intensity_topic[i],,100);
		#endif
		target_intensity=40;
		cv::createTrackbar( "Target instensity", "result", &target_intensity, 255, 0);
		
		text_size_height=12;
		iter_count=0;
		phase[0]=true; phase[1]=false; phase[2]=false;
		iter_count=0;		
	}

	~ImageConverter()
	{
		cv::destroyWindow("result");
	}

	void bot0PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[0].x=msg->x;
		bot_pose[0].y=msg->y;
		bot_pose[0].theta=msg->theta;
	}
	void bot1PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[1].x=msg->x;
		bot_pose[1].y=msg->y;
		bot_pose[1].theta=msg->theta;
	}
	void bot2PoseCb(const geometry_msgs::Pose2DConstPtr& msg)
	{
		bot_pose[2].x=msg->x;
		bot_pose[2].y=msg->y;
		bot_pose[2].theta=msg->theta;
	}
	void insertText(Mat img, Point2f target, char text[], Scalar color)
	{
		
		int fontFace = CV_FONT_HERSHEY_PLAIN;
		double fontScale = 1.25;
		int thickness = 1.5;
		int baseline=0;
		Size textSize = getTextSize(text, fontFace,fontScale, thickness, &baseline);
		//baseline += thickness;
		// then put the text itself
		text_size_height=textSize.height;
		putText(img, text, target, fontFace, fontScale, color, thickness, 8);	
	}
	void drawArrow(Mat image, Point2f p, Point2f q, Scalar color, int arrowMagnitude = 9, int thickness=1) 
	{
		//Draw the principle line
		line(image, p, q, color, thickness, CV_AA);
		const double PI = 3.141592653;
		//compute the angle alpha
		double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
		//compute the coordinates of the first segment
		p.x = (q.x+arrowMagnitude*cos(angle+(PI/6)));
		p.y = (q.y+arrowMagnitude*sin(angle+(PI/6)));
		//Draw the first segment
		line(image, p, q, color, thickness, CV_AA);
		//compute the coordinates of the second segment
		p.x = (q.x+arrowMagnitude*cos(angle-(PI/6)));
		p.y = (q.y+arrowMagnitude*sin(angle-(PI/6)));
		//Draw the second segment
		line(image, p, q, color, thickness, CV_AA);
	}
	Point2f vectorAddTranslate(Point 2f p1, Point 2f p2, Point 2f p3, Point 2f p4, Point 2f p5)
	{
		p4=p4-(p3-p2);
		p4=p4-(p1-p5);
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
			image=imread("/home/sunny/Pictures/swarm_edit1.jpg");
		#endif
		
		Mat result=image.clone();
		imshow("source", image);
		//Mark the location of the bots
		for(int i=0;i<NO_BOTS;i++)
		{
			int delta=target_intensity-light_intensity[i];
			circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), 2, Scalar(0,255,0), 2, CV_AA);
			//Label each bot
			char label[30];
			sprintf(label,"B: %d",i);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y), label, Scalar(0,0,255));
			sprintf(label,"D: %d",delta);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y+(text_size_height)+2), label, Scalar(0,0,255));
		}
		//Main logic begins here
		if(phase[0])
		{	
			insertText(result, Point2f(0.0f,(text_size_height)+2), "Phase 0", Scalar(0,0,255));
			min_delta=abs(target_intensity-light_intensity[0]);
			for(int i=0;i<NO_BOTS;i++)
			{
				int delta=target_intensity-light_intensity[i];
				if(abs(delta)<=min_delta)
				{
					min_delta=delta;
					min_delta_bot=i;
				
				}
			}
			source_point=Point2f(bot_pose[min_delta_bot].x, bot_pose[min_delta_bot].y);
			goal_point=source_point;
			for(int i=1;i<NO_BOTS;i++)
			{
				int index=(min_delta_bot+i)%NO_BOTS;
				drawArrow(result, Point2f(bot_pose[index].x, bot_pose[index].y), source_point, Scalar(0,0,0));
			}
			circle(result, Point2f(bot_pose[min_delta_bot].x, bot_pose[min_delta_bot].y), 100, Scalar(0,0,255), 1, CV_AA);
			for(int i=1;i<NO_BOTS;i++)
			{	int index=(min_delta_bot+i)%NO_BOTS;
				goal_point+=Point2f(-1*(bot_pose[index].x),-1*(bot_pose[index].y))+(source_point);
			}
			drawArrow(result, source_point, goal_point, Scalar(0,255,0));
			if((iter_count++)>20)
			{
				phase[0]=false;
				phase[1]=true;
			}
			cout<<iter_count<<endl;
		}
		if(phase[1])
		{
			insertText(result, Point2f(0.0f,(text_size_height)+2), "Phase 1", Scalar(0,0,255));
			for(int i=1;i<NO_BOTS;i++)
			{
				int index=(min_delta_bot+i)%NO_BOTS;
				drawArrow(result, Point2f(bot_pose[index].x, bot_pose[index].y), source_point, Scalar(0,0,0));
			}
			drawArrow(result, source_point, goal_point, Scalar(0,255,0));
			circle(result, Point2f(bot_pose[min_delta_bot].x, bot_pose[min_delta_bot].y), 100, Scalar(0,0,255), 1, CV_AA);
			if((target_intensity-light_intensity[min_delta_bot])<THRESHOLD)
			{
				phase[1]=false;
				phase[2]=true;
				//TODO:calculate intermediate goals
			}	
		}
		if(phase[2])
		{	
			insertText(result, Point2f(0.0f,(text_size_height)+2), "Phase 2", Scalar(0,0,255));
			bool bot_goal_flag[NO_BOTS-1]={false};
			for(int i=1;i<NO_BOTS;i++)	
			{	
				int index=(min_delta_bot+i)%NO_BOTS;
				if(!bot_goal_flag[i])
				{
					if((target_intensity-light_intensity[index])<THRESHOLD)
					{
						bot_goal_flag[i-1]=true;
						continue;
					}
					//TODO:Line following code here.
				}
			}
			source_point=Point2f(bot_pose[min_delta_bot].x, bot_pose[min_delta_bot].y);
			
			//The following code checks if the task is complete by iteratively checking if each robot has reached its goal.
			goal=true;
			for(int i=0;i<(NO_BOTS-1);i++)
			{
				goal&=bot_goal_flag[i];
			}
			if(goal)
			{
				phase[2]=false;
			}
		}
		if(goal)
		{
			insertText(result, Point2f(400.0f,(text_size_height)+2), "GOAL STATE REACHED!", Scalar(0,255,0));
		}
		imshow("result", result);
		cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "glow_planner");
	ImageConverter ic;
	ros::spin();
	return 0;
}
