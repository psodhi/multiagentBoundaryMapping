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
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#define DEBUG

//#define TEST_IMAGE
#define LAB

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
	
	ros::Publisher nxt_command_pub[NO_BOTS];
	ros::Publisher nxt_light_pub[NO_BOTS];

	cv::Mat image;
	
	string bot_pose_topic[NO_BOTS];
	geometry_msgs::Pose2D bot_pose[NO_BOTS];
	
	int target_intensity;
	int min_delta, min_delta_bot;
	
	int text_size_height;
	int iter_count;
	
	Point2f source_point;
	Point2f goal_point[NO_BOTS];

	int light_intensity[NO_BOTS];
	bool light_intensity_flag[NO_BOTS];
	#ifdef LAB
		ros::Subscriber bot_intensity_sub[NO_BOTS];
	#endif
	bool bot_goal[NO_BOTS];
	bool goal;
	float goal_heading[NO_BOTS];
	
	String bot_window[NO_BOTS];
	
	int repelling_radius;
	
	int frame_count;
	
	geometry_msgs::Twist nxt_command[NO_BOTS];
	
	public:
	ImageConverter()
		: it_(nh_)
	{
		
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		char temp[25];
		for(int i=0;i<NO_BOTS;i++)
		{	
			light_intensity_flag[i]=false;
			sprintf(temp,"/nxt%d/pose",i);
			bot_pose_topic[i]=temp;		
			sprintf(temp,"bot%d_plan",i);	
			bot_window[i]=temp;
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
				light_intensity[i]=(i+2)*10;
				cv::createTrackbar(label, "result", &light_intensity[i], 255, 0);	//Hue varies between 0 and 180
			}
		#endif
		
		#ifdef LAB
//			for(int i=0;i<NO_BOTS;i++)
//			{	
//				sprintf(temp,"Bot%dIntensityCb",i);
//				string callback=temp;
//				sprintf(temp,"Bot%dIntensityCb",i);
//				string callback=temp;		
//				
//			}
			bot_intensity_sub[0]=nh_.subscribe("/nxt0/intensity",1,&ImageConverter::bot0IntensityCb,this);
			bot_intensity_sub[1]=nh_.subscribe("/nxt1/intensity",1,&ImageConverter::bot1IntensityCb,this);
			bot_intensity_sub[2]=nh_.subscribe("/nxt2/intensity",1,&ImageConverter::bot2IntensityCb,this);
		#endif
	
		target_intensity=5;
		cv::createTrackbar( "Target Intensity", "result", &target_intensity, 255, 0);
		
		repelling_radius=100;
		cv::createTrackbar( "Repelling Radius", "result", &repelling_radius, 250, 0);
		
		text_size_height=12;
		bot_goal[0]=false; bot_goal[1]=false; bot_goal[2]=false;
		
		frame_count=0;
		
		for(int i=0;i<NO_BOTS;i++)
		{	
			sprintf(temp,"/nxt%d/cmd_vel",i);
			string nxt_command_topic=temp;
			nxt_command_pub[i]=nh_.advertise<geometry_msgs::Twist>(nxt_command_topic,1);
		}
		for(int i=0;i<NO_BOTS;i++)
		{	
			sprintf(temp,"/nxt%d/find_intensity",i);
			string nxt_light_topic=temp;
			nxt_light_pub[i]=nh_.advertise<std_msgs::Empty>(nxt_light_topic,1);
		}
	}

	~ImageConverter()
	{
		cv::destroyWindow("result");
	}
	
	void bot0IntensityCb(const std_msgs::UInt16ConstPtr& msg)
	{
		light_intensity[0]=msg->data;	
		cout<<"light 0:"<<light_intensity[0];
		light_intensity_flag[0]=true;
	}
	
	void bot1IntensityCb(const std_msgs::UInt16ConstPtr& msg)
	{
		light_intensity[1]=msg->data;
		cout<<"light 1:"<<light_intensity[1];
		light_intensity_flag[1]=true;
	}
	
	void bot2IntensityCb(const std_msgs::UInt16ConstPtr& msg)
	{
		light_intensity[2]=msg->data;
		cout<<"light 2:"<<light_intensity[2];
		light_intensity_flag[2]=true;
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
	int randomIndex(int delta[], int no)
	{
		int sum=0;
		for(int i=0;i<no;i++)
			sum+=delta[i];
		int random=(rand() % sum) + 1;
		sum=0;
		for(int i=0;i<no;i++)	
		{
			sum+=delta[i];
			if(random<sum)
				return i;
		}
	}
	//Function to draw an arrow to represent vectors
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
	//Calculates the sum of two vectors (p1p2 and p3p4) and translates the tail of the resultant to point p5;
	Point2f vectorAddTranslate(Point2f p1, Point2f p2, Point2f p3, Point2f p4, Point2f p5)
	{
		p4=p4-(p3-p2);
		p4=p4-(p1-p5);
		return p4;
	}
	//Function to aid in the drawing of the heading vector;
	Point2f headingPointCalc(Point2f bot_pos, float heading)
	{
		//x'=xcos(theta)-ysin(theta)
		//y'=xsin(theta)+ycos(theta)
		Point2f temp; 
		temp.x=50.0f*cosf(heading);
		temp.y=50.0f*sinf(heading);
		temp+=bot_pos;
		return temp; 
	}
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		/*
		if(frame_count<15)
		{
			frame_count++;
			return;
		}
		else
		{
			frame_count=0;
		}
		*/
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
		
		cout<<"Starting"<<endl;
		Mat result=image.clone();
		cv::Mat bot_plan[NO_BOTS];
		imshow("source", image);
		//Mark the location of the bots
		for(int i=0;i<NO_BOTS;i++)
		{	
			bot_plan[i]=image.clone();
			int delta=target_intensity-light_intensity[i];
			//Label each bot
			char label[30];
			sprintf(label,"B: %d",i);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y), label, Scalar(0,0,0));
			sprintf(label,"I: %d",light_intensity[i]);
			insertText(result, Point2f(bot_pose[i].x+5, bot_pose[i].y+(text_size_height)+2), label, Scalar(0,0,0));
		}
		//Main logic begins here
		//TODO:Replace the following with something more scalable
		Scalar bot_color[]={Scalar(0,0,255), Scalar(255,0,0), Scalar(0,255,255)};
		//Core logic begins here.
		
		bool light_initialised;
		for(int i=0;i<NO_BOTS;i++)	//GSO Algorithm starts here. Calculate heading for intermediate goals.
		{
			nxt_light_pub[i].publish(std_msgs::Empty());
			cout<<"publish "<<i<<endl;
		}
		do		//Wait for light intensities to be initialised.
		{
			light_initialised=true;
			for(int i=0;i<NO_BOTS;i++)
			{
				light_initialised&=light_intensity_flag[i];	
			}			
		}while(!light_initialised);
		for(int i=0;i<NO_BOTS;i++)	//Reset new light intensities flag
		{
			light_intensity_flag[i]=false;
		}
		for(int i=0;i<NO_BOTS;i++)	//GSO Algorithm starts here. Calculate heading for intermediate goals.
		{
			Point2f p[4]={Point2f(0.0f,0.0f)};
			int delta_bot=abs(target_intensity-light_intensity[i]);
			if(delta_bot<3)
			{
				bot_goal[i]=true;
			}
			cout<<"----------"<<i<<" delta "<<delta_bot;
			
			bool flag_pull=false,flag_push=false;
			int pull_count=0,push_count=0;
			int pull_delta[NO_BOTS],push_delta[NO_BOTS];
			Point2f pull_bots[NO_BOTS],push_bots[NO_BOTS];
			for(int j=1;j<NO_BOTS;j++)
			{
				int index=(i+j)%NO_BOTS;
				if(light_intensity[i]>light_intensity[index])
				{
					drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), Point2f(bot_pose[index].x, bot_pose[index].y), Scalar(0,0,0));
					#ifdef DEBUG
						drawArrow(bot_plan[i], Point2f(bot_pose[i].x, bot_pose[i].y), Point2f(bot_pose[index].x, bot_pose[index].y), Scalar(0,0,0));
					#endif
					pull_bots[pull_count]=Point2f(bot_pose[index].x, bot_pose[index].y);
					pull_delta[pull_count++]=abs(light_intensity[i]-light_intensity[index]);
					flag_pull=true;
				}
				else
				{
					drawArrow(result, Point2f(bot_pose[index].x, bot_pose[index].y), Point2f(bot_pose[i].x, bot_pose[i].y), Scalar(0,0,0));
					#ifdef DEBUG
						drawArrow(bot_plan[i], Point2f(bot_pose[index].x, bot_pose[index].y), Point2f(bot_pose[i].x, bot_pose[i].y), Scalar(0,0,0));
					#endif
					push_bots[push_count]=Point2f(bot_pose[index].x, bot_pose[index].y);
					push_delta[push_count++]=abs(light_intensity[i]-light_intensity[index]);
					flag_push=true;
				}
			}
			Point2f resultant;
			if(flag_pull)
			{
				p[0]=Point2f(bot_pose[i].x, bot_pose[i].y);
				p[1]=pull_bots[randomIndex(pull_delta,pull_count)];
				resultant=p[1];
			}
			if(flag_push)
			{
				p[2]=push_bots[randomIndex(push_delta,push_count)];
				p[3]=Point2f(bot_pose[i].x, bot_pose[i].y);
				resultant=(p[3]-p[2])+p[3];
			}
			if(flag_pull&flag_push)
			{
				resultant=vectorAddTranslate(p[0],p[1],p[2],p[3],Point2f(bot_pose[i].x, bot_pose[i].y));
			}
			goal_heading[i]=atan2f((resultant.y-bot_pose[i].y),(resultant.x-bot_pose[i].x));
			
		}
		
		bool heading_correct[NO_BOTS]={false},heading_flag=false;
		do //rotate robots until heading error (with respect to intermediate goal) is 0. Then translate one step.
		{		
			for(int i=0;i<NO_BOTS;i++)	 
			{
				//Mark the center of the bot
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), 2, bot_color[i], 2, CV_AA);
				//Draw the heading vector
				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), bot_pose[i].theta), Scalar(0,255,0));
				//Draw the repelling circle
				circle(result, Point2f(bot_pose[i].x, bot_pose[i].y), 100, bot_color[i], 1, CV_AA);
		
				float error_angle=goal_heading[i]-bot_pose[i].theta;
				drawArrow(result, Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), goal_heading[i]), bot_color[i]);
				float error_angle_degrees=error_angle*57.29;
				
				if(abs(error_angle_degrees)>5)
				{
					//Correct heading;
					cout<<"Bot "<<i<<" turn "<<error_angle_degrees<<endl;
					nxt_command[i].linear.x=0.0;
					nxt_command[i].linear.y=0.0;
					nxt_command[i].linear.z=0.0;
					nxt_command[i].angular.x=0.0f;
					nxt_command[i].angular.y=0.0f;
					if(error_angle_degrees<0.0f)
						nxt_command[i].angular.z=(error_angle_degrees/180.0f)-0.5f;
					else
						nxt_command[i].angular.z=(error_angle_degrees/180.0f)+0.5f;
					nxt_command_pub[i].publish(nxt_command[i]);
				}
				else
				{
					heading_correct[i]=true;	
				}
				#ifdef DEBUG
					cout<<i<<" error "<<(error_angle*57.29)<<endl;
					drawArrow(bot_plan[i], Point2f(bot_pose[i].x, bot_pose[i].y), headingPointCalc(Point2f(bot_pose[i].x, bot_pose[i].y), goal_heading[i]), Scalar(0,255,0));
					imshow(bot_window[i],bot_plan[i]);
				#endif	
			}
			heading_flag=true;
			for(int j=0;j<NO_BOTS;j++)	//rotate robots until heading error is 0. Then translate one step. 
			{
				heading_flag&=heading_correct[j];
			}
		}while(!heading_flag);
		
		for(int i=0;i<NO_BOTS;i++)	 //Once heading correct is complete, bots are facing the right direction. Move one step forward
		{
			//Move  one step towards intermediate goal
			cout<<"Bot "<<i<<" move forward"<<endl;
			nxt_command[i].linear.x=1.0;
			nxt_command[i].linear.y=0.0;
			nxt_command[i].linear.z=0.0;
			nxt_command[i].angular.x=0.0f;
			nxt_command[i].angular.y=0.0f;
			nxt_command[i].angular.z=0.0;
			nxt_command_pub[i].publish(nxt_command[i]);
		}						
		goal=bot_goal[0]&bot_goal[1]&bot_goal[2];
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
