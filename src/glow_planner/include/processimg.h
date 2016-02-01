/*
TODO:
Preamble:
Add brief problem definition.
Edit: add reasons for creating this code and reasons for shifting to the new version.
Clean up code.
*/

/*
Title: Hoop detection and command generation using PID
Author: Achal D Arvind
Date created: 5th Oct 2012
*/

#include "PID.h"
#include "cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;

int sliderPos = 90;
bool flag=false;
Mat image;

//Class for hole detection and command generation
class HDCG
{
	void processImage(int, void*);

	int main( int argc, char** argv )
	{	
		namedWindow("result", 1);
		//If no arguments passed, defualt image for processing is sample3.jpg
		const char* filename = argc == 2 ? argv[1] : (char*)"sample3.jpg";
		image
		if( image.empty() )
		{
		cout << "Couldn't open image " << filename << "\nUsage: fitellipse <image_name>\n";
		return 0;
		}
		//Show unprocessed image
		imshow("source", image);
		 
		// Create toolbars. HighGUI use.
		createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
		processImage(0, 0);

		// Wait for a key stroke; the same function arranges events processing
		waitKey();
		return 0;
	}

	// Define trackbar callback functon. This function find contours,
	// draw it and approximate it by ellipses.
	void processImage(int /*h*/, void*)
	{
		vector<vector<Point> > contours;
		Mat bimage = image >= sliderPos;
		findContours(bimage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		Mat cimage = Mat::zeros(bimage.size(), CV_8UC3);
		for(size_t i = 0; i < contours.size(); i++)
		{
			size_t count = contours[i].size();
			if( count < 6 )
				continue;     
			Mat pointsf,matellipse;
			Mat(contours[i]).convertTo(pointsf, CV_32F);
			RotatedRect box = fitEllipse(pointsf);  
			if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
				continue;
			drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);
		
			//ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
			cout<<"y: "<<cimage.rows<<endl<<"x: "<<cimage.cols<<endl;        
		
			//Obstacle growing to reduce chance of collision and to ensure that the ellipse is fit entirely into detected shape
			Point2f vtx[4];
			if(box.size.width<cimage.cols&&box.size.height<cimage.cols)
			{
				box.size.width=box.size.width*0.725f;
				box.size.height=box.size.height*0.725f;  
			}
			box.points(vtx);
			Point2f mid[4];
		
			//Eliminating noise
			//Removing contours that are outside frame by more than 10 pixels
			for(int i=0;i<4;i++)
				if((vtx[i].x>(cimage.cols+10))||(vtx[i].y>(cimage.rows+10))||(vtx[i].x<(-10))||(vtx[i].x<(-10)))
					flag=true;
		
			//Not processing contours that dont match criteria in terms of size
			if(!flag&&box.size.width>(0.125*cimage.cols)&&box.size.height>(0.125*cimage.rows))
			{
				ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,0,255), 1, CV_AA);
				for( int j = 0; j < 4; j++ )
				{
					//line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);
					mid[j].x=(vtx[j].x+ vtx[(j+1)%4].x)/2;
					mid[j].y=(vtx[j].y+ vtx[(j+1)%4].y)/2;	    	
					cout<<"Points"<<endl;
					cout<<vtx[j].x<<" "<<vtx[j].y<<endl;
					cout<<vtx[(j+1)%4].x<<" "<<vtx[(j+1)%4].y<<endl;
					cout<<"Mid-point"<<endl;
					cout<<mid[j].x<<" "<<mid[j].y<<endl;
				}
				    		
				cout<<"Points"<<endl;
				cout<<vtx[0].x<<" "<<vtx[0].y<<endl;
				cout<<vtx[2].x<<" "<<vtx[2].y<<endl;
				cout<<"Mid-point"<<endl;
				cout<<mid[0].x<<" "<<mid[0].y<<endl;
			
				//Draw minor axis
				line(cimage, mid[0], mid[2], Scalar(255,0,0), 1, CV_AA);
			
				//Draw major axis
				line(cimage, mid[1], mid[3], Scalar(255,0,0), 1, CV_AA);
			
				//Draw circles to represent end points of major and minor axes
				for(int j=0;j<4;j++)
					circle( cimage, mid[j], 2, Scalar(0,0,255), 2, CV_AA);
			
				//Calculation of foci
				//Calculate length of major axis
				int lenmajor=(sqrt(pow((mid[1].x-mid[3].x),2)+pow((mid[1].y-mid[3].y),2)))/2;
			
				//Calculate length of minor axis
				int lenminor=(sqrt(pow((mid[0].x-mid[2].x),2)+pow((mid[0].y-mid[2].y),2)))/2;
			
				//Calculate distange of foci from center
				int focid=sqrt(pow(lenmajor,2)-pow(lenminor,2));
			
				//Calculate slope of major axis
				float slope=((float)(mid[3].y-mid[1].y))/(mid[3].x-mid[1].x);
			
				//Place foci on image
				Point2f ellipsecenter,foci1,foci2;
				ellipsecenter.x=(mid[3].x+mid[1].x)/2;
				ellipsecenter.y=(mid[3].y+mid[1].y)/2;
				circle( cimage, ellipsecenter, 3, Scalar(0,255,0), 4, CV_AA);
				int offsetx,offsety;
			
				//57.32 conversion factor from degrees to radians 
				//90 as the box angle gives the angle for the minor axis
				offsety=(sin(((box.angle+90)/57.32f))*focid);
				offsetx=(cos(((box.angle+90)/57.32f))*focid);
				foci1.x=ellipsecenter.x+offsetx;
				foci1.y=ellipsecenter.y+offsety;
				foci2.x=ellipsecenter.x-offsetx;
				foci2.y=ellipsecenter.y-offsety;
				circle( cimage, foci1, 3, Scalar(0,255,0), 2, CV_AA);
				circle( cimage, foci2, 3, Scalar(0,255,0), 2, CV_AA);
			}
			flag=false;
		}
		imshow("result", cimage);
	}
}
