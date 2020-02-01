/************************************************************ 
|                                                           |
| NEXUS 2020 Code v1.0                                      |
| Made by Team  TM190F7D                                    |
|                                                           |
************************************************************/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

int fd;
void settings(const char *abc)
{
      fd = open(abc,O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
      usleep(3500000);
                                    /* O_RDWR Read/Write access to serial port           */
                                    /* O_NOCTTY - No terminal will control the process   */
                                    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                    /* -the status of DCD line,Open() returns immediatly */                                        
                                    
            if(fd == -1)                        /* Error Checking */
                   printf("\n  Error! in Opening ttyUSB0  ");
            else
                   printf("\n  ttyUSB0 Opened Successfully ");
       struct termios toptions;         /* get current serial port settings */
       tcgetattr(fd, &toptions);        /* set 9600 baud both ways */
       cfsetispeed(&toptions, B9600);
       cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8;         /* Canonical mode */
       toptions.c_lflag |= ICANON;       /* commit the serial port settings */
       tcsetattr(fd, TCSANOW, &toptions);
}
void sendCommand(const char *abc)
{
   write(fd, abc, 1);
}

// ARDUINO CODE ABOVE ^^^^^^^^^^^^^^

const int max_value_H = 360/2;
const int max_value = 255;
int low_HR = 0, low_SR = 50, low_VR = 60;
int high_HR = max_value_H, high_SR = max_value, high_VR = max_value;

int low_HG = 0, low_SG = 0, low_VG = 0;
int high_HG = max_value_H, high_SG = max_value, high_VG = max_value;

int low_HB = 0, low_SB = 0, low_VB = 0;
int high_HB = max_value_H, high_SB = max_value, high_VB = max_value;


static void on_low_H_thresh_trackbar(int, void *)
{
    low_HR = min(high_HR-1, low_HR);
    setTrackbarPos("Low HR", "RED", low_HR);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_HR = max(high_HR, low_HR+1);
    setTrackbarPos("High HR", "RED", high_HR);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_SR = min(high_SR-1, low_SR);
    setTrackbarPos("Low SR", "RED", low_SR);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_SR = max(high_SR, low_SR+1);
    setTrackbarPos("High SR", "RED", high_SR);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_VR = min(high_VR-1, low_VR);
    setTrackbarPos("Low VR", "RED", low_VR);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_VR = max(high_VR, low_VR+1);
    setTrackbarPos("High VR", "RED", high_VR);
}

double distance(Point a, Point b){
	double dist = std::hypot(a.x-b.x, a.y-b.y);
	return dist;
}

/*
//GREEN
static void on_low_HG_thresh_trackbar(int, void *)
{
    low_HG = min(high_HG-1, low_HG);
    setTrackbarPos("Low HG", "GREEN", low_HG);
}
static void on_high_HG_thresh_trackbar(int, void *)
{
    high_HG = max(high_HG, low_HG+1);
    setTrackbarPos("High HG", "GREEN", high_HG);
}
static void on_low_SG_thresh_trackbar(int, void *)
{
    low_SG = min(high_SG-1, low_SG);
    setTrackbarPos("Low SG", "GREEN", low_SG);
}
static void on_high_SG_thresh_trackbar(int, void *)
{
    high_SG = max(high_SG, low_SG+1);
    setTrackbarPos("High SG", "GREEN", high_SG);
}
static void on_low_VG_thresh_trackbar(int, void *)
{
    low_VG = min(high_VG-1, low_VG);
    setTrackbarPos("Low VG", "GREEN", low_VG);
}
static void on_high_VG_thresh_trackbar(int, void *)
{
    high_VG = max(high_VG, low_VG+1);
    setTrackbarPos("High VG", "GREEN", high_VG);
}

// BLUE
static void on_low_HB_thresh_trackbar(int, void *)
{
    low_HB = min(high_HB-1, low_HB);
    setTrackbarPos("Low HB", "BLUE", low_HB);
}
static void on_high_HB_thresh_trackbar(int, void *)
{
    high_HB = max(high_HB, low_HB+1);
    setTrackbarPos("High HB", "BLUE", high_HB);
}
static void on_low_SB_thresh_trackbar(int, void *)
{
    low_SB = min(high_SB-1, low_SB);
    setTrackbarPos("Low SG", "BLUE", low_SG);
}
static void on_high_SB_thresh_trackbar(int, void *)
{
    high_SB = max(high_SB, low_SB+1);
    setTrackbarPos("High SB", "BLUE", high_SB);
}
static void on_low_VB_thresh_trackbar(int, void *)
{
    low_VB = min(high_VB-1, low_VB);
    setTrackbarPos("Low VB", "BLUE", low_VB);
}
static void on_high_VB_thresh_trackbar(int, void *)
{
    high_VB = max(high_VB, low_VB+1);
    setTrackbarPos("High VB", "BLUE", high_VB);
}*/


/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(Mat& im, const string label, vector<Point>& contour)
{
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);
	Rect r = boundingRect(contour);

	Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

int flag=0;

int main()
{
	settings("/dev/ttyACM0");
	
	VideoCapture vid(0);  
	Mat src;
	char c = 65;
	// int Circles[100], CircleCount;
	int FrameCount = 0;

	namedWindow("RED", WINDOW_NORMAL);
	    // Trackbars to set thresholds for HSV values for RED filter
    createTrackbar("Low HR", "RED", &low_HR, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High HR", "RED", &high_HR, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low SR", "RED", &low_SR, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High SR", "RED", &high_SR, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low VR", "RED", &low_VR, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High VR", "RED", &high_VR, max_value, on_high_V_thresh_trackbar);
/*

        // Trackbars to set thresholds for HSV values for GREEN filter
    createTrackbar("Low HG", "GREEN", &low_HG, max_value_H, on_low_HG_thresh_trackbar);
    createTrackbar("High HG", "GREEN", &high_HG, max_value_H, on_high_HG_thresh_trackbar);
    createTrackbar("Low SG", "GREEN", &low_SG, max_value, on_low_SG_thresh_trackbar);
    createTrackbar("High SG", "GREEN", &high_SG, max_value, on_high_SG_thresh_trackbar);
    createTrackbar("Low VG", "GREEN", &low_VG, max_value, on_low_VG_thresh_trackbar);
    createTrackbar("High VG", "GREEN", &high_VG, max_value, on_high_VG_thresh_trackbar);

        // Trackbars to set thresholds for HSV values for Blue filter
    createTrackbar("Low HB", "BLUE", &low_HB, max_value_H, on_low_HB_thresh_trackbar);
    createTrackbar("High HB", "BLUE", &high_HB, max_value_H, on_high_HB_thresh_trackbar);
    createTrackbar("Low SB", "BLUE", &low_SB, max_value, on_low_SB_thresh_trackbar);
    createTrackbar("High SB", "BLUE", &high_SB, max_value, on_high_SB_thresh_trackbar);
    createTrackbar("Low VB", "BLUE", &low_VB, max_value, on_low_VB_thresh_trackbar);
    createTrackbar("High VB", "BLUE", &high_VB, max_value, on_high_VB_thresh_trackbar);
*/
	//waitKey(0); 
	while(c != 27)
	{
		
		//cv::Mat src = cv::imread("polygon.png");
		vid >> src; 
		double maxarea=0;
		// Takes in video input
		if (src.empty())
			return -1;
		Point cntr;
		cntr.x=src.cols/2;
		cntr.y=src.rows/2;
		// CircleCount = 0;
		//int flag=0;
		//stores area of circles in descending order
		//map<double, int, greater <double> > AreaOfCircles;

		//Convert to HSV scale
		Mat hsvFrame;
		cvtColor(src, hsvFrame, COLOR_BGR2HSV);

		//ColorFilteredOutput
		Mat Green, Red, Blue;
		inRange(hsvFrame, Scalar(low_HR, low_SR, low_VR), Scalar(high_HR, high_SR, high_VR), Red);
		// inRange(hsvFrame, Scalar(low_HG, low_SG, low_VG), Scalar(high_HG, high_SG, high_VG), Green);
		// inRange(hsvFrame, Scalar(low_HB, low_SB, low_VB), Scalar(high_HB, high_SB, high_VB), Blue);

		// Convert to grayscale
		Mat gray;
		cvtColor(src, gray, CV_BGR2GRAY);

		// Use Canny instead of threshold to catch squares with gradient shading
		Mat bw;
		Canny(gray, bw, 0, 50, 5);

		// Find contours
		vector<vector<Point> > contours, coutoursRT;

		vector<Point> LargestCirlcle;
		vector<Point> TriangleNear;
		vector<Point> RectangleNear;
		findContours(Red.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		// findContours(bw.clone(), contoursRT, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		vector<Point> approx;
		Mat dst = src.clone();
		Point center, centerT, centerR;
		int areaT=0, areaR=0;

		int i;
		//AreaOfCircles.clear();
		for (i = 0; i < contours.size(); i++)
		{
			// Approximate contour with accuracy proportional
			// to the contour perimeter
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

			// Skip small or non-convex objects 
			if (fabs(contourArea(contours[i])) < 2500 || !isContourConvex(approx))
				continue;

			if (approx.size() == 3)
			{
				setLabel(dst, "TRI", contours[i]);    // Triangles
				//if(fabs(contourArea(contours[i]))>0.1*src.cols*src.rows){
					// flag=1;
					// TriangleNear=contours[i];
					Rect rT = boundingRect(contours[i]);
					areaT = contourArea(contours[i]);
				// 	if(areatT>areaT){
				// 		centerT={rT.x+rT.width/2, rT.y+rT.height/2};
				// 		areaT=fabs(contourArea(contours[i]));
				// 	}
				// //}

			}
			else if (approx.size() >= 4 && approx.size() <= 6)
			{
				// Number of vertices of polygonal curve
				int vtc = approx.size();

				// Get the cosines of all corners
				std::vector<double> cos;
				for (int j = 2; j < vtc+1; j++)
					cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

				// Sort ascending the cosine values
				sort(cos.begin(), cos.end());

				// Get the lowest and the highest cosine
				double mincos = cos.front();
				double maxcos = cos.back();

				// Use the degrees obtained above and the number of vertices
				// to determine the shape of the contour
				if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3){
					setLabel(dst, "RECT", contours[i]);
					//if(fabs(contourArea(contours[i]))>0.1*src.cols*src.rows){
						// flag=2;
						// RectangleNear=contours[i];
						Rect rR = boundingRect(contours[i]);
						areaR = contourArea(contours[i]);
						// if(areatR>areaR){
						// 	centerR={rR.x+rR.width/2, rR.y+rR.height/2};
						// 	areaR=fabs(contourArea(contours[i]));
						// }
					//}
				}

				// else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
				// 	setLabel(dst, "PENTA", contours[i]);
				// else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
				// 	setLabel(dst, "HEXA", contours[i]);
			}
			else
			{
				// Detect and label circles
				double area = contourArea(contours[i]);
				Rect r = boundingRect(contours[i]);
				int radius = r.width / 2;

//				if (abs(1 - ((double)r.width / r.height)) <= 0.2 &&              for pure circles
//				    abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.2)
					setLabel(dst, "CIR", contours[i]);
					//AreaOfCircles.insert({fabs(area), i});
					// Circles[CircleCount] = i;
					// CircleCount++;
					if(area>maxarea){
						center={r.x+r.width/2, r.y+r.height/2};
						maxarea=area;
					}
			}
		}
		
		// float radius;
		// approxPolyDP(Mat(contours[(AreaOfCircles.begin())->second]), LargestCirlcle, 3, true);
		// minEnclosingCircle(LargestCirlcle, center, radius);
		//cout<<flag;
		// auto it = AreaOfCircles.begin();
		// it++;
		// AreaOfCircles.erase(it, AreaOfCircles.end());
		cout<<FrameCount<<endl;
		cout<<areaT<<endl;
		//cout<<maxarea<<endl;
		if(maxarea>areaT && maxarea>areaR){
			
			if((center.x && center.x < src.cols * 3/8) && fabs(maxarea)<0.1*src.rows*src.cols){
				cout<<'A'<<endl;
				sendCommand("A");
				//turn right;
			}
			else if((center.x && center.x > src.cols * 5/8) && fabs(maxarea)<0.1*src.rows*src.cols){
				cout<<'D'<<endl;
				sendCommand("D");
				//turn left;
			}
			else if(center.x && fabs(maxarea)<0.1*src.rows*src.cols){
				//move forward;
				cout<<'W'<<endl;
				sendCommand("W");
			}
		// 	else
		// 	{
		// 		cout<<'S'<<endl;
		// 		sendCommand("S");
		// 	}
		}
		if(areaT>4000 && FrameCount>30){
			//if(flag==0){
				cout<<"T"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				sendCommand("T");
				flag=1;
				areaT=0;
				FrameCount=0;
			//}
			// if((center.x && center.x < src.cols * 3/8) && fabs(maxarea)<0.1*src.rows*src.cols){
			// 	cout<<'A'<<endl;
			// 	sendCommand("A");
			// 	//turn right;
			// }
			// else if(center.x && fabs(maxarea)<0.1*src.rows*src.cols){
			// 	//move forward;
			// 	cout<<'W'<<endl;
			// 	sendCommand("W");
			// }
			// else
			// {
			// 	cout<<'S'<<endl;
			// 	sendCommand("S");
			// }

		}
		else if(areaR>7000 && FrameCount>30){
			//code for rectangle
			//if(flag==0){
				cout<<"R"<<endl;
				sendCommand("R");
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				cout<<"---------------------------------------------------------"<<endl;
				flag=2;
				areaR=0;
				FrameCount=0;
			//}
			// if((center.x && centerR.x > src.cols * 5/8) && fabs(maxarea)<0.1*src.rows*src.cols){
			// 	cout<<'D'<<endl;
			// 	sendCommand("D");
			// 	//turn left;
			// }
			// else if(center.x && fabs(maxarea)<0.1*src.rows*src.cols){
			// 	//move forward;
			// 	cout<<'W'<<endl;
			// 	sendCommand("W");
			// }
			// else
			// {
			// 	cout<<'S'<<endl;
			// 	sendCommand("S");
			// }

		}

		// if(flag != 0){
			FrameCount++;
			// if(FrameCount > 333)
		// 	{
		// 		FrameCount = 0;
		// 		flag = 0;
		// 	}
		// }


		// if(contours.size()){
		// 	cout<<"S"<<endl;
		// 	sendCommand("S");
		// }

		// if(flag != 0)
		// {
		// 	flag > 100 ? flag = 0 : flag++;
		// }

		imshow("src", src);
		imshow("dst", dst);
		imshow("RED", Red);
		imshow("bw", bw);
		
		c = waitKey(15);
	}
		
	
	destroyAllWindows();
	return 0;
}



//TODO Create colour filter for B, G, and R 9430470050