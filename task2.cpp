/************************************************************ 
|                                                           |
| NEXUS 2020 Code 2 v1.0                                    |
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
//Arduino linking above

const int max_value_H = 360/2;
const int max_value = 255;
int low_HR = 94, low_SR = 66, low_VR = 164;
int high_HR = 118, high_SR = 143, high_VR = max_value;

int low_HG = 0, low_SG = 0, low_VG = 0;
int high_HG = max_value_H, high_SG = max_value, high_VG = max_value;

int low_HB = 0, low_SB = 0, low_VB = 0;
int high_HB = max_value_H, high_SB = max_value, high_VB = max_value;

int low_HM = 0, low_SM = 0, low_VM = 0;
int high_HM = max_value_H, high_SM = max_value, high_VM = max_value;


//RED
static void on_low_HR_thresh_trackbar(int, void *)
{
    low_HR = min(high_HR-1, low_HR);
    setTrackbarPos("Low HR", "RED", low_HR);
}
static void on_high_HR_thresh_trackbar(int, void *)
{
    high_HR = max(high_HR, low_HR+1);
    setTrackbarPos("High HR", "RED", high_HR);
}
static void on_low_SR_thresh_trackbar(int, void *)
{
    low_SR = min(high_SR-1, low_SR);
    setTrackbarPos("Low SR", "RED", low_SR);
}
static void on_high_SR_thresh_trackbar(int, void *)
{
    high_SR = max(high_SR, low_SR+1);
    setTrackbarPos("High SR", "RED", high_SR);
}
static void on_low_VR_thresh_trackbar(int, void *)
{
    low_VR = min(high_VR-1, low_VR);
    setTrackbarPos("Low VR", "RED", low_VR);
}
static void on_high_VR_thresh_trackbar(int, void *)
{
    high_VR = max(high_VR, low_VR+1);
    setTrackbarPos("High VR", "RED", high_VR);
}

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
}

// BARCODE
static void on_low_HM_thresh_trackbar(int, void *)
{
    low_HM = min(high_HM-1, low_HM);
    setTrackbarPos("Low HM", "BARCODE", low_HM);
}
static void on_high_HM_thresh_trackbar(int, void *)
{
    high_HM = max(high_HM, low_HM+1);
    setTrackbarPos("High HM", "BARCODE", high_HM);
}
static void on_low_SM_thresh_trackbar(int, void *)
{
    low_SM = min(high_SM-1, low_SM);
    setTrackbarPos("Low SG", "BARCODE", low_SG);
}
static void on_high_SM_thresh_trackbar(int, void *)
{
    high_SM = max(high_SM, low_SM+1);
    setTrackbarPos("High SM", "BARCODE", high_SM);
}
static void on_low_VM_thresh_trackbar(int, void *)
{
    low_VM = min(high_VM-1, low_VM);
    setTrackbarPos("Low VM", "BARCODE", low_VM);
}
static void on_high_VM_thresh_trackbar(int, void *)
{
    high_VM = max(high_VM, low_VM+1);
    setTrackbarPos("High VM", "BARCODE", high_VM);
}


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

int main()
{
	
	settings("/dev/ttyACM0");int frame=0;
	VideoCapture vid(0);  
	Mat src;
	char c = 65;
	int Circles[100], CircleCount;
    int Barcodes[160], BarcodeCount = 0, BarFlag = 0; // Barflag becomes one after encountering a barcode


	namedWindow("RED", WINDOW_NORMAL);
    namedWindow("GREEN", WINDOW_NORMAL);
    namedWindow("BLUE", WINDOW_NORMAL);
	namedWindow("BARCODE", WINDOW_NORMAL);
    namedWindow("bw", WINDOW_NORMAL);

	    // Trackbars to set thresholds for HSV values for RED filter
    createTrackbar("Low HR", "RED", &low_HR, max_value_H, on_low_HR_thresh_trackbar);
    createTrackbar("High HR", "RED", &high_HR, max_value_H, on_high_HR_thresh_trackbar);
    createTrackbar("Low SR", "RED", &low_SR, max_value, on_low_SR_thresh_trackbar);
    createTrackbar("High SR", "RED", &high_SR, max_value, on_high_SR_thresh_trackbar);
    createTrackbar("Low VR", "RED", &low_VR, max_value, on_low_VR_thresh_trackbar);
    createTrackbar("High VR", "RED", &high_VR, max_value, on_high_VR_thresh_trackbar);

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

		// Trackbars to set thresholds for HSV values for Barcode filter
    createTrackbar("Low HM", "BARCODE", &low_HM, max_value_H, on_low_HM_thresh_trackbar);
    createTrackbar("High HM", "BARCODE", &high_HM, max_value_H, on_high_HM_thresh_trackbar);
    createTrackbar("Low SM", "BARCODE", &low_SM, max_value, on_low_SM_thresh_trackbar);
    createTrackbar("High SM", "BARCODE", &high_SM, max_value, on_high_SM_thresh_trackbar);
    createTrackbar("Low VM", "BARCODE", &low_VM, max_value, on_low_VM_thresh_trackbar);
    createTrackbar("High VM", "BARCODE", &high_VM, max_value, on_high_VM_thresh_trackbar);

	while(c != 27)
	{
		//cv::Mat src = cv::imread("polygon.png");
		vid >> src;  // Takes in video input
		if (src.empty())
			return -1;

		CircleCount = 0;

		double maxarea=0;
		Point center;
		//Convert to HSV scale
		Mat hsvFrame;
		cvtColor(src, hsvFrame, COLOR_BGR2HSV);

		//ColorFilteredbuegreenOutput
		Mat Green, Red, Blue, BarMat;
		inRange(hsvFrame, Scalar(low_HR, low_SR, low_VR), Scalar(high_HR, high_SR, high_VR), Red);
		inRange(hsvFrame, Scalar(low_HG, low_SG, low_VG), Scalar(high_HG, high_SG, high_VG), Green);
        inRange(hsvFrame, Scalar(low_HB, low_SB, low_VB), Scalar(high_HB, high_SB, high_VB), Blue);
		inRange(hsvFrame, Scalar(low_HM, low_SM, low_VM), Scalar(high_HM, high_SM, high_VM), BarMat);

		// Convert to grayscale
		Mat gray;
		cvtColor(src, gray, CV_BGR2GRAY);

		// Use Canny instead of threshold to catch squares with gradient shading
		Mat bw;
		Canny(gray, bw, 0, 50, 5);

		// Find contours
		vector<vector<Point> > contours;
		vector<vector<Point> > contoursBAR;

        if((BarcodeCount && Barcodes[BarcodeCount-1] % 3 == 0) || BarFlag == 0){
		    findContours(Green.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		    sendCommand("G");
        }
        else if(BarcodeCount && (Barcodes[BarcodeCount-1] % 3) == 1){
            findContours(Red.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            sendCommand("R");
        }
        else if(BarcodeCount && (Barcodes[BarcodeCount-1] % 2) == 2){
            findContours(Blue.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);  
            sendCommand("B");
        }

        
        findContours(BarMat.clone(), contoursBAR, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);   
 

		vector<Point> approx;
		Mat dst = src.clone();

		for (int i = 0; i < contours.size(); i++)
		{
			// Approximate contour with accuracy proportional
			// to the contour perimeter
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

			// Skip small or non-convex objects 
			if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
				continue;

			// if (approx.size() == 3)
			// {
			// 	setLabel(dst, "TRI", contours[i]);    // Triangles
			// }
			// else if (approx.size() >= 4 && approx.size() <= 6)
			// {
			// 	// Number of vertices of polygonal curve
			// 	int vtc = approx.size();

			// 	// Get the cosines of all corners
			// 	std::vector<double> cos;
			// 	for (int j = 2; j < vtc+1; j++)
			// 		cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

			// 	// Sort ascending the cosine values
			// 	sort(cos.begin(), cos.end());

			// 	// Get the lowest and the highest cosine
			// 	double mincos = cos.front();
			// 	double maxcos = cos.back();

			// 	// Use the degrees obtained above and the number of vertices
			// 	// to determine the shape of the contour
			// 	if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
			// 		setLabel(dst, "RECT", contours[i]);
			// 	// else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
			// 	// 	setLabel(dst, "PENTA", contours[i]);
			// 	// else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
			// 	// 	setLabel(dst, "HEXA", contours[i]);
			// }
			// else
			if(approx.size() > 6)
			{
				// Detect and label circles
				double area = contourArea(contours[i]);
				Rect r = boundingRect(contours[i]);
				int radius = r.width / 2;

//				if (abs(1 - ((double)r.width / r.height)) <= 0.2 &&              for pure circles
//				    abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.2)
					setLabel(dst, "CIR", contours[i]);
					//Circles[CircleCount] = i;
					//CircleCount++;
				if(area>maxarea && (r.y+r.height/2)<0.65*src.rows){
						center={r.x+r.width/2, r.y+r.height/2};
						maxarea=area;
				}
			}
		}
		if((center.x && center.x < src.cols * 3/8) ){
				cout<<'A'<<endl;
				sendCommand("A");
				//turn right;
			}
			else if((center.x && center.x > src.cols * 5/8) ){
				cout<<'D'<<endl;
				sendCommand("D");
				//turn left;
			}
			else if(center.x && fabs(maxarea)){
				//move forward;
				cout<<'W'<<endl;
				sendCommand("W");
		}
		double cent[2];
		int k=0;
		double areaBar[200];
		int l=0;
		if(frame>20){
			for (int i = 0; i < contoursBAR.size() ; i++)
			{
				approxPolyDP(Mat(contoursBAR[i]), approx, arcLength(Mat(contoursBAR[i]), true)*0.02, true);
				if (fabs(contourArea(contoursBAR[i])) < 500 || !isContourConvex(approx) || fabs(contourArea(contoursBAR[i]))>40000)
					continue;
				if(approx.size() < 6 )
				{
					if(l<2){
						Rect u = boundingRect(contoursBAR[i]);
						cent[l]=u.x+u.width/2;
						l++;
					}
					areaBar[k]=fabs(contourArea(contoursBAR[i]));
					cout<<areaBar[k]<<"  ";
					setLabel(dst, "BAR", contoursBAR[i]);
					k++;
				}
			}
			if(k==4){
				double avg=(areaBar[0]+areaBar[1]+areaBar[2]+areaBar[3])/4;
				int barvalue=0;
				for(int j=0; j<4; j++){
					if(areaBar[j]>avg){
						if(cent[0]<cent[1]){
							barvalue+=pow(2,(3-j));
						}
						else{
							barvalue+=pow(2,j);
						}
					}
				}
				Barcodes[BarcodeCount]=barvalue;
				BarcodeCount++;
				cout<<"current bar--------------------"<<endl;
				cout<<barvalue<<endl;
				cout<<"---------------------------------------------------------"<<endl;

			}
			else{
				cout<<"S"<<endl;
				sendCommand("S");
			}
			frame=0;
		}
		cout<<"\n";
		cout<<"---------------------------------------------------------"<<endl;
		cout<<frame<<endl;
		cout<<"---------------------------------------------------------"<<endl;
		frame++;


		imshow("src", src);
		imshow("bw",bw);
		imshow("dst", dst);
		imshow("RED", Red);
        imshow("GREEN", Green);
        imshow("BLUE", Blue);
		imshow("BARCODE", BarMat);
		c = waitKey(1);
		}
	
	destroyAllWindows();
	return 0;
}
