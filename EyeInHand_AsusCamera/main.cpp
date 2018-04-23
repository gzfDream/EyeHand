#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>

#include <windows.h>
#include <iostream>
#include <vector>
#include <fstream>  
#include <string>
#include "CalChessboard.h"
#include "HandEye.h"

#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "OpenNIKinect.h"
using namespace std;
using namespace openni;
using namespace cv;

void printMenu() {
	printf("AsusCamera Commands:\n");
	printf("  g  获得标定图片\n");
	printf("  c  计算相机外参\n");
	printf("  p  标定计算\n");
	printf("  q  退出\n");
}

bool Collect()
{
	string rgbfilepath = "../data/color/";
	string dfilepath = "../data/depth/";
	//double in2[4] = { 319.5, 239.5, 536.24, 536.24 };
	double in[4] = { 319.5, 239.5, 532.49, 532.49 };
	char key = 0;
	int t1=0;
	Mat rgbMat, depthMat;

	OpenNIKinect m_kinect(in);
	m_kinect.initEngine(0);

	namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
	namedWindow("Color Image", CV_WINDOW_AUTOSIZE);

	bool loop = true;
	while (loop)
	{
		depthMat = m_kinect.getDepthImage();
		rgbMat = m_kinect.getRGBImage();

		imshow("Color Image", rgbMat);
		imshow("Depth Image", depthMat);

		if (waitKey(1) == '1')
		{
			imwrite(dfilepath + std::to_string(t1) + ".png", depthMat);
			imwrite(rgbfilepath + std::to_string(t1) + ".png", rgbMat);
			t1++;
		}
		else if (waitKey(1) == 27){
			loop = false;
			destroyAllWindows();
			return true;
		}
	}
	return false;
	
}

void calibration( ){
	double camD1[9] = { 532.49, 0, 319.5,
		0, 532.49, 239.5,
		0, 0, 1 };
	double distCoeffD1[5] = { 0, 0, 0, 0, 0 };
	double markerRealSize = 3;

	CalChessboard cal = CalChessboard(camD1, distCoeffD1, markerRealSize);

	std::string imgpath, calibFile;
	imgpath = "../data/color/";
	calibFile = "../data/calibration/cal.txt";
	cal.calprocess(imgpath, calibFile);

}

int _tmain(int argc, _TCHAR* argv[])
{
	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'g':
			Collect();
			break;
		case 'c':
			calibration();
			break;
		case 'p':
			processes();
			break;
		case 'q':
			running = false;
			break;
		}
	}
	return 0;
}
