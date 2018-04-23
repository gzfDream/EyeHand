#pragma once

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include "OpenNI.h"
#include <iostream>

using namespace std;
using namespace openni;
using namespace cv;
typedef unsigned char uchar;

class OpenNIKinect
{
public:
	OpenNIKinect(double in[4]);
	~OpenNIKinect();

	
	int showdevicd();
	//int  switchdevice();
	void initEngine(int num);

	Mat getRGBImage();
	Mat getDepthImage();
	bool closeCamera();

	bool PCDsynthesis(string rgbfilepath, string dfilepath, string savePath);
private:

	// 创建深度数据流
	VideoStream streamDepth;
	// 创建彩色图像数据流
	VideoStream streamColor;

	// 循环读取数据流信息并保存在VideoFrameRef中
	VideoFrameRef  frameDepth;
	VideoFrameRef  frameColor;

	// 声明并打开Device设备。
	Device xtion;

	//设备列表
	Array<DeviceInfo> aDeviceList;

	int image_width;
	int image_height;

	//相机内参
	//const double u0 = 319.5;
	//const double v0 = 239.5;
	//const double fx = 533.9;
	//const double fy = 533.9;
	
	double u0;
	double v0;
	double fx;
	double fy;

};

