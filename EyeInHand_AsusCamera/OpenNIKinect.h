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

	// �������������
	VideoStream streamDepth;
	// ������ɫͼ��������
	VideoStream streamColor;

	// ѭ����ȡ��������Ϣ��������VideoFrameRef��
	VideoFrameRef  frameDepth;
	VideoFrameRef  frameColor;

	// ��������Device�豸��
	Device xtion;

	//�豸�б�
	Array<DeviceInfo> aDeviceList;

	int image_width;
	int image_height;

	//����ڲ�
	//const double u0 = 319.5;
	//const double v0 = 239.5;
	//const double fx = 533.9;
	//const double fy = 533.9;
	
	double u0;
	double v0;
	double fx;
	double fy;

};

