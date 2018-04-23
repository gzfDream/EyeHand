#include "OpenNIKinect.h"


OpenNIKinect::OpenNIKinect(double in[4])
{
	u0 = in[0];
	v0 = in[1];
	fx = in[2];
	fy = in[3];
}


OpenNIKinect::~OpenNIKinect()
{
	closeCamera();
}


int OpenNIKinect::showdevicd(){
	// 获取设备信息  
	
	OpenNI::enumerateDevices(&aDeviceList);

	cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << endl;

	for (int i = 0; i < aDeviceList.getSize(); ++i)
	{
		cout << "设备 " << i << endl;
		const DeviceInfo& rDevInfo = aDeviceList[i];
		cout << "设备名： " << rDevInfo.getName() << endl;
		cout << "设备Id： " << rDevInfo.getUsbProductId() << endl;
		cout << "供应商名： " << rDevInfo.getVendor() << endl;
		cout << "供应商Id: " << rDevInfo.getUsbVendorId() << endl;
		cout << "设备URI: " << rDevInfo.getUri() << endl;

	}
	return aDeviceList.getSize();
}

void OpenNIKinect::initEngine(int num){

	Status rc = STATUS_OK;

	// 初始化OpenNI环境
	OpenNI::initialize();

	showdevicd();

	if (num < aDeviceList.getSize())
	{
		const char * deviceURL = aDeviceList[num].getUri();//openni::ANY_DEVICE;  //设备名
		rc = xtion.open(deviceURL);
	}
	else
	{
		cout << "device ID error!!" << endl;
	}
	


	rc = streamDepth.create(xtion, SENSOR_DEPTH);
	if (rc == STATUS_OK)
	{
		// 设置深度图像视频模式
		VideoMode mModeDepth;
		// 分辨率大小
		mModeDepth.setResolution(640, 480);
		// 每秒30帧
		mModeDepth.setFps(30);
		// 像素格式
		mModeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

		streamDepth.setVideoMode(mModeDepth);

		// 打开深度数据流
		rc = streamDepth.start();
		if (rc != STATUS_OK)
		{
			cerr << "无法打开深度数据流：" << OpenNI::getExtendedError() << endl;
			streamDepth.destroy();
		}
	}
	else
	{
		cerr << "无法创建深度数据流：" << OpenNI::getExtendedError() << endl;
	}

	rc = streamColor.create(xtion, SENSOR_COLOR);
	if (rc == STATUS_OK)
	{
		// 同样的设置彩色图像视频模式
		VideoMode mModeColor;
		mModeColor.setResolution(320, 240);
		mModeColor.setFps(30);
		mModeColor.setPixelFormat(PIXEL_FORMAT_RGB888);

		streamColor.setVideoMode(mModeColor);

		// 打开彩色图像数据流
		rc = streamColor.start();
		if (rc != STATUS_OK)
		{
			cerr << "无法打开彩色图像数据流：" << OpenNI::getExtendedError() << endl;
			streamColor.destroy();
		}
	}
	else
	{
		cerr << "无法创建彩色图像数据流：" << OpenNI::getExtendedError() << endl;
	}

	if (!streamColor.isValid() || !streamDepth.isValid())
	{
		cerr << "彩色或深度数据流不合法" << endl;
		OpenNI::shutdown();
		//return 1;
	}

	// 图像模式注册,彩色图与深度图对齐
	if (xtion.isImageRegistrationModeSupported(
		IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		xtion.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

}

Mat OpenNIKinect::getDepthImage(){
	Mat frame;
	
	// 读取数据流
	Status rc = streamDepth.readFrame(&frameDepth);
	if (rc == STATUS_OK)
	{
		// 将深度数据转换成OpenCV格式
		const Mat mImageDepth(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
		//	imwrite(".\\depth\\"+std::to_string(i)+".png", mImageDepth);
		// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
		Mat mScaledDepth;
		Mat hScaledDepth;
		//mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);
		mImageDepth.convertTo(mScaledDepth, CV_16U, 5.0);
		//水平镜像深度图
		flip(mScaledDepth, hScaledDepth, 1);
		hScaledDepth.copyTo(frame);
	}
	return frame;
}

Mat OpenNIKinect::getRGBImage(){
	Mat frame;

	Status rc = streamColor.readFrame(&frameColor);
	if (rc == STATUS_OK)
	{
		// 同样的将彩色图像数据转化成OpenCV格式
		const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
		//	imwrite(".\\image\\" + std::to_string(i) + ".png", mImageRGB);
		// 首先将RGB格式转换为BGR格式
		Mat cImageBGR, bImageBGR, hImageBGR;
		cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);

		//水平镜像深度图
		flip(cImageBGR, hImageBGR, 1);
		resize(hImageBGR, hImageBGR, Size(640, 480));
		hImageBGR.copyTo(frame);
	}

	return frame;
}

bool OpenNIKinect::closeCamera(){
	streamDepth.destroy();
	streamColor.destroy();
	xtion.close();
	OpenNI::shutdown();

	return true;
}

bool OpenNIKinect::PCDsynthesis(string rgbfilepath, string dfilepath, string savePath){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cv::Mat color = cv::imread(rgbfilepath, CV_LOAD_IMAGE_ANYCOLOR);
	cv::Mat depth = cv::imread(dfilepath, -1);

	int rowNumber = color.rows;
	int colNumber = color.cols;

	cloud->height = 1;
	cloud->width = depth.cols*depth.rows;
	cloud->points.resize(depth.rows * depth.cols);

	for (unsigned int v = 0; v < depth.rows; ++v)
	{
		for (unsigned int u = 0; u < depth.cols; ++u)
		{
			unsigned int num = v*colNumber + u;
			double Xw = 0, Yw = 0, Zw = 0;
			if (depth.at<ushort>(v, u)>0)
			{
				Zw = depth.at<ushort>(v, u) / 5000.0;
				Xw = ((double)u - u0) * Zw / fx;
				Yw = ((double)v - v0) * Zw / fy;
			}

			cloud->points[num].b = color.at<cv::Vec3b>(v, u)[0];
			cloud->points[num].g = color.at<cv::Vec3b>(v, u)[1];
			cloud->points[num].r = color.at<cv::Vec3b>(v, u)[2];

			cloud->points[num].x = Xw;
			cloud->points[num].y = Yw;
			cloud->points[num].z = Zw;
		}
	}

	if (pcl::io::savePCDFile(savePath, *cloud))
	{
		return true;
	}
	else
	{
		return false;
	}
	
}
