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
	// ��ȡ�豸��Ϣ  
	
	OpenNI::enumerateDevices(&aDeviceList);

	cout << "������������ " << aDeviceList.getSize() << " ������豸." << endl;

	for (int i = 0; i < aDeviceList.getSize(); ++i)
	{
		cout << "�豸 " << i << endl;
		const DeviceInfo& rDevInfo = aDeviceList[i];
		cout << "�豸���� " << rDevInfo.getName() << endl;
		cout << "�豸Id�� " << rDevInfo.getUsbProductId() << endl;
		cout << "��Ӧ������ " << rDevInfo.getVendor() << endl;
		cout << "��Ӧ��Id: " << rDevInfo.getUsbVendorId() << endl;
		cout << "�豸URI: " << rDevInfo.getUri() << endl;

	}
	return aDeviceList.getSize();
}

void OpenNIKinect::initEngine(int num){

	Status rc = STATUS_OK;

	// ��ʼ��OpenNI����
	OpenNI::initialize();

	showdevicd();

	if (num < aDeviceList.getSize())
	{
		const char * deviceURL = aDeviceList[num].getUri();//openni::ANY_DEVICE;  //�豸��
		rc = xtion.open(deviceURL);
	}
	else
	{
		cout << "device ID error!!" << endl;
	}
	


	rc = streamDepth.create(xtion, SENSOR_DEPTH);
	if (rc == STATUS_OK)
	{
		// �������ͼ����Ƶģʽ
		VideoMode mModeDepth;
		// �ֱ��ʴ�С
		mModeDepth.setResolution(640, 480);
		// ÿ��30֡
		mModeDepth.setFps(30);
		// ���ظ�ʽ
		mModeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

		streamDepth.setVideoMode(mModeDepth);

		// �����������
		rc = streamDepth.start();
		if (rc != STATUS_OK)
		{
			cerr << "�޷��������������" << OpenNI::getExtendedError() << endl;
			streamDepth.destroy();
		}
	}
	else
	{
		cerr << "�޷����������������" << OpenNI::getExtendedError() << endl;
	}

	rc = streamColor.create(xtion, SENSOR_COLOR);
	if (rc == STATUS_OK)
	{
		// ͬ�������ò�ɫͼ����Ƶģʽ
		VideoMode mModeColor;
		mModeColor.setResolution(320, 240);
		mModeColor.setFps(30);
		mModeColor.setPixelFormat(PIXEL_FORMAT_RGB888);

		streamColor.setVideoMode(mModeColor);

		// �򿪲�ɫͼ��������
		rc = streamColor.start();
		if (rc != STATUS_OK)
		{
			cerr << "�޷��򿪲�ɫͼ����������" << OpenNI::getExtendedError() << endl;
			streamColor.destroy();
		}
	}
	else
	{
		cerr << "�޷�������ɫͼ����������" << OpenNI::getExtendedError() << endl;
	}

	if (!streamColor.isValid() || !streamDepth.isValid())
	{
		cerr << "��ɫ��������������Ϸ�" << endl;
		OpenNI::shutdown();
		//return 1;
	}

	// ͼ��ģʽע��,��ɫͼ�����ͼ����
	if (xtion.isImageRegistrationModeSupported(
		IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		xtion.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

}

Mat OpenNIKinect::getDepthImage(){
	Mat frame;
	
	// ��ȡ������
	Status rc = streamDepth.readFrame(&frameDepth);
	if (rc == STATUS_OK)
	{
		// ���������ת����OpenCV��ʽ
		const Mat mImageDepth(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
		//	imwrite(".\\depth\\"+std::to_string(i)+".png", mImageDepth);
		// Ϊ�������ͼ����ʾ�ĸ�������һЩ����CV_16UC1 ==> CV_8U��ʽ
		Mat mScaledDepth;
		Mat hScaledDepth;
		//mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);
		mImageDepth.convertTo(mScaledDepth, CV_16U, 5.0);
		//ˮƽ�������ͼ
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
		// ͬ���Ľ���ɫͼ������ת����OpenCV��ʽ
		const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
		//	imwrite(".\\image\\" + std::to_string(i) + ".png", mImageRGB);
		// ���Ƚ�RGB��ʽת��ΪBGR��ʽ
		Mat cImageBGR, bImageBGR, hImageBGR;
		cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);

		//ˮƽ�������ͼ
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
