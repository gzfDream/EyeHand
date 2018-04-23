#include <iostream>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include "OpenNIKinect.h"
using namespace std;
using namespace openni;
using namespace cv;
Array<DeviceInfo> aDeviceList;
int showdevice(){
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

void hMirrorTrans(const Mat &src, Mat &dst)
{
	dst.create(src.rows, src.cols, src.type());

	int rows = src.rows;
	int cols = src.cols;

	switch (src.channels())
	{
	case 1:   //1ͨ���������ͼ��
		const uchar *origal;
		uchar *p;
		for (int i = 0; i < rows; i++){
			origal = src.ptr<uchar>(i);
			p = dst.ptr<uchar>(i);
			for (int j = 0; j < cols; j++){
				p[j] = origal[cols - 1 - j];
			}
		}
		break;
	case 3:   //3ͨ�������ɫͼ��
		const Vec3b *origal3;
		Vec3b *p3;
		for (int i = 0; i < rows; i++) {
			origal3 = src.ptr<Vec3b>(i);
			p3 = dst.ptr<Vec3b>(i);
			for (int j = 0; j < cols; j++){
				p3[j] = origal3[cols - 1 - j];
			}
		}
		break;
	default:
		break;
	}

}


int main1()
{
	string rgbfilepath, dfilepath;
	char sig;
	uchar save_or = 0;
	double in1[4] = { 319.5, 239.5, 536.24, 536.24 };
	double in2[4] = { 319.5, 239.5, 532.49, 532.49 };
	
	Mat rgbMat, depthMat;

	string savePath1, savePath2;
	string rgbfilepath1, dfilepath1, rgbfilepath2, dfilepath2;

	OpenNIKinect m_kinect2(in2);
	OpenNIKinect m_kinect1(in1);
	printf("Open the which Kinect? Press '1'or'2' to chose\n");
	scanf_s("%c", &sig);
	printf("Opening the NO.%c Kinect?", &sig);

	if (sig == '1'){
		rgbfilepath = "..\\data\\color1\\";
		dfilepath = "..\\data\\depth1\\";
		m_kinect1.initEngine(sig - '1');
	}
	else if (sig == '2'){
		rgbfilepath = "..\\data\\color2\\";
		dfilepath = "..\\data\\depth2\\";
		m_kinect2.initEngine(sig - '1');
	}

	namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
	namedWindow("Color Image", CV_WINDOW_AUTOSIZE);

	printf("Save or not ? press [y] or [n] to go on\n");
	save_or = cv::waitKey(0);
	if (save_or == 'y')
	{
		printf("Save rgbd data and synthesis result of point cloud.\n");
	}
	else{
		printf("Don't save rgbd. Only to show.\n");
	}

	int i = 1;
	bool loop = true;
	while (loop)
	{
		if (sig == '1')
		{
			depthMat = m_kinect1.getDepthImage();
			rgbMat = m_kinect1.getRGBImage();
		}
		else if (sig == '2')
		{
			depthMat = m_kinect2.getDepthImage();
			rgbMat = m_kinect2.getRGBImage();
		}
		/*depthMat = m_kinect.getDepthImage();
		rgbMat = m_kinect.getRGBImage();*/

		imshow("Color Image", rgbMat);
		imshow("Depth Image", depthMat);

		if (save_or == 'y')
		{
			imwrite(dfilepath + std::to_string(i) + ".png", depthMat);
			imwrite(rgbfilepath + std::to_string(i) + ".png", rgbMat);
			i++;

			if (i == 6)
			{
				loop = false;
			}
		}

		if (waitKey(1) == 27)
			break;
	}

	if (save_or == 'y'){
		if (sig == '1'){
			rgbfilepath1 = "..\\data\\color1\\3.png";
			dfilepath1 = "..\\data\\depth1\\3.png";
			savePath1 = "..\\data\\pcd1\\1.pcd";
			cout << "start to save pcd..." << endl;
			m_kinect1.PCDsynthesis(rgbfilepath1, dfilepath1, savePath1);
		}
		else if (sig == '2'){
			rgbfilepath2 = "..\\data\\color2\\3.png";
			dfilepath2 = "..\\data\\depth2\\3.png";
			savePath2 = "..\\data\\pcd2\\1.pcd";
			cout << "start to save pcd..." << endl;
			m_kinect2.PCDsynthesis(rgbfilepath2, dfilepath2, savePath2);
		}
	}

	cout << "END" << endl;

	//system("pause");
	return 0;
}
