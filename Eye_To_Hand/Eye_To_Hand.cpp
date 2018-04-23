// EyeToHand_kinect2.0.cpp : Defines the entry point for the console application.
//
/*
*��������ʹ�á��������������������̶�����ı任��ϵ������̶�����
*���裺
*
*
*
*�������ƣ�ÿ����λ��һ��
*
*/

#include "stdafx.h"
#include "src/Pointcloud_processing.h"
#include "src/CalChessboard.h"
#include "src/HandEye.h"

void printMenu() {
	printf("Commands:\n");
	printf("  f  ��ñ궨ͼƬ(cal)\n");
	printf("  l  ����������(cal)\n");
	printf("  i  ��ú���ͼ��\n");
	printf("  k  ����������(Infrared)\n");
	printf("  g  ��ñ궨ͼƬ(����)\n");
	printf("  c  ����������(����)\n");
	printf("  h  ����robHcal��robHrgb*rgbHcal*calHdep*depHcal��\n");
	printf("  p  �궨����\n");
	printf("  q  �˳�\n");
}

void collection(){
	Pointcloud_processing pp;
	HRESULT hr = pp.init_kinect();

	bool m_stop = pp.show_pointCloud();
}

void calibration(Camera_Intrinsics camera_ins, bool robot){
	double camD1[9] = { camera_ins.FLX, 0, camera_ins.PPX,
		0, camera_ins.FLY, camera_ins.PPY,
		0, 0, 1 };
	double distCoeffD1[5] = { 0, 0, 0, 0, 0 };
	double markerRealSize = 3;

	CalChessboard cal = CalChessboard(camD1, distCoeffD1, markerRealSize);

	std::string imgpath, calibFile;
	if (robot)
	{
		imgpath = "../data/color_cam_robot/";
		calibFile = "../data/calibration/cal_cam_robot.txt";
	}
	else
	{
		imgpath = "../data/color_cam_cal/";
		calibFile = "../data/calibration/cal_cam_cal.txt";
	}
	cal.calprocess(imgpath, calibFile);
	cv::destroyAllWindows();
}

void caomputer_baseHcal_rgb(){

}

int _tmain(int argc, _TCHAR* argv[])
{
	Pointcloud_processing pp;
	HRESULT hr = pp.init_kinect();

	Camera_Intrinsics camera_ins_L, camera_ins_H;
	//���ͼ(����)
	camera_ins_L.FLX = 366.685;
	camera_ins_L.FLY = 366.685;
	camera_ins_L.PPX = 256.52;
	camera_ins_L.PPY = 208.1;
	//��ɫͼ(����)
	camera_ins_H.FLX = 1053.918;
	camera_ins_H.FLY = 1053.918;
	camera_ins_H.PPX = 942.797;
	camera_ins_H.PPY = 552.087;

	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'f':
			camera_ins_L = pp.getCameIns();
			pp.show_pointCloud();
			break;
		case 'l':
			calibration(camera_ins_L, false);
			//
			break;
		case 'i':
			pp.getInfrared();
			break;
		case 'k':
			calibration(camera_ins_L, true);
			break;
		case 'g':
			pp.getHDImage();
			break;
		case 'c':
			calibration(camera_ins_H, true);
			break;
		case 'H':
			caomputer_baseHcal_rgb();
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

