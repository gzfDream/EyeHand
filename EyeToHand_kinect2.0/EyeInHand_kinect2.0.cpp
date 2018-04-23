// EyeToHand_kinect2.0.cpp : Defines the entry point for the console application.
//
/*
*��������ʹ�á��������������������̶�����ı任��ϵ������̶��ڻ�е����
*���裺
*     
*    
*     
*�������ƣ�ÿ����λ��һ��
*
*/

#include "stdafx.h"
#include "Pointcloud_processing.h"
#include "CalChessboard.h"
#include "Tsai_HandEye.h"

void printMenu() {
	printf("Commands:\n");
	printf("  g  ��ñ궨ͼƬ\n");
	printf("  c  ����������\n");
	printf("  p  �궨����\n");
	printf("  q  �˳�\n");
}

void collection(){
	Pointcloud_processing pp;
	HRESULT hr = pp.init_kinect();

	bool m_stop = pp.show_pointCloud();
}

void calibration(Camera_Intrinsics camera_ins){
	double camD1[9] = { camera_ins.FLX, 0, camera_ins.PPX,
		0, camera_ins.FLY, camera_ins.PPY,
		0, 0, 1 };
	double distCoeffD1[5] = { 0,0,0,0,0 };
	double markerRealSize = 3;

	CalChessboard cal = CalChessboard(camD1, distCoeffD1, markerRealSize);

	std::string imgpath, calibFile;
	imgpath = "../data/color_cam_cal/";
	calibFile = "../data/calibration/cal_cam_cal.txt";
	cal.calprocess(imgpath, calibFile);

}

int _tmain(int argc, _TCHAR* argv[])
{
	Pointcloud_processing pp;
	HRESULT hr = pp.init_kinect();

	Camera_Intrinsics camera_ins;
	//���ͼ
	/*camera_ins.FLX = 366.685;
	camera_ins.FLY = 366.685;
	camera_ins.PPX = 256.52;
	camera_ins.PPY = 208.1;*/
	//��ɫͼ
	camera_ins.FLX = 1053.918;
	camera_ins.FLY = 1053.918;
	camera_ins.PPX = 942.797;
	camera_ins.PPY = 552.087;

	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'g':
			//camera_ins = pp.getCameIns();
			//pp.show_pointCloud();
			pp.getHDImage();
			break;
		case 'c':
			calibration(camera_ins);
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

