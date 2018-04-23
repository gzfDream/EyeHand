// EyeToHand_kinect2.0.cpp : Defines the entry point for the console application.
//
/*
*本程序是使用“两步法”计算机器人与固定相机的变换关系，相机固定在机械臂上
*步骤：
*     
*    
*     
*依次类推，每两个位置一组
*
*/

#include "stdafx.h"
#include "Pointcloud_processing.h"
#include "CalChessboard.h"
#include "Tsai_HandEye.h"

void printMenu() {
	printf("Commands:\n");
	printf("  g  获得标定图片\n");
	printf("  c  计算相机外参\n");
	printf("  p  标定计算\n");
	printf("  q  退出\n");
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
	//深度图
	/*camera_ins.FLX = 366.685;
	camera_ins.FLY = 366.685;
	camera_ins.PPX = 256.52;
	camera_ins.PPY = 208.1;*/
	//颜色图
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

