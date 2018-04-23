/*
*两步法手眼标定
*原理：（手在眼外标定） 标定板固定在手臂末端，让机器人走两个位置，保证这两个位置都能使得摄像头看到标定板
*输入：①两个位置手臂末端基于世界坐标系的变换矩阵
*     ②两个位置相机外参
*/

#include <iostream>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>  
#include <string>
#include <Eigen/Core>
#include "OpenNIKinect.h"
#include "CalChessboard.h"
#include <math.h>

using namespace Eigen;
using namespace std;
using namespace openni;
using namespace cv;

void hMirrorTrans(const Mat &src, Mat &dst)
{
	dst.create(src.rows, src.cols, src.type());

	int rows = src.rows;
	int cols = src.cols;

	switch (src.channels())
	{
	case 1:   //1通道比如深度图像
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
	case 3:   //3通道比如彩色图像
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

Matrix4f readf(string str){
	Matrix4f mat;

	ifstream  file(str);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> mat(i, j);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << mat(i, j) << ' ';
		}
		cout << endl;
	}
	file.close();

	return mat;
}

Mat readfcv(string str){
	Mat mat(4, 4, CV_64FC1);
	float x;
	ifstream  file(str);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			//file >> mat(i, j);
			file >> x;
			mat.at<double>(i, j) = x;
		}
	}

	file.close();
	//cout << mat << endl;
	return mat;
}

Mat skew(Mat A){
	CV_Assert(A.cols == 1 && A.rows == 3);
	Mat B(3, 3, CV_64FC1);

	B.at<double>(0, 0) = 0.0;
	B.at<double>(0, 1) = -A.at<double>(2, 0);
	B.at<double>(0, 2) = A.at<double>(1, 0);

	B.at<double>(1, 0) = A.at<double>(2, 0);
	B.at<double>(1, 1) = 0.0;
	B.at<double>(1, 2) = -A.at<double>(0, 0);

	B.at<double>(2, 0) = -A.at<double>(1, 0);
	B.at<double>(2, 1) = A.at<double>(0, 0);
	B.at<double>(2, 2) = 0.0;

	return B;
}

Vector3d Rotate2AxialAngle(Matrix3f m){
	Vector3d v, v0;
	float q;
	v0[0] = m(2, 1) - m(1, 2);
	v0[1] = m(0, 2) - m(2, 0);
	v0[2] = m(1, 0) - m(0, 1);
	q = acos((m(0, 0) + m(1, 1) + m(2, 2) - 1) / 2);
	v = (1 / sin(q))*v0;

	cout << "vector: " << v << endl;
	return v;
}

void calibration(){
	double camD1[9] = { 546.49, 0, 319.5,
		0, 546.49, 239.5,
		0, 0, 1 };
	double distCoeffD1[5] = { /*1.3365431359795218e-001, -8.8064028564220309e-001, 0., 0.,
							  1.8548424360444218e+000*/ -3.7375092947566635e-002, 1.5210414013521094e+000, 0., 0.,
							  -7.3090578619797810e+000 };
	/*double camD2[9] = { 532.49, 0, 319.5,
		0, 532.49, 239.5,
		0, 0, 1 };
		double distCoeffD2[5] = { 4.7049745556333566e-002, -1.7143637680137072e-001, 0., 0.,
		-2.0746566883707779e-001 };*/
	double markerRealSize = 3;

	CalChessboard cal1 = CalChessboard(camD1, distCoeffD1, markerRealSize);
	//CalChessboard cal2 = CalChessboard(camD2, distCoeffD2, markerRealSize);
	std::string imgpath, calibFile;
	imgpath = "..\\data\\color1\\2.png";
	calibFile = "..\\data\\calibration\\cal1.txt";
	cal1.calprocess(imgpath, calibFile, 1);

	imgpath = "..\\data\\color2\\2.png";
	calibFile = "..\\data\\calibration\\cal2.txt";
	cal1.calprocess(imgpath, calibFile, 2);

}

void collection(){
	double in1[4] = { 319.5, 239.5, 536.24, 536.24 };

	Mat rgbMat, depthMat;

	string savePath1, savePath2;
	string rgbfilepath1, dfilepath1, rgbfilepath2, dfilepath2;

	OpenNIKinect m_kinect(in1);
	m_kinect.initEngine(0);
	rgbfilepath1 = "..\\data\\color1\\";
	rgbfilepath2 = "..\\data\\color2\\";

	namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
	namedWindow("Color Image", CV_WINDOW_AUTOSIZE);

	int i = 1, j = 1;
	while (true)
	{
		depthMat = m_kinect.getDepthImage();
		rgbMat = m_kinect.getRGBImage();

		imshow("Color Image", rgbMat);
		imshow("Depth Image", depthMat);
		
		if (waitKey(1) == '1')//数字1
		{
			//imwrite(dfilepath1 + std::to_string(i) + ".png", depthMat);
			imwrite(rgbfilepath1 + std::to_string(i) + ".png", rgbMat);
			cout << "Success to save the img" << i << " in color1" << endl;
			i++;
		}
		else if (waitKey(1) == '2')//数字2
		{
			//imwrite(dfilepath2 + std::to_string(j) + ".png", depthMat);
			imwrite(rgbfilepath2 + std::to_string(j) + ".png", rgbMat);
			cout << "Success to save the img" << j << " in color2"<< endl;
			j++;
		}
		else if (waitKey(1) == 27){
			break;
		}
			
	}

	//if (save_or == 'y'){
	//	if (sig == '1'){
	//		rgbfilepath1 = "..\\data\\color1\\3.png";
	//		dfilepath1 = "..\\data\\depth1\\3.png";
	//		savePath1 = "..\\data\\pcd1\\1.pcd";
	//		cout << "start to save pcd..." << endl;
	//		//m_kinect1.PCDsynthesis(rgbfilepath1, dfilepath1, savePath1);
	//		m_kinect2.PCDsynthesis(rgbfilepath1, dfilepath1, savePath1);
	//	}
	//	else if (sig == '2'){
	//		rgbfilepath2 = "..\\data\\color2\\3.png";
	//		dfilepath2 = "..\\data\\depth2\\3.png";
	//		savePath2 = "..\\data\\pcd2\\1.pcd";
	//		cout << "start to save pcd..." << endl;
	//		m_kinect2.PCDsynthesis(rgbfilepath2, dfilepath2, savePath2);
	//	}
	//}

	cout << "END" << endl;

}

void process(){
	/*
	Matrix3f R;
	Vector3d t;
	Vector3d tC, tA;
	Matrix4f matA1, matA2, matC1, matC2, mat;
	Matrix4f matA, matC;
	Matrix3f mat3A, mat3C;
	matC1 = readf("..//data//calibration//cal1.txt");
	matC2 = readf("..//data//calibration//cal2.txt");
	matA1 = readf("..//data//calibration//robotMat1.txt");
	matA2 = readf("..//data//calibration//robotMat2.txt");
	matA = matA2.inverse()*matA1;
	matC = matC2*matC1.inverse();

	mat3A << matA(0, 0), matA(0, 1), matA(0, 2),
		matA(1, 0), matA(1, 1), matA(1, 2),
		matA(2, 0), matA(2, 1), matA(2, 2);

	mat3C << matC(0, 0), matC(0, 1), matC(0, 2),
		matC(1, 0), matC(1, 1), matC(1, 2),
		matC(2, 0), matC(2, 1), matC(2, 2);
	tC << matC(0, 3), matC(1, 3), matC(2, 3);
	tA << matA(0, 3), matA(1, 3), matA(2, 3);

	Vector3d kA, kC;
	kA = Rotate2AxialAngle(mat3A); 
	kC = Rotate2AxialAngle(mat3C);
    */
	
	Mat matA1(4, 4, CV_64FC1), matA2(4, 4, CV_64FC1), 
		matC1(4, 4, CV_64FC1), matC2(4, 4, CV_64FC1), Hmat(4, 4, CV_64FC1);
	Mat matA(4, 4, CV_64FC1), matC(4, 4, CV_64FC1);
	Mat matA_r(3, 3, CV_64FC1), matA_t(3, 1, CV_64FC1),
		matC_r(3, 3, CV_64FC1), matC_t(3, 1, CV_64FC1);

	Mat ra(3, 1, CV_64FC1), rc(3, 1, CV_64FC1);
	Mat Pra(3, 1, CV_64FC1), Prc(3, 1, CV_64FC1);
	double theta_ra, theta_rc;

	Mat tempA(3, 3, CV_64FC1);
	Mat tempb(3, 1, CV_64FC1);

	Mat A;
	Mat b;
	Mat pinA;

	matC1 = readfcv("..//data//calibration//cal1.txt");
	matC2 = readfcv("..//data//calibration//cal2.txt");
	matA1 = readfcv("..//data//calibration//robotMat1.txt");
	matA2 = readfcv("..//data//calibration//robotMat2.txt");

	matA = matA2.inv()*matA1;
	matC = matC2*matC1.inv();

	CV_Assert(matA.size() == matC.size());

	matA(Rect(0, 0, 3, 3)).copyTo(matA_r);
	matC(Rect(0, 0, 3, 3)).copyTo(matC_r);
	
	

	//利用罗德里格斯变换将旋转矩阵变成旋转向量
	Rodrigues(matA_r, ra);
	Rodrigues(matC_r, rc);
	//修正的罗德里格斯参数表示姿态变化
	theta_ra = norm(ra);
	theta_rc = norm(rc);
	Pra = 2 * sin(theta_ra / 2)*(ra / theta_ra);
	Prc = 2 * sin(theta_rc / 2)*(rc / theta_rc);

	//反对称运算
	Mat m_pr(3, 1, CV_64FC1);
	m_pr = Pra + Prc;
	tempA = skew(m_pr);
	/*tempA.at<double>(0, 0) = 0;
	tempA.at<double>(0, 1) = -m_pr.at<double>(0, 0);
	tempA.at<double>(0, 2) = m_pr.at<double>(1, 0);
	tempA.at<double>(1, 0) = m_pr.at<double>(2, 0);
	tempA.at<double>(1, 1) = 0;
	tempA.at<double>(1, 2) = -m_pr.at<double>(0, 0);
	tempA.at<double>(2, 0) = -m_pr.at<double>(1, 0);
	tempA.at<double>(2, 1) = m_pr.at<double>(2, 0);
	tempA.at<double>(2, 2) = 0;*/

	tempb = Prc - Pra;

	//A.push_back(tempA);
	//b.push_back(tempb);

	//Compute rotation
	Mat Pcg_prime(3, 1, CV_64FC1);
	Mat Pcg(3, 1, CV_64FC1);
	Mat PcgTrs(1, 3, CV_64FC1);

	Mat Rcg(3, 3, CV_64FC1);
	Mat eyeM = Mat::eye(3, 3, CV_64FC1);

	//invert(A, pinA, DECOMP_SVD);
	invert(tempA, pinA, DECOMP_SVD);

	//Pcg_prime = pinA * b;
	Pcg_prime = pinA * tempb;
	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	PcgTrs = Pcg.t();
	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation
	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);

	//Mat AA;
	//Mat bb;
	Mat pinAA;

	Mat Tcg(3, 1, CV_64FC1);

	matA(Rect(0, 0, 3, 3)).copyTo(matA_r);
	matC(Rect(0, 0, 3, 3)).copyTo(matC_r);
	matA(Rect(3, 0, 1, 3)).copyTo(matA_t);
	matC(Rect(3, 0, 1, 3)).copyTo(matC_t);


	tempAA = matA_r - eyeM;
	tempbb = Rcg * matC_t - matA_t;

	//AA.push_back(tempAA);
	//bb.push_back(tempbb);

	/*invert(AA, pinAA, DECOMP_SVD);
	Tcg = pinAA * bb;*/
	invert(tempAA, pinAA, DECOMP_SVD);
	Tcg = pinAA * tempbb;

	cout << Tcg << endl;
	
	Rcg.copyTo(Hmat(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hmat(Rect(3, 0, 1, 3)));
	Hmat.at<double>(3, 0) = 0.0;
	Hmat.at<double>(3, 1) = 0.0;
	Hmat.at<double>(3, 2) = 0.0;
	Hmat.at<double>(3, 3) = 1.0;

	std::ofstream ofsCalib("..//data//calibration//cal.txt");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << Hmat.at<double>(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();

	cout << Hmat << endl;
}

void printMenu() {
	printf("Commands:\n");
	printf("  g  获得标定图片图片\n");
	printf("  c  计算相机外参\n");
	printf("  p  标定计算\n");
	printf("  q  退出\n");
}

int main(){
	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'g':
			collection();
			break;
		case 'c':
			calibration();
			break;
		case 'p':
			process();
			break;
		case 'q':
			running = false;
			break;
		}
	}
	
	//system("pause");
	return 0;
}
