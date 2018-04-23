#include "CalChessboard.h"


CalChessboard::CalChessboard(double camD[9], double distCoeffD[5], double &markerRealSize)
{
	camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	m_markerCorners3d.push_back(cv::Point3f(0, 0, 0));
	m_markerCorners3d.push_back(cv::Point3f(4 * markerRealSize, 0, 0));
	m_markerCorners3d.push_back(cv::Point3f(8 * markerRealSize, 0, 0));
	m_markerCorners3d.push_back(cv::Point3f(8 * markerRealSize, 5 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(4 * markerRealSize, 5 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(0, 5 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(2 * markerRealSize, 1 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(6 * markerRealSize, 1 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(6 * markerRealSize, 4 * markerRealSize, 0));
	m_markerCorners3d.push_back(cv::Point3f(2 * markerRealSize, 4 * markerRealSize, 0));
}


CalChessboard::~CalChessboard()
{
}

void CalChessboard::corner_detection(std::string &imgpath, int x){
	image_color = cv::imread(imgpath, cv::IMREAD_COLOR);
	cv::Mat image_gray;
	cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);

	std::vector<cv::Point2f> corners, corner_t;

	bool ret = cv::findChessboardCorners(image_gray,
		cv::Size(6, 9),
		corners,
		cv::CALIB_CB_ADAPTIVE_THRESH |
		CV_CALIB_CB_FAST_CHECK |
		cv::CALIB_CB_NORMALIZE_IMAGE);

	/*cv::Mat image_gray;
	cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);*/
	
	//ָ�������ؼ��������ע  
	cv::TermCriteria criteria = cv::TermCriteria(
		cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
		40,
		0.1);

	//�����ؼ��  
	cv::cornerSubPix(image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
	
	/*cornerSubPix(image_gray, corners, cv::Size(11, 11),
		cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));*/
	if (x==1)
	{
		m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[13]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[16]);
		/*m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[16]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[13]);*/
	}
	else if (x==2)
	{
		m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[13]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[16]);
		/*m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[16]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[13]);*/
	}

}


void CalChessboard::calibration(){

	for (int i = 0; i < m_markerCorners2d.size(); i++)
	{
		cv::circle(image_color, m_markerCorners2d[i], 1, cv::Scalar(255, 40*i, 255), 2);
	}
	cv::circle(image_color, m_markerCorners2d[0], 1, cv::Scalar(0, 0, 255), 2);

	cv::Mat objPM;

	cv::Mat(m_markerCorners3d).convertTo(objPM, CV_32F);

	//cv::solvePnPRansac(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);
	cv::solvePnP(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);
	
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(objPM, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);

	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
	{
		circle(image_color, projectedPoints[i], 1, cv::Scalar(255, 0, 0), -1, 8);
	}

	cv::imshow("chessboard corners", image_color);
	cv::waitKey(0);
	std::cout << "R:\n" << rvec << std::endl;
	std::cout << "T:\n" << tvec << std::endl;
}

void CalChessboard::save_transMatrix(std::string &calibFile){

	std::ofstream ofsCalib(calibFile);//�ļ�д�������ڴ�д��洢�豸
	cv::Mat trans_mat(4, 4, CV_32F), rotM;

	Rodrigues(rvec, rotM);


	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans_mat.at<float>(i, j) = rotM.at<double>(i, j);
		}
	}
	trans_mat.at<float>(0, 3) = tvec.at<double>(0, 0)/100;
	trans_mat.at<float>(1, 3) = tvec.at<double>(1, 0)/100;
	trans_mat.at<float>(2, 3) = tvec.at<double>(2, 0)/100;

	trans_mat.at<float>(3, 0) = 0;
	trans_mat.at<float>(3, 1) = 0;
	trans_mat.at<float>(3, 2) = 0;
	trans_mat.at<float>(3, 3) = 1;
	
	std::cout << "trans_mat:\n" << trans_mat << std::endl;

	trans_mat = trans_mat.inv();

	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << trans_mat.at<float>(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();
}

void CalChessboard::calprocess(std::string &imgpath, std::string &calibFile, int x){

	corner_detection(imgpath, x);
	calibration();
	save_transMatrix(calibFile);
}
