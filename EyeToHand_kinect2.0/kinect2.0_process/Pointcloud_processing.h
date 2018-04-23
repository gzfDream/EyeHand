#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef struct _Camera_Intrinsics
{
	float FLX;
	float FLY;
	float PPX;
	float PPY;
}Camera_Intrinsics;

class Pointcloud_processing
{
public:
	Pointcloud_processing();
	~Pointcloud_processing();
	// 初始化Kinect
	HRESULT init_kinect();
	bool show_pointCloud();
	Camera_Intrinsics getCameIns();
	bool getHDImage();

private:
	cv::Mat ConvertMatx(const RGBQUAD* pBuffer, int nWidth, int nHeight);
	cv::Mat ConvertMat16x(const UINT16* pBuffer, int nWidth, int nHeight);
	bool PointCloud_Synthesis(cv::Mat color_mat, cv::Mat depth_mat, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	Camera_Intrinsics getCameIns(ICoordinateMapper* pCoordinateMapper);

private:
	// Create Sensor Instance
	IKinectSensor* pSensor;
	// Retrieved Coordinate Mapper 
	ICoordinateMapper* pCoordinateMapper;
	// Retrieved Color Frame Source
	IColorFrameSource* pColorFrameSource;
	// Retrieved Depth Frame Source
	IDepthFrameSource* pDepthFrameSource;
	// Open Color Frame Reader
	IColorFrameReader* pColorReader;
	// Open Depth Frame Reader
	IDepthFrameReader* pDepthReader;
	// Retrieved Color Frame Size
	IFrameDescription* pColorDescription;
	// Retrieved Depth Frame Size
	IFrameDescription* pDepthDescription;
	//Camera Intrinsics
	Camera_Intrinsics camera_ins;

	//颜色帧和深度帧
	const int ColorWidth = 1920;
	const int ColorHeight = 1080;
	const int DepthWidth = 512;
	const int DepthHeight = 424;

};

