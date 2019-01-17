/*
* Copyright (c) 2019 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 来源：
*			皮带上快递箱子、包裹分割测量。
*			每帧图像保存为一个bin文件，按序存放320*240图片数据，16位深度。
*
*	 处理过程：
*			读入一个bin文件，重新处理为一张320*240图片，16位深度。
*			深度图像处理经滤波、畸变矫正后，转PointCloud点云。
*			保存pcd格式点云文件。
*
*/

#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <list>
#include <conio.h>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl\visualization\cloud_viewer.h>

using namespace cv;
using namespace std;
using namespace pcl;

#define   Img_width   (320)
#define   Img_height  (240)
#define   THRESHOLD	   3000
#define   FX  286.6034
#define   FY  287.3857	
#define   CX  176.2039
#define   CY  126.5788	
#define   K1  -0.12346
#define   K2  0.159423

Mat imgDev;

//图片滤波
Mat imageFilter(Mat src, float threshold)
{
	imgDev = Mat::zeros(240, 320, CV_32F);
	Mat rst = Mat::zeros(src.size(), CV_16U);
	for (int i = 1; i < src.size().height - 1; i++)
	{
		for (int j = 1; j < src.size().width - 1; j++)
		{
			int dist[9];
			dist[0] = src.at<ushort>(i - 1, j - 1);
			dist[1] = src.at<ushort>(i - 1, j);
			dist[2] = src.at<ushort>(i - 1, j + 1);
			dist[3] = src.at<ushort>(i, j - 1);
			dist[4] = src.at<ushort>(i, j);
			dist[5] = src.at<ushort>(i, j + 1);
			dist[6] = src.at<ushort>(i + 1, j - 1);
			dist[7] = src.at<ushort>(i + 1, j);
			dist[8] = src.at<ushort>(i + 1, j + 1);

			//计算平均值
			float aveg = 0;
			for (int k = 0; k < 9; k++)
			{
				aveg += dist[k];
			}
			aveg = aveg / 9;

			//计算均方差
			float deviation = 0;
			for (int k = 0; k < 9; k++)
			{
				deviation += (dist[k] - aveg) * (dist[k] - aveg);
			}
			deviation = deviation / 9;
			imgDev.at<float>(i, j) = deviation;

			//方差大于阈值置零
			if (deviation > threshold)
			{
				rst.at<ushort>(i, j) = 0;
			}
			else
				rst.at<ushort>(i, j) = src.at<ushort>(i, j);
		}
	}

	return rst.clone();
}

//畸变矫正
//输入： 待矫正的图片
//输出： 校正后的图片
Mat imageUndist(Mat src)
{
	//畸变矫正
	Mat img;

	//内参矩阵
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = FX;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = CX;
	cameraMatrix.at<double>(1, 1) = FY;
	cameraMatrix.at<double>(1, 2) = CY;
	cameraMatrix.at<double>(2, 2) = 1;

	//畸变参数
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = K1;
	distCoeffs.at<double>(1, 0) = K2;
	distCoeffs.at<double>(2, 0) = 0;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;

	Size imageSize = src.size();
	Mat map1, map2;
	//参数1：相机内参矩阵
	//参数2：畸变矩阵
	//参数3：可选输入，第一和第二相机坐标之间的旋转矩阵
	//参数4：校正后的3X3相机矩阵
	//参数5：无失真图像尺寸
	//参数6：map1数据类型，CV_32FC1或CV_16SC2
	//参数7、8：输出X/Y坐标重映射参数
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//计算畸变映射
	//参数1：畸变原始图像
	//参数2：输出图像
	//参数3、4：X\Y坐标重映射
	//参数5：图像的插值方式
	//参数6：边界填充方式
	cv::remap(src, img, map1, map2, INTER_LINEAR);																	//畸变矫正
	return img.clone();
}

//pcd文件保存
void pcdSave(Mat mImageDepth)
{
	int imgWidth = mImageDepth.size().width;
	int imgHeight = mImageDepth.size().height;

	PointCloud<PointXYZ> pointCloud;
	for (int i = 0; i < imgHeight; i++)
	{
		for (int j = 0; j < imgWidth; j++)
		{
			float picDist = sqrt((i - imgHeight / 2.0)*(i - imgHeight / 2.0) + (j - imgWidth / 2.0)*(j - imgWidth / 2.0));	//图像上点到中心的像素点个数
			float picAngle = atan2(i - imgHeight / 2.0, j - imgWidth / 2.0);												//图像上x,y和中心点角度关系
			float angle = atan(sqrt((j - imgWidth / 2.0)*(j - imgWidth / 2.0) / FX / FX + (i - imgHeight / 2.0)*(i - imgHeight / 2.0) / FY / FY));
			float dist = mImageDepth.at<ushort>(i, j) / 3000.0 * 125;				//原始图像深度

			PointXYZ p;
			p.z = dist*cos(angle);									//坐标变换后的深度
			p.x = dist*sin(angle)*cos(picAngle);
			p.y = dist*sin(angle)*sin(picAngle);

			pointCloud.points.push_back(p);
		}
	}

	//点云可视化
	PointCloud<PointXYZ>::Ptr cloud = pointCloud.makeShared();
	visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{

	}


	//保存点云
	pointCloud.height = imgHeight;
	pointCloud.width = imgWidth;
	io::savePCDFileBinary("pcl.pcd", pointCloud);
}

//鼠标点击相应事件
void on_mouse(int event, int x, int y, int flags, void* userdata)
{
	Mat hh;
	char file[64];
	hh = *(Mat*)userdata;
	Point p(x, y);
	if (event == EVENT_LBUTTONDOWN)
	{
		//sprintf(file, "(%d)", (int)(hh.at<uchar>(p)));
		//putText(src, file, p, FONT_HERSHEY_PLAIN, 1, Scalar(255), 1, 8);
		printf("b=%0.3f\n", hh.at<ushort>(p) / 30000.0*12.5);
		printf("n=%d\n", hh.at<ushort>(p));
		printf("d=%f\n", imgDev.at<float>(p));
		// circle(hh, p, 2, Scalar(255), 3);
	}
	else if (event == CV_EVENT_RBUTTONDOWN)
	{
		circle(hh, p, 2, Scalar(255), 3);
	}


}

//读取Bin文件转Mat
//bin内容数值0-30000
Mat  getDataFromfile(const char * filename)
{

	ifstream myi(filename, ios::binary);
	unsigned short * imagebuf = new unsigned short[Img_width*Img_height];
	Mat outmat(Img_height, Img_width, CV_16UC1);
	if (!myi.is_open())
	{
		return outmat;
	}
	//读取原始数据
	myi.read((char*)imagebuf, Img_width*Img_height*sizeof(unsigned short));
	myi.close();

	//行列数据转Mat
	for (int i = 0; i < Img_height; i++)
	{
		for (int j = 0; j < Img_width; j++)
		{
			if (imagebuf[j + i * Img_width] < 30000)
				outmat.at<ushort>(i, j) = imagebuf[j + i * Img_width];
			else
				outmat.at<ushort>(i, j) = 0;
		}
	}

	delete[] imagebuf;
	return outmat;
}

int _tmain(int argc, _TCHAR* argv[])
{


	bool bsave = 0;
	char imgfilename85[_MAX_PATH];
	char outfilename[_MAX_PATH];
	_TCHAR inipath[_MAX_PATH];
	_TCHAR  depthcfgpath[_MAX_PATH];

	_TCHAR drive[_MAX_DRIVE];
	_TCHAR dir[_MAX_DIR];
	//	char	szdepthcfgpath[_MAX_PATH];
	_tsplitpath(argv[0], drive, dir, NULL, NULL);
	sprintf(imgfilename85, "D:\\样本\\箱子拼接分割\\save\\save\\110915\\CAM2_4.bin");

	Mat outmat85 = getDataFromfile(imgfilename85);

	if (outmat85.empty())
	{
		cout << "err when open file!" << endl;
		return -1;
	}


	//滤波
	outmat85 = imageFilter(outmat85, THRESHOLD);
	//畸变矫正
	outmat85 = imageUndist(outmat85);
	//转伪彩色图
	Mat zip;
	outmat85.convertTo(zip, CV_8U, 25.0 / 3000, 0);
	applyColorMap(zip, zip, COLORMAP_HSV);

	namedWindow("win1", 0);
	imshow("win1", zip);

	//设置鼠标点击事件
	setMouseCallback("win1", on_mouse, &outmat85);
	waitKey(0);


	//保存PCD文件
	pcdSave(outmat85);

	return 0;
}



