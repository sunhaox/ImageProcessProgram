/*
* Copyright (c) 2019 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 来源：
*			EPC官方原始保存的伪彩色图像。根据伪彩色规律转为深度图像。
*
*	 处理过程：
*			读入伪彩色PNG图像，根据伪彩色规则转换为12位深度数据。
*			把深度数据转为点云数据，并保存pcd文件。
*
*/


#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <stdio.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <io.h>
#include <pcl\io\pcd_io.h>
#include <pcl\point_cloud.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\filters\statistical_outlier_removal.h>

using namespace std;
using namespace cv;
using namespace pcl;

#define RESOLUTION	1022
#define   FX  296.041247147697
#define   FY  296.839046024906	
#define   CX  178.091392048995
#define   CY  129.480179021482	
#define   K1  -0.145637046815776
#define   K2  0.276467521132465


Mat remapMatToDepthMat(Mat imgOrigin)
{
	Mat imgDepth = Mat::zeros(Size(320, 240), CV_16U);
	Mat lut = imread("lut.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

	for (int h = 6; h < imgOrigin.size().height - 6; h++)
	{
		for (int w = 4; w < imgOrigin.size().width - 4; w++)
		{
			if (imgOrigin.at<Vec3b>(h, w)[0] == 128 && imgOrigin.at<Vec3b>(h, w)[1] == 0 && imgOrigin.at<Vec3b>(h, w)[2] == 255)
			{
				imgDepth.at<ushort>(h - 6, w - 4) = 0;
				//printf("过曝点(%d,%d)\n", w-4, h-6);
				continue;
			}

			if (imgOrigin.at<Vec3b>(h, w)[0] == 0 && imgOrigin.at<Vec3b>(h, w)[1] == 0 && imgOrigin.at<Vec3b>(h, w)[2] == 0)
			{
				imgDepth.at<ushort>(h - 6, w - 4) = 0;
				//printf("无效点(%d,%d)\n", w - 4, h - 6);
				continue;
			}


			int img_1 = imgOrigin.at<Vec3b>(h, w)[0];
			int img_2 = imgOrigin.at<Vec3b>(h, w)[1];
			int img_3 = imgOrigin.at<Vec3b>(h, w)[2];

			if (img_3 == 0)
			{
				if (img_2 == 0)
				{
					imgDepth.at<ushort>(h - 6, w - 4) = img_1 - 127;
				}
				else
				{
					imgDepth.at<ushort>(h - 6, w - 4) = 128 + img_2;
				}
			}
			else
			{
				if (img_2 == 255)
				{
					imgDepth.at<ushort>(h - 6, w - 4) = 383 + img_3;
				}
				else
				{
					imgDepth.at<ushort>(h - 6, w - 4) = 638 + (255 - img_2) * 3 / 2;
				}
			}
			imgDepth.at<ushort>(h - 6, w - 4) = RESOLUTION - imgDepth.at<ushort>(h - 6, w - 4);
		}
	}

	return imgDepth.clone();
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
			float dist = mImageDepth.at<ushort>(i, j) / 300.0 * 125;				//原始图像深度

			PointXYZ p;
			p.z = dist*cos(angle);									//坐标变换后的深度
			p.x = dist*sin(angle)*cos(picAngle);
			p.y = dist*sin(angle)*sin(picAngle);

			pointCloud.points.push_back(p);
		}
	}

	//滤波
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	cloud = pointCloud.makeShared();
	//创建滤波器对象
	StatisticalOutlierRemoval<PointXYZ> sor;
	//该算法迭代整个输入两次：在第一次迭代期间，它将计算每个点与其最近的k个邻居之间的平均距离。 可以使用setMeanK（）设置k的值。
	//接下来，计算所有这些距离的平均值和标准偏差，以便确定距离阈值。
	//距离阈值将等于：mean + stddev_mult * stddev。 可以使用setStddevMulThresh（）设置标准偏差的乘数。
	//在下一次迭代期间，如果它们的平均相邻距离分别低于或高于该阈值，则点将被分类为内部或异常值。
	//为每个查询点找到的邻居将在setInputCloud（）的所有点中找到，而不仅仅是由setIndices（）索引的那些点。 
	//setIndices（）方法仅索引将作为搜索查询点迭代的点。
	sor.setInputCloud(cloud);
	sor.setMeanK(50);					//设置与50个邻居计算平均距离
	sor.setStddevMulThresh(1.0);		//设置标准差放大系数
	sor.filter(*cloud_filtered);

	int max = 0;
	//点云可视化
	PointCloud<PointXYZRGB> cloud_color;
	for (int i = 0; i < cloud_filtered->size(); i++)
	{
		PointXYZRGB p;
		p.x = cloud_filtered->points.at(i).x;
		p.y = cloud_filtered->points.at(i).y;
		p.z = cloud_filtered->points.at(i).z;

		if (max < p.z)
			max = p.z;
		if (p.z < 100)
			continue;

		p.z -= 200;
		if (p.z < 64)
		{
			p.r = 255;
			p.g = p.z*4;
			p.b = 0;
		}
		else if (p.z < 128)
		{
			p.r = 512 - p.z * 4;
			p.g = 255;
			p.b = 0;
		}
		else if (p.z < 192)
		{
			p.r = 0;
			p.g = 255;
			p.b = p.z*4 - 512;
		}
		else if (p.z < 256)
		{
			p.r = p.z * 4 - 768;
			p.g = 255;
			p.b = 255;
		}
		else
		{
			p.r = 100;
			p.g = 100;
			p.b = 100;
		}
		

		cloud_color.points.push_back(p);
	}
	printf("max is %d", max);
	visualization::CloudViewer viewer("Cloud Viewer");
	
	PointCloud<PointXYZRGB>::Ptr cloud_show = cloud_color.makeShared();
	viewer.showCloud(cloud_show);
	while (!viewer.wasStopped())
	{

	}


	//保存点云
	//cloud_filtered->height = imgHeight;
	//cloud_filtered->width = imgWidth;
	//io::savePCDFileBinary("pcl.pcd", *cloud_filtered);
	cloud_color.height = 1;
	cloud_color.width = cloud_color.size();
	io::savePCDFileASCII("pcl.pcd", cloud_color);
}


int main()
{

	Mat srcImg = imread("D:\\样本\\叉车托盘叉取\\原始数据\\q-1_020.png");

	Mat depthImg = remapMatToDepthMat(srcImg);

	Mat undistImg = imageUndist(depthImg);

	pcdSave(undistImg);

	return 0;
}