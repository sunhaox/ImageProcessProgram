#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


//读取文件路径
#define FILEDIR		"D:\\pcd\\csv\\"
//保存文件路径
#define FILEDIRNEW	"D:\\pcd\\pcd\\"

#define   Img_width   (320)
#define   Img_height  (240)
#define   FX  196.638316
#define   FY  197.0980708
#define   CX  167.7838572
#define   CY  124.3448776
#define   K1  -0.289046233
#define   K2  0.091629024
#define   THRESHOLD	   3000

#define   IMAGE_SHOW    0
#define   CLOUD_SAVE	1
#define   CLOUD_SHOW	0


using namespace std;
using namespace cv;
using namespace pcl;


//将 CV_16UC1 深度图 转换成伪彩色图
Mat gray2rainbow(const Mat& scaledGray, int min, int max)
{
	Mat outputRainbow(scaledGray.size(), CV_8UC3);       //初始化了一个outputRainbow的彩色图像
	unsigned short grayValue;
	float tempvalue;

	float par = (float)255 / (max - min);


	for (int y = 0; y < scaledGray.rows; y++)
	for (int x = 0; x < scaledGray.cols; x++)
	{

		grayValue = scaledGray.at<ushort>(y, x);
		if ((grayValue > 0) && (grayValue < min))        //可能会出现找到的min并不是真正的最小值
		{
			tempvalue = (float)min;
		}
		else if (grayValue > max)                     //也可能会出现找到的max并不是真正的最大值
		{
			tempvalue = 0;
		}
		else
		{
			tempvalue = (float)(grayValue - min);
		}
		tempvalue = tempvalue*par;          //为了把深度值规划到(0~255之间)
		/*
		* color    R   G   B   gray
		* red      255 0   0   255
		* orange   255 127 0   204
		* yellow   255 255 0   153
		* green    0   255 0   102
		* cyan     0   255 255 51
		* blue     0   0   255 0
		*/

		Vec3b& pixel = outputRainbow.at<Vec3b>(y, x);
		tempvalue = 256 - tempvalue;

		if ((tempvalue <= 0) || (tempvalue >= 255))
		{
			pixel[0] = 0;
			pixel[1] = 0;
			pixel[2] = 0;
		}
		else if (tempvalue <= 51)
		{
			pixel[0] = 255;
			pixel[1] = (unsigned char)(tempvalue * 5);
			pixel[2] = 0;
		}
		else if (tempvalue <= 102)
		{
			tempvalue -= 51;
			pixel[0] = 255 - (unsigned char)(tempvalue * 5);
			pixel[1] = 255;
			pixel[2] = 0;
		}
		else if (tempvalue <= 153)
		{
			tempvalue -= 102;
			pixel[0] = 0;
			pixel[1] = 255;
			pixel[2] = (unsigned char)(tempvalue * 5);
		}
		else if (tempvalue <= 204)
		{
			tempvalue -= 153;
			pixel[0] = 0;
			pixel[1] = 255 - static_cast<unsigned char>(tempvalue * 128.0 / 51 + 0.5);
			pixel[2] = 255;
		}
		else if (tempvalue < 255)
		{
			tempvalue -= 204;
			pixel[0] = 0;
			pixel[1] = 127 - static_cast<unsigned char>(tempvalue * 127.0 / 51 + 0.5);
			pixel[2] = 255;
		}
	}

	return outputRainbow;
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
void pcdSave(Mat mImageDepth, char* saveFile)
{
	int imgWidth = mImageDepth.size().width;
	int imgHeight = mImageDepth.size().height;

	PointCloud<PointXYZ> pointCloud;
	for (int i = 0; i < imgHeight; i++)
	{
		for (int j = 0; j < imgWidth; j++)
		{
			if (mImageDepth.at<ushort>(i, j) > 30000)
				continue;

			if (mImageDepth.at<ushort>(i, j) < 10)
				continue;

			float picDist = sqrt((i - imgHeight / 2.0)*(i - imgHeight / 2.0) + (j - imgWidth / 2.0)*(j - imgWidth / 2.0));	//图像上点到中心的像素点个数
			float picAngle = atan2(FX*(i - imgHeight / 2.0), FY*(j - imgWidth / 2.0));												//图像上x,y和中心点角度关系
			float angle = atan(sqrt((j - imgWidth / 2.0)*(j - imgWidth / 2.0) / FX / FX + (i - imgHeight / 2.0)*(i - imgHeight / 2.0) / FY / FY));
			float dist = mImageDepth.at<ushort>(i, j) / 3000.0 * 125;				//原始图像深度



			PointXYZ p;
			p.z = dist*cos(angle);									//坐标变换后的深度
			p.x = dist*sin(angle)*cos(picAngle);
			p.y = dist*sin(angle)*sin(picAngle);

			pointCloud.points.push_back(p);
		}
	}

	if (CLOUD_SHOW)
	{
		//点云可视化
		PointCloud<PointXYZ>::Ptr cloud = pointCloud.makeShared();
		visualization::CloudViewer viewer("Cloud Viewer");
		viewer.showCloud(cloud);
		while (!viewer.wasStopped())
		{

		}
	}


	if (CLOUD_SAVE)
	{
		//保存点云
		pointCloud.height = 1;
		pointCloud.width = pointCloud.size();
		io::savePCDFileBinary(saveFile, pointCloud);		//二进制保存
		//io::savePCDFile(saveFile, pointCloud);				//ASCII保存
	}

}

void imageConvert(char* dir, char* file)
{

	char fileDir[300];
	char fileDirNew[300];

	strcpy(fileDirNew, FILEDIRNEW);

	strcpy(fileDir, dir);
	strcat(fileDir, file);

	Mat src_1 = imread(fileDir, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);


	if (IMAGE_SHOW)
	{
		//伪彩色显示
		Mat color;
		src_1.convertTo(color, CV_8U);
		applyColorMap(color, color, COLORMAP_JET);
		imshow("win", color);

		waitKey(1);
	}


	sprintf(fileDirNew, "%s%s.pcd", FILEDIRNEW, file);
	//畸变矫正
	Mat undist = imageUndist(src_1);
	//点云变换保存
	pcdSave(undist, fileDirNew);
			
}

void listFiles(const char * dir)
{
	char dirNew[200];
	strcpy(dirNew, dir);
	strcat(dirNew, "*.*");    // 在目录后面加上"\\*.*"进行第一次搜索

	intptr_t handle;
	_finddata_t findData;

	handle = _findfirst(dirNew, &findData);
	if (handle == -1)        // 检查是否成功
		return;

	do
	{
		if (findData.attrib & _A_SUBDIR)
		{
			if (strcmp(findData.name, ".") == 0 || strcmp(findData.name, "..") == 0)
				continue;

			cout << findData.name << "\t<dir>\n";

			// 在目录后面加上"\\"和搜索到的目录名进行下一次搜索
			strcpy(dirNew, dir);
			strcat(dirNew, findData.name);
			strcat(dirNew, "\\");

			listFiles(dirNew);
		}
		else
		{
			strcpy(dirNew, dir);
			imageConvert(dirNew, findData.name);
			cout << findData.name << "\t" << findData.size << " bytes.\n";
		}
	} while (_findnext(handle, &findData) == 0);

	_findclose(handle);    // 关闭搜索句柄
}

int main()
{

	char* file = FILEDIR;

	listFiles(file);
}