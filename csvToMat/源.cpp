/*
* Copyright (c) 2019 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 来源：
*			EPC软件默认保存格式CSV文件。
*			每帧数据(320+8)*(240+12)
*
*	 处理过程：
*			读入csv文件，按行列分割成单帧数据
*			每帧数据转存Mat格式（CV_16U）
*			转存PCD点云
*
*/


#include <opencv2\opencv.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <pcl\visualization\cloud_viewer.h>


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

#define   LEFT_BLANK	0
#define	  RIGHT_BLANK	0
#define	  TOP_BLANK		0
#define   BOTTOM_BLANK	0

#define   IMAGE_SAVE	0
#define   IMAGE_SHOW    0
#define   CLOUD_SAVE	1
#define   CLOUD_SHOW	0


using namespace std;
using namespace cv;
using namespace pcl;







//图片滤波
Mat imageFilter(Mat src, float threshold)
{
	Mat imgDev = Mat::zeros(240, 320, CV_32F);
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

//提取全路径中的文件后缀）
std::string getFileType(std::string file_name)
{
	std::string subname;
	for (auto i = file_name.end() - 1; *i != '.'; i--)
	{
		subname.insert(subname.begin(), *i);
	}

	return subname;
}

void imageConvert(char* dir, char* file)
{
	if (getFileType(file) != "csv")
		return;

	char fileDir[300];
	char fileDirNew[300];

	strcpy(fileDirNew, FILEDIRNEW);

	strcpy(fileDir, dir);
	strcat(fileDir, file);

	//打开文件
	FILE *p_openFile = NULL;
	p_openFile = fopen(fileDir, "r");			//打开文件
	if (!p_openFile)
	{
		printf("文件不存在！\r\n");
		exit(-1);
	}

	char c;
	int pixelDep = 0;					//像素点数值
	int pixelCounter = 0;				//行像素计数
	int rowCounter = 0;					//行计数
	int frameCounter = 0;				//帧计数
	uint16_t depth[240][320];
	printf("文件打开成功！\r\n");
	while (1)
	{
		c = fgetc(p_openFile);

		if (c == -1)
		{
			printf("完成！\r\n");
			break;
		}

		if ((pixelCounter >= LEFT_BLANK && pixelCounter < 320 + LEFT_BLANK) && (rowCounter >= TOP_BLANK && rowCounter < 240 + TOP_BLANK))
		{
			if (c == 'N' || c == 'a')
				pixelDep = 0;
			else if (c != ',' && c != '\n')
				pixelDep = pixelDep * 10 + c - '0';

			if (c == ',')
			{
				depth[rowCounter - TOP_BLANK][pixelCounter - LEFT_BLANK] = pixelDep;
			}
		}


		if (c == ',')
		{

			pixelDep = 0;
			pixelCounter++;
		}

		//if (c >= '0' && c <= '9')
		//c = c * 10 + c - '0';

		if (c == '\n')
		{
			pixelCounter = 0;
			rowCounter++;
			if (rowCounter > 239 + TOP_BLANK + BOTTOM_BLANK)		//新帧
			{
				rowCounter = 0;
				frameCounter++;
				printf("成功切割%d\r\n", frameCounter);
				Mat src_1(240, 320, CV_16UC1, Scalar(0));
				for (int i = 0; i < 240; i++)
				{
					for (int j = 0; j < 320; j++)
					{
						if (depth[240 - i][j] > 30000)
							src_1.at<ushort>(i, j) = 0;
						else
							src_1.at<ushort>(i, j) = depth[240 - i][j];
					}
				}

				if (IMAGE_SAVE)
				{
					//图像保存
					sprintf(fileDirNew, "%s%s-%d.png", FILEDIRNEW, file, frameCounter);
					imwrite(fileDirNew, src_1);
				}
				if (IMAGE_SHOW)
				{
					//伪彩色显示
					Mat color;
					src_1.convertTo(color, CV_8U);
					applyColorMap(color, color, COLORMAP_JET);
					imshow("win", color);

					waitKey(1);
				}
					

				sprintf(fileDirNew, "%s%s-%d.pcd", FILEDIRNEW, file, frameCounter);
				//畸变矫正
				Mat undist = imageUndist(src_1);
				//点云变换保存
				pcdSave(undist, fileDirNew);
			}
		}
	}
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

void main()
{

	char* file = FILEDIR;

	listFiles(file);
}
