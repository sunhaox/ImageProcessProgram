/*
* Copyright (c) 2019 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 来源：	
*			人员计数功能采集图片使用。
*			为了保证图像采集速度和保存速度，每帧图像处理为1*(640*480)信息，
*			30帧图像处理为307200*30的png图片，16位深度。
*
*	 处理过程：
*			读入一张图片，一行数据重新处理为一张640*480图片，16位深度。
*
*/


#include <stdio.h>
#include <io.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace std;
using namespace cv;


// 分割后的图片保存位置
#define FILEDIRNEW	"D:\\公交采集\\restore\\rst\\"
#define FILEDIR		"D:\\公交采集\\restore\\data"


int number = 0;

void imageCut(char* dir, char* file)
{
	char fileDir[300];
	char fileDirNew[300];

	strcpy(fileDirNew, FILEDIRNEW);

	strcpy(fileDir, dir);		//合成文件目录
	strcat(fileDir, "\\");
	strcat(fileDir, file);

	Mat ori = imread(fileDir, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

	Mat depth = Mat(Size(640, 480), CV_16U);
	for (int k = 0; k < 30; k++)
	{
		for (int i = 0; i < 480; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				depth.at<ushort>(i, j) = ori.at<ushort>(k, i * 640 + j);
			}
		}
		//显示伪彩色图片
		Mat colorMat;
		depth.convertTo(colorMat, CV_8U, 1.0 / 16, 0);
		applyColorMap(colorMat, colorMat, COLORMAP_JET);	  //将mImageDepth变成看似的深度图(伪彩色图)
		imshow("win", colorMat);
		waitKey(1);
		//保存结果文件名
		char imgFile[200];
		sprintf(imgFile, "%s%06d.png", fileDirNew, number++);
		imwrite(imgFile, depth);
	}
}

void listFiles(const char * dir)
{
	char dirNew[200];
	strcpy(dirNew, dir);
	strcat(dirNew, "\\*.*");    // 在目录后面加上"\\*.*"进行第一次搜索

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
			strcat(dirNew, "\\");
			strcat(dirNew, findData.name);

			listFiles(dirNew);
		}
		else
		{
			strcpy(dirNew, dir);
			imageCut(dirNew, findData.name);
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