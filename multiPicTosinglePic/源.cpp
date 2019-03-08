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


//原始图像位置
#define FILEDIR		"D:\\样本\\公交数据20190228\\2019-0228(9)\\"
// 分割后的图片保存位置
#define FILEDIRNEW	"E:\\公交采集0228\\2019-0228(9)\\"
//开始文件名称 "NULL"从头开始
#define START_FILE_NAME	"NULL"
//保存彩色图像 0不保存
#define SAVE_COLORMAP 1
//保存深度图像 0不保存
#define SAVE_DEPTH    1
//间隔显示帧数 0不显示
#define SHOW_NUM 0
//每幅图像内包含的帧数
#define PIC_NUM 30


int number = 0;

void imageCut(char* dir, char* file)
{
	char fileDir[300];
	char fileDirNew[300];

	strcpy(fileDirNew, FILEDIRNEW);

	strcpy(fileDir, dir);		//合成文件目录
	strcat(fileDir, file);

	Mat ori;
	try{
		ori = imread(fileDir, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	}
	catch (exception e)
	{
		sprintf("Can not open %s%s!\n", dir, file);
		return;
	}
	//检查图片文件
	if (ori.size().width == 0 || ori.size().height == 0)
	{
		printf("Open %s%s error!\n", dir, file);
		return;
	}

	Mat depth = Mat(Size(640, 480), CV_16U);
	MatConstIterator_<ushort> it = ori.begin<ushort>();
	for (int k = 0; k < PIC_NUM; k++)
	{
		for (int i = 0; i < 480; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				//depth.at<ushort>(i, j) = ori.at<ushort>(k, i * 640 + j);
				depth.at<ushort>(i, j) = *(it++);
			}
		}

		Mat colorMat;
		char imgFile[200];
		//显示伪彩色图片
		if (SHOW_NUM != 0 && number % SHOW_NUM == 0)
		{
			if (colorMat.size().height == 0)
			{
				depth.convertTo(colorMat, CV_8U, 1.0 / 16, 0);
				applyColorMap(colorMat, colorMat, COLORMAP_JET);	  //将mImageDepth变成看似的深度图(伪彩色图)
			}
			imshow("win", colorMat);
			waitKey(1);
		}
		if (SAVE_COLORMAP)
		{
			if (colorMat.size().height == 0)
			{
				depth.convertTo(colorMat, CV_8U, 1.0 / 16, 0);
				applyColorMap(colorMat, colorMat, COLORMAP_JET);	  //将mImageDepth变成看似的深度图(伪彩色图)
			}
			sprintf(imgFile, "%scolor-%06d.png", fileDirNew, number);
			imwrite(imgFile, colorMat);
		}
		if (SAVE_DEPTH)
		{
			//保存结果文件名
			sprintf(imgFile, "%s%06d.png", fileDirNew, number);
			imwrite(imgFile, depth);
		}
		number++;
	}
}

void listFiles(const char * dir)
{
	int saveflag = 0;
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
			//判断起始文件
			if (!strcmp(findData.name,START_FILE_NAME))
			{
				saveflag = 1;
			}
			if (!strcmp("NULL", START_FILE_NAME))
				saveflag = 1;
		

			//处理文件
			if (saveflag)
			{
				strcpy(dirNew, dir);
				imageCut(dirNew, findData.name);
			}
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