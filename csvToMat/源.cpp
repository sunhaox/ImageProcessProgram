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
*
*/


#include <stdio.h>
#include <stdint.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace std;
using namespace cv;


uint16_t depth[240][320];


void main(int argc, char *argv[])
{
	FILE *p_openFile = NULL;
	FILE *p_saveFile = NULL;
	char *fileName;
	char saveFileName[100] = { 0 };

	if (argc < 2)						//默认文件名1.scv
	{
		char* fileNameTem = "t.csv";
		fileName = fileNameTem;
	}
	else
	{
		fileName = argv[1];
	}

	p_openFile = fopen(fileName, "r");			//打开文件
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
	printf("文件打开成功！\r\n");
	while (1)
	{
		c = fgetc(p_openFile);

		if (c == -1)
		{
			printf("完成！\r\n");
			break;
		}

		if ((pixelCounter > 3 && pixelCounter < 324) && (rowCounter > 5 && rowCounter < 246))
		{
			//if (pixelCounter == 323 && c == ',')
			//fputc('\n', p_saveFile);
			//else
			//fputc(c, p_saveFile);

			if (c == 'N' || c == 'a')
				pixelDep = 0;
			else if (c != ',' && c != '\n')
				pixelDep = pixelDep * 10 + c - '0';

			if (c == ',')
			{
				depth[rowCounter - 6][pixelCounter - 4] = pixelDep;
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
			if (rowCounter > 251)		//新帧
			{
				rowCounter = 0;
				frameCounter++;
				printf("成功切割%d\r\n", frameCounter);
				_itoa(frameCounter, saveFileName, 10);
				for (int i = 0; i < 20; i++)
				{
					if (saveFileName[i] != 0)
						continue;
					else
					{
						saveFileName[i] = '.';
						saveFileName[i + 1] = 'b';
						saveFileName[i + 2] = 'm';
						saveFileName[i + 3] = 'p';
						saveFileName[i + 4] = 0;
						break;
					}
				}
				Mat src_1(240, 320, CV_16UC1, Scalar(0));
				Mat src_2(240, 320, CV_8UC1, Scalar(0));
				for (int i = 0; i < 240; i++)
				{
					for (int j = 0; j < 320; j++)
					{
						src_1.at<ushort>(i, j) = depth[240 - i][j];
					}
				}
				for (int i = 0; i < 240; i++)
				{
					for (int j = 0; j < 320; j++)
					{
						src_2.at<uchar>(i, j) = 255 - (uchar)(src_1.at<ushort>(i, j) * 255 / 3000);

					}
				}
				imwrite(saveFileName, src_2);

				//waitKey(0);
				//fclose(p_saveFile);
				//p_saveFile = fopen(saveFileName, "w+");
			}
		}
	}
	//fclose(p_saveFile);
	fclose(p_openFile);


}