// blending.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <cv.h>
#include <highgui.h>

using namespace cv;

/// 全局变量的声明与初始化
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

/// 声明存储图像的变量
Mat src1;
Mat src2;
Mat dst;

/**
* @function on_trackbar
* @定义响应滑动条的回调函数
*/
void on_trackbar(int, void*)
{
	alpha = (double)alpha_slider / alpha_slider_max;
	beta = (1.0 - alpha);

	addWeighted(src1, alpha, src2, beta, 0.0, dst);

	imshow("Linear Blend", dst);
}

int main(int argc, char** argv)
{
	/// 加载图像 (两图像的大小与类型要相同)
	src1 = imread("../../images/LinuxLogo.jpg");
	src2 = imread("../../images/WindowsLogo.jpg");

	if (!src1.data) { printf("Error loading src1 \n"); return -1; }
	if (!src2.data) { printf("Error loading src2 \n"); return -1; }

	/// 初始化为零
	alpha_slider = 0;

	/// 创建窗体
	namedWindow("Linear Blend", 1);

	/// 在创建的窗体中创建一个滑动条控件
	char TrackbarName[50];
	sprintf(TrackbarName, "Alpha x %d", alpha_slider_max);

	createTrackbar(TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar);

	/// 结果在回调函数中显示
	on_trackbar(alpha_slider, 0);

	/// 按任意键退出
	waitKey(0);
	return 0;
}