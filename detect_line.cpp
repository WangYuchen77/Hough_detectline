#include <librealsense2/rs.hpp>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <unistd.h>

#include "camera.h"

using namespace cv;
using namespace std;

class LineFinder{
private:
	// 原图
    cv::Mat img;
	//向量中包含检测到的直线的端点
    std::vector<cv::Vec4i> lines;
	//累加器的分辨率
    double deltaRho;
    double deltaTheta;
	//直线被接受时所需的最小投票数
    int minVote;
	//直线的最小长度
    double minLength;
	//在直线上两个点的最大缺口
    double maxGap;
public:
	//默认累加器的分辨率为单个像素及1°
    LineFinder():deltaRho(1),deltaTheta(CV_PI/180),minVote(10),minLength(0.),maxGap(0.){}
	//设置累加器的分辨率
    void setAccResolution(double dRho , double dTheta){
        deltaRho = dRho;
        deltaTheta = dTheta;
    }
	//设置最小投票数
    void setminVote(int minv){
        minVote = minv;
    }
	//设置缺口及最小长度
    void setLineLengthAndGap(double length, double gap){
        minLength = length;
        maxGap = gap;
    }
	//使用概率霍夫变换
    std::vector<cv::Vec4i>findLines(cv::Mat& binary){
        lines.clear();
        cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
        return lines;
    }
	//检测到的直线
    void drawDetectedLines(cv::Mat &image, cv::Scalar color = cv::Scalar(0,0,255)){
        std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
        while(it2!= lines.end()){
            cv::Point pt1((*it2)[0], (*it2)[1]);
            cv::Point pt2((*it2)[2], (*it2)[3]);
            cv::line(image, pt1, pt2, color);
            ++it2;
        }
    }
};


int main(int argc, char** argv){
    
    int matrix_size = 2000;
    int lineLength = 15;
    int pointGap = 10;
    int minVote = 10;

	//加载图像
    cv::Mat image;
    image = imread("../3.png");
    cv::namedWindow("origin");
    cv::resize(image, image, cv::Size(640, 480));
    cv::imshow("origin",image);
	//对图像进行canny边缘检测，形成二值化的图像矩阵
    cv::Mat binary;
    cv::Canny(image, binary, 125, 350);
    
	//找直线
    LineFinder finder;
    finder.setLineLengthAndGap(lineLength,pointGap);
    finder.setminVote(minVote);
    std::vector<cv::Vec4i>lines = finder.findLines(binary);

    cout<<"find lines: "<<lines.size()<<endl;
    for (int i = 0 ; i < lines.size(); i++){
	// 用圆形画出每条线的端点
	// 参数: 画圆(图像，圆形，半径，颜色(B,G，R), 正数：粗细/负数：填充，线条类型)
        cv::circle(image,Point2f(lines[i][0],lines[i][1]),2,Scalar(0,255,0),1,8);
        cv::circle(image,Point2f(lines[i][2],lines[i][3]),2,Scalar(255,255,0),1,8);
    }

    
    cv::namedWindow("binary");
    cv::imshow("binary",binary);
    
    finder.drawDetectedLines(image);
    cv::namedWindow("hough");
    cv::imshow("hough",image);
    cvWaitKey(0);
    return 0;
}

