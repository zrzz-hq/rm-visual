
#include <V4l2Device.h>
#include <V4l2Capture.h>
#include <iostream>
#include "logger.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ArmorDetector.h"
#include "AngleSolver.hpp"
#include "Serials/Serial.h"


volatile uint8_t _task;
#define CAMERA_NUMBER 8

int main()
{
    int verbose = 0;
    int stop = 0;
    const char *in_devname = "/dev/video0"; /* V4L2_PIX_FMT_YUYV V4L2_PIX_FMT_MJPEG*/
    /*
     *使用说明，我们读取UVC免驱的摄像头时，应该避免直接使用opencv的videocpature，
     * 因为简单的API使得我们并不知道我们到底获取的时摄像头的哪种图片格式。应该直接使用Qt v4l2 test benchmark软件去获取我们真正需要的
     * 图像帧格式。
     * V4L2_PIX_FMT_MJPEG （MJPEG）
     */
    V4L2DeviceParameters param(in_devname, V4L2_PIX_FMT_MJPEG, 800, 600, 30, 0, verbose);
    V4l2Capture *videoCapture = V4l2Capture::create(param, V4l2Access::IOTYPE_MMAP);
    
    rm::AngleSolverParam angleParam;
    angleParam.readFile(CAMERA_NUMBER);//choose camera
    rm::AngleSolver anglesolver;
    
    anglesolver.init(angleParam);
    anglesolver.setResolution(cv::Size(800,600));
    
    rm::ArmorParam armorParam;
    rm::ArmorDetector armorDetector;
    armorDetector.init(armorParam);
    armorDetector.setEnemyColor(rm::BLUE);

    rm::Serial serial;
    serial.openPort();
    serial.setDebug(true);
    int self_color;
    while(serial.setup(self_color) != rm::Serial::OJBK)
    {
       sleep(1);
    }
    int enemy_color;
    self_color= rm::BLUE ? enemy_color=rm::RED : enemy_color=rm::BLUE;
    armorDetector.setEnemyColor(enemy_color);
    std::cout<<"I am"<<(self_color == rm::BLUE ? "blue" : "red")<<"."<<std::endl;
    
    
    
    if (videoCapture == NULL)
    {
        LOG(WARN) << "Cannot create V4L2 capture interface for device:"
                  << "/dev/video0";
        return -1;
    }
    timeval tv;
    LOG(NOTICE) << "USB bus:" << videoCapture->getBusInfo();
    LOG(NOTICE) << "Start Uncompressing " << in_devname;

    while (!stop)
    {
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        int ret = videoCapture->isReadable(&tv);
        if (ret == 1)
        {
            cv::Mat v4l2Mat;
            ret = videoCapture->read(v4l2Mat);

            if (ret != 0)
            {
                LOG(NOTICE) << "stop ";
                stop = 1;
            }
            else
            {
                armorDetector.loadImg(v4l2Mat);
                int armorFlag = armorDetector.detect();
                
                if(armorFlag == rm::ArmorDetector::ARMOR_LOCAL || armorFlag == rm::ArmorDetector::ARMOR_GLOBAL)
                {
                    std::vector<cv::Point2f> armorVertex = armorDetector.getArmorVertex();
                    int armorType = armorDetector.getArmorType();
                    anglesolver.setTarget(armorVertex, armorType);
                    int angleFlag = anglesolver.solve();
                    if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
                    {
                        cv::Vec2f targetAngle = anglesolver.getAngle();
                        std::cout<<"X err"<<targetAngle[0]<<std::endl;
                        std::cout<<"Y err"<<targetAngle[1]<<std::endl;
                        rm::ControlData controldata;
                        controldata.pitch_dev = targetAngle[1];
                        controldata.yaw_dev = targetAngle[0];
                        if(serial.tryControl(controldata,std::chrono::milliseconds(3)) == rm::Serial::OJBK)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            std::cout<<"success to sent"<<std::endl;
                        }
                        else
                        {
                            std::cout<<"not sent"<<std::endl;
                        }
                    }

                    for (int i=0;i<armorVertex.size();i++)
                        cv::circle(v4l2Mat,cv::Point(armorVertex[i].x,armorVertex[i].y),10,CV_RGB(255,0,0),2);
                    //cv::drawContours(v4l2Mat,armorVertex,-1,cv::Scalar(255,0,0));

                   

                }
                cv::imshow("test", v4l2Mat);
                cv::waitKey(10);
            }
        }
        else if (ret == -1) // 返回错误
        {
            LOG(NOTICE) << "stop " << strerror(errno);
            stop = 1;
        }
        else if (ret == 0) // 返回超时
        {
            LOG(NOTICE) << "timeout" << strerror(errno);
            stop = 1;
        }
    }
    delete videoCapture;
}
//     V4L2DeviceParameters mparam(in_devname, V4L2_PIX_FMT_YUYV , 1920, 1080, 5, 0,verbose);
//     videoCapture = V4l2Capture::create(mparam, V4l2Access::IOTYPE_MMAP);
//     while (!stop)
//     {
//         tv.tv_sec=1;
//         tv.tv_usec=0;
//         int ret = videoCapture->isReadable(&tv);
//         if (ret == 1)
//         {
//             cv::Mat v4l2Mat;
//             ret =  videoCapture->read(v4l2Mat);

//             if (ret != 0)
//             {
//                 LOG(NOTICE) << "stop " ;
//                 stop=1;
//             }
//             else
//             {
//                cv::imshow("test",v4l2Mat);
//                if(cv::waitKey(10))
//                     break;
//             }
//         }
//         else if (ret == -1) //返回错误
//         {
//             LOG(NOTICE) << "stop " << strerror(errno);
//             stop=1;
//         }else if( ret == 0) // 返回超时
//         {
//             LOG(NOTICE) << "timeout" << strerror(errno);
//         }
//     }
// }
