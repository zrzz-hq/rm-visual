/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** V4l2Capture.cpp
** 
** V4L2 wrapper 
**
** -------------------------------------------------------------------------*/


// libv4l2
#include <linux/videodev2.h>
#include <V4l2Capture.h>

// project
#include "logger.h"
#include "V4l2Capture.h"
#include "V4l2MmapDevice.h"
#include "V4l2ReadWriteDevice.h"
#include "opencv2/opencv.hpp"

#include <vector>

// -----------------------------------------
//    create video capture interface
// -----------------------------------------
V4l2Capture* V4l2Capture::create(const V4L2DeviceParameters & param, IoType iotype)
{
	V4l2Capture* videoCapture = NULL;
	V4l2Device* videoDevice = NULL; 
	int caps = V4L2_CAP_VIDEO_CAPTURE;
	switch (iotype)
	{
		case IOTYPE_MMAP: 
			videoDevice = new V4l2MmapDevice(param, V4L2_BUF_TYPE_VIDEO_CAPTURE); 
			caps |= V4L2_CAP_STREAMING;
		break;
		case IOTYPE_READWRITE:
			videoDevice = new V4l2ReadWriteDevice(param, V4L2_BUF_TYPE_VIDEO_CAPTURE); 
			caps |= V4L2_CAP_READWRITE;
		break;
	}
	
	if (videoDevice &&  !videoDevice->init(caps))
	{
		delete videoDevice;
		videoDevice=NULL; 
	}
	
	if (videoDevice)
	{
		videoCapture = new V4l2Capture(videoDevice);
	}	
	return videoCapture;
}

// -----------------------------------------
//    constructor
// -------InputArray----------------------------------
V4l2Capture::V4l2Capture(V4l2Device* device) : V4l2Access(device)
{
}

// -----------------------------------------
//    destructor
// -----------------------------------------
V4l2Capture::~V4l2Capture() 
{
}

// -----------------------------------------
//    check readability
// -----------------------------------------
int V4l2Capture::isReadable(timeval* tv)
{
	int fd = m_device->getFd();
	fd_set fdset;
	FD_ZERO(&fdset);	
	FD_SET(fd, &fdset);
    return select(fd+1, &fdset, NULL, NULL, tv);
}

const char *V4l2Capture::getBusInfo()
{
    return (const char *)this->m_device->getBusInfo();
}

// -----------------------------------------
//    read from V4l2Device
// -----------------------------------------
size_t V4l2Capture::read(char* buffer, size_t bufferSize)
{
    return m_device->readInternal(buffer, bufferSize);
}

int V4l2Capture::read(cv::Mat &readImage)
{
    //清除原图像的数据，避免错误
    if(!readImage.empty()){
        readImage.release();
    }
    char buffer[this->getBufferSize()];
    int rsize = this->read(buffer, sizeof(buffer));
    if (rsize == -1)
    {
        return -1;
    }
    else
    {
        /*
        2.6.1. Packed YUV formats
        2.6.2. V4L2_PIX_FMT_GREY (‘GREY’)
        2.6.3. V4L2_PIX_FMT_Y10 (‘Y10 ‘)
        2.6.4. V4L2_PIX_FMT_Y12 (‘Y12 ‘)
        2.6.5. V4L2_PIX_FMT_Y10BPACK (‘Y10B’)
        2.6.6. V4L2_PIX_FMT_Y16 (‘Y16 ‘)
        2.6.7. V4L2_PIX_FMT_Y16_BE (‘Y16 ‘ | (1 << 31))
        2.6.8. V4L2_PIX_FMT_Y8I (‘Y8I ‘)
        2.6.9. V4L2_PIX_FMT_Y12I (‘Y12I’)
        2.6.10. V4L2_PIX_FMT_UV8 (‘UV8’)
        2.6.11. V4L2_PIX_FMT_YUYV (‘YUYV’)
        2.6.12. V4L2_PIX_FMT_UYVY (‘UYVY’)
        2.6.13. V4L2_PIX_FMT_YVYU (‘YVYU’)
        2.6.14. V4L2_PIX_FMT_VYUY (‘VYUY’)
        2.6.15. V4L2_PIX_FMT_Y41P (‘Y41P’)
        2.6.16. V4L2_PIX_FMT_YVU420 (‘YV12’), V4L2_PIX_FMT_YUV420 (‘YU12’)
        2.6.17. V4L2_PIX_FMT_YUV420M (‘YM12’), V4L2_PIX_FMT_YVU420M (‘YM21’)
        2.6.18. V4L2_PIX_FMT_YUV422M (‘YM16’), V4L2_PIX_FMT_YVU422M (‘YM61’)
        2.6.19. V4L2_PIX_FMT_YUV444M (‘YM24’), V4L2_PIX_FMT_YVU444M (‘YM42’)
        2.6.20. V4L2_PIX_FMT_YVU410 (‘YVU9’), V4L2_PIX_FMT_YUV410 (‘YUV9’)
        2.6.21. V4L2_PIX_FMT_YUV422P (‘422P’)
        2.6.22. V4L2_PIX_FMT_YUV411P (‘411P’)
        2.6.23. V4L2_PIX_FMT_NV12 (‘NV12’), V4L2_PIX_FMT_NV21 (‘NV21’)
        2.6.24. V4L2_PIvectorX_FMT_NV12M (‘NM12’), V4L2_PIX_FMT_NV21M (‘NM21’), V4L2_PIX_FMT_NV12MT_16X16
        2.6.25. V4L2_PIX_FMT_NV12MT (‘TM12’)
        2.6.26. V4L2_PIX_FMT_NV16 (‘NV16’), V4L2_PIX_FMT_NV61 (‘NV61’)
        2.6.27. V4L2_PIX_FMT_NV16M (‘NM16’), V4L2_PIX_FMT_NV61M (‘NM61’)
        2.6.28. V4L2_PIX_FMT_NV24 (‘NV24’), V4L2_PIX_FMT_NV42 (‘NV42’)
        2.6.29. V4L2_PIX_FMT_M420 (‘M420’)

         * */
        if(m_device->getFormat() == V4L2_PIX_FMT_YUYV){
            cv::Mat v4l2Mat = cv::Mat( m_device->getHeight(),m_device->getWidth(), CV_8UC2, (void*)buffer);
            cv::cvtColor(v4l2Mat,readImage,cv::COLOR_YUV2BGRA_YUYV);
        }else if(m_device->getFormat() == V4L2_PIX_FMT_MJPEG){
            //cv::Mat v4l2Mat = cv::Mat( m_device->getHeight(),m_device->getWidth(), CV_8UC3, (void*)buffer);
            std::vector<unsigned char> buff(buffer,buffer+m_device->getHeight()*m_device->getWidth());
            readImage = cv::imdecode(buff, 1);
        }else  if(m_device->getFormat() == V4L2_PIX_FMT_H264){

        }else  if(m_device->getFormat() == V4L2_PIX_FMT_NV12){
            cv::Mat v4l2Mat = cv::Mat (m_device->getHeight()* 3 / 2, m_device->getWidth(), CV_8UC1, (void*)buffer);
            cv::cvtColor(v4l2Mat,readImage,cv::COLOR_YUV2BGR_NV12);
        }else if ((m_device->getFormat()  == V4L2_PIX_FMT_BGR24) || (m_device->getFormat() ==  V4L2_PIX_FMT_RGB24)) {
            readImage = cv::Mat (m_device->getHeight(), m_device->getWidth(), CV_8UC3, (void*)buffer);
        }else if ((m_device->getFormat()  == V4L2_PIX_FMT_YVU420) || (m_device->getFormat() ==  V4L2_PIX_FMT_YUV420)) {
            cv::Mat v4l2Mat = cv::Mat (m_device->getHeight()* 3 / 2, m_device->getWidth(), CV_8UC1, (void*)buffer);
            cv::cvtColor(v4l2Mat,readImage,cv::COLOR_YUV420p2BGR);
        }
        return 0;
    }

}

				
