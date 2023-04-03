/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** V4l2Device.h
** 
** V4L2 wrapper 
**
** -------------------------------------------------------------------------*/


#ifndef V4L2_DEVICE
#define V4L2_DEVICE

#include <string>
#include <list>
#include <linux/videodev2.h>
#include <fcntl.h>

#ifndef V4L2_PIX_FMT_VP8
#define V4L2_PIX_FMT_VP8  v4l2_fourcc('V', 'P', '8', '0')
#endif
#ifndef V4L2_PIX_FMT_VP9
#define V4L2_PIX_FMT_VP9  v4l2_fourcc('V', 'P', '9', '0')
#endif
#ifndef V4L2_PIX_FMT_HEVC
#define V4L2_PIX_FMT_HEVC  v4l2_fourcc('H', 'E', 'V', 'C')
#endif

// ---------------------------------
// V4L2 Device parameters
// ---------------------------------
/*
 * YUV 格式对照表，根据Qt v4l2 test benchmark软件(debain类系统 apt-get install qv4l2 安装)的capture image format一栏进行对应
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
2.6.24. V4L2_PIX_FMT_NV12M (‘NM12’), V4L2_PIX_FMT_NV21M (‘NM21’), V4L2_PIX_FMT_NV12MT_16X16
2.6.25. V4L2_PIX_FMT_NV12MT (‘TM12’)
2.6.26. V4L2_PIX_FMT_NV16 (‘NV16’), V4L2_PIX_FMT_NV61 (‘NV61’)
2.6.27. V4L2_PIX_FMT_NV16M (‘NM16’), V4L2_PIX_FMT_NV61M (‘NM61’)
2.6.28. V4L2_PIX_FMT_NV24 (‘NV24’), V4L2_PIX_FMT_NV42 (‘NV42’)
2.6.29. V4L2_PIX_FMT_M420 (‘M420’)
*/
struct V4L2DeviceParameters 
{   /**
     * @brief V4L2DeviceParameters
     * @param devname 设备名，如输入"/dev/video0"
     * @param formatList 多个图像帧格式
     * @param width 设置的输出图像宽，并不一定生效。
     * @param height 设置的输出图像高，并不一定生效。
     * @param fps 设置的输出图像帧率，并不一定生效。
     * @param input_index 单设备多输入的情况下，指定输入index的参数，如果单设备单输入，直接传入参数0
     * @param verbose
     * @param openFlags
     */
    V4L2DeviceParameters(const char* devname, const std::list<unsigned int> & formatList, unsigned int width, unsigned int height, int fps,unsigned int input_index = 0, int verbose = 0, int openFlags = O_RDWR | O_NONBLOCK) :
        m_devName(devname), m_inputIndex(input_index), m_formatList(formatList), m_width(width), m_height(height), m_fps(fps), m_verbose(verbose), m_openFlags(openFlags) {}
    /**
     * @brief V4L2DeviceParameters
     * @param devname
     * @param format 图像帧格式
     * @param width 设置的输出图像宽，并不一定生效。
     * @param height 设置的输出图像高，并不一定生效。
     * @param fps 设置的输出图像帧率，并不一定生效。
     * @param input_index 单设备多输入的情况下，指定输入index的参数，如果单设备单输入，直接传入参数0
     * @param verbose
     * @param openFlags
     */
    V4L2DeviceParameters(const char* devname, unsigned int format, unsigned int width, unsigned int height, int fps,unsigned int input_index = 0, int verbose = 0, int openFlags = O_RDWR | O_NONBLOCK) :
        m_devName(devname), m_inputIndex(input_index), m_width(width), m_height(height), m_fps(fps), m_verbose(verbose), m_openFlags(openFlags) {
			if (format) {
				m_formatList.push_back(format);
			}
	}
		
	std::string m_devName;
    unsigned int m_inputIndex;
	std::list<unsigned int> m_formatList;
	unsigned int m_width;
	unsigned int m_height;
	int m_fps;			
	int m_verbose;
	int m_openFlags;
};

// ---------------------------------
// V4L2 Device
// ---------------------------------
class V4l2Device
{		
	friend class V4l2Capture;
	friend class V4l2Output;
	
	protected:	
		void close();	
	
        int initdevice(const char *dev_name , unsigned int mandatoryCapabilities );
		int checkCapabilities(int fd, unsigned int mandatoryCapabilities);
		int configureFormat(int fd);
		int configureFormat(int fd, unsigned int format, unsigned int width, unsigned int height);
		int configureParam(int fd);

        virtual bool init(unsigned int mandatoryCapabilities);
		virtual size_t writeInternal(char*, size_t) { return -1; }
		virtual bool startPartialWrite(void)        { return false; }
		virtual size_t writePartialInternal(char*, size_t) { return -1; }
		virtual bool endPartialWrite(void)          { return false; }
		virtual size_t readInternal(char*, size_t)  { return -1; }
	
	public:
		V4l2Device(const V4L2DeviceParameters&  params, v4l2_buf_type deviceType);		
		virtual ~V4l2Device();
	
		virtual bool isReady() { return (m_fd != -1); }
		virtual bool start()   { return true; }
		virtual bool stop()    { return true; }
	
		unsigned int getBufferSize() { return m_bufferSize; }
		unsigned int getFormat()     { return m_format;     }
		unsigned int getWidth()      { return m_width;      }
		unsigned int getHeight()     { return m_height;     }
        unsigned char *getBusInfo() { return  bus_info;    }
		int getFd()         { return m_fd;         }
		void queryFormat();	

	protected:
		V4L2DeviceParameters m_params;
		int m_fd;
		v4l2_buf_type m_deviceType;	
	
		unsigned int m_bufferSize;
		unsigned int m_format;
		unsigned int m_width;
		unsigned int m_height;	

		struct v4l2_buffer m_partialWriteBuf;
		bool m_partialWriteInProgress;
        unsigned char bus_info[32];
};


#endif
