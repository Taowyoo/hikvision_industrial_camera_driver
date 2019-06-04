#ifndef HIK_CAMERA_DRIVER_H_
#define HIK_CAMERA_DRIVER_H_
// #include <stdio.h>
// #include <string>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <opencv2/opencv.hpp>

#include "hikvision_SDK/MvCameraControl.h"

class hik_camera_driver
{
private:
  int nRet = MV_OK;
  void* handle = NULL;
  MV_CC_DEVICE_INFO_LIST stDeviceList;       // ch:设备信息列表 | en:Device Information List
  unsigned int nIndex = 0;                   // 设备ID号,现在只用一个相机,所以使用默认的0
  MVCC_INTVALUE stParam;                     // ch:数据包大小 | en:Get payload size
  MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };  // ch:输出帧的信息 | en:Output Frame Information
  MV_SAVE_IMAGE_PARAM_EX stSaveParam;        // ch:图片保存参数 | en:Save Image Parameters
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
  int pub_rate = 10;
  double timeout = 0.1;  // 发布间隔 单位:s
  // 图像数据指针
  unsigned char* pData = NULL;
  unsigned char* pDataForBGR = NULL;
  unsigned char* pDataForSaveImage = NULL;
  cv::Mat picBGR;
  // ros 相关
  image_transport::CameraPublisher image_pub;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr;
  ros::ServiceServer SetCameraInfoSrv;
  ros::Timer timer;
  // camera parameters
  int image_width;
  int image_height;
  std::string frame_id;
  std::string camera_name;
  std::string camera_info_url;
  void decodeCallback(const ros::TimerEvent& event);
  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
  std::string expandUserPath(std::string path);
  bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

public:
  int nInput = 1;  // input 0 to do nothing, 1 to convert to BGR and publish img, 2 to save as BMP
  hik_camera_driver();
  ~hik_camera_driver();
  bool initHikSDK();
  bool startGrabbing();
  void initROSIO(ros::NodeHandle& priv_node);
  bool stopGrabbing();
  void PressEnterToExit(void);
  // int opencvVideoCapture();
};

#endif