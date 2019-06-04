
#include "hik_camera_driver/hik_camera_driver.h"

// public functions:

hik_camera_driver::hik_camera_driver()
{
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
  timeout = 1.0 / pub_rate;
}

hik_camera_driver::~hik_camera_driver()
{
  do
  {
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
      ROS_INFO("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
      ROS_INFO("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }
  } while (0);

  if (handle != NULL)
  {
    MV_CC_DestroyHandle(handle);
    handle = NULL;
  }
  if (pData)
  {
    free(pData);
    pData = NULL;
  }
  if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = NULL;
  }
  if (pDataForSaveImage)
  {
    free(pDataForSaveImage);
    pDataForSaveImage = NULL;
  }
  ROS_INFO("exit\n");
}

bool hik_camera_driver::initHikSDK()
{
  do
  {
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
      ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
      break;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
      for (int i = 0; i < stDeviceList.nDeviceNum; i++)
      {
        ROS_INFO("[device %d]:\n", i);
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
          break;
        }
        PrintDeviceInfo(pDeviceInfo);
      }
    }
    else
    {
      ROS_ERROR("Find No Devices!\n");
      break;
    }

    // 让用户输入设备ID号,但是现在只用一个相机,所以使用默认的0
    // ROS_INFO("Please Intput camera index:");
    // scanf("%d", &nIndex);
    // if (nIndex >= stDeviceList.nDeviceNum)
    // {
    //   ROS_INFO("Intput error!\n");
    //   break;
    // }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
      ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
      break;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
      ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE
    // camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
      int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
      if (nPacketSize > 0)
      {
        nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
        if (nRet != MV_OK)
        {
          ROS_ERROR("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
        }
      }
      else
      {
        ROS_ERROR("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
      }
    }

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
      ROS_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }
    //设置相机报告为自动调整模式
    nRet = MV_CC_SetExposureAutoMode(handle, MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    if (MV_OK == nRet)
    {
      ROS_INFO("set ExposureMode OK!\n\n");
    }
    else
    {
      ROS_INFO("set ExposureMode failed! nRet [%x]\n\n", nRet);
    }
    // // 设置int型变量
    // // set IInteger variable
    // unsigned int nHeightValue = 2592;
    // // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
    // // Step (16) should be considered when setting width and height, that is the width and height should be a multiple
    // // of 16
    // nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);
    // if (MV_OK == nRet)
    // {
    //   printf("set height OK!\n\n");
    // }
    // else
    // {
    //   printf("set height failed! nRet [%x]\n\n", nRet);
    // }

  } while (0);

  if (nRet != MV_OK)
  {
    if (handle != NULL)
    {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
    return false;
  }

  ROS_INFO("Init HikSDK succeed\n");
  return true;
}

bool hik_camera_driver::startGrabbing()
{
  do
  {
    // ch:获取数据包大小 | en:Get payload size
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
      ROS_INFO("Get PayloadSize fail! nRet [0x%x]\n", nRet);
      break;
    }
    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
      ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

  } while (0);

  if (nRet != MV_OK)
  {
    if (handle != NULL)
    {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
    return false;
  }
  return true;
}

void hik_camera_driver::decodeCallback(const ros::TimerEvent& event)
{
  // 使用sdk获取一帧图像
  pData = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
  {
    return;
  }
  unsigned int nDataSize = stParam.nCurValue;

  nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, timeout * 1000);
  if (nRet == MV_OK)
  {
#ifndef NDEBUG
    ROS_INFO("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n", stImageInfo.nWidth, stImageInfo.nHeight,
             stImageInfo.nFrameNum);
#endif
    
    // 处理图像
    // image processing
    // nInput = 0;
    // ROS_INFO("input 0 to do nothing, 1 to convert RGB, 2 to save as BMP\n");
    // scanf("%d", &nInput);
    switch (nInput)
    {
      // 不做任何事，继续往下走
      // do nothing, and go on next
      case 0:
      {
        break;
      }
      // 转换图像为RGB格式，用户可根据自身需求转换其他格式
      // convert image format to RGB, user can convert to other format by their requirement
      case 1:
      {
        // size_t = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
        size_t buffsize = stImageInfo.nWidth * stImageInfo.nHeight * 3;
        pDataForBGR = (unsigned char*)malloc(buffsize);
        if (NULL == pDataForBGR)
        {
          break;
        }
        // 像素格式转换
        // convert pixel format
        // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
        // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
        // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
        // destination pixel format, output data buffer, provided output buffer size
        stConvertParam.nWidth = stImageInfo.nWidth;
        stConvertParam.nHeight = stImageInfo.nHeight;
        stConvertParam.pSrcData = pData;
        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = pDataForBGR;
        stConvertParam.nDstBufferSize = buffsize;
        nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
        if (MV_OK != nRet)
        {
          ROS_INFO("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
          break;
        }
#ifndef NDEBUG
        ROS_INFO("convert succeed! \n");
#endif
        picBGR = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info_mgr->getCameraInfo()));
        ci->header = camera_info_mgr->getCameraInfo().header;
        ci->header.stamp = ros::Time::now();

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(ci->header, "bgr8", picBGR).toImageMsg();

        // publish the image
        image_pub.publish(*msg, *ci);

        break;
      }
      case 2:
      {
        size_t buffsize = stImageInfo.nWidth * stImageInfo.nHeight * 3;
        pDataForSaveImage = (unsigned char*)malloc(buffsize);
        if (NULL == pDataForSaveImage)
        {
          break;
        }
        // 填充存图参数
        // fill in the parameters of save image
        // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
        // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
        // Top to bottom are：
        stSaveParam.enImageType = MV_Image_Bmp;
        stSaveParam.enPixelType = stImageInfo.enPixelType;
        stSaveParam.nBufferSize = buffsize;
        stSaveParam.nWidth = stImageInfo.nWidth;
        stSaveParam.nHeight = stImageInfo.nHeight;
        stSaveParam.pData = pData;
        stSaveParam.nDataLen = stImageInfo.nFrameLen;
        stSaveParam.pImageBuffer = pDataForSaveImage;
        stSaveParam.nJpgQuality = 80;

        nRet = MV_CC_SaveImageEx2(handle, &stSaveParam);
        if (MV_OK != nRet)
        {
          ROS_INFO("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
          break;
        }

        FILE* fp = fopen("image_.bmp", "wb");
        if (NULL == fp)
        {
          ROS_INFO("fopen failed\n");
          break;
        }
        fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
        fclose(fp);
#ifndef NDEBUG
        ROS_INFO("save image succeed!\n");
#endif
        break;
      }
      default:
        break;
    }
  }

  if (pData)
  {
    free(pData);
    pData = NULL;
  }
  if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = NULL;
  }
  if (pDataForSaveImage)
  {
    free(pDataForSaveImage);
    pDataForSaveImage = NULL;
  }
}
bool hik_camera_driver::stopGrabbing()
{
  // 停止取流
  // end grab image
  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet)
  {
    ROS_ERROR("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);

    if (handle != NULL)
    {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
    return false;
  }
  return true;
}
void hik_camera_driver::PressEnterToExit(void)
{
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  ROS_WARN("\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
}
// private functions:

bool hik_camera_driver::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    ROS_INFO("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
    ROS_INFO("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    ROS_INFO("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    ROS_INFO("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  }
  else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
  {
    ROS_INFO("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    ROS_INFO("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  }
  else
  {
    ROS_INFO("Not support.\n");
  }

  return true;
}
bool hik_camera_driver::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                                      sensor_msgs::SetCameraInfo::Response& rsp)
{
  ROS_INFO("New camera info received");
  sensor_msgs::CameraInfo& info = req.camera_info;
  info.header.frame_id = frame_id;

  if (info.width != image_width || info.height != image_height)
  {
    rsp.success = 0;
    char buf[100];
    sprintf(buf, "Camera_info resolution %ix%i does not match current video "
                 "setting, camera running at resolution %ix%i.",
            info.width, info.height, image_width, image_height);
    rsp.status_message = buf;
    ROS_ERROR("%s", rsp.status_message.c_str());
    return true;
  }
  if (camera_info_url.empty())
    camera_info_url = expandUserPath("~/.ros/camera_info/" + camera_name + ".yaml");

  if (!camera_calibration_parsers::writeCalibration(camera_info_url, camera_name, info))
  {
    rsp.status_message = "Error formatting camera_info for storage. filename = " + camera_info_url;
    rsp.success = 0;
  }
  else
  {
    rsp.success = 1;
    rsp.status_message = "Write camera_info to " + camera_info_url + " success.";
  }

  if (!rsp.success)
  {
    ROS_ERROR("[ %s ] %s", camera_name.c_str(), rsp.status_message.c_str());
  }
  else
  {
    ROS_INFO("[ %s ] %s", camera_name.c_str(), rsp.status_message.c_str());
  }
  return true;
}
void hik_camera_driver::initROSIO(ros::NodeHandle& priv_node)
{
  priv_node.param("camera_frame_id", frame_id, std::string("hik_camera"));
  priv_node.param("camera_name", camera_name, std::string("hik_camera"));
  priv_node.param("camera_info_url", camera_info_url, std::string(""));
  priv_node.param<int>("camera_pub_rate", pub_rate, 10);
  priv_node.param<int>("image_width", image_width, 2592);
  ROS_INFO("[%s] image width:  \t%d", camera_name.c_str(), image_width);
  priv_node.param<int>("image_height", image_height, 2048);
  ROS_INFO("[%s] image height: \t%d", camera_name.c_str(), image_height);

  camera_info_mgr.reset(new camera_info_manager::CameraInfoManager(priv_node, camera_name, camera_info_url));
  if (!camera_info_mgr->isCalibrated())
  {
    camera_info_mgr->setCameraName(camera_name);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = frame_id;
    camera_info.width = (unsigned int)(image_width);
    camera_info.height = (unsigned int)(image_height);
    camera_info_mgr->setCameraInfo(camera_info);
  }

  image_transport::ImageTransport it(priv_node);
  image_pub = it.advertiseCamera(camera_name, 1);
  timer = priv_node.createTimer(ros::Duration(timeout), &hik_camera_driver::decodeCallback, this);
  SetCameraInfoSrv =
      priv_node.advertiseService(camera_name + "/set_camera_info", &hik_camera_driver::setCameraInfo, this);
}

std::string hik_camera_driver::expandUserPath(std::string path)
{
  if (not path.empty() and path[0] == '~')
  {
    assert(path.size() == 1 or path[1] == '/');  // or other error handling
    char const* home = getenv("HOME");
    if (home or ((home = getenv("USERPROFILE"))))
    {
      path.replace(0, 1, home);
    }
    else
    {
      char const *hdrive = getenv("HOMEDRIVE"), *hpath = getenv("HOMEPATH");
      assert(hdrive);  // or other error handling
      assert(hpath);
      path.replace(0, 1, std::string(hdrive) + hpath);
    }
  }
  return path;
}