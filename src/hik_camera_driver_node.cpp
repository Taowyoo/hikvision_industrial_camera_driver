#include "hik_camera_driver/hik_camera_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hik_camera_driver_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);

  hik_camera_driver camera;
  camera.initROSIO(node_handle);
  camera.initHikSDK();
  camera.startGrabbing();
  spinner.start();
  camera.PressEnterToExit();
  return 0;
}