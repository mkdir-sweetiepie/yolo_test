#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

using namespace cv;
using namespace std;

namespace yolo_master
{
class YOLO_MASTER
{
public:
  YOLO_MASTER(int argc, char** argv);
  ~YOLO_MASTER();

  bool init();
  void run();
  void update();

  Mat* original;
  Mat clone_mat;
  Mat frame;
  Mat blob;

  bool isRecv;
  bool isOverlapping;

  cv::dnn::Net net;
  std::vector<std::string> class_names;

  void set_yolo();
  bool isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2);

  std::string cam1_topic_name;
  std::string cam2_topic_name;

private:
  int init_argc;
  char** init_argv;

  // <-> cam*_topic_name
  image_transport::Subscriber img_sub;
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img);

  // <-> yolo_ui
  image_transport::Publisher img_result;
};

}  // namespace yolo_master
