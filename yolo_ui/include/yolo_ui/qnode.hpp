/**
 * @file /include/yolo_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yolo_ui_QNODE_HPP_
#define yolo_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yolo_ui
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  cv::Mat* imgRaw = NULL;
  bool isRecv_img = false;

Q_SIGNALS:
  void rosShutdown();
  void recvImg();

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber img_sub;
  void img_Callback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace yolo_ui

#endif /* yolo_ui_QNODE_HPP_ */
