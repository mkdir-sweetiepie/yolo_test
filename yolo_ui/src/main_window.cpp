/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/yolo_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yolo_ui
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(slotUpdateImg()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/
void MainWindow::slotUpdateImg()
{
  cv::Mat clone_mat = qnode.imgRaw->clone();                                    // 원본 이미지 복사
  cv::resize(clone_mat, clone_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);  // 이미지 크기 조정
  ui.label->setPixmap(QPixmap::fromImage(
      QImage((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888)));

  delete qnode.imgRaw;  // 동적 할당된 원본 이미지 메모리 해제
  qnode.imgRaw = NULL;
  qnode.isRecv_img = false;  // 이미지 수신 플래그 재설정
}
}  // namespace yolo_ui
