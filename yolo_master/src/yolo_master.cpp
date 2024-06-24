#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <ros/package.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "../include/yolo_master/yolo_master.hpp"

constexpr float CONFIDENCE_THRESHOLD = 0.48;  // 확률 경계값
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 1;
const cv::Scalar colors[] = { { 0, 255, 255 }, { 255, 255, 0 }, { 0, 255, 0 }, { 255, 0, 0 } };
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

const int cam_x = 640;
const int cam_y = 480;
const float cam_center_x = 320.0;
const float cam_center_y = 240.0;

int main(int argc, char** argv)
{
  yolo_master::YOLO_MASTER start(argc, argv);
  start.run();
}

namespace yolo_master
{
using namespace cv;
using namespace std;
using namespace ros;

YOLO_MASTER::YOLO_MASTER(int argc, char** argv) : init_argc(argc), init_argv(argv), isRecv(false)
{
  std::string packagePath = ros::package::getPath("yolo_master");  // ROS 패키지에서 경로 가져오기
  std::cout << packagePath << std::endl;                           // 패키지 경로 출력
  std::string dir = packagePath + "/yolo/";                        // YOLO 디렉터리 경로 설정
  {
    std::ifstream class_file(dir + "classes.txt");  // 클래스 파일 열기
    if (!class_file)
    {
      std::cerr << "failed to open .txt\n";
    }

    std::string line;  // 클래스 이름을 읽어와 벡터에 추가
    while (std::getline(class_file, line))
      class_names.push_back(line);
  }

  std::string modelConfiguration = dir + "yolov7_tiny_cup.cfg";     // 모델 구성 파일 경로 설정
  std::string modelWeights = dir + "yolov7_tiny_cup_4000.weights";  // 모델 가중치 파일 경로 설정

  net = cv::dnn::readNetFromDarknet(modelConfiguration,
                                    modelWeights);  // Darknet 형식의 모델 파일로부터 네트워크 구성 및 가중치 로드
  // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); // CUDA 백엔드 및 타겟 설정 (주석 처리됨)
  // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);  // OpenCV 백엔드와 CPU 타겟으로 설정
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  init();
}

YOLO_MASTER::~YOLO_MASTER()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

bool YOLO_MASTER::init()
{
  ros::init(init_argc, init_argv, "yolo_master");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  n.param<std::string>("cam1_topic", cam1_topic_name, "/camera/color/image_raw");
  n.param<std::string>("cam2_topic", cam2_topic_name, "/usb_cam/image_raw");
  ROS_INFO("Starting Rescue Vision With Camera : %s", cam1_topic_name.c_str());

  // Add your ros communications here.

  // <-> cam*_topic_name
  image_transport::ImageTransport img(n);
  img_sub = img.subscribe(cam1_topic_name, 100, &YOLO_MASTER::imageCallBack, this);  /// camera/color/image_raw

  // <-> vision24_ui
  img_result = img.advertise("/ui_image", 100);

  return true;
}

void YOLO_MASTER::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (isRecv == true)
    {
      update();
      img_result.publish(
          cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
    }
  }
}

// <-> cam*_topic_name
void YOLO_MASTER::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img)
{
  if (!isRecv)
  {
    original = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
    if (original != NULL)
    {
      isRecv = true;
    }
  }
}

void YOLO_MASTER::update()
{
  clone_mat = original->clone();
  cv::resize(clone_mat, clone_mat, cv::Size(cam_x, cam_y), 0, 0, cv::INTER_CUBIC);
  set_yolo();
  delete original;
  isRecv = false;
}

void YOLO_MASTER::set_yolo()
{
  auto output_names = net.getUnconnectedOutLayersNames();  // 출력 레이어 이름 가져오기
  frame = clone_mat.clone();                               // 프레임 복제
  std::vector<cv::Mat> detections;                         // 감지 결과를 담을 벡터

  auto total_start = std::chrono::steady_clock::now();  // 전체 시작 시간 측정
  cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(416, 416), cv::Scalar(), true, false,
                         CV_32F);  // 이미지를 blob으로 변환
  net.setInput(blob);              // 네트워크 입력 설정

  auto dnn_start = std::chrono::steady_clock::now();  // 딥러닝 전파 시작 시간 측정
  net.forward(detections, output_names);              // 딥러닝 전파 실행
  auto dnn_end = std::chrono::steady_clock::now();    // 딥러닝 전파 종료 시간 측정

  // 감지된 객체의 박스와 점수를 저장할 벡터들
  std::vector<int> indices[NUM_CLASSES];
  std::vector<cv::Rect> boxes[NUM_CLASSES];
  std::vector<float> scores[NUM_CLASSES];

  // 각 detection에 대해 반복
  for (auto& output : detections)
  {
    const auto num_boxes = output.rows;
    for (int i = 0; i < num_boxes; i++)
    {
      auto x = output.at<float>(i, 0) * frame.cols;                 // 중심 x 좌표 계산
      auto y = output.at<float>(i, 1) * frame.rows;                 // 중심 y 좌표 계산
      auto width = output.at<float>(i, 2) * frame.cols;             // 너비 계산
      auto height = output.at<float>(i, 3) * frame.rows;            // 높이 계산
      cv::Rect rect(x - width / 2, y - height / 2, width, height);  // 박스 생성

      // 클래스에 대한 반복
      for (int c = 0; c < NUM_CLASSES; c++)
      {
        auto confidence = *output.ptr<float>(i, 5 + c);  // 확률 가져오기
        if (confidence >= CONFIDENCE_THRESHOLD)          // 신뢰도 임계값 이상인 경우
        {
          boxes[c].push_back(rect);         // 박스 추가
          scores[c].push_back(confidence);  // 점수 추가
        }
      }
    }
  }

  // 각 클래스에 대해 비최대 억제 수행
  for (int c = 0; c < NUM_CLASSES; c++)
    cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);

  // 비최대 억제 후의 각 박스에 대해 처리
  for (int c = 0; c < NUM_CLASSES; c++)
  {
    for (size_t i = 0; i < indices[c].size(); ++i)
    {
      const auto color = colors[c % NUM_COLORS];  // 클래스별 색상 선택
      auto idx = indices[c][i];
      const auto& rect = boxes[c][idx];  // 현재 박스

      // 같은 클래스의 중복되는 박스 확인
      isOverlapping = false;
      if (indices[c].size() != 0)
      {
        for (size_t j = 0; j < indices[c].size(); ++j)
        {
          if (j != i)
          {
            auto idx2 = indices[c][j];
            const auto& rect2 = boxes[c][idx2];
            if (isRectOverlapping(rect, rect2))
            {
              // 중복되는 박스가 있으면 크기를 비교
              if (rect2.area() < rect.area())
              {
                // 다른 박스가 더 작으면 중복 표시 후 종료
                isOverlapping = true;
                break;
              }
              else
              {
                // 현재 박스가 더 작으면 건너뜀
                continue;
              }
            }
          }
        }
      }

      // 작은 박스와 중복되지 않는 경우에만 박스 그리기
      if (!isOverlapping)
      {
        cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color,
                      3);  //박스그리기

        // 라벨과 점수 출력
        std::ostringstream label_ss;
        label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
        auto label = label_ss.str();

        int baseline;
        auto label_bg_sz =
            cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);  // 라벨 크기 계산
        cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10),
                      cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);  // 라벨 배경
        cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                    cv::Scalar(0, 0, 0));  // 라벨 텍스트
      }
    }

    auto total_end = std::chrono::steady_clock::now();  //전체 종료 시간 측정

    // 추론 및 전체 FPS 계산
    float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
    float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    std::ostringstream stats_ss;
    stats_ss << std::fixed << std::setprecision(2);
    stats_ss << "Inference FPS: " << inference_fps << ", Total FPS: " << total_fps;
    auto stats = stats_ss.str();

    int baseline;
    auto stats_bg_sz =
        cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);  // 통계 크기 계산
    // cv::rectangle(frame, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0),
    //               cv::FILLED);  // 통계 배경
    // cv::putText(frame, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
    //             cv::Scalar(255, 255, 255));  // 통계 텍스트 출력
  }
}

bool YOLO_MASTER::isRectOverlapping(const cv::Rect& rect1, const cv::Rect& rect2)
{
  int x1 = std::max(rect1.x, rect2.x);
  int y1 = std::max(rect1.y, rect2.y);
  int x2 = std::min(rect1.x + rect1.width, rect2.x + rect2.width);
  int y2 = std::min(rect1.y + rect1.height, rect2.y + rect2.height);

  if (x1 < x2 && y1 < y2)
    return true;
  else
    return false;
}

}  // namespace yolo_master
