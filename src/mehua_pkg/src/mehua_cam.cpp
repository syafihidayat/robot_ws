#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/point.hpp>
#include <filesystem>
#include "mehua_pkg/utils.h"
#include "mehua_pkg/yolov8Predictor.h"

#define CLASS_FALSE 0
#define CLASS_TRUE 1

class DetectionPublish : public rclcpp::Node
{
public:
  DetectionPublish() : Node("object_detection")
  {
    float confThreshold = 0.25f;      //0.04f;
    float iouThreshold = 0.04f;
    float maskThreshold = 0.5f;
    bool isGPU = false;

    std::string modelPath = "/home/syafihidayat/Documents/robot_ws/src/mehua_pkg/models/fullmerah2.onnx";
    // std::string modelPath = "/home/syafihidayat/Documents/robot_ws/src/mehua_pkg/models/best2.onnx";
    std::string classNamesPath = "/home/syafihidayat/Documents/robot_ws/src/mehua_pkg/models/KFS.names";

    classNames = utils::loadNames(classNamesPath);
    if (classNames.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "FILE CLASS NAME KOSONG!");
      rclcpp::shutdown();
      return;
    }

    if (!std::filesystem::exists(modelPath))
    {
      RCLCPP_ERROR(this->get_logger(), "MOdel YOlo tidak di temukan di path %s", modelPath.c_str());
      rclcpp::shutdown();
      return;
    }

    try
    {
      predictor = std::make_unique<YOLOPredictor>(modelPath, isGPU, confThreshold, iouThreshold, maskThreshold);
      RCLCPP_INFO(this->get_logger(), "Model YOLO behasil di inisialisasikan");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Gagal inisialisasi model: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    assert(classNames.size() == static_cast<size_t>(predictor->classNums));
    // assert(classNames.size() == predictor.classNums);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    try
    {
      try
      {
        pipe.stop();
      }
      catch (...)
      {
      }

      pipe.start(cfg);
      RCLCPP_INFO(this->get_logger(), "Real sense camera berhasil di inisialisasi");
    }
    catch (const rs2::error &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Gaagal membuka Realsense: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    boundingBox_pub = this->create_publisher<geometry_msgs::msg::Point>("coordinate_boundingBox", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&DetectionPublish::detectionLoop, this));

    is_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Node berahsil diinisialisasikan");
  }

  bool isInitialized() const { return is_initialized; }

private:

  bool target_locked = false;
  float locked_cx = 0.0f;
  float locked_cy = 0.0f;
  int lost_count = 0;

  const float MAX_JUMP = 80.0f;
  const int LOST_THRESHOLD = 5;


  void detectionLoop()
  {

    if (!is_initialized)
      return;

    auto frames = pipe.wait_for_frames();
    auto color_frame = frames.get_color_frame();
    auto depth_frame = frames.get_depth_frame();

    if (!color_frame || !depth_frame)
    {
      RCLCPP_WARN(this->get_logger(), "Frame kosong dari realsense");
      return;
    }

    cv::Mat color_image(cv::Size(1280, 720), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat frame = color_image.clone();

    if (frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "frame kosong dari kamera");
      return;
    }

    std::vector<Yolov8Result> result = predictor->predict(frame);

    bool target_found = false;
    Yolov8Result best_real;
    float min_depth = 999.0f;
    float best_cx = 0, best_cy = 0;

    for (const auto &res : result)
    {
      if (res.classId != CLASS_TRUE)
        continue;

      float cx = res.box.x + res.box.width * 0.5f;
      float cy = res.box.y + res.box.height * 0.5f;

      float depth = depth_frame.get_distance((int)cx, (int)cy);

      if (depth < 0.1f || depth > 2.0f)
        continue;

      if (target_locked)
      {
        float dist = std::hypot(cx - locked_cx, cy - locked_cy);

        if (dist < MAX_JUMP)
        {
          best_real = res;
          best_cx = cx;
          best_cy = cy;
          min_depth = depth;
          target_found = true;
          break;
        }
      }

      else
      {
        if (depth < min_depth)
        {
          best_real = res;
          best_cx = cx;
          best_cy = cy;
          min_depth = depth;
          target_found = true;
        }
      }
    }

    utils::visualizeDetection(frame, result, classNames);

    geometry_msgs::msg::Point msg;

    if (!target_found)
    {

      lost_count++;
      if(lost_count > LOST_THRESHOLD)
      {
        target_locked = false;
      }

      msg.x = 0;
      msg.y = -1;
      msg.z = 0;
      boundingBox_pub->publish(msg);

      cv::putText(frame, "NO TRUE TARGET", cv::Point(450, 360), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
      cv::imshow("YOLO CAM", frame);
      cv::waitKey(1);
      return;
    }

    float img_cx = frame.cols / 2.0f;
    float error_x = best_cx - img_cx;

    lost_count = 0;
    target_locked = true;
    // locked_target = best;
    locked_cx = best_cx;
    locked_cy = best_cy;

    msg.x = error_x;
    msg.y = min_depth;
    msg.z = best_real.conf;
    boundingBox_pub->publish(msg);

    auto bbox = best_real.box;

    cv::rectangle(frame,
                  cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height),
                  cv::Scalar(0, 255, 0), 3);

    cv::circle(frame, cv::Point(best_cx, best_cy), 6, cv::Scalar(0, 255, 0), -1);

    cv::line(frame, cv::Point(frame.cols / 2, 0), cv::Point(frame.cols / 2, frame.rows),
             cv::Scalar(255, 0, 0), 1);

    std::string info = "ERR_X: " + std::to_string((int)error_x) + "  DEPTH: " + std::to_string(min_depth);

    cv::putText(frame, info, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
                0.8, cv::Scalar(0, 255, 255), 2);

    cv::imshow("YOLO CAM", frame);
    cv::waitKey(1);
  }

  std::unique_ptr<YOLOPredictor> predictor;
  std::vector<std::string> classNames;
  rs2::pipeline pipe;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr boundingBox_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_initialized;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<DetectionPublish>();
    RCLCPP_INFO(node->get_logger(), "Node nya berjalan");
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }

  rclcpp::shutdown();
  cv::destroyAllWindows();

  return 0;
}