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

class DetectionPublish : public rclcpp::Node
{
public:
  DetectionPublish() : Node("object_detection")
  {
    float confThreshold = 0.04f;
    float iouThreshold = 0.04f;
    float maskThreshold = 0.5f;
    bool isGPU = false;

    std::string modelPath = "/home/syafihidayat/Documents/robot_ws/src/mehua_pkg/models/best2.onnx";
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
  void detectionLoop()
  {

    if (!is_initialized)
      return;

    try
    {
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

      std::map<int, int> classCounts;
      std::map<std::string, int> classNameCounts;
      int totalDtections = 0;

      for (const auto &res : result)
      {
        classCounts[res.classId]++;
        classNameCounts[classNames[res.classId]]++;
        totalDtections++;
      }

      std::stringstream detectionInfo;
      detectionInfo << "Total detections: " << totalDtections;
      RCLCPP_INFO(this->get_logger(), "%s", detectionInfo.str().c_str());

      int yOffset = 30;

      cv::rectangle(frame, cv::Point(10, 10), cv::Point(150, 40), cv::Scalar(0, 0, 0), -1);
      cv::putText(frame, detectionInfo.str(), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

      yOffset = 70;
      for (const auto &[className, count] : classNameCounts)
      {
        if (count > 0)
        {
          std::string classInfo = className + " : " + std::to_string(count);

          cv::rectangle(frame, cv::Point(10, yOffset - 20), cv::Point(200, yOffset + 5), cv::Scalar(0, 0, 0), -1);

          cv::putText(frame, classInfo, cv::Point(15, yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

          yOffset += 30;
        }
      }

      utils::visualizeDetection(frame, result, classNames);

      cv::imshow("YOLOv8 Webcam", frame);
      cv::waitKey(1);

      if (!result.empty())
      {
        auto bbox = result[0].box;
        geometry_msgs::msg::Point point;
        point.x = bbox.x;
        point.y = bbox.y;
        point.z = 0;
        boundingBox_pub->publish(point);
      }
    }

      catch(const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in detection loop: %s", e.what());
      }
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