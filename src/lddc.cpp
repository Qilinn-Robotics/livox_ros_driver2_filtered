//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <limits>
#include <math.h>
#include <stdint.h>
#include <sstream>
#include <stdexcept>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

namespace livox_ros {

namespace {

std::string Trim(const std::string& text) {
  const auto start = text.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(start, end - start + 1);
}

std::vector<double> SplitDoubles(const std::string& text) {
  std::vector<double> values;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = Trim(token);
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

Lddc::BoxFilter ParseBoxFilter(const std::string& value) {
  const auto separator = value.find(':');
  std::string name = "box";
  std::string payload = value;
  if (separator != std::string::npos) {
    name = Trim(value.substr(0, separator));
    payload = value.substr(separator + 1);
  }
  if (name.empty()) {
    name = "box";
  }

  const auto parts = SplitDoubles(payload);
  Lddc::BoxFilter box;
  box.name = name;
  if (parts.size() == 6) {
    box.center.x = parts[0];
    box.center.y = parts[1];
    box.center.z = parts[2];
    box.rpy.x = 0.0;
    box.rpy.y = 0.0;
    box.rpy.z = 0.0;
    box.size.x = parts[3];
    box.size.y = parts[4];
    box.size.z = parts[5];
  } else if (parts.size() == 9) {
    box.center.x = parts[0];
    box.center.y = parts[1];
    box.center.z = parts[2];
    box.rpy.x = parts[3];
    box.rpy.y = parts[4];
    box.rpy.z = parts[5];
    box.size.x = parts[6];
    box.size.y = parts[7];
    box.size.z = parts[8];
  } else {
    throw std::runtime_error(
        "self_filter_box_filters entries must have 6 or 9 values");
  }

  if (box.size.x <= 0.0 || box.size.y <= 0.0 || box.size.z <= 0.0) {
    throw std::runtime_error("self-filter box '" + box.name + "' size must be positive");
  }
  return box;
}

Lddc::Mat3 RotationMatrixFromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Lddc::Mat3 m;
  m.v[0][0] = cy * cp;
  m.v[0][1] = cy * sp * sr - sy * cr;
  m.v[0][2] = cy * sp * cr + sy * sr;
  m.v[1][0] = sy * cp;
  m.v[1][1] = sy * sp * sr + cy * cr;
  m.v[1][2] = sy * sp * cr - cy * sr;
  m.v[2][0] = -sp;
  m.v[2][1] = cp * sr;
  m.v[2][2] = cp * cr;
  return m;
}

Lddc::Vec3 TransformPoint(const Lddc::Vec3& point, const Lddc::Mat4& transform) {
  Lddc::Vec3 out;
  out.x = transform.v[0][0] * point.x + transform.v[0][1] * point.y +
      transform.v[0][2] * point.z + transform.v[0][3];
  out.y = transform.v[1][0] * point.x + transform.v[1][1] * point.y +
      transform.v[1][2] * point.z + transform.v[1][3];
  out.z = transform.v[2][0] * point.x + transform.v[2][1] * point.y +
      transform.v[2][2] * point.z + transform.v[2][3];
  return out;
}

bool PointInBox(const Lddc::Vec3& point, const Lddc::BoxFilter& box, double padding) {
  const auto rotation = RotationMatrixFromRpy(box.rpy.x, box.rpy.y, box.rpy.z);
  const double dx = point.x - box.center.x;
  const double dy = point.y - box.center.y;
  const double dz = point.z - box.center.z;

  const double lx = dx * rotation.v[0][0] + dy * rotation.v[1][0] + dz * rotation.v[2][0];
  const double ly = dx * rotation.v[0][1] + dy * rotation.v[1][1] + dz * rotation.v[2][1];
  const double lz = dx * rotation.v[0][2] + dy * rotation.v[1][2] + dz * rotation.v[2][2];

  return std::abs(lx) <= 0.5 * box.size.x + padding &&
         std::abs(ly) <= 0.5 * box.size.y + padding &&
         std::abs(lz) <= 0.5 * box.size.z + padding;
}

#ifdef BUILDING_ROS2
uint64_t StampToNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<uint64_t>(stamp.nanosec);
}
#endif

}  // namespace

/** Lidar Data Distribute Control--------------------------------------------*/
#ifdef BUILDING_ROS1
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      enable_lidar_bag_(lidar_bag),
      enable_imu_bag_(imu_bag) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  cur_node_ = nullptr;
  bag_ = nullptr;
}
#elif defined BUILDING_ROS2
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
#if 0
  bag_ = nullptr;
#endif
}
#endif

Lddc::~Lddc() {
#ifdef BUILDING_ROS1
  if (global_pub_) {
    delete global_pub_;
  }

  if (global_imu_pub_) {
    delete global_imu_pub_;
  }
#endif

  PrepareExit();

#ifdef BUILDING_ROS1
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i];
    }
  }
#endif
  std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

void Lddc::SetRosNode(livox_ros::DriverNode *node) {
  cur_node_ = node;
  ConfigureSelfFilterTransform();
  LoadSelfFilterConfigFromRosParams();
}

void Lddc::ConfigureSelfFilterTransform() {
  const auto rotation = RotationMatrixFromRpy(
      self_filter_lidar_to_base_roll_, self_filter_lidar_to_base_pitch_,
      self_filter_lidar_to_base_yaw_);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      self_filter_lidar_to_base_.v[r][c] = 0.0;
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      self_filter_lidar_to_base_.v[r][c] = rotation.v[r][c];
    }
  }
  self_filter_lidar_to_base_.v[0][3] = self_filter_lidar_to_base_x_;
  self_filter_lidar_to_base_.v[1][3] = self_filter_lidar_to_base_y_;
  self_filter_lidar_to_base_.v[2][3] = self_filter_lidar_to_base_z_;
  self_filter_lidar_to_base_.v[3][3] = 1.0;
}

void Lddc::LoadSelfFilterConfigFromRosParams() {
#ifdef BUILDING_ROS2
  if (!cur_node_) {
    return;
  }
  auto& node = cur_node_->GetNode();

  auto declare_bool = [&node](const std::string& name, bool default_value) {
    if (!node.has_parameter(name)) {
      node.declare_parameter<bool>(name, default_value);
    }
    bool value = default_value;
    node.get_parameter(name, value);
    return value;
  };
  auto declare_double = [&node](const std::string& name, double default_value) {
    if (!node.has_parameter(name)) {
      node.declare_parameter<double>(name, default_value);
    }
    double value = default_value;
    node.get_parameter(name, value);
    return value;
  };
  auto declare_int = [&node](const std::string& name, int default_value) {
    if (!node.has_parameter(name)) {
      node.declare_parameter<int>(name, default_value);
    }
    int value = default_value;
    node.get_parameter(name, value);
    return value;
  };
  auto declare_string_vector =
      [&node](const std::string& name, const std::vector<std::string>& default_value) {
    if (!node.has_parameter(name)) {
      node.declare_parameter<std::vector<std::string>>(name, default_value);
    }
    std::vector<std::string> value = default_value;
    node.get_parameter(name, value);
    return value;
  };

  self_filter_enabled_ = declare_bool("self_filter_enabled", false);
  self_filter_box_padding_ =
      std::max(0.0, declare_double("self_filter_box_padding", self_filter_box_padding_));
  self_filter_lidar_to_base_x_ =
      declare_double("self_filter_lidar_to_base_x", self_filter_lidar_to_base_x_);
  self_filter_lidar_to_base_y_ =
      declare_double("self_filter_lidar_to_base_y", self_filter_lidar_to_base_y_);
  self_filter_lidar_to_base_z_ =
      declare_double("self_filter_lidar_to_base_z", self_filter_lidar_to_base_z_);
  self_filter_lidar_to_base_roll_ =
      declare_double("self_filter_lidar_to_base_roll", self_filter_lidar_to_base_roll_);
  self_filter_lidar_to_base_pitch_ =
      declare_double("self_filter_lidar_to_base_pitch", self_filter_lidar_to_base_pitch_);
  self_filter_lidar_to_base_yaw_ =
      declare_double("self_filter_lidar_to_base_yaw", self_filter_lidar_to_base_yaw_);
  self_filter_front_crop_enabled_ =
      declare_bool("self_filter_front_crop_enabled", self_filter_front_crop_enabled_);
  self_filter_front_x_min_ =
      declare_double("self_filter_front_x_min", self_filter_front_x_min_);
  self_filter_front_x_max_ =
      declare_double("self_filter_front_x_max", self_filter_front_x_max_);
  self_filter_front_y_min_ =
      declare_double("self_filter_front_y_min", self_filter_front_y_min_);
  self_filter_front_y_max_ =
      declare_double("self_filter_front_y_max", self_filter_front_y_max_);
  self_filter_front_z_min_ =
      declare_double("self_filter_front_z_min", self_filter_front_z_min_);
  self_filter_front_z_max_ =
      declare_double("self_filter_front_z_max", self_filter_front_z_max_);
  self_filter_enforce_monotonic_timestamps_ =
      declare_bool("self_filter_enforce_monotonic_timestamps",
                   self_filter_enforce_monotonic_timestamps_);
  self_filter_drop_stale_stamps_ =
      declare_bool("self_filter_drop_stale_stamps", self_filter_drop_stale_stamps_);
  self_filter_max_stamp_age_s_ =
      std::max(0.0, declare_double("self_filter_max_stamp_age_s", self_filter_max_stamp_age_s_));
  self_filter_stamp_regression_tolerance_s_ = std::max(
      0.0, declare_double("self_filter_stamp_regression_tolerance_s",
                          self_filter_stamp_regression_tolerance_s_));
  self_filter_timebase_regression_tolerance_s_ = std::max(
      0.0, declare_double("self_filter_timebase_regression_tolerance_s",
                          self_filter_timebase_regression_tolerance_s_));
  self_filter_min_input_points_ =
      std::max(0, declare_int("self_filter_min_input_points", self_filter_min_input_points_));
  self_filter_min_output_points_ =
      std::max(0, declare_int("self_filter_min_output_points", self_filter_min_output_points_));
  self_filter_stats_log_period_s_ =
      std::max(0.1, declare_double("self_filter_stats_log_period_s",
                                   self_filter_stats_log_period_s_));

  self_filter_boxes_.clear();
  const auto box_values =
      declare_string_vector("self_filter_box_filters", std::vector<std::string>{});
  try {
    for (const auto& value : box_values) {
      if (!Trim(value).empty()) {
        self_filter_boxes_.push_back(ParseBoxFilter(value));
      }
    }
  } catch (const std::exception& exc) {
    DRIVER_ERROR(node, "Invalid Livox self-filter box config: %s", exc.what());
    self_filter_enabled_ = false;
    self_filter_boxes_.clear();
  }

  ConfigureSelfFilterTransform();

  if (self_filter_enabled_) {
    std::ostringstream names;
    for (size_t i = 0; i < self_filter_boxes_.size(); ++i) {
      if (i > 0) {
        names << ", ";
      }
      names << self_filter_boxes_[i].name;
    }
    DRIVER_INFO(
        node,
        "Livox driver embedded self-filter enabled: /livox/lidar is filtered in-place, "
        "boxes=%s, box_padding=%.3f, min_input_points=%d, min_output_points=%d, "
        "max_stamp_age=%.3fs, lidar_to_base=(%.3f, %.3f, %.3f, %.3f, %.3f, %.4f)",
        self_filter_boxes_.empty() ? "(none)" : names.str().c_str(),
        self_filter_box_padding_, self_filter_min_input_points_, self_filter_min_output_points_,
        self_filter_max_stamp_age_s_, self_filter_lidar_to_base_x_,
        self_filter_lidar_to_base_y_, self_filter_lidar_to_base_z_,
        self_filter_lidar_to_base_roll_, self_filter_lidar_to_base_pitch_,
        self_filter_lidar_to_base_yaw_);
  }
#endif
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, index);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, index);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPclMsg(p_queue, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
#ifdef BUILDING_ROS1
  if (bag_) {
    DRIVER_INFO(*cur_node_, "Waiting to save the bag file!");
    bag_->close();
    DRIVER_INFO(*cur_node_, "Save the bag file successfully!");
    bag_ = nullptr;
  }
#endif
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud2 cloud;
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);
    PublishPointcloud2Data(index, timestamp, cloud);
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    if (!AcceptCustomMsgTimestamp(livox_msg)) {
      continue;
    }
    if (!ApplySelfFilter(livox_msg)) {
      continue;
    }
    PublishCustomPointData(livox_msg, index);
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) {
#ifdef BUILDING_ROS2
  static bool first_log = true;
  if (first_log) {
    std::cout << "error: message type 'pcl::PointCloud' is NOT supported in ROS2, "
              << "please modify the 'xfer_format' field in the launch file"
              << std::endl;
  }
  first_log = false;
  return;
#endif
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud cloud;
    uint64_t timestamp = 0;
    InitPclMsg(pkg, cloud, timestamp);
    FillPointsToPclMsg(pkg, cloud);
    PublishPclData(index, timestamp, cloud);
  }
  return;
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2& cloud) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;
  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp) {
  InitPointcloud2MsgHeader(cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);

  cloud.width = pkg.points_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense     = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  #ifdef BUILDING_ROS1
      cloud.header.stamp = ros::Time( timestamp / 1000000000.0);
  #elif defined BUILDING_ROS2
      cloud.header.stamp = rclcpp::Time(timestamp);
  #endif

  std::vector<LivoxPointXyzrtlt> points;
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point;
    point.x = pkg.points[i].x;
    point.y = pkg.points[i].y;
    point.z = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag = pkg.points[i].tag;
    point.line = pkg.points[i].line;
    point.timestamp = static_cast<double>(pkg.points[i].offset_time);
    points.push_back(std::move(point));
  }
  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<PointCloud2>::SharedPtr publisher_ptr =
    std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index));
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud);
    }
#endif
  }
}

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg.header.frame_id.assign(frame_id_);

#ifdef BUILDING_ROS1
  static uint32_t msg_seq = 0;
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
#endif

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg.timebase = timestamp;

#ifdef BUILDING_ROS1
  livox_msg.header.stamp = ros::Time(timestamp / 1000000000.0);
#elif defined BUILDING_ROS2
  livox_msg.header.stamp = rclcpp::Time(timestamp);
#endif

  livox_msg.point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index);
    livox_msg.lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

bool Lddc::AcceptCustomMsgTimestamp(const CustomMsg& livox_msg) {
#ifndef BUILDING_ROS2
  return true;
#else
  if (!self_filter_enabled_) {
    return true;
  }
  const uint64_t stamp_ns = StampToNs(livox_msg.header.stamp);
  const bool has_stamp = stamp_ns > 0;

  double age_ms = std::numeric_limits<double>::quiet_NaN();
#ifdef BUILDING_ROS2
  if (cur_node_ && has_stamp) {
    const int64_t now_ns = cur_node_->now().nanoseconds();
    age_ms = static_cast<double>(now_ns - static_cast<int64_t>(stamp_ns)) * 1.0e-6;
  }
#endif

  if (self_filter_drop_stale_stamps_ && has_stamp && std::isfinite(age_ms) &&
      age_ms > self_filter_max_stamp_age_s_ * 1000.0) {
    ++self_filter_dropped_stale_count_;
    if (cur_node_ && self_filter_dropped_stale_count_ % 20 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping stale Livox frame before FAST-LIO: stamp_age=%.1fms > %.1fms, "
          "timebase=%" PRIu64 ", dropped_stale=%" PRIu64,
          age_ms, self_filter_max_stamp_age_s_ * 1000.0, livox_msg.timebase,
          self_filter_dropped_stale_count_);
    }
    return false;
  }

  const uint64_t stamp_tolerance_ns = static_cast<uint64_t>(
      self_filter_stamp_regression_tolerance_s_ * 1000000000.0);
  if (self_filter_enforce_monotonic_timestamps_ && has_stamp &&
      self_filter_last_lidar_stamp_ns_ > 0 &&
      stamp_ns + stamp_tolerance_ns < self_filter_last_lidar_stamp_ns_) {
    ++self_filter_dropped_regression_count_;
    if (cur_node_ && self_filter_dropped_regression_count_ % 20 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping out-of-order Livox frame before FAST-LIO: stamp regressed by %.3fs, "
          "last=%" PRIu64 ", current=%" PRIu64 ", dropped_regression=%" PRIu64,
          static_cast<double>(self_filter_last_lidar_stamp_ns_ - stamp_ns) * 1.0e-9,
          self_filter_last_lidar_stamp_ns_, stamp_ns, self_filter_dropped_regression_count_);
    }
    return false;
  }

  const uint64_t timebase_tolerance_ns = static_cast<uint64_t>(
      self_filter_timebase_regression_tolerance_s_ * 1000000000.0);
  if (self_filter_enforce_monotonic_timestamps_ && livox_msg.timebase > 0 &&
      self_filter_last_lidar_timebase_ > 0 &&
      livox_msg.timebase + timebase_tolerance_ns < self_filter_last_lidar_timebase_) {
    ++self_filter_dropped_regression_count_;
    if (cur_node_ && self_filter_dropped_regression_count_ % 20 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping out-of-order Livox frame before FAST-LIO: timebase regressed by %.3fs, "
          "last=%" PRIu64 ", current=%" PRIu64 ", dropped_regression=%" PRIu64,
          static_cast<double>(self_filter_last_lidar_timebase_ - livox_msg.timebase) * 1.0e-9,
          self_filter_last_lidar_timebase_, livox_msg.timebase,
          self_filter_dropped_regression_count_);
    }
    return false;
  }

  if (has_stamp) {
    self_filter_last_lidar_stamp_ns_ = std::max(self_filter_last_lidar_stamp_ns_, stamp_ns);
  }
  if (livox_msg.timebase > 0) {
    self_filter_last_lidar_timebase_ =
        std::max(self_filter_last_lidar_timebase_, livox_msg.timebase);
  }
  return true;
#endif
}

bool Lddc::AcceptImuTimestamp(uint64_t timestamp) {
  if (!self_filter_enabled_ || timestamp == 0) {
    return true;
  }

  double age_ms = std::numeric_limits<double>::quiet_NaN();
#ifdef BUILDING_ROS2
  if (cur_node_) {
    const int64_t now_ns = cur_node_->now().nanoseconds();
    age_ms = static_cast<double>(now_ns - static_cast<int64_t>(timestamp)) * 1.0e-6;
  }
#endif

  if (self_filter_drop_stale_stamps_ && std::isfinite(age_ms) &&
      age_ms > self_filter_max_stamp_age_s_ * 1000.0) {
    ++self_filter_dropped_stale_count_;
    if (cur_node_ && self_filter_dropped_stale_count_ % 50 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping stale Livox IMU before FAST-LIO: stamp_age=%.1fms > %.1fms, "
          "dropped_stale=%" PRIu64,
          age_ms, self_filter_max_stamp_age_s_ * 1000.0, self_filter_dropped_stale_count_);
    }
    return false;
  }

  const uint64_t tolerance_ns = static_cast<uint64_t>(
      self_filter_stamp_regression_tolerance_s_ * 1000000000.0);
  if (self_filter_enforce_monotonic_timestamps_ && self_filter_last_imu_stamp_ns_ > 0 &&
      timestamp + tolerance_ns < self_filter_last_imu_stamp_ns_) {
    ++self_filter_dropped_regression_count_;
    if (cur_node_ && self_filter_dropped_regression_count_ % 50 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping out-of-order Livox IMU before FAST-LIO: stamp regressed by %.3fs, "
          "last=%" PRIu64 ", current=%" PRIu64 ", dropped_regression=%" PRIu64,
          static_cast<double>(self_filter_last_imu_stamp_ns_ - timestamp) * 1.0e-9,
          self_filter_last_imu_stamp_ns_, timestamp, self_filter_dropped_regression_count_);
    }
    return false;
  }

  self_filter_last_imu_stamp_ns_ = std::max(self_filter_last_imu_stamp_ns_, timestamp);
  return true;
}

bool Lddc::RejectBySelfBoxes(const Vec3& point) const {
  if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))) {
    return false;
  }
  for (const auto& box : self_filter_boxes_) {
    if (PointInBox(point, box, self_filter_box_padding_)) {
      return true;
    }
  }
  return false;
}

bool Lddc::RejectByCrop(const Vec3& point) const {
  if (!self_filter_front_crop_enabled_) {
    return false;
  }
  return !(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           point.x >= self_filter_front_x_min_ && point.x <= self_filter_front_x_max_ &&
           point.y >= self_filter_front_y_min_ && point.y <= self_filter_front_y_max_ &&
           point.z >= self_filter_front_z_min_ && point.z <= self_filter_front_z_max_);
}

void Lddc::MaybeLogSelfFilterStats(uint32_t total, uint32_t kept, uint32_t rejected_self,
                                   uint32_t rejected_crop, double stamp_age_ms,
                                   double process_ms) {
#ifndef BUILDING_ROS2
  (void)total;
  (void)kept;
  (void)rejected_self;
  (void)rejected_crop;
  (void)stamp_age_ms;
  (void)process_ms;
  return;
#else
  if (!cur_node_) {
    return;
  }
  const uint64_t now_ns = static_cast<uint64_t>(cur_node_->now().nanoseconds());
  const uint64_t period_ns = static_cast<uint64_t>(
      self_filter_stats_log_period_s_ * 1000000000.0);
  if (self_filter_last_stats_log_ns_ > 0 &&
      now_ns < self_filter_last_stats_log_ns_ + period_ns) {
    return;
  }
  self_filter_last_stats_log_ns_ = now_ns;
  const double ratio = 100.0 * static_cast<double>(kept) /
      static_cast<double>(std::max<uint32_t>(total, 1));
  if (std::isfinite(stamp_age_ms)) {
    DRIVER_INFO(
        *cur_node_,
        "livox embedded self-filter stats: input=%u, kept=%u (%.1f%%), "
        "self_rejected=%u, crop_rejected=%u, stamp_age=%.1fms, process=%.1fms",
        total, kept, ratio, rejected_self, rejected_crop, stamp_age_ms, process_ms);
  } else {
    DRIVER_INFO(
        *cur_node_,
        "livox embedded self-filter stats: input=%u, kept=%u (%.1f%%), "
        "self_rejected=%u, crop_rejected=%u, stamp_age=unknown, process=%.1fms",
        total, kept, ratio, rejected_self, rejected_crop, process_ms);
  }
#endif
}

bool Lddc::ApplySelfFilter(CustomMsg& livox_msg) {
  if (!self_filter_enabled_) {
    return true;
  }

  const auto callback_start = std::chrono::steady_clock::now();
  const uint32_t input_points = static_cast<uint32_t>(livox_msg.points.size());
  double stamp_age_ms = std::numeric_limits<double>::quiet_NaN();
#ifdef BUILDING_ROS2
  const uint64_t stamp_ns = StampToNs(livox_msg.header.stamp);
  if (cur_node_ && stamp_ns > 0) {
    stamp_age_ms =
        static_cast<double>(cur_node_->now().nanoseconds() - static_cast<int64_t>(stamp_ns)) *
        1.0e-6;
  }
#endif

  if (self_filter_min_input_points_ > 0 &&
      input_points < static_cast<uint32_t>(self_filter_min_input_points_)) {
    ++self_filter_dropped_low_points_count_;
    if (cur_node_ && self_filter_dropped_low_points_count_ % 10 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping low-point Livox frame before FAST-LIO: input=%u, point_num=%u, "
          "min_input_points=%d, stamp_age=%.1fms, dropped_low_points=%" PRIu64,
          input_points, livox_msg.point_num, self_filter_min_input_points_, stamp_age_ms,
          self_filter_dropped_low_points_count_);
    }
    return false;
  }

  std::vector<CustomPoint> kept_points;
  kept_points.reserve(livox_msg.points.size());
  uint32_t rejected_self = 0;
  uint32_t rejected_crop = 0;

  for (const auto& point : livox_msg.points) {
    Vec3 lidar_point;
    lidar_point.x = point.x;
    lidar_point.y = point.y;
    lidar_point.z = point.z;
    const Vec3 base_point = TransformPoint(lidar_point, self_filter_lidar_to_base_);
    const bool self_reject = RejectBySelfBoxes(base_point);
    const bool crop_reject = !self_reject && RejectByCrop(base_point);
    if (self_reject) {
      ++rejected_self;
      continue;
    }
    if (crop_reject) {
      ++rejected_crop;
      continue;
    }
    kept_points.push_back(point);
  }

  livox_msg.points.swap(kept_points);
  livox_msg.point_num = static_cast<uint32_t>(livox_msg.points.size());

  const double process_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();

  if (self_filter_min_output_points_ > 0 &&
      livox_msg.point_num < static_cast<uint32_t>(self_filter_min_output_points_)) {
    ++self_filter_dropped_low_points_count_;
    if (cur_node_ && self_filter_dropped_low_points_count_ % 10 == 1) {
      DRIVER_WARN(
          *cur_node_,
          "dropping low-point Livox frame after self-filter: input=%u, kept=%u, "
          "min_output_points=%d, self_rejected=%u, crop_rejected=%u, "
          "stamp_age=%.1fms, process=%.1fms, dropped_low_points=%" PRIu64,
          input_points, livox_msg.point_num, self_filter_min_output_points_, rejected_self,
          rejected_crop, stamp_age_ms, process_ms, self_filter_dropped_low_points_count_);
    }
    return false;
  }

  MaybeLogSelfFilterStats(
      input_points, livox_msg.point_num, rejected_self, rejected_crop, stamp_age_ms, process_ms);
  return true;
}

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg);
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(livox_msg.timebase / 1000000000.0), livox_msg);
    }
#endif
  }
}

void Lddc::InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp) {
#ifdef BUILDING_ROS1
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = pkg.points_num;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  cloud.header.stamp = timestamp / 1000.0;  // to pcl ros time stamp
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl;
#endif
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg) {
#ifdef BUILDING_ROS1
  if (pkg.points.empty()) {
    return;
  }

  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    pcl::PointXYZI point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.intensity = points[i].intensity;

    pcl_msg.points.push_back(std::move(point));
  }
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl;
#endif
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud);
    }
  }
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl;
#endif
  return;
}

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
  imu_msg.header.frame_id = "livox_frame";

  timestamp = imu_data.time_stamp;
#ifdef BUILDING_ROS1
  imu_msg.header.stamp = ros::Time(timestamp / 1000000000.0);  // to ros time stamp
#elif defined BUILDING_ROS2
  imu_msg.header.stamp = rclcpp::Time(timestamp);  // to ros time stamp
#endif

  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp);
  if (!AcceptImuTimestamp(timestamp)) {
    return;
  }

#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = GetCurrentImuPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg);
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_imu_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), imu_msg);
    }
#endif
  }
}

#ifdef BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {
    if (kPointCloud2Msg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud2>(topic_name, queue_size);
    } else if (kLivoxCustomMsg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format", topic_name.c_str());
      return cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
    }
#if 0
    else if (kPclPxyziMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud>(topic_name, queue_size);
    }
#endif
    else if (kLivoxImuMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use imu format", topic_name.c_str());
      return cur_node_->create_publisher<ImuMsg>(topic_name,
          queue_size);
    } else {
      PublisherPtr null_publisher(nullptr);
      return null_publisher;
    }
}
#endif

#ifdef BUILDING_ROS1
PublisherPtr Lddc::GetCurrentPublisher(uint8_t index) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_pub_[index];
    queue_size = queue_size / 8; // queue size is 4 for only one lidar
  } else {
    pub = &global_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      std::string ip_string = IpNumToString(lds_->lidars_[index].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      DRIVER_INFO(*cur_node_, "Support multi topics.");
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->GetNode().advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<livox_ros_driver2::CustomMsg>(name_str,
                                                                queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<PointCloud>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format, set ROS publisher queue "
          "size %d",
          name_str, queue_size);
    }
  }

  return *pub;
}

PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_imu_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      DRIVER_INFO(*cur_node_, "Support multi topics.");
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->GetNode().advertise<sensor_msgs::Imu>(name_str, queue_size);
    DRIVER_INFO(*cur_node_, "%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}
#elif defined BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name,
          queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
}
#endif

void Lddc::CreateBagFile(const std::string &file_name) {
#ifdef BUILDING_ROS1
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str());
  }
#endif
}

}  // namespace livox_ros
