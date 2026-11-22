#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using LivoxFrame = livox_ros_driver2::msg::CustomMsg;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Point = std::array<float, 4>;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000;
constexpr std::size_t kQueueDepth = 12;

struct Frame
{
  LivoxFrame::ConstSharedPtr message;
  std::int64_t first;
  std::int64_t last;
};

struct Sensor
{
  std::string frame_id;
  std::deque<Frame> pending;
  std::int64_t newest_stamp = 0;
  rclcpp::Publisher<PointCloud>::SharedPtr output;
  rclcpp::Subscription<LivoxFrame>::SharedPtr input;
};

struct Motion
{
  tf2::Transform first;
  tf2::Transform last;
  tf2::Transform sensor_from_base;
};

bool IsValidPoint(const livox_ros_driver2::msg::CustomPoint & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
    (point.x != 0.0F || point.y != 0.0F || point.z != 0.0F);
}

tf2::Transform ToTransform(const geometry_msgs::msg::TransformStamped & stamped)
{
  const auto & rotation = stamped.transform.rotation;
  const auto & translation = stamped.transform.translation;
  return tf2::Transform(
    tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    tf2::Vector3(translation.x, translation.y, translation.z));
}

PointCloud MakeCloud(
  const std::vector<Point> & points, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  PointCloud cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.is_dense = true;
  const std::uint16_t byte_order = 1;
  cloud.is_bigendian = *reinterpret_cast<const unsigned char *>(&byte_order) == 0;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  constexpr auto kFloat = sensor_msgs::msg::PointField::FLOAT32;
  modifier.setPointCloud2Fields(
    4, "x", 1, kFloat, "y", 1, kFloat, "z", 1, kFloat, "intensity", 1, kFloat);
  modifier.resize(points.size());
  if (!points.empty()) {
    static_assert(sizeof(Point) == 4 * sizeof(float), "point must have no padding");
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  }
  return cloud;
}
}  // namespace

class DualMid360Fusion : public rclcpp::Node
{
public:
  DualMid360Fusion()
  : Node("dual_mid360_fusion"),
    buffer_(get_clock()),
    listener_(buffer_, this, false)
  {
    const auto primary_topic = declare_parameter<std::string>(
      "primary_topic", "/livox/lidar_192_168_1_182");
    const auto secondary_topic = declare_parameter<std::string>(
      "secondary_topic", "/livox/lidar_192_168_1_3");
    primary_.frame_id = declare_parameter<std::string>("primary_frame", "livox_frame");
    secondary_.frame_id = declare_parameter<std::string>(
      "secondary_frame", "livox_secondary_frame");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");

    const auto max_delta = declare_parameter<double>("max_pair_delta", 0.06);
    const auto max_age = declare_parameter<double>("max_age", 0.5);
    min_z_ = declare_parameter<double>("min_obstacle_z", 0.05);
    max_z_ = declare_parameter<double>("max_obstacle_z", 0.50);
    if (primary_topic.empty() || primary_topic == secondary_topic || secondary_topic.empty() ||
      primary_.frame_id.empty() || secondary_.frame_id.empty() ||
      primary_.frame_id == secondary_.frame_id || base_frame_.empty())
    {
      throw std::invalid_argument("Two distinct input topics and sensor frames are required");
    }
    if (!std::isfinite(max_delta) || !std::isfinite(max_age) ||
      max_delta < 0.0 || max_age <= 0.0 || max_delta > max_age ||
      !std::isfinite(min_z_) || !std::isfinite(max_z_) || min_z_ > max_z_)
    {
      throw std::invalid_argument("Invalid point cloud time or height bounds");
    }
    max_delta_ = rclcpp::Duration::from_seconds(max_delta).nanoseconds();
    max_age_ = rclcpp::Duration::from_seconds(max_age).nanoseconds();

    const auto qos = rclcpp::SensorDataQoS();
    fused_ = create_publisher<PointCloud>("/lidar/points_fused", qos);
    primary_.output = create_publisher<PointCloud>("/lidar/points_primary", qos);
    secondary_.output = create_publisher<PointCloud>("/lidar/points_secondary", qos);
    primary_.input = create_subscription<LivoxFrame>(
      primary_topic, qos,
      [this](LivoxFrame::ConstSharedPtr message) { QueueFrame(primary_, std::move(message)); });
    secondary_.input = create_subscription<LivoxFrame>(
      secondary_topic, qos,
      [this](LivoxFrame::ConstSharedPtr message) { QueueFrame(secondary_, std::move(message)); });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { PublishPair(); });
  }

private:
  std::int64_t Now()
  {
    const auto now = get_clock()->now().nanoseconds();
    if (now < last_now_) {
      for (auto * sensor : {&primary_, &secondary_}) {
        sensor->pending.clear();
        sensor->newest_stamp = 0;
      }
      last_reference_ = 0;
    }
    last_now_ = now;
    return now;
  }

  void QueueFrame(Sensor & sensor, LivoxFrame::ConstSharedPtr message)
  {
    const auto now = Now();
    const auto & stamp = message->header.stamp;
    const auto first = stamp.sec * kNanosecondsPerSecond + stamp.nanosec;
    if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond ||
      first <= sensor.newest_stamp || now - first > max_age_)
    {
      return;
    }
    std::uint32_t last_offset = 0;
    bool has_points = false;
    for (const auto & point : message->points) {
      if (IsValidPoint(point)) {
        has_points = true;
        last_offset = std::max(last_offset, point.offset_time);
      }
    }
    const auto last = first + last_offset;
    if (!has_points || last - now > max_age_) {
      return;
    }

    sensor.newest_stamp = first;
    sensor.pending.push_back({std::move(message), first, last});
    if (sensor.pending.size() > kQueueDepth) {
      sensor.pending.pop_front();
    }
  }

  tf2::Transform Lookup(
    const std::string & target, const std::string & source, std::int64_t stamp)
  {
    const auto time = tf2::TimePoint(tf2::Duration(stamp));
    return ToTransform(buffer_.lookupTransform(target, source, time));
  }

  Motion GetMotion(
    const Sensor & sensor, const Frame & frame, std::int64_t reference,
    const tf2::Transform & base_from_fixed)
  {
    const auto base_from_sensor = Lookup(base_frame_, sensor.frame_id, reference);
    const auto sensor_from_base = base_from_sensor.inverse();
    if (fixed_frame_.empty()) {
      return {base_from_sensor, base_from_sensor, sensor_from_base};
    }

    const auto at_time = [&](std::int64_t stamp) {
        return stamp == reference ? base_from_sensor :
               base_from_fixed * Lookup(fixed_frame_, sensor.frame_id, stamp);
      };
    const auto at_first = at_time(frame.first);
    const auto at_last = frame.last == frame.first ? at_first : at_time(frame.last);
    return {at_first, at_last, sensor_from_base};
  }

  void AppendPoints(
    const Frame & frame, const Motion & motion, std::vector<Point> & combined,
    std::vector<Point> & obstacles)
  {
    const auto first_rotation = motion.first.getRotation();
    const auto last_rotation = motion.last.getRotation();
    const auto duration = static_cast<double>(frame.last - frame.first);
    const bool interpolate = !fixed_frame_.empty() && duration > 0.0;
    for (const auto & point : frame.message->points) {
      if (!IsValidPoint(point)) {
        continue;
      }
      const tf2::Vector3 input(point.x, point.y, point.z);
      auto base_point = motion.first * input;
      if (interpolate) {
        const auto fraction = point.offset_time / duration;
        const auto rotation = first_rotation.slerp(last_rotation, fraction);
        const auto origin = motion.first.getOrigin().lerp(motion.last.getOrigin(), fraction);
        base_point = tf2::quatRotate(rotation, input) + origin;
      }
      const float intensity = point.reflectivity;
      combined.push_back({
        static_cast<float>(base_point.x()), static_cast<float>(base_point.y()),
        static_cast<float>(base_point.z()), intensity});
      if (base_point.z() >= min_z_ && base_point.z() <= max_z_) {
        const auto sensor_point = motion.sensor_from_base * base_point;
        obstacles.push_back({
          static_cast<float>(sensor_point.x()), static_cast<float>(sensor_point.y()),
          static_cast<float>(sensor_point.z()), intensity});
      }
    }
  }

  void PublishPair()
  {
    const auto now = Now();
    for (auto * sensor : {&primary_, &secondary_}) {
      while (!sensor->pending.empty() && now - sensor->pending.front().first > max_age_) {
        sensor->pending.pop_front();
      }
    }
    while (!primary_.pending.empty() && !secondary_.pending.empty()) {
      const auto & primary = primary_.pending.front();
      const auto & secondary = secondary_.pending.front();
      const auto delta = primary.first - secondary.first;
      if (delta < -max_delta_) {
        primary_.pending.pop_front();
        continue;
      }
      if (delta > max_delta_) {
        secondary_.pending.pop_front();
        continue;
      }

      const auto reference = std::max(primary.last, secondary.last);
      if (reference <= last_reference_) {
        primary_.pending.pop_front();
        secondary_.pending.pop_front();
        continue;
      }
      // A late TF update can still make this pair usable. Retry on the next timer tick.
      if (reference > now) {
        return;
      }
      Motion primary_motion;
      Motion secondary_motion;
      try {
        const auto base_from_fixed = fixed_frame_.empty() ? tf2::Transform::getIdentity() :
          Lookup(base_frame_, fixed_frame_, reference);
        primary_motion = GetMotion(primary_, primary, reference, base_from_fixed);
        secondary_motion = GetMotion(secondary_, secondary, reference, base_from_fixed);
      } catch (const tf2::TransformException & error) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        return;
      }

      std::vector<Point> combined;
      std::vector<Point> primary_obstacles;
      std::vector<Point> secondary_obstacles;
      combined.reserve(primary.message->points.size() + secondary.message->points.size());
      primary_obstacles.reserve(primary.message->points.size());
      secondary_obstacles.reserve(secondary.message->points.size());
      AppendPoints(primary, primary_motion, combined, primary_obstacles);
      AppendPoints(secondary, secondary_motion, combined, secondary_obstacles);
      primary_.pending.pop_front();
      secondary_.pending.pop_front();

      const auto stamp = rclcpp::Time(reference, get_clock()->get_clock_type());
      fused_->publish(MakeCloud(combined, base_frame_, stamp));
      primary_.output->publish(MakeCloud(primary_obstacles, primary_.frame_id, stamp));
      secondary_.output->publish(MakeCloud(secondary_obstacles, secondary_.frame_id, stamp));
      last_reference_ = reference;
      return;
    }
  }

  Sensor primary_;
  Sensor secondary_;
  std::string base_frame_;
  std::string fixed_frame_;
  std::int64_t max_delta_;
  std::int64_t max_age_;
  std::int64_t last_now_ = 0;
  std::int64_t last_reference_ = 0;
  double min_z_;
  double max_z_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<PointCloud>::SharedPtr fused_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DualMid360Fusion>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("dual_mid360_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
