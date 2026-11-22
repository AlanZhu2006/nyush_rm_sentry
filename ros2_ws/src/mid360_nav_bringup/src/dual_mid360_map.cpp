#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using PointCloud = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Voxel = std::array<std::int32_t, 3>;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

struct VoxelHash
{
  std::size_t operator()(const Voxel & voxel) const noexcept
  {
    std::size_t hash = 0;
    for (const auto axis : voxel) {
      hash ^= std::hash<std::int32_t>{}(axis) + 0x9e3779b9U + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

struct Centroid
{
  std::array<double, 4> sum{};
  std::uint64_t count = 0;

  void Add(const tf2::Vector3 & point, float intensity)
  {
    sum[0] += point.x();
    sum[1] += point.y();
    sum[2] += point.z();
    sum[3] += intensity;
    ++count;
  }
};

using VoxelMap = std::unordered_map<Voxel, Centroid, VoxelHash>;

const PointField * FindFloatField(const PointCloud & cloud, const char * name)
{
  const auto field = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [name](const PointField & candidate) {return candidate.name == name;});
  if (field == cloud.fields.end()) {
    return nullptr;
  }
  if (field->datatype != PointField::FLOAT32 || field->count != 1 ||
    field->offset > cloud.point_step - sizeof(float))
  {
    throw std::invalid_argument(std::string("Invalid point field: ") + name);
  }
  return &*field;
}

float ReadFloat(const std::uint8_t * data, const PointField & field, bool reverse_bytes)
{
  std::array<std::uint8_t, sizeof(float)> bytes;
  std::memcpy(bytes.data(), data + field.offset, bytes.size());
  if (reverse_bytes) {
    std::reverse(bytes.begin(), bytes.end());
  }
  float value;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

bool BigEndian()
{
  const std::uint16_t one = 1;
  return *reinterpret_cast<const unsigned char *>(&one) == 0;
}

struct TemporaryDirectory
{
  std::string path;
  ~TemporaryDirectory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};

void WritePcd(const std::filesystem::path & path, const VoxelMap & voxels)
{
  // A private directory keeps partial data inaccessible while letting the PCD
  // inherit the destination's default ACL or umask before the atomic rename.
  std::string directory = path.string() + ".tmp.XXXXXX";
  if (::mkdtemp(directory.data()) == nullptr) {
    throw std::system_error(errno, std::generic_category(), "Create temporary map directory");
  }
  TemporaryDirectory temporary{std::move(directory)};
  const auto filename = std::filesystem::path(temporary.path) / "map.pcd";
  auto * stream = std::fopen(filename.c_str(), "wb");
  if (stream == nullptr) {
    throw std::system_error(errno, std::generic_category(), "Open temporary PCD");
  }
  std::unique_ptr<std::FILE, decltype(&std::fclose)> file(stream, &std::fclose);
  if (std::fprintf(
      file.get(),
      "# .PCD v0.7 - Point Cloud Data file format\n"
      "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
      "COUNT 1 1 1 1\nWIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      "POINTS %zu\nDATA binary\n",
      voxels.size(), voxels.size()) < 0)
  {
    throw std::runtime_error("Could not write PCD header");
  }

  const bool reverse_bytes = BigEndian();
  for (const auto & entry : voxels) {
    const auto & voxel = entry.second;
    std::array<std::array<std::uint8_t, sizeof(float)>, 4> bytes;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      const auto value = static_cast<float>(voxel.sum[i] / voxel.count);
      std::memcpy(bytes[i].data(), &value, sizeof(value));
      if (reverse_bytes) {
        std::reverse(bytes[i].begin(), bytes[i].end());
      }
    }
    if (std::fwrite(bytes.data(), sizeof(float), bytes.size(), file.get()) != bytes.size()) {
      throw std::runtime_error("Could not write PCD points");
    }
  }
  if (std::fflush(file.get()) != 0 || ::fsync(::fileno(file.get())) != 0) {
    throw std::system_error(errno, std::generic_category(), "Flush PCD");
  }
  if (std::fclose(file.release()) != 0) {
    throw std::system_error(errno, std::generic_category(), "Close PCD");
  }
  std::filesystem::rename(filename, path);
}
}  // namespace

class DualMid360Map : public rclcpp::Node
{
public:
  DualMid360Map()
  : Node("dual_mid360_map"), buffer_(get_clock()), listener_(buffer_)
  {
    path_ = declare_parameter<std::string>("map_file_path", "");
    frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    resolution_ = declare_parameter<double>("voxel_resolution", 0.05);
    const auto max_voxels = declare_parameter<std::int64_t>("max_voxels", 1000000);
    if (path_.empty() || path_.filename().empty() || frame_.empty() ||
      !std::isfinite(resolution_) || resolution_ <= 0.0 || max_voxels <= 0)
    {
      throw std::invalid_argument("Set map_file_path, fixed_frame and positive voxel bounds");
    }
    max_voxels_ = static_cast<std::size_t>(max_voxels);

    cloud_ = create_subscription<PointCloud>(
      "/lidar/points_fused", rclcpp::SensorDataQoS(),
      [this](PointCloud::ConstSharedPtr message) {Accumulate(*message);});
    save_ = create_service<std_srvs::srv::Trigger>(
      "/dual_map_save", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {Save(*response);});
  }

private:
  void Accumulate(const PointCloud & cloud)
  {
    if (!accumulation_error_.empty() || cloud.width == 0 || cloud.height == 0) {
      return;
    }
    const auto & stamp = cloud.header.stamp;
    if (cloud.point_step < sizeof(float) || cloud.header.frame_id.empty() ||
      stamp.sec < 0 || stamp.nanosec >= 1000000000 ||
      (stamp.sec == 0 && stamp.nanosec == 0) ||
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
      static_cast<std::uint64_t>(cloud.height) * cloud.row_step > cloud.data.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid fused cloud layout or stamp");
      return;
    }
    try {
      const auto * x = FindFloatField(cloud, "x");
      const auto * y = FindFloatField(cloud, "y");
      const auto * z = FindFloatField(cloud, "z");
      const auto * intensity_field = FindFloatField(cloud, "intensity");
      if (x == nullptr || y == nullptr || z == nullptr) {
        throw std::invalid_argument("Fused point cloud must contain x, y and z");
      }

      // Zero stamps were rejected above so TF never substitutes the latest pose.
      const auto pose = buffer_.lookupTransform(
        frame_, cloud.header.frame_id, rclcpp::Time(stamp),
        rclcpp::Duration::from_seconds(0.05));
      const auto & rotation = pose.transform.rotation;
      const auto & translation = pose.transform.translation;
      const tf2::Transform transform(
        tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        tf2::Vector3(translation.x, translation.y, translation.z));
      const bool reverse_bytes = cloud.is_bigendian != BigEndian();
      for (std::uint32_t row = 0; row < cloud.height; ++row) {
        const auto * data = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step;
        for (std::uint32_t column = 0; column < cloud.width; ++column, data += cloud.point_step) {
          const auto px = ReadFloat(data, *x, reverse_bytes);
          const auto py = ReadFloat(data, *y, reverse_bytes);
          const auto pz = ReadFloat(data, *z, reverse_bytes);
          if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
            continue;
          }
          const auto intensity = intensity_field ?
            ReadFloat(data, *intensity_field, reverse_bytes) : 0.0F;
          AddPoint(transform * tf2::Vector3(px, py, pz), std::isfinite(intensity) ? intensity : 0.0F);
        }
      }
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping fused cloud without stamped TF: %s", error.what());
    } catch (const std::exception & error) {
      accumulation_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Dual map is incomplete and will not be saved: %s", error.what());
    }
  }

  void AddPoint(const tf2::Vector3 & point, float intensity)
  {
    Voxel voxel;
    for (std::size_t axis = 0; axis < voxel.size(); ++axis) {
      const double cell = std::floor(point[axis] / resolution_);
      if (!std::isfinite(cell) || cell < std::numeric_limits<std::int32_t>::min() ||
        cell > std::numeric_limits<std::int32_t>::max())
      {
        throw std::runtime_error("Fused point exceeds the supported voxel coordinate range");
      }
      voxel[axis] = static_cast<std::int32_t>(cell);
    }
    auto entry = voxels_.find(voxel);
    if (entry == voxels_.end()) {
      if (voxels_.size() >= max_voxels_) {
        throw std::runtime_error("Dual map reached max_voxels=" + std::to_string(max_voxels_));
      }
      entry = voxels_.try_emplace(voxel).first;
    }
    entry->second.Add(point, intensity);
  }

  void Save(std_srvs::srv::Trigger::Response & result)
  {
    if (!accumulation_error_.empty()) {
      result.message = "Incomplete map: " + accumulation_error_;
      return;
    }
    if (voxels_.empty()) {
      result.message = "No transformed fused points have been collected";
      return;
    }
    try {
      result.message = "Saved " + std::to_string(voxels_.size()) + " voxels to " + path_.string();
      WritePcd(path_, voxels_);
      result.success = true;
    } catch (const std::exception & error) {
      result.message = error.what();
    }
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::filesystem::path path_;
  std::string frame_;
  double resolution_;
  std::size_t max_voxels_;
  VoxelMap voxels_;
  std::string accumulation_error_;
  rclcpp::Subscription<PointCloud>::SharedPtr cloud_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DualMid360Map>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("dual_mid360_map"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
