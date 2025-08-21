#include "path_generator/resample_path.hpp"
#include <cmath>

namespace path_generator
{
  ResamplePath::ResamplePath(const rclcpp::NodeOptions &option)
      : Node("resample_path", option)
  {
    declare_parameter("resample_distance", 0.3);
    resample_distance_ = get_parameter("resample_distance").as_double();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ResamplePath::pathCallback, this, std::placeholders::_1));
    resample_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/resample_path", 1);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("resample_path_markers", 10);
  }

  double ResamplePath::calculateDistance(
      const geometry_msgs::msg::PoseStamped &p1,
      const geometry_msgs::msg::PoseStamped &p2)
  {
    double dx = p2.pose.position.x - p1.pose.position.x;
    double dy = p2.pose.position.y - p1.pose.position.y;
    double dz = p2.pose.position.z - p1.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  geometry_msgs::msg::Pose ResamplePath::interpolate(
      const geometry_msgs::msg::Pose &p1,
      const geometry_msgs::msg::Pose &p2,
      double ratio)
  {
    geometry_msgs::msg::Pose pose;

    // 位置の線形補間
    pose.position.x = p1.position.x + ratio * (p2.position.x - p1.position.x);
    pose.position.y = p1.position.y + ratio * (p2.position.y - p1.position.y);
    pose.position.z = p1.position.z + ratio * (p2.position.z - p1.position.z);

    // 方向ベクトルからyaw角を計算
    const double dx = p2.position.x - p1.position.x;
    const double dy = p2.position.y - p1.position.y;
    const double yaw = std::atan2(dy, dx);

    // クォータニオンに変換（yaw角のみ考慮）
    const double half_yaw = yaw * 0.5;
    pose.orientation.w = std::cos(half_yaw);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(half_yaw);

    return pose;
  }

  visualization_msgs::msg::MarkerArray ResamplePath::createMarkers(const nav_msgs::msg::Path &path)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // パスのサイズに基づいてメモリを事前確保
    const size_t path_size = path.poses.size();
    marker_array.markers.reserve(path_size * 2); // ポイント + ライン用

    for (size_t i = 0; i < path_size; i++)
    {
      // ポイントマーカー
      visualization_msgs::msg::Marker marker;
      marker.header = path.header;
      marker.ns = "resample_path_points";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = path.poses[i].pose;
      marker.scale.x = 0.015;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(std::move(marker));

      // ラインマーカー（最初のポイント以外）
      if (i > 0)
      {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header = path.header;
        line_marker.ns = "resample_path_lines";
        line_marker.id = static_cast<int>(i - 1);
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.03; // 線の太さ
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0;

        line_marker.points.reserve(2);
        line_marker.points.push_back(path.poses[i - 1].pose.position);
        line_marker.points.push_back(path.poses[i].pose.position);
        marker_array.markers.push_back(std::move(line_marker));
      }
    }
    return marker_array;
  }

  void ResamplePath::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2)
    {
      RCLCPP_WARN(this->get_logger(), "Received path has fewer than 2 points - cannot resample");
      return;
    }

    nav_msgs::msg::Path resample_path;
    resample_path.header = msg->header;

    // メモリ事前確保（推定サイズ）
    const size_t original_size = msg->poses.size();
    resample_path.poses.reserve(original_size * 2);

    // 最初の点を追加
    resample_path.poses.push_back(msg->poses[0]);

    for (size_t i = 1; i < original_size; i++)
    {
      const auto &prev_pose = msg->poses[i - 1];
      const auto &curr_pose = msg->poses[i];

      const double segment_length = calculateDistance(prev_pose, curr_pose);

      if (segment_length < 1e-6)
      {
        continue;
      }

      const int num_points = static_cast<int>(std::floor(segment_length / resample_distance_));

      for (int j = 1; j <= num_points; j++)
      {
        const double ratio = j * resample_distance_ / segment_length;
        if (ratio >= 1.0)
        {
          break;
        }

        geometry_msgs::msg::PoseStamped interpolated_pose;
        interpolated_pose.header = msg->header;
        interpolated_pose.pose = interpolate(prev_pose.pose, curr_pose.pose, ratio);
        resample_path.poses.push_back(std::move(interpolated_pose));
      }

      // セグメントの終点を追加
      resample_path.poses.push_back(curr_pose);
    }

    resample_path_pub_->publish(resample_path);
    auto markers = createMarkers(resample_path);
    markers_pub_->publish(markers);
  }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_generator::ResamplePath)
