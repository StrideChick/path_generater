#ifndef RESAMPLE_PATH_HPP_
#define RESAMPLE_PATH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace path_generator
{
  class ResamplePath : public rclcpp::Node
  {
  public:
    explicit ResamplePath(const rclcpp::NodeOptions &options);

  private:
    // 2点間の距離を計算
    double calculateDistance(
        const geometry_msgs::msg::PoseStamped &p1,
        const geometry_msgs::msg::PoseStamped &p2);

    // 線形補間関数
    geometry_msgs::msg::Pose interpolate(
        const geometry_msgs::msg::Pose &p1,
        const geometry_msgs::msg::Pose &p2,
        double ratio);

    visualization_msgs::msg::MarkerArray createMarkers(const nav_msgs::msg::Path &path);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr resample_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    double resample_distance_; // [m]
  };
}

#endif // RESAMPLE_PATH_HPP_
