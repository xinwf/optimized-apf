// optimized_apf ROS 节点封装：话题接口 / 参数加载 / 激光转换 / 安全超时
// 核心算法见 apf_core.h（纯 C++，与 ROS 解耦）
#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <string>
#include <vector>

#include "optimized_apf/OptimizedAPFConfig.h"
#include "optimized_apf/apf_core.h"

namespace optimized_apf {

class OptimizedAPFNode {
 public:
  OptimizedAPFNode();
  void spin();

 private:
  void loadParams();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void attractiveCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void resetCallback(const std_msgs::Empty::ConstPtr& msg);
  void reconfigureCallback(optimized_apf::OptimizedAPFConfig& config, uint32_t level);
  void publishStatus(const Output& out);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber attractive_sub_;
  ros::Subscriber reset_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher state_pub_;
  ros::Publisher replan_pub_;
  ros::Publisher deviation_pub_;           // 兼容旧接口，语义同 avoidance_distance
  ros::Publisher avoidance_distance_pub_;
  ros::Publisher repulsive_pub_;
  ros::Publisher nearest_pub_;

  dynamic_reconfigure::Server<optimized_apf::OptimizedAPFConfig> dr_server_;
  dynamic_reconfigure::Server<optimized_apf::OptimizedAPFConfig>::CallbackType dr_cb_;

  APFCore core_;
  Params params_;

  // 输入缓存（单线程 spinOnce，无并发）
  std::vector<Vec2> points_;
  Vec2 v_des_;
  Vec2 position_;
  double yaw_ = 0.0;
  double v_actual_ = 0.0;
  ros::Time last_scan_time_;
  ros::Time last_odom_time_;
  ros::Time last_cmd_time_;
  ros::Time last_time_;
  bool scan_valid_ = false;
  bool odom_valid_ = false;
  bool cmd_valid_ = false;
  bool last_replan_ = false;

  // 接口参数
  double control_rate_ = 30.0;
  double scan_timeout_ = 0.5;
  double odom_timeout_ = 0.5;
  double cmd_timeout_ = 0.5;
  double laser_x_ = 0.0;
  double laser_y_ = 0.0;
  double laser_yaw_ = 0.0;
};

}  // namespace optimized_apf