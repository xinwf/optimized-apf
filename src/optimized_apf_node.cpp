#include "optimized_apf/optimized_apf_node.h"

#include <algorithm>
#include <cmath>

namespace optimized_apf {

namespace {

void sanitizePositive(double* value, double fallback, const char* name) {
  if (!std::isfinite(*value) || *value <= 0.0) {
    ROS_WARN("[optimized_apf] 参数 %s 必须为正有限数，当前 %.3f，恢复为 %.3f",
             name, *value, fallback);
    *value = fallback;
  }
}

void sanitizeNonNegative(double* value, double fallback, const char* name) {
  if (!std::isfinite(*value) || *value < 0.0) {
    ROS_WARN("[optimized_apf] 参数 %s 必须为非负有限数，当前 %.3f，恢复为 %.3f",
             name, *value, fallback);
    *value = fallback;
  }
}

bool inputExpired(const ros::Time& now, const ros::Time& last, double timeout) {
  const double age = (now - last).toSec();
  return !std::isfinite(age) || age < 0.0 || age > timeout;
}

}  // namespace

OptimizedAPFNode::OptimizedAPFNode()
    : nh_(), pnh_("~"), dr_server_(pnh_), last_time_(ros::Time::now()) {
  loadParams();
  core_.setParams(params_);

  scan_sub_ = nh_.subscribe("scan", 1, &OptimizedAPFNode::scanCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &OptimizedAPFNode::odomCallback, this);
  attractive_sub_ =
      nh_.subscribe("attractive_velocity", 1, &OptimizedAPFNode::attractiveCallback, this);
  reset_sub_ = pnh_.subscribe("reset", 1, &OptimizedAPFNode::resetCallback, this);

  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  state_pub_ = pnh_.advertise<std_msgs::Int8>("state", 1);
  replan_pub_ = pnh_.advertise<std_msgs::Bool>("replan_request", 1, true);
  deviation_pub_ = pnh_.advertise<std_msgs::Float32>("deviation", 1);
  avoidance_distance_pub_ = pnh_.advertise<std_msgs::Float32>("avoidance_distance", 1);
  repulsive_pub_ = pnh_.advertise<geometry_msgs::Vector3>("repulsive", 1);
  nearest_pub_ = pnh_.advertise<std_msgs::Float32>("nearest_obstacle", 1);

  // latched 话题也发布确定的初始状态，避免晚启动的上层一直收不到 false。
  std_msgs::Bool initial_replan;
  initial_replan.data = false;
  replan_pub_.publish(initial_replan);

  dr_cb_ = boost::bind(&OptimizedAPFNode::reconfigureCallback, this, _1, _2);
  dr_server_.setCallback(dr_cb_);

  ROS_INFO("[optimized_apf] started: rate=%.1fHz eta_0=%.2f d_stop=%.2f radius=%.2f",
           control_rate_, params_.eta_0, params_.d_stop, params_.robot_radius);
}

void OptimizedAPFNode::loadParams() {
  const Params defaults;
  pnh_.param("k_rep", params_.k_rep, defaults.k_rep);
  pnh_.param("eta_0", params_.eta_0, defaults.eta_0);
  pnh_.param("robot_radius", params_.robot_radius, defaults.robot_radius);
  pnh_.param("d_stop", params_.d_stop, defaults.d_stop);
  pnh_.param("estop_hysteresis", params_.estop_hysteresis, defaults.estop_hysteresis);
  pnh_.param("v_nominal", params_.v_nominal, defaults.v_nominal);
  pnh_.param("k_v_scale", params_.k_v_scale, defaults.k_v_scale);
  pnh_.param("v_max", params_.v_max, defaults.v_max);
  pnh_.param("w_max", params_.w_max, defaults.w_max);
  pnh_.param("a_max", params_.a_max, defaults.a_max);
  pnh_.param("alpha_max", params_.alpha_max, defaults.alpha_max);
  pnh_.param("k_yaw", params_.k_yaw, defaults.k_yaw);
  pnh_.param("yaw_deadband", params_.yaw_deadband, defaults.yaw_deadband);
  pnh_.param("stall_speed", params_.stall_speed, defaults.stall_speed);
  pnh_.param("stall_time", params_.stall_time, defaults.stall_time);
  pnh_.param("progress_check_time", params_.progress_check_time,
             defaults.progress_check_time);
  pnh_.param("progress_min_distance", params_.progress_min_distance,
             defaults.progress_min_distance);
  pnh_.param("escape_speed", params_.escape_speed, defaults.escape_speed);
  pnh_.param("escape_min_distance", params_.escape_min_distance,
             defaults.escape_min_distance);
  pnh_.param("escape_time", params_.escape_time, defaults.escape_time);
  pnh_.param("escape_max_tries", params_.escape_max_tries, defaults.escape_max_tries);
  pnh_.param("block_time", params_.block_time, defaults.block_time);
  pnh_.param("normal_reset_time", params_.normal_reset_time, defaults.normal_reset_time);
  pnh_.param("control_rate", control_rate_, 30.0);
  pnh_.param("scan_timeout", scan_timeout_, 0.5);
  pnh_.param("odom_timeout", odom_timeout_, 0.5);
  pnh_.param("cmd_timeout", cmd_timeout_, 0.5);
  pnh_.param("laser_x", laser_x_, 0.0);
  pnh_.param("laser_y", laser_y_, 0.0);
  pnh_.param("laser_yaw", laser_yaw_, 0.0);

  sanitizeNonNegative(&params_.k_rep, defaults.k_rep, "k_rep");
  sanitizeNonNegative(&params_.robot_radius, defaults.robot_radius, "robot_radius");
  sanitizeNonNegative(&params_.estop_hysteresis, defaults.estop_hysteresis,
                      "estop_hysteresis");
  sanitizePositive(&params_.v_nominal, defaults.v_nominal, "v_nominal");
  sanitizeNonNegative(&params_.k_v_scale, defaults.k_v_scale, "k_v_scale");
  sanitizeNonNegative(&params_.v_max, defaults.v_max, "v_max");
  sanitizeNonNegative(&params_.w_max, defaults.w_max, "w_max");
  sanitizePositive(&params_.a_max, defaults.a_max, "a_max");
  sanitizePositive(&params_.alpha_max, defaults.alpha_max, "alpha_max");
  sanitizeNonNegative(&params_.k_yaw, defaults.k_yaw, "k_yaw");
  sanitizeNonNegative(&params_.yaw_deadband, defaults.yaw_deadband, "yaw_deadband");
  sanitizeNonNegative(&params_.stall_speed, defaults.stall_speed, "stall_speed");
  sanitizePositive(&params_.stall_time, defaults.stall_time, "stall_time");
  sanitizePositive(&params_.progress_check_time, defaults.progress_check_time,
                   "progress_check_time");
  sanitizeNonNegative(&params_.progress_min_distance, defaults.progress_min_distance,
                      "progress_min_distance");
  sanitizeNonNegative(&params_.escape_speed, defaults.escape_speed, "escape_speed");
  sanitizePositive(&params_.escape_min_distance, defaults.escape_min_distance,
                   "escape_min_distance");
  sanitizePositive(&params_.escape_time, defaults.escape_time, "escape_time");
  sanitizePositive(&params_.block_time, defaults.block_time, "block_time");
  sanitizeNonNegative(&params_.normal_reset_time, defaults.normal_reset_time,
                      "normal_reset_time");
  sanitizePositive(&control_rate_, 30.0, "control_rate");
  sanitizePositive(&scan_timeout_, 0.5, "scan_timeout");
  sanitizePositive(&odom_timeout_, 0.5, "odom_timeout");
  sanitizePositive(&cmd_timeout_, 0.5, "cmd_timeout");
  if (!std::isfinite(laser_x_)) {
    ROS_WARN("[optimized_apf] 参数 laser_x 非有限数，恢复为 0");
    laser_x_ = 0.0;
  }
  if (!std::isfinite(laser_y_)) {
    ROS_WARN("[optimized_apf] 参数 laser_y 非有限数，恢复为 0");
    laser_y_ = 0.0;
  }
  if (!std::isfinite(laser_yaw_)) {
    ROS_WARN("[optimized_apf] 参数 laser_yaw 非有限数，恢复为 0");
    laser_yaw_ = 0.0;
  }

  if (params_.escape_max_tries < 1) {
    ROS_WARN("[optimized_apf] escape_max_tries 必须 >= 1，恢复为 %d",
             defaults.escape_max_tries);
    params_.escape_max_tries = defaults.escape_max_tries;
  }
  params_.yaw_deadband = std::min(params_.yaw_deadband, 3.14159265358979323846);
  if (!std::isfinite(params_.eta_0) || params_.eta_0 <= params_.robot_radius) {
    const double old_eta = params_.eta_0;
    params_.eta_0 = params_.robot_radius + 0.5;
    ROS_WARN("[optimized_apf] eta_0(%.3f) 必须大于 robot_radius(%.3f)，调整为 %.3f",
             old_eta, params_.robot_radius, params_.eta_0);
  }
  if (!std::isfinite(params_.d_stop) || params_.d_stop <= params_.robot_radius) {
    ROS_WARN("[optimized_apf] d_stop(%.3f) <= robot_radius(%.3f)，急停距离已抬升",
             params_.d_stop, params_.robot_radius);
    params_.d_stop = params_.robot_radius + 0.15;
  }
  if (params_.escape_speed > params_.v_max) {
    ROS_WARN("[optimized_apf] escape_speed 超过 v_max，已限制为 %.3f", params_.v_max);
    params_.escape_speed = params_.v_max;
  }
  if (params_.stall_speed >= params_.v_max) {
    params_.stall_speed = 0.5 * params_.v_max;
    ROS_WARN("[optimized_apf] stall_speed 相对 v_max 过大，已调整为 %.3f",
             params_.stall_speed);
  }
  const double max_progress = params_.v_max * params_.progress_check_time;
  if (params_.progress_min_distance > max_progress) {
    params_.progress_min_distance = 0.5 * max_progress;
    ROS_WARN("[optimized_apf] progress_min_distance 超出速度能力，已调整为 %.3f",
             params_.progress_min_distance);
  }
  const double max_escape_distance = params_.v_max * params_.escape_time;
  if (params_.escape_min_distance > max_escape_distance) {
    params_.escape_min_distance = 0.5 * max_escape_distance;
    ROS_WARN("[optimized_apf] escape_min_distance 超出单次逃逸能力，已调整为 %.3f",
             params_.escape_min_distance);
  }
}

void OptimizedAPFNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  points_.clear();
  last_scan_time_ = ros::Time::now();
  scan_valid_ = false;

  const bool metadata_valid =
      !msg->ranges.empty() && std::isfinite(msg->angle_min) &&
      std::isfinite(msg->angle_increment) && std::isfinite(msg->range_min) &&
      std::isfinite(msg->range_max) && msg->range_min >= 0.0 &&
      msg->range_max > msg->range_min;
  if (!metadata_valid) {
    ROS_WARN_THROTTLE(2.0, "[optimized_apf] 激光消息为空或元数据无效，输出零速度");
    return;
  }

  const double ca = std::cos(laser_yaw_);
  const double sa = std::sin(laser_yaw_);
  points_.reserve(msg->ranges.size());
  size_t valid_rays = 0;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double r = msg->ranges[i];
    // +inf 表示该射线量程内无返回，是合法的空旷观测，不等于传感器故障。
    if (std::isinf(r) && r > 0.0) {
      ++valid_rays;
      continue;
    }
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    if (!std::isfinite(angle)) continue;
    const double xl = r * std::cos(angle);
    const double yl = r * std::sin(angle);
    const double xb = laser_x_ + ca * xl - sa * yl;
    const double yb = laser_y_ + sa * xl + ca * yl;
    if (!std::isfinite(xb) || !std::isfinite(yb)) continue;
    ++valid_rays;
    points_.emplace_back(xb, yb);
  }

  scan_valid_ = valid_rays > 0;
  if (!scan_valid_) {
    points_.clear();
    ROS_WARN_THROTTLE(2.0, "[optimized_apf] 激光帧没有任何有效测量，输出零速度");
  }
}

void OptimizedAPFNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  last_odom_time_ = ros::Time::now();
  odom_valid_ = false;

  const auto& q = msg->pose.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const double v = msg->twist.twist.linear.x;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(v) ||
      !std::isfinite(norm) || norm < 1e-6) {
    ROS_WARN_THROTTLE(2.0, "[optimized_apf] 里程计包含无效位置、速度或四元数");
    return;
  }

  const double qx = q.x / norm;
  const double qy = q.y / norm;
  const double qz = q.z / norm;
  const double qw = q.w / norm;
  position_ = Vec2(x, y);
  v_actual_ = v;
  yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));
  odom_valid_ = std::isfinite(yaw_);
}

void OptimizedAPFNode::attractiveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  last_cmd_time_ = ros::Time::now();
  cmd_valid_ = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y);
  if (cmd_valid_) {
    v_des_ = Vec2(msg->linear.x, msg->linear.y);
  } else {
    v_des_ = Vec2(0.0, 0.0);
    ROS_WARN_THROTTLE(2.0, "[optimized_apf] 期望速度包含 NaN/Inf，输出零速度");
  }
}

void OptimizedAPFNode::resetCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  ROS_INFO("[optimized_apf] external reset: 清除阻塞/重规划/逃逸状态");
  core_.reset();
  last_replan_ = true;
}

void OptimizedAPFNode::reconfigureCallback(optimized_apf::OptimizedAPFConfig& config, uint32_t /*level*/) {
  Params p = core_.params();
  if (std::isfinite(config.k_rep)) p.k_rep = std::max(0.0, config.k_rep);
  if (std::isfinite(config.eta_0)) p.eta_0 = config.eta_0;
  if (std::isfinite(config.d_stop)) p.d_stop = config.d_stop;
  if (std::isfinite(config.v_max)) p.v_max = std::max(0.0, config.v_max);
  if (std::isfinite(config.w_max)) p.w_max = std::max(0.0, config.w_max);
  if (std::isfinite(config.k_yaw)) p.k_yaw = std::max(0.0, config.k_yaw);

  if (p.eta_0 <= p.robot_radius) {
    p.eta_0 = p.robot_radius + 0.5;
    config.eta_0 = p.eta_0;
    ROS_WARN("[optimized_apf] eta_0 必须大于 robot_radius，已调整为 %.3f", p.eta_0);
  }
  if (p.d_stop <= p.robot_radius) {
    p.d_stop = p.robot_radius + 0.15;
    config.d_stop = p.d_stop;
  }
  if (p.stall_speed >= p.v_max) {
    p.stall_speed = 0.5 * p.v_max;
    ROS_WARN("[optimized_apf] v_max 变化后 stall_speed 过大，已调整为 %.3f", p.stall_speed);
  }
  const double max_progress = p.v_max * p.progress_check_time;
  if (p.progress_min_distance > max_progress) {
    p.progress_min_distance = 0.5 * max_progress;
    ROS_WARN("[optimized_apf] v_max 变化后进展阈值不可达，已调整为 %.3f",
             p.progress_min_distance);
  }
  const double max_escape_distance = p.v_max * p.escape_time;
  if (p.escape_min_distance > max_escape_distance) {
    p.escape_min_distance = 0.5 * max_escape_distance;
    ROS_WARN("[optimized_apf] v_max 变化后逃逸位移阈值不可达，已调整为 %.3f",
             p.escape_min_distance);
  }
  core_.setParams(p);
}

void OptimizedAPFNode::publishStatus(const Output& out) {
  std_msgs::Int8 state_msg;
  state_msg.data = static_cast<int8_t>(out.state);
  state_pub_.publish(state_msg);

  if (out.replan_request != last_replan_) {
    std_msgs::Bool replan_msg;
    replan_msg.data = out.replan_request;
    replan_pub_.publish(replan_msg);
    last_replan_ = out.replan_request;
    if (out.replan_request) {
      ROS_WARN("[optimized_apf] 永久阻塞：已发布重规划请求，等待上层处理或 /optimized_apf/reset");
    }
  }

  std_msgs::Float32 distance_msg;
  distance_msg.data = static_cast<float>(out.avoidance_distance);
  deviation_pub_.publish(distance_msg);  // 旧话题兼容，不代表横向路径偏差
  avoidance_distance_pub_.publish(distance_msg);

  geometry_msgs::Vector3 rep_msg;
  rep_msg.x = out.repulsive.x;
  rep_msg.y = out.repulsive.y;
  rep_msg.z = 0.0;
  repulsive_pub_.publish(rep_msg);

  std_msgs::Float32 nearest_msg;
  nearest_msg.data = static_cast<float>(out.nearest_obstacle);
  nearest_pub_.publish(nearest_msg);
}

void OptimizedAPFNode::spin() {
  ros::Rate rate(control_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (!std::isfinite(dt) || dt <= 0.0) dt = 1.0 / control_rate_;
    last_time_ = now;

    const bool scan_ok = scan_valid_ && !inputExpired(now, last_scan_time_, scan_timeout_);
    const bool odom_ok = odom_valid_ && !inputExpired(now, last_odom_time_, odom_timeout_);
    const bool cmd_ok = cmd_valid_ && !inputExpired(now, last_cmd_time_, cmd_timeout_);

    Output out;
    if (!scan_ok) {
      out = core_.halt(State::SENSOR_FAULT);
      ROS_WARN_THROTTLE(2.0, "[optimized_apf] 激光未收到、无效或超时，输出零速度");
    } else if (!odom_ok) {
      out = core_.halt(State::ODOM_TIMEOUT);
      ROS_WARN_THROTTLE(2.0, "[optimized_apf] 里程计未收到、无效或超时，输出零速度");
    } else if (!cmd_ok) {
      out = core_.halt(State::CMD_TIMEOUT);
      ROS_WARN_THROTTLE(2.0, "[optimized_apf] 期望速度未收到、无效或超时，输出零速度");
    } else {
      out = core_.update(v_des_, position_, yaw_, v_actual_, points_, dt);
    }

    geometry_msgs::Twist cmd;
    if (std::isfinite(out.v) && std::isfinite(out.w)) {
      cmd.linear.x = out.v;
      cmd.angular.z = out.w;
    }
    cmd_pub_.publish(cmd);
    publishStatus(out);
    rate.sleep();
  }

  geometry_msgs::Twist stop;
  cmd_pub_.publish(stop);
}

}  // namespace optimized_apf

int main(int argc, char** argv) {
  ros::init(argc, argv, "optimized_apf");
  optimized_apf::OptimizedAPFNode node;
  node.spin();
  return 0;
}
