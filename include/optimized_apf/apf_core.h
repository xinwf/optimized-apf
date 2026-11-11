// 核心 APF 局部避障算法（纯 C++，不依赖 ROS，可独立移植）
//
// 移植自 artificial_potential_fields0/APF_unomni.cpp，面向差速割草机器人：
//  [修复] 机体系转向、角度归一化、低速斥力、硬编码参数和无效分段条件
//  [增强] 急停滞环、输出斜率限制、低速停滞及位置无进展检测
//  [增强] 带实时斥力的侧向逃逸、固定参考航向换侧、持续受阻重规划
//  [增强] 零期望速度禁止自主避障移动，输入故障由 ROS 节点触发 halt()
//  [诊断] avoidance_distance 是避障期间实际行驶里程，不是路径横向偏差
//
// 坐标约定：期望速度、斥力和障碍点均为机体系(base_footprint)；
//           position/yaw 属于同一世界/odom 系，仅用于进展检查和逃逸。
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace optimized_apf {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
  Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
  Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
  Vec2 operator/(double s) const { return Vec2(x / s, y / s); }
  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  double norm() const { return std::hypot(x, y); }
};

// 节点状态（经 /optimized_apf/state 以 Int8 发布，原 0~4 数值保持不变）
enum class State : uint8_t {
  NORMAL = 0,        // 正常跟踪（无斥力）
  AVOIDING = 1,      // 避障中（斥力生效）
  ESCAPING = 2,      // 局部极小逃逸中
  ESTOP = 3,         // 急停（最近障碍 < d_stop）
  BLOCKED = 4,       // 永久阻塞，等待上层重规划
  IDLE = 5,          // 有效零期望速度，禁止避障自主移动
  CMD_TIMEOUT = 6,   // 期望速度未收到、无效或超时
  SENSOR_FAULT = 7,  // 激光未收到、无效或超时
  ODOM_TIMEOUT = 8   // 里程计未收到、无效或超时
};

struct Params {
  // 斥力场
  double k_rep = 0.5;             // 斥力增益
  double eta_0 = 2.0;             // 障碍影响距离(m)，须 > robot_radius
  double robot_radius = 0.4;      // 机体外接圆半径(m)
  double d_stop = 0.55;           // 急停距离(m)，须 > robot_radius
  double estop_hysteresis = 0.1;  // 急停解除滞环(m)
  // 速度自适应
  double v_nominal = 0.8;         // 标称速度(m/s)
  double k_v_scale = 0.5;         // 速度自适应系数
  // 限幅与平滑
  double v_max = 1.0;             // 最大前进速度(m/s)
  double w_max = 1.0;             // 最大角速度(rad/s)
  double a_max = 0.6;             // 线加速度限幅(m/s^2)
  double alpha_max = 1.5;         // 角加速度限幅(rad/s^2)
  double k_yaw = 1.5;             // 航向 P 增益
  double yaw_deadband = 0.05;     // 航向误差死区(rad)
  // 停滞、无进展检测与逃逸
  double stall_speed = 0.05;        // 低速停滞判定速度(m/s)
  double stall_time = 2.0;          // 低速停滞触发时间(s)
  double progress_check_time = 5.0; // 避障中位置进展检查窗口(s)
  double progress_min_distance = 0.08; // 窗口内最小净位移(m)
  double escape_speed = 0.3;        // 逃逸期望速度(m/s)，仍受 v_max 限制
  double escape_min_distance = 0.3; // 沿逃逸目标方向成功脱困所需净位移(m)
  double escape_time = 3.0;         // 单次逃逸时长(s)
  int escape_max_tries = 3;         // 逃逸失败次数上限
  double block_time = 20.0;         // 持续受阻上限(s)
  // 恢复
  double normal_reset_time = 3.0;   // 正常行驶持续该时间后清除阻塞/重规划
};

struct Output {
  double v = 0.0;                  // 前进速度指令(m/s)
  double w = 0.0;                  // 角速度指令(rad/s)
  State state = State::NORMAL;
  bool replan_request = false;     // 锁存的重规划请求
  double avoidance_distance = 0.0; // 本次避障实际行驶里程，不是横向路径偏差
  double nearest_obstacle = std::numeric_limits<double>::infinity();
  Vec2 repulsive;                  // 合斥力（机体系，调试用）
  bool avoiding = false;           // 斥力是否生效
};

class APFCore {
 public:
  void setParams(const Params& p) { p_ = p; }
  const Params& params() const { return p_; }

  // 上层重规划完成后调用，清除阻塞、重规划、逃逸及内部速度
  void reset();

  // 输入失效时立即停车并退出逃逸，保留阻塞/重规划事件供上层处理
  Output halt(State state);

  // 主更新；调用前必须保证激光、里程计和期望速度均在有效期内
  //   v_des    : 路径跟踪器期望速度（机体系），零向量表示停车
  //   position : 当前世界/odom 系位置，用于避障过程中的净位移进展检查
  //   yaw      : 当前世界/odom 系航向(rad)，仅逃逸用
  //   v_actual : 实际前进速度(m/s，来自里程计)
  //   points   : 机体系障碍点（激光转换后）
  //   dt       : 距上次更新的时间(s)
  Output update(const Vec2& v_des, const Vec2& position, double yaw, double v_actual,
                const std::vector<Vec2>& points, double dt);

 private:
  Vec2 computeRepulsive(const std::vector<Vec2>& points, double v_des_x,
                        double* nearest, bool* avoiding);
  void startEscape(const Vec2& position, double yaw, const std::vector<Vec2>& points,
                   bool flip_side);
  void clearTransientMotion();
  void forceToCommand(const Vec2& force, double* v, double* w) const;

  Params p_;
  State state_ = State::NORMAL;

  bool estop_ = false;            // 急停锁存（滞环）
  double stall_timer_ = 0.0;      // 低速停滞累计

  bool progress_active_ = false;  // 避障净位移检查窗口
  Vec2 progress_origin_;
  double progress_timer_ = 0.0;

  bool escaping_ = false;         // 逃逸状态机
  double escape_reference_yaw_ = 0.0; // 同组逃逸尝试共用参考航向
  double escape_target_yaw_ = 0.0;
  Vec2 escape_start_position_;       // 当前这次逃逸尝试的位置起点
  double escape_timer_ = 0.0;
  int escape_side_ = 1;           // +1 左 / -1 右
  int escape_tries_ = 0;

  bool blocked_ = false;          // 永久阻塞锁存
  double blocked_timer_ = 0.0;
  bool replan_ = false;

  double avoidance_distance_ = 0.0; // 避障事件实际行驶里程
  bool episode_active_ = false;

  double normal_timer_ = 0.0;     // 正常行驶恢复计时
  double last_v_ = 0.0;           // 平滑用上一拍输出
  double last_w_ = 0.0;
};

}  // namespace optimized_apf
