#include "optimized_apf/apf_core.h"

#include <algorithm>

namespace optimized_apf {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCommandEps = 1e-6;

inline double wrapAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

inline double slew(double current, double target, double max_delta) {
  return current + clamp(target - current, -max_delta, max_delta);
}

inline bool finite(const Vec2& v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

}  // namespace

void APFCore::clearTransientMotion() {
  escaping_ = false;
  estop_ = false;
  stall_timer_ = 0.0;
  progress_active_ = false;
  progress_timer_ = 0.0;
  escape_timer_ = 0.0;
  blocked_timer_ = 0.0;
  normal_timer_ = 0.0;
  episode_active_ = false;
  last_v_ = 0.0;
  last_w_ = 0.0;
}

void APFCore::reset() {
  blocked_ = false;
  replan_ = false;
  escape_tries_ = 0;
  avoidance_distance_ = 0.0;
  clearTransientMotion();
}

Output APFCore::halt(State state) {
  clearTransientMotion();
  state_ = state;
  Output out;
  out.state = state_;
  out.replan_request = replan_;
  out.avoidance_distance = avoidance_distance_;
  return out;
}

Vec2 APFCore::computeRepulsive(const std::vector<Vec2>& points, double v_des_x,
                               double* nearest, bool* avoiding) {
  Vec2 rep;
  int n = 0;
  double dmin = std::numeric_limits<double>::infinity();

  for (const auto& pt : points) {
    const double eta = pt.norm();
    if (!std::isfinite(eta)) continue;
    if (eta < dmin) dmin = eta;

    if (eta < p_.eta_0 && eta > p_.robot_radius) {
      const Vec2 shifted = pt - pt * (p_.robot_radius / eta);
      rep += shifted * ((1.0 / eta - 1.0 / p_.eta_0) / (eta * eta));
      ++n;
    }
  }

  *nearest = dmin;
  *avoiding = (n > 0);
  if (n > 0) rep = rep / static_cast<double>(n);

  const double scale =
      1.0 + p_.k_v_scale * std::max(0.0, v_des_x) / std::max(1e-3, p_.v_nominal);
  return rep * (p_.k_rep * scale);
}

void APFCore::startEscape(const Vec2& position, double yaw,
                          const std::vector<Vec2>& points, bool flip_side) {
  escape_start_position_ = position;
  if (flip_side) {
    escape_side_ = -escape_side_;
  } else {
    escape_reference_yaw_ = yaw;

    // 比较左右候选走廊内最近障碍的纵向距离，而不是仅比较点云数量。
    double left_clearance = p_.eta_0;
    double right_clearance = p_.eta_0;
    const double corridor_half_width = std::max(p_.robot_radius, p_.d_stop);
    for (const auto& pt : points) {
      const double distance = pt.norm();
      if (!std::isfinite(distance) || distance >= p_.eta_0 ||
          std::fabs(pt.x) > corridor_half_width) {
        continue;
      }
      if (pt.y > 0.0) left_clearance = std::min(left_clearance, pt.y);
      if (pt.y < 0.0) right_clearance = std::min(right_clearance, -pt.y);
    }
    escape_side_ = (left_clearance >= right_clearance) ? 1 : -1;
  }

  // 同一组逃逸尝试始终相对初始参考航向换侧，避免失败后退回原航向。
  escape_target_yaw_ = wrapAngle(escape_reference_yaw_ + escape_side_ * kPi / 2.0);
  escape_timer_ = 0.0;
  stall_timer_ = 0.0;
  progress_active_ = false;
  progress_timer_ = 0.0;
  escaping_ = true;
}

void APFCore::forceToCommand(const Vec2& force, double* v, double* w) const {
  *v = 0.0;
  *w = 0.0;
  if (!finite(force)) return;

  const double mag = force.norm();
  if (mag <= 1e-3) return;

  double err = wrapAngle(std::atan2(force.y, force.x));
  if (std::fabs(err) < p_.yaw_deadband) err = 0.0;
  *v = std::min(mag, p_.v_max) * std::max(0.0, std::cos(err));
  *w = clamp(p_.k_yaw * err, -p_.w_max, p_.w_max);
}

Output APFCore::update(const Vec2& v_des, const Vec2& position, double yaw, double v_actual,
                       const std::vector<Vec2>& points, double dt) {
  if (!finite(v_des) || !std::isfinite(v_des.norm())) return halt(State::CMD_TIMEOUT);
  if (!finite(position) || !std::isfinite(position.norm()) || !std::isfinite(yaw) ||
      !std::isfinite(v_actual)) {
    return halt(State::ODOM_TIMEOUT);
  }
  if (!std::isfinite(dt) || dt <= 0.0) dt = 1e-3;

  Output out;
  // 状态计时和里程积分使用真实周期；仅斜率限制使用控制周期上限，避免卡顿后速度突跳。
  const double control_dt = clamp(dt, 1e-3, 0.5);

  double nearest = std::numeric_limits<double>::infinity();
  bool avoiding = false;
  const Vec2 rep = computeRepulsive(points, v_des.x, &nearest, &avoiding);
  const bool command_active = v_des.norm() > kCommandEps;
  out.nearest_obstacle = nearest;
  out.repulsive = rep;
  out.avoiding = avoiding;

  // 急停安全层优先级最高，零期望速度时仍保留障碍状态监测。
  if (!estop_ && nearest < p_.d_stop) {
    estop_ = true;
  } else if (estop_ && nearest > p_.d_stop + p_.estop_hysteresis) {
    estop_ = false;
  }

  if (!estop_) {
    if (!command_active) {
      // 零期望速度是明确停车命令，不能由 -rep 产生自主运动，也不能继续逃逸。
      escaping_ = false;
      escape_timer_ = 0.0;
      stall_timer_ = 0.0;
      progress_active_ = false;
      progress_timer_ = 0.0;
      blocked_timer_ = 0.0;
    } else {
      bool stalled = false;
      if (avoiding && !escaping_) {
        if (std::fabs(v_actual) < p_.stall_speed) {
          stall_timer_ += dt;
        } else {
          stall_timer_ = 0.0;
        }

        // 即使轮速不为零，长时间净位移不足也视为无进展，覆盖爬行、振荡和绕圈。
        if (!progress_active_) {
          progress_active_ = true;
          progress_origin_ = position;
          progress_timer_ = 0.0;
        } else {
          progress_timer_ += dt;
          if (progress_timer_ >= p_.progress_check_time) {
            if ((position - progress_origin_).norm() < p_.progress_min_distance) {
              stalled = true;
              stall_timer_ = p_.stall_time;
            } else {
              progress_origin_ = position;
              progress_timer_ = 0.0;
            }
          }
        }
        stalled = stalled || stall_timer_ >= p_.stall_time;
      } else {
        stall_timer_ = 0.0;
        progress_active_ = false;
        progress_timer_ = 0.0;
      }

      if (!escaping_ && !blocked_ && stalled) {
        startEscape(position, yaw, points, false);
      }

      if (escaping_) {
        escape_timer_ += dt;
        const Vec2 displacement = position - escape_start_position_;
        const double escape_progress =
            displacement.x * std::cos(escape_target_yaw_) +
            displacement.y * std::sin(escape_target_yaw_);
        // 沿墙逃逸时仍可能有其他方向障碍，因此只要求逃逸方向产生实际净位移。
        const bool cleared = escape_progress >= p_.escape_min_distance &&
                             std::fabs(v_actual) > p_.stall_speed;
        if (cleared) {
          escaping_ = false;
          escape_timer_ = 0.0;
          stall_timer_ = 0.0;
        } else if (escape_timer_ >= p_.escape_time) {
          ++escape_tries_;
          if (escape_tries_ >= p_.escape_max_tries) {
            escaping_ = false;
            blocked_ = true;
            replan_ = true;
          } else {
            startEscape(position, yaw, points, true);
          }
        }
      }

      if (escaping_ || (avoiding && stalled)) {
        blocked_timer_ += dt;
      } else if (!blocked_) {
        blocked_timer_ = 0.0;
      }
      if (!blocked_ && blocked_timer_ >= p_.block_time) {
        blocked_ = true;
        replan_ = true;
        escaping_ = false;
      }
    }
    // BLOCKED 只能由上层完成重规划后的 reset() 解除，避免未重规划就恢复行驶。
  }

  double v_cmd = 0.0;
  double w_cmd = 0.0;
  if (estop_) {
    state_ = State::ESTOP;
  } else if (blocked_) {
    state_ = State::BLOCKED;
  } else if (!command_active) {
    state_ = State::IDLE;
  } else if (escaping_) {
    state_ = State::ESCAPING;
    // 逃逸子目标仍提供方向，但实时叠加斥力，且速度始终受 v_max 限制。
    const double err = wrapAngle(escape_target_yaw_ - yaw);
    const double escape_speed = std::min(p_.escape_speed, p_.v_max);
    const Vec2 escape_des(std::cos(err) * escape_speed, std::sin(err) * escape_speed);
    forceToCommand(escape_des - rep, &v_cmd, &w_cmd);
  } else {
    state_ = avoiding ? State::AVOIDING : State::NORMAL;
    forceToCommand(v_des - rep, &v_cmd, &w_cmd);
  }

  if (state_ == State::ESTOP || state_ == State::BLOCKED) {
    last_v_ = 0.0;
    last_w_ = 0.0;
  } else {
    last_v_ = slew(last_v_, v_cmd, p_.a_max * control_dt);
    last_w_ = slew(last_w_, w_cmd, p_.alpha_max * control_dt);
  }
  if (!std::isfinite(last_v_) || !std::isfinite(last_w_)) {
    last_v_ = 0.0;
    last_w_ = 0.0;
    state_ = State::ESTOP;
  }
  out.v = last_v_;
  out.w = last_w_;

  // 兼容旧 deviation 话题的实际含义：避障事件内行驶里程，不是横向路径偏差。
  const bool in_episode = command_active && (avoiding || escaping_);
  if (in_episode && !episode_active_) {
    avoidance_distance_ = 0.0;
    episode_active_ = true;
  }
  if (in_episode) {
    avoidance_distance_ += std::fabs(v_actual) * dt;
  } else {
    episode_active_ = false;
  }
  out.avoidance_distance = avoidance_distance_;

  if (state_ == State::NORMAL) {
    normal_timer_ += dt;
    if (normal_timer_ >= p_.normal_reset_time) {
      normal_timer_ = p_.normal_reset_time;
      escape_tries_ = 0;
      blocked_timer_ = 0.0;
      replan_ = false;
    }
  } else {
    normal_timer_ = 0.0;
  }

  out.state = state_;
  out.replan_request = replan_;
  return out;
}

}  // namespace optimized_apf
