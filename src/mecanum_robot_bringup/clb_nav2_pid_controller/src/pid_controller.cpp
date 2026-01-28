#include "clb_nav2_pid_controller/pid_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/utils.h"

namespace clb_nav2_pid_controller
{

void PidPathController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node_->get_clock();
  base_frame_ = costmap_ros_->getBaseFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_holonomic", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".integrator_limit", rclcpp::ParameterValue(0.5));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kp_x", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".ki_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kd_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kp_y", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".ki_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kd_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kp_yaw", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".ki_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".kd_yaw", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_vel_x", rclcpp::ParameterValue(-0.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_vel_x", rclcpp::ParameterValue(0.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_vel_y", rclcpp::ParameterValue(-0.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_vel_y", rclcpp::ParameterValue(0.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_vel_theta", rclcpp::ParameterValue(-0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_vel_theta", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_speed_xy", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_speed_xy", rclcpp::ParameterValue(0.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_base_controller", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_dwb_base", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_blend", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_blend_xy", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_blend_yaw", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".base_plugin_name", rclcpp::ParameterValue("dwb_core::DWBLocalPlanner"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".base_param_namespace", rclcpp::ParameterValue("FollowPath"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".dwb_plugin_name", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".dwb_param_namespace", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_fuzzy_pid", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_e_max_xy", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_de_max_xy", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_e_max_yaw", rclcpp::ParameterValue(0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_de_max_yaw", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_kp_scale_xy", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_ki_scale_xy", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_kd_scale_xy", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_kp_scale_yaw", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_ki_scale_yaw", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_kd_scale_yaw", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_7_width", rclcpp::ParameterValue(0.33));
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_rules_kp", rclcpp::ParameterValue(std::vector<double>{}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_rules_ki", rclcpp::ParameterValue(std::vector<double>{}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fuzzy_rules_kd", rclcpp::ParameterValue(std::vector<double>{}));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".publish_debug", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".debug_topic_prefix", rclcpp::ParameterValue("/pid_debug"));

  node_->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(name_ + ".use_holonomic", use_holonomic_);
  node_->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node_->get_parameter(name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node_->get_parameter(name_ + ".integrator_limit", integrator_limit_);

  node_->get_parameter(name_ + ".kp_x", kp_x_);
  node_->get_parameter(name_ + ".ki_x", ki_x_);
  node_->get_parameter(name_ + ".kd_x", kd_x_);
  node_->get_parameter(name_ + ".kp_y", kp_y_);
  node_->get_parameter(name_ + ".ki_y", ki_y_);
  node_->get_parameter(name_ + ".kd_y", kd_y_);
  node_->get_parameter(name_ + ".kp_yaw", kp_yaw_);
  node_->get_parameter(name_ + ".ki_yaw", ki_yaw_);
  node_->get_parameter(name_ + ".kd_yaw", kd_yaw_);

  node_->get_parameter(name_ + ".min_vel_x", min_vel_x_);
  node_->get_parameter(name_ + ".max_vel_x", max_vel_x_);
  node_->get_parameter(name_ + ".min_vel_y", min_vel_y_);
  node_->get_parameter(name_ + ".max_vel_y", max_vel_y_);
  node_->get_parameter(name_ + ".min_vel_theta", min_vel_theta_);
  node_->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
  node_->get_parameter(name_ + ".min_speed_xy", min_speed_xy_);
  node_->get_parameter(name_ + ".max_speed_xy", max_speed_xy_);
  node_->get_parameter(name_ + ".use_base_controller", use_base_controller_);
  node_->get_parameter(name_ + ".use_dwb_base", use_dwb_base_);
  node_->get_parameter(name_ + ".pid_blend", pid_blend_);
  node_->get_parameter(name_ + ".pid_blend_xy", pid_blend_xy_);
  node_->get_parameter(name_ + ".pid_blend_yaw", pid_blend_yaw_);
  if (pid_blend_xy_ < 0.0) {
    pid_blend_xy_ = pid_blend_;
  }
  if (pid_blend_yaw_ < 0.0) {
    pid_blend_yaw_ = pid_blend_;
  }
  node_->get_parameter(name_ + ".base_plugin_name", base_plugin_name_);
  node_->get_parameter(name_ + ".base_param_namespace", base_param_namespace_);
  std::string dwb_plugin_name;
  std::string dwb_param_namespace;
  node_->get_parameter(name_ + ".dwb_plugin_name", dwb_plugin_name);
  node_->get_parameter(name_ + ".dwb_param_namespace", dwb_param_namespace);
  if (!dwb_plugin_name.empty()) {
    base_plugin_name_ = dwb_plugin_name;
  }
  if (!dwb_param_namespace.empty()) {
    base_param_namespace_ = dwb_param_namespace;
  }
  node_->get_parameter(name_ + ".use_fuzzy_pid", use_fuzzy_pid_);
  node_->get_parameter(name_ + ".fuzzy_e_max_xy", fuzzy_e_max_xy_);
  node_->get_parameter(name_ + ".fuzzy_de_max_xy", fuzzy_de_max_xy_);
  node_->get_parameter(name_ + ".fuzzy_e_max_yaw", fuzzy_e_max_yaw_);
  node_->get_parameter(name_ + ".fuzzy_de_max_yaw", fuzzy_de_max_yaw_);
  node_->get_parameter(name_ + ".fuzzy_kp_scale_xy", fuzzy_kp_scale_xy_);
  node_->get_parameter(name_ + ".fuzzy_ki_scale_xy", fuzzy_ki_scale_xy_);
  node_->get_parameter(name_ + ".fuzzy_kd_scale_xy", fuzzy_kd_scale_xy_);
  node_->get_parameter(name_ + ".fuzzy_kp_scale_yaw", fuzzy_kp_scale_yaw_);
  node_->get_parameter(name_ + ".fuzzy_ki_scale_yaw", fuzzy_ki_scale_yaw_);
  node_->get_parameter(name_ + ".fuzzy_kd_scale_yaw", fuzzy_kd_scale_yaw_);
    node_->get_parameter(name_ + ".fuzzy_7_width", fuzzy_7_width_);
node_->get_parameter(name_ + ".publish_debug", publish_debug_);
  node_->get_parameter(name_ + ".debug_topic_prefix", debug_topic_prefix_);
  if (!debug_topic_prefix_.empty() && debug_topic_prefix_.back() == '/') {
    debug_topic_prefix_.pop_back();
  }

  auto expand9_to_27 = [](const std::array<double, 9> & src, std::array<double, 27> & dst) {
    for (size_t k = 0; k < 3; ++k) {
      for (size_t i = 0; i < 9; ++i) {
        dst[k * 9 + i] = src[i];
      }
    }
  };

  std::vector<double> rules_kp;
  std::vector<double> rules_ki;
  std::vector<double> rules_kd;
  node_->get_parameter(name_ + ".fuzzy_rules_kp", rules_kp);
  node_->get_parameter(name_ + ".fuzzy_rules_ki", rules_ki);
  node_->get_parameter(name_ + ".fuzzy_rules_kd", rules_kd);
  const bool rules_9 = rules_kp.size() == 9 && rules_ki.size() == 9 && rules_kd.size() == 9;
  const bool rules_27 = rules_kp.size() == 27 && rules_ki.size() == 27 && rules_kd.size() == 27;
  const bool rules_49 = rules_kp.size() == 49 && rules_ki.size() == 49 && rules_kd.size() == 49;
  use_fuzzy_rules_3d_ = false;
  use_fuzzy_rules_7_ = false;
  if (rules_49) {
    std::copy(rules_kp.begin(), rules_kp.end(), fuzzy_rules_kp_49_.begin());
    std::copy(rules_ki.begin(), rules_ki.end(), fuzzy_rules_ki_49_.begin());
    std::copy(rules_kd.begin(), rules_kd.end(), fuzzy_rules_kd_49_.begin());
    use_fuzzy_rules_7_ = true;
  } else if (rules_27) {
    std::copy(rules_kp.begin(), rules_kp.end(), fuzzy_rules_kp_27_.begin());
    std::copy(rules_ki.begin(), rules_ki.end(), fuzzy_rules_ki_27_.begin());
    std::copy(rules_kd.begin(), rules_kd.end(), fuzzy_rules_kd_27_.begin());
    use_fuzzy_rules_3d_ = true;
  } else if (rules_9) {
    std::copy(rules_kp.begin(), rules_kp.end(), fuzzy_rules_kp_.begin());
    std::copy(rules_ki.begin(), rules_ki.end(), fuzzy_rules_ki_.begin());
    std::copy(rules_kd.begin(), rules_kd.end(), fuzzy_rules_kd_.begin());
  } else if (!rules_kp.empty() || !rules_ki.empty() || !rules_kd.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "fuzzy_rules_* must have size 9, 27, or 49 (all three). Using defaults.");
  }
  if (!use_fuzzy_rules_3d_ && !use_fuzzy_rules_7_) {
    expand9_to_27(fuzzy_rules_kp_, fuzzy_rules_kp_27_);
    expand9_to_27(fuzzy_rules_ki_, fuzzy_rules_ki_27_);
    expand9_to_27(fuzzy_rules_kd_, fuzzy_rules_kd_27_);
  }

  if (publish_debug_) {
    const auto qos = rclcpp::QoS(10);
    pub_kp_x_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kp_x", qos);
    pub_ki_x_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/ki_x", qos);
    pub_kd_x_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kd_x", qos);
    pub_kp_y_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kp_y", qos);
    pub_ki_y_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/ki_y", qos);
    pub_kd_y_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kd_y", qos);
    pub_kp_yaw_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kp_yaw", qos);
    pub_ki_yaw_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/ki_yaw", qos);
    pub_kd_yaw_ = node_->create_publisher<std_msgs::msg::Float64>(debug_topic_prefix_ + "/kd_yaw", qos);
  }

  resetPid();

  if (use_dwb_base_ && !use_base_controller_) {
    use_base_controller_ = true;
  }

  if (use_base_controller_) {
    try {
      base_loader_ = std::make_shared<pluginlib::ClassLoader<nav2_core::Controller>>(
        "nav2_core", "nav2_core::Controller");
      base_controller_ = base_loader_->createSharedInstance(base_plugin_name_);
      base_controller_->configure(parent, base_param_namespace_, tf_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(),
        "Failed to create base controller '%s': %s",
        base_plugin_name_.c_str(), e.what());
      use_base_controller_ = false;
      base_controller_.reset();
      base_loader_.reset();
    }
  }
}

void PidPathController::cleanup()
{
  if (base_controller_) {
    base_controller_->cleanup();
  }
  resetPid();
  global_plan_.poses.clear();
}

void PidPathController::activate()
{
  if (base_controller_) {
    base_controller_->activate();
  }
  if (publish_debug_) {
    if (pub_kp_x_) pub_kp_x_->on_activate();
    if (pub_ki_x_) pub_ki_x_->on_activate();
    if (pub_kd_x_) pub_kd_x_->on_activate();
    if (pub_kp_y_) pub_kp_y_->on_activate();
    if (pub_ki_y_) pub_ki_y_->on_activate();
    if (pub_kd_y_) pub_kd_y_->on_activate();
    if (pub_kp_yaw_) pub_kp_yaw_->on_activate();
    if (pub_ki_yaw_) pub_ki_yaw_->on_activate();
    if (pub_kd_yaw_) pub_kd_yaw_->on_activate();
    debug_active_ = true;
  }
  resetPid();
}

void PidPathController::deactivate()
{
  if (base_controller_) {
    base_controller_->deactivate();
  }
  if (publish_debug_) {
    if (pub_kp_x_) pub_kp_x_->on_deactivate();
    if (pub_ki_x_) pub_ki_x_->on_deactivate();
    if (pub_kd_x_) pub_kd_x_->on_deactivate();
    if (pub_kp_y_) pub_kp_y_->on_deactivate();
    if (pub_ki_y_) pub_ki_y_->on_deactivate();
    if (pub_kd_y_) pub_kd_y_->on_deactivate();
    if (pub_kp_yaw_) pub_kp_yaw_->on_deactivate();
    if (pub_ki_yaw_) pub_ki_yaw_->on_deactivate();
    if (pub_kd_yaw_) pub_kd_yaw_->on_deactivate();
    debug_active_ = false;
  }
  resetPid();
}

void PidPathController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (base_controller_) {
    base_controller_->setPlan(path);
  }
  resetPid();
}

void PidPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped PidPathController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Global plan is empty");
  }

  if (isGoalReached(pose)) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = clock_->now();
    stop.header.frame_id = base_frame_;
    resetPid();
    return stop;
  }

  const auto now = clock_->now();
  double dt = 0.0;
  if (has_time_) {
    dt = (now - last_time_).seconds();
  } else {
    has_time_ = true;
  }
  last_time_ = now;
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.01;
  }

  const size_t closest_idx = findClosestIndex(pose);
  const size_t target_idx = findLookaheadIndex(closest_idx, lookahead_dist_);
  const auto & target = global_plan_.poses[target_idx];

  const double yaw = yawFromQuat(pose.pose.orientation);
  const double dx = target.pose.position.x - pose.pose.position.x;
  const double dy = target.pose.position.y - pose.pose.position.y;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double ex = c * dx + s * dy;
  const double ey = -s * dx + c * dy;
  const double eyaw = normalizeAngle(targetYaw(target_idx, pose) - yaw);

  double kp_x = kp_x_;
  double ki_x = ki_x_;
  double kd_x = kd_x_;
  double kp_y = kp_y_;
  double ki_y = ki_y_;
  double kd_y = kd_y_;
  double kp_yaw = kp_yaw_;
  double ki_yaw = ki_yaw_;
  double kd_yaw = kd_yaw_;
  if (use_fuzzy_pid_) {
    if (use_fuzzy_rules_3d_) {
      applyFuzzyGains3D(
        ex, pid_x_, dt, fuzzy_e_max_xy_, fuzzy_de_max_xy_, integrator_limit_,
        kp_x_, ki_x_, kd_x_, fuzzy_kp_scale_xy_, fuzzy_ki_scale_xy_, fuzzy_kd_scale_xy_,
        fuzzy_rules_kp_27_, fuzzy_rules_ki_27_, fuzzy_rules_kd_27_, kp_x, ki_x, kd_x);
      applyFuzzyGains3D(
        ey, pid_y_, dt, fuzzy_e_max_xy_, fuzzy_de_max_xy_, integrator_limit_,
        kp_y_, ki_y_, kd_y_, fuzzy_kp_scale_xy_, fuzzy_ki_scale_xy_, fuzzy_kd_scale_xy_,
        fuzzy_rules_kp_27_, fuzzy_rules_ki_27_, fuzzy_rules_kd_27_, kp_y, ki_y, kd_y);
      applyFuzzyGains3D(
        eyaw, pid_yaw_, dt, fuzzy_e_max_yaw_, fuzzy_de_max_yaw_, integrator_limit_,
        kp_yaw_, ki_yaw_, kd_yaw_, fuzzy_kp_scale_yaw_, fuzzy_ki_scale_yaw_, fuzzy_kd_scale_yaw_,
        fuzzy_rules_kp_27_, fuzzy_rules_ki_27_, fuzzy_rules_kd_27_, kp_yaw, ki_yaw, kd_yaw);
    } else {
      applyFuzzyGains(
        ex, pid_x_, dt, fuzzy_e_max_xy_, fuzzy_de_max_xy_,
        kp_x_, ki_x_, kd_x_, fuzzy_kp_scale_xy_, fuzzy_ki_scale_xy_, fuzzy_kd_scale_xy_,
        fuzzy_rules_kp_, fuzzy_rules_ki_, fuzzy_rules_kd_, kp_x, ki_x, kd_x);
      applyFuzzyGains(
        ey, pid_y_, dt, fuzzy_e_max_xy_, fuzzy_de_max_xy_,
        kp_y_, ki_y_, kd_y_, fuzzy_kp_scale_xy_, fuzzy_ki_scale_xy_, fuzzy_kd_scale_xy_,
        fuzzy_rules_kp_, fuzzy_rules_ki_, fuzzy_rules_kd_, kp_y, ki_y, kd_y);
      applyFuzzyGains(
        eyaw, pid_yaw_, dt, fuzzy_e_max_yaw_, fuzzy_de_max_yaw_,
        kp_yaw_, ki_yaw_, kd_yaw_, fuzzy_kp_scale_yaw_, fuzzy_ki_scale_yaw_, fuzzy_kd_scale_yaw_,
        fuzzy_rules_kp_, fuzzy_rules_ki_, fuzzy_rules_kd_, kp_yaw, ki_yaw, kd_yaw);
    }
  }

  publishDebug(kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_yaw, ki_yaw, kd_yaw);

  const double vx_cmd = computePid(ex, kp_x, ki_x, kd_x, pid_x_, dt);
  double vy_cmd = computePid(ey, kp_y, ki_y, kd_y, pid_y_, dt);
  const double wz_cmd = computePid(eyaw, kp_yaw, ki_yaw, kd_yaw, pid_yaw_, dt);

  geometry_msgs::msg::TwistStamped base_cmd;
  bool have_base = false;
  if (use_base_controller_ && base_controller_) {
    try {
      base_cmd = base_controller_->computeVelocityCommands(pose, velocity, goal_checker);
      have_base = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Base controller failed, fallback to PID: %s", e.what());
    }
  }

  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = wz_cmd;
  if (have_base) {
    const double alpha_xy = clamp(pid_blend_xy_, 0.0, 1.0);
    const double alpha_yaw = clamp(pid_blend_yaw_, 0.0, 1.0);
    vx = (1.0 - alpha_xy) * base_cmd.twist.linear.x + alpha_xy * vx_cmd;
    vy = (1.0 - alpha_xy) * base_cmd.twist.linear.y + alpha_xy * vy_cmd;
    wz = (1.0 - alpha_yaw) * base_cmd.twist.angular.z + alpha_yaw * wz_cmd;
  }

  vx = clamp(vx, min_vel_x_, max_vel_x_);
  wz = clamp(wz, min_vel_theta_, max_vel_theta_);
  if (use_holonomic_) {
    vy = clamp(vy, min_vel_y_, max_vel_y_);
  } else {
    vy = 0.0;
  }

  double max_speed_xy = max_speed_xy_;
  if (speed_limit_ > 0.0) {
    if (speed_limit_is_percentage_) {
      const double ratio = clamp(speed_limit_ / 100.0, 0.0, 1.0);
      max_speed_xy = max_speed_xy_ * ratio;
    } else {
      max_speed_xy = std::min(max_speed_xy_, speed_limit_);
    }
  }

  const double speed_xy = std::hypot(vx, vy);
  if (max_speed_xy > 0.0 && speed_xy > max_speed_xy) {
    const double scale = max_speed_xy / speed_xy;
    vx *= scale;
    vy *= scale;
  }
  if (min_speed_xy_ > 0.0 && speed_xy > 1e-6 && speed_xy < min_speed_xy_) {
    const double scale = min_speed_xy_ / speed_xy;
    vx *= scale;
    vy *= scale;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = base_frame_;
  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.angular.z = wz;
  return cmd;
}

void PidPathController::publishDebug(
  double kp_x, double ki_x, double kd_x,
  double kp_y, double ki_y, double kd_y,
  double kp_yaw, double ki_yaw, double kd_yaw)
{
  if (!publish_debug_ || !debug_active_) {
    return;
  }

  std_msgs::msg::Float64 msg;
  if (pub_kp_x_) { msg.data = kp_x; pub_kp_x_->publish(msg); }
  if (pub_ki_x_) { msg.data = ki_x; pub_ki_x_->publish(msg); }
  if (pub_kd_x_) { msg.data = kd_x; pub_kd_x_->publish(msg); }
  if (pub_kp_y_) { msg.data = kp_y; pub_kp_y_->publish(msg); }
  if (pub_ki_y_) { msg.data = ki_y; pub_ki_y_->publish(msg); }
  if (pub_kd_y_) { msg.data = kd_y; pub_kd_y_->publish(msg); }
  if (pub_kp_yaw_) { msg.data = kp_yaw; pub_kp_yaw_->publish(msg); }
  if (pub_ki_yaw_) { msg.data = ki_yaw; pub_ki_yaw_->publish(msg); }
  if (pub_kd_yaw_) { msg.data = kd_yaw; pub_kd_yaw_->publish(msg); }
}

void PidPathController::resetPid()
{
  pid_x_ = PidState{};
  pid_y_ = PidState{};
  pid_yaw_ = PidState{};
  has_time_ = false;
}

double PidPathController::computePid(
  double error, double kp, double ki, double kd, PidState & state, double dt) const
{
  double output = kp * error;
  if (dt > 0.0) {
    state.i += error * dt;
    if (integrator_limit_ > 0.0) {
      state.i = clamp(state.i, -integrator_limit_, integrator_limit_);
    }
    output += ki * state.i;
    if (state.has_last) {
      const double d = (error - state.last) / dt;
      output += kd * d;
    }
    state.last = error;
    state.has_last = true;
  }
  return output;
}

double PidPathController::clamp(double v, double mn, double mx)
{
  if (mn > mx) {
    std::swap(mn, mx);
  }
  return std::max(mn, std::min(v, mx));
}

double PidPathController::normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double PidPathController::yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

void PidPathController::applyFuzzyGains(
  double error,
  const PidState & state,
  double dt,
  double e_max,
  double de_max,
  double kp_base,
  double ki_base,
  double kd_base,
  double kp_scale,
  double ki_scale,
  double kd_scale,
  const std::array<double, 9> & kp_rules,
  const std::array<double, 9> & ki_rules,
  const std::array<double, 9> & kd_rules,
  double & kp_out,
  double & ki_out,
  double & kd_out) const
{
  if (e_max <= 1e-6 || de_max <= 1e-6 || dt <= 0.0) {
    kp_out = kp_base;
    ki_out = ki_base;
    kd_out = kd_base;
    return;
  }
  const double e_norm = clamp(error / e_max, -1.0, 1.0);
  double de = 0.0;
  if (state.has_last) {
    de = (error - state.last) / dt;
  }
  const double de_norm = clamp(de / de_max, -1.0, 1.0);

  const double d_kp = fuzzyOutput(kp_rules, e_norm, de_norm) * kp_scale;
  const double d_ki = fuzzyOutput(ki_rules, e_norm, de_norm) * ki_scale;
  const double d_kd = fuzzyOutput(kd_rules, e_norm, de_norm) * kd_scale;

  kp_out = std::max(0.0, kp_base + d_kp);
  ki_out = std::max(0.0, ki_base + d_ki);
  kd_out = std::max(0.0, kd_base + d_kd);
}

void PidPathController::applyFuzzyGains7(
  double error,
  const PidState & state,
  double dt,
  double e_max,
  double de_max,
  double kp_base,
  double ki_base,
  double kd_base,
  double kp_scale,
  double ki_scale,
  double kd_scale,
  const std::array<double, 49> & kp_rules,
  const std::array<double, 49> & ki_rules,
  const std::array<double, 49> & kd_rules,
  double & kp_out,
  double & ki_out,
  double & kd_out) const
{
  if (e_max <= 1e-6 || de_max <= 1e-6 || dt <= 0.0) {
    kp_out = kp_base;
    ki_out = ki_base;
    kd_out = kd_base;
    return;
  }
  const double e_norm = clamp(error / e_max, -1.0, 1.0);
  double de = 0.0;
  if (state.has_last) {
    de = (error - state.last) / dt;
  }
  const double de_norm = clamp(de / de_max, -1.0, 1.0);

  const double d_kp = fuzzyOutput7(kp_rules, e_norm, de_norm) * kp_scale;
  const double d_ki = fuzzyOutput7(ki_rules, e_norm, de_norm) * ki_scale;
  const double d_kd = fuzzyOutput7(kd_rules, e_norm, de_norm) * kd_scale;

  kp_out = std::max(0.0, kp_base + d_kp);
  ki_out = std::max(0.0, ki_base + d_ki);
  kd_out = std::max(0.0, kd_base + d_kd);
}

void PidPathController::applyFuzzyGains3D(
  double error,
  const PidState & state,
  double dt,
  double e_max,
  double de_max,
  double i_max,
  double kp_base,
  double ki_base,
  double kd_base,
  double kp_scale,
  double ki_scale,
  double kd_scale,
  const std::array<double, 27> & kp_rules,
  const std::array<double, 27> & ki_rules,
  const std::array<double, 27> & kd_rules,
  double & kp_out,
  double & ki_out,
  double & kd_out) const
{
  if (e_max <= 1e-6 || de_max <= 1e-6 || i_max <= 1e-6 || dt <= 0.0) {
    kp_out = kp_base;
    ki_out = ki_base;
    kd_out = kd_base;
    return;
  }
  const double e_norm = clamp(error / e_max, -1.0, 1.0);
  double de = 0.0;
  if (state.has_last) {
    de = (error - state.last) / dt;
  }
  const double de_norm = clamp(de / de_max, -1.0, 1.0);
  const double i_norm = clamp(state.i / i_max, -1.0, 1.0);

  const double d_kp = fuzzyOutput3D(kp_rules, e_norm, de_norm, i_norm) * kp_scale;
  const double d_ki = fuzzyOutput3D(ki_rules, e_norm, de_norm, i_norm) * ki_scale;
  const double d_kd = fuzzyOutput3D(kd_rules, e_norm, de_norm, i_norm) * kd_scale;

  kp_out = std::max(0.0, kp_base + d_kp);
  ki_out = std::max(0.0, ki_base + d_ki);
  kd_out = std::max(0.0, kd_base + d_kd);
}

double PidPathController::fuzzyOutput(
  const std::array<double, 9> & rules,
  double e_norm,
  double de_norm) const
{
  double e_n = 0.0, e_z = 0.0, e_p = 0.0;
  double de_n = 0.0, de_z = 0.0, de_p = 0.0;
  fuzzyMembership(e_norm, e_n, e_z, e_p);
  fuzzyMembership(de_norm, de_n, de_z, de_p);

  const double mu_e[3] = {e_n, e_z, e_p};
  const double mu_de[3] = {de_n, de_z, de_p};
  double num = 0.0;
  double den = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      const double w = mu_e[i] * mu_de[j];
      num += w * rules[i * 3 + j];
      den += w;
    }
  }
  if (den <= 1e-9) {
    return 0.0;
  }
  return num / den;
}

double PidPathController::fuzzyOutput7(
  const std::array<double, 49> & rules,
  double e_norm,
  double de_norm) const
{
  std::array<double, 7> mu_e{};
  std::array<double, 7> mu_de{};
  fuzzyMembership7(e_norm, mu_e);
  fuzzyMembership7(de_norm, mu_de);

  double num = 0.0;
  double den = 0.0;
  for (size_t i = 0; i < mu_e.size(); ++i) {
    if (mu_e[i] <= 0.0) {
      continue;
    }
    for (size_t j = 0; j < mu_de.size(); ++j) {
      if (mu_de[j] <= 0.0) {
        continue;
      }
      const double w = mu_e[i] * mu_de[j];
      num += w * rules[i * 7 + j];
      den += w;
    }
  }
  if (den <= 1e-9) {
    return 0.0;
  }
  return num / den;
}

double PidPathController::fuzzyOutput3D(
  const std::array<double, 27> & rules,
  double e_norm,
  double de_norm,
  double i_norm) const
{
  double e_n = 0.0, e_z = 0.0, e_p = 0.0;
  double de_n = 0.0, de_z = 0.0, de_p = 0.0;
  double i_n = 0.0, i_z = 0.0, i_p = 0.0;
  fuzzyMembership(e_norm, e_n, e_z, e_p);
  fuzzyMembership(de_norm, de_n, de_z, de_p);
  fuzzyMembership(i_norm, i_n, i_z, i_p);

  const double mu_e[3] = {e_n, e_z, e_p};
  const double mu_de[3] = {de_n, de_z, de_p};
  const double mu_i[3] = {i_n, i_z, i_p};
  double num = 0.0;
  double den = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        const double w = mu_e[i] * mu_de[j] * mu_i[k];
        num += w * rules[i * 9 + j * 3 + k];
        den += w;
      }
    }
  }
  if (den <= 1e-9) {
    return 0.0;
  }
  return num / den;
}

double PidPathController::tri(double x, double a, double b, double c)
{
  if (x <= a || x >= c) {
    return 0.0;
  }
  if (x == b) {
    return 1.0;
  }
  if (x < b) {
    return (x - a) / (b - a);
  }
  return (c - x) / (c - b);
}

void PidPathController::fuzzyMembership7(double x, std::array<double, 7> & mu) const
{
  const double x_clamped = clamp(x, -1.0, 1.0);
  const double w = std::max(fuzzy_7_width_, 1e-6);
  for (size_t i = 0; i < mu.size(); ++i) {
    const double c = fuzzy_7_centers_[i];
    mu[i] = tri(x_clamped, c - w, c, c + w);
  }
}

void PidPathController::fuzzyMembership(double x, double & mu_n, double & mu_z, double & mu_p)
{
  if (x <= -1.0) {
    mu_n = 1.0;
    mu_z = 0.0;
    mu_p = 0.0;
    return;
  }
  if (x >= 1.0) {
    mu_n = 0.0;
    mu_z = 0.0;
    mu_p = 1.0;
    return;
  }
  if (x < 0.0) {
    mu_n = -x;
    mu_p = 0.0;
  } else {
    mu_n = 0.0;
    mu_p = x;
  }
  mu_z = 1.0 - std::abs(x);
}

bool PidPathController::isGoalReached(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (global_plan_.poses.empty()) {
    return true;
  }
  const auto & goal = global_plan_.poses.back();
  const double dx = goal.pose.position.x - pose.pose.position.x;
  const double dy = goal.pose.position.y - pose.pose.position.y;
  const double dist = std::hypot(dx, dy);
  if (dist > xy_goal_tolerance_) {
    return false;
  }
  const double yaw = yawFromQuat(pose.pose.orientation);
  const double goal_yaw = targetYaw(global_plan_.poses.size() - 1, pose);
  const double dyaw = std::fabs(normalizeAngle(goal_yaw - yaw));
  return dyaw <= yaw_goal_tolerance_;
}

size_t PidPathController::findClosestIndex(const geometry_msgs::msg::PoseStamped & pose) const
{
  size_t best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & p = global_plan_.poses[i].pose.position;
    const double dx = p.x - pose.pose.position.x;
    const double dy = p.y - pose.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

size_t PidPathController::findLookaheadIndex(size_t start_idx, double lookahead) const
{
  if (global_plan_.poses.empty()) {
    return 0;
  }
  if (lookahead <= 0.0 || start_idx >= global_plan_.poses.size() - 1) {
    return std::min(start_idx, global_plan_.poses.size() - 1);
  }
  double acc = 0.0;
  size_t idx = start_idx;
  for (; idx + 1 < global_plan_.poses.size(); ++idx) {
    const auto & p0 = global_plan_.poses[idx].pose.position;
    const auto & p1 = global_plan_.poses[idx + 1].pose.position;
    acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (acc >= lookahead) {
      return idx + 1;
    }
  }
  return global_plan_.poses.size() - 1;
}

double PidPathController::targetYaw(size_t idx, const geometry_msgs::msg::PoseStamped & pose) const
{
  if (global_plan_.poses.empty()) {
    return yawFromQuat(pose.pose.orientation);
  }
  const auto & q = global_plan_.poses[idx].pose.orientation;
  const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (q_norm > 1e-6) {
    return yawFromQuat(q);
  }

  if (idx + 1 < global_plan_.poses.size()) {
    const auto & p = global_plan_.poses[idx].pose.position;
    const auto & n = global_plan_.poses[idx + 1].pose.position;
    return std::atan2(n.y - p.y, n.x - p.x);
  }
  if (idx > 0) {
    const auto & p = global_plan_.poses[idx - 1].pose.position;
    const auto & n = global_plan_.poses[idx].pose.position;
    return std::atan2(n.y - p.y, n.x - p.x);
  }
  return yawFromQuat(pose.pose.orientation);
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(clb_nav2_pid_controller::PidPathController, nav2_core::Controller)







