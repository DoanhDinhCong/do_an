
#ifndef CLB_NAV2_PID_CONTROLLER__PID_CONTROLLER_HPP_
#define CLB_NAV2_PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace clb_nav2_pid_controller
{

class PidPathController : public nav2_core::Controller
{
public:
  PidPathController() = default;
  ~PidPathController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct PidState
  {
    double i = 0.0;
    double last = 0.0;
    bool has_last = false;
  };

  void resetPid();
  double computePid(double error, double kp, double ki, double kd, PidState & state, double dt) const;
  static double clamp(double v, double mn, double mx);
  static double normalizeAngle(double a);
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q);

  bool isGoalReached(const geometry_msgs::msg::PoseStamped & pose) const;
  size_t findClosestIndex(const geometry_msgs::msg::PoseStamped & pose) const;
  size_t findLookaheadIndex(size_t start_idx, double lookahead) const;
  double targetYaw(size_t idx, const geometry_msgs::msg::PoseStamped & pose) const;
  void publishDebug(
    double kp_x, double ki_x, double kd_x,
    double kp_y, double ki_y, double kd_y,
    double kp_yaw, double ki_yaw, double kd_yaw);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;

  std::string name_;
  std::string base_frame_;

  nav_msgs::msg::Path global_plan_;

  double lookahead_dist_{0.5};
  bool use_holonomic_{true};
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.5};
  double integrator_limit_{0.5};

  double kp_x_{1.0};
  double ki_x_{0.0};
  double kd_x_{0.0};
  double kp_y_{1.0};
  double ki_y_{0.0};
  double kd_y_{0.0};
  double kp_yaw_{2.5};
  double ki_yaw_{0.0};
  double kd_yaw_{0.0};

  double min_vel_x_{-0.24};
  double max_vel_x_{0.24};
  double min_vel_y_{-0.24};
  double max_vel_y_{0.24};
  double min_vel_theta_{-0.50};
  double max_vel_theta_{0.50};
  double min_speed_xy_{0.0};
  double max_speed_xy_{0.24};

  bool use_base_controller_{false};
  bool use_dwb_base_{false};
  double pid_blend_{0.3};
  double pid_blend_xy_{-1.0};
  double pid_blend_yaw_{-1.0};
  std::string base_plugin_name_{"dwb_core::DWBLocalPlanner"};
  std::string base_param_namespace_{"FollowPath"};
  std::shared_ptr<pluginlib::ClassLoader<nav2_core::Controller>> base_loader_;
  std::shared_ptr<nav2_core::Controller> base_controller_;

  bool use_fuzzy_pid_{false};
  double fuzzy_e_max_xy_{0.5};
  double fuzzy_de_max_xy_{0.5};
  double fuzzy_e_max_yaw_{0.7};
  double fuzzy_de_max_yaw_{1.0};
  double fuzzy_kp_scale_xy_{0.5};
  double fuzzy_ki_scale_xy_{0.2};
  double fuzzy_kd_scale_xy_{0.1};
  double fuzzy_kp_scale_yaw_{1.0};
  double fuzzy_ki_scale_yaw_{0.2};
  double fuzzy_kd_scale_yaw_{0.1};
  double fuzzy_7_width_{0.33};
  std::array<double, 7> fuzzy_7_centers_{{-1.0, -0.66, -0.33, 0.0, 0.33, 0.66, 1.0}};
  std::array<double, 9> fuzzy_rules_kp_{{
    0.0, 0.5, 1.0,
    -0.5, 0.0, -0.5,
    1.0, 0.5, 0.0
  }};
  std::array<double, 9> fuzzy_rules_ki_{{
    -0.5, -0.5, 0.0,
    0.0, 0.5, 0.0,
    0.0, -0.5, -0.5
  }};
  std::array<double, 9> fuzzy_rules_kd_{{
    0.5, 0.0, -0.5,
    0.5, 0.0, -0.5,
    0.5, 0.0, -0.5
  }};
  std::array<double, 27> fuzzy_rules_kp_27_{};
  std::array<double, 27> fuzzy_rules_ki_27_{};
  std::array<double, 27> fuzzy_rules_kd_27_{};
  std::array<double, 49> fuzzy_rules_kp_49_{};
  std::array<double, 49> fuzzy_rules_ki_49_{};
  std::array<double, 49> fuzzy_rules_kd_49_{};
  bool use_fuzzy_rules_7_{false};
  bool use_fuzzy_rules_3d_{false};
  double speed_limit_{-1.0};
  bool speed_limit_is_percentage_{false};

  bool publish_debug_{false};
  bool debug_active_{false};
  std::string debug_topic_prefix_{"/pid_debug"};
  using FloatPub = rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;
  FloatPub::SharedPtr pub_kp_x_;
  FloatPub::SharedPtr pub_ki_x_;
  FloatPub::SharedPtr pub_kd_x_;
  FloatPub::SharedPtr pub_kp_y_;
  FloatPub::SharedPtr pub_ki_y_;
  FloatPub::SharedPtr pub_kd_y_;
  FloatPub::SharedPtr pub_kp_yaw_;
  FloatPub::SharedPtr pub_ki_yaw_;
  FloatPub::SharedPtr pub_kd_yaw_;

  PidState pid_x_;
  PidState pid_y_;
  PidState pid_yaw_;
  rclcpp::Time last_time_;
  bool has_time_{false};

  void applyFuzzyGains(
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
    double & kd_out) const;
  void applyFuzzyGains7(
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
    double & kd_out) const;
  void applyFuzzyGains3D(
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
    double & kd_out) const;
  double fuzzyOutput(
    const std::array<double, 9> & rules,
    double e_norm,
    double de_norm) const;
  double fuzzyOutput7(
    const std::array<double, 49> & rules,
    double e_norm,
    double de_norm) const;
  double fuzzyOutput3D(
    const std::array<double, 27> & rules,
    double e_norm,
    double de_norm,
    double i_norm) const;
  static double tri(double x, double a, double b, double c);
  static void fuzzyMembership(double x, double & mu_n, double & mu_z, double & mu_p);
  void fuzzyMembership7(double x, std::array<double, 7> & mu) const;
};

}  

#endif  

