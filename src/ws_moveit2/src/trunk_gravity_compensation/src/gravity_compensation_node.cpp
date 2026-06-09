#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "trunk_gravity_compensation/gravity_model.hpp"

namespace
{

std::string joinPath(const std::string& lhs, const std::string& rhs)
{
  if (lhs.empty()) {
    return rhs;
  }
  if (rhs.empty()) {
    return lhs;
  }
  if (lhs.back() == '/') {
    return lhs + rhs;
  }
  return lhs + "/" + rhs;
}

std::vector<double> readVectorParameter(
  rclcpp::Node& node,
  const std::string& name,
  const std::vector<double>& default_value,
  std::size_t expected_size)
{
  auto value = node.declare_parameter<std::vector<double>>(name, default_value);
  if (value.size() == expected_size) {
    return value;
  }

  RCLCPP_WARN(
    node.get_logger(),
    "Parameter '%s' expected %zu values but got %zu; using default.",
    name.c_str(),
    expected_size,
    value.size());
  return default_value;
}

}  // namespace

namespace trunk_gravity_compensation
{

class GravityCompensationNode : public rclcpp::Node
{
public:
  GravityCompensationNode()
  : Node("gravity_compensation_node")
  {
    robot_description_package_ =
      declare_parameter<std::string>("robot_description_package", "robot_model");
    urdf_relative_path_ = declare_parameter<std::string>("urdf_relative_path", "urdf/trunk_robot.urdf");
    urdf_path_ = declare_parameter<std::string>("urdf_path", "");
    base_link_ = declare_parameter<std::string>("base_link", "trunk_base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "trunk_link4");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"trunk_joint1", "trunk_joint2", "trunk_joint3", "trunk_joint4"});
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "joint_states");
    torque_topic_ = declare_parameter<std::string>(
      "torque_topic",
      "gravity_compensation/torque");
    publish_rate_hz_ = declare_parameter<double>("publish_rate", 100.0);
    warn_if_torque_exceeds_limit_ = declare_parameter<bool>("warn_if_torque_exceeds_limit", true);
    torque_limits_nm_ = readVectorParameter(
      *this,
      "torque_limits_nm",
      {300.0, 300.0, 300.0, 300.0},
      joint_names_.size());
    const auto gravity_xyz = readVectorParameter(*this, "gravity_xyz", {0.0, 0.0, -9.81}, 3);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate must be positive; using 100 Hz.");
      publish_rate_hz_ = 100.0;
    }

    if (urdf_path_.empty()) {
      urdf_path_ = joinPath(
        ament_index_cpp::get_package_share_directory(robot_description_package_),
        urdf_relative_path_);
    }

    std::string error_message;
    if (!gravity_model_.initialize(
        urdf_path_,
        base_link_,
        tip_link_,
        joint_names_,
        KDL::Vector(gravity_xyz[0], gravity_xyz[1], gravity_xyz[2]),
        error_message))
    {
      throw std::runtime_error(error_message);
    }

    latest_positions_.assign(joint_names_.size(), 0.0);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GravityCompensationNode::jointStateCallback, this, std::placeholders::_1));
    torque_pub_ = create_publisher<sensor_msgs::msg::JointState>(torque_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GravityCompensationNode::publishGravityTorque, this));

    RCLCPP_INFO(
      get_logger(),
      "Gravity compensation observer ready. urdf='%s' chain='%s -> %s' joint_states='%s' torque_topic='%s'",
      urdf_path_.c_str(),
      base_link_.c_str(),
      tip_link_.c_str(),
      joint_states_topic_.c_str(),
      torque_topic_.c_str());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unordered_map<std::string, std::size_t> received_index;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      received_index.emplace(msg->name[i], i);
    }

    std::vector<double> positions(joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = received_index.find(joint_names_[i]);
      if (it == received_index.end() || it->second >= msg->position.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for joint_states position for '%s'.",
          joint_names_[i].c_str());
        has_valid_state_ = false;
        return;
      }
      positions[i] = msg->position[it->second];
    }

    latest_positions_ = std::move(positions);
    latest_stamp_ = msg->header.stamp;
    has_valid_state_ = true;
  }

  void publishGravityTorque()
  {
    if (!has_valid_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No complete joint state received yet; gravity torque is not published.");
      return;
    }

    std::vector<double> torques;
    std::string error_message;
    if (!gravity_model_.computeGravityTorque(latest_positions_, torques, error_message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Gravity torque computation failed: %s",
        error_message.c_str());
      return;
    }

    warnIfTorqueExceedsLimit(torques);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = latest_stamp_.nanoseconds() > 0 ? latest_stamp_ : now();
    msg.name = gravity_model_.jointNames();
    msg.position = latest_positions_;
    msg.effort = torques;
    torque_pub_->publish(msg);
  }

  void warnIfTorqueExceedsLimit(const std::vector<double>& torques)
  {
    if (!warn_if_torque_exceeds_limit_) {
      return;
    }

    for (std::size_t i = 0; i < torques.size() && i < torque_limits_nm_.size(); ++i) {
      if (std::abs(torques[i]) > torque_limits_nm_[i]) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Gravity torque %.3f Nm for %s exceeds configured observer limit %.3f Nm.",
          torques[i],
          joint_names_[i].c_str(),
          torque_limits_nm_[i]);
      }
    }
  }

  std::string robot_description_package_;
  std::string urdf_relative_path_;
  std::string urdf_path_;
  std::string base_link_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;
  std::string joint_states_topic_;
  std::string torque_topic_;
  double publish_rate_hz_{100.0};
  bool warn_if_torque_exceeds_limit_{true};
  std::vector<double> torque_limits_nm_;

  GravityModel gravity_model_;
  std::vector<double> latest_positions_;
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};
  bool has_valid_state_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace trunk_gravity_compensation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trunk_gravity_compensation::GravityCompensationNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gravity_compensation_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
