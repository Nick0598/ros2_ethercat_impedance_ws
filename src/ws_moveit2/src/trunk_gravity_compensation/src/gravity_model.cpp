#include "trunk_gravity_compensation/gravity_model.hpp"

#include <sstream>

#include <kdl_parser/kdl_parser.hpp>

namespace trunk_gravity_compensation
{

bool GravityModel::initialize(
  const std::string& urdf_path,
  const std::string& base_link,
  const std::string& tip_link,
  const std::vector<std::string>& expected_joint_names,
  const KDL::Vector& gravity,
  std::string& error_message)
{
  tree_ = KDL::Tree();
  chain_ = KDL::Chain();
  joint_names_.clear();
  dynamics_solver_.reset();

  if (!kdl_parser::treeFromFile(urdf_path, tree_)) {
    error_message = "Failed to parse URDF file: " + urdf_path;
    return false;
  }

  if (!tree_.getChain(base_link, tip_link, chain_)) {
    error_message = "Failed to create KDL chain from '" + base_link + "' to '" + tip_link + "'.";
    return false;
  }

  joint_names_ = extractJointNames();
  if (joint_names_.empty()) {
    error_message = "KDL chain contains no movable joints.";
    return false;
  }

  if (!expected_joint_names.empty() && expected_joint_names != joint_names_) {
    std::ostringstream oss;
    oss << "Joint order mismatch. Expected [";
    for (std::size_t i = 0; i < expected_joint_names.size(); ++i) {
      oss << expected_joint_names[i] << (i + 1 < expected_joint_names.size() ? ", " : "");
    }
    oss << "], KDL chain has [";
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      oss << joint_names_[i] << (i + 1 < joint_names_.size() ? ", " : "");
    }
    oss << "].";
    error_message = oss.str();
    return false;
  }

  dynamics_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);
  return true;
}

bool GravityModel::computeGravityTorque(
  const std::vector<double>& positions,
  std::vector<double>& torques,
  std::string& error_message) const
{
  if (!dynamics_solver_) {
    error_message = "Gravity model has not been initialized.";
    return false;
  }
  if (positions.size() != joint_names_.size()) {
    error_message = "Position vector size does not match KDL joint count.";
    return false;
  }

  KDL::JntArray q(joint_names_.size());
  KDL::JntArray gravity_torque(joint_names_.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    q(static_cast<unsigned int>(i)) = positions[i];
  }

  const int result = dynamics_solver_->JntToGravity(q, gravity_torque);
  if (result < 0) {
    error_message = "KDL JntToGravity failed with error code " + std::to_string(result) + ".";
    return false;
  }

  torques.resize(joint_names_.size());
  for (std::size_t i = 0; i < torques.size(); ++i) {
    torques[i] = gravity_torque(static_cast<unsigned int>(i));
  }
  return true;
}

const std::vector<std::string>& GravityModel::jointNames() const
{
  return joint_names_;
}

std::size_t GravityModel::jointCount() const
{
  return joint_names_.size();
}

std::vector<std::string> GravityModel::extractJointNames() const
{
  std::vector<std::string> names;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      names.push_back(joint.getName());
    }
  }
  return names;
}

}  // namespace trunk_gravity_compensation
