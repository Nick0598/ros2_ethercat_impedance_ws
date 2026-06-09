#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

namespace trunk_gravity_compensation
{

class GravityModel
{
public:
  bool initialize(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link,
    const std::vector<std::string>& expected_joint_names,
    const KDL::Vector& gravity,
    std::string& error_message);

  bool computeGravityTorque(
    const std::vector<double>& positions,
    std::vector<double>& torques,
    std::string& error_message) const;

  const std::vector<std::string>& jointNames() const;
  std::size_t jointCount() const;

private:
  std::vector<std::string> extractJointNames() const;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_solver_;
};

}  // namespace trunk_gravity_compensation
