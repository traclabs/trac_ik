/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef TRAC_IK_KINEMATICS_PLUGIN_
#define TRAC_IK_KINEMATICS_PLUGIN_

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

// Forward declaration
namespace TRAC_IK {
  class TRAC_IK;
}

namespace trac_ik_kinematics {
  class ParamListener;
  class Params;
}

namespace trac_ik_kinematics_plugin
{

class TRAC_IKKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }


  /** @class
   *  @brief Interface for an TRAC-IK kinematics plugin
   */
  TRAC_IKKinematicsPlugin(): joint_names_(), link_names_(), num_joints_(0), chain_(), position_ik_(false), ik_solver_(nullptr), param_listener_(nullptr), params_(nullptr)
  {
  }

  ~TRAC_IKKinematicsPlugin() = default;

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   *
   * In contrast to the searchPositionIK methods, this one is expected to return the solution
   * closest to the seed state. Randomly re-seeding is explicitly not allowed.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options. See definition of KinematicsQueryOptions for details.
   * @return True if a valid solution was found, false otherwise
   */
  // Returns the first IK solution that is within joint limits, this is called by get_ik() service
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solution by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options. See definition of KinematicsQueryOptions for details.
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solution by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options. See definition of KinematicsQueryOptions for details.
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solution by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback to validate an IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options. See definition of KinematicsQueryOptions for details.
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solution by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param solution_callback A callback to validate an IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options. See definition of KinematicsQueryOptions for details.
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options,
                        const std::unique_ptr<std::string>& solve_type_override = nullptr) const;


  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  /**
   * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
   * @param robot_model - allow the URDF to be loaded much quicker by passing in a pre-parsed model of the robot
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frames The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   * @return true if initialization was successful, false otherwise
   *
   * Default implementation returns false and issues a warning to implement this new API.
   * TODO: Make this method purely virtual after some soaking time, replacing the fallback.
   */
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModel& robot_model,
                  const std::string& group_name,
                  const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

private:

  int getKDLSegmentIndex(const std::string& name) const;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  uint num_joints_;

  KDL::Chain chain_;
  bool position_ik_;

  KDL::JntArray joint_min_, joint_max_;

  std::string solve_type_;

  std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_; // Pointer is also used to indicate whether the plugin is active

  std::shared_ptr<trac_ik_kinematics::ParamListener> param_listener_;
  std::shared_ptr<trac_ik_kinematics::Params> params_;

  std::shared_ptr<random_numbers::RandomNumberGenerator> rng_;

}; // end class

}

#endif
