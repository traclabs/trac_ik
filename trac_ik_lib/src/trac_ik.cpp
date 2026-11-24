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


#include <trac_ik/trac_ik.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif

namespace TRAC_IK
{

TRAC_IK::TRAC_IK(rclcpp::Node::SharedPtr nh, const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double eps) :
  logger_(nh->get_logger()),
  initialized_(false),
  eps_(eps)
{
  urdf::Model robot_model;
  std::string xml_string;

  if(!nh->has_parameter(URDF_param))
    xml_string = nh->declare_parameter(URDF_param, std::string(""));
  else
    nh->get_parameter(URDF_param, xml_string);

  if(xml_string.empty())
  {
    RCLCPP_FATAL(logger_, "Could not load the xml from parameter: %s", URDF_param.c_str());
    return;
  }

  if (!robot_model.initString(xml_string))
  {
    RCLCPP_FATAL(logger_, "Unable to initialize urdf::Model from robot description.");
    return;
  }

  RCLCPP_DEBUG(logger_, "Reading joints and links from URDF");

  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    RCLCPP_FATAL(logger_, "Failed to extract kdl tree from xml robot description");

  if (!tree.getChain(base_link, tip_link, chain_))
    RCLCPP_FATAL(logger_, "Couldn't find chain %s to %s", base_link.c_str(), tip_link.c_str());

  std::vector<KDL::Segment> chain_segs = chain_.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  lb_.resize(chain_.getNrOfJoints());
  ub_.resize(chain_.getNrOfJoints());

  uint joint_num = 0;
  for (uint i = 0; i < chain_segs.size(); ++i)
  {
    joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        lb_(joint_num - 1) = lower;
        ub_(joint_num - 1) = upper;
      }
      else
      {
        lb_(joint_num - 1) = std::numeric_limits<float>::lowest();
        ub_(joint_num - 1) = std::numeric_limits<float>::max();
      }
      RCLCPP_DEBUG_STREAM(logger_, "IK Using joint " << joint->name << " " << lb_(joint_num - 1) << " " << ub_(joint_num - 1));
    }
  }

  initialize();
}

TRAC_IK::TRAC_IK(rclcpp::Node::SharedPtr nh, const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double eps):
  TRAC_IK(chain, q_min, q_max, eps, nh->get_logger()) {}

TRAC_IK::TRAC_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double eps, const rclcpp::Logger& logger):
  logger_(logger),
  initialized_(false),
  chain_(chain),
  lb_(q_min),
  ub_(q_max),
  eps_(eps)
{
  initialize();
}

void TRAC_IK::initialize()
{

  assert(chain_.getNrOfJoints() == lb_.data.size());
  assert(chain_.getNrOfJoints() == ub_.data.size());

  jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
  resetSolvers();

  for (uint i = 0; i < chain_.segments.size(); i++)
  {
    std::string type = chain_.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (ub_(types_.size()) >= std::numeric_limits<float>::max() &&
          lb_(types_.size()) <= std::numeric_limits<float>::lowest())
        types_.push_back(KDL::BasicJointType::Continuous);
      else
        types_.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      types_.push_back(KDL::BasicJointType::TransJoint);
  }

  assert(types_.size() == lb_.data.size());

  initialized_ = true;
}

bool TRAC_IK::unique_solution(const KDL::JntArray& sol, const double eps)
{

  for (uint i = 0; i < solutions_.size(); i++)
    if (KDL::Equal(sol, solutions_[i], eps))
      return false;
  return true;

}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    //Find actual angle offset
    double diffangle = fmod(val - max, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    //Find actual angle offset
    double diffangle = fmod(min - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  normalizeAngle(val, target - M_PI, target + M_PI);
}


template<typename T1, typename T2>
bool TRAC_IK::runSolver(T1& solver, T2& other_solver,
                        const KDL::JntArray &q_init,
                        const KDL::Frame &p_in,
                        const SolveType &solve_type,
                        const double max_time)
{
  KDL::JntArray q_out;
  KDL::JntArray seed = q_init;

  while (true)
  {
    auto timediff = system_clock_.now() - start_time_;
    auto time_left = max_time - timediff.seconds();

    if (time_left <= 0)
      break;

    int RC = solver.CartToJnt(seed, p_in, q_out, time_left, bounds_);
    if (RC >= 0)
    {
      switch (solve_type)
      {
      case Manip1:
      case Manip2:
      case Manip3:
        normalize_limits(q_init, q_out);
        break;
      default:
        normalize_seed(q_init, q_out);
        break;
      }
      mtx_.lock();
      if (unique_solution(q_out))
      {
        solutions_.push_back(q_out);
        uint curr_size = solutions_.size();
        errors_.resize(curr_size);
        double err, penalty, manip_value;
        switch (solve_type)
        {
        case Manip1:
          penalty = manipPenalty(q_out);
          manip_value = TRAC_IK::manipValue1(q_out);
          err = penalty * manip_value;
          break;
        case Manip2:
          penalty = manipPenalty(q_out);
          manip_value = TRAC_IK::manipValue2(q_out);
          err = penalty * manip_value;
          break;
        case Manip3:
          penalty = manipPenalty(q_out);
          manip_value = TRAC_IK::manipValue3(q_out);
          err = penalty * manip_value;
          break;
        default:
          err = TRAC_IK::JointErr(q_init, q_out);
          break;
        }
        errors_[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
      mtx_.unlock();
    }

    if (!solutions_.empty() && solve_type == Speed)
      break;

    for (uint j = 0; j < seed.data.size(); j++)
      if (types_[j] == KDL::BasicJointType::Continuous)
        seed(j) = fRand(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = fRand(lb_(j), ub_(j));
  }

  other_solver.abort();

  return true;
}


void TRAC_IK::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution)
{
  // Make sure rotational joint values are within 1 revolution of seed; then
  // ensure joint limits are met.

  for (uint i = 0; i < lb_.data.size(); i++)
  {

    if (types_[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types_[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb_(i), ub_(i));

    solution(i) = val;
  }
}

void TRAC_IK::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution)
{
  // Make sure rotational joint values are within 1 revolution of middle of
  // limits; then ensure joint limits are met.

  for (uint i = 0; i < lb_.data.size(); i++)
  {

    if (types_[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);

    if (types_[i] == KDL::BasicJointType::RotJoint && types_[i] != KDL::BasicJointType::Continuous)
      target = (ub_(i) + lb_(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types_[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb_(i), ub_(i));

    solution(i) = val;
  }

}


double TRAC_IK::manipPenalty(const KDL::JntArray& arr)
{
  double penalty = 1.0;
  for (uint i = 0; i < arr.data.size(); i++)
  {
    if (types_[i] == KDL::BasicJointType::Continuous)
      continue;
    double range = ub_(i) - lb_(i);
    penalty *= ((arr(i) - lb_(i)) * (ub_(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}


double TRAC_IK::manipValue1(const KDL::JntArray& arr)
{
  Eigen::MatrixXd singular_values = computeSingularValues(arr);

  double error = 1.0;
  for (uint i = 0; i < singular_values.rows(); ++i)
    error *= singular_values(i, 0);
  return error;
}

double TRAC_IK::manipValue2(const KDL::JntArray& arr)
{
  Eigen::MatrixXd singular_values = computeSingularValues(arr);

  return singular_values.minCoeff() / singular_values.maxCoeff();
}

double TRAC_IK::manipValue3(const KDL::JntArray& arr)
{
    Eigen::MatrixXd singular_values = computeSingularValues(arr);

    return singular_values.minCoeff();
}

Eigen::MatrixXd TRAC_IK::computeSingularValues(const KDL::JntArray& arr)
{
    KDL::Jacobian jac(arr.data.size());

    jac_solver_->JntToJac(arr, jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    return svdsolver.singularValues();
}


int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const double max_time, const KDL::Twist& bounds, SolveType solve_type)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(logger_, "TRAC-IK was not properly initialized with a valid chain or limits. IK cannot proceed");
    return -1;
  }

  start_time_ = system_clock_.now();

  nl_solver_->reset();
  ik_solver_->reset();

  // No lock as no threading yet
  solutions_.clear();
  errors_.clear();

  bounds_ = bounds;

  task1_ = std::thread(&TRAC_IK::runKDL, this, q_init, p_in, solve_type, max_time);
  task2_ = std::thread(&TRAC_IK::runNLOPT, this, q_init, p_in, solve_type, max_time);

  if (task1_.joinable())
    task1_.join();
  if (task2_.joinable())
    task2_.join();

  // No lock as no threading anymore
  if (solutions_.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (solve_type)
  {
  case Manip1:
  case Manip2:
  case Manip3:
    std::sort(errors_.rbegin(), errors_.rend()); // rbegin/rend to sort by max
    break;
  default:
    std::sort(errors_.begin(), errors_.end());
    break;
  }

  q_out = solutions_[errors_[0].second];

  return solutions_.size();
}


TRAC_IK::~TRAC_IK()
{
  if (initialized_)
  {
    ik_solver_->abort();
    nl_solver_->abort();
  }
  if (task1_.joinable())
    task1_.join();
  if (task2_.joinable())
    task2_.join();
}

} // end of namespace TRAC_IK
