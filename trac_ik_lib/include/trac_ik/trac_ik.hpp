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


#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <trac_ik/nlopt_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <memory>

namespace TRAC_IK
{

enum SolveType { Speed, Distance, Manip1, Manip2, Manip3 };

class TRAC_IK
{
public:
  TRAC_IK(rclcpp::Node::SharedPtr nh, const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double eps = 1e-5);
  TRAC_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double eps = 1e-5, const rclcpp::Logger& logger = rclcpp::get_logger("trac_ik.trac_ik_lib"));

  TRAC_IK(rclcpp::Node::SharedPtr nh, const std::string& base_link, const std::string& tip_link, const std::string& URDF_param = "robot_description", double eps = 1e-5);

  ~TRAC_IK();

  bool getKDLChain(KDL::Chain& chain)
  {
    chain = chain_;
    return initialized_;
  }

  bool getKDLLimits(KDL::JntArray& lb, KDL::JntArray& ub)
  {
    lb = lb_;
    ub = ub_;
    return initialized_;
  }

  // Requires a previous call to CartToJnt()
  bool getSolutions(std::vector<KDL::JntArray>& solutions)
  {
    solutions = solutions_;
    return initialized_ && !solutions.empty();
  }

  bool getSolutions(std::vector<KDL::JntArray>& solutions_, std::vector<std::pair<double, uint> >& errors)
  {
    errors = errors_;
    return getSolutions(solutions_);
  }

  bool setKDLLimits(KDL::JntArray& lb, KDL::JntArray& ub)
  {
    lb = lb_;
    ub = ub_;
    resetSolvers();
    return true;
  }

  static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2)
  {
    double err = 0;
    for (uint i = 0; i < arr1.data.size(); i++)
    {
      err += pow(arr1(i) - arr2(i), 2);
    }

    return err;
  }

  int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const double max_time = 0.005, const KDL::Twist& bounds = KDL::Twist::Zero(), SolveType _type = Speed);

private:
  rclcpp::Logger logger_;
  bool initialized_;
  KDL::Chain chain_;
  KDL::JntArray lb_, ub_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  double eps_;

  std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> ik_solver_;

  rclcpp::Clock system_clock_;
  rclcpp::Time start_time_;

  template<typename T1, typename T2>
  bool runSolver(T1& solver, T2& other_solver,
                 const KDL::JntArray &q_init,
                 const KDL::Frame &p_in,
                 const SolveType &solve_type,
                 const double max_time);

  inline bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in, SolveType solve_type, const double max_time)
  {
      return runSolver(*ik_solver_.get(), *nl_solver_.get(), q_init, p_in, solve_type, max_time);
  }

  inline bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in, SolveType solve_type, const double max_time)
  {
      return runSolver(*nl_solver_.get(), *ik_solver_.get(), q_init, p_in, solve_type, max_time);
  }

  void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
  void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

  std::vector<KDL::BasicJointType> types_;

  std::mutex mtx_;
  std::vector<KDL::JntArray> solutions_;
  std::vector<std::pair<double, uint> > errors_;

  std::thread task1_, task2_;
  KDL::Twist bounds_;

  bool unique_solution(const KDL::JntArray& sol, const double eps = 1e-4);

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

  /* @brief Manipulation metrics and penalties taken from "Workspace
  Geometric Characterization and Manipulability of Industrial Robots",
  Ming-June, Tsia, PhD Thesis, Ohio State University, 1986.
  https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
  */
  double manipPenalty(const KDL::JntArray& arr);
  double manipValue1(const KDL::JntArray& arr);
  double manipValue2(const KDL::JntArray& arr);
  double manipValue3(const KDL::JntArray& arr);

  Eigen::MatrixXd computeSingularValues(const KDL::JntArray& arr);

  void initialize();

  void resetSolvers()
  {
    nl_solver_.reset(new NLOPT_IK::NLOPT_IK(chain_, lb_, ub_, eps_, NLOPT_IK::SumSq, logger_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_TL(chain_, lb_, ub_, eps_, true, true));
  }

};

}

#endif
