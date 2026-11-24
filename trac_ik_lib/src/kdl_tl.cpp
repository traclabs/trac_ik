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

#include <trac_ik/kdl_tl.hpp>
#include <cfloat>

namespace KDL
{
ChainIkSolverPos_TL::ChainIkSolverPos_TL(const Chain& chain, const JntArray& q_min, const JntArray& q_max, double eps, bool random_restart, bool try_jl_wrap):
  chain_(chain), q_min_(q_min), q_max_(q_max), vik_solver_(chain_), fk_solver_(chain_), delta_q_(chain_.getNrOfJoints()),
  eps_(eps), rr_(random_restart), wrap_(try_jl_wrap)
{

  assert(chain.getNrOfJoints() == q_min.data.size());
  assert(chain.getNrOfJoints() == q_max.data.size());

  reset();

  for (uint i = 0; i < chain.segments.size(); i++)
  {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (q_max(types_.size()) >= std::numeric_limits<float>::max() &&
          q_min(types_.size()) <= std::numeric_limits<float>::lowest())
        types_.push_back(KDL::BasicJointType::Continuous);
      else types_.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      types_.push_back(KDL::BasicJointType::TransJoint);

  }

  assert(types_.size() == q_max.data.size());
}



int ChainIkSolverPos_TL::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const double max_time, const KDL::Twist bounds)
{

  if (aborted_)
    return -3;

  auto start_time = system_clock_.now();
  q_out = q_init;
  bounds_ = bounds;


  double time_left;

  do
  {
    fk_solver_.JntToCart(q_out, f_);
    delta_twist_ = diffRelative(p_in, f_);

    if (std::abs(delta_twist_.vel.x()) <= std::abs(bounds.vel.x()))
      delta_twist_.vel.x(0);

    if (std::abs(delta_twist_.vel.y()) <= std::abs(bounds.vel.y()))
      delta_twist_.vel.y(0);

    if (std::abs(delta_twist_.vel.z()) <= std::abs(bounds.vel.z()))
      delta_twist_.vel.z(0);

    if (std::abs(delta_twist_.rot.x()) <= std::abs(bounds.rot.x()))
      delta_twist_.rot.x(0);

    if (std::abs(delta_twist_.rot.y()) <= std::abs(bounds.rot.y()))
      delta_twist_.rot.y(0);

    if (std::abs(delta_twist_.rot.z()) <= std::abs(bounds.rot.z()))
      delta_twist_.rot.z(0);

    if (Equal(delta_twist_, Twist::Zero(), eps_))
      return 1;

    delta_twist_ = diff(f_, p_in);

    vik_solver_.CartToJnt(q_out, delta_twist_, delta_q_);
    KDL::JntArray q_curr;

    Add(q_out, delta_q_, q_curr);

    for (uint j = 0; j < q_min_.data.size(); j++)
    {
      if (types_[j] == KDL::BasicJointType::Continuous)
        continue;
      if (q_curr(j) < q_min_(j))
      {
        if (!wrap_ || types_[j] == KDL::BasicJointType::TransJoint)
          // KDL's default
          q_curr(j) = q_min_(j);
        else
        {
          // Find actual wrapped angle between limit and joint
          double diffangle = fmod(q_min_(j) - q_curr(j), 2 * M_PI);
          // Subtract that angle from limit and go into the range by a
          // revolution
          double curr_angle = q_min_(j) - diffangle + 2 * M_PI;
          if (curr_angle > q_max_(j))
            q_curr(j) = q_min_(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    for (uint j = 0; j < q_max_.data.size(); j++)
    {
      if (types_[j] == KDL::BasicJointType::Continuous)
        continue;

      if (q_curr(j) > q_max_(j))
      {
        if (!wrap_ || types_[j] == KDL::BasicJointType::TransJoint)
          // KDL's default
          q_curr(j) = q_max_(j);
        else
        {
          // Find actual wrapped angle between limit and joint
          double diffangle = fmod(q_curr(j) - q_max_(j), 2 * M_PI);
          // Add that angle to limit and go into the range by a revolution
          double curr_angle = q_max_(j) + diffangle - 2 * M_PI;
          if (curr_angle < q_min_(j))
            q_curr(j) = q_max_(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    Subtract(q_out, q_curr, q_out);

    if (q_out.data.isZero(FLT_EPSILON))
    {
      if (rr_)
      {
        for (uint j = 0; j < q_out.data.size(); j++)
          if (types_[j] == KDL::BasicJointType::Continuous)
            q_curr(j) = fRand(q_curr(j) - 2 * M_PI, q_curr(j) + 2 * M_PI);
          else
            q_curr(j) = fRand(q_min_(j), q_max_(j));
      }

      // Below would be an optimization to the normal KDL, where when it
      // gets stuck, it returns immediately.  Don't use to compare KDL with
      // random restarts or TRAC-IK to plain KDL.

      // else {
      //   q_out=q_curr;
      //   return -3;
      // }
    }

    q_out = q_curr;

    auto timediff = system_clock_.now() - start_time;
    time_left = max_time - timediff.seconds();
  }
  while (time_left > 0 && !aborted_);

  return -3;
}

}

