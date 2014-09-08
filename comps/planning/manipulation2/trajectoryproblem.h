/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Mike Vande Weghe <vandeweg@cmu.edu>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor Carnegie Mellon University,
       nor the names of their contributors, may be used to endorse or
       promote products derived from this software without specific prior
       written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file trajectoryproblem.h
    \brief Definition of the Trajectory Problem class. Contains functions for adding blends to trajectories. For use with open wam driver (owd).
 */

#ifndef OPENRAVE_TRAJECTORYPROBLEM_H
#define OPENRAVE_TRAJECTORYPROBLEM_H

class TrajectoryProblem : public ProblemInstance
{
public:
    TrajectoryProblem(EnvironmentBasePtr penv);
    virtual ~TrajectoryProblem();
    virtual void Destroy();
    virtual int main(const string& args);
    
private:
    RobotBasePtr robot;
    virtual bool BlendTrajectory(ostream& sout, istream& sinput);
    virtual bool ExecuteBlendedTrajectory(ostream& sout, istream& sinput);  // DEPRECATED
    virtual bool Traj(ostream& sout, istream& sinput);
    virtual bool CalculateTrajectoryBlends(TrajectoryBasePtr ptraj, RobotBasePtr robot, dReal blend_radius, unsigned int attempts, dReal step_size, EnvironmentBasePtr pEnviron, bool bIgnoreCollisions=false);
    bool VerifyTraj(ostream& sout, istream& sinput);///< This function will collision check a trajectory from a file. IMPORTANT: it uses the CheckCollision function with the same parameter used throughout this code
    bool CheckCollision(RobotBasePtr probot, const vector<dReal>& q0, const vector<dReal>& q1, const vector<dReal>& qresolutioninv);
    void OptimizePathRandomized(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv, int nMaxIterations);
    void OptimizePathAll(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv);
    TrajectoryBasePtr ExtractCorners(const TrajectoryBasePtr ptraj, dReal linearity_threshold);
    bool ExecuteDefault;
};

// a vector of joint angles defines a single robot configuration
class JointPos : public std::vector<dReal> {
public:
  JointPos() {}
  explicit JointPos(int s) : std::vector<dReal>(s) {}
  JointPos(const std::vector<dReal> &vd) : std::vector<dReal>(vd) {}
  JointPos(const OpenRAVE::Trajectory::TPOINT &tp) : 
     std::vector<dReal>(tp.q) {}
  dReal length() const;
  void cpy(dReal *out) const;
  void dump() const;

  // add two vectors
  inline JointPos operator+(const JointPos &rhs) const {
    if (size() != rhs.size()) {
      throw "Error: mismatched sizes";
    }
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i]+=rhs[i];
    }
    return jp;
  }

  // subtract two vectors
  inline JointPos operator-(const JointPos &rhs) const {
    if (size() != rhs.size()) {
      throw "Error: mismatched sizes";
    }
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i]-=rhs[i];
    }
    return jp;
  }

  // add two vectors in place
  inline JointPos &operator+=(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw "Error: mismatched sizes";
    }
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) += rhs[i];
    }
    return *this;
  }

  // subtract two vectors in place
  inline JointPos &operator-=(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw "Error: mismatched sizes";
    }
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) -= rhs[i];
    }
    return *this;
  }

  // multiply by a scalar
  inline JointPos operator*(const dReal &rhs) const {
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i] *= rhs;
    }
    return jp;
  }

  // divide by a scalar
  inline JointPos operator/(const dReal &rhs) const {
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i] /= rhs;
    }
    return jp;
  }

  // multiply by a scalar in place
  inline JointPos &operator*=(const dReal &rhs) {
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) *= rhs;
    }
    return *this;
  }

  // divide by a scalar in place
  inline JointPos &operator/=(const dReal &rhs) {
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) /= rhs;
    }
    return *this;
  }

  // dot product
  inline dReal operator*(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw "Error: mismatched sizes";
    }
    dReal result = 0.0;
    for (unsigned int i=0; i<size(); ++i) {
      result += this->operator[](i) * rhs[i];
    }
    return result;
  }
};  
#endif
