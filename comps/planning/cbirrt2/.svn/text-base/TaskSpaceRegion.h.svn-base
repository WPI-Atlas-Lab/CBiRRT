/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Dmitry Berenson <dberenso@cs.cmu.edu>

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
/** \file TaskSpaceRegion.h
    \brief Defines the Task Space Region and Task Space Region Chain class.es
 */
#ifndef  TSR_H
#define  TSR_H

/// Class defining a TSR: a simple representation of pose constraints
class TaskSpaceRegion
{

public:
    int manipind; ///< this specifies the index of the manipulator of the robot that is associated with this TSR
    std::string relativebodyname; ///< name of the body T0_w is attached to (NULL = world frame)
    std::string relativelinkname; ///< name of the link T0_w is attached to (NULL = world frame)
    KinBody::LinkPtr prelativetolink; ///< pointer to the link T0_w is attached to (NULL = world frame), this can be the link of a robot or of something else
    Transform T0_w; ///< the center of the TSR relative to the link it is attached to (or relative to world frame)
    Transform Tw_e; ///< the end-effector offset of this TSR
    dReal Bw[6][2]; ///< matrix defining maximum and minimum allowable deviation from T0_w in x,y,z,roll,pitch,and yaw

    TaskSpaceRegion();
    bool Initialize(EnvironmentBasePtr penv_in); ///< initialize the TSR
    void Print(); ///< print the TSR
    int GetManipInd(){return manipind;} ///< return the manipulator id of this TSR
    Transform GenerateSample(); ///< generate a sample from this TSR
    dReal GetVolume(){if(_volume < 0) RAVELOG_INFO("ERROR TSR not initialized\n"); else return _volume;} ///< return the 6D volume of this TSR
    dReal GetSumOfBounds(){if(_sumbounds < 0) RAVELOG_INFO("ERROR TSR not initialized\n"); else return _sumbounds;} ///< return the sum of the length of the bounds
    Transform GetClosestTransform(const Transform& T0_s); ///< get the closest transform in the TSR to a query transform
    dReal DistanceToTSR(const Transform& T0_s, std::vector<dReal>& dx); ///< get the distance to the TSR from a query transform
    void RPYToQuat(dReal* rpy, dReal* quat); ///< covert roll-pitch-yaw to a quaternion
    void QuatToRPY(dReal* quat, dReal& psi, dReal& theta, dReal& phi); ///< convert a quaternion to roll-pitch-yaw
    
    bool serialize(std::ostream& O) const; ///< write the TSR to a string
    bool deserialize(std::stringstream& _ss); ///< parse a string to set the values of the TSR
    bool deserialize_from_matlab(RobotBasePtr  robot, EnvironmentBasePtr penv_in, std::istream& _ss);  ///< parse a string from matlab to set the values of the TSR

    static Vector RPYIdentityOffsets[8]; ///< list of RPY identities
private:
    dReal _volume;
    dReal _sumbounds;
    dReal _dimensionality;
    //these are temporary variables used in TSR computations
    Transform Tw_s1,T0_link,Tw_rand;
    dReal dw_sample[6];
    dReal frand;
    dReal sumsqr;
    dReal a,b,c,d;
    dReal _cphi,_sphi,_ctheta,_stheta,_cpsi,_spsi;
    dReal _min_dist;
    Vector _temp_vec;
    dReal _temp_dist;
};

/// Class defining a TSR Chain: a more complex representation of pose constraints
class TaskSpaceRegionChain
{
public:
    std::vector<TaskSpaceRegion> TSRChain; ///< this is an ordered list of TSRs, where each one relies on the previous one to determine T0_w, note that the T0_w values of the TSRs in the chain will change (except the first one)
    bool Initialize(EnvironmentBasePtr penv_in); ///< initialize the TSR chain


    TaskSpaceRegionChain(){numdof = -1;_dx.resize(6);_sumbounds = -1;}
    ~TaskSpaceRegionChain(){DestoryRobotizedTSRChain();}
    int GetManipInd(){if(TSRChain.size() > 0) return TSRChain[0].manipind; return -1;} ///< return the manipulator index of the first TSR
    void AddTSR(TaskSpaceRegion& TSR){TSRChain.push_back(TSR);} ///< add a TSR to the chain

   
    Transform GenerateSample(); ///< generate a sample from this TSR Chain
    dReal GetClosestTransform(const Transform& T0_s, dReal * TSRJointVals, Transform& T0_closeset);  ///< get the closest transform in the TSR Chain to a query transform
    bool RobotizeTSRChain(EnvironmentBasePtr penv_in, RobotBasePtr& probot_out); ///< create a virtual manipulator (a robot) corresponding to the TSR chain to use for ik solver calls
    bool ApplyMimicValuesToMimicBody(const dReal* TSRJointVals); ///< apply mimiced joint values to a certain set of joints
    bool MimicValuesToFullMimicBodyValues(const dReal* TSRJointVals, std::vector<dReal>& mimicbodyvals); ///< turn the list of mimic joint values into a list of full joint values
    bool GetChainJointLimits(dReal* lowerlimits, dReal* upperlimits); ///< get the joint limits of the virtual manipulator
    dReal TransformDifference(const Transform& tm_ref, const Transform& tm_targ); ///< compute the distance between two transforms
    dReal GetSumOfBounds(){if(_sumbounds < 0) RAVELOG_INFO("ERROR TSR not initialized\n"); else return _sumbounds;} ///< return the sum of the length of the bounds, summed over all TSRs in the chain

    bool ExtractMimicDOFValues(dReal* TSRValues, dReal * MimicDOFVals); ///< get the values of the mimiced DOFs
    int GetNumMimicDOF(){return _mimicinds.size();} ///< get the number of mimiced DOFs
    std::vector<int> GetMimicDOFInds(){return _mimicinds;} ///< get the mimiced DOFs
    int GetNumDOF(){if(numdof == -1) RAVELOG_INFO("ERROR : this chain has not been robotized yet\n"); return numdof;} ///< get the number of DOFs of the virtual manipulator
    RobotBasePtr  GetMimicBody(){return _pmimicbody;} ///< get a pointer to the mimiced body

    bool IsForGoalSampling(){return bSampleGoalFromChain;} ///< is this TSR chain used for sampling goals?
    bool IsForStartSampling(){return bSampleStartFromChain;} ///< is this TSR chain used for sampling starts?
    bool IsForConstraint(){return bConstrainToChain;} ///< is this TSR chain used for constraining the whole path?

    bool serialize(std::ostream& O) const; ///< write the TSR Chain to a string
    bool deserialize(std::stringstream& _ss); ///< parse a string to set the values of the TSR Chain
    bool deserialize_from_matlab(RobotBasePtr  robot, EnvironmentBasePtr penv_in, std::istream& _ss); ///< parse a string from matlab to set the values of the TSR Chain

private:

    void DestoryRobotizedTSRChain(); ///< delete the virtual manipulator from the environment

    bool bSampleGoalFromChain;
    bool bSampleStartFromChain;
    bool bConstrainToChain;

    Transform Tw0_e;
    int numdof;

    RobotBasePtr  robot;
    IkSolverBasePtr _pIkSolver;
    EnvironmentBasePtr penv;

    std::string mimicbodyname;
    RobotBasePtr _pmimicbody;
    std::vector<int> _mimicinds;
    std::vector<dReal> _mimicjointvals_temp;
    std::vector<dReal> _mimicjointoffsets;
    std::vector<dReal> _lowerlimits;
    std::vector<dReal> _upperlimits;
    bool _bPointTSR;

    Transform _tmtemp;
    std::vector<dReal> _dx;
    dReal _sumsqr;

    dReal _sumbounds;

    std::vector<OpenRAVE::dReal> ikparams;
};

#endif
