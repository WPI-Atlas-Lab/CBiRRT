/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Authors: Dmitry Berenson and Rosen Diankov <dberenso@cs.cmu.edu and rdiankov@cs.cmu.edu>

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

/** \file manipulation.h
    \brief Definition of the Manipulation Problem class. Based on a branch of openrave's manipulation problem. Contains a List of useful functions that do small tasks in openrave.
 */
#ifndef OPENRAVE_MANIPULATION_H
#define OPENRAVE_MANIPULATION_H

class ManipulationProblem : public ProblemInstance
{
public:
    ManipulationProblem(EnvironmentBasePtr penv);
    virtual ~ManipulationProblem();
    virtual void Destroy();
    virtual int main(const string& args);
    virtual void SetActiveRobots(const std::vector<RobotBasePtr>& robots);
    
    virtual bool SendCommand(ostream& response, istream& cmd);


    void EndState();
    
private:

    /// point struct for general use
    struct POINT
    {
        POINT(){}
        POINT(dReal px, dReal py, dReal pz, dReal nx, dReal ny, dReal nz){pos.x = px; pos.y = py; pos.z = pz; norm.x = nx; norm.y = ny; norm.z = nz;}

        Vector pos, norm;
        float dist;
        std::vector<float> dists;
        void PrintToFile(ofstream& of){ of << dist << " " << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z << "\n";}
    };

    /// does a grasp (using ik)
    int DoGrasp(istream& cmd);

    /// lift arm in a given direction
    int LiftArm(istream& cmd);

    /// lift arm using GeneralIK solver
    vector<dReal> LiftArmGeneralIK(istream& cmd);

    /// interpolate to a transform using GeneralIK solver
    vector<dReal> InterpolateToEEPose(istream& cmd);

    /// get points on wrist trajectory
    vector<dReal> GetWristPoints(istream& cmd);

    /// grabs a body
    int GrabBody(istream& cmd);
    
    /// restores mass of endeffector in controller
    int RestoreEndEffectorMass(istream& cmd);

    /// move a hand in a straight line until collision or IK fails. Note: hand stops when it is colliding, not before
    int MoveHandStraight(istream& cmd);

    /// move a hand in a straight line until collision or out of reach using jacobian (not IK). Note: hand stops when it is colliding, not before
    int JMoveHandStraight(istream& cmd);

    /// get an IK solution
    vector<dReal> IKtest(istream& cmd);

    /// Sets active manipulator to left arm, right arm, left leg, right leg, or head
    int SetActiveManip(istream& cmd);

    /// Moves a manipulator to a given set of joint values
    int MoveManipulator(istream& cmd);

    /// move the manipulator to some hand position
    int MoveToHandPosition(istream& cmd);
    
    /// set the transparency of a body
    int SetTransparency(istream& cmd);

    /// used for checking uncertainty of grasps
    int MakeGraspTrajectory(string& response, istream& cmd);

    /// move the arm to a goal configuration
    TrajectoryBasePtr MoveArm(const vector<int>& activejoints, const std::vector<dReal>& activegoalconfig, const RobotBase::ManipulatorPtr manip, KinBodyPtr ptarget);

    /// returns the jacobian for the robot in the current position
    int GetJacobian(string& response,istream& cmd);

    /// function for debugging and testing IK
    int DebugIK(istream& cmd);

    /// function for getting multiple manipulator poses
    vector<dReal> GetMultiManipPoses(istream& cmd);

    /// close the hand many times and check contacts
    vector<int> MultiHandClose(istream& cmd);

    /// test many transforms of a body for collision
    vector<int> MultiColCheck(istream& cmd);

    /// test many joint values of a robot for collision
    vector<int> RobotMultiColCheck(istream& cmd);

    /// get contacts between robot and object (along with link indices)
    int GetContacts(string& response,istream& cmd);    

    /// switch models between padded and non-padded
    int SwitchModelsInternal(vector<pair<string, string> >& vpatterns, bool tofat);
    int SwitchModels(istream& cmd);
    int SwitchModel(const string&, bool bToFatModel);

    /// if ptarget is NULL, will loo at hands, otherwise at ptarget
    void SetTrajectory(TrajectoryBasePtr ptraj, float fSpeed = 0.5f);

    /// randomly move a small amount to escape collision
    bool JitterActiveDOF(RobotBasePtr robot);

    /// randomly move a small amount to escape collision
    bool JitterTransform(KinBodyPtr pbody, float fJitter);


    RobotBasePtr robot;

    string _strRobotName; ///< name of the active robot
    PlannerBase::PlannerParameters params;
    ProblemInstancePtr pGrasperProblem;

    string _strRRTPlannerName;

    PlannerBasePtr _pPlanner; ///< for reusing planner across multiple planning invocations
    bool _reusePlanner;  ///< specify whether we want to reuse planner

    IkSolverBasePtr _pGeneralIKSolver;
};

#endif
