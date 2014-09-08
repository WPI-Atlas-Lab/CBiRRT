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


/** \file manipulation.cpp
    \brief Implementation of the Manipulation Problem class.
 */

#include "stdafx.h"


ManipulationProblem::ManipulationProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = ":Interface Author: Dmitry Berenson and Rosen Diankov\nA group of useful functions for manipulation. \n\n`C++ Documentation <http://automation.berkeley.edu/~berenson/docs/manipulation/index.html>`_";
    _reusePlanner = false;
}

void ManipulationProblem::Destroy()
{

}

ManipulationProblem::~ManipulationProblem()
{
    Destroy();
}

int ManipulationProblem::main(const string& args)
{
    const char* cmd = args.c_str();
    if( cmd == NULL )
        return 0;

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = p;

        _strRRTPlannerName.resize(0);
    
        // check whether we want to maintain single planner rather than
        // re-creating each planning episode
        const char* reusePlannerOpt = strstr(cmd, "reuseplanner ");
        if( reusePlannerOpt != NULL ) {
            
            reusePlannerOpt += 13;
            int reuse = 0;
            RAVELOG_INFO("Reuse planner parameter read as %s\n", reusePlannerOpt);

            sscanf(reusePlannerOpt, "%d", &reuse);
            
            // update our global reusePlanner parameter
            if (reuse == 1){
                _reusePlanner = true;
            } 
        }
        
        // check which planner we want to be using
        const char* planneropt = strstr(cmd, "planner ");
        if( planneropt != NULL ) {
            
        planneropt += 8;
        char plannername[255];
        if( sscanf(planneropt, "%s", plannername) == 1 ) {
            // try to create it
            PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),plannername);
            
                if( planner != NULL ) {
                _strRRTPlannerName = plannername;
            }
            else {
                RAVELOG_INFO("Manipulation: failed to find planner %s\n", plannername);
            }
        }
    }

    RAVELOG_DEBUG("Starting ManipulationProblem\n");
    

    if( _strRRTPlannerName.size() == 0 ) {
        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),"BiRRT");
        if( planner != NULL )
            _strRRTPlannerName = "BiRRT";
        if( planner == NULL ) {
            RAVELOG_INFO("failed to find birrt planner\n");
            return -1;
        }
    }

    RAVELOG_INFO("Manipulation using %s rrt planner\n", _strRRTPlannerName.c_str());

    // Now set up reusable planner with desired planning algorithm, if desired
    if (_reusePlanner){
        _pPlanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);

        RAVELOG_INFO("We are reusing planner so creating it now\n");
    }
    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots);    

    pGrasperProblem = RaveCreateProblem(GetEnv(),"GrasperProblem2");
    if( pGrasperProblem.get() == NULL ) {
        RAVELOG_DEBUG("Failed to create GrasperProblem2, trying standard grasper.\n");
        pGrasperProblem = RaveCreateProblem(GetEnv(),"Grasper");
    }

    if( pGrasperProblem.get() == NULL )
        RAVELOG_DEBUG("Failed to create GrasperProblem!\n");
    else
        RAVELOG_DEBUG("GrasperProblem created\n");
    pGrasperProblem->main(robot->GetName());


    _pGeneralIKSolver = RaveCreateIkSolver(GetEnv(),"GeneralIK");
    if(_pGeneralIKSolver.get() == NULL)
        RAVELOG_INFO("Could not find the GeneralIK plugin\n");
    else    
    {
        _pGeneralIKSolver->Init(robot->GetActiveManipulator());
        RAVELOG_INFO("GeneralIK solver initialized\n");
    }
    return 0;
}

void ManipulationProblem::SetActiveRobots(const std::vector<RobotBasePtr>& robots)
{
    RAVELOG_DEBUG("Starting SetActiveRobots...\n");
    robot.reset();

    if( robots.size() == 0 ) {
        RAVELOG_DEBUG("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr>::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( strcasecmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot.get() == NULL ) {
        RAVELOG_INFO("Failed to find %s\n", _strRobotName.c_str());
        return;
    }

}

bool ManipulationProblem::SendCommand(ostream& output, istream& sinput)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots); 
    string response;

    string p;
    while(!sinput.eof()) {
        sinput >> p;
        if( !sinput )
            break;

        if( stricmp(p.c_str(), "GrabBody") == 0 ) {
            if( robot.get() == NULL )
                return false;
            GrabBody(sinput);
        }
        else if( stricmp(p.c_str(), "RestoreEndEffectorMass") == 0 ) {
            if( robot.get() == NULL )
                return false;
            RestoreEndEffectorMass(sinput);
        }
        else if( stricmp(p.c_str(), "LiftArmGeneralIK") == 0 ) {
            if( robot.get() == NULL )
                return false;
            vector<dReal> v = LiftArmGeneralIK(sinput);

            if( v.size() > 0 ) {
                char str[16];
                for(size_t i = 0; i < v.size(); ++i) {
                    sprintf(str, "%f ", v[i]);
                    response += str;
                }
            }
            else
                response = "";

        }
	

        else if( stricmp(p.c_str(), "InterpolateToEEPose") == 0 ) {
            if( robot.get() == NULL )
                return false;
            vector<dReal> v = InterpolateToEEPose(sinput);

            if( v.size() > 0 ) {
                char str[16];
                for(size_t i = 0; i < v.size(); ++i) {
                    sprintf(str, "%f ", v[i]);
                    response += str;
                }
            }
            else
                response = "";

        }
        else if( stricmp(p.c_str(), "GetWristPoints") == 0 ) {
            if( robot.get() == NULL )
                return false;
            vector<dReal> v = GetWristPoints(sinput);

            if( v.size() > 0 ) {
                char str[16];
                for(size_t i = 0; i < v.size(); ++i) {
                    sprintf(str, "%f ", v[i]);
                    response += str;
                }
            }
            else
                response = "";

        }
        else if( stricmp(p.c_str(), "DoGrasp") == 0 ) {
            if( robot.get() == NULL )
                return false;

            if(DoGrasp(sinput) < 0)
                response = "0";
            else
                response = "1";
        }
        else if( stricmp(p.c_str(), "gettorques") == 0 ) {
            if( robot.get() == NULL || robot->GetController() == NULL)
                return false;

            vector<dReal> v;
            robot->GetController()->GetTorque(v);

            stringstream ss;
            FOREACH(itv, v) ss << *itv << " ";
            response = ss.str();
        }
        else if( stricmp(p.c_str(), "GetMultiManipPoses") == 0 ) {
            if( robot.get() == NULL )
                return false;

            vector<dReal> v = GetMultiManipPoses(sinput);

            if( v.size() > 0 ) {
                char str[16];
                for(size_t i = 0; i < v.size(); ++i) {
                    sprintf(str, "%f ", v[i]);
                    response += str;
                }
            }
            else
                response = "";
        }
        else if( stricmp(p.c_str(), "IKtest") == 0 ) {
            if( robot.get() == NULL )
                return false;

            vector<dReal> v = IKtest(sinput);

            if( v.size() > 0 ) {
                char str[16];
                for(size_t i = 0; i < v.size(); ++i) {
                    sprintf(str, "%f ", v[i]);
                    response += str;
                }
            }
            else
                response = "";
        }
        else if( stricmp(p.c_str(),  "MultiHandClose") == 0 ){
            if(robot.get() == NULL)
                return false;
            vector<int> v = MultiHandClose(sinput);
            char str[16];
            for(size_t i = 0; i < v.size(); ++i) {
                sprintf(str, "%d ", v[i]);
                response += str;
            }

        }
        else if( stricmp(p.c_str(),  "MultiColCheck") == 0 ){
            if(robot.get() == NULL)
                return false;
            vector<int> v = MultiColCheck(sinput);
            char str[16];
            for(size_t i = 0; i < v.size(); ++i) {
                sprintf(str, "%d ", v[i]);
                response += str;
            }

        }
        else if( stricmp(p.c_str(),  "RobotMultiColCheck") == 0 ){
            if(robot.get() == NULL)
                return false;
            vector<int> v = RobotMultiColCheck(sinput);

            char str[16];
            for(size_t i = 0; i < v.size(); ++i) {
                sprintf(str, "%d ", v[i]);
                response += str;
            }
            RAVELOG_INFO("Completed %d checks\n",v.size());
        }
        else if( stricmp(p.c_str(), "MoveToHandPosition") == 0 ) {
            if( robot.get() == NULL )
                return false;

            if(MoveToHandPosition(sinput) < 0)
                response = "0";
            else
                response = "1";
        }
        else if( stricmp(p.c_str(), "releaseall") == 0 ) {
            if( robot != NULL ) {
                RAVELOG_DEBUG("Releasing all bodies\n");
                robot->ReleaseAllGrabbed();
            }
        }
        else if( stricmp(p.c_str(), "LiftArm") == 0 ) {
            if( robot.get() == NULL )
                return false;

            if(LiftArm(sinput) < 0) response = "0";
            else response = "1";
        }
        else if( stricmp(p.c_str(), "MoveHandStraight" ) == 0) {
            if( robot.get() == NULL )
                return false;
            MoveHandStraight(sinput);
        }
        else if( stricmp(p.c_str(), "JMoveHandStraight" ) == 0) {
            if( robot.get() == NULL )
                return false;
            JMoveHandStraight(sinput);
        }
        else if( stricmp(p.c_str(), "grasper") == 0 ) {
            if( pGrasperProblem.get() != NULL )
            {
                stringstream outstr;
                outstr << response;
                pGrasperProblem->SendCommand(outstr,sinput);
                response = outstr.str();
            }
            else
                break;
        }

        else if( stricmp(p.c_str(), "stop") == 0 ) {
            // stop the robot
            if( robot != NULL && robot->GetController() != NULL ) {
                vector<dReal> values;
                robot->GetDOFValues(values);
                robot->GetController()->SetDesired(values);
            }
        }
        else if( stricmp(p.c_str(), "traj") == 0 ) {
            sinput >> p;
            if( p != "" && robot.get() != NULL ) {
                RAVELOG_DEBUG("ManipulationProblem: reading trajectory: %s\n", p.c_str());
                TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
                ifstream infile(p.c_str(),ios::in);
                ptraj->deserialize(infile);

                RAVELOG_VERBOSE("executing traj\n");
                robot->GetController()->SetPath(ptraj);
                infile.close();

                response="1";
            }
            else response="0";
        }
        else if( stricmp(p.c_str(), "SetActiveManip") == 0 ) {
            if( robot.get() == NULL )
                return false;
            SetActiveManip(sinput);
        }
        else if( stricmp(p.c_str(), "SwitchModels") == 0){
            if (robot.get() == NULL)
                return false;
            SwitchModels(sinput);
        }
        else if( stricmp(p.c_str(), "MoveManipulator") == 0 ) {
            if( robot.get() == NULL )
                return false;
            MoveManipulator(sinput);
        }
        else if( stricmp(p.c_str(), "GetJacobian") == 0 ) {
            if( robot.get() == NULL )
                return false;
            GetJacobian(response,sinput);
        }
        else if( stricmp(p.c_str(), "GetContacts") == 0 ) {
            if( robot.get() == NULL )
                return false;
            GetContacts(response,sinput);
        }
        else if( stricmp(p.c_str(), "SetTransparency") == 0 ) {
            if( robot.get() == NULL )
                return false;
            SetTransparency(sinput);
        }
        else if( stricmp(p.c_str(), "MakeGraspTrajectory") == 0 ) {
            if( robot.get() == NULL )
                return false;
            MakeGraspTrajectory(response,sinput);
        }
        else if( stricmp(p.c_str(), "DebugIK") == 0 ) {
            if( robot.get() == NULL )
                return false;
            DebugIK(sinput);
        }
        else if( stricmp(p.c_str(), "help") == 0 ) {
            response += "Please see manipulation.cpp for a list of the commands\n";
        }
        else {
            RAVELOG_WARN("unrecognized command: %s\n",p.c_str());
            break;
        }


        if( !sinput ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return false;
        }
    }

    output << response;
    return true;
}


vector<dReal> ManipulationProblem::GetMultiManipPoses(istream& cmd)
{
    RAVELOG_DEBUG("Starting GetMultiManipPoses...\n");

    vector<int> manipinds;
    vector< vector<dReal> > dofvals;
    
    vector<dReal> oldvalues;
    robot->GetDOFValues(oldvalues);

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "dofvals") == 0 ){
            int numdofvals, numdofs;
            cmd >> numdofvals;
            cmd >> numdofs;

            if(numdofs != robot->GetActiveDOF())
            {
                RAVELOG_INFO("ERROR: Number of input dofs does not match number of active dofs.\n");
                return vector<dReal> ();
            }

            dofvals.resize(numdofvals);
            for(int i = 0; i < numdofvals;i++)
            {
                dofvals[i].resize(numdofs);
                for(int j = 0; j < numdofs;j++)
                {
                    cmd >> dofvals[i][j];
                }
            }
        }
        else if( stricmp(p.c_str(), "manipinds") == 0 ) {
            int nummanipinds;
            cmd >> nummanipinds;
            manipinds.resize(nummanipinds);
            for(int i = 0; i < nummanipinds; i++)
                cmd >> manipinds[i];
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }
    }

    if( manipinds.size() == 0 )
    {
        RAVELOG_INFO("ERROR: No manipulator inds specified.\n");
        return vector<dReal>();
    }
    
    vector<dReal> out;

    Transform temptm;
    for(int i = 0; i < dofvals.size();i++)
    {
        robot->SetActiveDOFValues(dofvals[i]);
        for(int j = 0; j < manipinds.size();j++)
        {
            temptm = robot->GetManipulators()[manipinds[j]]->GetEndEffectorTransform();
            out.push_back(temptm.trans.x);
            out.push_back(temptm.trans.y);
            out.push_back(temptm.trans.z);
            out.push_back(temptm.rot.w);
            out.push_back(temptm.rot.x);
            out.push_back(temptm.rot.y);
            out.push_back(temptm.rot.z);
        }
    }


    robot->SetJointValues(oldvalues);
    return out;
}


vector<dReal> ManipulationProblem::IKtest(istream& cmd)
{
    RAVELOG_DEBUG("Starting IKtest...\n");

    TransformMatrix handTm;
    vector<dReal> vhandjointvals, varmjointvals, oldvalues, values;
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    
    robot->GetDOFValues(oldvalues);
    values = oldvalues;
    bool bCheckCollision = true;

    KinBodyPtr ptarget;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;
            cmd >> handTm.trans.y;
            cmd >> handTm.trans.z;
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];
            cmd >> handTm.m[4];
            cmd >> handTm.m[8];
            cmd >> handTm.m[1];
            cmd >> handTm.m[5];
            cmd >> handTm.m[9];
            cmd >> handTm.m[2];
            cmd >> handTm.m[6];
            cmd >> handTm.m[10];
        }
        else if( stricmp(p.c_str(), "handjoints") == 0 ) {
            if( pmanip.get() != NULL )
                vhandjointvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vhandjointvals.size(); ++i)
                cmd >> vhandjointvals[i];
        }
        else if( stricmp(p.c_str(), "setjoint") == 0 ) {
            int ind;
            cmd >> ind;
            if( ind >= 0 && ind < (int)values.size() ) {
                cmd >> values[ind];
            }
        }
        //this is useful b/c we always get the nearest IK to the current joints
        else if( stricmp(p.c_str(), "armjoints") == 0 ) {
            if( pmanip != NULL )
                varmjointvals.resize(pmanip->GetArmIndices().size());
            for(size_t i = 0; i < varmjointvals.size(); ++i)
                cmd >> varmjointvals[i];
        }
        else if( stricmp(p.c_str(), "nocol") == 0 ) {
            bCheckCollision = false;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }
    }

    if( pmanip == NULL )
        return vector<dReal>();

    Transform handTr(handTm);


    for(size_t i = 0; i < vhandjointvals.size(); i++) {
        values[pmanip->GetGripperIndices()[i]] = vhandjointvals[i];
    }


    for(size_t i = 0; i < varmjointvals.size(); i++) {
        values[pmanip->GetArmIndices()[i]] = varmjointvals[i];
    }

    robot->SetJointValues(values);

    vector<dReal> q1;
    q1.resize(pmanip->GetArmIndices().size());

    std::vector< std::vector<dReal> > qSolutions;
    qSolutions.resize(16);
    for(size_t i = 0; i < qSolutions.size(); i++)
    {            
        qSolutions[i].resize(7);
    }  

    RAVELOG_INFO("Colcheck: %d\n",bCheckCollision);

    pmanip->FindIKSolutions(handTr, qSolutions, bCheckCollision);
    stringstream s;
    s << L"ik sol: ";
    for(size_t i = 0; i < qSolutions.size(); i++)
    {            
        for(size_t j = 0; j < qSolutions[i].size(); j++)
            s << qSolutions[i][j] << " ";

        s << endl;
    }   

    RAVELOG_DEBUG(s.str().c_str());

    if( !pmanip->FindIKSolution(handTr, q1, bCheckCollision) )
    {
        RAVELOG_INFO("No IK solution found\n");   
        robot->SetJointValues(oldvalues);
        return vector<dReal>();
    }
    else
    {
        RAVELOG_INFO("IK solution found\n"); 
        stringstream s2;
        for(size_t i = 0; i < q1.size(); i++)
            s2 << q1[i] << " ";
        s2 << endl;
        RAVELOG_INFO(s2.str().c_str());
    }

    robot->SetJointValues(oldvalues);
    return q1;
}

std::vector<int> ManipulationProblem::MultiColCheck(istream& cmd)
{
    RAVELOG_DEBUG("Starting MultiColCheck...\n");
    
    std::vector<int> out;


    int numtms;
    std::vector<Transform> vtransforms;

    KinBodyPtr pbody;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    


    TransformMatrix handTm;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "body") == 0 ) {
            cmd >> p;
            pbody = GetEnv()->GetKinBody(p);   
            
        }
        else if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;
            for(int i = 0; i < numexcluded; i++)
            {
                cmd >> p;
                ptemp = GetEnv()->GetKinBody(p);
                if(ptemp != NULL)
                    vexcluded.push_back(KinBodyConstPtr(ptemp));   
                else
                {
                    RAVELOG_INFO("ERROR: Unknown excluded body\n");   
                    return out;
                }      
            }
        }
        else if( stricmp(p.c_str(), "transforms") == 0 ) { 
            cmd >> numtms;
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> vtransforms[i].rot.x;
                cmd >> vtransforms[i].rot.y;
                cmd >> vtransforms[i].rot.z;
                cmd >> vtransforms[i].rot.w;
                
                cmd >> vtransforms[i].trans.x;
                cmd >> vtransforms[i].trans.y;
                cmd >> vtransforms[i].trans.z;
            }
    
        }
        else if( stricmp(p.c_str(), "transforms12") == 0 ) {
            cmd >> numtms;
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> handTm.m[0];
                cmd >> handTm.m[4];
                cmd >> handTm.m[8];
                cmd >> handTm.m[1];
                cmd >> handTm.m[5];
                cmd >> handTm.m[9];
                cmd >> handTm.m[2];
                cmd >> handTm.m[6];
                cmd >> handTm.m[10];
                cmd >> handTm.trans.x;
                cmd >> handTm.trans.y;
                cmd >> handTm.trans.z;
                vtransforms[i] = (Transform) handTm;
            }

        }

        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return std::vector<int>();
        }
    

        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return std::vector<int>();
        }

    }
    
    if(pbody.get() == NULL)
    {       
        RAVELOG_INFO("ERROR: Unknown body\n");   
        return out;
    }

    Transform tbackup = pbody->GetTransform();
    int col = 0;
    out.resize(numtms);

    CollisionReportPtr preport(new CollisionReport());

    for(int i = 0; i < vtransforms.size(); i++)
    {    
        pbody->SetTransform(vtransforms[i]);
        if(numexcluded)
            col = GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vexcluded,vlinkexcluded,preport);
        else
            col = GetEnv()->CheckCollision(KinBodyConstPtr(pbody));

        out[i] = col;
        RAVELOG_DEBUG("col: %d\n",col);
    }

    pbody->SetTransform(tbackup);
    return out;
}

std::vector<int> ManipulationProblem::RobotMultiColCheck(istream& cmd)
{
    RAVELOG_DEBUG("Starting RobotMultiColCheck...\n");
    
    std::vector<int> out;


    int numtests;
    KinBodyPtr pbody;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    std::vector<dReal> vjointvals;
    std::vector<std::vector<dReal> > vvtestvals;

    robot->GetActiveDOFValues(vjointvals);
    int numdof = vjointvals.size();

    bool bSimpleCheck = false;

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;
            for(int i = 0; i < numexcluded; i++)
            {
                cmd >> p;
                ptemp = GetEnv()->GetKinBody(p);
                if(ptemp != NULL)
                    vexcluded.push_back(ptemp);   
                else
                {
                    RAVELOG_INFO("ERROR: Unknown excluded body\n");   
                    return out;
                }      
            }
        }
        else if( stricmp(p.c_str(), "values") == 0 ) {
            cmd >> numtests;
            vvtestvals.resize(numtests);
            out.resize(numtests);
            for(int i = 0; i < numtests; i++)
            {   
                vvtestvals[i].resize(numdof);
                for(int j = 0; j < numdof; j++)
                    cmd >> vvtestvals[i][j];
                
            }
        }
        else if( stricmp(p.c_str(), "simplecheck") == 0 ) {
            bSimpleCheck = true;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return std::vector<int>();
        }
    

        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return std::vector<int>();
        }

    }
    
    int col = 0;
    int selfcol = 0;
    int bodyindex = 0;
    double starttime;
    double colchecktime;
    if(bSimpleCheck)
    {
        for(int i = 0; i < vvtestvals.size(); i++)
        {
            double starttime = timeGetThreadTime();

            robot->SetActiveDOFValues(vvtestvals[i]);
            col = GetEnv()->CheckCollision(KinBodyConstPtr(robot));
            selfcol = robot->CheckSelfCollision();

            colchecktime = colchecktime + (timeGetThreadTime() - starttime);

            out[i] = col || selfcol; //0 means no collision, 1 means self collision, >1 is id of body collided with: id = val >> 1
        }
    }
    else
    {
        CollisionReportPtr report;
        report.reset(new CollisionReport);
        for(int i = 0; i < vvtestvals.size(); i++)
        {
            robot->SetActiveDOFValues(vvtestvals[i]);
            col = GetEnv()->CheckCollision(KinBodyConstPtr(robot),vexcluded,vlinkexcluded,report);
            selfcol = robot->CheckSelfCollision();

            bodyindex = 0;
            if( report->plink1 != NULL && report->plink1->GetParent() != robot )
                bodyindex = report->plink1->GetParent()->GetEnvironmentId();
            if( report->plink2 != NULL && report->plink2->GetParent() != robot )
                bodyindex = report->plink2->GetParent()->GetEnvironmentId();

            out[i] = (bodyindex<<1) | selfcol; //0 means no collision, 1 means self collision, >1 is id of body collided with: id = val >> 1
        }
    }
    RAVELOG_INFO("Time: %f\n",colchecktime);
    robot->SetActiveDOFValues(vjointvals);
    return out;
}


int ManipulationProblem::SetActiveManip(istream& cmd)
{
    RAVELOG_DEBUG("Starting SetActiveManip...\n");

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "rightarm") == 0 ) {
            
            robot->SetActiveManipulator(4);
            
        }
        else if( stricmp(p.c_str(), "leftarm") == 0 ) {
            
            robot->SetActiveManipulator(3);
            
        }
        else if( stricmp(p.c_str(), "leftleg") == 0 ) {
            
            robot->SetActiveManipulator(1);
           
        }
        else if( stricmp(p.c_str(), "rightleg") == 0 ) {
            
            robot->SetActiveManipulator(2);
            
        }
        else if( stricmp(p.c_str(), "head") == 0 ) {
            
            robot->SetActiveManipulator(0);
            
        }
        else if( stricmp(p.c_str(), "index") == 0 ) {
            int ind;
            cmd >> ind;
            if( ind >= 0 && ind < (int)(robot->GetManipulators().size()) ) 
            {
                robot->SetActiveManipulator(ind);
                return 0;
            }
            else
            {
                RAVELOG_INFO("ERROR: manipulator index out of bounds.\n");
                return -1;
            }
            
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }

    return 0;
}


int ManipulationProblem::MoveManipulator(istream& cmd)
{
    RAVELOG_DEBUG("Starting MoveManipulator...\n");

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    std::vector<dReal> goals;
    std::vector<int> activejoints;
    bool bUseHand = false;
    bool btestmode = false;

    


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "leftleg") == 0 ) {
            
            robot->SetActiveManipulator(1);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p.c_str(), "rightleg") == 0 ) {
            
            robot->SetActiveManipulator(2);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p.c_str(), "head") == 0 ) {
            
            robot->SetActiveManipulator(0);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p.c_str(), "armvals") == 0 ) {
            dReal temp;
            for(size_t i = 0; i < robot->GetActiveManipulator()->GetArmIndices().size(); i++)
            {            
                cmd >> temp;
                goals.push_back(temp);
                activejoints.push_back(pmanip->GetArmIndices()[i]);
            }   
            
        }
        else if( stricmp(p.c_str(), "handvals") == 0 ) {
            bUseHand = true;
            dReal temp;
            for(size_t i = 0; i < robot->GetActiveManipulator()->GetGripperIndices().size(); i++)
            {
                cmd >> temp;
                goals.push_back(temp);
                activejoints.push_back(pmanip->GetGripperIndices()[i]);
            }
        }
        else if( stricmp(p.c_str(), "test") == 0 ) {
            btestmode = true;
        } 
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }
    
    robot->SetActiveDOFs(pmanip->GetArmIndices());
    JitterActiveDOF(robot);
    
    TrajectoryBasePtr ptraj = MoveArm(activejoints, goals,pmanip,KinBodyPtr());
    if( ptraj == NULL )
        return -1;
    if(!btestmode)
        SetTrajectory(ptraj);

    return 0;
}


int ManipulationProblem::GetJacobian(string& response, istream& cmd)
{
    RAVELOG_DEBUG("Starting GetJacobian...\n");

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    Transform tEE;
    TransformMatrix tmEE;
    if(pmanip != NULL)
        tEE = pmanip->GetEndEffectorTransform();
    else
    {
        RAVELOG_INFO("Error: pmanip is null!\n");
        return -1;   
    }

    KinBodyPtr ptarget;


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "transform") == 0 ){
            cmd >> tmEE.m[0];
            cmd >> tmEE.m[4];
            cmd >> tmEE.m[8];
            cmd >> tmEE.m[1];
            cmd >> tmEE.m[5];
            cmd >> tmEE.m[9];
            cmd >> tmEE.m[2];
            cmd >> tmEE.m[6];
            cmd >> tmEE.m[10];
            cmd >> tmEE.trans.x;
            cmd >> tmEE.trans.y;
            cmd >> tmEE.trans.z;
            tEE = Transform(tmEE);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }
    
    std::vector<dReal> J(robot->GetDOF()*3);

    robot->CalculateJacobian(pmanip->GetEndEffector()->GetIndex(), tEE.trans, J);

    char str[16];
    for(size_t i = 0; i < J.size(); ++i) {
        sprintf(str, "%f ", J[i]);
        response += str;
    }

    return 0;

}

int ManipulationProblem::GetContacts(string& response, istream& cmd)
{
    RAVELOG_DEBUG("Starting GetContacts...\n");

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    
    KinBodyPtr pbody;
    bool bGetLinkInds = false;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "getlinkinds") == 0 ){
            bGetLinkInds = true;
        }
        else if( stricmp(p.c_str(), "body") == 0 ){
            string name; cmd >> name;
            pbody = GetEnv()->GetKinBody(name);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }

    CollisionReportPtr report(new CollisionReport());
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    vbodyexcluded.push_back(pbody);

    bool get_contacts = !GetEnv()->CheckCollision(KinBodyConstPtr(robot),vbodyexcluded,vlinkexcluded,report) && !robot->CheckSelfCollision();
    stringstream sout;
    if( get_contacts ) {
        // calculate the contact normals
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
        vector<KinBody::LinkPtr>::const_iterator itlink;
        int icont=0;
        FOREACHC(itlink, robot->GetLinks()) {
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(pbody), report) ) {
                RAVELOG_DEBUGA(str(boost::format("contact %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));

                for(size_t i = 0; i < report->contacts.size(); i++) {
                    Vector pos = report->contacts[i].pos;
                    Vector norm = report->contacts[i].norm;
                    if( report->plink1 != *itlink )
                        norm = -norm;
                    sout << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
                    if(bGetLinkInds)
                        sout << (*itlink)->GetIndex();
                    sout << endl;
                    icont++;
                    //GetEnv()->plot3(pos, 1, 0);
                }
            }
        }
        RAVELOG_DEBUGA("number of contacts: %d\n", icont);
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    
    }
    else if( !!report->plink1 && !!report->plink2 ) {
        RAVELOG_WARNA(str(boost::format("collision %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));
    }

    response = sout.str().c_str();
    return 0;

}


int ManipulationProblem::DebugIK(istream& cmd)
{
    RAVELOG_DEBUG("Starting DebugIK...\n");    
    
    int num_itrs = 10000;
    stringstream s;
    fstream fsfile;

    KinBodyPtr ptarget;
    string filename;
    bool bReadFile = false;
    bool bGenFile = false;


    


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "readfile") == 0 ) {
            cmd >> filename;
            bReadFile = true;
        }
        else if( stricmp(p.c_str(), "genfile") == 0 ) {
            cmd >> filename;
            bGenFile = true;
        }
        else if( stricmp(p.c_str(), "numtests") == 0 ) {
            cmd >> num_itrs;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }

    }


    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    std::vector<dReal> vjoints(pmanip->GetArmIndices().size(),0);
    std::vector<dReal> vzero(pmanip->GetArmIndices().size(),0);

    std::vector<dReal> vlowerlimit(pmanip->GetArmIndices().size(),0);
    std::vector<dReal> vupperlimit(pmanip->GetArmIndices().size(),0);


    std::vector<dReal> viksolution;

    robot->SetActiveDOFs(pmanip->GetArmIndices());
    robot->GetActiveDOFLimits(vlowerlimit, vupperlimit);


    if(bGenFile && bReadFile)
    {
        RAVELOG_INFO("ManipulationProblem::DebugIK - Error: Cannot generate file and use it at the same time.\n");
        return -1;
    }
    
    


    if(bGenFile)
    {
        fsfile.open(filename.c_str(),ios_base::out);
        fsfile << num_itrs <<endl;


    }
    if(bReadFile)
    {
        fsfile.open(filename.c_str(),ios_base::in);
        if(!fsfile.is_open())
        {
            RAVELOG_INFO("ManipulationProblem::DebugIK - Error: Cannot open specified file.\n");
            return -1;
        }

        fsfile >> num_itrs;

    }

    Transform twrist, twrist_out;
    float error,minuserr;
    int i = 0;
    int success = 0;
    float junk;
    while(i < num_itrs)
    {
        if(bReadFile)
        {
            for(int j = 0; j < vjoints.size(); j++)
                fsfile >> vjoints[j];

            //don't care about the transform
            fsfile >> junk;
            fsfile >> junk;
            fsfile >> junk;
            
            fsfile >> junk;
            fsfile >> junk;
            fsfile >> junk;
            fsfile >> junk;
        }
        else
            for(int j = 0; j < vjoints.size(); j++)
                vjoints[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*RANDOM_FLOAT();

        robot->SetActiveDOFValues(vjoints);

        if(GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || robot->CheckSelfCollision())
        {
            continue;
        }
        twrist = pmanip->GetEndEffector()->GetTransform();


        if(bGenFile)
        {
            for(int j = 0; j < vjoints.size(); j++)
                fsfile << vjoints[j] << " ";  

            fsfile << twrist.trans.x << " "<< twrist.trans.y << " "<< twrist.trans.z << " "<< twrist.rot.x << " "<< twrist.rot.y << " "<< twrist.rot.z << " "<< twrist.rot.w;

            fsfile << endl;            

        }

        robot->SetActiveDOFValues(vzero);
  

        if( !pmanip->FindIKSolution(twrist, viksolution, true) ) {
            
            s.str("");
            s << L"Error, No ik solution found, i = " << i << endl << L"Joint Val: ";
            for(size_t j = 0; j < vjoints.size(); j++)
                s << vjoints[j] << " ";   
            s << endl << L"Transform: " << twrist.trans.x << " "<< twrist.trans.y << " "<< twrist.trans.z << " "<< twrist.rot.x << " "<< twrist.rot.y << " "<< twrist.rot.z << " "<< twrist.rot.w << endl;
            RAVELOG_INFO(s.str());
            i++;
            continue;
        }

        robot->SetActiveDOFValues(viksolution);
        twrist_out = pmanip->GetEndEffector()->GetTransform();
        
        /*error = sqrt(lengthsqr4(twrist.rot - twrist_out.rot));
        minuserr = sqrt(lengthsqr4(twrist.rot + twrist_out.rot));*/
        RaveVector<dReal> temp4;
        temp4 = twrist.rot - twrist_out.rot;
        error = sqrt(temp4.lengthsqr4());
        temp4 = twrist.rot + twrist_out.rot;
        minuserr = sqrt(temp4.lengthsqr4());
        
        if(minuserr < error)
            error = minuserr;
        
        /*error += sqrt(lengthsqr3(twrist.trans - twrist_out.trans));*/
        RaveVector<dReal> temp3;
        temp3 = twrist.trans - twrist_out.trans;
        error += sqrt(temp3.lengthsqr3());

        if(error > 0.1f)
        {
            s.str("");
            s << L"Error - Incorrect IK, i = " << i <<L" error: " << error << endl << L"Joint Val: ";
            for(size_t j = 0; j < vjoints.size(); j++)
                s << vjoints[j] << " ";   
            s << endl << L"Transform in: " << twrist.trans.x << " "<< twrist.trans.y << " "<< twrist.trans.z << " "<< twrist.rot.x << " "<< twrist.rot.y << " "<< twrist.rot.z << " "<< twrist.rot.w << endl;
            s << endl << L"Transform out: " << twrist_out.trans.x << " "<< twrist_out.trans.y << " "<< twrist_out.trans.z << " "<< twrist_out.rot.x << " "<< twrist_out.rot.y << " "<< twrist_out.rot.z << " "<< twrist_out.rot.w << endl;
            RAVELOG_INFO(s.str());
        }

        success++;
        i++;
    }

    RAVELOG_INFO("DebugIK done, success rate %f.\n", (float)success/(float)num_itrs);

    return 0;
}


#define SWITCHMODELS(tofat) { \
    SwitchModelsInternal(vSwitchPatterns, tofat); \
    ptarget = GetEnv()->GetKinBody(targetname.c_str()); \
    pbodyTable = GetEnv()->GetKinBody(tablename.c_str()); \
    assert( ptarget != NULL && pbodyTable != NULL ); \
} \



int ManipulationProblem::DoGrasp(istream& cmd)
{
    RAVELOG_DEBUG("Starting DoGrasp...\n");

    TransformMatrix handTm;
    bool bOpenHand = false;
    bool bSetHand = false;

    vector<dReal> vopenhandvals;
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    vector<dReal> oldvalues, newvalues, q1;
        
    robot->GetDOFValues(oldvalues);
    newvalues = oldvalues;

    bool bTest = false;

    KinBodyPtr ptarget;


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;
            cmd >> handTm.trans.y;
            cmd >> handTm.trans.z;
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];
            cmd >> handTm.m[4];
            cmd >> handTm.m[8];
            cmd >> handTm.m[1];
            cmd >> handTm.m[5];
            cmd >> handTm.m[9];
            cmd >> handTm.m[2];
            cmd >> handTm.m[6];
            cmd >> handTm.m[10];
        }
        else if( stricmp(p.c_str(), "openhand") == 0 ) {
            bOpenHand = true;
            if( pmanip.get() != NULL )
                vopenhandvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vopenhandvals.size(); ++i)
                cmd >> vopenhandvals[i];
        }
        else if( stricmp(p.c_str(), "sethand") == 0 ) {
            bSetHand = true;
            if( pmanip.get() != NULL )
                vopenhandvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vopenhandvals.size(); ++i)
                cmd >> vopenhandvals[i];
        }
        else if( stricmp(p.c_str(), "target") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "test") == 0 ){
            bTest = true;
        }
        else if( stricmp(p.c_str(), "setjoint") == 0 ) {
            int ind;// = atoi(pindex);
            cmd >> ind;
            if( ind >= 0 && ind < (int)newvalues.size() ) {
                cmd >> newvalues[ind];// = (dReal)atof(pvalue);
            }
        }

        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
    }

    if( pmanip.get() == NULL )
        return -1;

    // check ik
    Transform handTr(handTm);

    if( bOpenHand || bSetHand ) {
        for(size_t i = 0; i < vopenhandvals.size(); i++) {
            newvalues[pmanip->GetGripperIndices()[i]] = vopenhandvals[i];
        }
    }
    
    robot->SetJointValues(newvalues, true);

    q1.resize(pmanip->GetArmIndices().size());
    if( !pmanip->FindIKSolution(handTr, q1, true) )
    {
        RAVELOG_INFO("No IK solution found\n"); 
        robot->SetJointValues(oldvalues, true);
        return -1;
    }
    else
    {
        stringstream s;
        s << "IK found: "; 
        for(size_t i = 0; i < q1.size(); i++)
            s << q1[i] << " ";
        s << endl;
        RAVELOG_INFO(s.str());
    }

    std::vector<dReal> armgoals;
    std::vector<int> activejoints;

    for(size_t i = 0; i < pmanip->GetArmIndices().size(); i++) {
        activejoints.push_back(pmanip->GetArmIndices()[i]);
        armgoals.push_back(q1[i]);
    }

    if( bOpenHand ) {
        for(size_t i = 0; i < vopenhandvals.size(); i++) {
            activejoints.push_back(pmanip->GetGripperIndices()[i]);
            armgoals.push_back(vopenhandvals[i]);
        }
    }

    // test collisions for the goal
    {
        RobotBase::RobotStateSaver saver(robot);
        robot->SetActiveDOFs(activejoints);
        robot->SetActiveDOFValues(armgoals);
        CollisionReportPtr report;
        report.reset(new CollisionReport);
        if( GetEnv()->CheckCollision(KinBodyConstPtr(robot), report) ) {
            RAVELOG_INFO("DoGrasp: collision %S:%S with %S:%S\n", report->plink1->GetParent()->GetName().c_str(), report->plink1->GetName().c_str(), report->plink2->GetParent()->GetName().c_str(), report->plink2->GetName().c_str());
            
            return -1;
        }
    }

    if( bTest ) // return to notify that IK finished
        return 0;

    if( bSetHand ) {
        vector<dReal> vjoints;
        robot->GetDOFValues(vjoints);
            
        for(size_t i = 0; i < vopenhandvals.size(); i++) {
            vjoints[pmanip->GetGripperIndices()[i]] = vopenhandvals[i];
        }
        
        robot->SetJointValues(vjoints, true);
        robot->GetController()->SetDesired(vjoints);
    }

    TrajectoryBasePtr ptraj = MoveArm(activejoints, armgoals,pmanip,ptarget);
    if( ptraj == NULL )
        return -1;
    SetTrajectory(ptraj);

    return 0;
}

vector<dReal> ManipulationProblem::LiftArmGeneralIK(istream& cmd)
{
    RAVELOG_DEBUG("Starting LiftArmGeneralIK...\n");
    dReal step_size = 0.003f;
    Vector direction = Vector(0,1,0);
    
    int i;
    int j;
    int minsteps = 10;
    int maxsteps = 60;
    bool bExec = true;
    bool bBreakonCollision = false;
    bool bAllowLimAdj = true;
    KinBody::JointPtr joint;
    vector<dReal> j_lower;
    vector<dReal> j_upper;
    vector<KinBody::JointPtr> limadj_joints;
    vector<vector<dReal> > limadj_lowers;
    vector<vector<dReal> > limadj_uppers;

    KinBodyPtr ptarget;
    string filename = "liftarmtraj.txt";

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "target") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "minsteps") == 0 ) {
            cmd >> minsteps;

        }
        else if( stricmp(p.c_str(), "maxsteps") == 0 ) {
            cmd >> maxsteps;

        }
        else if( stricmp(p.c_str(), "direction") == 0 ) {
            cmd >> direction.x;
            cmd >> direction.y;
            cmd >> direction.z;
        }
        else if( stricmp(p.c_str(), "exec") == 0 ) {
            cmd >> bExec;
        }
        else if( stricmp(p.c_str(), "breakoncollision") == 0 ) {
            cmd >> bBreakonCollision;
        }
        else if( stricmp(p.c_str(), "allowlimadj") == 0 ) {
            cmd >> bAllowLimAdj;
        }
        else if( stricmp(p.c_str(), "filename") == 0 ) {
            cmd >> filename;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }



    }

    if( ptarget.get() != NULL )
        robot->Grab(ptarget);

    std::vector<int> temp_indices = robot->GetActiveDOFIndices();
    
    robot->RegrabAll();
    RobotBase::RobotStateSaver saver(robot);
    
    Transform handTr = robot->GetActiveManipulator()->GetEndEffectorTransform();
    
    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());
    //Trajectory::TPOINT point;
    //point.q.resize(robot->GetActiveDOF());
    std::vector<dReal> curconfig;
    robot->GetActiveDOFValues(curconfig);
    ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());

    //ptraj->AddPoint(point);
    
    std::vector<dReal> qResult(robot->GetActiveDOF());

    
    boost::shared_ptr<std::vector<dReal> > pqResult(new std::vector<dReal> );


    vector<dReal> vPrevValues;
    robot->GetActiveDOFValues(vPrevValues);
    
    /* Fix joint limits if current values are outside! */
    if (bAllowLimAdj)
    {
        vector<dReal> start;
        vector<dReal> lower;
        vector<dReal> upper;
        vector<int> inds;
        vector<dReal> j_start;

        robot->GetActiveDOFValues(start);
        robot->GetActiveDOFLimits(lower, upper);
        inds = robot->GetActiveDOFIndices();
        
        /* Save all current joint values */
        for (i=0; i<start.size(); i++) if (start[i] < lower[i] || start[i] > upper[i])
        {
            RAVELOG_INFO("Temporarily adjusting joint limit [%d].\n", i);
            joint = robot->GetJointFromDOFIndex(inds[i]);
            joint->GetLimits(j_lower, j_upper);
            limadj_joints.push_back(joint);
            limadj_lowers.push_back(j_lower);
            limadj_uppers.push_back(j_upper);
        }
        for (i=0; i<start.size(); i++) if (start[i] < lower[i] || start[i] > upper[i])
        {
            joint = robot->GetJointFromDOFIndex(inds[i]);
            joint->GetValues(j_start);
            joint->GetLimits(j_lower, j_upper);
            for (j=0; j<j_start.size(); j++)
            {
                if (j_start[j] < j_lower[j]) j_lower[j] = j_start[j];
                if (j_start[j] > j_upper[j]) j_upper[j] = j_start[j];
            }
            joint->SetLimits(j_lower, j_upper);
        }
    }

    std::vector<dReal> ikparams;


    for (i = 0; i < maxsteps;  i++) {
        handTr.trans = handTr.trans + step_size*direction;

        //assemble parameters for passing to general IK solver
        ikparams.clear();
        ikparams.push_back(1); //number of manipulators to do IK for
        ikparams.push_back(robot->GetActiveManipulatorIndex()); //index of this manipulator
        ikparams.push_back(handTr.rot.x); ikparams.push_back(handTr.rot.y); ikparams.push_back(handTr.rot.z); ikparams.push_back(handTr.rot.w);
        ikparams.push_back(handTr.trans.x); ikparams.push_back(handTr.trans.y); ikparams.push_back(handTr.trans.z);
        ikparams.push_back(0); //don't do anything about COG
        ikparams.push_back(0); //select mode
        ikparams.push_back(0); //do rotation



        if( !_pGeneralIKSolver->Solve(IkParameterization(), curconfig, ikparams, false, pqResult)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }
        curconfig = *pqResult.get();



        size_t j = 0;
        for(; j < curconfig.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,point.q[j],vPrevValues[j]);
            if(fabsf(curconfig[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < curconfig.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }
        
        //ptraj->AddPoint(point);
        ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());
        robot->SetActiveDOFValues(curconfig);

        //Transform gottm = robot->GetActiveManipulator()->GetEndEffectorTransform();
        //RAVELOG_INFO("requested: %f %f %f got: %f %f %f\n",handTr.trans.x,handTr.trans.y, handTr.trans.z,gottm.trans.x,gottm.trans.y, gottm.trans.z);

        bool bInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot));

        if(bBreakonCollision && bInCollision)
        {
            RAVELOG_DEBUG("bBreakonCollision is true and arm is in collision, breaking\n");
            break;
        }


        if(!bInCollision && i > minsteps) {
	  RAVELOG_DEBUG("Arm Lifting: free of initial collision after %d steps\n",i);
            break;
        }

        vPrevValues = curconfig;
    }
    if (bBreakonCollision && (i<minsteps)) {
      RAVELOG_INFO("Arm Lifting failed: collided after %d steps (minsteps was %d)\n",
		   i, minsteps);
      for (j=0; j<limadj_joints.size(); j++)
      {
          joint = limadj_joints[j];
          j_lower = limadj_lowers[j];
          j_upper = limadj_uppers[j];
          joint->SetLimits(j_lower, j_upper);
      }
      return std::vector<dReal>();
    }

    RAVELOG_DEBUG("Arm Lifted: %d steps\n", i);

    if(bExec)
    {
        SetTrajectory(ptraj);
    }
    else
    {
        //TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
        //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        //pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), true, false);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj,robot,false,1,1,"LinearTrajectoryRetimer");
#else
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj,robot,false,1,"LinearTrajectoryRetimer");
#endif
        ofstream outfile(filename.c_str(), ios::out);
        ptraj->serialize(outfile);
        //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        outfile.close();
        chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
    }

    robot->SetActiveDOFs(temp_indices);
    std::vector<dReal> lastconfig;
    if(ptraj->GetNumWaypoints() > 1)
        ptraj->GetWaypoint(ptraj->GetNumWaypoints()-1,lastconfig);
        //lastconfig = ptraj->GetPoints()[ptraj->GetPoints().size()-1].q;

    for (j=0; j<limadj_joints.size(); j++)
    {
        joint = limadj_joints[j];
        j_lower = limadj_lowers[j];
        j_upper = limadj_uppers[j];
        joint->SetLimits(j_lower, j_upper);
    }
    return lastconfig;
}

vector<dReal> ManipulationProblem::InterpolateToEEPose(istream& cmd)
{
    RAVELOG_INFO("Starting InterpolateToEEPose...\n");
    int numsteps = 0;


    bool bExec = true;
    TransformMatrix targTm;


    KinBodyPtr ptarget;
    string filename = "liftarmtraj.txt";

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "numsteps") == 0 ) {
            cmd >> numsteps;

        }
        else if( stricmp(p.c_str(), "targtm") == 0 ){
            cmd >> targTm.m[0];
            cmd >> targTm.m[4];
            cmd >> targTm.m[8];
            cmd >> targTm.m[1];
            cmd >> targTm.m[5];
            cmd >> targTm.m[9];
            cmd >> targTm.m[2];
            cmd >> targTm.m[6];
            cmd >> targTm.m[10];
            cmd >> targTm.trans.x;
            cmd >> targTm.trans.y;
            cmd >> targTm.trans.z;
        }
        else if( stricmp(p.c_str(), "exec") == 0 ) {
            cmd >> bExec;
        }
        else if( stricmp(p.c_str(), "filename") == 0 ) {
            cmd >> filename;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }
    }

    Transform targTr = Transform(targTm);

    std::vector<int> temp_indices = robot->GetActiveDOFIndices();

    robot->RegrabAll();
    RobotBase::RobotStateSaver saver(robot);

    Transform handTr = robot->GetActiveManipulator()->GetEndEffectorTransform();

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());

    //Trajectory::TPOINT point;
    //point.q.resize(robot->GetActiveDOF());
    std::vector<dReal> curconfig;
    robot->GetActiveDOFValues(curconfig);

    std::vector<dReal> qResult(robot->GetActiveDOF());


    boost::shared_ptr<std::vector<dReal> > pqResult(new std::vector<dReal> );


    vector<dReal> vPrevValues;
    robot->GetActiveDOFValues(vPrevValues);

    std::vector<dReal> ikparams;

    Transform starthandTr = handTr;

    int i;
    for (i = 0; i <= numsteps;  i++) {
        handTr.trans = starthandTr.trans + ((dReal)i)/((dReal) numsteps)*(targTr.trans - starthandTr.trans);
        handTr.rot = quatSlerp(starthandTr.rot, targTr.rot, ((dReal)i)/((dReal) numsteps));
        //assemble parameters for passing to general IK solver
        ikparams.clear();
        ikparams.push_back(1); //number of manipulators to do IK for
        ikparams.push_back(robot->GetActiveManipulatorIndex()); //index of this manipulator
        ikparams.push_back(handTr.rot.x); ikparams.push_back(handTr.rot.y); ikparams.push_back(handTr.rot.z); ikparams.push_back(handTr.rot.w);
        ikparams.push_back(handTr.trans.x); ikparams.push_back(handTr.trans.y); ikparams.push_back(handTr.trans.z);
        ikparams.push_back(0); //don't do anything about COG
        ikparams.push_back(0); //select mode
        ikparams.push_back(0); //do rotation



        if( !_pGeneralIKSolver->Solve(IkParameterization(), curconfig, ikparams, false, pqResult)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }
        curconfig = *pqResult.get();



        size_t j = 0;
        for(; j < curconfig.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,curconfig[j],vPrevValues[j]);
            if(fabsf(curconfig[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < curconfig.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }

        ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());
        //ptraj->AddPoint(point);
        robot->SetActiveDOFValues(curconfig);
        //Transform gottm = robot->GetActiveManipulator()->GetEndEffectorTransform();
        //RAVELOG_INFO("requested: %f %f %f got: %f %f %f\n",handTr.trans.x,handTr.trans.y, handTr.trans.z,gottm.trans.x,gottm.trans.y, gottm.trans.z);

        if(GetEnv()->CheckCollision(KinBodyConstPtr(robot))) {
            RAVELOG_DEBUG("EE Interpolation broke due to collision\n");
            break;
        }

        vPrevValues = curconfig;
    }

    RAVELOG_INFO("Arm moved: %d/%d steps\n", i, numsteps);

    if(bExec)
    {
        SetTrajectory(ptraj);
    }
    else
    {
        //TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
        //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,1,"LinearTrajectoryRetimer");
#else
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");
#endif
        //pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), true, false);
        ofstream outfile(filename.c_str(), ios::out);
        ptraj->serialize(outfile);
        //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        outfile.close();
        chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
    }

    robot->SetActiveDOFs(temp_indices);
    std::vector<dReal> lastconfig;
    //don't return anything if there is only one (or zero) points
    //if(ptraj->GetPoints().size() > 1)
    //    lastconfig = ptraj->GetPoints()[ptraj->GetPoints().size()-1].q;
    if(ptraj->GetNumWaypoints() > 1)
        ptraj->GetWaypoint(ptraj->GetNumWaypoints()-1,lastconfig);

    return lastconfig;
}

vector<dReal> ManipulationProblem::GetWristPoints(istream& cmd)
{
    RAVELOG_INFO("Starting GetWristPoints...\n");



    string filename;

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "filename") == 0 ) {
            cmd >> filename;//
        }

        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }

    }
    RAVELOG_DEBUG("ManipulationProblem: reading trajectory: %s\n", p.c_str());
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    std::vector<dReal> eeposes;
    ifstream infile(filename.c_str(),ios::in);
    if( ptraj->deserialize(infile) ) {
        //FOREACH(itpoint, ptraj->GetPoints()){

        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);

        for(int i =0; i < pfulltraj->GetNumWaypoints(); i++)
        {
            std::vector<dReal> vtemp;
            pfulltraj->GetWaypoint(i,vtemp);
            robot->SetJointValues(vtemp);
            Vector eepos = robot->GetActiveManipulator()->GetEndEffectorTransform().trans;
            eeposes.push_back(eepos.x);            
            eeposes.push_back(eepos.y);
            eeposes.push_back(eepos.z);
        }
        infile.close();
    }
    else
    {
        RAVELOG_DEBUG("ManipulationProblem: failed to find trajectory\n");
        infile.close();
    }

    return eeposes;
}

int ManipulationProblem::LiftArm(istream& cmd)
{
    RAVELOG_INFO("Starting LiftArm...\n");
    dReal step_size = 0.003f;
    Vector direction = Vector(0,1,0);
    
    int minsteps = 10;
    int maxsteps = 60;



    KinBodyPtr ptarget;

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "target") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "minsteps") == 0 ) {
            cmd >> minsteps;

        }
        else if( stricmp(p.c_str(), "maxsteps") == 0 ) {
            cmd >> maxsteps;

        }
        else if( stricmp(p.c_str(), "direction") == 0 ) {
            cmd >> direction.x;
            cmd >> direction.y;
            cmd >> direction.z;
        }

        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
    }

    if( ptarget != NULL )
        robot->Grab(ptarget);

    std::vector<int> temp_indices = robot->GetActiveDOFIndices();
    
    robot->RegrabAll();
    RobotBase::RobotStateSaver saver(robot);
    
    Transform handTr = robot->GetActiveManipulator()->GetEndEffector()->GetTransform();
    
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());
    //Trajectory::TPOINT point;
    std::vector<dReal> curconfig;
    vector<dReal> vPrevValues;
    robot->GetActiveDOFValues(vPrevValues);

    int i;
    for (i = 0; i < maxsteps;  i++) {
        handTr.trans = handTr.trans + step_size*direction;
        if( !pmanip->FindIKSolution(handTr,curconfig,false)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }

        size_t j = 0;
        for(; j < curconfig.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,curconfig[j],vPrevValues[j]);
            if(fabsf(curconfig[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < curconfig.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }
        
        //ptraj->AddPoint(point);
        ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());
        robot->SetActiveDOFValues(curconfig);
        if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot)) && i > minsteps) {
            RAVELOG_DEBUG("Arm Lifting: broke due to collision\n");
            break;
        }

        vPrevValues = curconfig;
    }

    RAVELOG_INFO("Arm Lifting: %d steps\n", i);
    SetTrajectory(ptraj);
    robot->SetActiveDOFs(temp_indices);

    return 1;
}


int ManipulationProblem::JMoveHandStraight(istream& cmd)
{
    RAVELOG_DEBUG("Starting JMoveHandStraight...\n");

    int maxiterations = 20;

    TransformMatrix handTm;
    Vector direction;


    string trajfilename;

    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;
    
    vector<dReal> vOldValues;
    robot->GetDOFValues(vOldValues);
    

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "direction") == 0 ){
            cmd >> direction.x;
            cmd >> direction.y;
            cmd >> direction.z;
        }
        else if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;
            cmd >> handTm.trans.y;
            cmd >> handTm.trans.z;
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];
            cmd >> handTm.m[4];
            cmd >> handTm.m[8];
            cmd >> handTm.m[1];
            cmd >> handTm.m[5];
            cmd >> handTm.m[9];
            cmd >> handTm.m[2];
            cmd >> handTm.m[6];
            cmd >> handTm.m[10];
        }
        else if( stricmp(p.c_str(), "writetraj") == 0 ) {
            cmd >> trajfilename;
        }
        else if( stricmp(p.c_str(),"maxdist") == 0) {
            dReal maxdist;
            cmd >> maxdist;
            maxiterations = (int)(maxdist/0.001) + 1;
            RAVELOG_INFO("maxdis: %d\n",maxiterations);       
        }
        else if( stricmp(p.c_str(),"handvals") == 0 ) {
            vector<dReal> vhandvals(pmanip->GetGripperIndices().size());
            
            for(size_t i = 0; i < vhandvals.size(); ++i) {
                cmd >> vhandvals[i];
            }

            robot->SetActiveDOFs(pmanip->GetGripperIndices());
            robot->SetActiveDOFValues(vhandvals);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
        


    }

    Transform handTr(handTm);

    vector<dReal> qResultold;
    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    robot->GetActiveDOFValues(qResultold);

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());
    //Trajectory::TPOINT point, goodpoint;
    std::vector<dReal> curconfig, goodconfig;
    robot->GetActiveDOFValues(curconfig);
    ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());
    //ptraj->AddPoint(point;

    goodconfig = curconfig;
    Transform tempTr;
    
    dReal xyzstep = 0.0025;
    Vector xyz = direction;


    RAVELOG_INFO("xyz vel: %f %f %f\n",xyz.x,xyz.y,xyz.z);
    std::vector<dReal> vlowerlimits, vupperlimits;
    robot->GetDOFLimits(vlowerlimits,vupperlimits);
    vector<dReal> viksol(pmanip->GetArmIndices().size(),0);
    vector<dReal> vfulljoints(robot->GetDOF());
    robot->GetDOFValues(vfulljoints);
    int numdofs = robot->GetDOF();


    vector<dReal> J(numdofs*3);
    vector<dReal> Jplus(numdofs*3); //Jplus = JtJJt  is (nx3)


    vector<dReal> W(numdofs*numdofs); //nxn
    vector<dReal> JplusJ(numdofs*numdofs); //nxn
    vector<dReal> NullSpace(numdofs*numdofs); //nxn

    vector<dReal> vtheta_avg(numdofs);
    vector<dReal> vnullspace_jointvals(numdofs);
    
    //compute nullspace joint weighting matrix
    for(int i = 0; i < numdofs; i++)
    {
        vtheta_avg[i] = (vupperlimits[i] + vlowerlimits[i])/2;
        W[i*numdofs + i] = 1/((vupperlimits[i] - vlowerlimits[i])*(vupperlimits[i] - vlowerlimits[i]));
    }
    //W[4*numdofs + 4] = 0;
    //W[5*numdofs + 5] = 0;



    int iter;
    for(iter = 0; iter < maxiterations; iter++)
    {
        bool bgotthere = true;
        memset(&J[0], 0, sizeof(dReal)*J.size());
        dReal JJt[9], invJJt[9];
        const dReal flambda = 1e-4f;
        std::vector<dReal> vtempjoints = vfulljoints;

        std::vector<dReal> vbackupjoints = vfulljoints;

        Transform Tgoal = pmanip->GetEndEffectorTransform();
        Tgoal.trans += xyz*xyzstep;
        Transform Ttarg_backup = Tgoal;

        //Ttarg.trans += xyz*xyzstep;

        // descend on the jacobian
        int numsteps = 0;
        dReal preverror = 10000;


        //RAVELOG_INFO("handtr in : %.3f %.3f %.3f   %.3f %.3f %.3f %.3f\n", handTr.trans.x, handTr.trans.y, handTr.trans.z, handTr.rot.w,handTr.rot.x,handTr.rot.y, handTr.rot.z);

        while(1) {
            // get the translation jacobian
            Transform tEE = pmanip->GetEndEffectorTransform();

            
            if( (Tgoal.trans-tEE.trans).lengthsqr3() <= 0.002*0.002 )
            {
                RAVELOG_INFO("Goal reached.\n");
                break;
            }
            robot->CalculateJacobian(pmanip->GetEndEffector()->GetIndex(), tEE.trans, J);
                


            OpenRAVE::mathextra::multtrans_to2<dReal, dReal, dReal>(&J[0], &J[0], 3, numdofs, 3, JJt, false);

            //power through singularities
            JJt[0] += flambda*flambda;
            JJt[4] += flambda*flambda;
            JJt[8] += flambda*flambda;


            //stop on singularities
            //if (fabs(JJt[0]) < 1e-7 || fabs(JJt[4]) < 1e-7 || fabs(JJt[8]) < 1e-7)
            //{
            //    vfulljoints = vbackupjoints;
            //    robot->SetJointValues(NULL,NULL,&vfulljoints[0]);
            //    break;
            //}
            dReal det = 0;
            inv3(JJt, invJJt, &det, 3);
            RAVELOG_DEBUG("det: %f\n",det);
            

            //nullspace projection to avoid joint limits
            ///////////////////////////////////////////////
            //getJplus
            OpenRAVE::mathextra::multtrans<dReal, dReal, dReal>(&J[0], invJJt, 3, numdofs, 3, &Jplus[0], false);
            
            //get JplusJ
            OpenRAVE::mathextra::mult<dReal, dReal, dReal>(&Jplus[0], &J[0], numdofs, 3, numdofs, &JplusJ[0],false);
            
            //(I-JplusJ)
            for(int i = 0; i < numdofs; i++)
                for(int j = 0; j < numdofs;j++)
                {
                    if(i == j)
                        JplusJ[i*numdofs+j] = 1 - JplusJ[i*numdofs+j];
                    else
                        JplusJ[i*numdofs+j] = -JplusJ[i*numdofs+j];
                }
            //multiply (I-JplusJ) by W
            OpenRAVE::mathextra::mult<dReal, dReal, dReal>(&JplusJ[0], &W[0], numdofs, numdofs, numdofs, &NullSpace[0],false);



            //compute nullspace contribution
            dReal alpha = 0.01;
            for(int i = 0; i < numdofs; i++)
            {
                dReal temp = 0;
                for(int j = 0; j < numdofs; j++)
                    temp += NullSpace[i*numdofs + j]*(vtheta_avg[i] - vfulljoints[i]);

                vnullspace_jointvals[i] = alpha*temp;    
            }

            //stringstream s;
            //s << L"Nullspace Contribution: "<<endl;
            //for(int i = 0; i < numdofs; i++)
            //{            
            //    s << vnullspace_jointvals[i] << " ";
            //    s << endl;
            //}   
            stringstream s;
            s << L"Nullspace Contribution: ";
            for(int i = 0; i < 3;i++)
            {
                dReal temp=0;
                for(int j=0; j < numdofs; j++)
                {    
                    temp += J[i*numdofs + j]*vnullspace_jointvals[j];
                }
                s << temp << " " ;
            }
            RAVELOG_DEBUG(s.str().c_str());

            ///////////////////////////////////////////////



            //preserve manipulability
            //if(det < 0.001)
            //{             
            //    RAVELOG_INFO("Manipulability too low, breaking out of gradient descent\n");       
            //    vfulljoints = vbackupjoints;
            //    robot->SetJointValues(NULL,NULL,&vfulljoints[0]);
            //    break;
            //}    


            Vector e = Tgoal.trans - tEE.trans, v;
            dReal flength = sqrtf(e.lengthsqr3());

            if(flength > (preverror + 0.01))
            {                    
                RAVELOG_INFO("Error Increasing, breaking out of gradient descent\n");
                vfulljoints = vbackupjoints;
                robot->SetJointValues(vfulljoints);
                break;
            }
            else
                preverror = flength;

            RAVELOG_INFO("Position Error: %f\n",flength);

            // take constant steps if length is big
            if( flength > 0.01 )
                e = e * (0.01 / flength);
            
            /*OpenRAVE::mathextra::transnorm3(v, invJJt, e);*/
            dReal temp3_v[3] = {v.x, v.y, v.z};
            dReal temp3_e[3] = {e.x, e.y, e.z};
            OpenRAVE::mathextra::transnorm3(temp3_v, invJJt, temp3_e);

            stringstream s2;
            s2 << L"J: ";
            for(int i = 0; i < numdofs*3;i++)
            {
                s2 << J[i] << " " ;
            }
            s2 << "\n";
            RAVELOG_DEBUG(s2.str().c_str());
            
            RAVELOG_INFO("v: %f %f %f\n",v.x,v.y,v.z);

            dReal f = 0;
            bool bsuccess = true;
            for(int i = 0; i < numdofs; ++i) {
                dReal fdir = J[0*numdofs+i] * v.x + J[1*numdofs+i] * v.y + J[2*numdofs+i] * v.z;
                //fdir = fdir + vnullspace_jointvals[i];
                //if there is a pop or joint limit hit, stop

                if(fdir > 0.3 || fdir < -0.3)
                {
                    RAVELOG_DEBUG("Joint Change too big: %f\n", fdir);
                    bsuccess = false;
                    break;
                }

                if( vtempjoints[i] + fdir < vlowerlimits[i] || vtempjoints[i] + fdir > vupperlimits[i] ) 
                {
                    RAVELOG_DEBUG("J%d at limit %f\n",i,vtempjoints[i] + fdir);
                //    bsuccess = false;
                //    break;
                }
                
                RAVELOG_DEBUG("fdir: %f\n",fdir);
                vtempjoints[i] = vtempjoints[i] + fdir;

                f += fdir * fdir;
            }
            
            if(!bsuccess)
            {
                bgotthere = false;
                break;
            }
            vfulljoints = vtempjoints;
            robot->SetJointValues(vfulljoints);

            if(GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || robot->CheckSelfCollision())
            {
                RAVELOG_INFO("Collision while executing JMoveHandStraight\n");
                bgotthere = false;
                break;
            }
            // gradient descend won't move it anymore
            if( f < 1e-7 ) 
            {
                bgotthere = false;
                break;
            }

            //stuck, so back up to get unstuck
            if(numsteps++ == 25)
            {
                RAVELOG_INFO("Stuck\n");
                //vfulljoints = vbackupjoints;
                //robot->SetJointValues(NULL,NULL,&vfulljoints[0]);
                //if(pIKSphere != NULL) pIKSphere->SetTransform(Ttarg_backup);
                bgotthere = false;
                break;
            }
        }
        if(bgotthere)
        {
            for(int x = 0; x < pmanip->GetArmIndices().size(); x++)
                goodconfig[x] = vfulljoints[pmanip->GetArmIndices()[x]];
        }
        else
            break;

    }
    

    // restore robot joint values
    robot->SetJointValues(vOldValues);

    if( iter > 0 ) {
        ptraj->Insert(ptraj->GetNumWaypoints(),goodconfig,robot->GetActiveConfigurationSpecification());
        //ptraj->AddPoint(goodpoint);
        //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,1,"LinearTrajectoryRetimer");
#else
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");
#endif
        robot->GetController()->SetPath(ptraj);
        RAVELOG_INFO("moving hand straight: %d\n", iter);     
 
        if( trajfilename != "" ) {
            RAVELOG_DEBUG("Writing solved move-hand-straight trajectory to file %s\n", trajfilename.c_str());
            //TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
            //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream outfile(trajfilename.c_str(), ios::out);
            //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            ptraj->serialize(outfile);
            outfile.close();
            chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
            //delete pfulltraj;
        }
        
    }
    else RAVELOG_INFO("move hand straight didn't move\n");

    return 0;
}



int ManipulationProblem::MoveHandStraight(istream& cmd)
{
    RAVELOG_DEBUG("Starting MoveHandStraight...\n");

    int maxiterations = 20;

    TransformMatrix handTm;
    Vector direction;
    const char* delim = " \r\n\t";

    string trajfilename;

    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;
    
    vector<dReal> vOldValues;
    robot->GetDOFValues(vOldValues);
    

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "direction") == 0 ){
            cmd >> direction.x;
            cmd >> direction.y;
            cmd >> direction.z;
        }
        else if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;
            cmd >> handTm.trans.y;
            cmd >> handTm.trans.z;
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];
            cmd >> handTm.m[4];
            cmd >> handTm.m[8];
            cmd >> handTm.m[1];
            cmd >> handTm.m[5];
            cmd >> handTm.m[9];
            cmd >> handTm.m[2];
            cmd >> handTm.m[6];
            cmd >> handTm.m[10];
        }
        else if( stricmp(p.c_str(), "writetraj") == 0 ) {
            RAVELOG_INFO("writing trajectory to file\n");
            cmd >> trajfilename;
        }
        else if( stricmp(p.c_str(),"maxdist") == 0) {
            dReal maxdist;
            cmd >> maxdist;
            maxiterations = (int)(maxdist/0.001) + 1;
            RAVELOG_INFO("maxdis: %d\n",maxiterations);       
        }
        else if( stricmp(p.c_str(),"handvals") == 0 ) {
            vector<dReal> vhandvals(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vhandvals.size(); ++i) {
                cmd >> vhandvals[i];
            }

            robot->SetActiveDOFs(pmanip->GetGripperIndices());
            robot->SetActiveDOFValues(vhandvals);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }



    }

    Transform handTr(handTm);

    vector<dReal> qResultold;
    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    robot->GetActiveDOFValues(qResultold);

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());
    //Trajectory::TPOINT point, goodpoint;
    std::vector<dReal> curconfig, goodconfig;
    robot->GetActiveDOFValues(curconfig);
    ptraj->Insert(ptraj->GetNumWaypoints(),curconfig,robot->GetActiveConfigurationSpecification());
    //ptraj->AddPoint(point);

    goodconfig = curconfig;
    int iter = 0;
    Transform tempTr;
    
    while(iter < maxiterations) {
        handTr.trans = handTr.trans + 0.001f*direction;
        //RAVELOG_INFO("handtr in : %.3f %.3f %.3f   %.3f %.3f %.3f %.3f\n", handTr.trans.x, handTr.trans.y, handTr.trans.z, handTr.rot.w,handTr.rot.x,handTr.rot.y, handTr.rot.z);
        if( !pmanip->FindIKSolution(handTr, curconfig, true)  )
        {
            RAVELOG_INFO("\nMoveHandStraight - No IK solution found\n");     
            break;
        }
        else
        {
            bool badik = false;
            for(size_t i = 0; i < curconfig.size(); i++)
            {
                float thresh = i == 6 ? 0.5f : 0.25f;
                //s << qResult[i] << " ";  
                if(fabsf(qResultold[i] - curconfig[i]) > thresh)
                    badik = true;
            }
            //s << endl;
            //RAVEPRINT(s.str().c_str());
            
            if(badik) {
                RAVELOG_INFO("MoveHandStraight: bad ik detected\n");
                stringstream s;
                s << "old: ";
                FOREACH(it, qResultold) s << *it << " ";
                s << endl << "new: ";
                FOREACH(it, curconfig) s << *it << " ";
                s << endl;
                RAVELOG_INFO(s.str());
                break;
            }

        }
        
        robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
        robot->SetActiveDOFValues(curconfig);
        //tempTr = pmanip->GetEndEffectorTransform();
        
        if(GetEnv()->CheckCollision(KinBodyConstPtr(robot)))
            break;

        qResultold = curconfig;
        goodconfig = curconfig;
        ++iter;
    }

    // restore robot joint values
    robot->SetJointValues(vOldValues);

    if( iter > 0 ) {
        ptraj->Insert(ptraj->GetNumWaypoints(),goodconfig,robot->GetActiveConfigurationSpecification());
        //ptraj->AddPoint(goodpoint);
        //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,1,"LinearTrajectoryRetimer");
#else
        OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");
#endif
        //robot->SetActiveMotion(ptraj);
        robot->GetController()->SetPath(ptraj);
        RAVELOG_INFO("moving hand straight: %d\n", iter); 

        if( trajfilename != "" ) {
            RAVELOG_INFO("Writing solved move-hand-straight trajectory to file\n");
            //TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
            //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream outfile(trajfilename.c_str(), ios::out);
            //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            ptraj->serialize(outfile);
            outfile.close();
            chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

        }


    }
    else RAVELOG_INFO("move hand straight didn't move\n");
    return 0;
}


         
std::vector<int> ManipulationProblem::MultiHandClose(istream& cmd)
{
    RAVELOG_DEBUG("Starting MultiHandClose...\n");

    std::vector<int> out;


    int numtms;
    std::vector<Transform> vtransforms;

    RobotBasePtr probot;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    string targetname;


    TransformMatrix handTm;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "robot") == 0 ) {
            cmd >> p;
            probot = GetEnv()->GetRobot(p);

        }
        else if( stricmp(p.c_str(), "targetname") == 0 ) {
            cmd >> targetname;
        }
        else if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;
            for(int i = 0; i < numexcluded; i++)
            {
                cmd >> p;
                ptemp = GetEnv()->GetKinBody(p);
                if(ptemp != NULL)
                    vexcluded.push_back(KinBodyConstPtr(ptemp));
                else
                {
                    RAVELOG_INFO("ERROR: Unknown excluded body\n");
                    return out;
                }
            }
        }
        else if( stricmp(p.c_str(), "transforms") == 0 ) {
            cmd >> numtms;
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> vtransforms[i].rot.x;
                cmd >> vtransforms[i].rot.y;
                cmd >> vtransforms[i].rot.z;
                cmd >> vtransforms[i].rot.w;

                cmd >> vtransforms[i].trans.x;
                cmd >> vtransforms[i].trans.y;
                cmd >> vtransforms[i].trans.z;
            }

        }
        else if( stricmp(p.c_str(), "transforms12") == 0 ) {
            cmd >> numtms;
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> handTm.m[0];
                cmd >> handTm.m[4];
                cmd >> handTm.m[8];
                cmd >> handTm.m[1];
                cmd >> handTm.m[5];
                cmd >> handTm.m[9];
                cmd >> handTm.m[2];
                cmd >> handTm.m[6];
                cmd >> handTm.m[10];
                cmd >> handTm.trans.x;
                cmd >> handTm.trans.y;
                cmd >> handTm.trans.z;
                vtransforms[i] = (Transform) handTm;
            }

        }

        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return std::vector<int>();
        }


        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return std::vector<int>();
        }

    }

    if(probot.get() == NULL)
    {
        RAVELOG_INFO("ERROR: Unknown robot\n");
        return out;
    }

    Transform tbackup = probot->GetTransform();
    int col = 0;
    out.resize(numtms);

    CollisionReportPtr preport(new CollisionReport());

    const RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();

    if( pmanip == NULL )
    {
        RAVELOG_FATAL("No manipulator for this robot!\n");
        return  out;
    }

    probot->SetActiveDOFs(pmanip->GetGripperIndices());
    std:vector<dReal> preshape;
    probot->GetActiveDOFValues(preshape);


    TrajectoryBasePtr ptraj2 = RaveCreateTrajectory(GetEnv(),"");
    ptraj2->Init(robot->GetActiveConfigurationSpecification());
    //NOTE: YOU NEED GRASPER2 FOR THIS, NOT THE GRASPER THAT COMES WITH OPENRAVE
    PlannerBasePtr graspplanner = RaveCreatePlanner(GetEnv(),"Grasper2");
    if( graspplanner != NULL )
        RAVELOG_DEBUG("grasping planner created!\n");
    else
    {
        RAVELOG_FATAL("Error: Can not find grasper2, terminating!\n");
        return out;
    }

    boost::shared_ptr<PlannerBase::PlannerParameters> params2;
    params2.reset(new PlannerBase::PlannerParameters());

    params2->vinitialconfig.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(params2->vinitialconfig);

//    std::vector<dReal> jvals;

    for(int i = 0; i < vtransforms.size(); i++)
    {
        probot->SetTransform(vtransforms[i]);
        probot->SetActiveDOFValues(preshape);
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
        if(numexcluded)
            col = GetEnv()->CheckCollision(KinBodyConstPtr(probot),vexcluded,vlinkexcluded,preport);
        else
            col = GetEnv()->CheckCollision(KinBodyConstPtr(probot));

        if(col)
            out[i] = -1;
        else
        {
            if( !graspplanner->InitPlan(robot, params2) ) {
                RAVELOG_FATAL("InitPlan failed\n");
                return out;
            }

            graspplanner->PlanPath(ptraj2);
            if(ptraj2->GetNumWaypoints() == 0)
            {
                RAVELOG_FATAL("Trajectory from grasper is empty!\n");
                return out;
            }
//            Trajectory::TPOINT p = ptraj2->GetPoints().back();
//            probot->SetActiveDOFValues(p.q);
//            probot->GetJointValues(jvals);
//            probot->GetController()->SetDesired(jvals);

            GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
            vector<KinBody::LinkPtr>::const_iterator itlink;
            FOREACHC(itlink, robot->GetLinks()) {
                if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), preport) ) {
                    RAVELOG_DEBUG(str(boost::format("contact %s:%s with %s:%s\n")%preport->plink1->GetParent()->GetName()%preport->plink1->GetName()%preport->plink2->GetParent()->GetName()%preport->plink2->GetName()));

                    if(stricmp(preport->plink2->GetParent()->GetName().c_str(), targetname.c_str()) == 0 )
                    {
                        RAVELOG_DEBUG("OK Contact\n");

                    }
                    else
                    {
                        RAVELOG_DEBUG("Bad Contact\n");
                        out[i] = 1;
                        break;
                    }
                }
            }
            GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
        }
        RAVELOG_DEBUG("out[i]: %d\n",out[i]);
    }

    //probot->SetTransform(tbackup);
    return out;
}

         
int ManipulationProblem::SwitchModelsInternal(vector<pair<string, string> >& vpatterns, bool tofat)
{
    string strcmd;
    boost::regex re;
    char str[128];
    char strname[128];

    vector<KinBodyPtr>::const_iterator itbody, itbody2;
    vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    FORIT(itbody, vbodies) {
        
        FOREACH(itpattern, vpatterns) {
            try {
                re.assign(itpattern->first, boost::regex_constants::icase);
            }
            catch (boost::regex_error& e) {
                RAVELOG_INFO("error\n");
                    cout << itpattern->first << " is not a valid regular expression: \""
                         << e.what() << "\"" << endl;
                continue;
            }
            
            // convert
            sprintf(str, "%s", (*itbody)->GetName().c_str());
            if( boost::regex_match(str, re) ) {
                
                // check if already created
                bool bCreated = false;
                strcpy(strname, (*itbody)->GetName().c_str());
                strcat(strname, "thin");

                FORIT(itbody2, vbodies) {
                    if( strcmp((*itbody2)->GetName().c_str(), strname) == 0 ) {
                        bCreated = true;
                        break;
                    }
                }
                
                if( !bCreated ) {
                    strcpy(strname, (*itbody)->GetName().c_str());
                    strcat(strname, "fat");

                    FORIT(itbody2, vbodies) {
                        if( strcmp((*itbody2)->GetName().c_str(), strname) == 0 ) {
                            bCreated = true;
                            break;
                        }
                    }
                }

                if( tofat && !bCreated ) {
                    
                    RAVELOG_DEBUG("creating %s\n", strname);
                    
                    // create fat body
                    KinBodyPtr pfatbody = RaveCreateKinBody(GetEnv());

                    pfatbody = GetEnv()->ReadKinBodyXMLFile(itpattern->second.c_str());   

                    if( pfatbody.get() == NULL) {
                        RAVELOG_INFO("failed to open file: %s\n", itpattern->second.c_str());
                        continue;
                    }
                    pfatbody->SetName(strname); // should be %Sfat
                    GetEnv()->AddKinBody(pfatbody,true);

                    pfatbody->SetTransform(Transform(Vector(1,0,0,0),Vector(0,100,0)));
                }
                
                if( SwitchModel((*itbody)->GetName(), tofat) != 0 ) {
                    RAVELOG_INFO("SwitchModel failed\n");
                }
                break;
            }
        }
    }

    return -1;
}

int ManipulationProblem::SwitchModels(istream& cmd)
{
    vector<KinBodyPtr> vbodies;
    vector<bool> vpadded;




    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "name") == 0 ) {
            cmd >> p;
            vbodies.push_back(GetEnv()->GetKinBody(p));
        }
        else if( stricmp(p.c_str(), "padded") == 0 ) {
            int temp;
            cmd >> temp;
            vpadded.push_back(temp>0);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }



    }

    if(vbodies.size() != vpadded.size() ) {    
        RAVELOG_INFO("ERROR in ManipulationProblem::SwitchModels - Number of models must equal number of padding states.\n");
        return -1;
    }
    
    for(size_t i = 0; i < vbodies.size(); i++) {
        if( SwitchModel(vbodies[i]->GetName(), vpadded[i]) != 0 ) {
            RAVELOG_INFO("error switching: %s\n", vbodies[i]->GetName().c_str());
        }
    }

    return 0;
}

int ManipulationProblem::SwitchModel(const string& bodyname, bool bToFatModel)
{
    KinBodyPtr pbody = GetEnv()->GetKinBody(bodyname);
    if( pbody == NULL )
        return -1;

    string oldname, newname;

    if( bToFatModel ) {
        oldname = bodyname + "fat";
        newname = bodyname + "thin";
    }
    else {
        oldname = bodyname + "thin";
        newname = bodyname + "fat";
    }

    KinBodyPtr pswitch = GetEnv()->GetKinBody(oldname);
    if( pswitch == NULL ) {
        RAVELOG_DEBUG("Model %S doesn't need switching\n",bodyname.c_str());
        return 0;
    }

    bool bShouldLock = false;
    bool bEnabled = false;
    int nGlobalId = 0;

    Transform tinit = pbody->GetTransform(); 
    vector<dReal> vjoints; pbody->GetDOFValues(vjoints);
    
    Transform tprevtrans = tinit;

    
    pswitch->SetName(bodyname.c_str());
    pswitch->SetTransform(tinit);
    if( vjoints.size() > 0 )
        pswitch->SetJointValues(vjoints);
    
    pbody->SetName(newname.c_str());
    Transform temp; temp.trans.y = 100.0f;
    pbody->SetTransform(temp);
    

    return 0;
}

int ManipulationProblem::MoveToHandPosition(istream& cmd)
{
    RAVELOG_INFO("Starting MoveToHandPosition...\n");

    TransformMatrix handTm;
    vector<dReal> values, q1, vhandjointvals;
  
    robot->GetDOFValues(values);
    
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    string trajfilename;

    KinBodyPtr ptarget;


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;
            cmd >> handTm.trans.y;
            cmd >> handTm.trans.z;
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];
            cmd >> handTm.m[4];
            cmd >> handTm.m[8];
            cmd >> handTm.m[1];
            cmd >> handTm.m[5];
            cmd >> handTm.m[9];
            cmd >> handTm.m[2];
            cmd >> handTm.m[6];
            cmd >> handTm.m[10];
        }
        else if( stricmp(p.c_str(), "writetraj") == 0 ) {
            cmd >> trajfilename;
        }
        else if( stricmp(p.c_str(), "grab") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "handjoints") == 0 ) {
            if( pmanip.get() != NULL )
                vhandjointvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vhandjointvals.size(); ++i)
                cmd >> vhandjointvals[i];
        }
        else if( stricmp(p.c_str(), "setjoint") == 0 ) {
            int ind;// = atoi(pindex);
            cmd >> ind;
            if( ind >= 0 && ind < (int)values.size() ) {
                cmd >> values[ind];// = (dReal)atof(pvalue);
            }
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }

    if( pmanip.get() == NULL )
        return -1;

    if (ptarget.get() != NULL)
        robot->Grab(ptarget);
    

    Transform handTr(handTm);
    for(size_t i = 0; i < vhandjointvals.size(); i++) {
        values[pmanip->GetGripperIndices()[i]] = vhandjointvals[i];
    }

    std::vector<dReal> armgoals;
    std::vector<int> activejoints;

    robot->RegrabAll();

    q1.resize(pmanip->GetArmIndices().size());
    robot->SetJointValues(values);
    if( !pmanip->FindIKSolution(handTr, q1, true) )
    {
        RAVELOG_INFO("No IK Solution found.\n");     
        return -1;
    }
    else
    {
        stringstream s;
        s << L"ik sol: ";
        for(size_t i = 0; i < q1.size(); i++)
            s << q1[i] << " ";
        s << endl;
        RAVELOG_INFO(s.str());

    }

    for(size_t i = 0; i < pmanip->GetArmIndices().size(); i++) {
        activejoints.push_back(pmanip->GetArmIndices()[i]);
        armgoals.push_back(q1[i]);
    }

    robot->SetActiveDOFs(activejoints);

    // test collisions for the goal
    {
        RobotBase::RobotStateSaver saver(robot);
        robot->SetActiveDOFValues(armgoals);
        if( GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
            return -1;
        }
    }
    
    std::vector<dReal> activegoalconfig = armgoals; 
    if( activegoalconfig.size() != activejoints.size() ) {
        RAVELOG_INFO("Active joints unequal size\n");
        return -1;
    }

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());

    PlannerBase::PlannerParametersPtr params;
    
    robot->SetActiveDOFs(activejoints);
    std::vector<dReal> pzero(robot->GetActiveDOF());
    robot->GetActiveDOFValues(pzero);

    // make sure the initial and goal configs are not in collision
    robot->SetActiveDOFValues(activegoalconfig, true);


    
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_INFO("jitter failed for goal\n");
        robot->SetActiveDOFs(activejoints);
        return -1;
    }

    // restore
    robot->SetActiveDOFs(activejoints);

    robot->GetActiveDOFValues(params->vgoalconfig);
    robot->SetActiveDOFValues(pzero);

    //Trajectory::TPOINT pt;
    //pt.q = pzero;
    //ptraj->AddPoint(pt);
    ptraj->Insert(ptraj->GetNumWaypoints(),pzero,robot->GetActiveConfigurationSpecification());

    // jitter again for initial collision
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_INFO("jitter failed for initial\n");
        return -1;
    }

    if (!_reusePlanner){
        _pPlanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( _pPlanner == NULL ) {
            RAVELOG_INFO("failed to create planner\n");
            return -1;
        }
    }

    robot->GetActiveDOFValues(params->vinitialconfig);

    if(ptraj->GetNumWaypoints() > 0)
        ptraj->Remove(0,ptraj->GetNumWaypoints()-1);
    //ptraj->Clear();
    
    params->_nMaxIterations = 4000;
    bool bSuccess = false;
    u32 startplan = timeGetTime();
    RAVELOG_INFO("starting planning\n");

    for(int iter = 0; iter < 3; ++iter) {
        if( !_pPlanner->InitPlan(robot, params) ) {
            RAVELOG_DEBUG("InitPlan failed\n");
            //delete _pPlanner;
            //_pPlanner = NULL;
            return -1;
        }
        
        if( _pPlanner->PlanPath(ptraj) ) {
            bSuccess = true;
            RAVELOG_INFO("finished planning %dms\n", timeGetTime()-startplan);
            break;
        }
        else RAVELOG_INFO("PlanPath failed\n");
    }
    

    if( !bSuccess ) {
        return -1;
    }
    
    if( trajfilename != "" ) {
        //TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
        //robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        ofstream outfile(trajfilename.c_str(), ios::out);
        //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        ptraj->serialize(outfile);
        outfile.close();
        chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

    }
    SetTrajectory(ptraj);
    return 0;
}


TrajectoryBasePtr ManipulationProblem::MoveArm(const vector<int>& activejoints, const std::vector<dReal>& activegoalconfig, const RobotBase::ManipulatorPtr manip, KinBodyPtr ptarget)
{
    RAVELOG_DEBUG("Starting MoveArm...\n");

    if( activegoalconfig.size() != activejoints.size() ) {
        RAVELOG_DEBUG("Active joints unequal size\n");
        return TrajectoryBasePtr();
    }

    if( activejoints.size() == 0 ) {
        RAVELOG_DEBUG("move arm failed\n");
        return TrajectoryBasePtr();
    }

    if( GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
        RAVELOG_DEBUG("Hand in collision\n");
    }
    
    PlannerBase::PlannerParametersPtr params;
    
    robot->SetActiveDOFs(activejoints);
    std::vector<dReal> pzero(robot->GetActiveDOF());
    robot->GetActiveDOFValues(pzero);

    // make sure the initial and goal configs are not in collision
    robot->SetActiveDOFValues(activegoalconfig, true);

    // jitter only the manipulator! (jittering the hand causes big probs)
    robot->SetActiveDOFs(manip->GetGripperIndices());
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_INFO("failed\n");
        robot->SetActiveDOFs(activejoints);
        return TrajectoryBasePtr();
    }

    // restore
    robot->SetActiveDOFs(activejoints);

    robot->GetActiveDOFValues(params->vgoalconfig);
    robot->SetActiveDOFValues(pzero);
    
    // jitter again for initial collision
    if( !JitterActiveDOF(robot) )
        return TrajectoryBasePtr();

    if (!_reusePlanner){
        _pPlanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( _pPlanner == NULL ) {
            RAVELOG_INFO("failed to create planner\n");
            return TrajectoryBasePtr();
        }
    }
    
    robot->GetActiveDOFValues(params->vinitialconfig);
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
    ptraj->Init(robot->GetActiveConfigurationSpecification());

    if(ptraj->GetNumWaypoints() > 0)
        ptraj->Remove(0,ptraj->GetNumWaypoints()-1);
    //ptraj->Clear();

    params->_nMaxIterations = 4000;
    bool bSuccess = false;
    u32 startplan = timeGetTime();
    RAVELOG_INFO("starting planning\n");
    
    for(int iter = 0; iter < 3; ++iter) {
        if( !_pPlanner->InitPlan(robot, params) ) {
            RAVELOG_DEBUG("InitPlan failed\n");
            return TrajectoryBasePtr();
        }
        
        if( _pPlanner->PlanPath(ptraj) ) {
            bSuccess = true;
            //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
            OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,1,"LinearTrajectoryRetimer");
#else
            OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");
#endif
            RAVELOG_INFO("finished planning, %dms\n", timeGetTime()-startplan);
            break;
        }
        else RAVELOG_INFO("PlanPath failed\n");
    }
    

    if( !bSuccess ) {
        return TrajectoryBasePtr();
    }

    return ptraj;
}

bool ManipulationProblem::JitterActiveDOF(RobotBasePtr robot)
{
    //RAVELOG_INFO("Starting Jitter...\n");
    vector<dReal> curdof, newdof, lower, upper;
    robot->GetActiveDOFValues(curdof);
    robot->GetActiveDOFLimits(lower, upper);
    newdof = curdof;

    dReal fRand = 0.05f;
    int iter = 0;

    if(robot->CheckSelfCollision())
    {
        RAVELOG_INFO("JitterActiveDOFs: initial config in self collision!\n");
    }

    while(GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || robot->CheckSelfCollision() ) {
        if( iter > 5000 ) {
            RAVELOG_INFO("Failed to find noncolliding position for robot\n");

            robot->SetActiveDOFValues(curdof);

            // display collision report
            CollisionReportPtr report;
            report.reset(new CollisionReport());
            if( GetEnv()->CheckCollision(KinBodyConstPtr(robot), report) ) {
                if( report->plink1.get() != NULL && report->plink2.get() != NULL ) {
                    RAVELOG_DEBUG("Jitter collision %s:%s with %s:%s\n", report->plink1->GetParent()->GetName().c_str(), report->plink1->GetName().c_str(), report->plink2->GetParent()->GetName().c_str(), report->plink2->GetName().c_str());
                }
            }
            
            return false;        
        }

        for(int j = 0; j < robot->GetActiveDOF(); j++)
            newdof[j] = CLAMP_ON_RANGE(curdof[j] + fRand * (RANDOM_FLOAT()-0.5f), lower[j], upper[j]);
        
        if( (iter%1000) == 499 )
            fRand *= 2;

        robot->SetActiveDOFValues(newdof);
        ++iter;
    }
    
    return true;
}

bool ManipulationProblem::JitterTransform(KinBodyPtr pbody, float fJitter)
{
    // randomly add small offset to the body until it stops being in collision
    Transform transorig = pbody->GetTransform();
    Transform transnew = transorig;
    int iter = 0;
    while(GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {

        if( iter > 1000 ) {
            return false;
        }

        transnew.trans = transorig.trans + fJitter * Vector(RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f);
        pbody->SetTransform(transnew);
        ++iter;
    }

    return true;
}


void ManipulationProblem::SetTrajectory(TrajectoryBasePtr ptraj, float fSpeed)
{
    assert(robot != NULL);
    //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,1,"LinearTrajectoryRetimer");
#else
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, robot,false,1,"LinearTrajectoryRetimer");
#endif
    robot->GetController()->SetPath(ptraj);
}

int ManipulationProblem::GrabBody(istream& cmd)
{
    RAVELOG_DEBUG("Starting GrabBody...\n");

    KinBodyPtr ptarget;
    bool bSendToController = false;
    TransformMatrix tmEE;




    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "name") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "sendtocontroller") == 0 ) {
            bSendToController = true;
        }
        else if( stricmp(p.c_str(), "applytransform") == 0 ){
            //this transform is used to move from the RAVE end-effector frame (tEE_rave) to the controller's end-effector frame (tEE_controller)
            //it should be specified in tEE_rave coordinates
            // it is used like this: tEE_controller = tEE_rave * T_applytransform
            cmd >> tmEE.m[0];
            cmd >> tmEE.m[4];
            cmd >> tmEE.m[8];
            cmd >> tmEE.m[1];
            cmd >> tmEE.m[5];
            cmd >> tmEE.m[9];
            cmd >> tmEE.m[2];
            cmd >> tmEE.m[6];
            cmd >> tmEE.m[10];
            cmd >> tmEE.trans.x;
            cmd >> tmEE.trans.y;
            cmd >> tmEE.trans.z;
        }
        else if(stricmp(p.c_str(), "id") == 0){
            int netid;
            cmd >> netid;
            ptarget = GetEnv()->GetBodyFromEnvironmentId(netid);

        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
        


    }
    

    if(ptarget.get() == NULL)
    {
        RAVELOG_INFO("ERROR Manipulation::GrabBody - Invalid body name.\n");
        return -1;
    }

    robot->Grab(ptarget);
    
    if(bSendToController)
    {
        //RAVELOG_INFO("tEE (before): %f %f %f\n",robot->GetActiveManipulator()->GetEndEffectorTransform().trans.x,robot->GetActiveManipulator()->GetEndEffectorTransform().trans.y,robot->GetActiveManipulator()->GetEndEffectorTransform().trans.z);
        Transform tEE = robot->GetActiveManipulator()->GetEndEffectorTransform() * Transform(tmEE);
        //RAVELOG_INFO("tEE (after): %f %f %f\n",tEE.trans.x,tEE.trans.y,tEE.trans.z);
        //get mass, com, and inertia tensor in end-effector coordinates
        std::vector<KinBody::LinkPtr>::const_iterator itlink;
        Vector center, vcom;  //in EE  coordinates
        dReal fTotalMass = 0;

    
        std::vector<KinBody::LinkPtr> veclinks;
    
        //first get hand link pointer
        veclinks.push_back(robot->GetActiveManipulator()->GetEndEffector());
        //now get links of object
        RAVELOG_INFO("num links: %d\n", ptarget->GetLinks().size());

        for(int i = 0; i < ptarget->GetLinks().size(); i++)
            veclinks.push_back(ptarget->GetLinks()[i]);
   
        Transform linktm, linkrot;
        
        TransformMatrix Itotal, Inew, Itrans; //these aren't really transform matrices but it makes things convenient            
        Itotal.m[0] = Itotal.m[5] = Itotal.m[10] = 0;

        //first calculate the cg of the hand and grabbed object (in EE coordinates)
        FORIT(itlink, veclinks) {
            if( (*itlink) != NULL ) {
                linktm = tEE.inverse() * (*itlink)->GetTransform();
                linkrot = linktm;
                linkrot.trans.x = 0; linkrot.trans.y = 0; linkrot.trans.z = 0;

                vcom = linktm * (*itlink)->GetCOMOffset();
                RAVELOG_INFO("link mass: %f link cg: %f %f %f\n", (*itlink)->GetMass(), (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
                RAVELOG_INFO("vcom: %f %f %f\n",vcom.x,vcom.y,vcom.z);
                center +=  vcom * (*itlink)->GetMass();
                fTotalMass += (*itlink)->GetMass();
            }
            else
                RAVELOG_INFO("ERROR: Body is NULL!\n");
        }
        if( fTotalMass > 0 )
            center /= fTotalMass;


        //now aggregate inertia matrices at the cg (in EE coordinates)
        FORIT(itlink, veclinks) {
            if( (*itlink) != NULL ) {
                linktm = tEE.inverse() * (*itlink)->GetTransform();
                linkrot = linktm;
                linkrot.trans.x = 0; linkrot.trans.y = 0; linkrot.trans.z = 0;

                dReal tempmass = (*itlink)->GetMass();
                vcom = center - (linktm * (*itlink)->GetCOMOffset()); //this is a vector from the center of mass of this link to the cg


                //first get the inertia matrix in world coordinates
                Inew = (*itlink)->GetGlobalInertia();
                //Inew.m[0] = mass.I[0];  Inew.m[1] = mass.I[1];  Inew.m[2] = mass.I[2];
                //Inew.m[4] = mass.I[4];  Inew.m[5] = mass.I[5];  Inew.m[6] = mass.I[6];
                //Inew.m[8] = mass.I[8];  Inew.m[9] = mass.I[9];  Inew.m[10] = mass.I[10];
                Inew.trans.x = 0; Inew.trans.y = 0; Inew.trans.z = 0;
                RAVELOG_INFO("\nInew (before rotation):\n %f %f %f\n%f %f %f\n%f %f %f\n",Inew.m[0],Inew.m[1],Inew.m[2],Inew.m[4],Inew.m[5],Inew.m[6],Inew.m[8],Inew.m[9],Inew.m[10]);
                
                Inew = TransformMatrix(linkrot)*Inew*TransformMatrix(linkrot.inverse()); //rotate frame into EE coordinates
                RAVELOG_INFO("\nInew:\n %f %f %f\n%f %f %f\n%f %f %f\n",Inew.m[0],Inew.m[1],Inew.m[2],Inew.m[4],Inew.m[5],Inew.m[6],Inew.m[8],Inew.m[9],Inew.m[10]);
                
                //now add on the affects of translation to the matrix
                Itrans.m[0] = tempmass*(vcom.y*vcom.y + vcom.z*vcom.z);    Itrans.m[1] = -tempmass*vcom.x*vcom.y;                   Itrans.m[2] = -tempmass*vcom.x*vcom.z;
                Itrans.m[4] = -tempmass*vcom.y*vcom.x;                     Itrans.m[5] = tempmass*(vcom.x*vcom.x + vcom.z*vcom.z);  Itrans.m[6] = -tempmass*vcom.y*vcom.z;
                Itrans.m[8] = -tempmass*vcom.z*vcom.x;                     Itrans.m[9] = -tempmass*vcom.z*vcom.y;                   Itrans.m[10] = tempmass*(vcom.y*vcom.y + vcom.x*vcom.x);
                RAVELOG_INFO("\nItrans:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itrans.m[0],Itrans.m[1],Itrans.m[2],Itrans.m[4],Itrans.m[5],Itrans.m[6],Itrans.m[8],Itrans.m[9],Itrans.m[10]);
                
                Itotal.m[0] += Inew.m[0] + Itrans.m[0];  Itotal.m[1] += Inew.m[1] + Itrans.m[1];  Itotal.m[2] += Inew.m[2] + Itrans.m[2];
                Itotal.m[4] += Inew.m[4] + Itrans.m[4];  Itotal.m[5] += Inew.m[5] + Itrans.m[5];  Itotal.m[6] += Inew.m[6] + Itrans.m[6];
                Itotal.m[8] += Inew.m[8] + Itrans.m[8];  Itotal.m[9] += Inew.m[9] + Itrans.m[9];  Itotal.m[10] += Inew.m[10] + Itrans.m[10];
                RAVELOG_INFO("\nItotal:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[4],Itotal.m[5],Itotal.m[6],Itotal.m[8],Itotal.m[9],Itotal.m[10]);
            }
        }


        RAVELOG_INFO("\nItotal:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[4],Itotal.m[5],Itotal.m[6],Itotal.m[8],Itotal.m[9],Itotal.m[10]);
        RAVELOG_INFO("\nmass: %f\ncog: %f %f %f\n",fTotalMass,center.x,center.y,center.z);

        char sendstr[512];
        sprintf(sendstr, "grabobject %f %f %f %f %f %f %f %f %f %f",fTotalMass,center.x,center.y,center.z,Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[5],Itotal.m[6],Itotal.m[10]);
        stringstream in;
        in << sendstr;
        stringstream out;
        robot->GetController()->SendCommand(out,in);

    }
    return 0;

}

int ManipulationProblem::RestoreEndEffectorMass(istream& cmd)
{
    RAVELOG_DEBUG("Starting RestoreEndEffectorMass...\n");

    KinBodyPtr ptarget;
    bool bSendToController = false;
    TransformMatrix tmEE;


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "applytransform") == 0 ){
            //this transform is used to move from the RAVE end-effector frame (tEE_rave) to the controller's end-effector frame (tEE_controller)
            //it should be specified in tEE_rave coordinates
            // it is used like this: tEE_controller = tEE_rave * T_applytransform
            cmd >> tmEE.m[0];
            cmd >> tmEE.m[4];
            cmd >> tmEE.m[8];
            cmd >> tmEE.m[1];
            cmd >> tmEE.m[5];
            cmd >> tmEE.m[9];
            cmd >> tmEE.m[2];
            cmd >> tmEE.m[6];
            cmd >> tmEE.m[10];
            cmd >> tmEE.trans.x;
            cmd >> tmEE.trans.y;
            cmd >> tmEE.trans.z;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
    }
    

    //RAVELOG_INFO("tEE (before): %f %f %f\n",robot->GetActiveManipulator()->GetEndEffectorTransform().trans.x,robot->GetActiveManipulator()->GetEndEffectorTransform().trans.y,robot->GetActiveManipulator()->GetEndEffectorTransform().trans.z);
    Transform tEE = robot->GetActiveManipulator()->GetEndEffectorTransform() * Transform(tmEE);
    //RAVELOG_INFO("tEE (after): %f %f %f\n",tEE.trans.x,tEE.trans.y,tEE.trans.z);
    //get mass, com, and inertia tensor in end-effector coordinates
    std::vector<KinBody::LinkPtr>::const_iterator itlink;
    Vector center, vcom;  //in EE  coordinates

    dReal fTotalMass = 0;


    std::vector<KinBody::LinkPtr> veclinks;

    //first get hand link pointer
    veclinks.push_back(robot->GetActiveManipulator()->GetEndEffector());


    Transform linktm, linkrot;
    
    TransformMatrix Itotal, Inew, Itrans; //these aren't really transform matrices but it makes things convenient            
    Itotal.m[0] = Itotal.m[5] = Itotal.m[10] = 0;

    //first calculate the cg of the hand and grabbed object (in EE coordinates)
    FORIT(itlink, veclinks) {
        if( (*itlink) != NULL ) {
            linktm = tEE.inverse() * (*itlink)->GetTransform();
            linkrot = linktm;
            linkrot.trans.x = 0; linkrot.trans.y = 0; linkrot.trans.z = 0;

   
            vcom = linktm * (*itlink)->GetCOMOffset();
            RAVELOG_INFO("link mass: %f link cg: %f %f %f\n", (*itlink)->GetMass(), (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
            RAVELOG_INFO("vcom: %f %f %f\n",vcom.x,vcom.y,vcom.z);
            center +=  vcom * (*itlink)->GetMass();
            fTotalMass += (*itlink)->GetMass();
        }
        else
            RAVELOG_INFO("ERROR: Body is NULL!\n");
    }
    if( fTotalMass > 0 )
        center /= fTotalMass;


    //now aggregate inertia matrices at the cg (in EE coordinates)
    FORIT(itlink, veclinks) {
        if( (*itlink) != NULL ) {
            linktm = tEE.inverse() * (*itlink)->GetTransform();
            linkrot = linktm;
            linkrot.trans.x = 0; linkrot.trans.y = 0; linkrot.trans.z = 0;

            dReal tempmass = (*itlink)->GetMass();
            vcom = center - (linktm * (*itlink)->GetCOMOffset()); //this is a vector from the center of mass of this link to the cg

            //first get the inertia matrix in world coordinates
            Inew = (*itlink)->GetGlobalInertia();
            //Inew.m[0] = mass.I[0];  Inew.m[1] = mass.I[1];  Inew.m[2] = mass.I[2];
            //Inew.m[4] = mass.I[4];  Inew.m[5] = mass.I[5];  Inew.m[6] = mass.I[6];
            //Inew.m[8] = mass.I[8];  Inew.m[9] = mass.I[9];  Inew.m[10] = mass.I[10];
            Inew.trans.x = 0; Inew.trans.y = 0; Inew.trans.z = 0;
            RAVELOG_INFO("\nInew (before rotation):\n %f %f %f\n%f %f %f\n%f %f %f\n",Inew.m[0],Inew.m[1],Inew.m[2],Inew.m[4],Inew.m[5],Inew.m[6],Inew.m[8],Inew.m[9],Inew.m[10]);
            
            Inew = TransformMatrix(linkrot)*Inew*TransformMatrix(linkrot.inverse()); //rotate frame into EE coordinates
            RAVELOG_INFO("\nInew:\n %f %f %f\n%f %f %f\n%f %f %f\n",Inew.m[0],Inew.m[1],Inew.m[2],Inew.m[4],Inew.m[5],Inew.m[6],Inew.m[8],Inew.m[9],Inew.m[10]);
            
            //now add on the affects of translation to the matrix
            Itrans.m[0] = tempmass*(vcom.y*vcom.y + vcom.z*vcom.z);    Itrans.m[1] = -tempmass*vcom.x*vcom.y;                   Itrans.m[2] = -tempmass*vcom.x*vcom.z;
            Itrans.m[4] = -tempmass*vcom.y*vcom.x;                     Itrans.m[5] = tempmass*(vcom.x*vcom.x + vcom.z*vcom.z);  Itrans.m[6] = -tempmass*vcom.y*vcom.z;
            Itrans.m[8] = -tempmass*vcom.z*vcom.x;                     Itrans.m[9] = -tempmass*vcom.z*vcom.y;                   Itrans.m[10] = tempmass*(vcom.y*vcom.y + vcom.x*vcom.x);
            RAVELOG_INFO("\nItrans:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itrans.m[0],Itrans.m[1],Itrans.m[2],Itrans.m[4],Itrans.m[5],Itrans.m[6],Itrans.m[8],Itrans.m[9],Itrans.m[10]);
            
            Itotal.m[0] += Inew.m[0] + Itrans.m[0];  Itotal.m[1] += Inew.m[1] + Itrans.m[1];  Itotal.m[2] += Inew.m[2] + Itrans.m[2];
            Itotal.m[4] += Inew.m[4] + Itrans.m[4];  Itotal.m[5] += Inew.m[5] + Itrans.m[5];  Itotal.m[6] += Inew.m[6] + Itrans.m[6];
            Itotal.m[8] += Inew.m[8] + Itrans.m[8];  Itotal.m[9] += Inew.m[9] + Itrans.m[9];  Itotal.m[10] += Inew.m[10] + Itrans.m[10];
            RAVELOG_INFO("\nItotal:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[4],Itotal.m[5],Itotal.m[6],Itotal.m[8],Itotal.m[9],Itotal.m[10]);
        }
    }


    RAVELOG_INFO("\nItotal:\n %f %f %f\n%f %f %f\n%f %f %f\n",Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[4],Itotal.m[5],Itotal.m[6],Itotal.m[8],Itotal.m[9],Itotal.m[10]);
    RAVELOG_INFO("\nmass: %f\ncog: %f %f %f\n",fTotalMass,center.x,center.y,center.z);

    char sendstr[512];
    sprintf(sendstr, "grabobject %f %f %f %f %f %f %f %f %f %f",fTotalMass,center.x,center.y,center.z,Itotal.m[0],Itotal.m[1],Itotal.m[2],Itotal.m[5],Itotal.m[6],Itotal.m[10]);
    stringstream in;
    in << sendstr;
    stringstream out;
    robot->GetController()->SendCommand(out,in);


   return 0; 
}
int ManipulationProblem::SetTransparency(istream &cmd)
{
    RAVELOG_DEBUG("Starting SetTransparency...\n");

    KinBodyPtr ptarget;
    dReal ftransparency = 0.1;

    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "name") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
            if(ptarget.get() == NULL)
            {
                RAVELOG_ERRORA("SetTransparency: Cannot find body name %s\n",p.c_str());
                return -1;
            }
        }
        else if( stricmp(p.c_str(), "transparency") == 0 ) {
            cmd >> ftransparency;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
    }

    std::vector<KinBody::LinkPtr> plinks = ptarget->GetLinks();

    for(int i = 0; i < plinks.size(); i++)
    {
        std::vector<boost::shared_ptr<KinBody::Link::Geometry> >geoms = plinks[i]->GetGeometries();

        for(int j = 0; j < geoms.size(); j++)
        {
            plinks[i]->GetGeometry(j)->SetTransparency(ftransparency);
        }

//        std::list<KinBody::Link::GEOMPROPERTIES>::iterator itr;
//        for(itr=geoms.begin(); itr != geoms.end(); ++itr)
//        {
//            (*itr).SetTransparency(ftransparency);
//            RAVELOG_INFO("a\n");
//        }
    }

    return 0;
}

int ManipulationProblem::MakeGraspTrajectory(string& response, istream& cmd)
{

    RAVELOG_DEBUG("Starting MakeGraspTrajectory...\n");
    RobotBasePtr _robot;
    bool bdraw = false;
    bool bHasStepSize = false;
    bool bHasDirection = false;
    Vector direction;
    int numdof = -1;

    std::vector<std::vector<Vector> > vpoints;
    std::vector<std::vector<int>  > vpointsetinds;
    std::vector<int> vdofinds;
    std::vector<int> vpointsperdof;

    std::vector<dReal> vstepsizes;


    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "stepsizes") == 0 ) {
            int numsizes;
            cmd >> numsizes;
            vstepsizes.resize(numsizes);
            for(int i = 0; i < numsizes;i++)
                cmd >> vstepsizes[i];
            
            bHasStepSize = true;
        }
        else if( stricmp(p.c_str(), "draw") == 0 ) {
            bdraw = true;
        }
        else if( stricmp(p.c_str(), "robotid") == 0 ) {
            // specify which robot to use
            int robotid;
            cmd >> robotid;
            vector<RobotBasePtr>::const_iterator itrobot;
            std::vector<RobotBasePtr> robots;
            GetEnv()->GetRobots(robots);
            SetActiveRobots(robots); 

            FORIT(itrobot, robots) {
                if( (*itrobot)->GetEnvironmentId() == robotid ) {
                    _robot = *itrobot;
                    break;
                }
            }
            if( _robot.get() == NULL ) {
                RAVELOG_INFO("Failed to find robot with id: %d\n", robotid);
                return -1;
            }
            vpoints.resize(_robot->GetLinks().size());
            numdof = _robot->GetActiveDOF();
            vpointsperdof.resize(numdof);
        }
        else if( stricmp(p.c_str(), "link") == 0 ) {

            int linkind;
            cmd >> linkind;
            int numpoints;
            cmd >> numpoints;
            RAVELOG_DEBUG("number of links: %d\n",vpoints.size());
            RAVELOG_DEBUG("linkind: %d  numpoints: %d\n",linkind,numpoints);
            RAVELOG_DEBUG("number of links: %d\n",vpoints.size());

            if(linkind > vpoints.size())
            {
                RAVELOG_INFO("Error: link index out of bounds\n");
                return -1;
            }


            Vector temp;           
            std::vector<int> tempsetinds(2,0);

            for(size_t i = 0; i < numpoints; ++i)
            {
                cmd >> temp.x;
                cmd >> temp.y;
                cmd >> temp.z;
                vpoints[linkind].push_back(temp);
                //RAVELOG_DEBUG("%f %f %f\n",vpoints[linkind][i].x,vpoints[linkind][i].y,vpoints[linkind][i].z);
            }

            //find which dof this link corresponds to (take first active one)

            for(int i = 0; i < numdof; i++)
            {
                if(_robot->DoesAffect(_robot->GetActiveDOFIndices()[i],linkind))
                {
                    vdofinds.push_back(i);
                    tempsetinds[0] = vpointsperdof[i];
                    vpointsperdof[i] += numpoints;
                    tempsetinds[1] = vpointsperdof[i];
                    break;
                }
            }

            vpointsetinds.push_back(tempsetinds);
            //make sure a dof was found
            if(vpointsetinds.size() != vdofinds.size())
            {
                RAVELOG_INFO("Failed to find a corresponding active DOF for this link.\n");
                return -1;
            }


        }
        else if( stricmp(p.c_str(), "direction") == 0 ){
            cmd >> direction.x;
            cmd >> direction.y;
            cmd >> direction.z;
            bHasDirection = true;
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


    }

    std::vector<KinBody::LinkPtr> links = _robot->GetLinks();


    RAVELOG_DEBUG("Finished reading in points.\n");

    vector<dReal> closingdir;
    RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();    

    
    closingdir = pmanip->GetClosingDirection();


    vector<vector<vector<ManipulationProblem::POINT> > > vjointvshells(numdof+1); //extra 1 is for immobile points (like on link 0)
    vector<ManipulationProblem::POINT> dummy;
    std::vector<dReal> vlowerlimit(numdof), vupperlimit(numdof);
    _robot->GetActiveDOFLimits(vlowerlimit,vupperlimit);

    if(closingdir.size() != numdof)
    {

        RAVELOG_INFO("ManipulationProblem Error - Active dofs (%d) can only be manipulator vecjoints dofs \n",numdof);
        return -1;
    }
    

    if(bHasStepSize && (vstepsizes.size() != numdof))
    {
        RAVELOG_INFO("ManipulationProblem Error - Active dofs (%d) must be the same number as number of step sizes passed in (%d)\n",numdof,vstepsizes.size());
        return -1;

    }

    dReal defaultstepsize = 0.2f;
    dReal stepsize;

    vector<dReal> vcurrent(numdof); 
    vector<dReal> vstart(numdof);
    _robot->GetActiveDOFValues(vstart);
    Transform Ttemp;
    std::vector<dReal> J(3*_robot->GetDOF());
    Vector defaultdir;

    if(bHasDirection)
        defaultdir = direction;
    else
        defaultdir = Vector(0,0,1);
    Vector deltaxyz;
    float temp;
    ManipulationProblem::POINT pointtemp;

    for(int i = 0; i < numdof; i++)
    {
        if(bHasStepSize)
            stepsize = vstepsizes[i];
        else
            stepsize = defaultstepsize;

        vcurrent[i] = vstart[i] - stepsize*closingdir[i];
        //RAVELOG_INFO("Starting DOF %d.\n",i);

        for(int j = 0; ((closingdir[i] == 1.0f) && (vcurrent[i] < vupperlimit[i])) || ((closingdir[i] == -1.0f) && (vcurrent[i] > vlowerlimit[i])); j++)
        {
            //RAVELOG_INFO("Starting Shell %d.\n",j);

            vcurrent[i] += stepsize*closingdir[i];
            _robot->SetActiveDOFValues(vcurrent);

            vjointvshells[i].push_back(dummy);
            for(int q = 0; q < (int)links.size(); q++)
            {
                //RAVELOG_INFO("Starting Link %d.\n",q);
                if(_robot->DoesAffect(_robot->GetActiveDOFIndices()[i],q))
                {
                    Ttemp = links[q]->GetTransform(); 
                    for(int qp = 0; qp < vpoints[q].size(); qp++)
                    {
                        pointtemp.pos = Ttemp * vpoints[q][qp];

                        memset(&J[0], 0, sizeof(dReal)*J.size());
                        _robot->CalculateJacobian(q, pointtemp.pos, J);
                        //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                        for(int qj = 0; qj < 3; qj++)
                        {   


                            temp = J[qj*_robot->GetDOF() + _robot->GetActiveDOFIndices()[i]]*0.01f*closingdir[i];
                                                        
                            if( qj == 0)
                                deltaxyz.x  = temp;
                            else if( qj == 1)
                                deltaxyz.y = temp;
                            else if( qj == 2)
                                deltaxyz.z = temp;
                        }

                        //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                        //so treat it as if it were part of the base link
                        /*if(lengthsqr3(deltaxyz) < 0.000000001f)*/
                        if(deltaxyz.lengthsqr3() < 0.000000001f)
                            deltaxyz = defaultdir;
                        
                        /*normalize3(deltaxyz,deltaxyz);*/
                        deltaxyz.normalize3();

                        pointtemp.norm = deltaxyz;
                        vjointvshells[i][j].push_back(pointtemp);
#if 0
                        /* It appears this code no longer works, as of OpenRAVE 0.64;
                         * plot3 takes an array of points, and you must save the returned graph handle! */
                        if(bdraw)
                        {
                            GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );
                            GetEnv()->drawarrow(RaveVector<float>(pointtemp.pos),RaveVector<float>(pointtemp.pos + 0.01*pointtemp.norm), 0.002f,RaveVector<float>(0,1,0));
                        }
#endif
                    }
                }
            }
        }

    }
    //RAVELOG_INFO("Finished mobile links.\n");
    //add in immobile points in the last shell
    vjointvshells.back().push_back(dummy);
    bool affects;
    for(int i = 0; i < (int)links.size();i++)
    {
        affects = false;
        for(int j = 0; j < numdof; j++)
            if(_robot->DoesAffect(_robot->GetActiveDOFIndices()[i],i))
            {
                affects = true;
                break;
            }
        if(!affects)
        {              
            for(int qp = 0; qp < vpoints[i].size(); qp++)
            {
                Ttemp = links[i]->GetTransform(); 
                pointtemp.pos = Ttemp * vpoints[i][qp];
                pointtemp.norm = defaultdir;
                //pointtemp.pos = vpoints[i][qp];
                vjointvshells.back().back().push_back(pointtemp);
#if 0
                /* It appears this code no longer works, as of OpenRAVE 0.64;
                 * plot3 takes an array of points, and you must save the returned graph handle! */
                GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );
                GetEnv()->drawarrow(RaveVector<float>(pointtemp.pos),RaveVector<float>(pointtemp.pos + 0.01*pointtemp.norm), 0.002f,RaveVector<float>(0,1,0));
#endif
            }
        }
    }
    //RAVELOG_INFO("Finished immobile links.\n");
    //_robot->SetActiveDOFValues(NULL,&vlowerlimit[0]);

    std::stringstream outstream;
    //now print the shells to the return string


    //RAVELOG_INFO("Printing to response string...\n");
    int dof = -1;
    //assumes only one dof affects each point sets
    outstream << vdofinds.size() << "\n";
    for(int dofindsi = 0; dofindsi < vdofinds.size(); dofindsi++)
    {
        dof = vdofinds[dofindsi];        
        outstream << vjointvshells[dof].size() << "\n";
        for(int stepind = 0; stepind < vjointvshells[dof].size(); stepind++)
        {
            outstream << vpointsetinds[dofindsi][1] - vpointsetinds[dofindsi][0] << "\n";
            for(int p = vpointsetinds[dofindsi][0]; p < vpointsetinds[dofindsi][1]; p++)
            {
                outstream << vjointvshells[dof][stepind][p].pos.x << " "
                          << vjointvshells[dof][stepind][p].pos.y << " "
                          << vjointvshells[dof][stepind][p].pos.z << " " 
                          << vjointvshells[dof][stepind][p].norm.x << " " 
                          << vjointvshells[dof][stepind][p].norm.y << " " 
                          << vjointvshells[dof][stepind][p].norm.z << "\n";

            }
        }
    }
    response = outstream.str();
    return 0;

  
}




