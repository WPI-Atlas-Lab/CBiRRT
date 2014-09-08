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

#include "stdafx.h"

////////////////
// ManipulationProblem //
////////////////

#ifdef _WIN32
LARGE_INTEGER g_lfreq;
#endif

#ifdef _MSC_VER
#define snprintf _snprintf
#endif


typedef unsigned long __u64;

__u64 GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count;
    QueryPerformanceCounter(&count);
    return count.QuadPart * 1000000 / g_lfreq.QuadPart;
#else
    timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec*1000000+t.tv_usec;
#endif
}

ManipulationProblem::ManipulationProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    _graspsuccess = false;
    _done = false;
    _bLog = false;
    _bVisionEnable = true;
    //pGrasperProblem = NULL;
    nStartTime = 0;
    _state = PS_Nothing;
    nRealTrajTime = 4000;
    pfLog = NULL;
    fLogTime = fTotalTime = 0;
#ifdef USING_PLAYER
    _pvisionclient = NULL;
    _pjoystickclient = NULL;
#endif
    _pPRMGraph = NULL;
    //_pPlanner = NULL;
    _reusePlanner = false;
    _bisWAM = false;
    _prevjoystickmode = 0;

#ifdef _WIN32
    QueryPerformanceFrequency(&g_lfreq);
#endif
}

void ManipulationProblem::Destroy()
{
#ifdef USING_PLAYER
    delete _pvisionclient; _pvisionclient = NULL;
    delete _pjoystickclient; _pjoystickclient = NULL;
#endif
    //if(_pPRMGraph != NULL) 
    //{
        //delete _pPRMGraph;//this doesn't work b/c can't delete void * TODO: STORE whole PRM PLANNER instead
        //_pPRMGraph = NULL;
    //}     

    //if(_pPlanner != NULL) 
    //{
        //delete _pPlanner;
        //_pPlanner.delete();
    //}     
    //robot = NULL;
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
/*
#ifdef USING_PLAYER
    RAVELOG_INFO("env: %s\n", cmd);
    const char* pvisioncmd = strstr(cmd, "vision ");
    if( pvisioncmd != NULL ) {

        delete _pvisionclient; _pvisionclient = NULL;
        
        // start the vision client
        pvisioncmd += 7;

        char host[256];
        int port;
        RAVELOG_INFO("visioncmd: %s\n", pvisioncmd);
        if( sscanf(pvisioncmd, "%s %d", host, &port) == 2 ) {
            _pvisionclient = new RaveVisionClient();
            _bVisionEnable = true;
            if( _pvisionclient->Init(host, port) ) {
                RAVELOG_INFO("connected to vision server: %s:%d\n", host, port);                
                _pvisionclient->AddRegisteredBodies(GetEnv()->GetBodies());
            }
            else {
                // don't destroy pvisionclient
                RAVELOG_INFO("failed to init vision client\n");
            }
        }
        else RAVELOG_INFO("wrong parameters for vision. Need host port\n");
    }

    RAVELOG_INFO("env: %s\n", cmd);
    const char* pjoystickcmd = strstr(cmd, "joystick ");
    if( pjoystickcmd != NULL ) {

        delete _pjoystickclient; _pjoystickclient = NULL;
        
        // start the joystick client
        pjoystickcmd += 9;

        char host[256];
        int port;
        RAVELOG_INFO("joystickcmd: %s\n", pjoystickcmd);
        if( sscanf(pjoystickcmd, "%s %d", host, &port) == 2 ) {
            _pjoystickclient = new JoystickClient();
            _bjoystickEnable = true;
            if( _pjoystickclient->Init(host, port) ) {
                RAVELOG_INFO("connected to joystick server: %s:%d\n", host, port);                
            }
            else {
                // don't destroy pjoystickclient
                RAVELOG_INFO("failed to init joystick client\n");
            }
        }
        else RAVELOG_INFO("wrong parameters for joystick. Need host port\n");
    }


#endif
*/
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



//        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),"rBiRRT");
//        if( planner != NULL ) {
//            _strRRTPlannerName = "rBiRRT";
//        }
//        else {
//            PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),"BiRRT");
//            if( planner != NULL )
//                _strRRTPlannerName = "BiRRT";
//        }

        if( planner == NULL ) {
            RAVELOG_INFO("failed to find birrt planner\n");
            return -1;
        }
        //delete planner;
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
    //else if( pGrasperProblem->main(NULL) < 0 )
    //    return -1;

#ifdef USING_PLAYER
    _Tcamera = GetEnv()->GetCameraTransform();
    _Tjoystick = robot->GetActiveManipulator()->GetEndEffectorTransform();
#endif


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
    //RAVELOG_DEBUG("Starting SetupRobot...\n");
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

//    for(size_t i = 0; i < robot->GetManipulators().size(); ++i) {
//        if( strcasecmp(robot->GetManipulators()[i]->GetIKSolverName().c_str(), "WAM7") == 0 ) {
//            robot->SetActiveManipulator(0);
//            _bisWAM = true;
//        }
//    }
}

bool ManipulationProblem::SendCommand(ostream& output, istream& sinput)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    _graspsuccess = false;

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots); 
    string response;

//     std::stringstream temp;
//     do
//     {
//         char itemp = is.get();
//         if(is.good())
//            temp << char(itemp);
//     }while(is);
// 
//     RAVELOG_INFO("command: %s\n",temp.str().c_str());
//     const char* cmd = temp.str().c_str();


//     is.seekg (0, ios::end);
//     int length = is.tellg();
//     is.seekg (0, ios::beg);
// 
//     // allocate memory:
//     char* cmd = new char [length];
// 
//     // read data as a block:
//     is.read (cmd,length);
//     
//     RAVELOG_INFO("command: %s\n",cmd);


//     const char* delim = " \r\n\t";
//     char* mycmd = strdup(cmd);
// 
// 
//     char* p = strtok(mycmd, delim);

      string p;
      while(!sinput.eof()) {
        sinput >> p;
        if( !sinput )
            break;

        if( stricmp(p.c_str(), "CloseFingers") == 0 ) {
            if( robot.get() == NULL )
                return false;
            CloseFingers(sinput);
        }
//         else if( stricmp(p.c_str(), "TestAllGrasps") == 0 ) {
//             if( robot.get() == NULL )
//                 return false;
//             TestAllGrasps(response);
//         }
//         else if( stricmp(p.c_str(), "Release") == 0 ) {
//             if( robot.get() == NULL )
//                 return false;
//             ReleaseFingers();
//         }
        else if( stricmp(p.c_str(), "GrabBody") == 0 ) {
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
//         else if( stricmp(p.c_str(), "GetIKBias") == 0 ){
//             if(robot.get() == NULL)
//                 return false;
//              
//             vector<dReal> v = GetIKBias();
// 
//             char str[16];
//             for(size_t i = 0; i < v.size(); ++i) {
//                 sprintf(str, "%f ", v[i]);
//                 response += str;
//             }
// 
//         }
//         else if( stricmp(p.c_str(),  "DrawPoint") == 0 ){
//             if(robot.get() == NULL)
//                 return false;
//             if(DrawPoint() < 0)
//                 response = "0";
//             else
//                 response = "1";
// 
//         }
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
//         else if( stricmp(p.c_str(),  "CheckTolerance") == 0 ){
//             if(CheckTolerance())
//                 response = "1";
//             else
//                 response = "0";
//             
//         }
        else if( stricmp(p.c_str(), "MoveToHandPosition") == 0 ) {
            if( robot.get() == NULL )
                return false;

            if(MoveToHandPosition(sinput) < 0)
                response = "0";
            else
                response = "1";
        }
//         else if( stricmp(p.c_str(), "gohome") == 0 ) {
// 
//             if( robot.get() == NULL )
//                 return false;
//             
//             PlannerBase::PlannerParametersPtr params;
//             
// #ifdef INTELCONTROLLER
//             if( robot != NULL && dynamic_cast<IntelController*>(robot->GetController()) != NULL ) {
//                 dynamic_cast<IntelController*>(robot->GetController())->EnableHandMotors(_bRightHand, true);
//             }
// #endif
// 
//             vector<int> indices, varmgoal;
//             params->vgoalconfig.resize(robot->GetDOF(), 0);
//             for(int i = 0; i < robot->GetDOF(); ++i) {
//                 char* p = strtok(NULL, delim);
//                 if( p == NULL )
//                     break;
// 
//                 params->vgoalconfig[i] = (dReal)atof(p);
//                 indices.push_back(i);
//             }
//             
//             robot->SetActiveDOFs(indices);
//             robot->GetActiveDOFValues(params->vinitialconfig);
//             
//             RAVELOG_INFO("Planning with %d DOF\n", robot->GetDOF());
// 
//             response = "0";
//             if (!_reusePlanner){
//                 _pPlanner = GetEnv()->CreatePlanner(_strRRTPlannerName);
//                 if( _pPlanner == NULL ) {
//                     RAVELOG_INFO("failed to create planner\n");
//                     return false;
//                 }
//             }
// 
//             if( _pPlanner != NULL ) {
//                 if( _pPlanner->InitPlan(robot, params) ) {
//                     TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(robot->GetActiveDOF());
//                     if( _pPlanner->PlanPath(ptraj) ) {
//                         ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
//                         robot->SetActiveMotion(ptraj);
//                         response = "1";
//                     }
//                     else RAVELOG_DEBUG("PlanPath failed\n");
//                 }
//                 else RAVELOG_DEBUG("InitPlan failed\n");
//                 
//                 //if (!_reusePlanner){
//                     //delete _pPlanner;
//                     //_pPlanner = NULL;
//                 //}
//             }
//             else RAVELOG_DEBUG("Failed to find planner!\n");
//         }
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
                //instr << temp.str().c_str();
                //string junk;
                //instr >> junk;
        
                outstr << response;
                pGrasperProblem->SendCommand(outstr,sinput);
                response = outstr.str();
                //pGrasperProblem->SendCommand(p+8, response);
            }
            else
                break;
        }
        // disable/enable vision
#ifdef USING_PLAYER
        else if( stricmp(p.c_str(), "vision") == 0 ) {
            p = strtok(NULL, delim);
            if( p != NULL ) {
                bool bOld = _bVisionEnable;
                _bVisionEnable = atoi(p)>0;

                if( _bVisionEnable ) {
                    if( _pvisionclient != NULL )
                        _pvisionclient->UpdateBodies();
                    if( !bOld ) RAVELOG_INFO("vision enabled\n");
                }
                else if( bOld ) RAVELOG_INFO("vision disabled\n");
            }
        }

        else if( stricmp(p.c_str(), "visionenable") == 0 ) {
            
            if( _pvisionclient != NULL ) {
                char* pbodyid = strtok(NULL, delim);
                char* pusevision  = strtok(NULL, delim);
                
                if( pbodyid != NULL && pusevision != NULL ) {
                    
                    KinBodyPtr pbody = GetEnv()->GetBodyFromNetworkId(atoi(pbodyid));
                    
                    if( pbody != NULL ) {
                        if( _pvisionclient->EnableBody(pbody, atoi(pusevision)>0) ) {
                            RaveVisionClient::BODY* pvis = _pvisionclient->GetBody(pbody);
                            assert( pvis != NULL );
                        }
                    }
                }
            }
        }
#endif
        // disable/enable logging
//         else if( stricmp(p.c_str(), "log") == 0 ) {
//             p = strtok(NULL, delim);
//             if( p != NULL )
//                 _bLog = atoi(p)>0;
//         }
//         else if( stricmp(p.c_str(), "motors") == 0 ) {
// 
//             p = strtok(NULL, delim);
//             if( p != NULL ) {
//                 bool bEnable = atoi(p)>0;
// 
// #ifdef INTELCONTROLLER
//                 if( robot != NULL && dynamic_cast<IntelController*>(robot->GetController()) != NULL ) {
//                     dynamic_cast<IntelController*>(robot->GetController())->EnableAllMotors(bEnable);
//                 }
// #endif
//             }
//         }
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
                TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
                ifstream infile(p.c_str(),ios::in);
                if( ptraj->Read(infile, robot) ) {
                    
                    sinput >> p;
                    bool bResetTrans = p != "" && stricmp(p.c_str(), "notrans");

                    if( bResetTrans ) {
                        // set the transformation of every point to the current robot transformation
                        Transform tcur = robot->GetTransform();
                        FOREACH(itpoint, ptraj->GetPoints())
                            itpoint->trans = tcur;
                    }

                    robot->SetMotion(ptraj);
                    infile.close();
                }
                else
                {
                    RAVELOG_DEBUG("ManipulationProblem: failed to find trajectory\n");
                    infile.close();
                }

                //delete ptraj;
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
//        else if( stricmp(p.c_str(), "MoveManipulatorPRM") == 0 ) {
//           if( robot.get() == NULL )
//               return false;
//           if(MoveManipulatorPRM())
//               response="1";
//        }      
//         else if( stricmp(p.c_str(), "CheckContactStability") == 0 ) {
//             if( robot.get() == NULL )
//                 return false;
//             CheckContactStability();
//         }
//         else if( stricmp(p.c_str(), "CheckSlidingContact") == 0 ) {
//             if( robot.get() == NULL )
//                 return false;
//             CheckSlidingContact();
//         }
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
//         else if( stricmp(p.c_str(), "PlayBackPointTraj") == 0 ) {
//             PlayBackPointTraj();
//         }
        else if( stricmp(p.c_str(), "help") == 0 ) {
            response += "ManipulationProblem Commands:\n"
                "help - Command help list\n"
                "vision [1/0] - Enables updating objects from the mocap\n"
                "controller [1/0] - If 1 uses the real hrp2 controller, if 0 uses the ideal controller (for testing purposes)\n"
                "armiktest [1/0] translation_xyz quaternion_xyzw - If the first param is 1, uses the right hand, otherwise left hand.\n"
                "    Solves the IK given the hand's transformation\n";
        }

        //p = strtok(NULL, delim);
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
    //free(mycmd);
    return true;
}

void ManipulationProblem::Query(const char* query, string& response)
{

}

bool ManipulationProblem::SimulationStep(dReal fElapsedTime)
{
#ifdef USING_PLAYER
    if( _pvisionclient != NULL && _bVisionEnable ) {
        GetEnv()->LockPhysics(false);
        _pvisionclient->UpdateBodies();
        GetEnv()->LockPhysics(true);
    }

    if( _pjoystickclient != NULL)
    {
        float xyzstep = 0.0025;

        int joystickmode = _pjoystickclient->GetJoystickMode();
        Vector xyz = _pjoystickclient->GetJoystickXYZVelocity();
        Vector rpy = _pjoystickclient->GetJoystickRPYVelocity();
        int snapback = _pjoystickclient->GetSnapBack();
        float elbowbend = _pjoystickclient->GetElbowBend();
        KinBodyPtr pIKSphere = GetEnv()->GetKinBody("IKSphere");
        

        RAVELOG_INFO("xyz vel: %f %f %f\n",xyz.x,xyz.y,xyz.z);
        RAVELOG_INFO("rpy vel: %f %f %f\n",rpy.x,rpy.y,rpy.z);
        RAVELOG_INFO("snapback: %d   elbowbend: %f\n",snapback,elbowbend);
        RAVELOG_INFO("mode: %d\n",joystickmode);
        std::vector<dReal> vlowerlimits, vupperlimits;
        robot->GetActiveDOFLimits(vlowerlimits,vupperlimits);

        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
        
        vector<dReal> viksol(pmanip->GetArmIndices().size(),0);
        vector<dReal> vfulljoints(robot->GetDOF());
        robot->GetDOFValues(vfulljoints);
        int numdofs = robot->GetDOF();
        TransformMatrix Tmtarg = TransformMatrix(pmanip->GetEndEffectorTransform());
        
        Vector right,up,dir,pos;
        Tmtarg.Extract(right, up, dir, pos);


        Transform Ttarg(Tmtarg);
           
        
        if(joystickmode)
        {

            Ttarg.trans += (xyz.x*right + xyz.y*dir + xyz.z*up)*xyzstep;
            Transform TrotR;
            Transform TrotP;
            Transform TrotY;

            dQFromAxisAndAngle(TrotR.rot, up.x,up.y,up.z, rpy.x*M_PI*0.025);
            dQFromAxisAndAngle(TrotP.rot, right.x,right.y,right.z, rpy.y*M_PI*0.025);
            dQFromAxisAndAngle(TrotY.rot, dir.x,dir.y,dir.z, rpy.z*M_PI*0.025);
            //RAVELOG_INFO("old rot: %f %f %f %f\n", Ttarg.rot.x, Ttarg.rot.y, Ttarg.rot.z, Ttarg.rot.w);
            //RAVELOG_INFO("to apply: %f %f %f %f\n",TrotR.rot.x, TrotR.rot.y, TrotR.rot.z, TrotR.rot.w);


            dQMultiply0(Ttarg.rot,TrotR.rot,Ttarg.rot);
            dQMultiply0(Ttarg.rot,TrotP.rot,Ttarg.rot);
            dQMultiply0(Ttarg.rot,TrotY.rot,Ttarg.rot);
            
            //RAVELOG_INFO("new rot: %f %f %f %f\n", Ttarg.rot.x, Ttarg.rot.y, Ttarg.rot.z, Ttarg.rot.w);
            if(xyz.x != 0 || xyz.y !=0 || xyz.z != 0 || rpy.x != 0 || rpy.y != 0 || rpy.z != 0)
            {
                if( !pmanip->FindIKSolution(Ttarg, viksol, true) )
                {
                    RAVELOG_INFO("No IK Solution Found\n");
                }
                else
                {
                    for( int i = 0 ; i < viksol.size(); i++)
                        vfulljoints[i] = viksol[i];
                }

                robot->SetJointValues(vfulljoints);
            }

        
        }
        else if(snapback == 1)
        {
            _Tjoystick.trans = Ttarg.trans;
            if(pIKSphere != NULL) pIKSphere->SetTransform(_Tjoystick);
        }
        else
        {   
            if(xyz.x != 0 || xyz.y !=0 || xyz.z != 0)
            {
                static vector<dReal> J(numdofs*3);
                static vector<dReal> Jplus(numdofs*3); //Jplus = JtJJt  is (nx3)


                static vector<dReal> W(numdofs*numdofs); //nxn
                static vector<dReal> JplusJ(numdofs*numdofs); //nxn
                static vector<dReal> NullSpace(numdofs*numdofs); //nxn

                static vector<dReal> vtheta_avg(numdofs);
                static vector<dReal> vnullspace_jointvals(numdofs);
                
                //compute nullspace joint weighting matrix
                for(int i = 0; i < numdofs; i++)
                {
                    vtheta_avg[i] = (vupperlimits[i] + vlowerlimits[i])/2;
                    W[i*numdofs + i] = 1/((vupperlimits[i] - vlowerlimits[i])*(vupperlimits[i] - vlowerlimits[i]));
                }
                //W[4*numdofs + 4] = 0;
                //W[5*numdofs + 5] = 0;

                dReal JJt[9], invJJt[9];
                const dReal flambda = 1e-4f;

                memset(&J[0], 0, sizeof(dReal)*J.size());
                std::vector<dReal> vtempjoints = vfulljoints;
                std::vector<dReal> vbackupjoints = vfulljoints;



                //spherical coordinates
                //Transform Trobot = robot->GetTransform();
                //dJointGetHingeAnchor(robot->GetJoints()[0]->joint, Trobot.trans);


                //Vector p_b = Trobot.inverse()*Ttarg.trans;
                //float r = sqrt(p_b.lengthsqr3()) + xyz.z*M_PI*0.0025;
                //float phi = acos(p_b.x/sqrt(p_b.x*p_b.x + p_b.y*p_b.y)) + xyz.x*M_PI*0.0025;
                //float theta = acos(p_b.z/sqrt(p_b.lengthsqr3()))+ xyz.y*M_PI*0.0025;
                //Vector p_bnew = Vector(r*sin(theta)*cos(phi),r*sin(theta)*sin(phi),r*cos(theta));
                
                //Ttarg.trans = Trobot*p_bnew;






    
                
                //Transform Trobot = robot->GetTransform();
                //Transform TrotR;
                //Transform TrotP;
                //Transform TrotY;

                //dQFromAxisAndAngle(TrotR.rot, 1,0,0,xyz.z*M_PI*0.0025);
                //dQFromAxisAndAngle(TrotP.rot, 0,1,0,xyz.x*M_PI*0.0025);
                //dQFromAxisAndAngle(TrotY.rot, 0,0,1,xyz.y*M_PI*0.0025);
                //Ttarg.trans = Trobot*TrotY*TrotP*TrotR*Trobot.inverse()*Ttarg.trans;

                //if(sqrt(lengthsqr3(_Tjoystick.trans + xyz*xyzstep - Ttarg.trans)) < 0.5)
                    _Tjoystick.trans += xyz*xyzstep;

        
                Transform Ttarg_backup = _Tjoystick;

                //Ttarg.trans += xyz*xyzstep;
                
                if(pIKSphere != NULL) pIKSphere->SetTransform(_Tjoystick);


                // descend on the jacobian
                int numsteps = 0;
                dReal preverror = 10000;
                while(1) {
                    // get the translation jacobian
                    Transform tEE = pmanip->GetEndEffectorTransform();

                    
                    if( (_Tjoystick.trans-tEE.trans).lengthsqr3() <= 0.002*0.002 )
                        // done
                        break;

                    robot->CalculateJacobian(pmanip->GetEndEffector()->GetIndex(), tEE.trans, &J[0]);
                      


                    multtrans_to2<dReal, dReal, dReal>(&J[0], &J[0], 3, numdofs, 3, JJt, false);

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
                    //RAVELOG_INFO("det: %f\n",det);
                    





                    //nullspace projection to avoid joint limits
                    ///////////////////////////////////////////////
                    // pf1 is transposed before mult
                    // rows of pf2 must equal rows of pf1
                    // pfres will be c1xc2 matrix
                    //getJplus
                    multtrans<dReal, dReal, dReal>(&J[0], invJJt, 3, numdofs, 3, &Jplus[0], false);
                    
                    //get JplusJ
                    mult<dReal, dReal, dReal>(&Jplus[0], &J[0], numdofs, 3, numdofs, &JplusJ[0],false);
                    
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
                    mult<dReal, dReal, dReal>(&JplusJ[0], &W[0], numdofs, numdofs, numdofs, &NullSpace[0],false);
    




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


                    Vector e = _Tjoystick.trans - tEE.trans, v;
                    dReal flength = RaveSqrt(e.lengthsqr3());

                    if(flength > (preverror + 0.01))
                    {                    
                        RAVELOG_INFO("Error Increasing, breaking out of gradient descent\n");
                        vfulljoints = vbackupjoints;
                        robot->SetJointValues(vfulljoints);
                        if(pIKSphere != NULL) pIKSphere->SetTransform(Ttarg_backup);
                        break;
                    }
                    else
                        preverror = flength;

                    RAVELOG_INFO("Position Error: %f\n",flength);

                    // take constant steps if length is big
                    if( flength > 0.01 )
                        e = e * (0.01 / flength);

                    transnorm3(v, invJJt, e);
                    dReal f = 0;
                    bool bsuccess = true;

                    for(int i = 0; i < numdofs; ++i) {
                        dReal fdir = J[0*numdofs+i] * v.x + J[1*numdofs+i] * v.y + J[2*numdofs+i] * v.z;
                        fdir = fdir + vnullspace_jointvals[i];
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
                       

                        vtempjoints[i] = vtempjoints[i] + fdir;

                        f += fdir * fdir;
                    }

                    if( !bsuccess )
                       break;

                    vfulljoints = vtempjoints;
                    robot->SetJointValues(vfulljoints);

                    if(robot->CheckSelfCollision())
                    {
                        RAVELOG_INFO("Collision\n");
                        bsuccess = false;
                        break;
                    }    
                    // gradient descend won't move it anymore
                    if( f < 1e-7 ) break;

                    //stuck, so back up to get unstuck
                    if(numsteps++ == 25)
                    {
                        RAVELOG_INFO("Stuck, backing up\n");
                        //vfulljoints = vbackupjoints;
                        //robot->SetJointValues(NULL,NULL,&vfulljoints[0]);
                        //if(pIKSphere != NULL) pIKSphere->SetTransform(Ttarg_backup);
                        break;
                    }
                }
            }

            /* Jt
            std::vector<dReal> J(3*numdofs);
            std::vector<dReal> dq(numdofs);
            memset(&J[0], 0, sizeof(dReal)*J.size());
            robot->CalculateJacobian(pmanip->GetEndEffector()->GetIndex(), Ttarg.trans, &J[0]);
            float temp;
            //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
            for(int j = 0; j < numdofs; j++)
            {   
                temp = 0;
                for(int k = 0; k < 3; k++)
                    temp += J[k*numdofs + j]*xyz[k]*xyzstep; //this is J-transpose

                dq[j] = temp;                                            
            }

            for(int i = 0; i < numdofs; i++)
                vfulljoints[i] += dq[i];
            */
            //if not using jacboian
            /*
            if(xyz.x != 0 || xyz.y !=0 || xyz.z != 0)
            {
                Ttarg.trans += xyz*xyzstep;                

                if( !pmanip->FindIKSolution(Ttarg, viksol, true) )
                {
                    RAVELOG_INFO("No IK Solution Found\n");
                }
                else
                {
                    for( int i = 0 ; i < viksol.size(); i++)
                        vfulljoints[i] = viksol[i];
                }

                robot->SetJointValues(NULL,NULL,&vfulljoints[0]);

            }
            */

            dReal jointstep = 0.01;
            vector<dReal> vfulljoints_backup =  vfulljoints;
            if(_bisWAM)
                if(rpy.x != 0 || rpy.y != 0 || rpy.z != 0 || elbowbend != 0)
                {
                    //roll
                    vfulljoints[6] += rpy.x*jointstep;

                    //pitch
                    vfulljoints[5] += rpy.y*jointstep;

                    //yaw
                    vfulljoints[4] += rpy.z*jointstep;       
                    
                    //elbow
                    vfulljoints[3] += elbowbend*jointstep;  
     

                    for(int i = 3; i < vfulljoints.size(); i++) 
                    {
                        if(vfulljoints[i] < vlowerlimits[i] || vfulljoints[i] > vupperlimits[i])
                        {
                            //consider wrap arounds
                            dReal lowerdist = fabs(vfulljoints[i] - vlowerlimits[i]);
                            dReal lowerdist2 = 2*M_PI - fabs((vfulljoints[i] - vlowerlimits[i]));
                            if(lowerdist > lowerdist2)
                                lowerdist = lowerdist2;

                            dReal upperdist = fabs(vfulljoints[i] - vupperlimits[i]);
                            dReal upperdist2 = 2*M_PI - fabs((vfulljoints[i] - vupperlimits[i]));
                            if(upperdist > upperdist2)
                                upperdist = upperdist2;
            
                            if(upperdist > lowerdist)
                                vfulljoints[i] = vlowerlimits[i]-0.01;
                            else
                                vfulljoints[i] = vupperlimits[i]-0.01;

                        }
                    }
                    //make sure robot is not in self collision
                    robot->SetJointValues(vfulljoints);
                    if(robot->CheckSelfCollision())
                        vfulljoints = vfulljoints_backup;
                    else
                    {
                        _Tjoystick.trans = pmanip->GetEndEffectorTransform().trans;
                        if(pIKSphere != NULL) pIKSphere->SetTransform(_Tjoystick);                        
                    }
                }
        }



        Transform Toffset;
        Toffset.trans.z = 0.19;
        Toffset.rot = Vector(0,-1,0,0); //camera points the wrong way
        Toffset = Ttarg*Toffset;


        if(joystickmode && _prevjoystickmode) //in starfox mode
        {
            GetEnv()->SetCamera(Toffset.trans, Toffset.rot);
        }
        else if(joystickmode && !_prevjoystickmode) //transition to starfox mode
        {
            _Tcamera = GetEnv()->GetCameraTransform();
            GetEnv()->SetCamera(Toffset.trans, Toffset.rot);
        }    
        else if(!joystickmode && _prevjoystickmode) //transition to bird's eye mode
            GetEnv()->SetCamera(_Tcamera.trans, _Tcamera.rot);


        robot->GetController()->SetDesired(&vfulljoints[0]);
        _prevjoystickmode = joystickmode;
    }
#endif

    //if( _bLog )
    //    Log(fElapsedTime);

    switch(_state) {
        case PS_Releasing:

            if( robot != NULL && robot->GetController() != NULL && robot->GetController()->IsDone()) {
                // turn off motors
                _state = PS_Nothing;
            }
            
            break;

        case PS_ClosingFingers:
            // wait for fingers to close
            if( robot != NULL && robot->GetController() != NULL) {

//                if( (timeGetTime() - nStartTime > nRealTrajTime) ) {
//                
//                    if( _bCloseFingersPhase2 && robot->GetController()->SupportsCmd("squeeze") ) {
//                        
//                        robot->GetController()->SimulationStep(0);
//                        
//                        // continue closing fingers
//                        vector<dReal> values;
//                        robot->GetDOFValues(values);
//                        
//                        const vector<int>& vactive = robot->GetActiveJointIndices();
//                        vector<dReal> closingdir;
//
//                        //need to take thumb into account (sometimes thumb is disabled)
//                        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
//                        for(int i = 0; i < vactive.size(); ++i) {
//                            for(int j = 0; j < pmanip->GetGripperIndices().size(); j++)
//                                if(vactive[i] == pmanip->GetGripperIndices()[j]) {
//                                    if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
//                                        closingdir.push_back(-1.0f);
//                                    else
//                                        closingdir.push_back(1.0f);
//                                    
//                                }
//                            RAVELOG_INFO("%d.  %f\n",i,closingdir[i]);    
//                        }
//                        
//                        // send the velocities (radian/s)
//                        string cmd = "squeeze ";
//                        char strtemp[20];
//                        for(size_t i = 0; i < closingdir.size(); ++i) {
//                            sprintf(strtemp, "%d %f ", pmanip->GetGripperIndices()[i], closingdir[i]*0.5f);
//                            cmd += strtemp;
//                        }
//                        if( !robot->GetController()->SendCommand(cmd.c_str()) ) {
//                            RAVELOG_INFO("SendCommand: \"%s\" failed.\n", cmd.c_str());
//                            _state = PS_Nothing;
//                        }
//                        else {
//                            nStartTime = timeGetTime();
//                            _state = PS_ClosingFingers2;
//                            RAVELOG_INFO("Starting second phase of closing fingers\n");
//                        }
//                    }
//                    else {
                        // regular controller, so stop
                        _state = PS_Nothing;
//                    }
//                }
            }

            break;
    }

    return 0;
}

void ManipulationProblem::EndState()
{
    switch(_state) {
        case PS_Releasing:
        case PS_ClosingFingers:
#ifdef INTELCONTROLLER
            if( robot != NULL && dynamic_cast<HRP2Controller*>(robot->GetController()) != NULL ) {
                dynamic_cast<HRP2Controller*>(robot->GetController())->EnableHandMotors(_bRightHand, false);
            }
#endif
            break;
    }

    _state = PS_Nothing;
}

/*
int ManipulationProblem::PlayBackPointTraj()
{
    RAVELOG_DEBUG("Starting PlaybackPointTraj...\n");

    const char* delim = " \r\n\t";
    KinBodyPtr ptarget;
    char* p = strtok(NULL, delim);
    ifstream trajfile;

    while(p != NULL ) {
        if( stricmp(p, "file") == 0 ){
            p = strtok(NULL, delim);
            RAVELOG_DEBUG("ManipulationProblem: reading trajectory: %s\n", p);
            trajfile.open(p,ios::in);
        }
        else break;
        p = strtok(NULL, delim);
    }
    if(!trajfile.is_open())
    {
        RAVELOG_INFO("PlaybackPointTraj ERROR: Invalid file\n");
        return -1;
    }

    TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(robot->GetDOF());
    Trajectory::TPOINT point;
    point.q.resize(robot->GetDOF());

    //add currnet position
    robot->GetDOFValues(point.q);
    ptraj->AddPoint(point);

    bool bdone = false;
    int i = 0;
    while(1)
    {
        for(i = 0; i < robot->GetDOF(); i++)    
        {
            if(!trajfile.eof())    
                trajfile >> point.q[i];
            else
            {
                bdone = true;
                break;
            }
        }
        //add point to the traj if not done
        if(bdone)         
            break;
        else
            ptraj->AddPoint(point);

    } 


    stringstream s;
    s << L"traj: ";
    for(i = 0; i < ptraj->GetPoints().size(); i++)
    {            
        for(int j = 0; j < robot->GetDOF(); j++)
            s << ptraj->GetPoints()[i].q[j] << " ";

        s << endl;
    }   

    RAVELOG_INFO(s.str());
 
    ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
    robot->SetActiveMotion(ptraj);

    return 0;
}
*/

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

    //const char* delim = " \r\n\t";
    KinBodyPtr ptarget;
    //char* p = strtok(NULL, delim);
//    while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));    
        }
        else if( stricmp(p.c_str(), "handjoints") == 0 ) {
            if( pmanip.get() != NULL )
                vhandjointvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vhandjointvals.size(); ++i)
                cmd >> vhandjointvals[i];// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "setjoint") == 0 ) {
            int ind;// = atoi(pindex);
            cmd >> ind;
            if( ind >= 0 && ind < (int)values.size() ) {
                cmd >> values[ind];// = (dReal)atof(pvalue);
            }
//             char* pindex = strtok(NULL, delim);
//             char* pvalue = strtok(NULL, delim);
// 
//             if( pindex != NULL && pvalue != NULL ) {
//                 int ind = atoi(pindex);
//                 if( ind >= 0 && ind < (int)values.size() ) {
//                     values[ind] = (dReal)atof(pvalue);
//                 }
//             }
        }
        //this is useful b/c we always get the nearest IK to the current joints
        else if( stricmp(p.c_str(), "armjoints") == 0 ) {
            if( pmanip != NULL )
                varmjointvals.resize(pmanip->GetArmIndices().size());
            for(size_t i = 0; i < varmjointvals.size(); ++i)
                cmd >> varmjointvals[i];// = (dReal)atof(strtok(NULL, delim));
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

        //p = strtok(NULL, delim);
    }

    if( pmanip == NULL )
        return vector<dReal>();

    Transform handTr(handTm);

    //robot->GetDOFValues(values);
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
    //pmanip->_pIkSolver->Solve(handTr, bCheckCollision,qSolutions);
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

/*
//GetIKBias: This function is not updated to work with the current version of rave. Need to replace dQFromAxisAndAngle.
vector<dReal> ManipulationProblem::GetIKBias()
{

    RAVELOG_INFO("GetIKBias: This function is not updated to work with the current version of rave. Need to replace dQFromAxisAndAngle.\n");


    RAVELOG_DEBUG("Starting GetIKBias...\n");

    TransformMatrix handTm;

    vector<dReal> vhandjointvals, oldvalues, values;
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    
    
    robot->GetDOFValues(oldvalues);
    values = oldvalues;

    Vector updir;
    Transform ikoffset;
    bool bIKOffset = false;
    bool bDraw = false;

    const char* delim = " \r\n\t";
    KinBodyPtr ptarget = NULL;
    char* p = strtok(NULL, delim);
    while(p != NULL ) {
        if( stricmp(p, "trans") == 0 ){
            handTm.trans.x = (dReal)atof(strtok(NULL, delim));
            handTm.trans.y = (dReal)atof(strtok(NULL, delim));
            handTm.trans.z = (dReal)atof(strtok(NULL, delim));
        }
        else if ( stricmp(p, "updir") == 0 ){
            updir.x = (dReal)atof(strtok(NULL, delim));
            updir.y = (dReal)atof(strtok(NULL, delim));
            updir.z = (dReal)atof(strtok(NULL, delim));
        }
        else if ( stricmp(p, "ikoffset") == 0 ){
            ikoffset.trans.x = (dReal)atof(strtok(NULL, delim));
            ikoffset.trans.y = (dReal)atof(strtok(NULL, delim));
            ikoffset.trans.z = (dReal)atof(strtok(NULL, delim));
            bIKOffset = true;
        }
        else if ( stricmp(p, "draw") == 0 ){
            bDraw = true;
        }
        else break;
        p = strtok(NULL, delim);
    }

    if( pmanip == NULL )
        return vector<dReal>();


    Transform ikbackup = robot->GetActiveManipulator()->tGrasp;
    if(bIKOffset) {
        robot->GetActiveManipulator()->tGrasp = ikoffset;

        // have to reset because tGrasp changed
        if( robot->GetActiveManipulator()->HasIKSolver() && wcsicmp(robot->GetActiveManipulator()->GetIKSolverName().c_str(), L"WAM7") == 0 )
            robot->GetActiveManipulator()->InitIKSolver(0);
    }


    Transform handTr(handTm);
    

    //calculate vector pointing away from robot to point
    Vector away_dir = handTr.trans - robot->GetTransform().trans;
    away_dir -= updir * dot3(updir, away_dir);
    normalize3(away_dir,away_dir);

        //stringstream s2;
        //s2 << L"dir: " << away_dir.x << " "<<   away_dir.y << " " <<  away_dir.z << endl;
               
        //s2 << L"quat: " << handTr.rot.x << " " << handTr.rot.y << " " << handTr.rot.z << " " << handTr.rot.w << endl;
        //RAVEPRINT(s2.str().c_str());
    if(bDraw)
        GetEnv()->plot3(RaveVector<float>(handTr.trans),1,0,0.004f, RaveVector<float>(0,0,1));

    Vector away_dir_backup = away_dir;

    Vector vRotAxis;
    Transform tRot;
    cross3(vRotAxis, updir, away_dir);

    vector<dReal> q1;
    q1.resize(pmanip->GetArmIndices().size());

    vector<dReal> returnvals; 
    //approaches from the side and bottom and top
    for(int i = 0; i < 6; i++) {
        if(i >= 1 && i <= 3)
            dQFromAxisAndAngle(tRot.rot, updir.x,updir.y,updir.z, i*M_PI/2);
        else if(i > 3)
        {

            cross3(vRotAxis, updir, away_dir_backup);
            normalize3(vRotAxis, vRotAxis); 
            dQFromAxisAndAngle(tRot.rot, vRotAxis.x,vRotAxis.y,vRotAxis.z, M_PI/2 + i*M_PI);
        }
        
        away_dir = tRot * away_dir_backup;
        cross3(vRotAxis, updir, away_dir);
        normalize3(vRotAxis, vRotAxis);


        dQFromAxisAndAngle(handTr.rot, vRotAxis.x, vRotAxis.y, vRotAxis.z, RaveAcos(CLAMP_ON_RANGE<dReal>(dot3(updir, away_dir),-1.0f,1.0f)));
        

        if( !pmanip->FindIKSolution(handTr, q1, false) )
        {
            RAVELOG_INFO("No IK solution found\n");   
            robot->SetJointValues(NULL,NULL,&oldvalues[0]);
            if(bDraw)
                GetEnv()->plot3(RaveVector<float>(handTr.trans-0.1f*away_dir),1,0,0.004f, RaveVector<float>(1,0,0));
        }
        else
        {
            stringstream s;
            s << L"ik sol: ";
            for(size_t j = 0; j < q1.size(); j++)
                s << q1[j] << " ";
            s << endl;
            RAVEPRINT(s.str().c_str());

            returnvals.push_back(away_dir.x);
            returnvals.push_back(away_dir.y);
            returnvals.push_back(away_dir.z);

            if(bDraw)
                GetEnv()->plot3(RaveVector<float>(handTr.trans-0.1f*away_dir),1,0,0.004f, RaveVector<float>(0,1,0));

//            std::vector<dReal> jointvals;
//            robot->GetDOFValues(jointvals);
//            for(int i = 0; i < pmanip->GetArmIndices().size(); i++) {
//                jointvals[pmanip->GetArmIndices()[i]] = q1[i];
//            }
//            robot->GetController()->SetDesired(&jointvals[0]);
        }
    }

 
    if(bIKOffset) {
        robot->GetActiveManipulator()->tGrasp = ikbackup;
        // have to reset because tGrasp changed
        if( robot->GetActiveManipulator()->HasIKSolver() && wcsicmp(robot->GetActiveManipulator()->GetIKSolverName().c_str(), L"WAM7") == 0 )
            robot->GetActiveManipulator()->InitIKSolver(0);
    }

    return returnvals;

}
*/

// int ManipulationProblem::DrawPoint()
// {
//     RAVELOG_DEBUG("Starting Drawpoint...\n");
// 
//     const char* delim = " \r\n\t";
// 
//     Vector color,trans;
//     dReal size;
// 
//     char* p = strtok(NULL, delim);
//     while(p != NULL ) {
//         if( stricmp(p, "trans") == 0 ) {
//             trans.x = (dReal)atof(strtok(NULL, delim));
//             trans.y = (dReal)atof(strtok(NULL, delim));
//             trans.z = (dReal)atof(strtok(NULL, delim));
//             
//         }
//         else if( stricmp(p, "color") == 0 ) {
//             color.x = (dReal)atof(strtok(NULL, delim));
//             color.y = (dReal)atof(strtok(NULL, delim));
//             color.z = (dReal)atof(strtok(NULL, delim));
//         }
//         else if( stricmp(p, "size") == 0 ) { 
//             size = (dReal)atof(strtok(NULL, delim));
//         }
//         else break;
//         p = strtok(NULL, delim);
//     }
// 
//     
//     GetEnv()->plot3(RaveVector<float>(trans),1,0,size, RaveVector<float>(color));
// 
// 
//     return 0;
// 
// }




std::vector<int> ManipulationProblem::MultiColCheck(istream& cmd)
{
    RAVELOG_DEBUG("Starting MultiColCheck...\n");
    
    std::vector<int> out;
    //const char* delim = " \r\n\t";

    int numtms;
    std::vector<Transform> vtransforms;

    KinBodyPtr pbody;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    TransformMatrix handTm;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "body") == 0 ) {
            cmd >> p;//= strtok(NULL, delim);
            pbody = GetEnv()->GetKinBody(p);   
            
        }
        else if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;//= atoi(strtok(NULL, delim));
            for(int i = 0; i < numexcluded; i++)
            {
                cmd >> p;//strtok(NULL, delim);
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
            cmd >> numtms;// = atoi(strtok(NULL, delim));
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> vtransforms[i].rot.x;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.y;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.z;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.w;//= atof(strtok(NULL, delim));
                
                cmd >> vtransforms[i].trans.x;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].trans.y;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].trans.z;//= atof(strtok(NULL, delim));
            }
    
        }
        else if( stricmp(p.c_str(), "transforms12") == 0 ) {
            cmd >> numtms;// = atoi(strtok(NULL, delim));
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
                vtransforms[i] = (Transform) handTm;
            }

        }
        //p = strtok(NULL, delim);
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
    //const char* delim = " \r\n\t";

    int numtests;
    KinBodyPtr pbody;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    std::vector<dReal> vjointvals;
    std::vector<std::vector<dReal> > vvtestvals;

    robot->GetDOFValues(vjointvals);
    int numdof = vjointvals.size();

    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;// = atoi(strtok(NULL, delim));
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
            cmd >> numtests;// = atoi(strtok(NULL, delim));
            vvtestvals.resize(numtests);
            out.resize(numtests);
            for(int i = 0; i < numtests; i++)
            {   
                vvtestvals[i].resize(numdof);
                for(int j = 0; j < numdof; j++)
                    cmd >> vvtestvals[i][j];// = (dReal) atof(strtok(NULL, delim));                
                
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
        //p = strtok(NULL, delim);
    }
    
    int col = 0;
    int selfcol = 0;
    int bodyindex = 0;

    CollisionReportPtr report;
    report.reset(new CollisionReport);

    for(int i = 0; i < vvtestvals.size(); i++)
    {    
        robot->SetJointValues(vvtestvals[i],true);
        col = GetEnv()->CheckCollision(KinBodyConstPtr(robot),vexcluded,vlinkexcluded,report);
        selfcol = robot->CheckSelfCollision();
    
        bodyindex = 0;
        if( report->plink1 != NULL && report->plink1->GetParent() != robot )
            bodyindex = report->plink1->GetParent()->GetEnvironmentId();
        if( report->plink2 != NULL && report->plink2->GetParent() != robot )
            bodyindex = report->plink2->GetParent()->GetEnvironmentId();

        out[i] = (bodyindex<<1) | selfcol; //0 means no collision, 1 means self collision, >1 is id of body collided with: id = val >> 1
    }

    robot->SetJointValues(vjointvals,true);
    return out;
}

/*
//CheckTolerance: This function is not updated to work with the latest openrave. Need to fix colission-checker switching.
bool ManipulationProblem::CheckTolerance()
{
    RAVELOG_INFO("CheckTolerance: This function is not updated to work with the latest openrave. Need to fix colission-checker switching.\n");
/*
    RAVELOG_DEBUG("Starting CheckTolerance...\n");
    dReal tolerance;
    KinBodyPtr pbody;
    KinBodyPtr ptemp;
    bool retval = false;
    int numexcluded = 0;
    std::set<KinBodyPtr> vexcluded;
    std::set<KinBody::Link *> vlinkexcluded;
    const char* delim = " \r\n\t";
    char* p = strtok(NULL, delim);
    while(p != NULL ) {
        if( stricmp(p, "body") == 0 ) {
            p = strtok(NULL, delim);
            pbody = GetEnv()->GetKinBody(p);   
            
        }
        else if( stricmp(p, "excluded") == 0 ) {
            numexcluded = atoi(strtok(NULL, delim));
            for(int i = 0; i < numexcluded; i++)
            {
                p = strtok(NULL, delim);
                ptemp = GetEnv()->GetKinBody(p);
                if(ptemp != NULL)
                    vexcluded.insert(ptemp);   
                else
                {
                    RAVELOG_INFO("ERROR: Unknown excluded body\n");   
                    return false;
                }      
            }
        }
        else if( stricmp(p, "tolerance") == 0 ) {
            tolerance = (dReal)atof(strtok(NULL, delim));
        }
        else break;
        p = strtok(NULL, delim);
    }
    
    if(pbody == NULL)
    {       
        RAVELOG_INFO("ERROR: Unknown/unspecified body\n");   
        return false;
    }
    CollisionCheckers ccprev = GetEnv()->GetCollisionChecker();
    GetEnv()->SetCollisionChecker(CC_PQP);
    retval = GetEnv()->CheckCollision(pbody,vexcluded,vlinkexcluded,tolerance);
    GetEnv()->SetCollisionChecker(ccprev);
    return retval;
}
*/


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

        //p = strtok(NULL, delim);
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
    //const char* delim = " \r\n\t";
    
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
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

        //p = strtok(NULL, delim);
    }
    
    robot->SetActiveDOFs(pmanip->GetArmIndices());
    JitterActiveDOF(robot);
    
    TrajectoryBasePtr ptraj = MoveArm(activejoints, goals,pmanip,KinBodyPtr());
    if( ptraj == NULL )
        return -1;
    if(!btestmode)
        SetTrajectory(ptraj);
    //delete ptraj;
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
    //const char* delim = " \r\n\t";
    KinBodyPtr ptarget;
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "transform") == 0 ){
            cmd >> tmEE.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[10];// = (dReal)atof(strtok(NULL, delim));    
            cmd >> tmEE.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.z;// = (dReal)atof(strtok(NULL, delim));
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

        //p = strtok(NULL, delim);
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

        //p = strtok(NULL, delim);
    }
    //report.reset(new CollisionReport());
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


/*
class PRMParameters : public PlannerBase::PlannerParameters
{
public:
    PRMParameters() : pExtraParameters(NULL) {}
    
    void* pExtraParameters;
    
protected:
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<extra>" << pExtraParameters << "</extra>" << endl;
        return !!O;
    }
    virtual bool endElement(void *ctx, const char *name)
    {
        if( stricmp(name, "extra") == 0 )
            _ss >> pExtraParameters;
        else
            return PlannerParameters::endElement(ctx, name);

        return false;
    }
};

int ManipulationProblem::MoveManipulatorPRM()
{
    RAVELOG_DEBUG("Starting MoveManipulatorPRM...\n");

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    std::vector<dReal> goals;
    std::vector<int> activejoints;
    bool bUseHand = false;
    bool btestmode = false;
    PRMParameters params;
    const char* delim = " \r\n\t";
    params.bHasWorkspaceGoal = false; //hijacking this
    char* p = strtok(NULL, delim);
    params.pExtraParameters = NULL;
    while(p != NULL ) {
        if( stricmp(p, "leftleg") == 0 ) {
            
            robot->SetActiveManipulator(1);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p, "rightleg") == 0 ) {
            
            robot->SetActiveManipulator(2);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p, "head") == 0 ) {
            
            robot->SetActiveManipulator(0);
            pmanip = robot->GetActiveManipulator();
        }
        else if( stricmp(p, "armvals") == 0 ) {
            for(size_t i = 0; i < robot->GetActiveManipulator()->GetArmIndices().size(); i++)
            {            
                goals.push_back((dReal)atof(strtok(NULL, delim)));
                activejoints.push_back(pmanip->GetArmIndices()[i]);
            }   
            
        }
        else if( stricmp(p, "handvals") == 0 ) {
            bUseHand = true;
            for(size_t i = 0; i < robot->GetActiveManipulator()->GetGripperIndices().size(); i++)
            {
                goals.push_back((dReal)atof(strtok(NULL, delim)));
                activejoints.push_back(pmanip->GetGripperIndices()[i]);
            }
        }
        else if( stricmp(p, "useoldgraph") == 0 ) {
            params.pExtraParameters = _pPRMGraph;            

        }
        else if( stricmp(p, "savegraph") == 0 ) {
            params.bHasWorkspaceGoal = true;            

        }
        else if( stricmp(p, "test") == 0 ) {
            btestmode = true;
        }            
        else break;
        p = strtok(NULL, delim);
    }
    
    robot->SetActiveDOFs(pmanip->GetArmIndices());
    JitterActiveDOF(robot);
    

    if( goals.size() != activejoints.size() ) {
        RAVELOG_INFO("Active joints unequal size\n");
        return -1;
    }


    if( GetEnv()->CheckCollision(robot) ) {
        RAVELOG_INFO("Arm in collision in initial position\n");
    }
    
    
    
    std::vector<dReal> pzero(robot->GetActiveDOF());
    robot->GetActiveDOFValues(pzero);

    // make sure the initial and goal configs are not in collision
    robot->SetActiveDOFValues(goals, true);

    // jitter only the manipulator! (jittering the hand causes big probs)
    robot->SetActiveDOFs(pmanip->GetGripperIndices());
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_INFO("Jitter failed\n");
        robot->SetActiveDOFs(activejoints);
        return -1;
    }

    // restore
    robot->SetActiveDOFs(activejoints);

    robot->GetActiveDOFValues(params.vgoalconfig);
    robot->SetActiveDOFValues(pzero);
    
    // jitter again for initial collision
    if( !JitterActiveDOF(robot) )
        return -1;

    PlannerBase* PRMplanner = GetEnv()->CreatePlanner("PRM");
    if( PRMplanner == NULL ) {
        RAVELOG_INFO("failed to create PRMs\n");
        return -1;
    }
    
    robot->GetActiveDOFValues(params.vinitialconfig);
    TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(robot->GetActiveDOF());

//    if( wcsicmp(manip.strIkSolver.c_str(), L"WAM7") == 0 ) {
//        s_pWAMRobot = robot;
//        params.pConstraintFn = WAMConstraintFn;
//    }

    ptraj->Clear();
    
    if(params.bHasWorkspaceGoal)
        params.pExtraParameters = &_pPRMGraph; // PlanPath will fill this with the prm grah

    RAVELOG_INFO("starting planning\n");
    if( !PRMplanner->InitPlan(robot, &params) ) {
        RAVELOG_INFO("InitPlan failed\n");
        //delete PRMplanner;
        return -1;
    }

    if( !PRMplanner->PlanPath(ptraj) ) {
        RAVELOG_INFO("PlanPath failed\n");
        //delete PRMplanner;
        return -1;
    }
    RAVELOG_INFO("finished planning\n");
        
    //delete PRMplanner;
    if(!btestmode)
        SetTrajectory(ptraj);
    //delete ptraj;
    return 0;
}
*/

/*
//RAVELOG_INFO("CheckContactStability: This function is not updated to work with the latest openrave. Need to fix colission-checker switching.
int ManipulationProblem::CheckContactStability()
{
    RAVELOG_INFO("CheckContactStability: This function is not updated to work with the latest openrave. Need to fix colission-checker switching.\n");

    RAVELOG_DEBUG("Starting CheckContactStability...\n");
    stringstream s; 
    KinBodyPtr ptarget = NULL;
    wstring targetname;
    Vector direction;
    bool bdraw = false;
    float mu = -1;
    const char* delim = " \r\n\t";
    
    char* p = strtok(NULL, delim);
    while(p != NULL ) {
        if( stricmp(p, "target") == 0 ) {
            targetname = strtok(NULL, delim);
            ptarget = GetEnv()->GetKinBody(targetname);
        }
        else if( stricmp(p, "draw") == 0 ) {
            bdraw = true;
        }
        else if( stricmp(p, "approachdir") == 0 ) {
            direction.x = (dReal)atof(strtok(NULL, delim));
            direction.y = (dReal)atof(strtok(NULL, delim));
            direction.z = (dReal)atof(strtok(NULL, delim));
            normalize3(direction,direction);
        }
        else if( stricmp(p, "friction") == 0 ) {
            mu = (dReal)atof(strtok(NULL, delim));
        }
        else break;
        p = strtok(NULL, delim);
    }
    
    if(ptarget == NULL)
    {
        RAVELOG_INFO("ManipulationProblem::CheckContactStability - Error: Target not specified.\n");
        return -1;
    }
    GetEnv()->SetCollisionChecker(CC_PQP);
    GetEnv()->SetCollisionOptions(0);


    if(!GetEnv()->CheckCollision(robot,ptarget))
    {
        RAVELOG_INFO("ManipulationProblem::CheckContactStability - Error: Robot is not colliding with the target.\n");
        GetEnv()->SetCollisionChecker(CC_ODE);
        return -1;
    }

    if(mu < 0)
    {
        RAVELOG_INFO("ManipulationProblem::CheckContactStability - Error: Friction coefficient is invalid.\n");
        GetEnv()->SetCollisionChecker(CC_ODE);
        return -1;
    }

    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();


    int ibaselink = 0;
    if(pmanip->pBase != NULL)
    {
        ibaselink = pmanip->pBase->GetIndex();
    }
    RAVELOG_INFO("Using link %d as base link.\n",ibaselink);


    vector<dReal> closingdir;

    //make sure we get the right closing direction and don't look at irrelevant joints
    bool handjoint;
    for(size_t i = 0; i < robot->GetDOF(); ++i) 
    {
        handjoint = false;
        for(size_t j = 0; j < pmanip->GetGripperIndices().size(); j++)
        {
            if(i == pmanip->GetGripperIndices()[j])
            {
                handjoint = true;
                if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
                    closingdir.push_back(-1.0f);
                else
                    closingdir.push_back(1.0f);

                break;
            }
        }
        
        if(!handjoint)
            closingdir.push_back(0);
    }

    s.clear();
    s << L"closing dirs: ";
    for(int q = 0; q < closingdir.size(); q++)
        s << closingdir[q] << " ";
    s << endl;
    RAVEPRINT(s.str().c_str());

    // calculate the contact normals using the Jacobian
    int numdofs = robot->GetDOF();
    std::vector<dReal> J(3*numdofs);
    Vector deltaxyz;
    float temp;
    Vector vnormalpart;
    Vector vtangentpart;
   
    CollisionReport report;
    std::vector<KinBody::LinkPtr> vbodies = robot->GetLinks();
    
    for(int ilink = 0; ilink < vbodies.size(); ilink++)
    {
        if( GetEnv()->CheckCollision(vbodies[ilink],ptarget, &report) ) 
        {
         
            RAVELOG_INFO("contact %S:%S with %S:%S\n", report->plink1->GetParent()->GetName(), report->plink1->GetName(), report->plink2->GetParent()->GetName(), report->plink2->GetName());

            Transform linkTm = vbodies[ilink]->GetTransform();
            Transform pointTm;
            int icont=0;
            
            for(int i = 0; i < report->contacts.size(); i++) 
            {  
                report->contacts[i].norm = -report->contacts[i].norm;
                icont++;
                //check if this link is the base link, if so there will be no Jacobian
                if(ilink == ibaselink)
                {
                    deltaxyz = direction;

                }
                else
                {   
                    //calculate the jacobian for the contact point as if were part of the link
                    pointTm.trans = report->contacts[i].pos;
                    //pointTm.trans = pointTm.trans - linkTm.trans;
                    //pointTm = linkTm * pointTm;
                    memset(&J[0], 0, sizeof(dReal)*J.size());
                    robot->CalculateJacobian(vbodies[ilink]->GetIndex(), pointTm.trans, &J[0]);
                    
                    //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                    for(int j = 0; j < 3; j++)
                    {   
                        temp = 0;
                        for(int k = 0; k < numdofs; k++)
                            temp += J[j*numdofs + k]*0.01f*closingdir[k];
                                                    
                        if( j == 0)
                            deltaxyz.x  = temp;
                        else if( j == 1)
                            deltaxyz.y = temp;
                        else if( j == 2)
                            deltaxyz.z = temp;
                    }
                }
                
                //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                //so treat it as if it were part of the base link
                if(lengthsqr3(deltaxyz) < 0.000000001f)
                    deltaxyz = direction;
                
                normalize3(deltaxyz,deltaxyz);

                s.clear();
                s << L"link " << ilink << " delta XYZ: ";
                for(int q = 0; q < 3; q++)
                    s << deltaxyz[q] << " ";
                s << endl;
                RAVEPRINT(s.str().c_str());

                RAVELOG_INFO("number of contacts: %d\n", icont);

                //determine if contact is stable
                bool bstable = true;
                //if angle is obtuse, can't be in friction cone
                if (RaveAcos(dot3(report->contacts[i].norm,deltaxyz)) > M_PI/2.0f)
                    bstable = false;
                else
                {
                    vnormalpart = dot3(report->contacts[i].norm,deltaxyz)*report->contacts[i].norm;
                    vtangentpart = deltaxyz - vnormalpart;
                    //check if tangent force is outside friction cone
                    if( mu*sqrt(lengthsqr3(vnormalpart)) < sqrt(lengthsqr3(vtangentpart)) )
                        bstable = false;
  
                }


                if(bdraw)
                {
                    if(bstable)
                        GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(0,1,0) );
                    else
                        GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );

                    GetEnv()->plot3(RaveVector<float>(report->contacts[i].pos + 0.02*report->contacts[i].norm), 1, 0, 0.004f, RaveVector<float>(0,0,1) );
                    GetEnv()->plot3(RaveVector<float>(report->contacts[i].pos + 0.02*deltaxyz), 1, 0, 0.004f, RaveVector<float>(1,1,0) );
                }

            }
        }
    }

    GetEnv()->SetCollisionChecker(CC_ODE);
    return 0;

}
*/

/*
//CheckSlidingContact: This function is not updated to work with the latest openrave. Need to fix colission-checker switching
int ManipulationProblem::CheckSlidingContact()
{
    RAVELOG_INFO("CheckSlidingContact: This function is not updated to work with the latest openrave. Need to fix colission-checker switching.\n");
/*
    const char* delim = " \r\n\t";
    bool bdraw = false;
    char* p = strtok(NULL, delim);

    std::vector<dReal> vprevvals(robot->GetActiveDOF());

    while(p != NULL ) {
        if( stricmp(p, "draw") == 0 ) {
            bdraw = true;
        }
        else if( stricmp(p, "prevvals") == 0 ) {
            for(int i = 0; i < robot->GetActiveDOF(); i++)
            {
                vprevvals[i] = (dReal)atof(strtok(NULL, delim));
            }
        }
        else break;
        p = strtok(NULL, delim);
    }


    stringstream s; 


    dReal mu = 0.3f;

    CollisionCheckers ccprev = GetEnv()->GetCollisionChecker();
    GetEnv()->SetCollisionChecker(CC_PQP);
    GetEnv()->SetCollisionOptions(0);


    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();


    if(!GetEnv()->CheckCollision(robot))
    {
        RAVELOG_INFO("ManipulationProblem::CheckSlidingContact - Error: Robot is not colliding.\n");
        GetEnv()->SetCollisionChecker(ccprev);
        return -1;
    }

    if( pmanip == NULL ) {
        RAVELOG_INFO("ManipulationProblem::CheckSlidingContact - Error: no active manipulator\n");
        GetEnv()->SetCollisionChecker(ccprev);
        return true;
    }

    int ibaselink = 0;
    if(pmanip != NULL && pmanip->pBase != NULL)
    {
        ibaselink = pmanip->pBase->GetIndex();
    }
    RAVELOG_DEBUG("Using link %d as base link.\n",ibaselink);


    // calculate the contact normals using the Jacobian
    int numdofs = robot->GetDOF();
    std::vector<dReal> J(3*numdofs);
    Vector deltaxyz;
    float temp;
    Vector vnormalpart;
    Vector vtangentpart;
    Vector direction(0,0,1);
    std::vector<dReal> vtorque(robot->GetDOF(),0);
    std::vector<dReal> vactivedofvals;
    robot->GetActiveDOFValues(vactivedofvals);

    for(int i =0; i < robot->GetActiveDOF(); i++)
        vtorque[robot->GetActiveJointIndex(i)] = vactivedofvals[i] - vprevvals[i];



    bool retval = false;    

    CollisionReport report;
    std::vector<KinBody::LinkPtr> vbodies = robot->GetLinks();
    
    for(int ilink = 0; ilink < vbodies.size(); ilink++)
    {
        if( GetEnv()->CheckCollision(vbodies[ilink], &report) ) 
        {
         
            RAVELOG_DEBUG("contact %S:%S with %S:%S\n", report->plink1->GetParent()->GetName(), report->plink1->GetName(), report->plink2->GetParent()->GetName(), report->plink2->GetName());

            Transform linkTm = vbodies[ilink]->GetTransform();
            //s.str() = L"Link Tm: ";
            //s << linkTm.trans.x << " " << linkTm.trans.y << " " << linkTm.trans.z << " " << linkTm.rot.x << " " << linkTm.rot.y << " " << linkTm.rot.z << " " << linkTm.rot.w << " ";
            //s << endl;
            //RAVEPRINT(s.str().c_str());


            Transform pointTm;
            int icont=0;
            
            for(int i = 0; i < report->contacts.size(); i++) 
            {  
                icont++;
               
                report->contacts[i].norm = -report->contacts[i].norm;
                //check if this link is the base link, if so there will be no Jacobian
                if(ilink == ibaselink)
                {
                    deltaxyz = direction;

                }
                else
                {   
                    //calculate the jacobian for the contact point as if were part of the link
                    pointTm.trans = report->contacts[i].pos;
   
                    memset(&J[0], 0, sizeof(dReal)*J.size());
                    robot->CalculateJacobian(vbodies[ilink]->GetIndex(), pointTm.trans, &J[0]);
                    //GetEnv()->plot3(pointTm.trans, 1, 0, 0.004f, Vector(0.5,0.5,0.5) );
                    
                    //s.str() = L"Jacobian 2: ";
                    //for(int q = 0; q < J.size(); q++)
                    //        s << J[q] << " ";
                    //s << endl;
                    //RAVEPRINT(s.str().c_str());

                    //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                    for(int j = 0; j < 3; j++)
                    {   
                        temp = 0;
                        for(int k = 0; k < numdofs; k++)
                            temp += J[j*numdofs + k]*vtorque[k];
                                                    
                        if( j == 0)
                            deltaxyz.x  = temp;
                        else if( j == 1)
                            deltaxyz.y = temp;
                        else if( j == 2)
                            deltaxyz.z = temp;
                    }
                }
                
                //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                //so treat it as if it were part of the base link
                if(lengthsqr3(deltaxyz) < 0.000000001f)
                    deltaxyz = direction;
                
                normalize3(deltaxyz,deltaxyz);

                s.str().erase();
                s << L"link " << ilink << " delta XYZ: ";
                for(int q = 0; q < 3; q++)
                    s << deltaxyz[q] << " ";
                s << endl;
                RAVELOG_DEBUG(s.str().c_str());

                RAVELOG_DEBUG("number of contacts: %d\n", icont);

                //determine if contact is stable
                bool bstable = true;
                //if angle is obtuse, can't be in friction cone
                if (RaveAcos(dot3(report->contacts[i].norm,deltaxyz)) > M_PI/2.0f)
                    bstable = false;
                else
                {
                    vnormalpart = dot3(report->contacts[i].norm,deltaxyz)*report->contacts[i].norm;
                    vtangentpart = deltaxyz - vnormalpart;
                    //check if tangent force is outside friction cone
                    if( mu*sqrt(lengthsqr3(vnormalpart)) < sqrt(lengthsqr3(vtangentpart)) )
                        bstable = false;
  
                }


                if(bdraw)
                {
                    if(bstable)
                        GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );
                    else
                        GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(0,1,0) );

                    GetEnv()->drawarrow(RaveVector<float>(report->contacts[i].pos),RaveVector<float>(report->contacts[i].pos + 0.02f*report->contacts[i].norm), 0.004f, RaveVector<float>(1,0,1) );
                    GetEnv()->drawarrow(RaveVector<float>(report->contacts[i].pos),RaveVector<float>(report->contacts[i].pos + 0.02f*deltaxyz), 0.004f, RaveVector<float>(1,1,0) );
                }
                
                if(bstable)
                    retval = true;
            }
        }
    }

    GetEnv()->SetCollisionChecker(ccprev);
    return retval;

}
*/

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

    //const char* delim = " \r\n\t";
    
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "readfile") == 0 ) {
            cmd >> filename;//make = strtok(NULL, delim);
            bReadFile = true;
        }
        else if( stricmp(p.c_str(), "genfile") == 0 ) {
            cmd >> filename;// = strtok(NULL, delim);
            bGenFile = true;
        }
        else if( stricmp(p.c_str(), "numtests") == 0 ) {
            cmd >> num_itrs;// = (int)atof(strtok(NULL, delim));
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }
        //p = strtok(NULL, delim);
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
        
        error = sqrt(lengthsqr4(twrist.rot - twrist_out.rot));
        minuserr = sqrt(lengthsqr4(twrist.rot + twrist_out.rot));
        if(minuserr < error)
            error = minuserr;
        
        error += sqrt(lengthsqr3(twrist.trans - twrist_out.trans));

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

/*
//TestAllGrasps: This function is not updated to work with the current version of rave. Need to replace ODE calls.
int ManipulationProblem::TestAllGrasps(string& response)
{
    RAVELOG_INFO("TestAllGrasps: This function is not updated to work with the current version of rave. Need to replace ODE calls.\n");

    RAVELOG_DEBUG("TestingAllGrasps...\n");

    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;

//    RAVELOG_INFO("creating fat\n");
//    for(int i = 0; i < 1; ++i) {
//        // create fat body
//        KinBodyPtr pfatbody = GetEnv()->CreateKinBody();
//        if( !pfatbody->Init("data/intel/mug1fat.kinbody.xml", NULL) ) {
//            RAVELOG_INFO("failed to open file \n");
//            continue;
//        }
//        pfatbody->SetName("temp"); // should be %Sfat
//        if( !GetEnv()->AddKinBody(pfatbody) ) {
//            RAVELOG_INFO("failed to add body\n");
//            delete pfatbody;
//            continue;
//        }
//        
//        pfatbody->SetTransform(Transform(Vector(1,0,0,0),Vector(0,100,0)));
//    }
//    return 0;

    vector<dReal> vgrasps;

    vector<int> vPreshapeIndices, vPreshapeIndicesRobot, vHandJoints; // one is for the indices of the test hand, the other for the real robot 
    
    RobotBase::RobotStateSaver saver(robot);

    const char* delim = " \r\n\t";
    KinBodyPtr ptarget = NULL, *pbodyTable = NULL;
    RobotBasePtr probotHand = NULL;
    char* p = strtok(NULL, delim);
    int nNumGrasps=0, nGraspDim=0;
    Vector vpalmdir; // normal of plam dir (in local coord system of robot hand)
    dReal fOffset=0.01f; // offset before approaching to the target
    bool bUseNearestIK = false; // true when ik of robot is constrained
    vector<pair<string, string> > vSwitchPatterns;
    wstring targetname, tablename;
    Vector vtableup;
    float fTableDist;
    vector<Vector> vTableDestinations;
    Vector quatDestPose; // destination rotation of the object

    while(p != NULL ) {
        if( stricmp(p, "grasps") == 0 ){
            nNumGrasps = atoi(strtok(NULL, delim));
            nGraspDim = atoi(strtok(NULL, delim));
            vgrasps.resize(nNumGrasps*nGraspDim);
            for(size_t i = 0; i < vgrasps.size(); ++i)
                vgrasps[i] = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p, "target") == 0 ) {
            targetname = _ravembstowcs(strtok(NULL, delim));
            ptarget = GetEnv()->GetKinBody(targetname.c_str());
        }
        else if( stricmp(p, "robothand") == 0 ) {
            KinBodyPtr ptemp = GetEnv()->GetKinBody(_ravembstowcs(strtok(NULL, delim)).c_str());
            if( ptemp != NULL && stricmp(ptemp->GetXMLId(), "KinBody") != 0 )
                probotHand = (RobotBasePtr)ptemp;
        }
        else if( stricmp(p, "preshape") == 0 ) {
            // specify indices of the same joint for the different robots
            vPreshapeIndices.push_back(atoi(strtok(NULL, delim)));
            vPreshapeIndicesRobot.push_back(atoi(strtok(NULL, delim)));
        }
        else if( stricmp(p, "palmdir") == 0 ) {
            vpalmdir.x = (dReal)atof(strtok(NULL, delim));
            vpalmdir.y = (dReal)atof(strtok(NULL, delim));
            vpalmdir.z = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p, "handjoints") == 0 ) {
            int n = atoi(strtok(NULL, delim));
            vHandJoints.resize(n);
            for(int i = 0; i < n; ++i)
                vHandJoints.push_back(atoi(strtok(NULL, delim)));
        }
        else if( stricmp(p, "offset") == 0 ) {
            fOffset = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p, "destpose") == 0 ) {
            quatDestPose.x = (dReal)atof(strtok(NULL, delim));
            quatDestPose.y = (dReal)atof(strtok(NULL, delim));
            quatDestPose.z = (dReal)atof(strtok(NULL, delim));
            quatDestPose.w = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p, "nearestik") == 0 ) {
            bUseNearestIK = true;
        }
        else if( stricmp(p, "switch") == 0 ) {
            char* pattern = strtok(NULL, delim);
            char* fatfilename = strtok(NULL, delim);
            if( pattern != NULL && fatfilename != NULL )
                vSwitchPatterns.push_back(pair<string, string>(pattern, fatfilename));
        }
        else if( stricmp(p, "table") == 0 ) {
            // format: id updir num_dests destinations
            tablename = _ravembstowcs(strtok(NULL, delim));
            pbodyTable = GetEnv()->GetKinBody(tablename.c_str());
            vtableup.x = (dReal)atof(strtok(NULL, delim));
            vtableup.y = (dReal)atof(strtok(NULL, delim));
            vtableup.z = (dReal)atof(strtok(NULL, delim));
            fTableDist = sqrtf(lengthsqr3(vtableup));
            vtableup /= fTableDist; // normalize

            int numdests = atoi(strtok(NULL, delim));
            vTableDestinations.resize(numdests);
            for(int i = 0; i < numdests; ++i) {
                vTableDestinations[i].x = (dReal)atof(strtok(NULL, delim));
                vTableDestinations[i].y = (dReal)atof(strtok(NULL, delim));
                vTableDestinations[i].z = (dReal)atof(strtok(NULL, delim));
            }
        }
        else break;

        p = strtok(NULL, delim);
    }

    if( ptarget == NULL ) {
        RAVELOG_INFO("Could not find target %S\n", targetname.c_str());
        return -1;
    }
    if( pbodyTable == NULL ) {
        RAVELOG_INFO("Could not find table %S\n", tablename.c_str());
        return -1;
    }
    if( probotHand == NULL ) {
        RAVELOG_INFO("Couldn't not find test hand\n");
        return -1;
    }
    if( vTableDestinations.size() == 0 ) {
        RAVELOG_INFO("no table destinations!\n");
        return -2;
    }

    const int iGraspDir = 0;
    const int iGraspPos = 3;
    const int iGraspRoll = 6;
    const int iGraspPreshape = 7; // use vPreshapeIndices for the indices
    const int iGraspStandoff = 10;
    const int iGraspStartJoints = 11;

    char strCmd[256]; // command buffer
    string strResponse;
    Transform transTarg = ptarget->GetTransform();
    Transform transInvTarget = transTarg.inverse();
    Transform transDummy(Vector(1,0,0,0), Vector(100,0,0));
    vector<dReal> viksolution, vikgoal, vjointsvalues;

    Vector tablecenter = pbodyTable->GetTransform().trans + vtableup*fTableDist;
    
    DistMetric _distmetric;
    _distmetric.thresh = 0.02f;
    _distmetric.SetRobot(robot);

    PlannerBase::PlannerParameters bispaceparams;
    bispaceparams.pdistmetric = &_distmetric;
    bispaceparams.pcostfn = NULL;
    bispaceparams.nMaxIterations = 10000;
    bispaceparams.vParameters.resize(4);
    bispaceparams.vnParameters.resize(4);
    bispaceparams.vnParameters[0] = 0; // rrt
    bispaceparams.vnParameters[1] = 1; // 6D
    bispaceparams.vnParameters[2] = 8; // expansion multiplier
    bispaceparams.vnParameters[3] = 128; // stochastic gradient samples
    bispaceparams.vParameters[0] = 0.04f; // config step
    bispaceparams.vParameters[1] = 0.004f; // work step
    bispaceparams.vParameters[2] = 0.05f; // Configuration Follow Probability
    bispaceparams.vParameters[3] = 0.004f; // Stochastic Accept Probability

    Transform transDestHand;
    TrajectoryBasePtr ptraj = NULL;
    int i = 0;
    int iDestIndex = -1;

    for(; i < nNumGrasps; ++i) {
        dReal* pgrasp = &vgrasps[i*nGraspDim];

        SWITCHMODELS(false);

        // set the hand joints
        probotHand->SetActiveDOFs(vPreshapeIndices);
        probotHand->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape,true);
        probotHand->SetActiveDOFs(vHandJoints, 0);
        probotHand->SetActiveDOFValues(NULL, pgrasp+iGraspStartJoints, true);
        probotHand->SetActiveDOFs(vHandJoints, RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z);

        robot->Enable(false);
        
        int newlen = snprintf(strCmd, sizeof(strCmd), "exec direction %f %f %f body %S robot %d roll %f standoff %f centeroffset %f %f %f palmdir %f %f %f",
            pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2], ptarget->GetName(), probotHand->GetEnvironmentId(), pgrasp[iGraspRoll], pgrasp[iGraspStandoff],
            pgrasp[iGraspPos]-transTarg.trans.x, pgrasp[iGraspPos+1]-transTarg.trans.y, pgrasp[iGraspPos+2]-transTarg.trans.z, vpalmdir.x, vpalmdir.y, vpalmdir.z);
        assert(newlen < sizeof(strCmd)-1); // make sure nothing is exceeded

        pGrasperProblem->SendCommand(strCmd, strResponse);

        if( bUseNearestIK ) {
            // get the real achievable transformation of the hand before calculating failures
            if( !pmanip->FindIKSolution(probotHand->GetTransform()*pmanip->tGrasp, viksolution, false) ) {
                RAVELOG_INFO("grasp %d: failed to find Nearest IK solution\n", i);
                continue;
            }

            {
                RobotBase::RobotStateSaver saver(robot);
                robot->SetActiveDOFs(pmanip->GetArmIndices());
                robot->SetActiveDOFValues(NULL, &viksolution[0], true);
                probotHand->SetTransform(pmanip->GetEndEffector()->GetTransform());
                //while (getchar() != '\n') usleep(1000);
            }

            // restart the grasper, but without moving the arm
            probotHand->SetActiveDOFs(vPreshapeIndices);
            probotHand->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape,true);
            probotHand->SetActiveDOFs(vHandJoints);
            probotHand->SetActiveDOFValues(NULL, pgrasp+iGraspStartJoints, true);

            strcat(strCmd, " notrans");
            pGrasperProblem->SendCommand(strCmd, strResponse);
        }

        if( strResponse.size() == 0 ) {
            RAVELOG_INFO("grasp planner failed: %d\n", i);
            continue; // failed
        }

        robot->Enable(true);
        Transform transRobot = probotHand->GetTransform();

        // send the robot somewhere
        probotHand->GetController()->SetPath(NULL); // reset
        probotHand->SetTransform(transDummy);

        // set the initial hand joints
        robot->SetActiveDOFs(vPreshapeIndicesRobot);
        robot->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape, true);
        robot->SetActiveDOFs(pmanip->GetGripperIndices());
        robot->SetActiveDOFValues(NULL, pgrasp+iGraspStartJoints, true);

        // check ik
        dReal fSmallOffset = 0.002f;
        if( bUseNearestIK )
            fSmallOffset = 0;

        Transform tnewrobot = transRobot;
        tnewrobot.trans -= fSmallOffset * Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
        
        // first test the IK solution at the destination transRobot
        if( !pmanip->FindIKSolution(tnewrobot*pmanip->tGrasp, vikgoal, true) ) {
            RAVELOG_INFO("grasp %d: No IK solution found (final)\n", i);
            continue;
        }
        
        // switch to fat models
        SWITCHMODELS(true);
        
        // now test at the approach point (with offset)
        tnewrobot.trans -= (fOffset-fSmallOffset) * Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
        
        if( !pmanip->FindIKSolution(tnewrobot*pmanip->tGrasp, viksolution, true) ) {
            RAVELOG_INFO("grasp %d: No IK solution found (approach)\n", i);
            continue;
        }
        else {
            stringstream s;
            s << "IK found: "; 
            for(size_t j = 0; j < viksolution.size(); j++)
                s << viksolution[j] << " ";
            s << endl;
            RAVEPRINT(s.str().c_str());
        }
        
        bool badik = false;
        for(size_t j = 0; j < viksolution.size(); j++) {
            //s << qResult[i] << " ";  
            if(fabsf(viksolution[j] - vikgoal[j]) > (j<4?0.3f:0.5f) )
                badik = true;
        }
        
        if( badik ) {
            stringstream s;
            s << "Goal IK: "; 
            FOREACH(it, vikgoal)  s << *it << " ";
            s << endl;
            s << "Bad IK detected for grasp " << i << "!\n" << endl;
            RAVEPRINT(s.str().c_str());
            continue;
        }
        
        iDestIndex = -1;
        
        Transform transgoal;
        Vector quatDestTarget, qrandomrot;
        dQMultiply0(quatDestTarget, transTarg.rot, quatDestPose);

        // set the joints that the grasper plugin calculated
        probotHand->SetActiveDOFs(vHandJoints, 0);
        probotHand->GetActiveDOFValues(vjointsvalues);
        robot->SetActiveDOFs(pmanip->GetGripperIndices());
        robot->SetActiveDOFValues(NULL, &vjointsvalues[0], true);

        for(size_t j = 0; j < vTableDestinations.size(); ++j) {
            transgoal.trans = tablecenter + pbodyTable->GetTransform().rotate(vTableDestinations[j]);

            for(int k = 0; k < 20; ++k) {
                // random roll around table axis
                dQFromAxisAndAngle(qrandomrot, vtableup.x, vtableup.y, vtableup.z, RANDOM_FLOAT()*2*PI);
                // get final object position
                dQMultiply0(transgoal.rot, qrandomrot, quatDestTarget);
                // get final hand position
                transDestHand = transgoal * transInvTarget * transRobot;
         
                ptarget->SetTransform(transgoal);
                CollisionReport report;
                if( GetEnv()->CheckCollision(ptarget, &report) ) {
                    //RAVELOG_INFO("%f %f %f %f %f %f %f\n", transgoal.trans.x, transgoal.trans.y,
//                              transgoal.trans.z, transgoal.rot.x, transgoal.rot.y,
//                              transgoal.rot.z, transgoal.rot.w);
//                    RAVELOG_INFO("target collision at dest %S:%S with %S:%S\n",
//                            report->plink1 != NULL ? report->plink1->GetParent()->GetName() : L"(NULL",
//                        report->plink1 != NULL ? report->plink1->GetName() : L"(NULL)",
//                        report->plink2 != NULL ? report->plink2->GetParent()->GetName() : L"(NULL)",
//                        report->plink2 != NULL ? report->plink2->GetName() : L"(NULL)");
                    Vector pos = report->contacts.front().pos;
                    ptarget->SetTransform(transTarg);
                    continue;
                }
                ptarget->SetTransform(transTarg);

                if( pmanip->FindIKSolution(transDestHand*pmanip->tGrasp, vikgoal, true) ) {
                    iDestIndex = (int)j;
                    break;
                }
            }

            if( iDestIndex >= 0 )
                break;
        }
        
        if( iDestIndex < 0 ) {
            RAVELOG_INFO("grasp %d: could not find destination\n", i);
            continue;
        }

        // finally start planning
        // set back to the initial hand joints
        robot->SetActiveDOFValues(NULL, pgrasp+iGraspStartJoints, true);
        
        assert( vikgoal.size() == pmanip->GetArmIndices().size());
        ptraj = MoveArm(pmanip->GetArmIndices(), viksolution, *pmanip, ptarget);
        RAVELOG_INFO("grasp %d: failure to plan\n", i);

        if( ptraj != NULL )
            break;
    }

    if( ptraj != NULL ) {
        SWITCHMODELS(false);
        
        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(robot->GetDOF());
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        
        // save the trajectory
        const char* filename = "movetraj.txt";
        pfulltraj->Write(filename, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
       
        // format: ikgoal goaltrans grasp_id destination_index traj_filename
        stringstream s;
        for(size_t j = 0; j < vikgoal.size(); ++j )
            s << vikgoal[j] << " ";

        
        // get the final transforamtion
        TransformMatrix t = TransformMatrix(transDestHand);

        s   << t.m[0] << " " << t.m[4] << " " << t.m[8] << " "
            << t.m[1] << " " << t.m[5] << " " << t.m[9] << " "
            << t.m[2] << " " << t.m[6] << " " << t.m[10] << " "
            << t.trans.x << " " << t.trans.y << " " << t.trans.z << " ";

        s << i << " " << iDestIndex << " " << filename;
        response = s.str();
        RAVELOG_INFO("testgrasps success: %s\n", response.c_str());
        delete ptraj;
        delete pfulltraj;

        probotHand->SetTransform(transDummy);
        return 0;
    }
    
    // send the robot somewhere
    robot->Enable(true);
    probotHand->SetTransform(transDummy);

    SWITCHMODELS(false);

    return -3; // couldn't not find for this cup

}
*/

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
    //const char* delim = " \r\n\t";
    KinBodyPtr ptarget;
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));    
        }
        else if( stricmp(p.c_str(), "openhand") == 0 ) {
            bOpenHand = true;
            if( pmanip.get() != NULL )
                vopenhandvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vopenhandvals.size(); ++i)
                cmd >> vopenhandvals[i];// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "sethand") == 0 ) {
            bSetHand = true;
            if( pmanip.get() != NULL )
                vopenhandvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vopenhandvals.size(); ++i)
                cmd >> vopenhandvals[i];// = (dReal)atof(strtok(NULL, delim));
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
        //p = strtok(NULL, delim);
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
    //delete ptraj;
    return 0;
}

vector<dReal> ManipulationProblem::LiftArmGeneralIK(istream& cmd)
{
    RAVELOG_INFO("Starting LiftArmGeneralIK...\n");
    dReal step_size = 0.003f;
    Vector direction = Vector(0,1,0);
    
    int minsteps = 10;
    int maxsteps = 60;
    bool bExec = true;

//    const char* delim = " \r\n\t";
//    char* p = strtok(NULL, delim);
    KinBodyPtr ptarget;
    string filename = "liftarmtraj.txt";
//    while(p != NULL ) {
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
            cmd >> minsteps;// = atoi(strtok(NULL, delim));

        }
        else if( stricmp(p.c_str(), "maxsteps") == 0 ) {
            cmd >> maxsteps;// = atoi(strtok(NULL, delim));

        }
        else if( stricmp(p.c_str(), "direction") == 0 ) {
            cmd >> direction.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "exec") == 0 ) {
            cmd >> bExec;// = atoi(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "filename") == 0 ) {
            cmd >> filename;// = strtok(NULL, delim);
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return vector<dReal>();
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return vector<dReal>();
        }


        //p = strtok(NULL, delim);
    }

    if( ptarget.get() != NULL )
        robot->Grab(ptarget);

    std::vector<int> temp_indices = robot->GetActiveDOFIndices();
    
    robot->RegrabAll();
    RobotBase::RobotStateSaver saver(robot);
    
    Transform handTr = robot->GetActiveManipulator()->GetEndEffectorTransform();
    
    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());

    Trajectory::TPOINT point;
    point.q.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(point.q);
    
    std::vector<dReal> qResult(robot->GetActiveDOF());

    
    boost::shared_ptr<std::vector<dReal> > pqResult(new std::vector<dReal> );


    vector<dReal> vPrevValues;
    robot->GetActiveDOFValues(vPrevValues);

    std::vector<dReal> ikparams;


    int i;
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



        if( !_pGeneralIKSolver->Solve(IkParameterization(), point.q, ikparams, false, pqResult)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }
        point.q = *pqResult.get();



        size_t j = 0;
        for(; j < point.q.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,point.q[j],vPrevValues[j]);
            if(fabsf(point.q[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < point.q.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }
        
        ptraj->AddPoint(point);
        robot->SetActiveDOFValues(point.q);
        //Transform gottm = robot->GetActiveManipulator()->GetEndEffectorTransform();
        //RAVELOG_INFO("requested: %f %f %f got: %f %f %f\n",handTr.trans.x,handTr.trans.y, handTr.trans.z,gottm.trans.x,gottm.trans.y, gottm.trans.z);

        if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot)) && i > minsteps) {
            RAVELOG_DEBUG("Arm Lifting: broke due to collision\n");
            break;
        }

        vPrevValues = point.q;
    }

    RAVELOG_INFO("Arm Lifted: %d steps\n", i);

    if(bExec)
    {
        SetTrajectory(ptraj);
    }
    else
    {
        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), true, false);
        ofstream outfile(filename.c_str(), ios::out);
        pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        outfile.close();
        chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
    }

    robot->SetActiveDOFs(temp_indices);
    std::vector<dReal> lastconfig;
    if(ptraj->GetPoints().size() > 0)
        lastconfig = ptraj->GetPoints()[ptraj->GetPoints().size()-1].q;
    //delete ptraj;
    return lastconfig;
}

vector<dReal> ManipulationProblem::InterpolateToEEPose(istream& cmd)
{
    RAVELOG_INFO("Starting InterpolateToEEPose...\n");
    int numsteps = 0;


    bool bExec = true;
    TransformMatrix targTm;
//    const char* delim = " \r\n\t";
//    char* p = strtok(NULL, delim);
    KinBodyPtr ptarget;
    string filename = "liftarmtraj.txt";
//    while(p != NULL ) {
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
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());

    Trajectory::TPOINT point;
    point.q.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(point.q);

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



        if( !_pGeneralIKSolver->Solve(IkParameterization(), point.q, ikparams, false, pqResult)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }
        point.q = *pqResult.get();



        size_t j = 0;
        for(; j < point.q.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,point.q[j],vPrevValues[j]);
            if(fabsf(point.q[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < point.q.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }

        ptraj->AddPoint(point);
        robot->SetActiveDOFValues(point.q);
        //Transform gottm = robot->GetActiveManipulator()->GetEndEffectorTransform();
        //RAVELOG_INFO("requested: %f %f %f got: %f %f %f\n",handTr.trans.x,handTr.trans.y, handTr.trans.z,gottm.trans.x,gottm.trans.y, gottm.trans.z);

        if(GetEnv()->CheckCollision(KinBodyConstPtr(robot))) {
            RAVELOG_DEBUG("EE Interpolation broke due to collision\n");
            break;
        }

        vPrevValues = point.q;
    }

    RAVELOG_INFO("Arm moved: %d/%d steps\n", i, numsteps);

    if(bExec)
    {
        SetTrajectory(ptraj);
    }
    else
    {
        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), true, false);
        ofstream outfile(filename.c_str(), ios::out);
        pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        outfile.close();
        chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
    }

    robot->SetActiveDOFs(temp_indices);
    std::vector<dReal> lastconfig;
    //don't return anything if there is only one (or zero) points
    if(ptraj->GetPoints().size() > 1)
        lastconfig = ptraj->GetPoints()[ptraj->GetPoints().size()-1].q;
    //delete ptraj;
    return lastconfig;
}

vector<dReal> ManipulationProblem::GetWristPoints(istream& cmd)
{
    RAVELOG_INFO("Starting GetWristPoints...\n");

//    const char* delim = " \r\n\t";
//    char* p = strtok(NULL, delim);
    string filename;
//    while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "filename") == 0 ) {
            cmd >> filename;//
        }
//        p = strtok(NULL, delim);
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
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetDOF());
    std::vector<dReal> eeposes;
    ifstream infile(filename.c_str(),ios::in);
    if( ptraj->Read(infile, robot) ) {
        FOREACH(itpoint, ptraj->GetPoints()){
            robot->SetJointValues(itpoint->q);
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

    //const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    KinBodyPtr ptarget;
    //while(p != NULL ) {
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
            cmd >> minsteps;// = atoi(strtok(NULL, delim));

        }
        else if( stricmp(p.c_str(), "maxsteps") == 0 ) {
            cmd >> maxsteps;// = atoi(strtok(NULL, delim));

        }
        else if( stricmp(p.c_str(), "direction") == 0 ) {
            cmd >> direction.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.z;// = (dReal)atof(strtok(NULL, delim));
        }
        //p = strtok(NULL, delim);
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
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());
    Trajectory::TPOINT point;
    vector<dReal> vPrevValues;
    robot->GetActiveDOFValues(vPrevValues);

    int i;
    for (i = 0; i < maxsteps;  i++) {
        handTr.trans = handTr.trans + step_size*direction;
        if( !pmanip->FindIKSolution(handTr,point.q,false)) {
            RAVELOG_DEBUG("Arm Lifting: broke due to ik\n");
            break;
        }

        size_t j = 0;
        for(; j < point.q.size(); j++) {
            //RAVELOG_INFO("%d new: %f old: %f\n",j,point.q[j],vPrevValues[j]);
            if(fabsf(point.q[j] - vPrevValues[j]) > 0.1)
                break;
        }

        if( j < point.q.size()) {
            RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
            break;
        }
        
        ptraj->AddPoint(point);
        robot->SetActiveDOFValues(point.q);
        if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot)) && i > minsteps) {
            RAVELOG_DEBUG("Arm Lifting: broke due to collision\n");
            break;
        }

        vPrevValues = point.q;
    }

    RAVELOG_INFO("Arm Lifting: %d steps\n", i);
    SetTrajectory(ptraj);
    robot->SetActiveDOFs(temp_indices);
    //delete ptraj;
    return 1;
}


int ManipulationProblem::JMoveHandStraight(istream& cmd)
{
    RAVELOG_DEBUG("Starting JMoveHandStraight...\n");

    int maxiterations = 20;

    TransformMatrix handTm;
    Vector direction;
    //const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    string trajfilename;

    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;
    
    vector<dReal> vOldValues;
    robot->GetDOFValues(vOldValues);
    
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "direction") == 0 ){
            cmd >> direction.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));
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
        

        //p = strtok(NULL, delim);
    }

    Transform handTr(handTm);

    vector<dReal> qResultold;
    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    robot->GetActiveDOFValues(qResultold);

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());
    Trajectory::TPOINT point, goodpoint;
    robot->GetActiveDOFValues(point.q);
    ptraj->AddPoint(point);

    goodpoint = point;
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
    //TransformMatrix Tmtarg = TransformMatrix(pmanip->GetEndEffectorTransform());
    //Transform Ttarg(Tmtarg);



//    for(int i = 0; i < vlowerlimits.size(); i++)
//    {
//        if(vlowerlimits[i] < -179)
//            vlowerlimits[i] = -179;
//        if(vupperlimits[i] > 179)
//            vupperlimits[i] = 179;
//    }

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

            OpenRAVE::mathextra::transnorm3(v, invJJt, e);

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
                goodpoint.q[x] = vfulljoints[pmanip->GetArmIndices()[x]];
        }
        else
            break;

    }
    




        /*
        if( !pmanip->FindIKSolution(handTr, point.q, true)  )
        {
            RAVELOG_INFO("\nMoveHandStraight - No IK solution found\n");     
            break;
        }
        else
        {
            bool badik = false;
            for(size_t i = 0; i < point.q.size(); i++)
            {
                float thresh = i == 6 ? 0.5f : 0.25f;
                //s << qResult[i] << " ";  
                if(fabsf(qResultold[i] - point.q[i]) > thresh)
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
                FOREACH(it, point.q) s << *it << " ";
                s << endl;
                RAVEPRINT(s.str().c_str());
                break;
            }

        }
        
        robot->SetActiveDOFs(pmanip->GetArmIndices(),0,NULL);
        robot->SetActiveDOFValues(NULL,&point.q[0]);
        //tempTr = pmanip->GetEndEffectorTransform();
        
        if(GetEnv()->CheckCollision(robot))
            break;
        
        qResultold = point.q;
        goodpoint = point;
        ++iter;
    }
    */

    // restore robot joint values
    robot->SetJointValues(vOldValues);

    if( iter > 0 ) {
        ptraj->AddPoint(goodpoint);
        ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
        robot->SetActiveMotion(ptraj);
        RAVELOG_INFO("moving hand straight: %d\n", iter);     
 
        if( trajfilename != "" ) {
            RAVELOG_DEBUG("Writing solved move-hand-straight trajectory to file %s\n", trajfilename.c_str());
            TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
            robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream outfile(trajfilename.c_str(), ios::out);
            pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            outfile.close();
            chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777
            //delete pfulltraj;
        }
        
    }
    else RAVELOG_INFO("move hand straight didn't move\n");
    //delete ptraj;
    return 0;
}



int ManipulationProblem::MoveHandStraight(istream& cmd)
{
    RAVELOG_DEBUG("Starting MoveHandStraight...\n");

    int maxiterations = 20;

    TransformMatrix handTm;
    Vector direction;
    const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    string trajfilename;

    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;
    
    vector<dReal> vOldValues;
    robot->GetDOFValues(vOldValues);
    
//    while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "direction") == 0 ){
            cmd >> direction.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "writetraj") == 0 ) {
            RAVELOG_INFO("writing trajectory to file\n");
            cmd >> trajfilename;// = strtok(NULL, delim);
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
        //else break;
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }


        //p = strtok(NULL, delim);
    }

    Transform handTr(handTm);

    vector<dReal> qResultold;
    robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
    robot->GetActiveDOFValues(qResultold);

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());
    Trajectory::TPOINT point, goodpoint;
    robot->GetActiveDOFValues(point.q);
    ptraj->AddPoint(point);

    goodpoint = point;
    int iter = 0;
    Transform tempTr;
    
    while(iter < maxiterations) {
        handTr.trans = handTr.trans + 0.001f*direction;
        //RAVELOG_INFO("handtr in : %.3f %.3f %.3f   %.3f %.3f %.3f %.3f\n", handTr.trans.x, handTr.trans.y, handTr.trans.z, handTr.rot.w,handTr.rot.x,handTr.rot.y, handTr.rot.z);
        if( !pmanip->FindIKSolution(handTr, point.q, true)  )
        {
            RAVELOG_INFO("\nMoveHandStraight - No IK solution found\n");     
            break;
        }
        else
        {
            bool badik = false;
            for(size_t i = 0; i < point.q.size(); i++)
            {
                float thresh = i == 6 ? 0.5f : 0.25f;
                //s << qResult[i] << " ";  
                if(fabsf(qResultold[i] - point.q[i]) > thresh)
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
                FOREACH(it, point.q) s << *it << " ";
                s << endl;
                RAVELOG_INFO(s.str());
                break;
            }

        }
        
        robot->SetActiveDOFs(pmanip->GetArmIndices(),0);
        robot->SetActiveDOFValues(point.q);
        //tempTr = pmanip->GetEndEffectorTransform();
        
        if(GetEnv()->CheckCollision(KinBodyConstPtr(robot)))
            break;

        qResultold = point.q;
        goodpoint = point;
        ++iter;
    }

    // restore robot joint values
    robot->SetJointValues(vOldValues);

    if( iter > 0 ) {
        ptraj->AddPoint(goodpoint);
        ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
        robot->SetActiveMotion(ptraj);
        RAVELOG_INFO("moving hand straight: %d\n", iter); 

        if( trajfilename != "" ) {
            RAVELOG_INFO("Writing solved move-hand-straight trajectory to file\n");
            TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
            robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream outfile(trajfilename.c_str(), ios::out);
            pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            outfile.close();
            chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

        }
    
    }
    else RAVELOG_INFO("move hand straight didn't move\n");
    return 0;
}


class GraspParameters : public PlannerBase::PlannerParameters
{
 public:
 GraspParameters() : stand_off(0), roll_hand(0), face_target(false), bReturnTrajectory(false) {}
        
    dReal stand_off; /// start closing fingers when at this distance
    dReal roll_hand; /// rotate the hand about the palm normal by this many radians
    Vector direction,palmnormal;
    bool face_target; ///point the hand at the target or not (1 = yes, else no)
    bool bReturnTrajectory;
        
 protected:
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<stand_off>" << stand_off << "</stand_off>" << endl;
        O << "<face_target>" << face_target << "</face_target>" << endl;
        O << "<roll_hand>" << roll_hand << "</roll_hand>" << endl;
        O << "<direction>" << direction << "</direction>" << endl;
        O << "<palmnormal>" << palmnormal << "</palmnormal>" << endl;
        O << "<returntrajectory>" << bReturnTrajectory << "</returntrajectory>" << endl;
        return !!O;
    }
 
    // called at the end of every XML tag, _ss contains the data 
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( name == "stand_off")
            _ss >> stand_off;
        else if( name == "face_target")
            _ss >> face_target;
        else if( name == "roll_hand")
            _ss >> roll_hand;
        else if( name == "direction")
            _ss >> direction;
        else if( name == "palmnormal")
            _ss >> palmnormal;
        else if( name == "returntrajectory")
            _ss >> bReturnTrajectory;
        else // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        return false;
    }
};        
         
std::vector<int> ManipulationProblem::MultiHandClose(istream& cmd)
{
    RAVELOG_DEBUG("Starting MultiHandClose...\n");

    std::vector<int> out;
    //const char* delim = " \r\n\t";

    int numtms;
    std::vector<Transform> vtransforms;

    RobotBasePtr probot;
    KinBodyPtr ptemp;

    int numexcluded = 0;
    std::vector<KinBodyConstPtr> vexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    string targetname;
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    TransformMatrix handTm;
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "robot") == 0 ) {
            cmd >> p;//= strtok(NULL, delim);
            probot = GetEnv()->GetRobot(p);

        }
        else if( stricmp(p.c_str(), "targetname") == 0 ) {
            cmd >> targetname;//= strtok(NULL, delim);
        }
        else if( stricmp(p.c_str(), "excluded") == 0 ) {
            cmd >> numexcluded;//= atoi(strtok(NULL, delim));
            for(int i = 0; i < numexcluded; i++)
            {
                cmd >> p;//strtok(NULL, delim);
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
            cmd >> numtms;// = atoi(strtok(NULL, delim));
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> vtransforms[i].rot.x;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.y;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.z;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].rot.w;//= atof(strtok(NULL, delim));

                cmd >> vtransforms[i].trans.x;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].trans.y;//= atof(strtok(NULL, delim));
                cmd >> vtransforms[i].trans.z;//= atof(strtok(NULL, delim));
            }

        }
        else if( stricmp(p.c_str(), "transforms12") == 0 ) {
            cmd >> numtms;// = atoi(strtok(NULL, delim));
            vtransforms.resize(numtms);
            for(int i = 0; i< numtms; i++)
            {
                cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
                cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
                vtransforms[i] = (Transform) handTm;
            }

        }
        //p = strtok(NULL, delim);
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


    TrajectoryBasePtr ptraj2 = RaveCreateTrajectory(GetEnv(), robot->GetActiveDOF());

    //NOTE: YOU NEED GRASPER2 FOR THIS, NOT THE GRASPER THAT COMES WITH OPENRAVE
    PlannerBasePtr graspplanner = RaveCreatePlanner(GetEnv(),"Grasper2");
    if( graspplanner != NULL )
        RAVELOG_DEBUG("grasping planner created!\n");
    else
    {
        RAVELOG_FATAL("Error: Can not find grasper2, terminating!\n");
        return out;
    }

    boost::shared_ptr<GraspParameters> params2;
    params2.reset(new GraspParameters());

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
            if(ptraj2->GetPoints().size() == 0)
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

         
int ManipulationProblem::CloseFingers(istream& cmd)
{        
    RAVELOG_DEBUG("Starting CloseFingers...\n");
         
    Vector direction;
    KinBodyPtr ptarget;
    _bCloseFingersPhase2 = false;
         
    const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
         
//    const char* delim = " \r\n\t";
//    char* p = strtok(NULL, delim);
//    while(p != NULL ) {
//        if( stricmp(p, "squeeze") == 0 ) {
//            _bCloseFingersPhase2 = true;
//        }
//        else break;
//       
//        p = strtok(NULL, delim);
//    }  
         
    if( pmanip == NULL )
        return -1;
         
    //close fingers
    vector<int> activejoints = pmanip->GetGripperIndices();
    vector<dReal> closingdir = pmanip->GetClosingDirection();;
    /*   
    for(size_t i = 0; i < activejoints.size(); ++i) {
        for(size_t j = 0; j < pmanip->GetGripperIndices().size(); j++) {
            if(activejoints[i] == pmanip->GetGripperIndices()[j]) {
                if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
                    closingdir.push_back(-1.0f);
                else
                    closingdir.push_back(1.0f);
            }
        }
    }    
     */  
    robot->SetActiveDOFs(activejoints,0);
         
    TrajectoryBasePtr ptraj2 = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());

    // have to add the first point
    Trajectory::TPOINT ptfirst;
    robot->GetActiveDOFValues(ptfirst.q);
 
    PlannerBasePtr graspplanner = RaveCreatePlanner(GetEnv(),"Grasper");
    if( graspplanner != NULL )
        RAVELOG_DEBUG("grasping planner created!\n");
    
    boost::shared_ptr<GraspParameters> params2;
    params2.reset(new GraspParameters());

    params2->vinitialconfig.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(params2->vinitialconfig);    

    params2->stand_off = 0.0f; ///start closing fingers when at this distance
    params2->face_target = false; ///point the hand at the target or not (1 = yes, else no)
    params2->roll_hand = 0.0f; /// rotate the hand about the palm normal by this many radians
    params2->direction = direction;    //direction of approach
    params2->palmnormal = Vector(0,0,0); //palm normal (not used here)
    params2->bReturnTrajectory  = false;//return a trajectory

    if( !graspplanner->InitPlan(robot, params2) ) {
        RAVELOG_DEBUG("InitPlan failed\n");
        return -1;
    }
    
    if( !graspplanner->PlanPath(ptraj2) ) {
        RAVELOG_DEBUG("PlanPath failed\n");
        return -1;
    }   

    if( ptraj2->GetPoints().size() > 0 ) {
        Trajectory::TPOINT p = ptraj2->GetPoints().back();
        //for(int i = 0; i < ptraj2->GetPoints().back().q.size(); ++i)
        //    p.q[i] += (10/180.0f)*closingdir[i];
        ptraj2->Clear();
        ptraj2->AddPoint(ptfirst);
        ptraj2->AddPoint(p);
        ptraj2->CalcTrajTiming(robot, Trajectory::LINEAR, true, true);
        
//        nRealTrajTime = (u32)(ptraj2->GetPoints().back().time*4000)+1000;
//        RAVELOG_INFO("close fingers time %d\n", nRealTrajTime);
//        
//        if( nRealTrajTime > 10000 )
//            nRealTrajTime = 10000;
    }

    if( !robot->CheckSelfCollision() )
        SetTrajectory(ptraj2, 4);
    else
        RAVELOG_DEBUG("Robot self colliding!\n");

    // turn on motors
#ifdef INTELCONTROLLER
    if( robot != NULL && dynamic_cast<IntelController*>(robot->GetController()) != NULL ) {
        dynamic_cast<IntelController*>(robot->GetController())->EnableHandMotors(_bRightHand, true);
    }
#endif

    _state = PS_ClosingFingers;
    nStartTime = timeGetTime();

    //delete graspplanner;
    return 0;
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
                    if( !pfatbody->InitFromFile(itpattern->second.c_str(), std::list<std::pair<std::string,std::string> >() ) ) {
                        RAVELOG_INFO("failed to open file: %s\n", itpattern->second.c_str());
                        continue;
                    }
                    pfatbody->SetName(strname); // should be %Sfat
                    if( !GetEnv()->AddKinBody(pfatbody) ) {
                        //delete pfatbody;
                        continue;
                    }

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

    //const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
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


        //p = strtok(NULL, delim);
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
#ifdef USING_PLAYER
    RaveVisionClient::VisionTrackingData visiondata;
    if( _pvisionclient != NULL ) {
        RaveVisionClient::BODY* pvisionbody = _pvisionclient->GetBody(pbody);
        if( pvisionbody != NULL ) {
            bShouldLock = pvisionbody->bLock;
            nGlobalId = pvisionbody->nGlobalId;
            visiondata = pvisionbody->_initdata;
            bEnabled = pvisionbody->bEnabled;
        }
        _pvisionclient->RemoveKinBody(pbody, false);
    }
#endif
    
    pswitch->SetName(bodyname.c_str());
    pswitch->SetTransform(tinit);
    if( vjoints.size() > 0 )
        pswitch->SetJointValues(vjoints);
    
    pbody->SetName(newname.c_str());
    Transform temp; temp.trans.y = 100.0f;
    pbody->SetTransform(temp);
    
#ifdef USING_PLAYER
    if( _pvisionclient != NULL ) {
        
        RaveVisionClient::BODY* pvisionbody = _pvisionclient->AddKinBody(pswitch, &visiondata);
        if( pvisionbody != NULL ) {
            pvisionbody->bLock = bShouldLock;
            pvisionbody->nGlobalId = nGlobalId;
            pvisionbody->bEnabled = bEnabled;
        }

        pswitch->SetTransform(tprevtrans);
    }
#endif

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
    //const char* delim = " \r\n\t";
    KinBodyPtr ptarget;
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "trans") == 0 ){
            cmd >> handTm.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "rot") == 0 ) {
            cmd >> handTm.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> handTm.m[10];// = (dReal)atof(strtok(NULL, delim));    
        }
        else if( stricmp(p.c_str(), "writetraj") == 0 ) {
            cmd >> trajfilename;// = strtok(NULL, delim);
        }
        else if( stricmp(p.c_str(), "grab") == 0 ) {
            cmd >> p;
            ptarget = GetEnv()->GetKinBody(p);
        }
        else if( stricmp(p.c_str(), "handjoints") == 0 ) {
            if( pmanip.get() != NULL )
                vhandjointvals.resize(pmanip->GetGripperIndices().size());
            for(size_t i = 0; i < vhandjointvals.size(); ++i)
                cmd >> vhandjointvals[i];// = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p.c_str(), "setjoint") == 0 ) {
            int ind;// = atoi(pindex);
            cmd >> ind;
            if( ind >= 0 && ind < (int)values.size() ) {
                cmd >> values[ind];// = (dReal)atof(pvalue);
            }

//             char* pindex = strtok(NULL, delim);
//             char* pvalue = strtok(NULL, delim);
// 
//             if( pindex != NULL && pvalue != NULL ) {
//                 int ind = atoi(pindex);
//                 if( ind >= 0 && ind < (int)values.size() ) {
//                     values[ind] = (dReal)atof(pvalue);
//                 }
//             }

        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }

        //p = strtok(NULL, delim);
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

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(), robot->GetActiveDOF());
        
//    if( GetEnv()->CheckCollision(robot) ) {
//        RAVELOG_INFO("Hand in collision\n");
//        return -1;
//    }
    
    PlannerBase::PlannerParametersPtr params;
    
    robot->SetActiveDOFs(activejoints);
    std::vector<dReal> pzero(robot->GetActiveDOF());
    robot->GetActiveDOFValues(pzero);

    // make sure the initial and goal configs are not in collision
    robot->SetActiveDOFValues(activegoalconfig, true);


    //robot->SetActiveDOFs(pmanip->GetGripperIndices());
    
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_INFO("jitter failed for goal\n");
        robot->SetActiveDOFs(activejoints);
        return -1;
    }

    // restore
    robot->SetActiveDOFs(activejoints);

    robot->GetActiveDOFValues(params->vgoalconfig);
    robot->SetActiveDOFValues(pzero);

    Trajectory::TPOINT pt;
    pt.q = pzero;
    ptraj->AddPoint(pt);
    
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
    ptraj->Clear();
    
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
    
    //if (!_reusePlanner){
        //delete _pPlanner;
        //_pPlanner = NULL;
    //}
    
    if( !bSuccess ) {
        return -1;
    }
    
    if( trajfilename != "" ) {
        TrajectoryBasePtr pfulltraj(RaveCreateTrajectory(GetEnv(),robot->GetDOF()));
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        ofstream outfile(trajfilename.c_str(), ios::out);
        pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        outfile.close();
        chmod(trajfilename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

    }
    SetTrajectory(ptraj);
    return 0;
}

// constrains new configurations so that torques aren't exceeded, WAM only!
static RobotBasePtr s_pWAMRobot;
bool WAMConstraintFn(const dReal* pSrcConf, dReal* pDestConf, Transform* ptrans, int settings)
{
    RAVELOG_INFO("WAMConstraintFn: This function is not updated to work with the current version of rave. Need to replace ODE calls.\n");
/*
    assert( s_pWAMRobot != NULL );

    // don't move joint 2 moves against gravity
    dReal j1d = fabs(pDestConf[1]);
    dReal j1s = fabs(pSrcConf[1]);
    // assuming gravity is towards -z
    if( j1d > PI*0.25f && ( (settings&PlannerBase::CS_TimeBackward) ? j1d > j1s : j1d < j1s)  ) {
        
        if( fabsf(pDestConf[3]) < 2 ) {
            // don't move it if elbow is extended
            pDestConf[1] = pSrcConf[1];
        }
    }
    
    // compute the approximate torque on joint 3
    // v = sin(j2), 0, cos(j2)
    Vector vj3(sinf(pDestConf[1]), 0, cosf(pDestConf[1]));
    Transform t;
    dQFromAxisAndAngle(t.rot, vj3.x, vj3.y, vj3.z, pDestConf[2]);
    Vector velbow = t.rotate(Vector(sinf(pDestConf[1]+pDestConf[3]), 0, cosf(pDestConf[1]+pDestConf[3])));

    float fTorque = vj3.x * velbow.y;

    if( (pSrcConf[2] < pDestConf[2]) ^ (settings&PlannerBase::CS_TimeBackward) ) {
        // moving pos, torque should be < 0.3
        if( fTorque > 0.3 )
            pDestConf[2] = pSrcConf[2];
    }
    else {
        // torque should be > -0.3
        if( fTorque < -0.3f )
            pDestConf[2] = pSrcConf[2];
    }

    return true;
*/
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
    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF());

//    if( wcsicmp(manip.strIkSolver.c_str(), L"WAM7") == 0 ) {
//        s_pWAMRobot = robot;
//        params.pConstraintFn = WAMConstraintFn;
//    }

    ptraj->Clear();

    params->_nMaxIterations = 4000;
    bool bSuccess = false;
    u32 startplan = timeGetTime();
    RAVELOG_INFO("starting planning\n");
    
    for(int iter = 0; iter < 3; ++iter) {
        if( !_pPlanner->InitPlan(robot, params) ) {
            RAVELOG_DEBUG("InitPlan failed\n");
            //delete _pPlanner;
            //_pPlanner = NULL;
            return TrajectoryBasePtr();
        }
        
        if( _pPlanner->PlanPath(ptraj) ) {
            bSuccess = true;
            ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
            RAVELOG_INFO("finished planning, %dms\n", timeGetTime()-startplan);
            break;
        }
        else RAVELOG_INFO("PlanPath failed\n");
    }
    
    //if (!_reusePlanner){
    //    delete _pPlanner;
    //    _pPlanner = NULL;
    //}
    
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

// int ManipulationProblem::ReleaseFingers()
// {
//     RAVELOG_DEBUG("Releasing fingers...\n");
// 
//     KinBodyPtr ptarget;
//     vector<dReal> vopenconfig;
//     const RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
// 
//     const char* delim = " \r\n\t";
//     char* p = strtok(NULL, delim);
//     while(p != NULL ) {
//         if( stricmp(p, "target") == 0 ) {
//             ptarget = GetEnv()->GetKinBody(strtok(NULL, delim));
//         }
//         else if( stricmp(p, "open") == 0 ) {
//             if( pmanip != NULL )
//                 vopenconfig.resize(pmanip->GetGripperIndices().size());
// 
//             for(int i = 0; i < (int)vopenconfig.size(); ++i )
//                 vopenconfig[i] = (dReal)atof(strtok(NULL, delim));
//         }
//         else break;
// 
//         p = strtok(NULL, delim);
//     }
// 
//     if( pmanip.get() == NULL || vopenconfig.size() != pmanip->GetGripperIndices().size() )
//         return -1;
// 
//     TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory((int)pmanip->GetGripperIndices().size());
//     robot->SetActiveDOFs(pmanip->GetGripperIndices());
// 
//     Trajectory::TPOINT pt;
//     robot->GetActiveDOFValues(pt.q);
//     ptraj->AddPoint(pt);
//     pt.q = vopenconfig;
//     ptraj->AddPoint(pt);
// 
// 
//     if( ptarget.get() != NULL )
//         robot->Release(ptarget);
// 
//     robot->SetActiveDOFValues(vopenconfig);
//     SetTrajectory(ptraj);
//     
// #ifdef INTELCONTROLLER
//     if( dynamic_cast<IntelController*>(robot->GetController()) != NULL )
//         dynamic_cast<IntelController*>(robot->GetController())->EnableHandMotors(_bRightHand, true);
// #endif
// 
//     //delete ptraj;
//     return 0;
// }

void ManipulationProblem::SetTrajectory(TrajectoryBasePtr ptraj, float fSpeed)
{
    assert(robot != NULL);
    ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
    robot->SetActiveMotion(ptraj);
}

int ManipulationProblem::GrabBody(istream& cmd)
{
    RAVELOG_DEBUG("Starting GrabBody...\n");

    KinBodyPtr ptarget;
    bool bSendToController = false;
    TransformMatrix tmEE;

    //const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
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
            cmd >> tmEE.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[10];// = (dReal)atof(strtok(NULL, delim));    
            cmd >> tmEE.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.z;// = (dReal)atof(strtok(NULL, delim));
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
        

        //p = strtok(NULL, delim);
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
                Inew = (*itlink)->GetInertia();
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

    //const char* delim = " \r\n\t";
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;
        if( stricmp(p.c_str(), "applytransform") == 0 ){
            //this transform is used to move from the RAVE end-effector frame (tEE_rave) to the controller's end-effector frame (tEE_controller)
            //it should be specified in tEE_rave coordinates
            // it is used like this: tEE_controller = tEE_rave * T_applytransform
            cmd >> tmEE.m[0];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[4];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[8];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[1];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[5];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[9];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[2];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[6];// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.m[10];// = (dReal)atof(strtok(NULL, delim));    
            cmd >> tmEE.trans.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> tmEE.trans.z;// = (dReal)atof(strtok(NULL, delim));
        }
        else {
            RAVELOG_ERRORA("unrecognized command: %s\n",p.c_str());
            return -1;
        }
        if( !cmd ) {
            RAVELOG_ERRORA("failed processing command: %s\n",p.c_str());
            return -1;
        }

//        p = strtok(NULL, delim);
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
            Inew = (*itlink)->GetInertia();
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

/*
void ManipulationProblem::Log(float fElapsedTime)
{
    if( pfLog == NULL ) {
        pfLog = fopen("motion.txt", "wb");
    }

    fTotalTime += fElapsedTime;
    if( pfLog != NULL && (fTotalTime-fLogTime) > 0.05f ) {

        fwrite(&fTotalTime, sizeof(float), 1, pfLog);

        int numobjs = (int)GetEnv()->GetBodies().size();
        fwrite(&numobjs, 4, 1, pfLog);
        
        Transform t;
        vector<dReal> vjoints;
        std::vector<KinBodyPtr>::const_iterator itbody;
        FORIT(itbody, GetEnv()->GetBodies()) {
            size_t len = wcslen((*itbody)->GetName());
            fwrite(&len, 4, 1, pfLog);
            fwrite((*itbody)->GetName(), len*sizeof((*itbody)->GetName()[0]), 1, pfLog);

            t = (*itbody)->GetTransform();
            fwrite(&t, sizeof(t), 1, pfLog);

            (*itbody)->GetDOFValues(vjoints);

            len = vjoints.size();
            fwrite(&len, 4, 1, pfLog);
            if( len > 0 )
                fwrite(&vjoints[0], sizeof(vjoints[0]) * vjoints.size(), 1, pfLog);
        }

        fLogTime = fTotalTime;
    }
}
*/
/*
float DistMetric::Eval(const void* c0, const void* c1)
{
    float out = 0;
    for(int i=0; i < _robot->GetActiveDOF(); i++)
        out += (((dReal *)c0)[i]-((dReal *)c1)[i])*(((dReal *)c0)[i]-((dReal *)c1)[i]);

    return out;
}
*/

int ManipulationProblem::MakeGraspTrajectory(string& response, istream& cmd)
{

    RAVELOG_DEBUG("Starting MakeGraspTrajectory...\n");
    RobotBasePtr _robot;
    bool bdraw = false;
    bool bHasStepSize = false;
    bool bHasDirection = false;
    Vector direction;
    int numdof = -1;
    //const char* delim = " \r\n\t";
    std::vector<std::vector<Vector> > vpoints;
    std::vector<std::vector<int>  > vpointsetinds;
    std::vector<int> vdofinds;
    std::vector<int> vpointsperdof;

    std::vector<dReal> vstepsizes;
    //char* p = strtok(NULL, delim);
    //while(p != NULL ) {
    string p;
    while(!cmd.eof()) {
        cmd >> p;
        if( !cmd )
            break;

        if( stricmp(p.c_str(), "stepsizes") == 0 ) {
            int numsizes;// = atoi(strtok(NULL, delim));
            cmd >> numsizes;
            vstepsizes.resize(numsizes);
            for(int i = 0; i < numsizes;i++)
                cmd >> vstepsizes[i];// = (dReal)atof(strtok(NULL, delim)); 
            
            bHasStepSize = true;
        }
        else if( stricmp(p.c_str(), "draw") == 0 ) {
            bdraw = true;
        }
        else if( stricmp(p.c_str(), "robotid") == 0 ) {
            // specify which robot to use
            int robotid;// = atoi(strtok(NULL, delim));
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

            int linkind;// = atoi(strtok(NULL, delim));
            cmd >> linkind;
            int numpoints;// = atoi(strtok(NULL, delim));
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
                cmd >> temp.x;// = (dReal)atof(strtok(NULL, delim));
                cmd >> temp.y;// = (dReal)atof(strtok(NULL, delim));
                cmd >> temp.z;// = (dReal)atof(strtok(NULL, delim));
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
            cmd >> direction.x;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.y;// = (dReal)atof(strtok(NULL, delim));
            cmd >> direction.z;// = (dReal)atof(strtok(NULL, delim));
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

        //p = strtok(NULL, delim);
    }

    std::vector<KinBody::LinkPtr> links = _robot->GetLinks();


    //RAVELOG_INFO("Finished reading in points.\n");

    vector<dReal> closingdir;
    RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();    
//     //make sure we get the right closing direction and don't look at irrelevant joints
//     bool handjoint;
//     for(size_t i = 0; i < _robot->GetDOF(); ++i) 
//     {
//         handjoint = false;
//         for(size_t j = 0; j < pmanip->GetGripperIndices.size(); j++)
//         {
//             if(i == pmanip->GetGripperIndices[j])
//             {
//                 handjoint = true;
//                 
//                 if( pmanip->_vOpenGrasp[j] > pmanip->_vClosedGrasp[j])
//                     closingdir.push_back(-1.0f);
//                 else
//                     closingdir.push_back(1.0f);
// 
//                 break;
//             }
//         }
//         
//       
// 
//         //if(!handjoint)
//         //    closingdir.push_back(0);
//     }
    
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
                        if(lengthsqr3(deltaxyz) < 0.000000001f)
                            deltaxyz = defaultdir;
                        
                        normalize3(deltaxyz,deltaxyz);

                        pointtemp.norm = deltaxyz;
                        vjointvshells[i][j].push_back(pointtemp);
                        if(bdraw)
                        {
                            GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );
                            GetEnv()->drawarrow(RaveVector<float>(pointtemp.pos),RaveVector<float>(pointtemp.pos + 0.01*pointtemp.norm), 0.002f,RaveVector<float>(0,1,0));
                        }
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
                GetEnv()->plot3(RaveVector<float>(pointtemp.pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );
                GetEnv()->drawarrow(RaveVector<float>(pointtemp.pos),RaveVector<float>(pointtemp.pos + 0.01*pointtemp.norm), 0.002f,RaveVector<float>(0,1,0));
            }
        }
    }
    //RAVELOG_INFO("Finished immobile links.\n");
    //_robot->SetActiveDOFValues(NULL,&vlowerlimit[0]);

    std::stringstream outstream;
    //now print the shells to the return string
/*
    outstream << vjointvshells.size() << "\n";
    for(int i = 0; i < vjointvshells.size(); i++)
    {
        outstream << vjointvshells[i].size() << "\n";
        for(int j = 0; j < vjointvshells[i].size(); j++)
        {
            outstream << vjointvshells[i][j].size() << "\n";
            for(int p = 0; p < vjointvshells[i][j].size(); p++)
                outstream << vjointvshells[i][j][p].pos.x << " " << vjointvshells[i][j][p].pos.y << " " << vjointvshells[i][j][p].pos.z << " " << vjointvshells[i][j][p].norm.x << " " << vjointvshells[i][j][p].norm.y << " " << vjointvshells[i][j][p].norm.z << "\n";
        }
    }
    response = outstream.str();
*/

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



