/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Authors:Mike Vande Weghe <vandeweg@cmu.edu>

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


/** \file trajectoryproblem.cpp
    \brief Implementation of the Trajectory Problem class.
 */

#include "stdafx.h"
#include <algorithm>

TrajectoryProblem::TrajectoryProblem(EnvironmentBasePtr penv) : ProblemInstance(penv), ExecuteDefault(false)
{
    __description = ":Interface Author: Mike Vande Weghe\nA trajectory blending plugin that interfaces with Open WAM Driver. \n\n`C++ Documentation <http://www.cs.cmu.edu/~dberenso/docs/manipulation/index.html>`_";
    RegisterCommand("ExecuteBlendedTrajectory",boost::bind(&TrajectoryProblem::ExecuteBlendedTrajectory,this,_1,_2),
                    "blends a trajectory\n");
    RegisterCommand("BlendTrajectory",boost::bind(&TrajectoryProblem::BlendTrajectory,this,_1,_2),
                    "blends a trajectory\n");
    RegisterCommand("Traj",boost::bind(&TrajectoryProblem::Traj,this,_1,_2),
                    "Execute a trajectory from a file on the local filesystem");
    RegisterCommand("VerifyTraj",boost::bind(&TrajectoryProblem::VerifyTraj,this,_1,_2),
                    "Collision check a trajectory at a fine interpolation interval.");
    //RegisterCommand("help",boost::bind(&TrajectoryProblem::Help,this,_1,_2), "display this message.");
}

TrajectoryProblem::~TrajectoryProblem()
{
}

void TrajectoryProblem::Destroy()
{
}

int TrajectoryProblem::main(const string& args)
{
  const char* cmd = args.c_str();
  std::vector<RobotBasePtr> robots;
  GetEnv()->GetRobots(robots);
  const char *delim = " \r\n\t";
  char *cmdstr = strdup(cmd);
  char *p = strtok(cmdstr,delim);
  if (p == NULL) {
    free(cmdstr);
    return 1;
  }
  string robotname = p;
  std::vector<RobotBasePtr>::const_iterator itrobot;
  FORIT(itrobot,robots) {
    if (strcmp((*itrobot)->GetName().c_str(), robotname.c_str()) == 0) {
      robot = *itrobot;
      break;
    }
  }
  free(cmdstr);

#ifdef TRY_CATCH_TEST
  RAVELOG_ERROR("TrajectoryProblem started with arg %s\n",cmd);
  TrajectoryBasePtr pnewtraj = RaveCreateTrajectory(GetEnv());
  OpenRAVE::ConfigurationSpecification cspec =
    pnewtraj->GetConfigurationSpecification();
  try {
    OpenRAVE::ConfigurationSpecification::Group blend_group =
      cspec.GetGroupFromName(std::string("owd_blend_radius"));
  } catch (const OpenRAVE::openrave_exception &or_except) {
    RAVELOG_ERROR("Caught const openrave_exception &.\n");
  } catch (OpenRAVE::openrave_exception &or_except) {
    RAVELOG_ERROR("Caught openrave_exception &.\n");
  } catch (openrave_exception or_except) {
    RAVELOG_ERROR("Caught openrave_exception.\n");
  } catch (const openrave_exception or_except) {
    RAVELOG_ERROR("Caught const openrave_exception.\n");
  }
#endif // TRY_CATCH_TEST
  
  return 0;
}

string getfilename_withseparator(istream& sinput, char separator)
{
    string filename;
    if( !getline(sinput, filename, separator) ) {
        // just input directly
        RAVELOG_ERRORA("graspset filename not terminated with ';'\n");
        sinput >> filename;
    }

    // trim leading spaces
    size_t startpos = filename.find_first_not_of(" \t");
    size_t endpos = filename.find_last_not_of(" \t");

    // if all spaces or empty return an empty string  
    if(( string::npos == startpos ) || ( string::npos == endpos))
        return "";

    filename = filename.substr( startpos, endpos-startpos+1 );
    return filename;
}

bool TrajectoryProblem::Traj(ostream& sout, istream& sinput)
{
    string filename; sinput >> filename;
    if( !sinput )
        return false;

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv());
    
    char sep = ' ';
    if( filename == "sep" ) {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if( filename == "stream" ) {
        // the trajectory is embedded in the stream
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory from stream\n");

        if( !ptraj->deserialize(sinput) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to get trajectory\n");
            return false;
        }
    }
    else {
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory: %s\n", filename.c_str());
        ifstream infile(filename.c_str(),ios::in);
        if( !ptraj->deserialize(infile) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to read trajectory %s\n", filename.c_str());
            infile.close();
            return false;
        }
        infile.close();
    }
        
    bool bResetTrans = false; sinput >> bResetTrans;
    
    // This code broke in OpenRave 0.5.1 because
    // ptraj->GetPoints() is a constant, so the value
    // of its trans cannot be set using
    // itpoint->trans = ...
    // is this code necessary?  why do we feel like we have
    // to set the transformation of every point to the current
    // transform?  why would it be any different from current?
    //
    //    if( bResetTrans ) {
    //        RAVELOG_VERBOSEA("resetting transformations of trajectory\n");
    //        Transform tcur = robot->GetTransform();
    //        // set the transformation of every point to the current robot transformation
    //        FOREACH(itpoint, ptraj->GetPoints())
    //            itpoint->trans = tcur;
    //    }

    RAVELOG_VERBOSEA("executing traj with %d points\n", ptraj->GetNumWaypoints());
    bool success;
    success = robot->GetController()->SetPath(ptraj);
    if (!success)
    {
      RAVELOG_INFO("!! SetPath failed! Controller problem?\n");
      sout << "SetPath failed! Controller problem?\n";
      return false;
    }

    sout << "1";
    return true;
}

bool TrajectoryProblem::VerifyTraj(ostream& sout, istream& sinput)
{
	//get trajectory 
	string filename; sinput >> filename;
    	if( !sinput )
        	return false;

	TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv());
    
    char sep = ' ';
    if( filename == "sep" ) {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if( filename == "stream" ) {
        // the trajectory is embedded in the stream
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory from stream\n");

        if( !ptraj->deserialize(sinput) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to get trajectory\n");
            return false;
        }
    }
    else {
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory: %s\n", filename.c_str());
        ifstream infile(filename.c_str(),ios::in);
        if( !ptraj->deserialize(infile) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to read trajectory %s\n", filename.c_str());
            infile.close();
            return false;
        }
        infile.close();
    } 


    //check every segment with the existing CheckCollision function
    vector<dReal> qresolutioninv(robot->GetActiveDOF(),50.0f);
    if( ptraj->GetNumWaypoints() == 0 ) {
        RAVELOG_INFO("trajectory not initialized\n");
        return false;
    }
    
    std::vector<Trajectory::TPOINT> path;
    FOREACH(itpoint, ptraj->GetPoints())
        path.push_back(*itpoint);

    
    for (int i=0;i<(int)path.size()-1;i++)
    {
        if (CheckCollision(robot, path[i].q, path[i+1].q, qresolutioninv))
        {
            RAVELOG_INFOA("found a collision on the %d to %d segment\n",i,i+1);            
            sout<<"0";
            return true;
        }
    }
    

    sout<<"1";
	return true;
}

bool TrajectoryProblem::ExecuteBlendedTrajectory(ostream& sout, istream& sinput) {
  ExecuteDefault=true;
  bool ret=BlendTrajectory(sout, sinput);
  ExecuteDefault=false;
  return ret;
}

bool TrajectoryProblem::BlendTrajectory(ostream& sout, istream& sinput) {

  bool bExecute = ExecuteDefault;
    bool bIgnoreCollisions = false;
    dReal blend_radius = 0.4f;
    dReal blend_step_size = 0.02f;
    unsigned int blend_attempts = 3;
    dReal linearity_threshold = 0.001;
    int nMaxSmoothIterations = 100;
    TrajectoryBasePtr  ptraj = RaveCreateTrajectory(GetEnv());
    ptraj->Init(robot->GetActiveConfigurationSpecification());
    vector<dReal> qresolutioninv(robot->GetActiveDOF(),50.0f);

    bool bWriteTraj = false;
    string trajfileout;
    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput ) {
            break;
        }
	std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
	bool syntax_error=false;
        if( cmd == "traj") {
            if( !ptraj->deserialize(sinput) ) {
                RAVELOG_ERROR("BlendTrajectory: failed to read trajectory from input stream\n");
		throw openrave_exception("failed to read trajectory from input stream");
            }
	    // deserialize(sinput) will read the remainder of the input
	    // stream, so we won't bother trying to read anything else
	    // afterwards
	    break;
        }
        else if (cmd == "trajfile") {
            string filename;
            sinput >> filename;
            ifstream infile(filename.c_str(),ios::in);
            if( !ptraj->deserialize(infile) ) {
                infile.close();
                RAVELOG_ERROR("failed to read trajectory from file %s\n", filename.c_str());
		throw openrave_exception("failed to read trajectory from file");
            }
            infile.close();
        }
        else if (cmd == "maxsmoothiter") {
            sinput >> nMaxSmoothIterations;
        }
        else if (cmd == "resolution") {
	  FOREACH(it,qresolutioninv)
	    sinput >> *it;
        }
        else if (cmd == "execute") {
            sinput >> bExecute;
        }
	else if (cmd == "ignore_collisions") {
	    sinput >> bIgnoreCollisions;
	}
	else if (cmd == "blend_radius") {
	    sinput >> blend_radius;
	}	  
	else if (cmd == "blend_attempts") {
	    sinput >> blend_attempts;
	}	  
	else if (cmd == "linearity_threshold") {
	    sinput >> linearity_threshold;
	}	 
	else if (cmd == "blend_step_size") {
	    sinput >> blend_step_size;
	}	  
        else if (cmd == "trajfileout") {
            bWriteTraj = true;
            sinput >> trajfileout;
        }
        else {
	  syntax_error=true;
	}
	if (!sinput || syntax_error) {
	  RAVELOG_ERROR("BlendTrajectory: syntax error\n");
	  RAVELOG_INFO("BlendTrajectory usage:\n");
	  RAVELOG_INFO("BlendTrajectory traj <serialized_traj> - reads from stream.  Use this as the last argument only\n");
	  RAVELOG_INFO("BlendTrajectory trajfile <filename>    - reads from file\n");
	  RAVELOG_INFO("\n");
	  RAVELOG_INFO("options: maxsmoothiter <int> - number of smoothing iterations\n");
	  RAVELOG_INFO("         resolution <float> <float> .. <float> - per-joint collision checking\n");
	  RAVELOG_INFO("                                             step size (for straight segments\n");
	  RAVELOG_INFO("         execute [0/1] - whether to send to robot controller\n");
	  RAVELOG_INFO("         blend_radius <float> - blend size in radians\n");
	  RAVELOG_INFO("         linearity_threshold  <float> - tolerance for linearsegments\n");
	  RAVELOG_INFO("         blend_attempts <int> - how many tries to make for a single point\n");
	  RAVELOG_INFO("                                (blend radius will be cut in half each try\n");
	  RAVELOG_INFO("         blend_step_size <float> - path step size (radians) for\n");
	  RAVELOG_INFO("                                   checking blends\n");;
	  throw openrave_exception("syntax error");
        }
    }

    if( ptraj->GetNumWaypoints() == 0 ) {
        RAVELOG_ERROR("trajectory is empty\n");
	throw openrave_exception("trajectory is empty");
    }

    TrajectoryBasePtr pnewtraj = ExtractCorners(ptraj, linearity_threshold);
    RAVELOG_DEBUG("number of corners: %d\n",pnewtraj->GetNumWaypoints());

    if (!CalculateTrajectoryBlends(pnewtraj,robot,blend_radius,blend_attempts,blend_step_size,GetEnv(),bIgnoreCollisions)) {
      RAVELOG_ERROR("CalculateTrajectoryBlends failed\n");
      throw openrave_exception("CalculateTrajectoryBlends failed");
    }
    
    int blend_index(-1);
    try {
      RAVELOG_DEBUG("Final blend radii:\n");
      OpenRAVE::ConfigurationSpecification cspec =
	pnewtraj->GetConfigurationSpecification();
      OpenRAVE::ConfigurationSpecification::Group blend_group =
	cspec.GetGroupFromName(std::string("owd_blend_radius"));
      blend_index = blend_group.offset;
      for (size_t j=0; j<pnewtraj->GetNumWaypoints(); ++j) {
	std::vector<dReal> point;
	pnewtraj->GetWaypoint(j,point);
	RAVELOG_DEBUG("%d = %1.3f\n",j,point[blend_index]);
      }
    } catch (const openrave_exception &or_except) {
      RAVELOG_WARN("Could not find owd_blend_radius group in new trajectory\n");
    } catch (...) {
      RAVELOG_WARN("Could not find owd_blend_radius group in new trajectory and something other than openrave_exception was thrown\n");
    }

    if( bExecute ) {
      bool success;
      pnewtraj->CalcTrajTiming(robot, pnewtraj->GetInterpMethod(), true, false); //this retiming is just for playback using idealcontroller
      success = robot->GetController()->SetPath(pnewtraj);
      if (!success)
      {
        RAVELOG_WARN("!! SetPath failed! Controller problem?\n");
        throw openrave_exception("SetPath failed");
      }
    } else if(bWriteTraj) {
      ofstream outstream(trajfileout.c_str(),ios::out);
      pnewtraj->serialize(outstream);
      outstream.close();
    } else {
      // return the serialized trajectory in the output stream
      pnewtraj->serialize(sout);
    }
    return true;
}







//bool TrajectoryProblem::Help(ostream& sout, istream& sinput)
//{
//    sout << "TrajectoryProblem help:" << endl;
//    GetCommandHelp(sout,sinput);
//    return true;
//}

bool TrajectoryProblem::CalculateTrajectoryBlends(TrajectoryBasePtr ptraj, RobotBasePtr robot, dReal blend_radius, unsigned int attempts, dReal step_size, EnvironmentBasePtr pEnviron, bool bIgnoreCollisions) {

  ConfigurationSpecification cspec = 
    ptraj->GetConfigurationSpecification();
  ConfigurationSpecification::Group jval_group;
  try {
    jval_group = cspec.GetGroupFromName(std::string("joint_values"));
  } catch (const OpenRAVE::openrave_exception &or_except) {
    RAVELOG_WARN("TrajectoryProblem::CalculateTrajectoryBlends: No joint_values group found in this trajectory.\n");
    return false;
  } catch (...) {
    RAVELOG_DEBUG("Could not find joint_values group in new trajectory and something other than openrave_exception was thrown\n");
    return false;
  }

  RAVELOG_DEBUG("Beginning to add blends to a %d DOF trajectory\n",
		jval_group.dof);
  RAVELOG_DEBUG("Robot active DOF is %d\n",robot->GetActiveDOF());

  std::vector<dReal> jointdata;
  std::vector<dReal> blend_radii;
  size_t trajsize = ptraj->GetNumWaypoints();
  ConfigurationSpecification::Group blend_group;
  try {
    blend_group = cspec.GetGroupFromName(std::string("owd_blend_radius"));
  } catch (const OpenRAVE::openrave_exception &or_except) {
    // the blend group did not exist, so we need to recreate
    // the trajectory with space for the blend radii
    cspec.AddGroup(std::string("owd_blend_radius"),1,"previous");
    try {
      blend_group = cspec.GetGroupFromName(std::string("owd_blend_radius"));
    } catch (const OpenRAVE::openrave_exception &or_except) {
      RAVELOG_ERROR("Still could not find group owd_blend_radius even after adding it\n");
      return false;
    } catch (...) {
      RAVELOG_ERROR("Still could not find group owd_blend_radius even after adding it, and something other than openrave_exception was thrown\n");
      return false;
    }

    std::vector<dReal> trajdata;
    ptraj->GetWaypoints(0,trajsize,trajdata,cspec);
    ptraj->Init(cspec);
    ptraj->Insert(0,trajdata);
  } catch (...) {
    RAVELOG_DEBUG("Could not find group blend_radius and something other than openrave_exception was thrown.\n");
    // the blend group did not exist, so we need to recreate
    // the trajectory with space for the blend radii
    cspec.AddGroup(std::string("owd_blend_radius"),1,"previous");
    try {
      blend_group = cspec.GetGroupFromName(std::string("owd_blend_radius"));
    } catch (const openrave_exception &or_except) {
      RAVELOG_ERROR("Still could not find group owd_blend_radius even after adding it\n");
      return false;
    } catch (...) {
      RAVELOG_ERROR("Still could not find group owd_blend_radius even after adding it, and something other than openrave_exception was thrown\n");
      return false;
    }
    std::vector<dReal> trajdata;
    // extract the existing data
    ptraj->GetWaypoints(0,trajsize,trajdata,cspec);
    // reinitialize the trajectory with the new config spec (so that it has space for the blends)
    ptraj->Init(cspec);
    // reinsert the original data into the new trajectory
    ptraj->Insert(0,trajdata);
  }
  // now get all the joint values
  ConfigurationSpecification jointspec;
  jointspec.AddGroup(jval_group.name,
		     jval_group.dof,
		     jval_group.interpolation);
  ptraj->GetWaypoints(0,trajsize,jointdata,jointspec);
  size_t jointdof = jval_group.dof;

  blend_radii.resize(trajsize);
  int collision_check=0;
  int collision_count=0;
  //  std::vector<double> tempv;
  RAVELOG_DEBUG("Beginning blend computation\n");
  for (unsigned int i=1; i<trajsize-1; ++i) {
    std::vector<dReal>::iterator it = jointdata.begin();
    JointPos first_p, second_p, third_p;
    first_p.insert(first_p.begin(),
		   it + (i-1) * jointdof,
		   it + (i-1) * jointdof + jointdof);
    second_p.insert(second_p.begin(),
		    it +  i   * jointdof,
		    it  +  i   * jointdof + jointdof);
    third_p.insert(third_p.begin(),
		   it + (i+1) * jointdof,
		   it + (i+1) * jointdof + jointdof);
    JointPos direction1 = second_p - first_p;
    JointPos direction2 = third_p - second_p;
    dReal distance1 = direction1.length();
    dReal distance2 = direction2.length();
    direction1 /= distance1;
    direction2 /= distance2;
    dReal b_radius = RaveFabs(blend_radius);
    dReal prev_radius = blend_radii[i-1];
    for (int a = attempts; a > 0; --a) {
      // only try the blend if we have enough space for it
      if (((distance1-prev_radius) > 2*b_radius) 
	  && (distance2 > 3*b_radius)) {
	JointPos blend_start(second_p - direction1 * b_radius);  
	JointPos blend_end(second_p + direction2 * b_radius);
	JointPos b1(blend_start);
	JointPos m1(direction1 *2 *b_radius);
	JointPos m2(direction2 *2 *b_radius);
	JointPos b2(blend_end - m2);
	JointPos m2_minus_m1(m2-m1);
	bool collision=false;

	if (!bIgnoreCollisions) {
          // run through the blend checking for collisions.
          // blend will cover 2*b_radius distance for sigma 0 to 1,
          // so we will increment sigma by step_size/(2*b_radius).
          for (dReal sigma = 0; (sigma <= 1.0) && !collision; 
               sigma += step_size/(2*b_radius)) {
            dReal sigma2=sigma*sigma;
            dReal sigma3=sigma2*sigma;
            dReal sigma4=sigma3*sigma;
            dReal sigma5=sigma4*sigma;
            dReal sigma6=sigma5*sigma;
            dReal alpha=6 * sigma5 - 15 * sigma4 + 10 * sigma3;
            dReal alphadot = 30 * sigma4 - 60 * sigma3 + 30 * sigma2;
            dReal alphadotdot = 120 * sigma3 - 180 * sigma2 + 60 * sigma;
            dReal beta = sigma6 - 3 * sigma5 + 3 * sigma4 - sigma3;
            dReal betadot = 6 * sigma5 - 15 * sigma4 + 12 * sigma3 - 3 * sigma2;
            dReal betadotdot = 30 * sigma4 - 60 * sigma3 + 36 * sigma2 - 6 * sigma;
            static const dReal k=7.5;
            
            JointPos x2_minus_x1 = b2 - b1 + m2_minus_m1*sigma;
	  
            JointPos x = b1
              + m1*sigma 
              + x2_minus_x1*alpha
              - m2_minus_m1*(k*beta);
            
	    RAVELOG_DEBUG("About to SetActiveDOFValues\n");
            robot->SetActiveDOFValues(x);
	    RAVELOG_DEBUG("Finished SetActiveDOFValues\n");
            ++collision_check;
            if (robot->CheckSelfCollision() ||
                pEnviron->CheckCollision(KinBodyConstPtr(robot))) {
              collision=true;
              ++collision_count;
              //RAVELOG_INFO("Point %d radius %1.3f: hit at sigma=%1.4f\n",i,b_radius,sigma);
              break;
            }
	    RAVELOG_DEBUG("Done collision checking for this config\n");
          }
        }
        if (!collision) {
	  blend_radii[i] = b_radius;  // yay!
          RAVELOG_DEBUG("Point %d radius %1.3f: clear\n",i,b_radius);
          break;
        }
      } else {
	RAVELOG_DEBUG("Point %d radius %1.3f: skipped\n",i,b_radius);
      }
      b_radius /= 2.0; // try a smaller radius next time
    }
    if (blend_radii[i]==0) {
      RAVELOG_DEBUG("Point %d: no blend added\n",i);
    }
  }
  RAVELOG_DEBUG("Collisions: %d out of %d checks\n",
                collision_count,collision_check);

  // now insert the blend radii back into the trajectory
  ConfigurationSpecification blendspec;
  blendspec.AddGroup(blend_group.name,
		     blend_group.dof,
		     blend_group.interpolation);
  ptraj->Insert(0,blend_radii,blendspec,true);

  return true;
}
  

bool TrajectoryProblem::CheckCollision(RobotBasePtr probot, const vector<dReal>& q0, const vector<dReal>& q1, const vector<dReal>& qresolutioninv)
{
    //this is for trajectory points and they should have all dofs
    assert( probot != NULL && probot->GetActiveDOF() == q0.size() && probot->GetActiveDOF()== q1.size() && probot->GetActiveDOF() == qresolutioninv.size() );
    
    // set the bounds based on the interval type
    int start = 0;

    // first make sure the end is free
    vector<dReal> vtempconfig(probot->GetActiveDOF()), jointIncrement(probot->GetActiveDOF());

    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < (int)vtempconfig.size(); i++) {
        int steps = (int)(fabs(q1[i] - q0[i]) * qresolutioninv[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < (int)vtempconfig.size(); i++)
        jointIncrement[i] = (q1[i] - q0[i])/((float)numSteps);

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    vector<int> activeIndices = probot->GetActiveDOFIndices();
    for (int f = start; f <= numSteps; f++) {

        for (i = 0; i < (int)vtempconfig.size(); i++)
            vtempconfig[i] = q0[i] + (jointIncrement[i] * f);
        
        //make a vector just for the active indices
        vector<dReal> vtempconfig_active;
        for (i = 0; i < (int)activeIndices.size(); i++)
        {
            vtempconfig_active.push_back(vtempconfig[activeIndices[i]]);
        }    
	RAVELOG_DEBUG("About to SetActiveDOFValues\n");
        probot->SetActiveDOFValues(vtempconfig_active);
	RAVELOG_DEBUG("Finished SetActiveDOFValues\n");
        if( GetEnv()->CheckCollision(KinBodyConstPtr(probot)) || probot->CheckSelfCollision() )
            return true;
    }

    return false;
}

void TrajectoryProblem::OptimizePathRandomized(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv, int nMaxIterations)
{
    if( path.size() <= 2 )
        return;

    list< Trajectory::TPOINT >::iterator itstart, itend;
    
    u32 nrejected = 0;
    int i = nMaxIterations;
    while(i > 0 && nrejected < path.size() ) {

        --i;

        // pick a random node on the path, and a random jump ahead
        int endIndex = 1+RANDOM_INT((int)path.size()-1);
        int startIndex = RANDOM_INT(endIndex);
        
        itstart = path.begin();
        advance(itstart, startIndex);
        itend = itstart;
        advance(itend, endIndex-startIndex);
        nrejected++;

        // check if the nodes can be connected by a straight line
        if (CheckCollision(probot, itstart->q, itend->q, qresolutioninv)) {
            if( nrejected++ > path.size()*2 )
                break;
            continue;
        }

        // splice out in-between nodes in path
        path.erase(++itstart, itend);
        nrejected = 0;

        if( path.size() <= 2 )
            return;
    }
}

// check all pairs of nodes
void TrajectoryProblem::OptimizePathAll(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv)
{
    list< Trajectory::TPOINT >::iterator itstart = path.begin();
    while(itstart != path.end() ) {
        
        list< Trajectory::TPOINT >::iterator itend = path.end();
        while(--itend != itstart) {
            if (!CheckCollision(probot, itstart->q, itend->q, qresolutioninv)) {
                // splice out in-between nodes in path
                list< Trajectory::TPOINT >::iterator itnext = itstart;
                path.erase(++itnext, itend);
                break;
            }
        }

        ++itstart;
    }
}

TrajectoryBasePtr TrajectoryProblem::ExtractCorners(const TrajectoryBasePtr ptraj, dReal linearity_threshold) {
  TrajectoryBasePtr pnewtraj = RaveCreateTrajectory(GetEnv());
  pnewtraj->Init(ptraj->GetConfigurationSpecification());

  ConfigurationSpecification cspec = ptraj->GetConfigurationSpecification();
  ConfigurationSpecification::Group jval_group;
  try {
    jval_group = cspec.GetGroupFromName(std::string("joint_values"));
  } catch (const OpenRAVE::openrave_exception &or_except) {
    RAVELOG_WARN("TrajectoryProblem::ExtractCorners: No joint_values group found in this trajectory.\n");
    throw openrave_exception("No joint_values in trajectory");
  } catch (...) {
    RAVELOG_WARN("TrajectoryProblem::ExtractCorners: No joint_values group found in this trajectory and something other than openrave_exception was thrown\n");
    throw openrave_exception("No joint_values in trajectory");
  }
  // now make a new config spec with just the joint values so that we can
  // use it to extract points from the trajectory
  ConfigurationSpecification jointspec;
  jointspec.AddGroup(jval_group.name,
		     jval_group.dof,
		     jval_group.interpolation);

  // if the trajectory has only a single point, just return that point
  if (ptraj->GetNumWaypoints() == 1) {
    JointPos p1;
    ptraj->GetWaypoint(0,p1);
    pnewtraj->Insert(0,p1);
    return pnewtraj;
  }

  // if it has 2 points, only keep them both if the joint vals are unique
  if (ptraj->GetNumWaypoints() == 2) {
    JointPos p1, p2;
    ptraj->GetWaypoint(0,p1, jointspec);
    ptraj->GetWaypoint(1,p2, jointspec);
    if (p1 == p2) {
      // get the full waypoint
      ptraj->GetWaypoint(0,p1);
      // send back the single unique point
      pnewtraj->Insert(0,p1);
      return pnewtraj;
    }
    // otherwise return both points after re-retrieving them
    //   with all their waypoint values (time, vel, etc)
    ptraj->GetWaypoint(0,p1);
    pnewtraj->Insert(0,p1);
    ptraj->GetWaypoint(1,p2);
    pnewtraj->Insert(1,p2);
    return pnewtraj;
  }
    
  // if there are 3 or more points, then run the corner-finding algorithm

  // line from start to end
  JointPos first_p, last_p;
  ptraj->GetWaypoint(0,first_p, jointspec);
  ptraj->GetWaypoint(ptraj->GetNumWaypoints()-1,last_p, jointspec);
  
  // calc distance from each point to line
  double max_dist = 0;
  unsigned int furthest_point = 0;
  JointPos v(last_p - first_p);
  for (int i=1; i<ptraj->GetNumWaypoints()-1; ++i) {
    
    JointPos current_p;
    ptraj->GetWaypoint(i,current_p, jointspec);
    JointPos w(current_p - first_p);
    dReal c1 = w*v;
    dReal c2 = v*v;
    dReal b = c1 / c2;
    JointPos Pb = first_p + v*b;
    JointPos diff = current_p - Pb;
    dReal dist = diff.length();

    if (dist > max_dist) {
      max_dist = dist;
      furthest_point = i;
    }
  }

  // if all distances are less than threshold, return endpoints.
  if (max_dist < linearity_threshold) {
    // get the full waypoint
    ptraj->GetWaypoint(0,first_p);
    pnewtraj->Insert(0,first_p);
    ptraj->GetWaypoint(ptraj->GetNumWaypoints()-1,last_p);
    pnewtraj->Insert(1,last_p);
    return pnewtraj;
  }

  // otherwise, take point with max distance and use it to split into
  // 2 new groups
  JointPos p;
  TrajectoryBasePtr first_half = RaveCreateTrajectory(GetEnv());
  first_half->Init(ptraj->GetConfigurationSpecification());
  for (int i=furthest_point; i>=0; --i) {
    ptraj->GetWaypoint(i,p);
    first_half->Insert(0,p);
  }
  TrajectoryBasePtr second_half = RaveCreateTrajectory(GetEnv());
  second_half->Init(ptraj->GetConfigurationSpecification());
  for (int i=ptraj->GetNumWaypoints()-1; i>=furthest_point; --i) {
    ptraj->GetWaypoint(i,p);
    second_half->Insert(0,p);
  }
  // call extractcorners on each vector
  first_half = ExtractCorners(first_half, linearity_threshold);
  second_half = ExtractCorners(second_half, linearity_threshold);

  // add two trajectories together and return (don't duplicate max point)
  if (first_half->GetNumWaypoints() == 1) {
    first_half->GetWaypoint(0,p);
    second_half->Insert(0,p);
  } else {
    for (int i=first_half->GetNumWaypoints()-2; i>=0; --i) {
      first_half->GetWaypoint(i,p);
      second_half->Insert(0,p);
    }
  }
  return second_half;
}

void JointPos::cpy(dReal *out) const {
  for (unsigned int i=0; i<size(); ++i) {
    out[i]=this->operator[](i);
  }
}

dReal JointPos::length() const {
  dReal sum=0;
  for (unsigned int i=0; i<size(); ++i) {
    sum += pow(this->operator[](i),2);
  }
  return sqrt(sum);
}

void JointPos::dump() const {
  printf("[ ");
  for (unsigned int i=0; i<size(); ++i) {
    printf("%2.3f ",this->operator[](i));
  }
  printf("]\n");
}
