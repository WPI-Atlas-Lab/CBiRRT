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
/** \file TaskSpaceRegion.cpp
    \brief Defines the Task Space Region and Task Space Region Chain class.es
 */
#include "stdafx.h"

TaskSpaceRegion::TaskSpaceRegion()
{ 
    manipind = -1; 
    Bw[0][0] = 0;
    Bw[0][1] = 0;
    Bw[1][0] = 0;
    Bw[1][1] = 0;
    Bw[2][0] = 0;
    Bw[2][1] = 0;
    Bw[3][0] = 0;
    Bw[3][1] = 0;
    Bw[4][0] = 0;
    Bw[4][1] = 0;
    Bw[5][0] = 0;
    Bw[5][1] = 0;

    _volume = -1; 
    _sumbounds = -1;
    _dimensionality = -1;

}

bool TaskSpaceRegion::Initialize(EnvironmentBasePtr penv_in)
{
    _volume = 0;
    _sumbounds = 0;
    _dimensionality = 0;

    for(int i = 0; i < 6; i++)
    { 
        //compute volume in whatever dimensionality this defines                
        //when Bw values are backwards, it signifies an axis flip (not an error)
        if(Bw[i][1] != Bw[i][0])
        {
            _volume = _volume * fabs(Bw[i][1] - Bw[i][0]);
            _sumbounds = _sumbounds + fabs(Bw[i][1] - Bw[i][0]);
            _dimensionality++;
        }
    }

    if( stricmp(relativebodyname.c_str(), "NULL") == 0 )
    {
        prelativetolink.reset();
    }
    else
    {
        KinBodyPtr pobject;
        pobject = penv_in->GetKinBody(relativebodyname.c_str());
        if(pobject.get() == NULL)
        {
            RAVELOG_INFO("Error: could not find the specified object to attach frame\n");
            return false;
        }
       
        //find the link
        vector<KinBody::LinkPtr> vlinks = pobject->GetLinks();
        bool bGotLink = false;
        for(int j =0; j < vlinks.size(); j++)
        {
            if(strcmp(relativelinkname.c_str(), vlinks[j]->GetName().c_str()) == 0 )
            {
                RAVELOG_INFO("frame link: %s:%s\n",vlinks[j]->GetParent()->GetName().c_str(),vlinks[j]->GetName().c_str() );
                prelativetolink = vlinks[j];
                bGotLink = true;
                break;
            }
        }
        if(!bGotLink)
        {
            RAVELOG_INFO("Error: could not find the specified link of the object to attach frame\n");
            return false;
        }       
    }  


    //Print();
    return true;
}

Vector TaskSpaceRegion::RPYIdentityOffsets[8] = { Vector(M_PI,M_PI,M_PI),
                                                  Vector(M_PI,M_PI,-M_PI),
                                                  Vector(M_PI,-M_PI,M_PI),
                                                  Vector(M_PI,-M_PI,-M_PI),
                                                  Vector(-M_PI,M_PI,M_PI),
                                                  Vector(-M_PI,M_PI,-M_PI),
                                                  Vector(-M_PI,-M_PI,M_PI),
                                                  Vector(-M_PI,-M_PI,-M_PI)};

void TaskSpaceRegion::QuatToRPY(dReal* quat, dReal& psi, dReal& theta, dReal& phi)
{
    a = quat[0];
    b = quat[1];
    c = quat[2];
    d = quat[3];

    //psi theta and phi will always be between -pi and pi
    psi = atan2(2*a*b + 2*c*d, a*a - b*b - c*c + d*d); //psi
    theta = -asin(2*b*d-2*a*c); //theta
    phi = atan2(2*a*d+2*b*c, a*a + b*b - c*c - d*d); //phi
    

    //go through all the identities and find which one minimizes the total rotational distance
    //don't need to consider +/-2pi b/c all three angles are between -pi and pi
    _min_dist = 10000;
    for(int i =0; i < 9; i++)
    {

        if(i == 0) //for first element, use original values
        {
            _temp_vec.x = psi;
            _temp_vec.y = theta;
            _temp_vec.z = phi;
        }
        else
        {
            _temp_vec.x = psi + RPYIdentityOffsets[i-1].x;
            _temp_vec.y = -theta + RPYIdentityOffsets[i-1].y;//note that theta is negative
            _temp_vec.z = phi + RPYIdentityOffsets[i-1].z;
        }
        
        _temp_dist = _temp_vec.lengthsqr3();
        if(_temp_dist < _min_dist)
        {
            _min_dist = _temp_dist;
            psi = _temp_vec.x;
            theta = _temp_vec.y;
            phi = _temp_vec.z;
        }
    }
    //RAVELOG_INFO("psi: %f, theta: %f, phi: %f\n",psi,theta,phi);
}


void TaskSpaceRegion::RPYToQuat(dReal* rpy, dReal* quat)
{
    _cphi = cos(rpy[0]/2);
    _sphi = sin(rpy[0]/2);
    _ctheta = cos(rpy[1]/2);
    _stheta = sin(rpy[1]/2);
    _cpsi = cos(rpy[2]/2);
    _spsi = sin(rpy[2]/2);

    quat[0] = _cphi*_ctheta*_cpsi + _sphi*_stheta*_spsi;
    quat[1] = _sphi*_ctheta*_cpsi - _cphi*_stheta*_spsi;
    quat[2] = _cphi*_stheta*_cpsi + _sphi*_ctheta*_spsi;
    quat[3] = _cphi*_ctheta*_spsi - _sphi*_stheta*_cpsi;

}

dReal TaskSpaceRegion::DistanceToTSR(const Transform& T0_s, std::vector<dReal>& dx)
{
    dx.resize(6);
    
    if(prelativetolink.get() == NULL)
        T0_link = Transform();
    else
        T0_link = prelativetolink->GetTransform();

    Tw_s1 = (T0_link*T0_w).inverse()*T0_s*Tw_e.inverse();


    //convert to task coordinates
    dx[0] = Tw_s1.trans.x;
    dx[1] = Tw_s1.trans.y;
    dx[2] = Tw_s1.trans.z;

    QuatToRPY(&Tw_s1.rot[0],dx[3],dx[4],dx[5]);


    
    sumsqr = 0;

    for(int i = 0; i < 6; i++)
    {   
        if(dx[i] > Bw[i][1])
            dx[i] = dx[i] - Bw[i][1];
        else if(dx[i] < Bw[i][0])
            dx[i] = dx[i] - Bw[i][0];
        else
            dx[i] = 0;
        sumsqr += dx[i]*dx[i];
    }        

    return sqrt(sumsqr);
}


Transform TaskSpaceRegion::GetClosestTransform(const Transform& T0_s)
{
    //dw_sample is just used here b/c it's convenient, it's not actually sampling anything

    if(prelativetolink.get() == NULL)
        T0_link = Transform();
    else
        T0_link = prelativetolink->GetTransform();

    Tw_s1 = (T0_link*T0_w).inverse()*T0_s*Tw_e.inverse();

    //convert to task coordinates
    dw_sample[0] = Tw_s1.trans.x;
    dw_sample[1] = Tw_s1.trans.y;
    dw_sample[2] = Tw_s1.trans.z;
    QuatToRPY(&Tw_s1.rot[0],dw_sample[3],dw_sample[4],dw_sample[5]);

    //RAVELOG_INFO("dw_s: %f %f %f %f %f %f\n",dw_sample[0],dw_sample[1],dw_sample[2],dw_sample[3],dw_sample[4],dw_sample[5]);

    for(int i = 0; i < 6; i++)
    {   
        if(dw_sample[i] > Bw[i][1])
            dw_sample[i] = Bw[i][1];
        else if(dw_sample[i] < Bw[i][0])
            dw_sample[i] = Bw[i][0];
    }        
    //RAVELOG_INFO("closest: %f %f %f %f %f %f\n",dw_sample[0],dw_sample[1],dw_sample[2],dw_sample[3],dw_sample[4],dw_sample[5]);
      
    Tw_rand.trans.x = dw_sample[0];
    Tw_rand.trans.y = dw_sample[1];
    Tw_rand.trans.z = dw_sample[2];
    RPYToQuat(&dw_sample[3],&Tw_rand.rot[0]);
   
   
    if(prelativetolink.get() == NULL)
        T0_link = Transform();
    else
        T0_link = prelativetolink->GetTransform();

    return (T0_link*T0_w*Tw_rand*Tw_e);

}

Transform TaskSpaceRegion::GenerateSample()
{
    for(int i = 0; i < 6; i++)
    {
        frand = RANDOM_FLOAT();
        dw_sample[i] = Bw[i][1]*frand + Bw[i][0]*(1-frand);
    }

    RPYToQuat(&dw_sample[3],&Tw_rand.rot[0]);
   
    Tw_rand.trans.x = dw_sample[0];
    Tw_rand.trans.y = dw_sample[1];
    Tw_rand.trans.z = dw_sample[2];
    
    if(prelativetolink.get() == NULL)
        T0_link = Transform();
    else
        T0_link = prelativetolink->GetTransform();

    return (T0_link*T0_w*Tw_rand*Tw_e);
}



void TaskSpaceRegion::Print()
{     
    stringstream O;
    O << endl;
    O << "Manipulator Pointer: " << manipind << " ";
    O << endl;

    O << "Link Pointer: ";
    if(prelativetolink.get() == NULL)
      O << "NULL";
    else
      O << prelativetolink->GetParent()->GetName() << ":" << prelativetolink->GetName() << " ";
    O << endl;

    O << "T0_w:"<<endl;
    O << T0_w.rot.x << " " << T0_w.rot.y << " " << T0_w.rot.z << " " << T0_w.rot.w << " ";
    O << T0_w.trans.x << " " << T0_w.trans.y << " " << T0_w.trans.z << " ";
    O << endl;

    O << "Tw_e:"<<endl;
    O << Tw_e.rot.x << " " << Tw_e.rot.y << " " << Tw_e.rot.z << " " << Tw_e.rot.w << " ";
    O << Tw_e.trans.x << " " << Tw_e.trans.y << " " << Tw_e.trans.z << " ";
    O << endl;

    O << "Bw:"<<endl;
    O << Bw[0][0] << " ";
    O << Bw[0][1] << " ";
    O << endl;
    O << Bw[1][0] << " ";
    O << Bw[1][1] << " ";
    O << endl;
    O << Bw[2][0] << " ";
    O << Bw[2][1] << " ";
    O << endl;
    O << Bw[3][0] << " ";
    O << Bw[3][1] << " ";
    O << endl;
    O << Bw[4][0] << " ";
    O << Bw[4][1] << " ";
    O << endl;
    O << Bw[5][0] << " ";
    O << Bw[5][1] << " ";
    O << endl;
    RAVELOG_INFO(O.str().c_str());
}


bool TaskSpaceRegion::serialize(std::ostream& O) const
{
    O << manipind << " ";

    if( prelativetolink.get() == NULL)
    {
        O << "NULL" << " ";
    }
    else
    {
        O << prelativetolink->GetParent()->GetName() << " ";
        O << prelativetolink->GetName() << " ";
    }
    
    O << T0_w.rot.x << " " << T0_w.rot.y << " " << T0_w.rot.z << " " << T0_w.rot.w << " ";
    O << T0_w.trans.x << " " << T0_w.trans.y << " " << T0_w.trans.z << " ";

    O << Tw_e.rot.x << " " << Tw_e.rot.y << " " << Tw_e.rot.z << " " << Tw_e.rot.w << " ";
    O << Tw_e.trans.x << " " << Tw_e.trans.y << " " << Tw_e.trans.z << " ";

    O << Bw[0][0] << " ";
    O << Bw[0][1] << " ";

    O << Bw[1][0] << " ";
    O << Bw[1][1] << " ";

    O << Bw[2][0] << " ";
    O << Bw[2][1] << " ";

    O << Bw[3][0] << " ";
    O << Bw[3][1] << " ";

    O << Bw[4][0] << " ";
    O << Bw[4][1] << " ";

    O << Bw[5][0] << " ";
    O << Bw[5][1] << " ";
    return true;
}

bool TaskSpaceRegion::deserialize(std::stringstream& _ss)
{

    //RAVELOG_INFO(_ss.str().c_str());
    _ss >> manipind;

    string tempstring;
    _ss >> relativebodyname;
    
    if( relativebodyname != "NULL" )
    {
        _ss >> relativelinkname;  
    }  
    
    _ss >> T0_w.rot.x;
    _ss >> T0_w.rot.y;
    _ss >> T0_w.rot.z;
    _ss >> T0_w.rot.w;

    _ss >> T0_w.trans.x;
    _ss >> T0_w.trans.y;
    _ss >> T0_w.trans.z;

    _ss >> Tw_e.rot.x;
    _ss >> Tw_e.rot.y;
    _ss >> Tw_e.rot.z;
    _ss >> Tw_e.rot.w;

    _ss >> Tw_e.trans.x;
    _ss >> Tw_e.trans.y;
    _ss >> Tw_e.trans.z;

    _ss >> Bw[0][0];
    _ss >> Bw[0][1];

    _ss >> Bw[1][0];
    _ss >> Bw[1][1];

    _ss >> Bw[2][0];
    _ss >> Bw[2][1];

    _ss >> Bw[3][0];
    _ss >> Bw[3][1];

    _ss >> Bw[4][0];
    _ss >> Bw[4][1];

    _ss >> Bw[5][0];
    _ss >> Bw[5][1];

    //Print();
    return true;
}

bool TaskSpaceRegion::deserialize_from_matlab(RobotBasePtr  robot_in, EnvironmentBasePtr penv_in, std::istream& _ss)
{
    assert(robot_in.get() != NULL);

    int tempint;
    string tempstring;
    TransformMatrix temptm;

    //get pointer to manipulator at this index
    _ss >> tempint;
    if(tempint >= robot_in->GetManipulators().size() || tempint < 0)
    {
        RAVELOG_INFO("Error: Manipulator index out of bounds\n");
        return false;
    }
    else
        manipind = tempint;


    _ss >> tempstring;
    if( stricmp(tempstring.c_str(), "NULL") == 0 )
    {
        prelativetolink.reset();
    }
    else
    {
        KinBodyPtr pobject;
        pobject = penv_in->GetKinBody(tempstring.c_str());
        if(pobject.get() == NULL)
        {
            RAVELOG_INFO("Error: could not find the specified object to attach frame\n");
            return false;
        }
        _ss >> tempstring;  
        //find the link
        vector<KinBody::LinkPtr> vlinks = pobject->GetLinks();
        bool bGotLink = false;
        for(int j =0; j < vlinks.size(); j++)
        {
            if(strcmp(tempstring.c_str(), vlinks[j]->GetName().c_str()) == 0 )
            {
                RAVELOG_INFO("frame link: %s:%s\n",vlinks[j]->GetParent()->GetName().c_str(),vlinks[j]->GetName().c_str() );
                prelativetolink = vlinks[j];
                bGotLink = true;
                break;
            }
        }
        if(!bGotLink)
        {
            RAVELOG_INFO("Error: could not find the specified link of the object to attach frame\n");
            return false;
        }       
    }   


    _ss >> temptm.m[0];
    _ss >> temptm.m[4];
    _ss >> temptm.m[8];
    _ss >> temptm.m[1];        
    _ss >> temptm.m[5];
    _ss >> temptm.m[9];
    _ss >> temptm.m[2];
    _ss >> temptm.m[6];
    _ss >> temptm.m[10];
    _ss >> temptm.trans.x;
    _ss >> temptm.trans.y;
    _ss >> temptm.trans.z;
    T0_w = Transform(temptm);

    _ss >> temptm.m[0];
    _ss >> temptm.m[4];
    _ss >> temptm.m[8];
    _ss >> temptm.m[1];
    _ss >> temptm.m[5];
    _ss >> temptm.m[9];
    _ss >> temptm.m[2];
    _ss >> temptm.m[6];
    _ss >> temptm.m[10];
    _ss >> temptm.trans.x;
    _ss >> temptm.trans.y;
    _ss >> temptm.trans.z;
    Tw_e = Transform(temptm);

    _ss >> Bw[0][0];
    _ss >> Bw[0][1];

    _ss >> Bw[1][0];
    _ss >> Bw[1][1];

    _ss >> Bw[2][0];
    _ss >> Bw[2][1];

    _ss >> Bw[3][0];
    _ss >> Bw[3][1];

    _ss >> Bw[4][0];
    _ss >> Bw[4][1];

    _ss >> Bw[5][0];
    _ss >> Bw[5][1];

    //Print();
    return true;
}


Transform TaskSpaceRegionChain::GenerateSample()
{
    for(int i = 1; i < TSRChain.size();i++)
        TSRChain[i].T0_w = TSRChain[i-1].GenerateSample();

    return TSRChain[TSRChain.size() - 1].GenerateSample();
}


bool TaskSpaceRegionChain::GetChainJointLimits(dReal* lowerlimits, dReal* upperlimits)
{
    for(int i = 0; i < _lowerlimits.size();i++)
    {
        lowerlimits[i] = _lowerlimits[i];
        upperlimits[i] = _upperlimits[i];
        RAVELOG_DEBUG("lower: %f   upper: %f\n",lowerlimits[i],upperlimits[i]);
    }

    return true;
}


dReal TaskSpaceRegionChain::GetClosestTransform(const Transform& T0_s, dReal * TSRJointVals, Transform& T0_closeset)
{
    assert(robot.get() != NULL);


    if(_bPointTSR)
    {
        T0_closeset = TSRChain[0].GetClosestTransform(T0_s);
        return TSRChain[0].DistanceToTSR(T0_s, _dx);
    }

    assert(TSRJointVals != NULL);

    


    Transform Ttarg = T0_s*TSRChain[TSRChain.size()-1].Tw_e.inverse();

    ikparams[2] = Ttarg.rot.x; ikparams[3] = Ttarg.rot.y; ikparams[4] = Ttarg.rot.z; ikparams[5] = Ttarg.rot.w;
    ikparams[6] = Ttarg.trans.x; ikparams[7] = Ttarg.trans.y; ikparams[8] = Ttarg.trans.z;



    std::vector<dReal> q0, q_s;
    q_s.resize(robot->GetDOF());
    robot->GetDOFValues(q_s);

    //RAVELOG_INFO("TSRJointVals: %f %f\n",TSRJointVals[0],TSRJointVals[1]);

    for(int i=0; i < q_s.size();i++)
        q_s[i] = TSRJointVals[i];

    q0 = q_s;
    boost::shared_ptr<std::vector<dReal> > pq_s(new std::vector<dReal> );

    _pIkSolver->Solve(IkParameterization(), q0, ikparams, false, pq_s);
    q_s = *pq_s.get();

    robot->SetJointValues(q_s,true);
    for(int i=0; i < q_s.size();i++)
        TSRJointVals[i] = q_s[i];

    //RAVELOG_INFO("TSRJointVals: %f %f\n",TSRJointVals[0],TSRJointVals[1]);
    T0_closeset = robot->GetActiveManipulator()->GetEndEffectorTransform()*TSRChain[TSRChain.size()-1].Tw_e;
    return TransformDifference(T0_s,T0_closeset);
}

//NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
bool TaskSpaceRegionChain::MimicValuesToFullMimicBodyValues(const dReal* TSRJointVals, std::vector<dReal>& mimicbodyvals)
{
    if(_pmimicbody == NULL)
       return false;

    mimicbodyvals.resize(_pmimicbody->GetDOF());
    _pmimicbody->GetDOFValues(mimicbodyvals);
    for(int i = 0; i < _mimicinds.size(); i++)
    {
        mimicbodyvals[_mimicinds[i]] = _mimicjointoffsets[_mimicinds[i]] + TSRJointVals[i];
    }

    return true;
}


bool TaskSpaceRegionChain::ApplyMimicValuesToMimicBody(const dReal* TSRJointVals)
{
    if(_pmimicbody == NULL)
       return false;

    MimicValuesToFullMimicBodyValues(TSRJointVals,_mimicjointvals_temp);

    _pmimicbody->SetJointValues(_mimicjointvals_temp,true);
    return true;
}


bool TaskSpaceRegionChain::ExtractMimicDOFValues(dReal * TSRValues, dReal * MimicDOFVals)
{
    //NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
    for(int i = 0; i < _mimicinds.size(); i++)
        MimicDOFVals[i] = TSRValues[i];


    return true;
}


bool TaskSpaceRegionChain::serialize(std::ostream& O) const
{

    O << " ";

    O << (int)bSampleStartFromChain << " ";
    O << (int)bSampleGoalFromChain << " ";
    
    O << (int)bConstrainToChain << " ";


    O << TSRChain.size();
    for(int i = 0; i < TSRChain.size(); i++)
    {
        O << " ";
        if(!TSRChain[i].serialize(O))
        {
            RAVELOG_INFO("ERROR SERIALIZING TSR CHAIN\n");
            return false;
        }
        O << " ";
    }

    if(_pmimicbody.get() == NULL)
      O << " " << "NULL" << " ";
    else
      O << " " << _pmimicbody->GetName() << " ";
    

    O << " " << _mimicinds.size() << " ";
    for(int i = 0; i < _mimicinds.size(); i++)
    {
        O << " " << _mimicinds[i] << " ";
    }

    return true;
}
bool TaskSpaceRegionChain::deserialize(std::stringstream& _ss)
{

    _ss >> bSampleStartFromChain;
    _ss >> bSampleGoalFromChain;
    _ss >> bConstrainToChain;

    if(!bSampleGoalFromChain && !bSampleStartFromChain && !bConstrainToChain)
        RAVELOG_INFO("WARNING: This chain is not sampled or constrained to, are you sure you defined it correctly?\n");

    int temp;
    _ss >> temp;
    TSRChain.resize(temp);


    for(int i = 0; i < temp; i++)
    {
        if(!TSRChain[i].deserialize(_ss))
        {
            RAVELOG_INFO("ERROR DESERIALIZING TSR CHAIN\n");
            return false;
        }
    }

    _ss >> mimicbodyname;
    
    if( mimicbodyname != "NULL" )
    {
        _ss >> temp;
        _mimicinds.resize(temp);
        for(int i = 0; i < temp; i++)
        {   
           _ss >> _mimicinds[i];
        }
    }

    return true;
}

bool TaskSpaceRegionChain::deserialize_from_matlab(RobotBasePtr robot_in, EnvironmentBasePtr penv_in, std::istream& _ss)
{
    _ss >> bSampleStartFromChain;
    _ss >> bSampleGoalFromChain;
    _ss >> bConstrainToChain;

    if(!bSampleGoalFromChain && !bSampleStartFromChain && !bConstrainToChain)
        RAVELOG_INFO("WARNING: This chain is not sampled or constrained to, are you sure you defined it correctly?\n");


    int temp;
    string tempstring;

    _ss >> temp;
    
    TSRChain.resize(temp);
    
    for(int i = 0; i < temp; i++)
    {
        if(!TSRChain[i].deserialize_from_matlab(robot_in,penv_in,_ss))
        {
            RAVELOG_INFO("ERROR DESERIALIZING TSR CHAIN FROM MATLAB\n");
            return false;
        }
    }

    _ss >> tempstring;
    if( stricmp(tempstring.c_str(), "NULL") == 0 )
    {
        _pmimicbody.reset();
    }
    else
    {
        _pmimicbody = penv_in->GetRobot(tempstring.c_str());
        if(_pmimicbody.get() == NULL)
        {
            RAVELOG_INFO("Error: could not find the specified mimic body\n");
            return false;
        }

        _ss >> temp;
        _mimicinds.resize(temp);
        for(int i = 0; i < temp; i++)
        {   
           _ss >> _mimicinds[i];
        }
    }
    return true;
}

bool TaskSpaceRegionChain::Initialize(EnvironmentBasePtr penv_in)
{
    _sumbounds = 0;
    for(int i = 0; i < TSRChain.size(); i++)
    {
        if(!TSRChain[i].Initialize(penv_in))
        {
            RAVELOG_INFO("Error: Failed to initialize TSR %d\n",i);
            return false;
        }

        _sumbounds += TSRChain[i].GetSumOfBounds();
    }
    if(_sumbounds == 0)//for point TSRs
        _sumbounds = 0.001;

    if( stricmp(mimicbodyname.c_str(), "NULL") == 0 )
    {
        _pmimicbody.reset();
    }
    else
    {
        _pmimicbody = penv_in->GetRobot(mimicbodyname.c_str());
        if(_pmimicbody.get() == NULL)
        {
            RAVELOG_INFO("Error: could not find the specified kinbody to make a mimic\n");
            return false;
        }

    }
    return true;
}


bool TaskSpaceRegionChain::RobotizeTSRChain(EnvironmentBasePtr penv_in,RobotBasePtr& probot_out)
{
    bool bFlipAxis;
    if(penv_in.get() == NULL)
    {
        RAVELOG_INFO("Environment pointer is null!\n");
        probot_out = RobotBasePtr();
        return false;
    }

    _lowerlimits.resize(0);
    _upperlimits.resize(0);

    //store this pointer for later robot destruction
    penv = penv_in;

    char robotname[32], xmlfile[256], robottype[32];
    sprintf(robottype,"GenericRobot");
    sprintf(xmlfile,"TSRChain%lu.robot.xml",(unsigned long int)this);
    sprintf(robotname,"TSRChain%lu",(unsigned long int)this);//give a unique name to this robot


    robot = RaveCreateRobot(penv_in,robottype);
    if( robot.get() == NULL ) {
        RAVELOG_INFO("Failed to create robot %s", robottype);
        probot_out = RobotBasePtr();
        return false;
    }

    if(_pmimicbody != NULL)
    {    
        _mimicjointvals_temp.resize(_pmimicbody->GetDOF());
        //mimic body may not be starting at 0 position for all joints, so get the joint offsets
        _mimicjointoffsets.resize(_pmimicbody->GetDOF());
        _pmimicbody->GetDOFValues(_mimicjointoffsets);
    }


    //now write out the XML string for this robot

    stringstream O;
    
    O << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
    O << "<Robot name=\"" << robotname << "\">" << endl;
    O << "\t<KinBody>" << endl;

    O << "\t\t<Body name = \"Body0\" type=\"dynamic\">" << endl;
    O << "\t\t\t<Geom type=\"sphere\">" << endl;
    O << "\t\t\t\t<Radius>0.01</Radius>" << endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << endl;
    O << "\t\t\t</Geom>" << endl;
    O << "\t\t</Body>" << endl;

    int bodynumber = 1;
    Tw0_e = Transform();

    for(int i = 0; i < TSRChain.size(); i++)
    {
        for(int j = 0; j < 6; j++)
        {
            bFlipAxis = false;
            //don't add a body if there is no freedom in this dimension
            if(TSRChain[i].Bw[j][0] == 0 && TSRChain[i].Bw[j][1] == 0)
                continue;
          
            //TODO: If the bounds are equal and non-zero, do something reasonable
            if(TSRChain[i].Bw[j][0] == TSRChain[i].Bw[j][1])
            {
                RAVELOG_FATAL("ERROR: TSR Chains are currently unable to deal with cases where two bounds are equal but non-zero, cannot robotize.\n");
                probot_out = RobotBasePtr();
                return false;
            }

            //check for axis flip, this is marked by the Bw values being backwards
            if(TSRChain[i].Bw[j][0] > TSRChain[i].Bw[j][1])
            {
                TSRChain[i].Bw[j][0] = -TSRChain[i].Bw[j][0];
                TSRChain[i].Bw[j][1] = -TSRChain[i].Bw[j][1];
                bFlipAxis = true;
            }

            //now take care of joint offsets if we are mimicing
            if(_pmimicbody.get() != NULL)
            {   
                //we may only be mimicing some of the joints of the TSR
                if(bodynumber-1 <  _mimicinds.size())
                {
                    TSRChain[i].Bw[j][0] = TSRChain[i].Bw[j][0] - _mimicjointoffsets[_mimicinds[bodynumber-1]];
                    TSRChain[i].Bw[j][1] = TSRChain[i].Bw[j][1] - _mimicjointoffsets[_mimicinds[bodynumber-1]];
                }
            }    

            _lowerlimits.push_back(TSRChain[i].Bw[j][0]);
            _upperlimits.push_back(TSRChain[i].Bw[j][1]);

            O << "\t\t<Body name = \"Body" << bodynumber << "\" type=\"dynamic\">" << endl;
            O << "\t\t\t<offsetfrom>Body0</offsetfrom>" << endl;
            O << "\t\t\t<Translation>" << Tw0_e.trans.x << " "<< Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << endl;
            O << "\t\t\t<Quat>" << Tw0_e.rot.x << " "<< Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << endl;

            if(j < 3)
                O << "\t\t\t<Geom type=\"box\">" << endl;
            else
                O << "\t\t\t<Geom type=\"cylinder\">" << endl;        


            switch(j)
            {
            case 0:
                O << "\t\t\t\t<extents>0.04 0.02 0.02</extents>" <<endl;
                break;
            case 1:
                O << "\t\t\t\t<extents>0.02 0.04 0.02</extents>" <<endl;
                break;
            case 2:
                O << "\t\t\t\t<extents>0.02 0.02 0.04</extents>" <<endl;
                break;
            case 3:
                O << "\t\t\t\t<RotationAxis>0 0 1 90</RotationAxis>" << endl;
                O << "\t\t\t\t<Radius>0.03</Radius>" << endl;
                O << "\t\t\t\t<Height>0.07</Height>" << endl;
                break;
            case 4:
                O << "\t\t\t\t<Radius>0.03</Radius>" << endl;
                O << "\t\t\t\t<Height>0.07</Height>" << endl;
                break;
            case 5:
                O << "\t\t\t\t<RotationAxis>1 0 0 90</RotationAxis>" << endl;
                O << "\t\t\t\t<Radius>0.03</Radius>" << endl;
                O << "\t\t\t\t<Height>0.07</Height>" << endl;
                break;
            }

            O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << endl;
            O << "\t\t\t</Geom>" << endl;

            O << "\t\t</Body>" << endl;

            if(j < 3)
                O << "\t\t<Joint name=\"J"<< bodynumber << "\" type=\"slider\">" << endl;
            else
                O << "\t\t<Joint name=\"J"<< bodynumber << "\" type=\"hinge\">" << endl;

            O << "\t\t\t<Body>Body"<<  bodynumber-1 << "</Body>" << endl;
            O << "\t\t\t<Body>Body"<<  bodynumber << "</Body>" << endl;
            O << "\t\t\t<offsetfrom>Body"<<  bodynumber << "</offsetfrom>" << endl;
            O << "\t\t\t<weight>1</weight>" << endl;
            O << "\t\t\t<maxvel>1</maxvel>" << endl;
            O << "\t\t\t<resolution>1</resolution>" << endl;

            O << "\t\t\t<limits>" << TSRChain[i].Bw[j][0] << " " << TSRChain[i].Bw[j][1]<< "</limits>" << endl;

            switch(j)
            {
            case 0:
                O << "\t\t\t<axis>1 0 0</axis>" <<endl;
                break;
            case 1:
                O << "\t\t\t<axis>0 1 0</axis>" <<endl;
                break;
            case 2:
                O << "\t\t\t<axis>0 0 1</axis>" <<endl;
                break;
            case 3:
                if(bFlipAxis)
                    O << "\t\t\t<axis>-1 0 0</axis>" <<endl;
                else
                    O << "\t\t\t<axis>1 0 0</axis>" <<endl;
                break;
            case 4:
                if(bFlipAxis)
                    O << "\t\t\t<axis>0 -1 0</axis>" <<endl;
                else
                    O << "\t\t\t<axis>0 1 0</axis>" <<endl;
                break;
            case 5:
                if(bFlipAxis)
                    O << "\t\t\t<axis>0 0 -1</axis>" <<endl;
                else
                    O << "\t\t\t<axis>0 0 1</axis>" <<endl;
                break;
            }

            O << "\t\t</Joint>" << endl;

            bodynumber++;
        }
        Tw0_e = Tw0_e*TSRChain[i].Tw_e;
    }


    //now add a geometry to the last body with the offset of the last TSR, this will be the target for the manipulator
    //NOTE: this is appending a body, not making a new one
    Transform Told = Tw0_e;
    Tw0_e = TSRChain[TSRChain.size() - 1].Tw_e;

    O << "\t\t<Body name = \"Body" << bodynumber-1 << "\" type=\"dynamic\">" << endl;
    O << "\t\t\t<Geom type=\"sphere\">" << endl;
    O << "\t\t\t\t<Translation>" << Tw0_e.trans.x << " "<< Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << endl;
    O << "\t\t\t\t<Quat>" << Tw0_e.rot.x << " "<< Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << endl;
    O << "\t\t\t\t<Radius>0.03</Radius>" << endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << endl;
    O << "\t\t\t</Geom>" << endl;
    O << "\t\t</Body>" << endl;


    O << "\t</KinBody>" << endl;


    if(bodynumber > 1)
    {
        _bPointTSR = false;
        //finally, write out the manipulator parameters
        O << "\t<Manipulator name=\"dummy\">" << endl;
        O << "\t\t<base>Body0</base>" << endl;
        O << "\t\t<effector>Body" << bodynumber-1 << "</effector>" << endl;
        O << "\t</Manipulator>" << endl;
    }
    else
    {
        _bPointTSR = true;
        numdof = bodynumber-1;
        RAVELOG_INFO("This is a point TSR, no robotized TSR needed\n");
        probot_out = RobotBasePtr();
        return true;
    }

    if(_bPointTSR && TSRChain.size() != 1)
    {
        RAVELOG_INFO("Can't yet handle case where the tsr chain has no freedom but multiple TSRs, try making it a chain of length 1\n");
        probot_out = RobotBasePtr();
        return false;
    }


    O << "</Robot>" << endl;

    robot = penv->ReadRobotXMLData(RobotBasePtr(), O.str(), std::list<std::pair<std::string,std::string> >());
    
    if(robot.get() == NULL)
    {
        RAVELOG_INFO("Could not init robot from data!\n");
        probot_out = RobotBasePtr();
        return false;
    }
    
    if(robot.get() == NULL )
    {
        RAVELOG_INFO("Robot is NULL!\n");
        probot_out = RobotBasePtr();
        return false;
    }

    robot->SetName(robotname);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    penv_in->Add(robot,true);
#else
    penv_in->AddRobot(robot,true);
#endif

    if(TSRChain[0].prelativetolink.get() == NULL)
        robot->SetTransform(TSRChain[0].T0_w);
    else
        robot->SetTransform(TSRChain[0].prelativetolink->GetTransform()*TSRChain[0].T0_w);
   

    _pIkSolver = RaveCreateIkSolver(penv_in,"GeneralIK");

    if(_pIkSolver.get() == NULL)
    {
        RAVELOG_INFO("Cannot create IK solver, make sure you have the GeneralIK plugin loadable by openrave\n");
        probot_out = RobotBasePtr();
        return false;
    }
    
    RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    _pIkSolver->Init(pmanip);
    numdof = bodynumber-1;
  
    //initialize parameters to send to ik solver
    ikparams.resize(12);
    ikparams[0] = 1;
    ikparams[1] = 0;
    ikparams[9] = 0; //don't do any balancing
    ikparams[10] = 0; //select the mode
    ikparams[11] = 0; //do rotation

    //note that openrave will creat traj files for each TSR in your .openrave directory, you may want to remove them

    robot->Enable(false);

    probot_out = robot;
    return true;
}

dReal TaskSpaceRegionChain::TransformDifference(const Transform& tm_ref, const Transform& tm_targ)
{
    _tmtemp = tm_ref.inverse()*tm_targ;

    _dx[0] = _tmtemp.trans.x;
    _dx[1] = _tmtemp.trans.y;
    _dx[2] = _tmtemp.trans.z;

    TSRChain[0].QuatToRPY(&_tmtemp.rot[0],_dx[3],_dx[4],_dx[5]);

    _sumsqr = 0;
    for(int i = 0; i < 6; i++)
        _sumsqr += _dx[i]*_dx[i];
    //RAVELOG_INFO("dx: %f %f %f %f %f %F\n",dx[0],dx[1],dx[2],dx[3],dx[4],dx[5]);

    return sqrt(_sumsqr);
}


void TaskSpaceRegionChain::DestoryRobotizedTSRChain()
{
   if(robot != NULL) 
   {
       penv->Remove(robot);
   }
}
