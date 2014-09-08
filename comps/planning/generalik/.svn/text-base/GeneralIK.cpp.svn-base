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
/** \file GeneralIK.cpp
    \brief Implements the generalik class.
 */
#include "stdafx.h"
#include "GeneralIK.h"


const int INF = 1000000;


GeneralIK::GeneralIK(EnvironmentBasePtr penv) : IkSolverBase(penv)
{
    __description = ":Interface Author: Dmitry Berenson\nAn iterative IK solver for kinematic chains. Can also take into account balance constraints. \n\n`C++ Documentation <http://automation.berkeley.edu/~berenson/docs/generalik/index.html>`_";
}


bool GeneralIK::Init(const RobotBase::ManipulatorPtr pmanip)
{
    _pmanip = pmanip;
    _pRobot = _pmanip->GetRobot();


    bPRINT = false;
    bDRAW = false;
    bWRITETRAJ = false;
    bQUAT = false;
    bTRANSLATION_ONLY = false;


    RAVELOG_INFO("Initializing GeneralIK Solver\n");

    

    _oldnumdofs = -1;
    _numdofs = -1;
    _oldnumtargdims = -1;
    _numtargdims = -1;

    _tasktm.ReSize(3,3);
    _E_rpy.ReSize(3,3);
    _E_rpy_inv.ReSize(3,3);

    balancedx.ReSize(2);
    Mbal.ReSize(2);
    Mbalinv.ReSize(2);

    Mbalperp.ReSize(2);
    Mbalperpinv.ReSize(2);

    curquat.ReSize(4);
    angveltoquat.ReSize(4,4);

    bBalance = false;

    return true;
}


void GeneralIK::ResizeMatrices()
{
    J.ReSize(_numtargdims,_numdofs);

    Jtrans.ReSize(_numtransdims,_numdofs);

    Jtemp.ReSize(_dimspergoal,_numdofs);
    Jplus.ReSize(_numdofs,_numtargdims);
    dx.ReSize(_numtargdims);
    dx_trans.ReSize(_numtransdims);
    step.ReSize(_numdofs);
    nullspacestep.ReSize(_numdofs);

    M.ReSize(_numtargdims);
    Minv.ReSize(_numtargdims);

    q_s_old.resize(_numdofs);
    _curtms.resize(_targtms.size());

    _Jp.ReSize(3,_numdofs);
    _Jp0.ReSize(3,_numdofs);
    _Jr.ReSize(3,_numdofs);
    _Jr0.ReSize(3,_numdofs);
    _Jr_proper.ReSize(3,_numdofs);
    q_limits.ReSize(_numdofs);

    Jtemp2.ReSize(2,_numdofs);

    _Jr_quat.ReSize(4,_numdofs);

    _S.ReSize(_numtargdims);
    _V.ReSize(_numtargdims,_numtargdims);
}


bool GeneralIK::Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
{

    int activemanipind = _pRobot->GetActiveManipulatorIndex();

    dReal* qResult = &(*result.get())[0];
    const dReal* pFreeParameters = &vFreeParameters[0];
    solutionpath.resize(0);
    _targmanips.resize(0);
    _targtms.resize(0);
    movementlimit = INF;
    
    bool bsuccess = true;
    _pRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);
    
    int numdofs = _pRobot->GetActiveDOF();


    std::vector<dReal> q_s(numdofs);

    for(int i = 0; i < numdofs; i++)
        q_s[i] = q0[i];    



    //read in the ik targets

    int numtargets = (int) pFreeParameters[0];
    
    for(int i = 0; i < numtargets; i++)
    {
        _targmanips.push_back(pFreeParameters[i*8 + 1]);
        _targtms.push_back(Transform(Vector(pFreeParameters[i*8 + 2],pFreeParameters[i*8 + 3],pFreeParameters[i*8 + 4],pFreeParameters[i*8 + 5]),Vector(pFreeParameters[i*8 + 6],pFreeParameters[i*8 + 7],pFreeParameters[i*8 + 8]) ));
        if(bPRINT)       
             RAVELOG_INFO("Targtm: %f %f %f %f    %f %f %f\n",_targtms[i].rot.x,_targtms[i].rot.y,_targtms[i].rot.z,_targtms[i].rot.w,_targtms[i].trans.x,_targtms[i].trans.y,_targtms[i].trans.z);
    }

    int offset = numtargets*8 + 1;

    //read in cog target if there is one
    if((bool)pFreeParameters[offset++])
    {
        cogtarg.x = pFreeParameters[offset++];
        cogtarg.y = pFreeParameters[offset++];
        cogtarg.z = pFreeParameters[offset++];
    
        if(bPRINT)
            RAVELOG_INFO("cog target: %f %f %f\n",cogtarg.x,cogtarg.y,cogtarg.z);    

        //read in the support polygon
        int numpoints = pFreeParameters[offset++];
        //int offset = numtargets*8 + 6;

        _vsupportpolyx.resize(numpoints);
        _vsupportpolyy.resize(numpoints);
        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyx[i] = pFreeParameters[offset++];
        }

        offset = numtargets*8 + 6 + numpoints;

        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyy[i] = pFreeParameters[offset++];
        }

        for(int i =0; i < _vsupportpolyy.size(); i++)
        {
            if(bPRINT)
                RAVELOG_INFO("x: %f    y:%f\n",_vsupportpolyx[i],_vsupportpolyy[i]);
        }

        bBalance = true;
    }
    else
    {
        bBalance = false;
    }

    //this used to be a mode but is currently ignored
    int junk = (int)pFreeParameters[offset++]; //mode = (GeneralIK::Mode)pFreeParameters[offset++];
    bTRANSLATION_ONLY = (bool)pFreeParameters[offset++];
    
    if(vFreeParameters.size()-1 == offset)
    {
        movementlimit = pFreeParameters[offset];
    }


    if(bTRANSLATION_ONLY)
    {
        _dimspergoal = 3;
        _numrotdims = 0;
    }
    else
    {
        if(bQUAT)
        {
            _dimspergoal = 7;
            _numrotdims = _targtms.size()*4;
        }
        else
        {
            _dimspergoal = 6;
            _numrotdims = _targtms.size()*3;
        }
    }

    _numtransdims = _targtms.size()*3;
    _numtargdims = _targtms.size()*_dimspergoal;
    //RAVELOG_INFO("extratarg = %f\n",pFreeParameters[numtargets*8 + 1]);


    _numdofs = numdofs;
    

    if( (_oldnumdofs != _numdofs) || (_oldnumtargdims != _numtargdims) )
    {
        ResizeMatrices();
        _oldnumdofs = _numdofs;
        _oldnumtargdims = _numtargdims;
    }

    if(bWRITETRAJ)
    {
        ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_pRobot->GetActiveConfigurationSpecification());
        //_trajpoint.q.resize(_pRobot->GetActiveDOF());
        //_trajpoint.qdot.resize(_pRobot->GetActiveDOF());
    }

    if(!CheckDOFValueIntegrity(q_s))
    {
        RAVELOG_INFO("ERROR: DOF values are incorrect, cannot start IK solver!\n");
        return false;
    }


    bsuccess  = _SolveStopAtLimits(q_s);

    
    //always copy the joint vals into q_s (this could be the closest the robot could get)
    *result.get() = q_s;

    if(bDRAW)
        DrawSolutionPath();

    if(bPRINT)
        RAVELOG_INFO("Number of iterations: %d x_error: %f\n",_numitr,x_error);

    if(bWRITETRAJ)
        WriteTraj();

    _pRobot->SetActiveManipulator(activemanipind);
    return (bsuccess);

}





dReal GeneralIK::TransformDifferenceVectorized(dReal * dx,std::vector<Transform>& tm_refs, std::vector<Transform>& tm_targs)
{


    for(int i = 0; i < tm_refs.size(); i++)
    {
        TransformDifference(dx + i*_dimspergoal,tm_refs[i],tm_targs[i]);
    }

    _sumsqr2 = 0;
    for(int i = 0; i < tm_refs.size()*_dimspergoal; i++)
        _sumsqr2 += dx[i]*dx[i];

    return sqrt(_sumsqr2);
}


dReal GeneralIK::TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ)
{
    _tmtemp = tm_ref.inverse()*tm_targ;

    dx[0] = _tmtemp.trans.x;
    dx[1] = _tmtemp.trans.y;
    dx[2] = _tmtemp.trans.z;

    if(bTRANSLATION_ONLY)
    {
        //do nothing
    }
    else
    {
        if(bQUAT)
        {
            //check which version of quaternion is closer
            _q1 = tm_targ.rot - tm_ref.rot;
            _q2 = -tm_targ.rot - tm_ref.rot;
            if(_q1.lengthsqr4() < _q2.lengthsqr4())
            {
                dx[3] = _q1.x;
                dx[4] = _q1.y;
                dx[5] = _q1.z;
                dx[6] = _q1.w;
            }
            else
            {
                dx[3] = _q2.x;
                dx[4] = _q2.y;
                dx[5] = _q2.z;
                dx[6] = _q2.w;
            }
        }
        else
            QuatToRPY(_tmtemp,dx[3],dx[4],dx[5]);
    }

    _sumsqr = 0;
    for(int i = 0; i < _dimspergoal; i++)
        _sumsqr += dx[i]*dx[i];

    //RAVELOG_INFO("dx: %f %f %f %f %f %F\n",dx[0],dx[1],dx[2],dx[3],dx[4],dx[5]);

    return sqrt(_sumsqr);
}


void GeneralIK::GetFullJacobian(Transform tEE, Transform taskframe_in, NEWMAT::Matrix& J)
{

    std::vector<dReal> temp;

    _pRobot->CalculateActiveJacobian(_pRobot->GetActiveManipulator()->GetEndEffector()->GetIndex(), tEE.trans, temp);
    //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");
    memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));

    _pRobot->CalculateActiveAngularVelocityJacobian(_pRobot->GetActiveManipulator()->GetEndEffector()->GetIndex(), temp);
    memcpy(_Jr0.Store(),&temp[0],temp.size()*sizeof(dReal));
    
    //PrintMatrix(_Jr0.Store(),3,_numdofs,"Jr0: ");

    
    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

    _Jp = _tasktm * _Jp0;

    if(bTRANSLATION_ONLY)
    {
        J = _Jp;
        return;
    }

    _Jr = _tasktm * (-_Jr0);

    //PrintMatrix(_tasktm.Store(),3,3,"tasktm: ");
    //PrintMatrix(_Jp.Store(),3,_numdofs,"Jp: ");
    //PrintMatrix(_Jr.Store(),3,_numdofs,"Jr: ");


    if(bQUAT)
    {
        Vector vcurquat = (taskframe_in.inverse()*tEE).rot;
        NEWMAT::ColumnVector curquat(4);
        curquat(1) = vcurquat.x;
        curquat(2) = vcurquat.y;
        curquat(3) = vcurquat.z;
        curquat(4) = vcurquat.w;


        NEWMAT::Matrix angveltoquat(4,4);
        
        for(int i =1; i <= _numdofs; i++)
        {   
            //get correction matrix using angular velocity from Jr
            angveltoquat(1,1) = 0;          angveltoquat(1,2) = -_Jr(1,i);    angveltoquat(1,3) = -_Jr(2,i);  angveltoquat(1,4) = -_Jr(3,i);
            angveltoquat(2,1) = _Jr(1,i);   angveltoquat(2,2) = 0;            angveltoquat(2,3) = -_Jr(3,i);  angveltoquat(2,4) = _Jr(2,i);
            angveltoquat(3,1) = _Jr(2,i);   angveltoquat(3,2) = _Jr(3,i);     angveltoquat(3,3) = 0;          angveltoquat(3,4) = -_Jr(1,i);
            angveltoquat(4,1) = _Jr(3,i);   angveltoquat(4,2) = -_Jr(2,i);    angveltoquat(4,3) = _Jr(1,i);   angveltoquat(4,4) = 0;

            _Jr_quat.Column(i) = 0.5 * (angveltoquat * curquat);
        }

        //PrintMatrix(Jr_proper.Store(),3,_numdofs,"Jr_proper: ");

        J = _Jp & -_Jr_quat;
    }
    else
    {

        //convert current rotation to euler angles (RPY) 
        QuatToRPY(taskframe_in.inverse()*tEE,_psi,_theta,_phi);    
        //RAVELOG_INFO("psi:  %f  theta:  %f   phi:  %f\n",psi,theta,phi);

        Cphi = cos(_phi);
        Ctheta = cos(_theta);
        Cpsi = cos(_psi);

        Sphi = sin(_phi);
        Stheta = sin(_theta);
        Spsi = sin(_psi);

        _E_rpy(1,1) = Cphi/Ctheta;         _E_rpy(1,2) = Sphi/Ctheta;         _E_rpy(1,3) = 0;
        _E_rpy(2,1) = -Sphi;               _E_rpy(2,2) = Cphi;                _E_rpy(2,3) = 0;
        _E_rpy(3,1) = Cphi*Stheta/Ctheta;  _E_rpy(3,2) = Sphi*Stheta/Ctheta;  _E_rpy(3,3) = 1;


        //PrintMatrix(E_rpy.Store(),3,3,"E_rpy: ");

        _Jr_proper = _E_rpy * _Jr;

        //PrintMatrix(Jr_proper.Store(),3,_numdofs,"Jr_proper: ");

        J = _Jp & -_Jr_proper;

        //PrintMatrix(J.Store(),3,_numdofs,"J1: ");
    }

}

void GeneralIK::GetCOGJacobian(Transform taskframe_in, NEWMAT::Matrix& J, Vector& center)
{


    J = 0.0;

    center = Vector(0,0,0,0);
    dReal fTotalMass = 0;
    std::vector<KinBody::LinkPtr>::const_iterator itlink;



    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];


    //PrintMatrix(E_rpy.Store(),3,3,"E_rpy: ");

    std::vector<dReal> temp;
    FORIT(itlink, _pRobot->GetLinks()) {
        //RAVELOG_INFO("comoffset: %f %f %f\n", (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
        //GetEnv()->plot3((*itlink)->GetTransform() * (*itlink)->GetCOMOffset(), 1, 0, 0.01*(*itlink)->GetMass(), Vector(1,0,0),1);

        _pRobot->CalculateActiveJacobian((*itlink)->GetIndex(), ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset()), temp);
        //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");
        memcpy(_Jp0.Store(),&temp[0],temp.size()*sizeof(dReal));



        _Jp = _tasktm * _Jp0;

        //RAVELOG_INFO("Link name: %s, mass: %f\n",(*itlink)->GetName().c_str(),(*itlink)->GetMass());
        //PrintMatrix(_tasktm.Store(),3,3,"tasktm: ");
        //PrintMatrix(_Jp.Store(),3,_numdofs,"Jp: ");
        //PrintMatrix(_Jr.Store(),3,_numdofs,"Jr: ");

        J = J + ((*itlink)->GetMass()*(_Jp.Rows(1,2)));


        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 )
        center /= fTotalMass;
    //RAVELOG_INFO("\nmass: %f\ncog: %f %f %f\n",fTotalMass,center.x,center.y,center.z);
   
    J = J/fTotalMass;

}



void GeneralIK::PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement)
{
    //return;

    stringstream s;
    s.setf(ios::fixed,ios::floatfield);
    s.precision(5);
    s << "\n"; 
    if(statement != NULL)
        s << statement <<"\n";
    for(int i = 0; i < numrows; i++)
    {
        for(int j = 0; j < numcols; j++)
        {
            s << pMatrix[i*numcols + j] << " \t";
            if(fabs(pMatrix[i*numcols + j]) > 10000)
                RAVELOG_INFO("WARNING: %d %d is huge!!!\n",i,j);
        }
        s << endl;
    }
    RAVELOG_INFO(s.str().c_str());

}


Vector GeneralIK::RPYIdentityOffsets[8] = { Vector(M_PI,M_PI,M_PI),
                                            Vector(M_PI,M_PI,-M_PI),
                                            Vector(M_PI,-M_PI,M_PI),
                                            Vector(M_PI,-M_PI,-M_PI),
                                            Vector(-M_PI,M_PI,M_PI),
                                            Vector(-M_PI,M_PI,-M_PI),
                                            Vector(-M_PI,-M_PI,M_PI),
                                            Vector(-M_PI,-M_PI,-M_PI)};

void GeneralIK::QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi)
{
    
    a = tm.rot.x;
    b = tm.rot.y;
    c = tm.rot.z;
    d = tm.rot.w;

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


bool GeneralIK::CheckSupport(Vector center)
{
    //return true;
    int nvert = _vsupportpolyx.size();
    if(nvert == 0)
        return false;


    dReal testx = center.x;
    dReal testy = center.y;
    dReal * vertx = &_vsupportpolyx[0];
    dReal * verty = &_vsupportpolyy[0];
    

    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
    }
   
    center.z = 0;
    solutionpath.push_back(center);
    //RAVELOG_INFO("cog: %f %f %f\n", center.x,center.y,center.z);
    if(c)    
    {
        if(bDRAW)
            graphptrs.push_back(GetEnv()->plot3(&(DoubleVectorToFloatVector(center)[0]), 1, 0, 10, Vector(0,1,0) ));
        return true;
    }
    else
    {
        if(bDRAW)
            graphptrs.push_back(GetEnv()->plot3(&(DoubleVectorToFloatVector(center)[0]), 1, 0, 10, Vector(1,0,0) ));
        return false;
    }
}


int GeneralIK::invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{
    int didfix = 0;
	NEWMAT::EigenValues(A,_S,_V);
	// Find the maximum eigenvalue
	dReal maxEig = 0;
    dReal minEig = 0;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e > maxEig) maxEig = e;
        if (i == 1 || e < minEig) minEig = e;
	}
    //RAVELOG_INFO("min/max eigenvalue: %f/%f\n", minEig, maxEig);

	dReal minEigDesired = maxEig/maxConditionNumber;
    int notfixcount = 0;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e < minEigDesired) {e = minEigDesired; didfix = 1; if(bPRINT) RAVELOG_INFO("MIN EIG COND FIX!\n");}
        else
            notfixcount++;
            
        if (maxEig > 100) { e = e/maxEig*100; if(bPRINT) RAVELOG_INFO("MAX EIG COND FIX!\n");}
        _S(i) = e;
	}
    if(bPRINT) RAVELOG_INFO("notfixcount: %d\n",notfixcount);
    //this just reconstructs the A matrix with better conditioning
	//Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();
    return didfix;
}

void GeneralIK::invConditioning(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{

	NEWMAT::EigenValues(A,_S,_V);
	// Find the maximum eigenvalue
	dReal maxEig = 0;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e > maxEig) maxEig = e;
	}
    //RAVELOG_INFO("max eigenvalue: %f\n", maxEig);

	dReal minEigDesired = maxEig/maxConditionNumber;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e < minEigDesired) e = minEigDesired;
        //if (maxEig > 100) e = e/maxEig*100;
        _S(i) = e;
	}

    //this just reconstructs the A matrix with better conditioning
	//Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();

}

void GeneralIK::DrawSolutionPath()
{

    int pathlength = solutionpath.size();

    if(pathlength == 0)
        return;

    //float solutionpath0_x_float = (float)solutionpath[0].x;
    //graphptrs.push_back(GetEnv()->drawlinestrip(&solutionpath0_x_float,pathlength,sizeof(solutionpath[0]),3, RaveVector<float>(0, 0, 1, 0)));
    //graphptrs.push_back(GetEnv()->drawlinestrip(&solutionpath0_x_float,pathlength,sizeof(RaveVector<float>(0, 0, 1, 0)),3, RaveVector<float>(0, 0, 1, 0)));
    std::vector<RaveVector<float> > fSolutionPath;
    for(int i =0; i < solutionpath.size(); i++)
        fSolutionPath.push_back(RaveVector<float>(solutionpath[i].x,solutionpath[i].y,solutionpath[i].z));

    RaveVector<float> vblue = RaveVector<float>(0, 0, 1, 0);
    graphptrs.push_back(GetEnv()->drawlinestrip(&(fSolutionPath[0].x),pathlength,sizeof(vblue),3, vblue));

    graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(solutionpath[pathlength-1])[0]), 1, 0, 15, Vector(1,1,0) ));
    RAVELOG_INFO("cog at end: %f %f %f\n",solutionpath[pathlength-1].x,solutionpath[pathlength-1].y,solutionpath[pathlength-1].z);
}



void GeneralIK::GetClosestPointOnPolygon(Vector& point,Vector& closestpoint, Vector& perpvec)
{
    perpvec.z = 0;
    dReal xout,yout,dist;
    dReal mindist = 10000000;
    dReal endpntx,endpnty;

    for(int i = 0; i < _vsupportpolyx.size(); i++)
    {

        if( i == _vsupportpolyx.size()-1)
        {
            endpntx = _vsupportpolyx[0];
            endpnty = _vsupportpolyy[0];
        }
        else
        {
            endpntx = _vsupportpolyx[i+1];
            endpnty = _vsupportpolyy[i+1];
        }

        GetDistanceFromLineSegment(point.x, point.y, _vsupportpolyx[i], _vsupportpolyy[i],endpntx, endpnty, dist, xout, yout);

        if(dist < mindist)
        {
            mindist = dist;
            closestpoint.x = xout;
            closestpoint.y = yout;
            //get vector perpendicular to the edge
            perpvec.x = -(_vsupportpolyy[i] - endpnty);
            perpvec.y = _vsupportpolyx[i] - endpntx;

        }
    }

    perpvec = perpvec*(1/(sqrt(perpvec.x*perpvec.x + perpvec.y*perpvec.y)));

}


void GeneralIK::GetDistanceFromLineSegment(dReal cx, dReal cy, dReal ax, dReal ay ,
					  dReal bx, dReal by, dReal& distanceSegment,
					  dReal& xout, dReal& yout)
{
    dReal distanceLine;

	dReal r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
	dReal r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
	dReal r = r_numerator / r_denomenator;

    dReal px = ax + r*(bx-ax);
    dReal py = ay + r*(by-ay);

    dReal s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;

	distanceLine = fabs(s)*sqrt(r_denomenator);

	dReal xx = px;
	dReal yy = py;

	if ( (r >= 0) && (r <= 1) )
	{
		distanceSegment = distanceLine;
	}
	else
	{

		dReal dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
		dReal dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
		if (dist1 < dist2)
		{
			xx = ax;
			yy = ay;
			distanceSegment = sqrt(dist1);
		}
		else
		{
			xx = bx;
			yy = by;
			distanceSegment = sqrt(dist2);
		}


	}
    xout = xx;
    yout = yy;
}

void GeneralIK::WriteTraj()
{
  
    //TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),"");
    //_pRobot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    /* new openrave added a fmaxaccelmult parameter (number 5) */
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, _pRobot,false,1,1,"LinearTrajectoryRetimer");
#else
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(ptraj, _pRobot,false,1,"LinearTrajectoryRetimer");
#endif
    // save the constrained trajectory
    const char* filename2 = "iktraj.txt";
    //pfulltraj->CalcTrajTiming(_pRobot, pfulltraj->GetInterpMethod(), true, false);
    ofstream outfile(filename2,ios::out);
    ptraj->serialize(outfile);
    //pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    outfile.close();
}


bool GeneralIK::CheckDOFValueIntegrity(const std::vector<dReal>& q_in)
{
    bool bok = true;
    for(int i = 0; i < _numdofs; i++)
    {
        if(q_in[i] < _lowerLimit[i] || q_in[i] > _upperLimit[i])
        {
            RAVELOG_INFO("ERROR: DOF %d has value %f, which is outside %f to %f\n",i,q_in[i],_lowerLimit[i],_upperLimit[i]);
            bok = false;
        }

        if(isnan(q_in[i]))
        {
            RAVELOG_INFO("ERROR: DOF %d is nan!\n",i);
            bok = false;
        }  
    }    
    return bok;
}


bool GeneralIK::_SolveStopAtLimits(std::vector<dReal>& q_s)
{

    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;
    badjointinds.resize(0);
    prev_error = 1000000.0;
    std::vector<dReal> q_s_backup = q_s;
    //initialize stepsize and epsilon
    maxstep = 0.1*_targtms.size();
    stepsize = maxstep;
    epsilon = 0.001;
    q_s_old = q_s;
    bClearBadJoints = true; //setting this to true will make the algorithm attempt to move joints that are at joint limits at every iteration


    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(_targtms[i].trans)[0]), 1, 0, 5, Vector(0,1,0) ));
        }
    }

    bBalanceGradient = false;
    Vector perpvec;

    for(int kk = 0; kk < 200; kk++)
    {
        _numitr++;

        bLimit = false;

        _pRobot->SetActiveDOFValues(q_s);
        if(bWRITETRAJ)
        {
            for(int i = 0; i < _pRobot->GetActiveDOF(); i++)
            {
                //_trajpoint.q[i] = q_s[i];
                ptraj->Insert(ptraj->GetNumWaypoints(),q_s,_pRobot->GetActiveConfigurationSpecification());
            }
            //ptraj->AddPoint(_trajpoint);
        }

        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                graphptrs.push_back(GetEnv()->plot3( &(DoubleVectorToFloatVector(_curtms[i].trans)[0]), 1, 0, 5, Vector(1,0,0) ));
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");


        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);


        //if balance stuff is on, error will go up sometimes, so don't do this check

        if(!bBalance && (x_error >= prev_error || (prev_error - x_error < epsilon/10))&& x_error > epsilon)
        {
            if(bPRINT)
            {
                RAVELOG_INFO("no progress\n");
                RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);
            }
            stepsize = stepsize/2;
            x_error = prev_error;
            q_s = q_s_old;

        }
        else
            stepsize = maxstep;


        if(bPRINT)
            RAVELOG_INFO("stepsize: %f\n",stepsize);

        //don't let step size get too small

        if(stepsize < epsilon)
        {
            //if(bPRINT)
            //    RAVELOG_INFO("Projection stalled _numitr: %d\n",_numitr);
            RAVELOG_DEBUG("Projection stalled _numitr: %d\n",_numitr);
            return false;
        }

        if(movementlimit != INF)
        {
            dReal qsnorm = 0;
            for(int i = 0; i < q_s.size(); i++)
            {
                qsnorm += (q_s[i] - q_s_backup[i])*(q_s[i] - q_s_backup[i]);
            }
            qsnorm = sqrt(qsnorm);
            if(qsnorm > movementlimit)
            {
                q_s = q_s_old;
                RAVELOG_DEBUG("Projection hit movement limit at itr %d\n",_numitr);
                return false;
            }
        }

        if(bClearBadJoints)
        {
            badjointinds.resize(0);
        }
        do{
            q_s_old = q_s;

            if(bLimit == false)
                prev_error = x_error;

            if(bBalance)
            {
               GetCOGJacobian(Transform(),Jtemp2,curcog);
            }

            //RAVELOG_INFO("xerror: %f\n",x_error);
            if(x_error < epsilon && (!bBalance || CheckSupport(curcog)))
            {
                //if(bPRINT)
                //    RAVELOG_INFO("Projection successfull _numitr: %d\n",_numitr);
                RAVELOG_DEBUG("Projection successfull _numitr: %d\n",_numitr);
                return true;
            }

            //only need to compute the jacobian once if there are joint limit problems
            if(bLimit == false)
            {
                for(int i = 0; i < _targmanips.size();i++)
                {
                    _pRobot->SetActiveManipulator(_targmanips[i]);
                    GetFullJacobian(_curtms[i],_targtms[i],Jtemp);
                    J.Rows(i*_dimspergoal +1,(i+1)*_dimspergoal) = Jtemp;

                }


                if(bBalance)
                {
                   //the cog jacobian should only be 2 dimensional, b/c we don't care about z
                   GetCOGJacobian(Transform(),Jtemp2,curcog);

                   if(!CheckSupport(curcog))
                   {
                        bBalanceGradient = true;
                        balancedx(1) = (curcog.x - cogtarg.x);
                        balancedx(2) = (curcog.y - cogtarg.y);
                   }
                   else
                   {
                        balancedx(1) = 0;
                        balancedx(2) = 0;
                        bBalanceGradient = false;
                   }
                }


            }
            //eliminate bad joint columns from the Jacobian
            for(int j = 0; j < badjointinds.size(); j++)
                for(int k = 0; k < _numtargdims; k++)
                      J(k+1,badjointinds[j]+1) = 0;

            if(bBalance)
            {
                for(int j = 0; j < badjointinds.size(); j++)
                    for(int k = 0; k <2; k++)
                          Jtemp2(k+1,badjointinds[j]+1) = 0;
            }



            if(x_error > stepsize)
                magnitude = stepsize/x_error;
            else
                magnitude = 1;

            NEWMAT::DiagonalMatrix Reg(_numtargdims);
            NEWMAT::DiagonalMatrix Reg2(Jtemp2.Nrows());
            Reg = 0.0001;
            Reg2 = 0.0001;
            M << (J*J.t()) + Reg;

            invConditioningBound(10000,M,Minv);

            //Minv = M.i();
            //PrintMatrix(Minv.Store(),_numtargdims,_numtargdims,"Minv: ");

            Jplus = J.t()*Minv;
            //PrintMatrix(Jplus.Store(),_numdofs,_numtargdims,"Jplus: ");
            if(bBalanceGradient)
            {
               Mbal << (Jtemp2*Jtemp2.t()) + Reg2;
               invConditioningBound(10000,Mbal,Mbalinv);


               //do ik, then move toward balance in null space
               nullspacestep = (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2.t()*Mbalinv)*(1*balancedx);

            }
            else
            {
                nullspacestep = 0;

            }

            step = magnitude*Jplus*(dx) + nullspacestep;

            //RAVELOG_INFO("stepnorm: %f   nullspacestepnorm: %f\n", step.NormFrobenius(), nullspacestep.NormFrobenius());
            if(bPRINT)
                PrintMatrix(step.Store(),1,step.Nrows(),"step: ");


            //add step and check for joint limits
            bLimit = false;
            for(int i = 0; i < _numdofs; i++)
            {
                q_s[i] = q_s_old[i] - step(i+1);
                if(q_s[i] < _lowerLimit[i] || q_s[i] > _upperLimit[i])
                {
                    if(bPRINT)
                        RAVELOG_INFO("Jacobian going past joint limit. J%d: %f outside %f to %f\n",i,q_s[i],_lowerLimit[i],_upperLimit[i]);

                    if(q_s[i] < _lowerLimit[i])
                        q_s[i] = _lowerLimit[i];
                    if(q_s[i] > _upperLimit[i])
                        q_s[i] = _upperLimit[i];

                    badjointinds.push_back(i); //note this will never add the same joint twice, even if bClearBadJoints = false
                    bLimit = true;

                }


            }

            //move back to previous point if any joint limits
            if(bLimit)
            {

                q_s = q_s_old;
            }

        }while(bLimit);


        if(bPRINT)
            RAVELOG_INFO("after limits\n");

    }

    RAVELOG_INFO("Iteration limit reached\n");
    return false;

}
