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

#include "stdafx.h"

#include "GeneralIK.h"

GeneralIK::GeneralIK(EnvironmentBase* penv) : IkSolverBase(penv)
{

}


bool GeneralIK::Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options)
{
    SIGMOID_BOUND = 1000000;
    LINEAR_TF_SLOPE = 1.0;
    SIGMOID_TF_SLOPE = 1.0;

    bPRINT = false;
    bDRAW = false;
    //numDOF = pmanip->_vecarmjoints.size();
    //_pRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);
    _pRobot = probot;
    RAVELOG_INFO("Initializing GeneralIK Solver\n");

    bTRANSLATION_ONLY = false;

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

    bBalance = false;

    return true;
}


void GeneralIK::ResizeMatrices()
{
    J.ReSize(_numtargdims,_numdofs);
    Jtemp.ReSize(6,_numdofs);
    Jplus.ReSize(_numdofs,_numtargdims);
    dx.ReSize(_numtargdims);
    step.ReSize(_numdofs);
    M.ReSize(_numdofs);
    Minv.ReSize(_numdofs);

    q_s_old.resize(_numdofs);
    _curtms.resize(_targtms.size());

    _Jp.ReSize(3,_numdofs);
    _Jp0.ReSize(3,_numdofs);
    _Jr.ReSize(3,_numdofs);
    _Jr0.ReSize(3,_numdofs);
    _Jr_proper.ReSize(3,_numdofs);
    q_limits.ReSize(_numdofs);

    Jtemp2.ReSize(2,_numdofs);

    ///////////////Nathan//////////////
    D.ReSize(_numdofs);  
    theta.resize(_numdofs);
    theta_old.resize(_numdofs);
    whichTF.resize(_numdofs,TF_Sigmoid);
    whichTF_old.resize(_numdofs,TF_Sigmoid);

	_S.ReSize(_numtargdims);
	_V.ReSize(_numtargdims,_numtargdims);


}

// end eff transform is the transform of the wrist with respect to the base arm link
bool GeneralIK::Solve(const Transform &_T, const dReal* q0, bool bCheckEnvCollision, dReal* qResult)
{    

}

bool GeneralIK::Solve(const Transform &_T, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{

    return true;
}

bool GeneralIK::Solve(const Transform &_T, const dReal* q0, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, dReal* qResult)
{
    solutionpath.resize(0);
    _targmanips.resize(0);
    _targtms.resize(0);

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


    //read in cog target if there is one
    if((bool)pFreeParameters[numtargets*8 + 1])
    {
        _numtargdims = _targtms.size()*6;//+3;
        cogtarg.x = pFreeParameters[numtargets*8 + 2];
        cogtarg.y = pFreeParameters[numtargets*8 + 3];
        cogtarg.z = pFreeParameters[numtargets*8 + 4];
        RAVELOG_INFO("cog target: %f %f %f\n",cogtarg.x,cogtarg.y,cogtarg.z);    

        //read in the support polygon
        int numpoints = pFreeParameters[numtargets*8 + 5];
        int offset = numtargets*8 + 6;

        _vsupportpolyx.resize(numpoints);
        _vsupportpolyy.resize(numpoints);
        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyx[i] = pFreeParameters[offset + i];
        }

        offset = numtargets*8 + 6 + numpoints;

        for(int i = 0; i < numpoints; i++)
        {
            _vsupportpolyy[i] = pFreeParameters[offset + i];
        }

        for(int i =0; i < _vsupportpolyy.size(); i++)
        {
            RAVELOG_INFO("x: %f    y:%f\n",_vsupportpolyx[i],_vsupportpolyy[i]);
        }

        bBalance = true;
    }
    else
    {
        bBalance = false;
        _numtargdims = _targtms.size()*6;
    }

    //RAVELOG_INFO("extratarg = %f\n",pFreeParameters[numtargets*8 + 1]);


    _numdofs = numdofs;
    

    if( (_oldnumdofs != _numdofs) || (_oldnumtargdims != _numtargdims) )
    {
        ResizeMatrices();
        _oldnumdofs = _numdofs;
        _oldnumtargdims = _numtargdims;
    }



    //bsuccess  = _SolveNathan(q_s);
    //bsuccess  = _SolveNathanHacked(q_s);
    bsuccess  = _Solve(q_s);

    //always copy the joint vals into q_s (this could be the closest the robot could get)
    memcpy(qResult, &q_s[0], numdofs*sizeof(dReal));

    DrawSolutionPath();
    RAVELOG_INFO("Number of iterations: %d x_error: %f\n",_numitr,x_error);
    return (bsuccess);

}

bool GeneralIK::Solve(const Transform &_T, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{

    return true;
}


bool GeneralIK::GetFreeParameters(dReal* pFreeParameters) const
{

    return true;
}


bool GeneralIK::_Solve(std::vector<dReal>& q_s)
{

    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;    
    badjointinds.resize(0);
    prev_error = 1000000.0;

    //initialize stepsize and epsilon
    maxstep = 0.1*_targtms.size();
    stepsize = maxstep;
    epsilon = 0.001;

    bClearBadJoints = true; //setting this to true will make the algorithm attempt to move joints that are at joint limits at every iteration
        

    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            GetEnv()->plot3( _targtms[i].trans, 1, 0, 5, Vector(0,1,0) );
        }
    }

    bBalanceGradient = false;
    Vector perpvec;
    //while(1)
    for(int kk = 0; kk < 3000; kk++)
    {

        _numitr++;

        bLimit = false;

        _pRobot->SetActiveDOFValues(NULL, &q_s[0]);
        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                GetEnv()->plot3( _curtms[i].trans, 1, 0, 5, Vector(1,0,0) );
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

            RAVELOG_INFO("Projection stalled _numitr: %d\n",_numitr);
            return false;
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
            if(x_error < epsilon && (!bBalance || CheckSupport(curcog)))
            {        
                if(bPRINT)
                    RAVELOG_INFO("Projection successfull _numitr: %d\n",_numitr);
                return true;
            }

            //only need to compute the jacobian once if there are joint limit problems
            if(bLimit == false)
            {
                for(int i = 0; i < _targmanips.size();i++)
                {
                    _pRobot->SetActiveManipulator(_targmanips[i]);
                    GetFullJacobian(_curtms[i],_targtms[i],Jtemp);
                    J.Rows(i*6 +1,(i+1)*6) = Jtemp;
                }

                if(bBalance)
                {
                   //the cog jacobian should only be 2 dimensional, b/c we don't care about z
                   GetCOGJacobian(Transform(),Jtemp2,curcog);
                   curcog.z = 0;
                   if(!CheckSupport(curcog))
                   {    

                        GetClosestPointOnPolygon(curcog,cogtarg,perpvec);
                        bBalanceGradient = true;
                        balancedx[0] = (curcog.x - cogtarg.x);
                        balancedx[1] = (curcog.y - cogtarg.y);
                   }
                   else
                   {
                        balancedx[0] = 0;
                        balancedx[1] = 0;
                        bBalanceGradient = false;
                   }
                }

    
            }
            //eliminate bad joint columns from the Jacobian
            for(int j = 0; j < badjointinds.size(); j++)
                for(int k = 0; k < _numtargdims; k++)
                      J[k][badjointinds[j]] = 0;

            //if(bBalanceGradient)
            //{

                for(int j = 0; j < badjointinds.size(); j++)
                    for(int k = 0; k <2; k++)
                          Jtemp2[k][badjointinds[j]] = 0;

            //}

            //PrintMatrix(J.Store(),_numtargdims,_numdofs,"J: ");



            if(x_error > stepsize)
                magnitude = stepsize/x_error;
            else
                magnitude = 1;       


            M << (J*J.t());// + Reg;

            invConditioningBound(100,M,Minv);
            //Minv = (M + NEWMAT::IdentityMatrix(_numtargdims)*0.001).i();


            //Jplus = J.t()*((J*J.t()) + Reg).i();
            Jplus = J.t()*Minv;

            //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");

            
            for(int i = 0; i < _targmanips.size(); i++)
            {
                dx[3 + i*6] = -dx[3 + i*6];
                dx[4 + i*6] = -dx[4 + i*6];
                dx[5 + i*6] = -dx[5 + i*6];
            }
       

            //this will try to move toward the joint centers
            //for(int i = 0; i < _numdofs; i++)   
            //{
            //    dReal value = 0.0;
            //    value = q_s[i] - 0.5*(_lowerLimit[i] + _upperLimit[i]);
            //    q_limits[i] = value;
            //}
            //step = magnitude*Jplus*(dx) + 0.01*(NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*q_limits/q_limits.NormFrobenius();

            if(bBalanceGradient)
            {
               Mbal << (Jtemp2*Jtemp2.t());
               invConditioningBound(100,Mbal,Mbalinv);


               //Nullspace allows motion parallel to edge of support poylgon (problem is corners)
               //NEWMAT::RowVector perp(2);
               //perp[0] = perpvec.x;
               //perp[1] = perpvec.y;

               //Mbalperp << ( (perp*Jtemp2)*(perp*Jtemp2).t());
               //invConditioningBound(100,Mbalperp,Mbalperpinv);


               //step = (Jtemp2.t()*Mbalinv)*(1.01*balancedx) + (NEWMAT::IdentityMatrix(_numdofs) - ((perp*Jtemp2).t()*Mbalperpinv)*(perp*Jtemp2))*Jplus*(magnitude*dx);

               //don't allow any motion of cg
               //step = (Jtemp2.t()*Mbalinv)*(1.01*balancedx) + (NEWMAT::IdentityMatrix(_numdofs) - (Jtemp2.t()*Mbalinv)*Jtemp2)*Jplus*(magnitude*dx);

               //move critical cg motion, then do ik, then do optional cg motion
               NEWMAT::ColumnVector optionalbalancedx(2); //this should point to the center of the polygon
               optionalbalancedx[0] = curcog.x - 0;
               optionalbalancedx[1] = curcog.y - 0;

               //step = (Jtemp2.t()*Mbalinv)*(1.01*balancedx) + 
               //       (NEWMAT::IdentityMatrix(_numdofs) - (Jtemp2.t()*Mbalinv)*Jtemp2)*Jplus*(magnitude*dx)  +
               //       (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2.t()*Mbalinv)*(optionalbalancedx);


               step =  magnitude*Jplus*(dx) + (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2.t()*Mbalinv)*(1*optionalbalancedx);

               //PrintMatrix(step.Store(),1,_numdofs,"step: ");
                //step = magnitude*Jplus*(dx);
            }
            else
            {

                if(1 || !bBalance)
                    step = magnitude*Jplus*(dx);
                else
                {
                   //this should try to improve the cg if possible but it makes it take longer to converge
                   NEWMAT::ColumnVector optionalbalancedx(2); //this should point to the center of the polygon
                   optionalbalancedx[0] = curcog.x - 0;
                   optionalbalancedx[1] = curcog.y - 0;

                   step =  Jplus*(magnitude*dx)  +
                          (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2.t()*Mbalinv)*(optionalbalancedx);

                }

            }
            if(bPRINT)
                PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
           

            //set dx back in case we are in a joint limit loop
            for(int i = 0; i < _targmanips.size(); i++)
            {
                dx[3 + i*6] = -dx[3 + i*6];
                dx[4 + i*6] = -dx[4 + i*6];
                dx[5 + i*6] = -dx[5 + i*6];
            }
        

            //add step and check for joint limits
            bLimit = false;
            for(int i = 0; i < _numdofs; i++)
            {
                q_s[i] = q_s_old[i] - step[i];
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



dReal GeneralIK::TransformDifferenceVectorized(dReal * dx,std::vector<Transform>& tm_refs, std::vector<Transform>& tm_targs)
{


    for(int i = 0; i < tm_refs.size(); i++)
    {
        TransformDifference(dx + i*6,tm_refs[i],tm_targs[i]);
    }

    _sumsqr2 = 0;
    for(int i = 0; i < tm_refs.size()*6; i++)
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
        dx[3] = 0;
        dx[4] = 0;
        dx[5] = 0;
    }
    else
    {
        QuatToRPY(_tmtemp,dx[3],dx[4],dx[5]);
    }

    _sumsqr = 0;
    for(int i = 0; i < 6; i++)
        _sumsqr += dx[i]*dx[i];

    //RAVELOG_INFO("dx: %f %f %f %f %f %F\n",dx[0],dx[1],dx[2],dx[3],dx[4],dx[5]);

    return sqrt(_sumsqr);
}


void GeneralIK::GetFullJacobian(Transform tEE, Transform taskframe_in, NEWMAT::Matrix& J)
{

    _pRobot->CalculateActiveJacobian(_pRobot->GetActiveManipulator()->pEndEffector->GetIndex(), tEE.trans, _Jp0.Store());
    //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");


    _pRobot->CalculateActiveAngularVelocityJacobian(_pRobot->GetActiveManipulator()->pEndEffector->GetIndex(), _Jr0.Store());

    //PrintMatrix(_Jr0.Store(),3,_numdofs,"Jr0: ");

    
    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];

    _Jp = _tasktm * _Jp0;
    _Jr = _tasktm * _Jr0;

    //PrintMatrix(_tasktm.Store(),3,3,"tasktm: ");
    //PrintMatrix(_Jp.Store(),3,_numdofs,"Jp: ");
    //PrintMatrix(_Jr.Store(),3,_numdofs,"Jr: ");

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

    J = _Jp & _Jr_proper;

    //PrintMatrix(J.Store(),3,_numdofs,"J1: ");

}

void GeneralIK::GetCOGJacobian(Transform taskframe_in, NEWMAT::Matrix& J, Vector& center)
{


    J = 0.0;

    center = Vector(0,0,0,0);
    dReal fTotalMass = 0;
    std::vector<KinBody::Link*>::const_iterator itlink;



    _TMtask = TransformMatrix(taskframe_in.inverse());
    _tasktm(1,1) = _TMtask.m[0];        _tasktm(1,2) = _TMtask.m[1];        _tasktm(1,3) = _TMtask.m[2];
    _tasktm(2,1) = _TMtask.m[4];        _tasktm(2,2) = _TMtask.m[5];        _tasktm(2,3) = _TMtask.m[6];
    _tasktm(3,1) = _TMtask.m[8];        _tasktm(3,2) = _TMtask.m[9];        _tasktm(3,3) = _TMtask.m[10];


    //PrintMatrix(E_rpy.Store(),3,3,"E_rpy: ");


    FORIT(itlink, _pRobot->GetLinks()) {
        //RAVELOG_INFO("comoffset: %f %f %f\n", (*itlink)->GetCOMOffset().x,(*itlink)->GetCOMOffset().y,(*itlink)->GetCOMOffset().z);
        //GetEnv()->plot3((*itlink)->GetTransform() * (*itlink)->GetCOMOffset(), 1, 0, 10, Vector(0,0,1) );



        _pRobot->CalculateActiveJacobian((*itlink)->GetIndex(), ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset()), _Jp0.Store());
        //PrintMatrix(_Jp0.Store(),3,_numdofs,"Jp0: ");



        _Jp = _tasktm * _Jp0;


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


    //center.z = 0;
    //GetEnv()->plot3(center, 1, 0, 10, Vector(0,1,0) );
}



void GeneralIK::PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement)
{
    //return;

    wstringstream s;
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
        }
        s << endl;
    }
    RAVEPRINT(s.str().c_str());

}


void GeneralIK::QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi)
{
    
    a = tm.rot.x;
    b = tm.rot.y;
    c = tm.rot.z;
    d = tm.rot.w;

    psi = atan2(2*a*b + 2*c*d, a*a - b*b - c*c + d*d); //psi
    theta = -asin(2*b*d-2*a*c); //theta
    phi = atan2(2*a*d+2*b*c, a*a + b*b - c*c - d*d); //phi


    //keep the rotations between -pi and pi
    if(psi > M_PI) psi -= 2*M_PI;
    if(theta > M_PI) theta -= 2*M_PI;
    if(phi > M_PI) phi -= 2*M_PI;

    if(psi < -M_PI) psi += 2*M_PI;
    if(theta < -M_PI) theta += 2*M_PI;
    if(phi < -M_PI) phi += 2*M_PI;

    //psi = theta = phi = 0;
}


bool GeneralIK::CheckSupport(Vector center)
{
    //return true;

    dReal testx = center.x;
    dReal testy = center.y;
    dReal * vertx = &_vsupportpolyx[0];
    dReal * verty = &_vsupportpolyy[0];
    int nvert = _vsupportpolyx.size();
//int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
    }
   
    center.z = 0;
    solutionpath.push_back(center);
    if(c)    
    {
        GetEnv()->plot3(center, 1, 0, 10, Vector(0,1,0) );
        return true;
    }
    else
    {
        GetEnv()->plot3(center, 1, 0, 10, Vector(1,0,0) );
        return false;
    }
}



////////////////////////////////////Nathan's sigmoiding thing/////////////////////////////
bool GeneralIK::_SolveNathanHacked(std::vector<dReal>& q_s)
{
    RAVELOG_INFO("\nSolve called!!!\n");

    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;    
    badjointinds.resize(0);
    prev_error = 1000000.0;

    //initialize stepsize and epsilon
    //maxstep = 0.2;
    maxstep = .1;
    //maxstep = .8;
    //stepsize = maxstep;
    epsilon = 0.001;

    QToTheta(whichTF, q_s, theta);


    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            GetEnv()->plot3( _targtms[i].trans, 1, 0, 5, Vector(0,1,0) );
        }
    }


    int t = 1;
    //while(1)
    for(int kk = 0; kk < 3000; kk++)
    {
        //RAVELOG_INFO("kk: %d\n",kk);
        _numitr++;

        bLimit = false;
        
        // Setup robot stuff
        _pRobot->SetActiveDOFValues(NULL, &q_s[0]);
        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                GetEnv()->plot3( _curtms[i].trans, 1, 0, 5, Vector(1,0,0) );
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");           
        

        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);
    

        // Stepsize stuff - automatically reduces stepsize if error increases
        //if(bLimit || x_error >= prev_error || prev_error - x_error < epsilon)
        /*
        if((x_error >= prev_error || prev_error - x_error < epsilon/10) && x_error > epsilon)
        {
            if(bPRINT)
            {
                RAVELOG_INFO("no progress\n");
                RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);
            }
            stepsize = stepsize/2;


            q_s = q_s_old;
            theta = theta_old;
            whichTF = whichTF_old;
            x_error = prev_error;
            //return false;
        }
        else
            stepsize = maxstep;
        */

        if(x_error < prev_error && prev_error - x_error < epsilon/10)
        {
            RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);

        }


        stepsize = maxstep;

        
        if(bPRINT)
            RAVELOG_INFO("stepsize: %f\n",stepsize);

        //don't let step size get too small
        if(stepsize < epsilon)
        {
            RAVELOG_INFO("Projection stalled _numitr: %d\n",_numitr);
            return false;
        }


        q_s_old = q_s;
        theta_old = theta;
        whichTF_old = whichTF;
        prev_error = x_error;
    

        if(x_error < epsilon)
        {        
            //if(bPRINT)
            RAVELOG_INFO("Projection successful _numitr: %d\n",_numitr);
            return true;
        }


        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            GetFullJacobian(_curtms[i],_targtms[i],Jtemp);
            J.Rows(i*6 +1,(i+1)*6) = Jtemp;
        }

        //PrintMatrix(J.Store(),_numtargdims,_numdofs,"J: ");

        if(x_error > stepsize)
            magnitude = stepsize/x_error;
        else
            magnitude = 1;       

        GetD(whichTF,theta,D);
        /*
        NEWMAT::Matrix d(1,D.Ncols());
        for (int i = 0; i < D.Nrows(); ++i){
            d[0][i] = D[i];        
        }
        PrintMatrix(d.Store(),d.Nrows(),d.Ncols(),"D: ");
        */

        M = 0.0;
        Minv = 0.0;
        

        //M << (J.t()*J);
        //invConditioning(1000,M,Minv);
        //Jplus = D.i()*(Minv)*J.t();
        

        J = J*D;
        M << (J*J.t());
        invConditioningBound(100.0,M,Minv);
        //Minv = NEWMAT::IdentityMatrix(_numtargdims);
        Jplus = J.t()*(Minv);
        //Jplus = J.t();
        
        //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");

        
        for(int i = 0; i < _targmanips.size(); i++)
        {
            dx[3 + i*6] = -dx[3 + i*6];
            dx[4 + i*6] = -dx[4 + i*6];
            dx[5 + i*6] = -dx[5 + i*6];
        }
   

        step = magnitude*Jplus*(dx);

        if(bPRINT)
            PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
       

        //set dx back in case we are in a joint limit loop
        for(int i = 0; i < _targmanips.size(); i++)
        {
            dx[3 + i*6] = -dx[3 + i*6];
            dx[4 + i*6] = -dx[4 + i*6];
            dx[5 + i*6] = -dx[5 + i*6];
        }

        for(int i = 0; i < _numdofs; i++)
        {
            //RAVELOG_INFO("Dinv: %f Step: %f Theta_old: %f\n",Dinv[i*numdofs + i],stepXYZ[i] + stepRPY[i],theta_old[i]);
            theta[i] = theta_old[i] - step[i];
        }

        ThetaToQ(whichTF,theta,q_s);


        //determine which transfer function we should use for the next step
        for(int i =0; i < _numdofs; i++)
        {
            
            if( step[i]*(q_s[i] - 0.5*(_lowerLimit[i] + _upperLimit[i])) < 0)
            {
               // RAVELOG_INFO("%d: old: %d  new: %d\n",i,whichTF[i],TF_Sigmoid);    
                whichTF[i] = TF_Sigmoid;
            }
            else
            {
                //if(i == 19)
               //     RAVELOG_INFO("Elbow is linear!\n");
               //RAVELOG_INFO("%d: old: %d  new: %d\n",i,whichTF[i],TF_Linear);
                whichTF[i] = TF_Linear;
            }
        }
        QToTheta(whichTF,q_s,theta);


        //temporary check for joint limits, shouldn't need this
        bLimit = false;
        for(int i = 0; i < _numdofs; i++)
        {
            if(q_s[i] < _lowerLimit[i] || q_s[i] > _upperLimit[i])
            {
                //if(bPRINT)
                RAVELOG_INFO("Jacobian going past joint limit. J%d: %f outside %f to %f q_old: %f\n",i,q_s[i],_lowerLimit[i],_upperLimit[i],q_s_old[i]);

                if(q_s[i] < _lowerLimit[i])
                    q_s[i] = _lowerLimit[i]+0.01;
                if(q_s[i] > _upperLimit[i])
                    q_s[i] = _upperLimit[i]-0.01;
    
                bLimit = true;

            }

        }
        if(bLimit)
            return false;
        //sleep(0.1);

        t = t + 1;

    }
    //return true;
    RAVELOG_INFO("Goal not reached...");
    return false;    
}

bool GeneralIK::_SolveNathan(std::vector<dReal>& q_s)
{


    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;    
    badjointinds.resize(0);
    prev_error = 1000000.0;

    //initialize stepsize and epsilon
    maxstep = 0.2;
    stepsize = maxstep;
    epsilon = 0.001;

    QToTheta(whichTF, q_s, theta);


    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            GetEnv()->plot3( _targtms[i].trans, 1, 0, 5, Vector(0,1,0) );
        }
    }


    while(1)
    //for(int kk = 0; kk < 1; kk++)
    {

        _numitr++;

        bLimit = false;

        _pRobot->SetActiveDOFValues(NULL, &q_s[0]);
        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                GetEnv()->plot3( _curtms[i].trans, 1, 0, 5, Vector(1,0,0) );
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");           
        

        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);
    

        //if(bLimit || x_error >= prev_error || prev_error - x_error < epsilon)
        if((x_error >= prev_error || prev_error - x_error < epsilon/10) && x_error > epsilon)
        {
            if(bPRINT)
            {
                RAVELOG_INFO("no progress\n");
                RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);
            }
            stepsize = stepsize/2;


            q_s = q_s_old;
            theta = theta_old;
            whichTF = whichTF_old;
            x_error = prev_error;
            //return false;
        }
        else
            stepsize = maxstep;

        
        if(bPRINT)
            RAVELOG_INFO("stepsize: %f\n",stepsize);

        //don't let step size get too small
        if(stepsize < epsilon)
        {
            RAVELOG_INFO("Projection stalled _numitr: %d\n",_numitr);
            return false;
        }


        q_s_old = q_s;
        theta_old = theta;
        whichTF_old = whichTF;
        prev_error = x_error;
    

        if(x_error < epsilon)
        {        
            if(bPRINT)
                RAVELOG_INFO("Projection successfull _numitr: %d\n",_numitr);
            return true;
        }


        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            GetFullJacobian(_curtms[i],_targtms[i],Jtemp);
            J.Rows(i*6 +1,(i+1)*6) = Jtemp;
        }

        //PrintMatrix(J.Store(),_numtargdims,_numdofs,"J: ");

        if(x_error > stepsize)
            magnitude = stepsize/x_error;
        else
            magnitude = 1;       

        GetD(whichTF,theta,D);
        

        M = 0.0;
        Minv = 0.0;
        

        //M << (J.t()*J);
        //invConditioning(1000,M,Minv);
        //Jplus = D.i()*(Minv)*J.t();
        

        J = J*D;
        M << (J*J.t());
        invConditioning(10000,M,Minv);
        //Minv = NEWMAT::IdentityMatrix(_numtargdims);
        Jplus = J.t()*(Minv);
        
        //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");

        
        for(int i = 0; i < _targmanips.size(); i++)
        {
            dx[3 + i*6] = -dx[3 + i*6];
            dx[4 + i*6] = -dx[4 + i*6];
            dx[5 + i*6] = -dx[5 + i*6];
        }
   

        step = magnitude*Jplus*(dx);

        if(bPRINT)
            PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
       

        //set dx back in case we are in a joint limit loop
        for(int i = 0; i < _targmanips.size(); i++)
        {
            dx[3 + i*6] = -dx[3 + i*6];
            dx[4 + i*6] = -dx[4 + i*6];
            dx[5 + i*6] = -dx[5 + i*6];
        }

        for(int i = 0; i < _numdofs; i++)
        {
            //RAVELOG_INFO("Dinv: %f Step: %f Theta_old: %f\n",Dinv[i*numdofs + i],stepXYZ[i] + stepRPY[i],theta_old[i]);
            theta[i] = theta_old[i] - step[i];
        }

        ThetaToQ(whichTF,theta,q_s);


        //determine which transfer function we should use for the next step
        for(int i =0; i < _numdofs; i++)
        {
            
            if( step[i]*(q_s[i] - 0.5*(_lowerLimit[i] + _upperLimit[i])) < 0)
            {
               // RAVELOG_INFO("%d: old: %d  new: %d\n",i,whichTF[i],TF_Sigmoid);    
                whichTF[i] = TF_Sigmoid;
            }
            else
            {
                //if(i == 19)
               //     RAVELOG_INFO("Elbow is linear!\n");
               //RAVELOG_INFO("%d: old: %d  new: %d\n",i,whichTF[i],TF_Linear);
                whichTF[i] = TF_Linear;
            }
        }
        QToTheta(whichTF,q_s,theta);


        //temporary check for joint limits, shouldn't need this
        bLimit = false;
        for(int i = 0; i < _numdofs; i++)
        {
            if(q_s[i] < _lowerLimit[i] || q_s[i] > _upperLimit[i])
            {
                //if(bPRINT)
                RAVELOG_INFO("Jacobian going past joint limit. J%d: %f outside %f to %f q_old: %f\n",i,q_s[i],_lowerLimit[i],_upperLimit[i],q_s_old[i]);

                if(q_s[i] < _lowerLimit[i])
                    q_s[i] = _lowerLimit[i]+0.01;
                if(q_s[i] > _upperLimit[i])
                    q_s[i] = _upperLimit[i]-0.01;
    
                bLimit = true;

            }

        }
        if(bLimit)
            return false;
        //sleep(0.1);
  




    }
    return false;
    
}


void GeneralIK::ThetaToQ(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& theta_in, std::vector<dReal>& q_out)
{


    for(int i = 0; i < theta_in.size(); i++)
    {

        if(whichTF_in[i] == TF_Sigmoid)
        {
            //_eTheta = exp(CLAMP_ON_RANGE(-SIGMOID_TF_SLOPE*theta_in[i],-SIGMOID_BOUND,SIGMOID_BOUND));
            _eTheta = exp(-SIGMOID_TF_SLOPE*theta_in[i]);
            q_out[i] = (_eTheta/(1+_eTheta)) * _lowerLimit[i] + (1/(1+_eTheta)) * _upperLimit[i];
        }
        else if(whichTF_in[i] == TF_Linear)
        {
            q_out[i] = LINEAR_TF_SLOPE * theta_in[i];
        }
    }

}


void GeneralIK::QToTheta(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& q_in, std::vector<dReal>& theta_out)
{

    for(int i = 0; i < theta_out.size(); i++)
    {
        if(whichTF_in[i] == TF_Sigmoid)
        {
            theta_out[i] = -log((_upperLimit[i] - q_in[i])/(q_in[i] - _lowerLimit[i]))/SIGMOID_TF_SLOPE;
        }
        else if(whichTF_in[i] == TF_Linear)
        {
            theta_out[i] = (1.0/LINEAR_TF_SLOPE) * q_in[i];
        }
    }

}


void GeneralIK::GetD(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& theta_in, NEWMAT::DiagonalMatrix& D)
{
    
    for(int i = 0; i < _numdofs; i++)
    {
        if(whichTF_in[i] == TF_Sigmoid)
        {
            //_temp = CLAMP_ON_RANGE(theta_in[i],-SIGMOID_BOUND,SIGMOID_BOUND);
            _temp = theta_in[i];
            D[i] = SIGMOID_TF_SLOPE*(_upperLimit[i] - _lowerLimit[i])*(1.0/(1.0+exp(-SIGMOID_TF_SLOPE*_temp))) * (1.0 - (1.0/(1.0+exp(-SIGMOID_TF_SLOPE*_temp))));
        }
        else if(whichTF_in[i] == TF_Linear)
        {
            D[i] = LINEAR_TF_SLOPE;
        }

    }   
}

/*
void GeneralIK::invConditioning(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{

	NEWMAT::EigenValues(A,_S,_V);
	// Find the maximum eigenvalue
	dReal maxEig = 0;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e > maxEig) maxEig = e;
	}

	dReal minEigDesired = maxEig/maxConditionNumber;
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		//if (e < minEigDesired) e = minEigDesired;
        dReal a = 50;
		_S(i) = 1/a*log( exp(a*e) + exp(a*maxEig/maxConditionNumber) );
	}

    //this just reconstructs the A matrix with better conditioning
	//Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();

}
*/
void GeneralIK::invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed)
{

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
	for (int i = 1; i <= _S.Nrows(); ++i){
		dReal e = _S(i);
		if (e < minEigDesired) e = minEigDesired;
        if (maxEig > 100) e = e/maxEig*100;
        _S(i) = e;
	}

    //this just reconstructs the A matrix with better conditioning
	//Afixed << _V * _S * _V.t();
    

    //this will do the inversion 
    Afixed << _V * _S.i() * _V.t();

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

    GetEnv()->drawlinestrip(&solutionpath[0].x,pathlength,sizeof(solutionpath[0]),3, RaveVector<float>(0, 0, 1, 0));



    GetEnv()->plot3( solutionpath[pathlength-1], 1, 0, 15, Vector(1,1,0) );
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



    /*
    std::vector<Vector> tempvecs(2);
    tempvecs[0] = point;
    tempvecs[1] = closestpoint;

   // GetEnv()->plot3(point, 1, 0, 8, Vector(1,0,0) );
    GetEnv()->drawlinestrip(&tempvecs[0].x,tempvecs.size(),sizeof(tempvecs[0]),6, RaveVector<float>(0, 1, 0, 1));
    */
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
