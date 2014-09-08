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

bool GeneralIK::_SolveJLWeighting(std::vector<dReal>& q_s)
{
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


    if(bDRAW)
    {
        for(int i = 0; i < _targmanips.size();i++)
        {
            GetEnv()->plot3( &(DoubleVectorToFloatVector(_targtms[i].trans)[0]), 1, 0, 10, Vector(0,1,0) );
        }
    }

    NEWMAT::DiagonalMatrix W(_numdofs);
    std::vector<dReal> dHdTheta(_numdofs);
    std::vector<dReal> dHdTheta_old(_numdofs);
    //while(1)
    for(int kk = 0; kk < 500; kk++)
    {
        //RAVELOG_INFO("kk: %d\n",kk);
        _numitr++;

        bLimit = false;
        
        // Setup robot stuff
        _pRobot->SetActiveDOFValues(q_s);
        if(bWRITETRAJ)
        {
            for(int i = 0; i < _pRobot->GetActiveDOF(); i++)
            {
                _trajpoint.q[i] = q_s[i];
            }
            ptraj->AddPoint(_trajpoint);
        }
        for(int i = 0; i < _targmanips.size();i++)
        {
            _pRobot->SetActiveManipulator(_targmanips[i]);
            _curtms[i] = _pRobot->GetActiveManipulator()->GetEndEffectorTransform();
            if(bDRAW)
                GetEnv()->plot3( &(DoubleVectorToFloatVector(_curtms[i].trans)[0]), 1, 0, 5, Vector(1,0,0) );
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");           
        

        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);
    

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
            J.Rows(i*_dimspergoal +1,(i+1)*_dimspergoal) = Jtemp;
        }

        //PrintMatrix(J.Store(),_numtargdims,_numdofs,"J: ");

        //compute W for joints
        dHdTheta_old = dHdTheta;
        for(int i = 0; i < _numdofs; i++) 
        {
            //W[i] = 0.1*(q_s[i] - (_lowerLimit[i] + _upperLimit[i])/2); // /(_upperLimit[i] - _lowerLimit[i]);
            dHdTheta[i] = fabs( (_upperLimit[i] - _lowerLimit[i])*(_upperLimit[i] - _lowerLimit[i])*(2*q_s[i] - _upperLimit[i] - _lowerLimit[i]) )/(4*(_upperLimit[i] - q_s[i])*(_upperLimit[i] - q_s[i])*(q_s[i] - _lowerLimit[i])*(q_s[i] - _lowerLimit[i]) );


            if( (dHdTheta[i] - dHdTheta_old[i]) >= 0)
                W(i+1) = 1 + dHdTheta[i];
            else
                W(i+1) = 1;
        }  




        if(x_error > stepsize)
            magnitude = stepsize/x_error;
        else
            magnitude = 1;       


        M = 0.0;
        Minv = 0.0;

        M << (J*W.i()*J.t());
        invConditioningBound(1000.0,M,Minv);
        //Minv << (M + (0.001*NEWMAT::IdentityMatrix(_numtargdims))).i();
        //Minv = NEWMAT::IdentityMatrix(_numtargdims);
        Jplus = W.i()*J.t()*(Minv);
        //Jplus = J.t();
        
        //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");

        //PrintMatrix(W.Store(),1,W.Nrows(),"W: ");

        step = magnitude*Jplus*(dx);
        
        if(bPRINT)
            PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
       

        //temporary check for joint limits, shouldn't need this
        bLimit = false;
        for(int i = 0; i < _numdofs; i++)
        {
            q_s[i] = q_s_old[i] - step[i];
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
        //if(bLimit)
        //    return false;

    }
    //return true;
    RAVELOG_INFO("Goal not reached...");
    return false;    
}
