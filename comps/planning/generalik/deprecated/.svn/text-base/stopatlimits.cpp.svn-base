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

bool GeneralIK::_Solve(std::vector<dReal>& q_s)
{

    if(bPRINT)
        RAVELOG_INFO("Solving...\n");

    //clear from previous run
    _numitr = 0;    
    badjointinds.resize(0);
    prev_error = 1000000.0;

    //initialize stepsize and epsilon
    maxstep = 0.1;//*_targtms.size();
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
    for(int kk = 0; kk < 500; kk++)
    {

        _numitr++;

        bLimit = false;
        if(!CheckDOFValueIntegrity(q_s))
            return false;
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
                GetEnv()->plot3( _curtms[i].trans, 1, 0, 5, Vector(1,0,0) );
        }

        x_error = TransformDifferenceVectorized(dx.Store(),_targtms,_curtms);

        if(bPRINT)
            PrintMatrix(dx.Store(),1,dx.Nrows(),"dx: ");           
        

        if(bPRINT)
            RAVELOG_INFO("x error: %f\n",x_error);


        //if balance stuff is on, error will go up sometimes, so don't do this check
        if(0 && !bBalance && (x_error >= prev_error || (prev_error - x_error < epsilon/10))&& x_error > epsilon)
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
                    J.Rows(i*_dimspergoal +1,(i+1)*_dimspergoal) = Jtemp;
                    
                }


                if(bBalance)
                {
                   //the cog jacobian should only be 2 dimensional, b/c we don't care about z
                   GetCOGJacobian(Transform(),Jtemp2,curcog);

                   if(!CheckSupport(curcog))
                   {    
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

            if(bBalance)
            {
                for(int j = 0; j < badjointinds.size(); j++)
                    for(int k = 0; k <2; k++)
                          Jtemp2[k][badjointinds[j]] = 0;
            }



            if(x_error > stepsize)
                magnitude = stepsize/x_error;
            else
                magnitude = 1;       


            M << (J*J.t());// + Reg;

            invConditioningBound(1000,M,Minv);
            //Minv = M.i();
            Jplus = J.t()*Minv;

            if(bBalanceGradient)
            {
               Mbal << (Jtemp2*Jtemp2.t());
               invConditioningBound(100,Mbal,Mbalinv);


               //do ik, then move toward balance in null space
               step =  magnitude*Jplus*(dx) + (NEWMAT::IdentityMatrix(_numdofs) - Jplus*J)*(Jtemp2.t()*Mbalinv)*(1*balancedx);

            }
            else
            {

                step = magnitude*Jplus*(dx);
            }
            if(bPRINT)
                PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
           

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
