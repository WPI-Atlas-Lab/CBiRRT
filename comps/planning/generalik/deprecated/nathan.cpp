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

// Copyright Carnegie Mellon University and Intel Corporation
//
//////////////////////////Nathan's sigmoiding thing/////////////////////////////

//put this GeneralIK.h
//    enum TF_Type {
//        TF_Sigmoid=0,
//        TF_Linear=1,
//    };
//    void QToTheta(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& q_in, std::vector<dReal>& theta_out);
//    void ThetaToQ(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& theta_in, std::vector<dReal>& q_out);
//    void GetD(const std::vector<TF_Type>& whichTF_in, const std::vector<dReal>& theta_in, NEWMAT::DiagonalMatrix& D);
//    NEWMAT::DiagonalMatrix D;
//    std::vector<TF_Type> whichTF;
//    std::vector<TF_Type> whichTF_old;
//    std::vector<dReal> theta;
//    std::vector<dReal> theta_old;
//    dReal _temp;
//    dReal _eTheta;
//    dReal LINEAR_TF_SLOPE;
//    dReal SIGMOID_TF_SLOPE;


//put these in GeneralIK::ResizeMatrices()
//    D.ReSize(_numdofs);  
//    theta.resize(_numdofs);
//    theta_old.resize(_numdofs);
//    whichTF.resize(_numdofs,TF_Sigmoid);
//    whichTF_old.resize(_numdofs,TF_Sigmoid);

//put this in GeneralIK::Init()
//    SIGMOID_BOUND = INF;
//    LINEAR_TF_SLOPE = 0.05;
//    SIGMOID_TF_SLOPE = 1.0;

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
            q_out[i] = LINEAR_TF_SLOPE*(_upperLimit[i] - _lowerLimit[i]) * theta_in[i];
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
            theta_out[i] = (1.0/(LINEAR_TF_SLOPE*(_upperLimit[i] - _lowerLimit[i]))) * q_in[i];
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
            D[i] = LINEAR_TF_SLOPE*(_upperLimit[i] - _lowerLimit[i]);
        }

    }   
}

bool GeneralIK::_SolveNathanHacked(std::vector<dReal>& q_s)
{
    RAVELOG_INFO("\nSolve called!!!\n");
    int numbaditr = 0;
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
            GetEnv()->plot3( &(DoubleVectorToFloatVector(_targtms[i].trans)[0]), 1, 0, 10, Vector(0,1,0) );
        }
    }


    int t = 1;
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

        if(x_error <= prev_error && prev_error - x_error < epsilon/10)
        {
            RAVELOG_INFO("prev: %f x_err: %f limit %d\n",prev_error,x_error,(int)bLimit);
            //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");
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


        //check for termination condition
        /*
        dReal gradmag = (D*J.t()*dx).NormFrobenius();
        RAVELOG_INFO("gradient: %f\n",gradmag);
        //if( gradmag < 0.000005)
        if(prev_error - x_error < epsilon/_targmanips.size() )
        {
            RAVELOG_INFO("error diff: %d\n",prev_error - x_error);
            numbaditr++;
        }
        else
            numbaditr = 0;
            
        if(numbaditr > 1000)
        {
            RAVELOG_INFO("Gradient magnitude is too small for (%d) iterations, terminating.\n",numbaditr);
            return false;
        }
        */

        q_s_old = q_s;
        theta_old = theta;
        whichTF_old = whichTF;
        prev_error = x_error;    


        M = 0.0;
        Minv = 0.0;
        

        //M << (J.t()*J);
        //invConditioning(1000,M,Minv);
        //Jplus = D.i()*(Minv)*J.t();
        

        J = J*D;
        M << (J*J.t());
        int didFix = invConditioningBound(1000,M,Minv);
        //Minv = NEWMAT::IdentityMatrix(_numtargdims);
        Jplus = J.t()*(Minv);
        //Jplus = J.t();
        
        //PrintMatrix(Jplus.Store(),Jplus.Nrows(),Jplus.Ncols(),"Jplus: ");

        
        step = magnitude*Jplus*(dx);

        if(bPRINT)
            PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
       

        for(int i = 0; i < _numdofs; i++)
        {
            //RAVELOG_INFO("Dinv: %f Step: %f Theta_old: %f\n",Dinv[i*numdofs + i],stepXYZ[i] + stepRPY[i],theta_old[i]);
            theta[i] = theta_old[i] - step[i];
        }

        ThetaToQ(whichTF,theta,q_s);
        bool bChanged = false;
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
            /*
            if(whichTF[i] != whichTF_old[i])
            {
                bChanged = true;
                RAVELOG_INFO("Changed (%d to %d) %d: q: %f theta: %f step: %f\n",whichTF[i],whichTF_old[i],i,q_s[i],theta[i],step[i]);
            }
            */
        }
        QToTheta(whichTF,q_s,theta);
/*
        if(!bChanged)
        {
            for(int i = 0; i < _numdofs; i++)
            {
                //RAVELOG_INFO("Dinv: %f Step: %f Theta_old: %f\n",Dinv[i*numdofs + i],stepXYZ[i] + stepRPY[i],theta_old[i]);
                theta[i] = theta_old[i] - step[i];
            }

            ThetaToQ(whichTF,theta,q_s);
            QToTheta(whichTF,q_s,theta);
            
        }
        else
        {

            QToTheta(whichTF,q_s,theta);
        }
*/


        //temporary check for joint limits, shouldn't need this
        bLimit = false;
        for(int i = 0; i < _numdofs; i++)
        {
            if(q_s[i] < _lowerLimit[i] || q_s[i] > _upperLimit[i])
            {
                //if(bPRINT)
                RAVELOG_INFO("Jacobian going past joint limit. J%d: %f outside %f to %f q_old: %f\n",i,q_s[i],_lowerLimit[i],_upperLimit[i],q_s_old[i]);
                RAVELOG_INFO("WhichTF_old: %d   WhichTF_new: %d\n",whichTF_old[i],whichTF[i]);
                if(q_s[i] < _lowerLimit[i])
                    q_s[i] = q_s_old[i];//_lowerLimit[i]+0.01;
                if(q_s[i] > _upperLimit[i])
                    q_s[i] = q_s_old[i];//_upperLimit[i]-0.01;
    
                bLimit = true;

            }

        }
        if(bLimit)
        {
            QToTheta(whichTF,q_s,theta);
        }
        //    return false;
        //sleep(0.1);

        t = t + 1;

    }
    //return true;
    RAVELOG_INFO("Goal not reached...");
    return false;    
}
/*
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
            J.Rows(i*_dimspergoal +1,(i+1)*_dimspergoal) = Jtemp;
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

        
  

        step = magnitude*Jplus*(dx);

        if(bPRINT)
            PrintMatrix(step.Store(),1,step.Nrows(),"step: ");
       

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

*/
