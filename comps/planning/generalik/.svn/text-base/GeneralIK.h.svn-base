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
/** \file GeneralIK.h
    \brief Defines the generalik class.
 */
#ifndef  GENERALIK_H
#define  GENERALIK_H


class GeneralIK : public IkSolverBase
{
public:

    GeneralIK(EnvironmentBasePtr penv);


#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int, boost::shared_ptr<std::vector<dReal> > result, IkReturnPtr) { return false; }
    bool Solve(const OpenRAVE::IkParameterization&, int, std::vector<std::vector<double, std::allocator<double> >, std::allocator<std::vector<double, std::allocator<double> > > >&, boost::shared_ptr<std::vector<boost::shared_ptr<OpenRAVE::IkReturn>, std::allocator<boost::shared_ptr<OpenRAVE::IkReturn> > > >) { return false; }
    bool Solve(const IkParameterization& param, const std::vector<double, std::allocator<double> >&, const std::vector<double, std::allocator<double> >&, int, boost::shared_ptr<std::vector<double, std::allocator<double> > >, OpenRAVE::IkReturnPtr) { return false; }
    bool Solve(const IkParameterization&, const std::vector<double, std::allocator<double> >&, int, std::vector<std::vector<double, std::allocator<double> >, std::allocator<std::vector<double, std::allocator<double> > > >&, boost::shared_ptr<std::vector<boost::shared_ptr<OpenRAVE::IkReturn>, std::allocator<boost::shared_ptr<OpenRAVE::IkReturn> > > >) { return false; }
    virtual bool Init(RobotBase::ManipulatorConstPtr pconstmanip){RobotBase::ManipulatorPtr pmanip = boost::const_pointer_cast<RobotBase::Manipulator>(pconstmanip); Init(pmanip);}
#endif

    virtual bool Init(RobotBase::ManipulatorPtr pmanip);
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result){return false;}

    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& qSolutions){return false;}

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result);

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& qSolutions){return false;}
    virtual bool SolveAll(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& solutions) { return false; }
    virtual bool SolveAll(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& solutions) { return false; }
    virtual int GetNumFreeParameters() const { return 0; }
    virtual bool GetFreeParameters(std::vector<dReal>& pFreeParameters) const {return false;}
    virtual RobotBasePtr GetRobot() const { return _pRobot; }

    virtual OpenRAVE::RobotBase::ManipulatorPtr GetManipulator() const {return _pmanip;}

    virtual int GetNumFreeParameters(){return 0;}
private:


    void invConditioning(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    int invConditioningBound(dReal maxConditionNumber, NEWMAT::SymmetricMatrix& A, NEWMAT::SymmetricMatrix &Afixed);
    dReal TransformDifferenceVectorized(dReal * dx,std::vector<Transform>& tm_refs, std::vector<Transform>& tm_targs);
    dReal TransformDifference(dReal * dx,Transform tm_ref, Transform tm_targ);
    void GetFullJacobian(Transform tEE, Transform taskframe_in, NEWMAT::Matrix& J);
    void GetCOGJacobian(Transform taskframe_in, NEWMAT::Matrix& J, Vector& center);
    void PrintMatrix(dReal* pMatrix, int numrows, int numcols,const char * statement);
    void QuatToRPY(Transform tm, dReal& psi, dReal& theta, dReal& phi);
    void ResizeMatrices();
    bool CheckSupport(Vector center);
    void DrawSolutionPath();
    void WriteTraj();
    void GetDistanceFromLineSegment(dReal cx, dReal cy, dReal ax, dReal ay,dReal bx, dReal by, dReal& distanceSegment,dReal& xout, dReal& yout);
    void GetClosestPointOnPolygon(Vector& point, Vector& closestpoint, Vector& perpvec);
    RobotBasePtr _pRobot;
    RobotBase::ManipulatorPtr  _pmanip;
    bool CheckDOFValueIntegrity(const std::vector<dReal>& q_in);


    dReal SIGMOID_BOUND;

    //Globally used
    int _numtargdims;
    int _numtransdims;
    int _numrotdims;
    int _oldnumtargdims;
    int _oldnumdofs;
    int _numdofs;
    int _dimspergoal;

    bool bTRANSLATION_ONLY;
    bool bPRINT;
    bool bDRAW;
    bool bWRITETRAJ;
    bool bQUAT;
    std::vector<int> _targmanips;
    std::vector<Transform> _targtms;
    std::vector<dReal> _lowerLimit, _upperLimit;
    Vector cogtarg;
    Vector curcog;
    bool bBalance;
    NEWMAT::ColumnVector balancedx;
    std::vector<Vector> solutionpath;
    TrajectoryBasePtr ptraj;
    //Trajectory::TPOINT _trajpoint;

    //used in the _Solve loop
    bool bBalanceGradient;
    NEWMAT::Matrix J;
    NEWMAT::Matrix Jtrans;
    NEWMAT::Matrix Jtemp;
    NEWMAT::Matrix Jtemp2;
    NEWMAT::Matrix Jplus;
    NEWMAT::ColumnVector dx;
    NEWMAT::ColumnVector dx_trans;
    NEWMAT::ColumnVector step;
    NEWMAT::ColumnVector nullspacestep;
    NEWMAT::SymmetricMatrix M;
    NEWMAT::SymmetricMatrix Minv;
    NEWMAT::SymmetricMatrix Mbal;
    NEWMAT::SymmetricMatrix Mbalinv;
    NEWMAT::SymmetricMatrix Mbalperp;
    NEWMAT::SymmetricMatrix Mbalperpinv;

    std::vector<Transform> _curtms;
    int _numitr;
    dReal magnitude;
    dReal x_error;
    bool bClearBadJoints;
    std::vector<dReal> q_s_old;
    std::vector<int> badjointinds;
    bool bLimit;
    dReal prev_error;
    dReal epsilon;
    dReal maxstep; 
    dReal stepsize;


    //used for computing the Jacobian
    NEWMAT::Matrix _Jp;
    NEWMAT::Matrix _Jp0;

    NEWMAT::Matrix _Jr;
    NEWMAT::Matrix _Jr0;
    NEWMAT::Matrix _Jr_proper;

    NEWMAT::Matrix _Jr_quat;

    NEWMAT::Matrix _tasktm;
    NEWMAT::Matrix _E_rpy;
    NEWMAT::Matrix _E_rpy_inv;

    TransformMatrix _TMtask;

    dReal _psi, _theta, _phi;
    dReal Cphi,Ctheta,Cpsi,Sphi,Stheta,Spsi;

    Vector vcurquat;
    NEWMAT::ColumnVector curquat;
    NEWMAT::Matrix angveltoquat;

    //used in transform difference
    Transform _tmtemp;
    dReal _sumsqr,_sumsqr2;
    Vector _q1, _q2;

    //used in QuatToRPY
    dReal a,b,c,d;
    static Vector RPYIdentityOffsets[8];
    dReal _min_dist;
    Vector _temp_vec;
    dReal _temp_dist;

    NEWMAT::ColumnVector q_limits;

    std::vector<boost::shared_ptr<void> > graphptrs;

    //used in balancing
    std::vector<dReal> _vsupportpolyx;
    std::vector<dReal> _vsupportpolyy;

    //for inverse conditioning
    NEWMAT::DiagonalMatrix _S;
    NEWMAT::Matrix _V;


    bool _SolveStopAtLimits(std::vector<dReal>& q_s);



    dReal movementlimit;

};



#endif
