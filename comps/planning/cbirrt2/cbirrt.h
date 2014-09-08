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
/** \file cbirrt.h
    \brief Defines the cbirrt planner class.
 */
#ifndef  CBIRRT_PLANNER_H
#define  CBIRRT_PLANNER_H

#include <sstream>

void  PrintMatrix(dReal* pMatrix, int numrows, int numcols, const char * statement);

/// class for nodes in search trees, contains configurations and other information
class RrtNode {
public:

    RrtNode(int size,bool bFromGoal,int id) { Reset(); SetSize(size);_bFromGoal = bFromGoal;_iID = id;}
    ~RrtNode() {}

    // Methods
    inline void SetSize(int num) { _data.resize(num); }
    inline void Reset() { _iParent = -1; _bFromGoal = false; }
    
    inline void SetConfig(const dReal *pConfig) { memcpy(&_data[0], pConfig, _data.size()*sizeof(_data[0])); }
    inline void SetParent(int parent){_iParent = parent;}
    inline int GetID(){return _iID;}
    inline dReal* GetData() { return &_data[0]; }
    inline vector<dReal>* GetDataVector() { return &_data; }
    inline int GetParent() { return _iParent; }

    inline bool GetFromGoal() { return _bFromGoal; }
    inline void SetFromGoal(bool newVal) { _bFromGoal = newVal; }

    inline void Print() {
        if( RaveGetDebugLevel() ) {
            stringstream s;
            s << "ID: " << _iID << " Parent: " << _iParent << " ";
            for(unsigned int i = 0; i < _data.size();i++) {
                s << _data[i]<<" ";
            }
            s << "\n";
            RAVELOG_DEBUG(s.str().c_str());
        }
    }

    inline void SetTSRChainValues(std::vector<dReal>& vTSRChainValues_in) {vTSRChainValues = vTSRChainValues_in;}
    inline std::vector<dReal> GetTSRChainValues(){return vTSRChainValues;} //THIS MUST RETURN A COPY, NOT A REFERENCE

private:
    // Data
    int        _iParent;
    vector<dReal> _data;
    bool       _bFromGoal;
    int        _iID;

    std::vector<dReal> vTSRChainValues;
};


/// class for the cbirrt planner
class CBirrtPlanner : public PlannerBase
{
public:
    CBirrtPlanner(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        __description = ":Interface Author: Dmitry Berenson\nRRT-based algorithm for planning with end-effector pose constraints described by Task Space Region Chains.\n\n`C++ Documentation <http://automation.berkeley.edu/~berenson/docs/cbirrt/index.html>`_";
        _pActiveNode = NULL;
        _pConnectNode = NULL;

        //these shouldn't be de-allocated so that multiple calls of this planner are faster
        _pForwardTree = new NodeTree(false);
        _pBackwardTree = new NodeTree(true);

        sIgnoredBodies.clear();
        sIgnoredLinks.clear();

        RegisterCommand("GetOutputMessage",boost::bind(&CBirrtPlanner::GetOutputMessage,this,_1,_2),"returns an ostream containing any errors or other information put there by the planner");

    }

    bool GetOutputMessage(std::ostream& os, std::istream& is)
    {
        os << _outputstream.str();
        return !!os;
    }


    /// evaluates distance between two configurations of the robot
    class CBirrtDistanceMetric
    {
    public:
        CBirrtDistanceMetric(CBirrtPlanner* pplanner_in){_pplanner = pplanner_in;}
        ~CBirrtDistanceMetric(){}

        /// evaluates distance between two configurations of the robot
        virtual dReal Eval(const void* c0, const void* c1);

    protected:
        CBirrtPlanner* _pplanner;
    };


    class NodeTree;

    /// class for extension operations, most of the work in the planner happens here
    class MakeNext
    {
    public:
        MakeNext(bool bFromGoal, int numdof, RobotBasePtr  robot, CBirrtPlanner * planner);

        ~MakeNext(){}

        bool MakeNodes(int TreeSize, RrtNode* StartNode, std::vector<dReal>* targetConfig,std::vector<RrtNode>* voutput,CBirrtPlanner* planner, bool bAllowGoPast); ///< main extension function
        bool ConstrainedNewConfig(std::vector<dReal>& q_s, std::vector<dReal>& q_near, std::vector<dReal>& vTSRChainValues, bool bCheckDistance = true, std::vector<Transform>* pvTtarg = NULL, bool bStartvsGoalSampling = false); ///< function that projects and/or rejects a configuration

        bool AddRootConfiguration(CBirrtPlanner::NodeTree* ptree,std::vector<dReal>& guess); ///< add a root configuration to the NodeTree
        int SampleTSRChainIndex(const std::vector<int>& TSRChaininds); ///< Pick a TSR Chain index to draw a sample from
        
        bool CheckSupport(bool bDraw = false); ///< function to check balance
       
        EnvironmentBasePtr GetEnv(){ return _planner->GetEnv();} ///< return the planner's environment

    private:
        
        bool _bFromGoal;
        RobotBasePtr  _probot;
        CBirrtPlanner * _planner;
        std::vector<dReal> _lowerLimit;
        std::vector<dReal> _upperLimit;
        //variables for tree extension
        dReal* startConfig;
        bool bExtendConnect;
        unsigned int numDOF;
        bool success;
        dReal normDiff;
        dReal configDiff;
        dReal normDist;
        bool targetReached;

        //these are used in the extension calculation
        std::vector<dReal> diff;
        std::vector<dReal> newConfig; 
        std::vector<dReal> oldConfig; 
        std::vector<dReal> dist;
        std::vector<dReal> oldoldConfig;
        std::vector<dReal> ancientConfig;
        std::vector<dReal> lastcollchecked;
        
        struct timeval start_time, end_time; ///< for measuring hold long different parts of the algorithm take
    };


    /// class that stores the search trees, each tree contains a vector of nodes, nodes point to their parents using their index in the vector
    class NodeTree
    {

    public:

        NodeTree(bool bFromGoal){_bFromGoal = bFromGoal; _pMakeNext = NULL; _vnodes.reserve(1000);}
        ~NodeTree(){};


        RrtNode* GetNode(const int index) {assert((index >= 0) && ((unsigned int)index < _vnodes.size()));return &_vnodes[index];} ///< get a node by its index in _vnodes
        RrtNode* GetRandomNode() {return &_vnodes[RANDOM_INT((int)_vnodes.size())];} ///< get a random node in the tree
        void AddNode(RrtNode node){_vnodes.push_back(node);} ///< add a node to the tree
        void AddNodes(std::vector<RrtNode>* nodes){for(unsigned int i=0;i < nodes->size();i++) _vnodes.push_back(nodes->at(i));} ///< add multiple nodes to the tree
        int GetSize() const {return (int)_vnodes.size();} ///< get the size of the tree
        bool GetFromGoal() const {return _bFromGoal;} ///< returns 1 if this tree is a goal tree, 0 otherwise
        void DeleteNodes() {_vnodes.clear(); _vnodes.reserve(1000);} ///< delete all nodes in the tree

        int _iConnected;
        MakeNext* _pMakeNext;
        

    private:
        bool _bFromGoal;
        std::vector<RrtNode> _vnodes;

    };

    
    ~CBirrtPlanner();


    bool InitPlan(RobotBasePtr  pbase, PlannerParametersConstPtr pparams); ///< load the parameters into the planner
    OpenRAVE::PlannerStatus PlanPath(TrajectoryBasePtr ptraj); ///< plan a path
    OpenRAVE::PlannerStatus CleanUpReturn(bool retval); ///< cleanup memory and return
    
    int GetNumDOF(){return _numdofs;} ///< get the number of DOF used by the planner
    RobotBasePtr  GetRobot() { return _pRobot; } ///< get the robot the planner is planning for

    // interval types   ( , )      ( , ]       [ , )      [ , ]
    enum IntervalType { OPEN=0,  OPEN_START,  OPEN_END,  CLOSED }; ///< types of collision-checking intervals

    vector<RrtNode> *GetPath() { return &vecpath; } ///< get the path

    /// set dof values for the robot and any bodies mimiced in the TSR chains
    void SetDOF(std::vector<dReal>& dofvals){
        //PrintMatrix(dofvals,GetNumDOF(),1," cfg: ");
        std::vector<dReal> tempvec = dofvals;
        tempvec.resize(_pRobot->GetActiveDOF());
        _pRobot->SetActiveDOFValues(tempvec);
        //now set the mimic DOF of the TSR chains
        for(int i = 0; i < _parameters->vTSRChains.size(); i++)
        {
            //it's ok if vTSRChainIndexToConfigurationOffset[i] == 0, it will be ignored
            _parameters->vTSRChains[i].ApplyMimicValuesToMimicBody(&dofvals[vTSRChainIndexToConfigurationOffset[i]]);

        }

    }

    /// get the parameters used by the planner
    //virtual PlannerParametersConstPtr GetParameters() const { return boost::shared_ptr<const OpenRAVE::PlannerBase::PlannerParameters>(&_parameters); }
    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }

    /// set the current path in the planner to a given trajectory (used if you only want to smooth the path, not plan)
    bool SetVecPathFromTraj(TrajectoryBasePtr ptraj_in);
    bool _OptimizePath(bool &bTerminated, double starttime); ///< path smoothing/optimization is done here, if the planning is terminated from outside (not because of time limit), this will be set to true
    bool _CreateTraj(TrajectoryBasePtr traj); ///< make a trajectory from the path, trajectories for mimiced bodies are also created here

private:

    // Methods
    inline RrtNode* _NewNode();
    inline bool _CheckCollision(std::vector<dReal>& pQ0, std::vector<dReal>& pQ1, IntervalType interval); ///< wrapper for checking collision along an interval
    inline bool _CheckCollision(std::vector<dReal>& pConfig); ///< core collision-checking function
    inline void _PickRandomConfig(); ///< generate a random configuration
    inline int _FindClosest(NodeTree* pNodeTree); ///< get the closest node in the tree to a random configuration
    inline bool _JoinPathHalves(); ///< join the path segments from the start and goal trees

    
    dReal GetPathLength(std::vector<RrtNode>& path, int indexstart, int indexend); ///< get the length of a sub-segment of a path
    void ClearTSRChainValues(){for(int i =0; i < vTSRChainValues_temp.size(); i++) vTSRChainValues_temp[i] = 0.0;} ///< clear a TSR Chain

    // Data
    RobotBasePtr _pRobot; ///< the robot the planner is planning for
    RrtNode* _pActiveNode;
    RrtNode* _pConnectNode;


    std::vector<dReal> _pInitConfig;  //!< initial and goal configs
    std::vector<dReal> _pGoalConfig;


    vector<RrtNode> vecpath; ///< the path found by the algorithm
    
    std::vector<dReal> _randomConfig;

    std::vector<dReal> _jointResolution;  //!< joint discretization
    std::vector<dReal> _jointResolutionInv;
    std::vector<dReal> _jointIncrement;

    std::vector<dReal> _lowerLimit;  //!< joint limits
    std::vector<dReal> _upperLimit;
    std::vector<bool> _viscircular;
    std::vector<dReal> _validRange;
    std::vector<dReal>  checkConfig;


    boost::shared_ptr<CBirrtParameters> _parameters;
    bool bInit; ///< 1 if the planner is properly initialized, 0 otherwise

    bool bdelete_distmetric;

    NodeTree* _pForwardTree;
    NodeTree* _pBackwardTree;

    double nntime;
    double collisionchecktime;
    double projectiontime;

    int nncalls;
    int collisioncheckcalls;
    int projectioncalls;

    bool bSmoothPath;

    dReal P_SAMPLE_IK;

  
    KinBodyPtr  pheldobject;
    int _numdofs;
    IkSolverBasePtr _pIkSolver;

    ofstream colfile;

    double max_planning_time;
    double max_firstik_time;
    CBirrtDistanceMetric* pdistmetric;
    std::stringstream _outputstream;

    //TSR
    std::vector<std::vector<int > > vvManipToGoalSamplingTSRChainind;
    std::vector<std::vector<int > > vvManipToStartSamplingTSRChainind;
    std::vector<std::vector<int > > vvManipToConstraintTSRChainind;

    std::vector<int> vTSRChainIndexToConfigurationOffset;
    std::vector<int> vTSRChainIndexToGuessOffset;
    std::vector<dReal> vTSRChainValues_temp;
    std::vector<OpenRAVE::KinBodyConstPtr > sIgnoredBodies;
    std::vector<OpenRAVE::KinBody::LinkConstPtr> sIgnoredLinks;

    std::vector<int> vJointIndToActiveInd;

    std::vector<KinBodyPtr> vMimicBodies;


    GraphHandlePtr figure; ///< for plotting
};


#endif   // CBIRRT_PLANNER_H

