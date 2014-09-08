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
/** \file cbirrtparameters.h
    \brief Child of the plannerparameters class specialized for passing parameters to cbirrt planner.
 */
#ifndef  CPARAMETERS_H
#define  CPARAMETERS_H

/// class for passing parameters to cbirrt planner
class CBirrtParameters : public PlannerBase::PlannerParameters
{
public:
    CBirrtParameters() : bgrabbed(false), Psample(0), bsmoothpath(true), smoothingitrs(-1), bsamplingstart(false), bsamplinggoal(false), timelimit(-1.0), bProcessing(false), bikfastsinglesolution(true)
    {
        _vXMLParameters.push_back("tsrchain");
        _vXMLParameters.push_back("grabbed");
        _vXMLParameters.push_back("samplingstart");
        _vXMLParameters.push_back("samplinggoal");
        _vXMLParameters.push_back("psample");
        _vXMLParameters.push_back("bsmoothpath");
        _vXMLParameters.push_back("smoothingitrs");
        _vXMLParameters.push_back("timelimit");
        _vXMLParameters.push_back("tattachedik_0");
        _vXMLParameters.push_back("supportpolyx");
        _vXMLParameters.push_back("supportpolyy");
        _vXMLParameters.push_back("ikguess");
        _vXMLParameters.push_back("bikfastsinglesolution");
        _vXMLParameters.push_back("pplannerstate");

    }
    bool bgrabbed; ///< are we grabbing an object?

    bool bsamplingstart; ///< are there TSRs specified for start sampling
    bool bsamplinggoal; ///< are there TSRs specified for goal sampling

    dReal Psample; ///< probability of sampling for a WGR at a planner iterations
    dReal bsmoothpath; ///< whether to smooth the path or not

    int smoothingitrs; ///< how many iterations of smoothing to apply to the final trajectory
    dReal timelimit;  ///< maximum planning time in seconds

    std::vector<Transform> Tattachedik_0; ///< converts between attachedik frame and 0 frame (default is identity)

    std::vector<dReal> vsupportpolyx; ///< x coords of support polygon
    std::vector<dReal> vsupportpolyy; ///< y coords of support polygon

    std::vector<TaskSpaceRegionChain> vTSRChains; ///< vector of Task-Space Region Chains

    std::vector<dReal> vikguess; ///< an optional joint value vector to use for the itertive IK solver when sampling goals
    bool bikfastsinglesolution; ///< an optional parameter which chooses between using a single solution from IKFast or using multiple solutions (default is true = single solution)

    enum PlannerState * pplannerstate;

protected:
    bool bProcessing;

    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;


        for(int i =0; i < vTSRChains.size();i++)
        {
            O << "<tsrchain>";
            if(!vTSRChains[i].serialize(O))
                return false;
            O << "</tsrchain>"<<endl;            
        }


        O << "<grabbed>" << bgrabbed << "</grabbed>" << endl;

        O << "<samplingstart>" << bsamplingstart << "</samplingstart>" << endl;

        O << "<samplinggoal>" << bsamplinggoal << "</samplinggoal>" << endl;

        O << "<psample>" << Psample << "</psample>" << endl;

        O << "<bsmoothpath>" << bsmoothpath << "</bsmoothpath>" << endl;
        O << "<smoothingitrs>" << smoothingitrs << "</smoothingitrs>" << endl;
        O << "<timelimit>" << timelimit << "</timelimit>" << endl;

        O << "<tattachedik_0>" << Tattachedik_0.size() << " ";
        for(int i = 0; i < Tattachedik_0.size(); i++)
        {
            O << Tattachedik_0[i].rot.w << " ";
            O << Tattachedik_0[i].rot.x << " ";
            O << Tattachedik_0[i].rot.y << " ";
            O << Tattachedik_0[i].rot.z << " ";
            O << Tattachedik_0[i].trans.x << " ";
            O << Tattachedik_0[i].trans.y << " ";
            O << Tattachedik_0[i].trans.z << " ";
        }
        O << "</tattachedik_0>" << endl;


        O << "<supportpolyx>";
        for(int i = 0; i < vsupportpolyx.size(); i++)
            O << vsupportpolyx[i]<< " ";
        O << "</supportpolyx>" << endl;

        O << "<supportpolyy>";
        for(int i = 0; i < vsupportpolyy.size(); i++)
            O << vsupportpolyy[i]<< " ";
        O << "</supportpolyy>" << endl;

        O << "<ikguess>";
        for(int i = 0; i < vikguess.size(); i++)
            O << vikguess[i]<< " ";
        O << "</ikguess>" << endl;

        O << "<bikfastsinglesolution>" << bikfastsinglesolution << "</bikfastsinglesolution>" << endl;
        
        unsigned long int uli;
        uli = (unsigned long int)pplannerstate;
        O << "<pplannerstate>" << uli << "</pplannerstate>" << endl;

        return !!O;
    }

    ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
    {
        if( bProcessing )
        {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
        }
        
        bProcessing = (name == "tsrchain" || 
                       name == "grabbed" || 
                       name == "samplingstart" || 
                       name == "samplinggoal" || 
                       name == "psample" || 
                       name == "bsmoothpath" || 
                       name == "smoothingitrs" || 
                       name == "timelimit" || 
                       name == "tattachedik_0" || 
                       name == "supportpolyx" || 
                       name == "supportpolyy" || 
                       name == "ikguess" ||
                       name == "bikfastsinglesolution" ||
                       name == "pplannerstate");

        return bProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const std::string& name)
    {
        if (bProcessing)
        {

            if( stricmp(name.c_str(), "tsrchain") == 0 )
            {        
                TaskSpaceRegionChain tmp;
                tmp.deserialize(_ss);
                vTSRChains.push_back(tmp);
            }
            else if( stricmp(name.c_str(), "psample") == 0 )
            {        
                _ss >> Psample;
            }
            else if( stricmp(name.c_str(), "bsmoothpath") == 0 )
            {        
                _ss >> bsmoothpath;
            }
            else if( stricmp(name.c_str(), "smoothingitrs") == 0 )
            {
                _ss >> smoothingitrs;
            }
            else if( stricmp(name.c_str(), "timelimit") == 0 )
            {
                _ss >> timelimit;
            }
            else if( stricmp(name.c_str(), "tattachedik_0") == 0 )
            {
                int size;
                _ss >> size;
                Tattachedik_0.resize(size);
                for(int i = 0; i < size; i++)
                {
                    _ss >> Tattachedik_0[i].rot.w;
                    _ss >> Tattachedik_0[i].rot.x;
                    _ss >> Tattachedik_0[i].rot.y;
                    _ss >> Tattachedik_0[i].rot.z;
                    _ss >> Tattachedik_0[i].trans.x;
                    _ss >> Tattachedik_0[i].trans.y;
                    _ss >> Tattachedik_0[i].trans.z;
                }
            }
            else if( stricmp(name.c_str(), "supportpolyx") == 0 )
            {
                vsupportpolyx.clear();
                dReal f;
                while(1) {
                    _ss >> f; 
                    if( !_ss )
                        break; // at the end of the stream, so break
                    vsupportpolyx.push_back(f);
                }

            }
            else if( stricmp(name.c_str(), "supportpolyy") == 0 )
            {
                vsupportpolyy.clear();
                dReal f;
                while(1) {
                    _ss >> f; 
                    if( !_ss )
                        break; // at the end of the stream, so break
                    vsupportpolyy.push_back(f);
                }

            }
            else if( stricmp(name.c_str(), "ikguess") == 0 )
            {
                vikguess.clear();
                dReal f;
                while(1) {
                    _ss >> f; 
                    if( !_ss )
                        break; // at the end of the stream, so break
                    vikguess.push_back(f);
                }

            }
            else if( stricmp(name.c_str(), "grabbed") == 0 )
            {
                _ss >> bgrabbed;
            }
            else if( stricmp(name.c_str(), "samplingstart") == 0 )
            {
                _ss >> bsamplingstart;
            }
            else if( stricmp(name.c_str(), "samplinggoal") == 0 )
            {
                _ss >> bsamplinggoal;
            }
            else if( stricmp(name.c_str(), "bikfastsinglesolution") == 0 )
            {
                _ss >> bikfastsinglesolution;
            }
            else if( stricmp(name.c_str(), "pplannerstate") == 0 )
            {
                unsigned long int uli;
                _ss >> uli;
                pplannerstate = (enum PlannerState *)uli;
            }
            else
            {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            bProcessing = false;
            return false;
        }

        return PlannerParameters::endElement(name);
    }
};

#endif
