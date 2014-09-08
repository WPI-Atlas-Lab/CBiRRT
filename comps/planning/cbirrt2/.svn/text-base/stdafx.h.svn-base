/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Authors: Dmitry Berenson and Rosen Diankov <dberenso@cs.cmu.edu and rdiankov@cs.cmu.edu>

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
/** \file stdafx.h
    \brief Includes and function definitions for cbirrt problem/planner.
 */
//This file contains includes of necessary headers and several useful macros and functions
#ifndef BIRRT_PLANNER_STDAFX
#define BIRRT_PLANNER_STDAFX

#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE

#include <assert.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>

#include <string.h>
#include <vector>
#include <list>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

#include <sys/stat.h> //for chmod

using namespace std;
typedef unsigned int u32;

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

inline double timeGetThreadTime()
{
    struct timespec tp;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tp);


    return ((double)tp.tv_sec + (((double)tp.tv_nsec)/1000000000.0));
}

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline unsigned long timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (unsigned long)(t.time*1000+t.millitm);
}

inline float RANDOM_FLOAT()
{
#if defined(__IRIX__)
    return drand48();
#else
    return rand()/((float)RAND_MAX);
#endif
}

inline float RANDOM_FLOAT(float maximum)
{
#if defined(__IRIX__)
    return (drand48() * maximum);
#else
    return (RANDOM_FLOAT() * maximum);
#endif
}

inline int RANDOM_INT(int maximum)
{
#if defined(__IRIX__)
    return (random() % maximum);
#else
    return (rand() % maximum);
#endif
}


#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)

#define FOREACHC FOREACH



#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/(sizeof( (x)[0] )))
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifdef _WIN32

#define WCSTOK(str, delim, ptr) wcstok(str, delim)

// define wcsicmp for MAC OS X
#elif defined(__APPLE_CC__)

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr);

#define strnicmp strncasecmp
#define stricmp strcasecmp

inline int wcsicmp(const wchar_t* s1, const wchar_t* s2)
{
  char str1[128], str2[128];
  sprintf(str1, "%S", s1);
  sprintf(str2, "%S", s2);
  return stricmp(str1, str2);
}


#else

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr)

#define strnicmp strncasecmp
#define stricmp strcasecmp
#define wcsnicmp wcsncasecmp
#define wcsicmp wcscasecmp

#endif

inline std::wstring _ravembstowcs(const char* pstr)
{
    size_t len = mbstowcs(NULL, pstr, 0);
    std::wstring w; w.resize(len);
    mbstowcs(&w[0], pstr, len);
    return w;
}



#include <rave/rave.h>
#include <openrave/planningutils.h>

using namespace OpenRAVE;
extern "C"
{
#include <stdio.h>
#include <stdlib.h>
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>


#include <iostream>
#include <newmat/newmatap.h> 
#include <newmat/newmatio.h> 


enum PlannerState
{
    PS_Idle = 0, ///< the planner is idle
    PS_Planning = 1, ///< the planner is planning
    PS_PlanSucceeded = 2, ///< the planner succeeded and generated a trajectory
    PS_PlanFailed = 3 ///< the planner failed on its own or was terminated from outside
};


#include "TaskSpaceRegion.h"
#include "cbirrtparameters.h"
#include "cbirrtproblem.h"
#include "cbirrt.h"


class DoubleVectorToFloatVector : public vector<float> {
    public:
        DoubleVectorToFloatVector(vector<double> doubleIn)
        {
            resize(doubleIn.size());
            for (size_t i = 0; i < doubleIn.size(); i++)
            {
                at(i) = (float)doubleIn[i];
            }
        }

         DoubleVectorToFloatVector(OpenRAVE::geometry::RaveVector<double>  doubleIn)
        {
            resize(4);
            at(0) = (float)doubleIn.x;
            at(1) = (float)doubleIn.y;
            at(2) = (float)doubleIn.z;
            at(3) = (float)doubleIn.w;
        }

         DoubleVectorToFloatVector(OpenRAVE::geometry::RaveVector<float>  doubleIn)
        {
            resize(4);
            at(0) = (float)doubleIn.x;
            at(1) = (float)doubleIn.y;
            at(2) = (float)doubleIn.z;
            at(3) = (float)doubleIn.w;
        }
};

class RaveDoubleVectorToRaveFloatVector : public RaveVector<float> {
    public:
        RaveDoubleVectorToRaveFloatVector(RaveVector<double> doubleIn)
        {
            x = (float)doubleIn.x;
            y = (float)doubleIn.y;
            z = (float)doubleIn.z;
            w = (float)doubleIn.w;
        }
};

//for plotting
extern std::vector<GraphHandlePtr> graphptrs;

#endif

