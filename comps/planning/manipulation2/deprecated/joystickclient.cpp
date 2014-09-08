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

#include "stdafx.h"

#ifdef USING_PLAYER

// boosting
#include <algorithm>
#include <functional>
#include <boost/bind.hpp>

//////////////////////
// JoystickClient //
//////////////////////

JoystickClient::JoystickClient()
{
    bInit = false;
    _pclient = NULL;
    _pposition3dserver = NULL;

    _bStopThread=true;
    _bDestroyThread = false;
    _pThread = NULL;
}

JoystickClient::~JoystickClient()
{
    Destroy();
}

bool JoystickClient::Init(const char* hostname, int port)
{
    Destroy();

    assert( hostname != NULL );

    // init player position3d server
    _strhost = hostname;
    _port = port;
    RAVELOG_INFO("JoystickClient connecting to [%s:%d]\n", hostname, _port);

    try {
        _pclient = new PlayerClient(hostname, _port);

        // set the timeout to 5s, remove this if not present
        _pclient->SetRequestTimeout(10);
        
        _pposition3dserver = new Position3dProxy(_pclient, 0);
        

        RAVELOG_DEBUG("RavePlayerClient: starting joystick server\n");
    }
    catch(...) {
        RAVELOG_INFO("Error in initializing Player\n");
        Destroy();
        return false;
    }

    
    bInit = true;

    _mode = PLAYER_DATAMODE_PULL;
    _pclient->SetRequestTimeout(3);
    _pclient->SetDataMode(_mode);
    StartThread();

    return bInit;
}

void JoystickClient::Destroy()
{
    RAVELOG_INFO("JoystickClient destroying\n");
    if (!_bStopThread)
        StopThread();

    // close player position3d server
    bInit = false;
    _bDestroyThread = false;

 
    delete _pposition3dserver; _pposition3dserver = NULL;
    delete _pclient; _pclient = NULL;
}



void JoystickClient::StartThread()
{
    assert(NULL == _pThread);
    _pThread = new boost::thread(boost::bind(&JoystickClient::RunThread, this));
}

void JoystickClient::StopThread()
{
    assert(_pThread);
    _bStopThread = true;
    _pThread->join();
    delete _pThread; _pThread = NULL;
}

// non-blocking
void JoystickClient::RunThread()
{
    _bStopThread = false;
    _bDestroyThread = false;

    RAVELOG_INFO("Joystick Client: starting thread\n");
    while (!_bStopThread) {

        if( !_bDestroyThread ) {
            try {
                if( _mode == PLAYER_DATAMODE_PUSH) {
                    if (_pclient->Peek()) {
                        _pclient->Read();
                    }
                }
                else {
                    _pclient->Read();
                }
            }
            catch(...) {
                RAVELOG_INFO("Joystick Cleint: failed to read from player...\n");
                _bDestroyThread = true;
            }
        }

        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC);
        // we sleep for 0.001 seconds
        usleep(1000);
    }
}

#else // USING_PLAYER

JoystickClient::JoystickClient() { }
JoystickClient::~JoystickClient() { }
bool JoystickClient::Init(const char* hostname, int port) { return false; }
void JoystickClient::Destroy() {}

#endif
