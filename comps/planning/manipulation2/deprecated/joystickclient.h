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

#ifndef OPENRAVE_JOYSTICKCLIENT_H
#define OPENRAVE_JOYSTICKCLIENT_H

#define XML_JOYSTICKTRACKING "joystickserver"

class JoystickClient
{
public:

   

    JoystickClient();
    virtual ~JoystickClient();

    virtual bool Init(const char* hostname, int port);
    virtual void Destroy();
#ifdef USING_PLAYER
    Vector GetJoystickXYZVelocity(){return(Vector((float)_pposition3dserver->GetXSpeed(), (float)_pposition3dserver->GetYSpeed(), (float)_pposition3dserver->GetZSpeed()));}

    Vector GetJoystickRPYVelocity(){return(Vector((float)_pposition3dserver->GetRollSpeed(), (float)_pposition3dserver->GetPitchSpeed(), (float)_pposition3dserver->GetYawSpeed()));}

    int GetJoystickMode(){return _pposition3dserver->GetStall();}

    int GetSnapBack(){return (int)_pposition3dserver->GetXPos();}

    float GetElbowBend(){return (float)_pposition3dserver->GetRoll();}
#else
    Vector GetJoystickXYZVelocity(){return Vector();}

    Vector GetJoystickRPYVelocity(){return Vector();}

    int GetJoystickMode(){return 0;}
    int GetSnapBack(){return 0;}
    float GetElbowBend(){return 0;}
#endif



#ifdef USING_PLAYER
private:

    string _strhost;
    int _port;
    bool bInit;

    PlayerClient* _pclient;
    Position3dProxy* _pposition3dserver;

    uint8_t _mode; ///< internal player delivery mode
    boost::thread* _pThread;
    bool _bStopThread; // notification from main thread to reader thread
    bool _bDestroyThread; // notification from reader thread to main thread
    void StartThread();
    void RunThread();
    void StopThread();
#endif

};

#endif
