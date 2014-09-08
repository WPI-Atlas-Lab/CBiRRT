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

TemplateStreamParser<float, char> g_floatparser;
StringStreamParser g_stringparser;
WStringStreamParser g_wstringparser;

// VisionTrackingXMLReader
RaveVisionClient::VisionTrackingXMLReader::VisionTrackingXMLReader(VisionTrackingData* pData, const char **atts)
{
    _pData = pData;
    if( _pData == NULL )
        _pData = new VisionTrackingData();
    _pcurparser = NULL;
}

void RaveVisionClient::VisionTrackingXMLReader::startElement(void *ctx, const char *name, const char **atts)
{
    if( stricmp((const char*)name, "modelname") == 0 ) {
        _pcurparser = &g_stringparser;
    }
    else if( stricmp((const char*)name, "offsetlink") == 0 ) {
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "parent") == 0 ) {
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "parentlink") == 0 ) {
        _pcurparser = &g_wstringparser;
    }
    else {
        _pcurparser = &g_floatparser;
    }
}

bool RaveVisionClient::VisionTrackingXMLReader::endElement(void *ctx, const char *name)
{
    float* pf = _pcurparser != NULL ? (float*)_pcurparser->GetData() : NULL;
    
    if( stricmp((const char*)name, "modelname") == 0 ) {
        _pData->modelname = (char*)_pcurparser->GetData();
    }
    else if( stricmp((const char*)name, "offsetlink") == 0 ) {
        _pData->strOffsetLink = (wchar_t*)_pcurparser->GetData();
    }
    else if( stricmp((const char*)name, "parent") == 0 ) {
        _pData->strParentBody = (wchar_t*)_pcurparser->GetData();
    }
    else if( stricmp((const char*)name, "parentlink") == 0 ) {
        _pData->strParentLink = (wchar_t*)_pcurparser->GetData();
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        _pData->transOffset.trans = Vector(pf[0],pf[1],pf[2]);
    }
    else if( stricmp((const char*)name, "pretranslation") == 0 ) {
        _pData->transPreOffset.trans = Vector(pf[0],pf[1],pf[2]);
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        TransformMatrix m;
        memcpy(&m.m[0], pf, 3*sizeof(dReal));
        memcpy(&m.m[4], pf+3, 3*sizeof(dReal));
        memcpy(&m.m[8], pf+6, 3*sizeof(dReal));
        _pData->transOffset.rot = Transform(m).rot;
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        _pData->transOffset.rot = Vector(pf[0],pf[1],pf[2],pf[3]);
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        normalize3(pf, pf);
        dQFromAxisAndAngle(_pData->transOffset.rot, pf[0], pf[1], pf[2], pf[3] * PI / 180.0f);
    }
    else if( stricmp((const char*)name, "prerotationaxis") == 0 ) {
        normalize3(pf, pf);
        dQFromAxisAndAngle(_pData->transPreOffset.rot, pf[0], pf[1], pf[2], pf[3] * PI / 180.0f);
    }
    else if( stricmp((const char*)name, XML_VISIONTRACKING) == 0 ) {
        return true;
    }

    _pcurparser = NULL;
    return false;
}

void RaveVisionClient::VisionTrackingXMLReader::characters(void *ctx, const char *ch, int len)
{
    if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
}

BaseXMLReader* RaveVisionClient::CreateVisionTrackingReader(KinBody* parent, const char **atts)
{
    return new VisionTrackingXMLReader(NULL, atts);
}

//////////////////////
// RaveVisionClient //
//////////////////////

void RaveVisionClient::BODY::Destroy()
{    
    if( !bLock && bEnabled ) {
        g_pEnviron->RemoveKinBody(pbody, true);
        pbody = NULL;
    }
}

RaveVisionClient::RaveVisionClient()
{
    bInit = false;
    _pclient = NULL;
    _pvisionserver = NULL;

    _bStopThread=true;
    _bDestroyThread = false;
    _pThread = NULL;
}

RaveVisionClient::~RaveVisionClient()
{
    Destroy();
}

bool RaveVisionClient::Init(const char* hostname, int port)
{
    Destroy();

    assert( hostname != NULL );

    // init player vision server
    _strhost = hostname;
    _port = port;
    RAVELOG_INFO("RaveVisionClient connecting to [%s:%d]\n", hostname, _port);

    try {
        _pclient = new PlayerClient(hostname, _port);

        // set the timeout to 5s, remove this if not present
        _pclient->SetRequestTimeout(10);
        
        _pvisionserver = new VisionserverProxy(_pclient, 0);
        _pvisionserver->RequestGeometry();

        RAVELOG_DEBUG("RavePlayerClient: starting vision server %s\n", _pvisionserver->GetType().c_str());
    }
    catch(...) {
        RAVELOG_INFO("Error in initializing Player\n");
        Destroy();
        return false;
    }

    // read all previous bodies
    vector<BODY> tempbodies = _vbodies;
    _vbodies.resize(0);

    FOREACH(it, tempbodies) {

        if( it->pbody != NULL ) {
            // preserve transformations
            bool bLock = it->bLock;
            bool bEnabled = it->bEnabled;
            if( !AddKinBody(it->pbody, &it->_initdata) ) {
                RAVELOG_INFO("RavePlayerClient: Failed to read body %S\n", it->pbody->GetName());
            }
            it->bLock = bLock;
            it->bEnabled = bEnabled;
        }
    }
    
    bInit = true;

    _mode = PLAYER_DATAMODE_PULL;
    _pclient->SetRequestTimeout(3);
    _pclient->SetDataMode(_mode);
    StartThread();

    return bInit;
}

void RaveVisionClient::Destroy()
{
    RAVELOG_INFO("RaveVisionClient destroying\n");
    if (!_bStopThread)
        StopThread();

    // close player vision server
    bInit = false;
    _bDestroyThread = false;

    // don't destroy _vbodies, only release player objs
    FOREACH(it, _vbodies)
        it->Destroy();
    delete _pvisionserver; _pvisionserver = NULL;
    delete _pclient; _pclient = NULL;
}

void RaveVisionClient::AddRegisteredBodies(const std::vector<KinBody*>& vbodies)
{
    // go through all bodies in the environment and check for vision data
    FOREACH(itbody, vbodies) {
        RaveVisionClient::VisionTrackingData* pvisiondata = (RaveVisionClient::VisionTrackingData*)((*itbody)->GetExtraInterface(XML_VISIONTRACKING));
        if( pvisiondata != NULL ) {
            
            // compensate for the initial offsetlink transformation
            RaveVisionClient::BODY* p = AddKinBody(*itbody, pvisiondata);
            
            // make sure vision system never deletes it
            if( p != NULL ) {
                assert( p->pOffsetLink != NULL );
                pvisiondata->inittrans = p->pOffsetLink->GetTransform();
                
                p->bLock = true;
            }
        }
    }
}

RaveVisionClient::BODY* RaveVisionClient::AddKinBody(KinBody* pbody, const void* pdata)
{
    if( pdata == NULL || !bInit )
        return NULL;

    // search for the player object
    player_visionserver_object_t* pobjdata=NULL;
    
    _vbodies.push_back(BODY());
    BODY& b = _vbodies.back();
    b.pbody = pbody;

    b.bPresent = true;
    b._initdata = *(const VisionTrackingData*)pdata;
    b.strBodyName = pbody->GetName();
    b.pOffsetLink = pbody->GetLink(b._initdata.strOffsetLink.c_str());
    if( b.pOffsetLink == NULL )
        b.pOffsetLink = pbody->GetLinks().front();

    KinBody* pparent = g_pEnviron->GetKinBody(b._initdata.strParentBody.c_str());
    if( pparent != NULL ) {
        b.pOffsetParentLink = pparent->GetLink(b._initdata.strParentLink.c_str());
        if( b.pOffsetParentLink == NULL ) {
            b.pOffsetParentLink = pparent->GetLinks().front();
            RAVELOG_DEBUG("failed to find parent link %S\n", b.pOffsetParentLink->GetName());
        }
    }
    else RAVELOG_DEBUG("failed to find parent\n");
    
    RAVELOG_DEBUG("Adding MocapBody: %S:%s (link:%S)\n", pbody->GetName(), b._initdata.modelname.c_str(), b.pOffsetLink->GetName());
    return &b;
}

bool RaveVisionClient::RemoveKinBody(KinBody* pbody, bool bDestroy)
{
    if( !bInit )
        return false;

    FOREACH(it, _vbodies) {
        if( it->pbody == pbody ) {
            if( bDestroy )
                it->Destroy();
            _vbodies.erase(it);
            return true;
        }
    }
    
    return false;
}

bool RaveVisionClient::IsBodyPresent(KinBody* pbody)
{
    if( !bInit )
        return false;

    
    FOREACH(it, _vbodies) {
        if( it->pbody == pbody ) {
            //bool bPresent = it->_pplayerobj->GetStatus()>0;
            bool bPresent = it->bPresent;
            return bPresent;
        }
    }
    
    return false;
}

RaveVisionClient::BODY* RaveVisionClient::GetBody(KinBody* pbody)
{
    if( !bInit )
        return NULL;

    
    FOREACH(it, _vbodies) {
        if( it->pbody == pbody ) {
            BODY* pbody = &(*it);
            return pbody;
        }
    }

    return NULL;
}

bool RaveVisionClient::EnableBody(KinBody* pbody, bool bEnable)
{
    BODY* p = GetBody(pbody);
    if( p != NULL ) {
        p->bEnabled = bEnable;
        return true;
    }

    return false;
}

void RaveVisionClient::UpdateBodies()
{
    if( !bInit ) {

        // try to reconnect
       // static u32 basetime = 0;
//        if( timeGetTime()-basetime > 3000 ) {
//            // retry ever 500ms
//            Init(_strhost.c_str(), _port);
//            basetime = timeGetTime();
//        }
//
//        if( !bInit )
            return;
    }

    if( _bDestroyThread ) {
        Destroy();
        return;
    }

    typeof(_vbodies.begin()) itbody;

    list<player_visionserver_object_t> listObjs;
    if( !_pvisionserver->GetObjects(listObjs) )
        return;

    itbody = _vbodies.begin();
    while(itbody != _vbodies.end()) {

        // make sure body is still around
        itbody->pbody = g_pEnviron->GetKinBody(itbody->strBodyName.c_str());
        
        if( itbody->pbody == NULL ) {
            itbody->pOffsetLink = NULL;
        }

        list<player_visionserver_object_t>::iterator itobj = listObjs.end();
        FORIT(itobj, listObjs) {
            if( stricmp(itobj->id.modelname, itbody->_initdata.modelname.c_str()) == 0 && itobj->playerid == itbody->nGlobalId ) {
                break;
            }
        }
        
        if( itobj == listObjs.end() && itbody->bLock && itbody->bEnabled) {
            // search for the first object that matches the type
            FORIT(itobj, listObjs) {
                if( stricmp(itobj->id.modelname, itbody->_initdata.modelname.c_str()) == 0 ) {
                    break;
                }
            }
        }

        if( itbody->pbody != NULL && itobj != listObjs.end() ) {

            if( !itbody->bEnabled ) {
                // remove itobj
                listObjs.erase(itobj);
                ++itbody;
                continue;
            }

            // found, so update
            itbody->bPresent = true;
            itbody->tcur.trans.x = itobj->pose.translation[0];
            itbody->tcur.trans.y = itobj->pose.translation[1];
            itbody->tcur.trans.z = itobj->pose.translation[2];
            itbody->tcur.rot.x = itobj->pose.rotation[0];
            itbody->tcur.rot.y = itobj->pose.rotation[1];
            itbody->tcur.rot.z = itobj->pose.rotation[2];
            itbody->tcur.rot.w = itobj->pose.rotation[3];
            itbody->nGlobalId = itobj->playerid;

            if( itbody->pOffsetLink == NULL )
                itbody->pOffsetLink = itbody->pbody->GetLinks().front();

            if( itbody->pOffsetParentLink == NULL ) {
                KinBody* pparent = g_pEnviron->GetKinBody(itbody->_initdata.strParentBody.c_str());
                if( pparent != NULL ) {
                    itbody->pOffsetParentLink = pparent->GetLink(itbody->_initdata.strParentLink.c_str());
                    if( itbody->pOffsetParentLink == NULL )
                        itbody->pOffsetParentLink = pparent->GetLinks().front();
                }
            }
                        
            // transform with respect to offset link
            TransformMatrix tlink = itbody->pOffsetLink->GetTransform();
            TransformMatrix tbase = itbody->pbody->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * itbody->_initdata.transOffset;
            
            TransformMatrix tfinal = toffset * itbody->tcur * itbody->_initdata.transPreOffset * itbody->_initdata.inittrans;
            
            if( itbody->pOffsetParentLink != NULL ) {
                tfinal = itbody->pOffsetParentLink->GetTransform() * tfinal;
            }

            itbody->pbody->SetTransform(tfinal);

            if( itbody->nFailedCount > 0 ) --itbody->nFailedCount;

            // remove itobj
            listObjs.erase(itobj);
        }
        else if( (!itbody->bLock&&itbody->bEnabled) || itbody->pbody == NULL ) {
            // destroy

            itbody->bPresent = false;
            itbody->nFailedCount++;

            if( itbody->pbody != NULL ) {
                //RAVELOG_INFO("erasing body: %S\n", itbody->strBodyName.c_str());
                g_pEnviron->RemoveKinBody(itbody->pbody, true);
            }
            itbody = _vbodies.erase(itbody);
            continue;
        }
        
        ++itbody;
    }

    // add any new objects
    FOREACH(itobj, listObjs) {

        // try to load the xml
        if( strstr(itobj->id.modelname, "xml") == NULL )
            continue;

        // try to create a KinBody from the model name
        KinBody* pbody = g_pEnviron->CreateKinBody();
        if( !pbody->Init(itobj->id.modelname, NULL) ) {
            delete pbody;
            continue;
        }

        RaveVisionClient::VisionTrackingData* pvisiondata = (RaveVisionClient::VisionTrackingData*)(pbody->GetExtraInterface(XML_VISIONTRACKING));
        if( pvisiondata != NULL ) {

            // give it a unique id
            wstringstream ws; ws << pbody->GetName() << itobj->id.id;
            pbody->SetName(ws.str().c_str());
            
            if( !g_pEnviron->AddKinBody(pbody) ) {
                delete pbody;
                continue;
            }
            
            BODY* pvisionbody = AddKinBody(pbody, pvisiondata);

            if( pvisionbody == NULL ) {
                g_pEnviron->RemoveKinBody(pbody, true);
                continue;
            }

            assert( pvisionbody->pOffsetLink != NULL );
            pvisionbody->_initdata.inittrans = pvisionbody->pOffsetLink->GetTransform();
    
            pvisionbody->tcur.trans.x = itobj->pose.translation[0];
            pvisionbody->tcur.trans.y = itobj->pose.translation[1];
            pvisionbody->tcur.trans.z = itobj->pose.translation[2];
            pvisionbody->tcur.rot.x = itobj->pose.rotation[0];
            pvisionbody->tcur.rot.y = itobj->pose.rotation[1];
            pvisionbody->tcur.rot.z = itobj->pose.rotation[2];
            pvisionbody->tcur.rot.w = itobj->pose.rotation[3];
            pvisionbody->pbody->SetTransform(pvisionbody->tcur);
            pvisionbody->nGlobalId = itobj->playerid; // update the id

            // transform with respect to offset link
            TransformMatrix tlink = pvisionbody->pOffsetLink->GetTransform();
            TransformMatrix tbase = pvisionbody->pbody->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * pvisionbody->_initdata.transOffset;
            
            TransformMatrix tfinal = toffset * pvisionbody->tcur * pvisionbody->_initdata.transPreOffset * pvisionbody->_initdata.inittrans;

            if( pvisionbody->pOffsetParentLink != NULL ) {
                tfinal = pvisionbody->pOffsetParentLink->GetTransform() * tfinal;
            }

            pvisionbody->pbody->SetTransform(tfinal);
        }
    }
}

void RaveVisionClient::StartThread()
{
    assert(NULL == _pThread);
    _pThread = new boost::thread(boost::bind(&RaveVisionClient::RunThread, this));
}

void RaveVisionClient::StopThread()
{
    assert(_pThread);
    _bStopThread = true;
    _pThread->join();
    delete _pThread; _pThread = NULL;
}

// non-blocking
void RaveVisionClient::RunThread()
{
    _bStopThread = false;
    _bDestroyThread = false;

    RAVELOG_INFO("VisionServer: starting thread\n");
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
                RAVELOG_INFO("VisionServer: failed to read from player...\n");
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

RaveVisionClient::RaveVisionClient() { }
RaveVisionClient::~RaveVisionClient() { }
bool RaveVisionClient::Init(const char* hostname, int port) { return false; }
void RaveVisionClient::Destroy() {}
RaveVisionClient::BODY* RaveVisionClient::AddKinBody(KinBody* pbody, const void* pdata) { return NULL; }
bool RaveVisionClient::RemoveKinBody(KinBody* pbody, bool bDestroy) { return false; }
bool RaveVisionClient::IsBodyPresent(KinBody* pbody) { return false; }
bool RaveVisionClient::EnableBody(KinBody* pbody, bool bEnable) { return false; }
RaveVisionClient::BODY* RaveVisionClient::GetBody(KinBody* pbody) { return NULL; }
void RaveVisionClient::UpdateBodies() {}
BaseXMLReader* RaveVisionClient::CreateVisionTrackingReader(KinBody* parent, const char **atts) { return NULL; }
void RaveVisionClient::AddRegisteredBodies(const std::vector<KinBody*>& vbodies) {return;}

#endif
