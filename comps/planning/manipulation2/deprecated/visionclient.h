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

#ifndef OPENRAVE_VISIONCLIENT_H
#define OPENRAVE_VISIONCLIENT_H

#define XML_VISIONTRACKING "visiontracking"

/// Base class for parsing data. Used mostly for XML readers
class BasicStreamParser
{
public:
    virtual ~BasicStreamParser() {}
    virtual void* Format(const char* pdata, int len) = 0;
    virtual int GetCount() const = 0;
    virtual void* GetData() = 0;

    /// releases the pointer (this might be different from GetData()!)
    /// Make sure to deallocate with delete[]
    virtual void* Release() = 0;
                                    
};

/// T is the type, Format is the char to go into sscanf
template <class T, class U>
class TemplateStreamParser : public BasicStreamParser
{
public:
    virtual ~TemplateStreamParser() {}
    virtual void* Format(const U* _pdata, int len)
    {
        T t;
        data.resize(0);
        std::basic_stringstream<U> ss(_pdata);

        while(!ss.eof() ) {
            ss >> t;
            if( !ss )
                break;
            data.push_back(t);
        }

        return data.size() > 0 ? &data[0] : NULL;
    }
    virtual int GetCount() const { return (int)data.size(); }
    virtual void* GetData() { return data.size() > 0 ? &data[0] : NULL; }
    virtual void* Release()
    {
        if( data.size() == 0 ) return NULL;

        T* temp = new T[data.size()];
        memcpy(temp, &data[0], sizeof(T)*data.size());
        data.resize(0);
        return temp;
    }

private:
    std::vector<T> data;
};

// Parses wide strings
class WStringStreamParser : public BasicStreamParser
{
public:
    WStringStreamParser() { p = NULL; }
    virtual ~WStringStreamParser() { delete[] p; }

    virtual void* Format(const char* pdata, int len)
    {
        delete[] p;
        p = new wchar_t[len+1];
        mbstowcs(p, (const char*)pdata, len);
        p[len] = 0;
        return p;
    }

    virtual int GetCount() const { return 1; }
    virtual void* GetData() { return p; }

    virtual void* Release() { wchar_t* temp = p; p = NULL; return temp; }
private:
    wchar_t* p;
};

/// Parses strings
class StringStreamParser : public BasicStreamParser
{
public:
    StringStreamParser() { p = NULL; }
    virtual ~StringStreamParser() { delete[] p; }

    virtual void* Format(const char* pdata, int len)
    {
        delete[] p;
        p = new char[len+1];
        strncpy(p, pdata, len);
        p[len] = 0;
        return p;
    }

    virtual int GetCount() const { return 1; }
    virtual void* GetData() { return p; }

    virtual void* Release() { char* temp = p; p = NULL; return temp; }
private:
    char* p;
};

class RaveVisionClient : public SensorSystemBase
{
public:

    class VisionTrackingData : public XMLReadable
    {
    public:
        virtual const char* GetXMLId() { return XML_VISIONTRACKING; }

        string modelname;
        Transform transOffset,transPreOffset; // final offset = transOffset * transReturnedFromVision * transPreOffset
        wstring strOffsetLink;
        Transform inittrans;///< initial transformation of KinBody in the virtual environment
        wstring strParentBody, strParentLink;
    };

    struct BODY : public BODYBASE
    {
        BODY() : BODYBASE(), nFailedCount(0), pOffsetParentLink(NULL) {}
        virtual void Destroy();
        
        VisionTrackingData _initdata; 
        Transform tcur; ///< current transform
        int nFailedCount;
        KinBody::Link* pOffsetParentLink; // transformation of the body is in this body's coordinate system

        friend class RaveVisionClient;
    };

    class VisionTrackingXMLReader : public BaseXMLReader
    {
    public:
        VisionTrackingXMLReader(VisionTrackingData* pData, const char **atts);
        virtual ~VisionTrackingXMLReader() { delete _pData; }
        
        void* Release() { VisionTrackingData* temp = _pData; _pData = NULL; return temp; }
        
    protected:
        virtual void startElement(void *ctx, const char *name, const char **atts);
        
        /// if returns true, XMLReader has finished parsing
        virtual bool endElement(void *ctx, const char *name);
        virtual void characters(void *ctx, const char *ch, int len);
        
        VisionTrackingData* _pData;
        BasicStreamParser* _pcurparser; ///< reads the character streams
    };

    RaveVisionClient();
    virtual ~RaveVisionClient();

    virtual bool Init(const char* hostname, int port);
    virtual void Destroy();

    virtual void AddRegisteredBodies(const std::vector<KinBody*>& vbodies);

    virtual BODY* AddKinBody(KinBody* pbody, const void* pdata);
    virtual bool RemoveKinBody(KinBody* pbody, bool bDestroy=false);
    virtual bool IsBodyPresent(KinBody* pbody);
    virtual bool EnableBody(KinBody* pbody, bool bEnable);
    virtual BODY* GetBody(KinBody* pbody);

    /// Propagate update changes to the real bodies
    virtual void UpdateBodies();

    static BaseXMLReader* CreateVisionTrackingReader(KinBody* parent, const char **atts);

#ifdef USING_PLAYER

    vector<BODY>& GetBodies() { return _vbodies; }

private:

    vector<BODY> _vbodies;

    string _strhost;
    int _port;
    bool bInit;

    PlayerClient* _pclient;
    VisionserverProxy* _pvisionserver;

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
