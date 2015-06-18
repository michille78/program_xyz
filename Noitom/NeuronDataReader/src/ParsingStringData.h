#pragma once

#include "IDataParseDelegate.hpp"
#include "../NeuronDataReader.h"

extern FrameDataReceived fnBvhDataCallback;
extern void* customedObject;

namespace PN_BVHDataReader
{
    class ParsingStringData : public IDataParseDelegate
    {
    private:
        std::mutex mtx;

        // String frame data splitter
        char _splitter[5];
        char _valueSplitter[5];
        char* _truncateData;
        
        float* _floatValues;

		BvhDataHeaderEx _bvhHeader;
        void calculateValueCount();

    public:
        ParsingStringData();
        ~ParsingStringData();

        void WithDisplement(bool yes)
        {
            _bvhHeader.WithDisp = yes;
            calculateValueCount();
        };

        void WithReferencePrefix(bool yes)
        {
            _bvhHeader.WithReference = yes;
            calculateValueCount();
        };


        inline void Parse(char* data, int len)
        {
            mtx.lock();
            {
                char* pos = data;
                char* posEnd = NULL;

                char* dataEnd = pos+len;

                // frame parsing
                while((posEnd = strstr(pos, _splitter))!= NULL && posEnd<dataEnd)
                {
                    int i = 0;
                    char* tmpStr = NULL;
                    char* frameEnd = posEnd;

                    // values parsing
                    while ((tmpStr = strstr(pos, _valueSplitter))!= NULL && tmpStr<frameEnd)
                    {
                        tmpStr[0] = '\0';
                        
                        if(i>1)
                        {
                            _floatValues[i-2] = atof(pos);
                        }
                        else
                        {
                            if(i==0)
                            {
                                // avatar index
                                _bvhHeader.AvatarIndex = atoi(pos);
                            }
                            else
                            {
                                // avatar name
                                sprintf_s((char*)_bvhHeader.AvatarName, sizeof(_bvhHeader.AvatarName), "%s", pos);
                            }
                        }

                        pos = tmpStr + 1;

                        i++;
                    }
                    
                    // output
                    if (fnBvhDataCallback != NULL)
                    {
						fnBvhDataCallback(customedObject, (SOCKET_REF)this, &_bvhHeader, _floatValues);
                    }
                }
            }
            mtx.unlock();
        };
    };
}