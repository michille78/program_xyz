#pragma once
#include "IDataParseDelegate.hpp"
#include "../BVHDataReader.h"

using namespace PN_BVHDataReader;


typedef struct _CalculationDataHeader
{
    UINT16 HeaderToken1;     // Package start token: 0xDDFF
    DATA_VER DataVersion;    // Version of community data format. e.g.: 1.0.0.2
    UINT32 DataCount;        // Values count
    UINT32 AvatarIndex;      // Avatar index
    UCHAR  AvatarName[32];   // Avatar name
    UINT32 DataStreamType;   // Reserved, only enable this package has 64bytes length
    UINT32 Reserved2;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved3;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved4;        // Reserved, only enable this package has 64bytes length
    UINT16 HeaderToken2;     // Package end token: 0xEEFF
}CalculationDataHeader;


//extern FrameCalculationDataReceived fnCalculationDataCallback;
extern void* customedObject;
class ParsingCalculationData: public IDataParseDelegate
{
private:
    // calculationData header
    CalculationDataHeader* _calculationHeader;
    float* _floatValues;
    int    _floatValuesBufferSize;
    
    int    _headSize;
    UINT16 _headTag;
    int    _headerLen;

    char   avatarName[32];
    
public:
    ParsingCalculationData(void);
    virtual ~ParsingCalculationData(void);

    /// <summary>
    /// Call this port to parse data
    /// </summary>
    /// <param name="data">Socket raw data.</param>
    /// <param name="len">Data length.</param>
    inline void Parse(char* data, int len)
    {
        // Too small, throw away
        if (len < _headSize) return;

        // to unsiged char
        unsigned char* udata = (unsigned char*)data;

        // Splite large pack
        int pos = 0;
        while (pos < len)
        {
            // check header
            // tag1
            _headTag = (udata + pos)[0] + ((udata + pos)[1]<<8);
            if (_headTag != 0xDDFF)
            {
                // move index
                pos++;
                continue;
            }

            // tag2
            _headTag = (udata + pos + _headSize - 2)[0] + ((udata + pos + _headSize - 2)[1]<<8);
            if (_headTag != 0xEEFF)
            {
                // move index
                pos++;
                continue;
            }

            // read whole header
            memcpy(_calculationHeader, udata+pos, _headerLen);

            // check length
            if (_calculationHeader->DataCount <= 0)
            {
                pos++;
                continue;
            }

            // No enough data to read, throw away
            if (_calculationHeader->DataCount > (len - pos)*sizeof(float)) return;

            // check memory length
            if (_calculationHeader->DataCount != _floatValuesBufferSize)
            {                    
                delete[] _floatValues;

                _floatValues = new float[_calculationHeader->DataCount];
                _floatValuesBufferSize = _calculationHeader->DataCount;
            }

            // update position
            pos += _headerLen;

            int dataLen = _floatValuesBufferSize*sizeof(float);

            // copy data
            memcpy(_floatValues, udata+pos, dataLen);

            // update position
            pos += dataLen;

            //// output
            //if (fnCalculationDataCallback != NULL)
            //{
            //    fnCalculationDataCallback(customedObject, _calculationHeader, _floatValues);
            //}
        }
    };

};

