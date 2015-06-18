#include "stdafx.h"
#include "ParsingStringData.h"

namespace PN_BVHDataReader
{
    ParsingStringData::ParsingStringData()
    {
        // String frame data splitter
        sprintf_s(_splitter, sizeof(_splitter), "%s", "||");
        sprintf_s(_valueSplitter, sizeof(_valueSplitter), "%s", " ");

        _truncateData = NULL;
            
        _bvhHeader.WithDisp = true;
        _bvhHeader.WithReference = true;
        _bvhHeader.Token1 = 0xDDFF;
        _bvhHeader.Token2 = 0xEEFF;
            
        calculateValueCount();
    }

    ParsingStringData::~ParsingStringData()
    {
        if(_floatValues)
        {
            delete[] _floatValues;
            _floatValues = NULL;
        }
    }


    void ParsingStringData::calculateValueCount()
    {
        mtx.lock();

        int len = 0;

        if (_bvhHeader.WithDisp)
        {
            len = 59 * 6;
        }
        else
        {
            len = 59 * 3 + 3;
        }

        if (_bvhHeader.WithReference)
        {
            len += 6;
        }

        // float buffer
        if(_bvhHeader.DataCount != len)
        {
            _bvhHeader.DataCount = len; // 59 * 3 + 3 + 1

            delete[] _floatValues;

            _floatValues = new float[_bvhHeader.DataCount];
        }

        mtx.unlock();
    }
}