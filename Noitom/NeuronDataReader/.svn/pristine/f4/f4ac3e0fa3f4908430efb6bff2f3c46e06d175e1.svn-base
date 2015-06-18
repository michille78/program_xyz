#pragma once

#include "IDataParseDelegate.hpp"
#include "../BVHDataReader.h"
#include "SocketClientsManager.h"

namespace PN_BVHDataReader
{
    class ParsingBinaryData : public IDataParseDelegate
    {
    private:

		std::mutex mtx;
        
		// Bvh header
        BvhDataHeader* _bvhHeader;
        float* _floatValues;
        int    _floatValuesBufferSize;
		int _headerSize;
        UINT16 _headTag;
        char avatarName[32];

        bool _bWithReferencePrefix;
        bool _bWithDisplement;

    public:
        ParsingBinaryData();
        ~ParsingBinaryData();

        /// <summary>
        /// Call this port to parse data
        /// </summary>
        /// <param name="data">Socket raw data.</param>
        /// <param name="len">Data length.</param>
		inline void Parse(void* sender, char* data, int len)
        {
			mtx.lock();

            // Too small, throw away
			if (len < _headerSize) return;

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
				_headTag = (udata + pos + _headerSize - 2)[0] + ((udata + pos + _headerSize - 2)[1] << 8);
                if (_headTag != 0xEEFF)
                {
                    // move index
                    pos++;
                    continue;
                }

                // read whole header
                memcpy(_bvhHeader, udata+pos, _headerSize);

                // check length
                if (_bvhHeader->DataCount <= 0)
                {
                    pos++;
                    continue;
                }

                // No enough data to read, throw away
                if (_bvhHeader->DataCount > (len - pos)/sizeof(float)) return;

                // check memory length
                if (_bvhHeader->DataCount != _floatValuesBufferSize)
                {                    
                    delete[] _floatValues;

                    _floatValues = new float[_bvhHeader->DataCount];
                    _floatValuesBufferSize = _bvhHeader->DataCount;
                }

                // update position
                pos += _headerSize;

                int dataLen = _floatValuesBufferSize*sizeof(float);

                // copy data
                memcpy(_floatValues, udata+pos, dataLen);
                
                // update position
                pos += dataLen;

                // output
				if (SocketClientsManager::fnBvhDataCallback != NULL)
                {
					SocketClientsManager::fnBvhDataCallback(SocketClientsManager::customedObject, sender, _bvhHeader, _floatValues);
                }
            }

			mtx.unlock();
        };
    };
}