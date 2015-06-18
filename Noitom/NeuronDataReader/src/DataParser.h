#pragma once

#include "IDataParseDelegate.hpp"
#include "../NeuronDataReader.h"
#include "SocketClientsManager.h"
#include "CompressInfoCode.h"
#include "CRCCode.h"

class DataParser : public IDataParseDelegate
{
private:	
    // Bvh header
	BvhDataHeaderEx* _bvhHeaderEx;
	float*           _floatValues;
	int              _floatValuesBufferSize;
	int              _bvhHeaderSize;

	CommandPack*     _cmdHeader;
	void*            _cmdDataBuffer;
	int              _cmdDataBufferSzie;
	int              _cmdHeaderSize;
    
    OptDataHeader*   _optHeader;
    int              _optHeaderSize;
    float*           _optDataBuffer;
    int              _optDataBufferSize;

    UINT16           _headTag;

	CompressInfoCode uncomp;	// bvh数据解压对象

	bool           _crcerro;

	inline int parseBVHData(void* sender, unsigned char* data, int len)
	{
		int dataLen = 0;
		// read whole header
		memcpy(_bvhHeaderEx, data, _bvhHeaderSize);

		// check length
		if (_bvhHeaderEx->DataCount <= 0)
		{
			return 0;
		}

		//解压
		if (_bvhHeaderEx->IsCompressed )
		{
			//位置信息List个数
			int listCount = _bvhHeaderEx->Reserved1;
			CompressMoveInfo* movelist = (CompressMoveInfo*)(data + _bvhHeaderSize);

			// 计算压缩后的数据总数
			int srcCount = _bvhHeaderEx->DataCount;
			float* src = (float*)(data + _bvhHeaderSize + listCount*sizeof(CompressMoveInfo));

			// 计算校验码，避免半包情况导致崩溃（TODO：应该考虑拼包）
			unsigned short crc = (_bvhHeaderEx->IsCompressed & 0xffff0000) >> 16;

			if (crc != CRCCode::GetCRC16(data + _bvhHeaderSize, listCount*sizeof(CompressMoveInfo) + _bvhHeaderEx->DataCount*sizeof(float)))
			{
				_crcerro = TRUE;
			}

			if (_crcerro)
			{
				if (0 != _bvhHeaderEx->Reserved1)
				{
					return 0;
				}
                _crcerro = FALSE;
			}

			//返回来数据个数
			int dataCount;
			 
			short Count = 0;
			//算总大小
			for (int i = 0; i < listCount; i++)
			{
				Count += movelist[i].count;
			}

			Count += srcCount;

			if (Count != _floatValuesBufferSize)
			{
				delete[] _floatValues;
				_floatValues = new float[Count];
				_floatValuesBufferSize = Count;
			}

			uncomp.Uncompress(movelist, listCount, src, srcCount, _floatValues, dataCount);

			//重新赋值
			_bvhHeaderEx->DataCount = dataCount;

			dataLen = Count*sizeof(float);
		}
		else
		{
			// data bytes
			dataLen = _bvhHeaderEx->DataCount * sizeof(float);

			// No enough data to read
			if (dataLen > len)
			{
				return 0;
			}

			// check memory length
			if (_bvhHeaderEx->DataCount != _floatValuesBufferSize)
			{
				delete[] _floatValues;

				_floatValues = new float[_bvhHeaderEx->DataCount];
				_floatValuesBufferSize = _bvhHeaderEx->DataCount;
			}

			// copy data
			memcpy(_floatValues, data + _bvhHeaderSize, dataLen);
		}

		// output
		if (SocketClientsManager::fnBvhDataCallback != NULL)
		{
			SocketClientsManager::fnBvhDataCallback(SocketClientsManager::customedObject, sender, _bvhHeaderEx, _floatValues);
		}

		return _bvhHeaderSize + dataLen;
	};

    // 解析OptData数据
    inline int parseOptData(void* sender, unsigned char* data, int len)
    {
        int dataLen = 0;
        // read whole header
        memcpy(_optHeader, data, _optHeaderSize);

        // check length
        if (_optHeader->DataCount <= 0)
        {
            return 0;
        }

        // data bytes
        dataLen = _optHeader->DataCount * sizeof(float);

        // No enough data to read
        if (dataLen > len)
        {
            return 0;
        }

        // check memory length
        if (_optHeader->DataCount > _optDataBufferSize)
        {
            delete[] _optDataBuffer;

            _optDataBuffer = new float[_optHeader->DataCount];
            _optDataBufferSize = _optHeader->DataCount;
        }

        // copy data
        memcpy(_optDataBuffer, data + _optHeaderSize, dataLen);


        // output
        if (SocketClientsManager::fnOptDataCallback != NULL)
        {
            SocketClientsManager::fnOptDataCallback(SocketClientsManager::customedObject, sender, _optHeader, _optDataBuffer);
        }
        return _optHeaderSize + dataLen;
    };


	inline int parseCommandsData(void* sender, unsigned char* data, int len)
	{
		// read whole header
		memcpy(_cmdHeader, data, _cmdHeaderSize);

		// check length
		if (_cmdHeader->DataLength <= 0)
		{
			return 0;
		}

		int dataLen = 0;

		if (_cmdHeader->CommandId == Cmd_BoneSize || _cmdHeader->CommandId == Cmd_BvhInheritanceTxt)
		{
			switch (_cmdHeader->CommandId)
			{
			case Cmd_BoneSize:            // Id used to request bone size from server
				dataLen = _cmdHeader->DataCount * sizeof(CmdResponseBoneSize);
				break;
			case Cmd_BvhInheritanceTxt:
				dataLen = _cmdHeader->DataCount;
				break;
			default:
				return 0;
				break;
			}

			// No enough data to read
			if (dataLen > len)
			{
				return 0;
			}

			// check memory length
			if (dataLen != _cmdDataBufferSzie)
			{
				delete[] _cmdDataBuffer;

				_cmdDataBuffer = new char[dataLen];
				_cmdDataBufferSzie = dataLen;
			}

			// copy data
			memcpy(_cmdDataBuffer, data + _cmdHeaderSize, dataLen);
		}

		// output
		if (SocketClientsManager::fnCommandDataCallback != NULL)
		{
			SocketClientsManager::fnCommandDataCallback(SocketClientsManager::customedObject, sender, _cmdHeader, _cmdDataBuffer);
		}

		return _bvhHeaderSize + dataLen;
	};
public:
    DataParser();
    ~DataParser();

    /// <summary>
    /// Call this port to parse data
    /// </summary>
    /// <param name="data">Socket raw data.</param>
    /// <param name="len">Data length.</param>
	inline void Parse(void* sender, unsigned char* data, int nLen)
    {
        // Parser large pack
		unsigned char* data_start = data;
		unsigned char* data_end = data + nLen;
        int pos = 0;
		while (data_start < data_end)
        {
            // check header
            // tag1
			_headTag = data_start[0] + (data_start[1] << 8);
            if (_headTag == 0xDDFF)
			{
				// tag2
				unsigned char* tmpHeadEndPos = data_start + _bvhHeaderSize - 2;
				_headTag = tmpHeadEndPos[0] + (tmpHeadEndPos[1] << 8);
				if (_headTag == 0xEEFF)
				{
					// parser bvh data
					int len = parseBVHData(sender, data_start, data_end - data_start);
					if (len>0)
					{
						// move to next pack
						data_start += len;
						continue;
					}
				}
			}
			else if (_headTag == 0xAAFF)
			{
				// tag2
				unsigned char* tmpHeadEndPos = data_start + _cmdHeaderSize - 2;
				_headTag = tmpHeadEndPos[0] + (tmpHeadEndPos[1] << 8);
				if (_headTag == 0xBBFF)
				{
					// parser command data
					int len = parseCommandsData(sender, data_start, data_end - data_start);
					if (len>0)
					{
						// move to next pack
						data_start += len;
						continue;
					}
				}
			}
            else if (_headTag == 0xBBFF)
            {
                // tag2
                unsigned char* tmpHeadEndPos = data_start + _cmdHeaderSize - 2;
                _headTag = tmpHeadEndPos[0] + (tmpHeadEndPos[1] << 8);
                if (_headTag == 0xCCFF)
                {
                    // parser command data
                    int len = parseOptData(sender, data_start, data_end - data_start);
                    if (len > 0)
                    {
                        // move to next pack
                        data_start += len;
                        continue;
                    }
                }
            }
			// move to next byte
			data_start += 1;
        }
    };
};