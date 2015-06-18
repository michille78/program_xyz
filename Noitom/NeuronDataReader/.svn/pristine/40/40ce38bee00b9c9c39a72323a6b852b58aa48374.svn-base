#pragma once
#include "../NeuronDataReader.h"

class CRCCode
{
private:
	CRCCode(void);
	~CRCCode(void);
	
	static BYTE crctable08[256];
	static USHORT crctable16[256];

public:
	static inline BYTE GetCRC8(BYTE* buff, int len)
	{
		BYTE crc = 0;
		int index = 0;
		while (index < len)
			crc = crctable08[crc ^ buff[index++]];
		return crc;
	}
	static inline unsigned short GetCRC16(BYTE* buff, int len)
	{
		unsigned short crc = 0;
		while (len-- > 0)
			crc = crctable16[(crc >> 8 ^ *buff++) & 0xff] ^ (crc << 8);
		return ~crc;
	}
};

