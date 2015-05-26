#pragma once
class CRC8
{
private:
	CRC8(void);
	~CRC8(void);
	
	static BYTE crctable[256];

public:
    static inline BYTE GetParity(BYTE* buff, int len)
    {
        BYTE crc = 0;
        int index = 0;
        while (index < len)
            crc = crctable[crc ^ buff[index++]];
        return crc;
    }
};

