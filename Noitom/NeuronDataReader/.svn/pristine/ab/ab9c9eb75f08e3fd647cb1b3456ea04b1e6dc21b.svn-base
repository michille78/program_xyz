#pragma once

class IDataParseDelegate
{
public:
    IDataParseDelegate(){};
    virtual ~IDataParseDelegate(){};

    /// <summary>
    /// Parse the input data. callback and output frame data by event
    /// </summary>
    /// <param name="data">Data.</param>
    /// <param name="len">Length.</param>
	virtual void Parse(void* sender, unsigned char* data, int len) = 0;
};