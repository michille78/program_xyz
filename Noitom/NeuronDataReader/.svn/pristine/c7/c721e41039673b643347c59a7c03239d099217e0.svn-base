#include "stdafx.h"
#include "DataParser.h"

DataParser::DataParser()
{

    _bvhHeaderSize = sizeof(BvhDataHeaderEx);
    _floatValuesBufferSize = 180;
    _floatValues = new float[_floatValuesBufferSize];
    _bvhHeaderEx = new BvhDataHeaderEx();
    _bvhHeaderEx->Token1 = 0xDDFF;
    _bvhHeaderEx->Token2 = 0xEEFF;

    // Ô¤·ÖÅä5¸öµã
    _optDataBufferSize = 5 * 3 * sizeof(float);
    _optDataBuffer = new float[];
    _optHeaderSize = sizeof(OptDataHeader);
    _optHeader = new OptDataHeader();

	_cmdDataBuffer = NULL;
	_cmdDataBufferSzie = 0;
	_cmdHeader = new CommandPack();
	_cmdHeaderSize = sizeof(CommandPack);

	_crcerro = FALSE;
}


DataParser::~DataParser()
{
	delete _cmdHeader;
    delete _bvhHeaderEx;
    delete _optHeader;

    if(_floatValues)
    {
        delete[] _floatValues;
        _floatValues = NULL;
    }

	if (_cmdDataBuffer)
	{
		delete[] _cmdDataBuffer;
		_cmdDataBuffer = NULL;
	}

    if (_optDataBuffer)
    {
        delete[] _optDataBuffer;
        _optDataBuffer = NULL;
    }
}