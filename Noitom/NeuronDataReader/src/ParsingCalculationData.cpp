#include "stdafx.h"
#include "ParsingCalculationData.h"


ParsingCalculationData::ParsingCalculationData(void)
{
    _calculationHeader = new CalculationDataHeader();
    _floatValuesBufferSize = 338;
    _floatValues = new float[_floatValuesBufferSize];
    
    _headSize = 64;
    _headerLen = sizeof(CalculationDataHeader);

    memset(avatarName, '\0', sizeof(avatarName));
}


ParsingCalculationData::~ParsingCalculationData(void)
{
    
    if (_calculationHeader != NULL)
    {
        delete _calculationHeader;
        _calculationHeader = NULL;
    }

    if(_floatValues)
    {
        delete[] _floatValues;
        _floatValues = NULL;
    }
}
