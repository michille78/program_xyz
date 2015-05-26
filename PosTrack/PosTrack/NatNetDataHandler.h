#pragma once

#include "NatNetSDK2.7/include/NatNetTypes.h"

class NatNetDataHandler
{
public:
	virtual void DataHandle(sFrameOfMocapData* data) = 0;
};