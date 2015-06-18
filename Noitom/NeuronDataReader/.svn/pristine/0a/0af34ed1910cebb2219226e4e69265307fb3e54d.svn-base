#include "stdafx.h"
#include "CompressInfoCode.h"


CompressInfoCode::CompressInfoCode()
{
	presrcCount = 400;
	presrc = new float[presrcCount];
	memset(presrc, 0, presrcCount*sizeof(float));

	infolistCount = 354;
	infolist = new CompressMoveInfo[infolistCount];

	accuracy = 0.001;
}


CompressInfoCode::~CompressInfoCode()
{
	if (presrc)
	{
		delete[] presrc;
		presrc = NULL;
	}
	if (infolist)
	{
		delete[] infolist;
		infolist = NULL;
	}
}
