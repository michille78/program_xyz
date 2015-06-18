#include "stdafx.h"
#include "Sock.h"


Sock::Sock()
{
	dataParser = NULL;
}


Sock::~Sock()
{
	if (dataParser)
	{
		delete dataParser;
		dataParser = NULL;
	}
}
