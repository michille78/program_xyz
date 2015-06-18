#pragma once
class IDataParseDelegate;
class Sock
{
public:
	Sock();
	~Sock();

	IDataParseDelegate* dataParser;
};

