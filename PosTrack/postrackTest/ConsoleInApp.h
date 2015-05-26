#pragma once
#include <stdio.h>

class ConsoleInApp
{
    FILE* consolefp;

public:
	ConsoleInApp(void);
	~ConsoleInApp(void);
	
	void Open();
	void Write(char* msg);
	void WriteLine(char* msg);
	void Close();
};

