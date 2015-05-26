#include "stdafx.h"
#include <io.h>
#include <fcntl.h>
#include "ConsoleInApp.h"


ConsoleInApp::ConsoleInApp(void)
{
}


ConsoleInApp::~ConsoleInApp(void)
{
}

void ConsoleInApp::Open()
{
	consolefp = NULL;

    ::AllocConsole();

	int nCrt = _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	if(nCrt!=-1)
	{
		consolefp = _fdopen(nCrt, "w");
		*stdout = *consolefp;
		setvbuf(stdout, NULL, _IONBF, 0);

		HWND console = GetConsoleWindow();

		//::SetWindowLong(console,GWL_STYLE,0);
		::SetWindowLong(console,GWL_EXSTYLE,0);
		//::SetParent(console,m_hWnd); 
		::SetWindowLong(console,WS_EX_TOPMOST,1);
	}
}

void ConsoleInApp::Close()
{
	FreeConsole(); 
}

void ConsoleInApp::Write(char* msg)
{		
	if(consolefp)
	{
		fprintf(consolefp, "%s", msg);
		fflush(consolefp);
	}
}
void ConsoleInApp::WriteLine(char* msg)
{		
	if(consolefp)
	{
		fprintf(consolefp, "%s\n", msg);
		fflush(consolefp);
	}
}

