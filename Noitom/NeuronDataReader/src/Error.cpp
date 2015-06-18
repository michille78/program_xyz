#include "stdafx.h"
#include "Error.h"

Error* Error::_instance = NULL;

Error::Error(void)
{
    sysErrorCode = 0;
    message = NULL;
}


Error::~Error(void)
{
	mtx.lock();

    if(message != NULL)
    {        
        delete[] message;
        message = NULL;
    }

    if (_instance != NULL)
    {
        delete _instance;
    }

	mtx.unlock();
}

Error* Error::Instance()
{
    if(_instance==NULL)
    {
        _instance = new Error();
    }
    return _instance;
}
    
void Error::SetLastError(char* msg)
{
	mtx.lock();

#ifdef __OS_XUN__
    sysErrorCode = errno;
#else
    sysErrorCode = ::GetLastError();
#endif

    if(message)
    {
        delete[] message;
        message = NULL;
    }

    char tmp[100];
	tmp[0] = '\0';
	if (sysErrorCode == 0)
	{
		sprintf_s(tmp, sizeof(tmp), "%s", msg);
	}
	else
	{
		sprintf_s(tmp, sizeof(tmp), "%s System error code: %d", msg, sysErrorCode);
	}

    int len = strlen(tmp)+1;

    message = new char[len];
    sprintf_s(message, len, "%s", tmp);

	mtx.unlock();
}

char* Error::GetLastError()
{
    if(message==NULL) return "No error.";
    return message;
}