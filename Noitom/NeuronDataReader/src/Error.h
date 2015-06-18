#pragma once
class Error
{
private:
	std::mutex mtx;

    Error(void);
    ~Error(void);

    int sysErrorCode;
    char* message;

    static Error* _instance;
public:
    static Error* Instance();
    
    void SetLastError(char* msg);
    char* GetLastError();
};

