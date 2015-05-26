#include "Funcs.h"
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>


void PrintMatrix(double* m, int rows, int cols, char* matrixName)
{
	OutputDebugStringA(matrixName);
	PrintMatrix(m, rows, cols);
}

void PrintMatrix(double* m, int rows, int cols)
{
	char strTmp[500];	
	sprintf_s(strTmp, "[%d, %d]\n", rows, cols);
	OutputDebugStringA(strTmp);
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			sprintf_s(strTmp, "%0.3f ", m[i*cols+j]);
			OutputDebugStringA(strTmp);
		}
		OutputDebugStringA("\r\n");
	}
}

void PrintMatrix(char* filename, double* m, int rows, int cols)
{
	FILE* dddd = NULL;

	char fn[100];
	memset(fn, '\0', sizeof(fn));
	sprintf_s(fn, "%s.txt", filename);

	fopen_s(&dddd, fn, "w");

	static char buff[1024000];
	static char strTmp[500];
	memset(buff, '\0', sizeof(buff));
		
	memset(strTmp, '\0', sizeof(strTmp));
	sprintf_s(strTmp, "[%d, ", rows);
	strcat_s(buff, strTmp);
	memset(strTmp, '\0', sizeof(strTmp));
	sprintf_s(strTmp, "%d]\n", cols);
	strcat_s(buff, strTmp);

	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			memset(strTmp, '\0', sizeof(strTmp));
			sprintf_s(strTmp, "%f ", m[i*cols+j]);
			strcat_s(buff, strTmp);
		}
		strcat_s(buff, "\n");
	}

	OutputDebugStringA(buff);
	
	fprintf(dddd, "%s", buff);
	fflush(dddd);
	
	fclose(dddd);
}


void PrintMatrix(double* v, int cols)
{
	PrintMatrix(v, 1, cols);
}


void PrintMatrixToConsole(double* m, int rows, int cols)
{
	printf("[%d, %d]\n", rows, cols);
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			printf("%f ", m[i*cols+j]);
		}
		printf("\r\n");
	}
}
void PrintMatrixToConsole(double* m, int rows, int cols, char* matrixName)
{
	printf("%s\n", matrixName);
	PrintMatrixToConsole(m, rows, cols);
}

