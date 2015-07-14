// 
// File: GetINSCompensateFromVNS_types.h 
//  
// MATLAB Coder version            : 2.6 
// C/C++ source code generated on  : 15-Jun-2015 10:51:25 
//

#ifndef __GETINSCOMPENSATEFROMVNS_TYPES_H__
#define __GETINSCOMPENSATEFROMVNS_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
typedef struct
{
    double time[11520];
    double HipQuaternion[46080];
    double HipPosition[34560];
    double HeadQuaternion[46080];
    double HeadPosition[34560];
    double BodyDirection[3];
    double frequency;
} struct0_T;
typedef struct
{
    double frequency;
    double Position[30];
    int otherMakersN;
    double time;
    signed char MarkerSet;
    double ContinuesFlag[10];
    double ContinuesLastPosition[30];
    double ContinuesLastTime[10];
    double ContinuesLastK[10];
} struct1_T;
typedef struct
{
    double HeadQuaternion[46080];
} struct2_T;

#endif
// 
// File trailer for GetINSCompensateFromVNS_types.h 
//  
// [EOF] 
//
