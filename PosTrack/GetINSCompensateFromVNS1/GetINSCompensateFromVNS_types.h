// 
// File: GetINSCompensateFromVNS_types.h 
//  
// MATLAB Coder version            : 2.6 
// C/C++ source code generated on  : 18-Jun-2015 17:49:21 
//

#ifndef __GETINSCOMPENSATEFROMVNS_TYPES_H__
#define __GETINSCOMPENSATEFROMVNS_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_b_real_T_5
#define struct_emxArray_b_real_T_5
struct emxArray_b_real_T_5
{
    double data[5];
    int size[1];
};
#endif /*struct_emxArray_b_real_T_5*/
#ifndef struct_emxArray_real_T_2
#define struct_emxArray_real_T_2
struct emxArray_real_T_2
{
    double data[2];
    int size[1];
};
#endif /*struct_emxArray_real_T_2*/
#ifndef struct_emxArray_real_T_5
#define struct_emxArray_real_T_5
struct emxArray_real_T_5
{
    double data[5];
    int size[1];
};
#endif /*struct_emxArray_real_T_5*/
#ifndef struct_emxArray_real_T_50x2
#define struct_emxArray_real_T_50x2
struct emxArray_real_T_50x2
{
    double data[100];
    int size[2];
};
#endif /*struct_emxArray_real_T_50x2*/
typedef struct
{
    double time[11520];
    int visual_k[11520];
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
    int inertial_k;
    signed char MarkerSet;
    double trackedMakerPosition[3];
    double ContinuesFlag[10];
    double ContinuesLastPosition[30];
    double ContinuesLastTime[10];
    double ContinuesLastK[10];
    unsigned char CalculatedTime;
} struct1_T;
typedef struct
{
    int CalStartVN;
    int CalEndVN;
    int CalStartIN;
    int CalEndIN;
} struct2_T;

#endif
// 
// File trailer for GetINSCompensateFromVNS_types.h 
//  
// [EOF] 
//
