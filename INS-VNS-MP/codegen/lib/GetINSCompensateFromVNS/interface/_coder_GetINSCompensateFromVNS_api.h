/* 
 * File: _coder_GetINSCompensateFromVNS_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 18-Jun-2015 20:22:25 
 */

#ifndef ___CODER_GETINSCOMPENSATEFROMVNS_API_H__
#define ___CODER_GETINSCOMPENSATEFROMVNS_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T
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
#endif /*typedef_struct0_T*/
#ifndef typedef_struct1_T
#define typedef_struct1_T
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
#endif /*typedef_struct1_T*/
#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct
{
    int CalStartVN;
    int CalEndVN;
    int CalStartIN;
    int CalEndIN;
} struct2_T;
#endif /*typedef_struct2_T*/

/* Function Declarations */
extern void GetINSCompensateFromVNS_initialize(emlrtContext *aContext);
extern void GetINSCompensateFromVNS_terminate(void);
extern void GetINSCompensateFromVNS_atexit(void);
extern void GetINSCompensateFromVNS_api(const mxArray * const prhs[4], const mxArray *plhs[3]);
extern void GetINSCompensateFromVNS_xil_terminate(void);
extern void GetINSCompensateFromVNS(struct0_T InertialData, struct1_T otherMakers[3600], double compensateRate, struct2_T CalculateOrder, double InertialPositionCompensate[34560], double HipDisplacementNew[34560]);

#endif
/* 
 * File trailer for _coder_GetINSCompensateFromVNS_api.h 
 *  
 * [EOF] 
 */
