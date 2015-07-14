/* 
 * File: _coder_DrawResult_InCpp_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 18-Jun-2015 19:05:33 
 */

#ifndef ___CODER_DRAWRESULT_INCPP_API_H__
#define ___CODER_DRAWRESULT_INCPP_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void DrawResult_InCpp_initialize(emlrtContext *aContext);
extern void DrawResult_InCpp_terminate(void);
extern void DrawResult_InCpp_atexit(void);
extern void DrawResult_InCpp_api(const mxArray *prhs[1]);
extern void DrawResult_InCpp(double InertialPositionCompensate[34560]);
extern void DrawResult_InCpp_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_DrawResult_InCpp_api.h 
 *  
 * [EOF] 
 */
