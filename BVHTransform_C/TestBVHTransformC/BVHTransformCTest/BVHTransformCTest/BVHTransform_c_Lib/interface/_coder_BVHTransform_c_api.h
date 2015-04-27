/* 
 * File: _coder_BVHTransform_c_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56 
 */

#ifndef ___CODER_BVHTRANSFORM_C_API_H__
#define ___CODER_BVHTRANSFORM_C_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void BVHTransform_c_initialize(emlrtContext *aContext);
extern void BVHTransform_c_terminate(void);
extern void BVHTransform_c_atexit(void);
extern void BVHTransform_c_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void BVHTransform_c(double BVHData[354], double NaoData[25]);
extern void BVHTransform_c_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_BVHTransform_c_api.h 
 *  
 * [EOF] 
 */
