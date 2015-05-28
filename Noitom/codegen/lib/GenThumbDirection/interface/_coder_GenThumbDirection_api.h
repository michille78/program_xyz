/* 
 * File: _coder_GenThumbDirection_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 27-May-2015 16:37:00 
 */

#ifndef ___CODER_GENTHUMBDIRECTION_API_H__
#define ___CODER_GENTHUMBDIRECTION_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void GenThumbDirection_initialize(emlrtContext *aContext);
extern void GenThumbDirection_terminate(void);
extern void GenThumbDirection_atexit(void);
extern void GenThumbDirection_api(const mxArray *prhs[4], const mxArray *plhs[1]);
extern void GenThumbDirection(float rotateAngle[3], float zd[3], char RightOrLeft, char pose, float bone_w[9]);
extern void GenThumbDirection_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_GenThumbDirection_api.h 
 *  
 * [EOF] 
 */
