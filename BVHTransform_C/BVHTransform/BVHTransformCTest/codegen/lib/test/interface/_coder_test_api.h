/* 
 * File: _coder_test_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 05-Apr-2015 15:40:16 
 */

#ifndef ___CODER_TEST_API_H__
#define ___CODER_TEST_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void test_initialize(emlrtContext *aContext);
extern void test_terminate(void);
extern void test_atexit(void);
extern void test_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
extern double test(double b);
extern void test_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_test_api.h 
 *  
 * [EOF] 
 */
