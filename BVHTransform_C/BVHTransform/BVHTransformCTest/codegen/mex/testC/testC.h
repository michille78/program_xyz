/*
 * testC.h
 *
 * Code generation for function 'testC'
 *
 */

#ifndef __TESTC_H__
#define __TESTC_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "testC_types.h"

/* Variable Declarations */
extern emlrtRSInfo emlrtRSI;
extern emlrtRSInfo b_emlrtRSI;
extern emlrtRSInfo c_emlrtRSI;

/* Function Declarations */
extern int8_T testC(const emlrtStack *sp, int16_T B);
extern void testC_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
extern void testC_atexit(void);
extern void testC_initialize(emlrtContext *aContext);
extern void testC_terminate(void);

#endif

/* End of code generation (testC.h) */
