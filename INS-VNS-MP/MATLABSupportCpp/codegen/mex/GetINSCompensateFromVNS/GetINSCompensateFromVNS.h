/*
 * GetINSCompensateFromVNS.h
 *
 * Code generation for function 'GetINSCompensateFromVNS'
 *
 */

#ifndef __GETINSCOMPENSATEFROMVNS_H__
#define __GETINSCOMPENSATEFROMVNS_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "GetINSCompensateFromVNS_types.h"

/* Function Declarations */
extern void GetINSCompensateFromVNS(const emlrtStack *sp, const struct0_T
  *InertialData, const emxArray_struct1_T *otherMakers, real_T compensateRate,
  struct0_T *InertialData1, emxArray_struct1_T *otherMakers1, real_T
  *compensateRate1, struct2_T *InertialData2);
extern void GetINSCompensateFromVNS_api(const mxArray * const prhs[3], const
  mxArray *plhs[4]);
extern void GetINSCompensateFromVNS_atexit();
extern void GetINSCompensateFromVNS_initialize(emlrtContext *aContext);
extern void GetINSCompensateFromVNS_terminate();

#endif

/* End of code generation (GetINSCompensateFromVNS.h) */
