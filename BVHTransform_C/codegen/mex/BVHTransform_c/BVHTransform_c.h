/*
 * BVHTransform_c.h
 *
 * Code generation for function 'BVHTransform_c'
 *
 */

#ifndef __BVHTRANSFORM_C_H__
#define __BVHTRANSFORM_C_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "BVHTransform_c_types.h"

/* Function Declarations */
extern void BVHTransform_c(const emlrtStack *sp, const real32_T BVHData[354],
  real_T NaoData[25], real_T ROOT_Hips_RotationNew[3]);

#endif

/* End of code generation (BVHTransform_c.h) */
