/*
 * BVHTransform_c_initialize.cpp
 *
 * Code generation for function 'BVHTransform_c_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "BVHTransform_c_initialize.h"
#include "BVHTransform_c_data.h"

/* Function Definitions */
void BVHTransform_c_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (BVHTransform_c_initialize.cpp) */
