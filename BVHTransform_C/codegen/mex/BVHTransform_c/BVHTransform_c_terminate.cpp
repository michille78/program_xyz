/*
 * BVHTransform_c_terminate.cpp
 *
 * Code generation for function 'BVHTransform_c_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "BVHTransform_c_terminate.h"

/* Function Definitions */
void BVHTransform_c_atexit()
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void BVHTransform_c_terminate()
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (BVHTransform_c_terminate.cpp) */
