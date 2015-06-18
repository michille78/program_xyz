/*
 * File: _coder_Cpp_OptitrackIneritalFusion_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-Jun-2015 22:07:14
 */

/* Include files */
#include "_coder_Cpp_OptitrackIneritalFusion_api.h"

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void Cpp_OptitrackIneritalFusion_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Cpp_OptitrackIneritalFusion_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Cpp_OptitrackIneritalFusion_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  Cpp_OptitrackIneritalFusion_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Cpp_OptitrackIneritalFusion_api(void)
{
  /* Invoke the target function */
  Cpp_OptitrackIneritalFusion();
}

/*
 * File trailer for _coder_Cpp_OptitrackIneritalFusion_api.c
 *
 * [EOF]
 */
