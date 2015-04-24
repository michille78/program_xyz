/*
 * pythagoras_terminate.c
 *
 * Code generation for function 'pythagoras_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "pythagoras.h"
#include "pythagoras_terminate.h"

/* Function Definitions */
void pythagoras_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void pythagoras_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (pythagoras_terminate.c) */
