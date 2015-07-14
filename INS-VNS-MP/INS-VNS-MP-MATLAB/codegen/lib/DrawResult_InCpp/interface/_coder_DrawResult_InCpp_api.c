/*
 * File: _coder_DrawResult_InCpp_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 18-Jun-2015 19:05:33
 */

/* Include files */
#include "_coder_DrawResult_InCpp_api.h"

/* Function Declarations */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *InertialPositionCompensate, const char *identifier))[34560];
static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[34560];
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[34560];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void DrawResult_InCpp_initialize(emlrtContext *aContext)
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
void DrawResult_InCpp_terminate(void)
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
void DrawResult_InCpp_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  DrawResult_InCpp_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[1]
 * Return Type  : void
 */
void DrawResult_InCpp_api(const mxArray *prhs[1])
{
  double (*InertialPositionCompensate)[34560];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);

  /* Marshall function inputs */
  InertialPositionCompensate = emlrt_marshallIn(&st, emlrtAlias(prhs[0]),
    "InertialPositionCompensate");

  /* Invoke the target function */
  DrawResult_InCpp(*InertialPositionCompensate);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *InertialPositionCompensate
 *                const char *identifier
 * Return Type  : double (*)[34560]
 */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *InertialPositionCompensate, const char *identifier))[34560]
{
  double (*y)[34560];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(InertialPositionCompensate), &thisId);
  emlrtDestroyArray(&InertialPositionCompensate);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[34560]
 */
  static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[34560]
{
  double (*y)[34560];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[34560]
 */
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[34560]
{
  double (*ret)[34560];
  int iv0[2];
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    iv0[i0] = 3 + 11517 * i0;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv0);
  ret = (double (*)[34560])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * File trailer for _coder_DrawResult_InCpp_api.c
 *
 * [EOF]
 */
