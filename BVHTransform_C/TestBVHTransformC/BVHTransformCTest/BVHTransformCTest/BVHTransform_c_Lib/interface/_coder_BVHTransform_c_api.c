/*
 * File: _coder_BVHTransform_c_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

/* Include files */
#include "_coder_BVHTransform_c_api.h"

/* Function Declarations */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *BVHData,
  const char *identifier))[354];
static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[354];
static const mxArray *emlrt_marshallOut(const double u[25]);
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[354];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void BVHTransform_c_initialize(emlrtContext *aContext)
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
void BVHTransform_c_terminate(void)
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
void BVHTransform_c_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  BVHTransform_c_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void BVHTransform_c_api(const mxArray *prhs[1], const mxArray *plhs[1])
{
  double (*NaoData)[25];
  double (*BVHData)[354];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  NaoData = (double (*)[25])mxMalloc(sizeof(double [25]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);

  /* Marshall function inputs */
  BVHData = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "BVHData");

  /* Invoke the target function */
  BVHTransform_c(*BVHData, *NaoData);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*NaoData);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *BVHData
 *                const char *identifier
 * Return Type  : double (*)[354]
 */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *BVHData,
  const char *identifier))[354]
{
  double (*y)[354];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(BVHData), &thisId);
  emlrtDestroyArray(&BVHData);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[354]
 */
  static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[354]
{
  double (*y)[354];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u[25]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const double u[25])
{
  const mxArray *y;
  static const int iv0[1] = { 0 };

  const mxArray *m0;
  static const int iv1[1] = { 25 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[354]
 */
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[354]
{
  double (*ret)[354];
  int iv2[2];
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    iv2[i0] = 1 + 353 * i0;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv2);
  ret = (double (*)[354])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * File trailer for _coder_BVHTransform_c_api.c
 *
 * [EOF]
 */
