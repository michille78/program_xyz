/*
 * File: _coder_GenThumbDirection_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 27-May-2015 16:37:00
 */

/* Include files */
#include "_coder_GenThumbDirection_api.h"

/* Function Declarations */
static float (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *rotateAngle,
  const char *identifier))[3];
static float (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static char c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RightOrLeft,
  const char *identifier);
static char d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const float u[9]);
static float (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static char f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void GenThumbDirection_initialize(emlrtContext *aContext)
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
void GenThumbDirection_terminate(void)
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
void GenThumbDirection_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  GenThumbDirection_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[4]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void GenThumbDirection_api(const mxArray *prhs[4], const mxArray *plhs[1])
{
  float (*bone_w)[9];
  float (*rotateAngle)[3];
  float (*zd)[3];
  char RightOrLeft;
  char pose;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  bone_w = (float (*)[9])mxMalloc(sizeof(float [9]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  rotateAngle = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "rotateAngle");
  zd = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "zd");
  RightOrLeft = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "RightOrLeft");
  pose = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "pose");

  /* Invoke the target function */
  GenThumbDirection(*rotateAngle, *zd, RightOrLeft, pose, *bone_w);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*bone_w);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *rotateAngle
 *                const char *identifier
 * Return Type  : float (*)[3]
 */
static float (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *rotateAngle,
  const char *identifier))[3]
{
  float (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(rotateAngle), &thisId);
  emlrtDestroyArray(&rotateAngle);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[3]
 */
  static float (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  float (*y)[3];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *RightOrLeft
 *                const char *identifier
 * Return Type  : char
 */
static char c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RightOrLeft,
  const char *identifier)
{
  char y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(RightOrLeft), &thisId);
  emlrtDestroyArray(&RightOrLeft);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : char
 */
static char d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  char y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const float u[9])
{
  const mxArray *y;
  static const int iv0[2] = { 0, 0 };

  const mxArray *m0;
  static const int iv1[2] = { 3, 3 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[3]
 */
static float (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  float (*ret)[3];
  int iv2[1];
  iv2[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv2);
  ret = (float (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : char
 */
  static char f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  char ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 0U, 0);
  emlrtImportChar(src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_GenThumbDirection_api.c
 *
 * [EOF]
 */
