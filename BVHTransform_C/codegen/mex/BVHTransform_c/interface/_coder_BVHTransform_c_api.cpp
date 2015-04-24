/*
 * _coder_BVHTransform_c_api.cpp
 *
 * Code generation for function '_coder_BVHTransform_c_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "_coder_BVHTransform_c_api.h"

/* Function Declarations */
static real32_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[354];
static const mxArray *b_emlrt_marshallOut(const real_T u[3]);
static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[354];
static real32_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *BVHData,
  const char_T *identifier))[354];
static const mxArray *emlrt_marshallOut(const real_T u[25]);

/* Function Definitions */
static real32_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[354]
{
  real32_T (*y)[354];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *b_emlrt_marshallOut(const real_T u[3])
{
  const mxArray *y;
  static const int32_T iv5[2] = { 0, 0 };

  const mxArray *m1;
  static const int32_T iv6[2] = { 1, 3 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv5, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[2])&iv6[0], 2);
  emlrtAssign(&y, m1);
  return y;
}

static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[354]
{
  real32_T (*ret)[354];
  int32_T iv7[2];
  int32_T i25;
  for (i25 = 0; i25 < 2; i25++) {
    iv7[i25] = 1 + 353 * i25;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv7);
  ret = (real32_T (*)[354])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real32_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *BVHData, const char_T *identifier))[354]
{
  real32_T (*y)[354];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(BVHData), &thisId);
  emlrtDestroyArray(&BVHData);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[25])
{
  const mxArray *y;
  static const int32_T iv3[1] = { 0 };

  const mxArray *m0;
  static const int32_T iv4[1] = { 25 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv3, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[1])&iv4[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

void BVHTransform_c_api(const mxArray * const prhs[1], const mxArray *plhs[2])
{
  real_T (*NaoData)[25];
  real_T (*ROOT_Hips_RotationNew)[3];
  real32_T (*BVHData)[354];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  NaoData = (real_T (*)[25])mxMalloc(sizeof(real_T [25]));
  ROOT_Hips_RotationNew = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));

  /* Marshall function inputs */
  BVHData = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]),
    "BVHData");

  /* Invoke the target function */
  BVHTransform_c(&st, *BVHData, *NaoData, *ROOT_Hips_RotationNew);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*NaoData);
  plhs[1] = b_emlrt_marshallOut(*ROOT_Hips_RotationNew);
}

/* End of code generation (_coder_BVHTransform_c_api.cpp) */
