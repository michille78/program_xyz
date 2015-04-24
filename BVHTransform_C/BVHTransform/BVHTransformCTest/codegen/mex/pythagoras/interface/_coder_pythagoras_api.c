/*
 * _coder_pythagoras_api.c
 *
 * Code generation for function '_coder_pythagoras_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "pythagoras.h"
#include "_coder_pythagoras_api.h"

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static char_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *color,
  const char_T *identifier);
static char_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u);
static char_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static char_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *color,
  const char_T *identifier)
{
  char_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(color), &thisId);
  emlrtDestroyArray(&color);
  return y;
}

static char_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  char_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(a), &thisId);
  emlrtDestroyArray(&a);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

static char_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  char_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 0U, 0);
  emlrtImportChar(src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

void pythagoras_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  real_T a;
  real_T b;
  char_T color;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  a = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "a");
  b = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "b");
  color = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "color");

  /* Invoke the target function */
  a = pythagoras(&st, a, b, color);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(a);
}

/* End of code generation (_coder_pythagoras_api.c) */
