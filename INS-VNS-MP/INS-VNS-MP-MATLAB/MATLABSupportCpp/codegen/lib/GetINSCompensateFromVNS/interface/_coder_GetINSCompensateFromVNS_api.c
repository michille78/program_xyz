/*
 * File: _coder_GetINSCompensateFromVNS_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 15-Jun-2015 10:51:25
 */

/* Include files */
#include "_coder_GetINSCompensateFromVNS_api.h"

/* Function Declarations */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *InertialData,
  const char *identifier, struct0_T *y);
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[11520]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[46080]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[34560]);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[3]);
static double g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char *identifier, struct1_T y[3600]);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[3600]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[30]);
static int k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static signed char l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[10]);
static double n_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char *identifier);
static const mxArray *emlrt_marshallOut(const struct0_T *u);
static const mxArray *b_emlrt_marshallOut(const double u);
static const mxArray *c_emlrt_marshallOut(const struct1_T u[3600]);
static const mxArray *d_emlrt_marshallOut(const struct2_T *u);
static const mxArray *e_emlrt_marshallOut(const struct1_T u_data[]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[11520]);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[46080]);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[34560]);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[3]);
static double s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[30]);
static int u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static signed char v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[10]);

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void GetINSCompensateFromVNS_initialize(emlrtContext *aContext)
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
void GetINSCompensateFromVNS_terminate(void)
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
void GetINSCompensateFromVNS_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  GetINSCompensateFromVNS_xil_terminate();
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[5]
 * Return Type  : void
 */
void GetINSCompensateFromVNS_api(const mxArray * const prhs[3], const mxArray
  *plhs[5])
{
  static struct0_T InertialData;
  static struct1_T otherMakers[3600];
  double compensateRate;
  static struct0_T b_InertialData;
  static struct1_T b_otherMakers[3600];
  int i0;
  int otherMakers2_size[2];
  struct1_T otherMakers2_data[1];
  static struct2_T InertialData2;
  double compensateRate1;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "InertialData",
                   &InertialData);
  h_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "otherMakers",
                     otherMakers);
  compensateRate = n_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "compensateRate");

  /* Invoke the target function */
  b_InertialData = InertialData;
  for (i0 = 0; i0 < 3600; i0++) {
    b_otherMakers[i0] = otherMakers[i0];
  }

  GetINSCompensateFromVNS(&b_InertialData, b_otherMakers, compensateRate,
    &InertialData, otherMakers, &compensateRate1, &InertialData2,
    otherMakers2_data, otherMakers2_size);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(&InertialData);
  plhs[1] = c_emlrt_marshallOut(otherMakers);
  plhs[2] = b_emlrt_marshallOut(compensateRate1);
  plhs[3] = d_emlrt_marshallOut(&InertialData2);
  plhs[4] = e_emlrt_marshallOut(otherMakers2_data);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *InertialData
 *                const char *identifier
 *                struct0_T *y
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *InertialData,
  const char *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  b_emlrt_marshallIn(sp, emlrtAlias(InertialData), &thisId, y);
  emlrtDestroyArray(&InertialData);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct0_T *y
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[7] = { "time", "HipQuaternion", "HipPosition",
    "HeadQuaternion", "HeadPosition", "BodyDirection", "frequency" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(sp, parentId, u, 7, fieldNames, 0U, 0);
  thisId.fIdentifier = "time";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "time")),
                     &thisId, y->time);
  thisId.fIdentifier = "HipQuaternion";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "HipQuaternion")), &thisId, y->HipQuaternion);
  thisId.fIdentifier = "HipPosition";
  e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "HipPosition")),
                     &thisId, y->HipPosition);
  thisId.fIdentifier = "HeadQuaternion";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "HeadQuaternion")), &thisId, y->HeadQuaternion);
  thisId.fIdentifier = "HeadPosition";
  e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "HeadPosition")),
                     &thisId, y->HeadPosition);
  thisId.fIdentifier = "BodyDirection";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "BodyDirection")), &thisId, y->BodyDirection);
  thisId.fIdentifier = "frequency";
  y->frequency = g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "frequency")), &thisId);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[11520]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[11520])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[46080]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[46080])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[34560]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[34560])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[3]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[3])
{
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double
 */
static double g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *otherMakers
 *                const char *identifier
 *                struct1_T y[3600]
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char *identifier, struct1_T y[3600])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  i_emlrt_marshallIn(sp, emlrtAlias(otherMakers), &thisId, y);
  emlrtDestroyArray(&otherMakers);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct1_T y[3600]
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[3600])
{
  emlrtMsgIdentifier thisId;
  int iv0[2];
  int i1;
  static const char * fieldNames[9] = { "frequency", "Position", "otherMakersN",
    "time", "MarkerSet", "ContinuesFlag", "ContinuesLastPosition",
    "ContinuesLastTime", "ContinuesLastK" };

  struct1_T (*r0)[3600];
  thisId.fParent = parentId;
  for (i1 = 0; i1 < 2; i1++) {
    iv0[i1] = 1 + 3599 * i1;
  }

  emlrtCheckStructR2012b(sp, parentId, u, 9, fieldNames, 2U, iv0);
  r0 = (struct1_T (*)[3600])y;
  for (i1 = 0; i1 < 3600; i1++) {
    thisId.fIdentifier = "frequency";
    (*r0)[i1].frequency = g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a
      (sp, u, i1, "frequency")), &thisId);
    thisId.fIdentifier = "Position";
    j_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i1, "Position")),
                       &thisId, (*r0)[i1].Position);
    thisId.fIdentifier = "otherMakersN";
    (*r0)[i1].otherMakersN = k_emlrt_marshallIn(sp, emlrtAlias
      (emlrtGetFieldR2013a(sp, u, i1, "otherMakersN")), &thisId);
    thisId.fIdentifier = "time";
    (*r0)[i1].time = g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u,
      i1, "time")), &thisId);
    thisId.fIdentifier = "MarkerSet";
    (*r0)[i1].MarkerSet = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a
      (sp, u, i1, "MarkerSet")), &thisId);
    thisId.fIdentifier = "ContinuesFlag";
    m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i1,
      "ContinuesFlag")), &thisId, (*r0)[i1].ContinuesFlag);
    thisId.fIdentifier = "ContinuesLastPosition";
    j_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i1,
      "ContinuesLastPosition")), &thisId, (*r0)[i1].ContinuesLastPosition);
    thisId.fIdentifier = "ContinuesLastTime";
    m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i1,
      "ContinuesLastTime")), &thisId, (*r0)[i1].ContinuesLastTime);
    thisId.fIdentifier = "ContinuesLastK";
    m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i1,
      "ContinuesLastK")), &thisId, (*r0)[i1].ContinuesLastK);
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[30]
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[30])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : int
 */
static int k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int y;
  y = u_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : signed char
 */
static signed char l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  signed char y;
  y = v_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[10]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[10])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *compensateRate
 *                const char *identifier
 * Return Type  : double
 */
static double n_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = g_emlrt_marshallIn(sp, emlrtAlias(compensateRate), &thisId);
  emlrtDestroyArray(&compensateRate);
  return y;
}

/*
 * Arguments    : const struct0_T *u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const struct0_T *u)
{
  const mxArray *y;
  const mxArray *b_y;
  static const int iv1[2] = { 1, 11520 };

  const mxArray *m0;
  double *pData;
  int i;
  const mxArray *c_y;
  static const int iv2[2] = { 4, 11520 };

  const mxArray *d_y;
  static const int iv3[2] = { 3, 11520 };

  const mxArray *e_y;
  static const int iv4[2] = { 4, 11520 };

  const mxArray *f_y;
  static const int iv5[2] = { 3, 11520 };

  const mxArray *g_y;
  static const int iv6[1] = { 3 };

  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  b_y = NULL;
  m0 = emlrtCreateNumericArray(2, iv1, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 11520; i++) {
    pData[i] = u->time[i];
  }

  emlrtAssign(&b_y, m0);
  emlrtAddField(y, b_y, "time", 0);
  c_y = NULL;
  m0 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 46080; i++) {
    pData[i] = u->HipQuaternion[i];
  }

  emlrtAssign(&c_y, m0);
  emlrtAddField(y, c_y, "HipQuaternion", 0);
  d_y = NULL;
  m0 = emlrtCreateNumericArray(2, iv3, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 34560; i++) {
    pData[i] = u->HipPosition[i];
  }

  emlrtAssign(&d_y, m0);
  emlrtAddField(y, d_y, "HipPosition", 0);
  e_y = NULL;
  m0 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 46080; i++) {
    pData[i] = u->HeadQuaternion[i];
  }

  emlrtAssign(&e_y, m0);
  emlrtAddField(y, e_y, "HeadQuaternion", 0);
  f_y = NULL;
  m0 = emlrtCreateNumericArray(2, iv5, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 34560; i++) {
    pData[i] = u->HeadPosition[i];
  }

  emlrtAssign(&f_y, m0);
  emlrtAddField(y, f_y, "HeadPosition", 0);
  g_y = NULL;
  m0 = emlrtCreateNumericArray(1, iv6, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m0);
  for (i = 0; i < 3; i++) {
    pData[i] = u->BodyDirection[i];
  }

  emlrtAssign(&g_y, m0);
  emlrtAddField(y, g_y, "BodyDirection", 0);
  emlrtAddField(y, b_emlrt_marshallOut(u->frequency), "frequency", 0);
  return y;
}

/*
 * Arguments    : const double u
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const double u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const struct1_T u[3600]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const struct1_T u[3600])
{
  const mxArray *y;
  int iv7[2];
  int i2;
  const struct1_T *r1;
  double b_u[30];
  int c_u;
  const mxArray *b_y;
  static const int iv8[2] = { 3, 10 };

  const mxArray *m2;
  double *pData;
  const mxArray *c_y;
  signed char d_u;
  const mxArray *d_y;
  double e_u[10];
  const mxArray *e_y;
  static const int iv9[2] = { 1, 10 };

  const mxArray *f_y;
  static const int iv10[2] = { 3, 10 };

  const mxArray *g_y;
  static const int iv11[2] = { 1, 10 };

  const mxArray *h_y;
  static const int iv12[2] = { 1, 10 };

  y = NULL;
  for (i2 = 0; i2 < 2; i2++) {
    iv7[i2] = 1 + 3599 * i2;
  }

  emlrtAssign(&y, emlrtCreateStructArray(2, iv7, 0, NULL));
  for (i2 = 0; i2 < 3600; i2++) {
    r1 = &u[i2];
    emlrtAddField(y, b_emlrt_marshallOut(r1->frequency), "frequency", i2);
    for (c_u = 0; c_u < 30; c_u++) {
      b_u[c_u] = r1->Position[c_u];
    }

    b_y = NULL;
    m2 = emlrtCreateNumericArray(2, iv8, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m2);
    memcpy(&pData[0], &b_u[0], 30U * sizeof(double));
    emlrtAssign(&b_y, m2);
    emlrtAddField(y, b_y, "Position", i2);
    c_u = r1->otherMakersN;
    c_y = NULL;
    m2 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    *(int *)mxGetData(m2) = c_u;
    emlrtAssign(&c_y, m2);
    emlrtAddField(y, c_y, "otherMakersN", i2);
    emlrtAddField(y, b_emlrt_marshallOut(r1->time), "time", i2);
    d_u = r1->MarkerSet;
    d_y = NULL;
    m2 = emlrtCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
    *(signed char *)mxGetData(m2) = d_u;
    emlrtAssign(&d_y, m2);
    emlrtAddField(y, d_y, "MarkerSet", i2);
    for (c_u = 0; c_u < 10; c_u++) {
      e_u[c_u] = r1->ContinuesFlag[c_u];
    }

    e_y = NULL;
    m2 = emlrtCreateNumericArray(2, iv9, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m2);
    memcpy(&pData[0], &e_u[0], 10U * sizeof(double));
    emlrtAssign(&e_y, m2);
    emlrtAddField(y, e_y, "ContinuesFlag", i2);
    for (c_u = 0; c_u < 30; c_u++) {
      b_u[c_u] = r1->ContinuesLastPosition[c_u];
    }

    f_y = NULL;
    m2 = emlrtCreateNumericArray(2, iv10, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m2);
    memcpy(&pData[0], &b_u[0], 30U * sizeof(double));
    emlrtAssign(&f_y, m2);
    emlrtAddField(y, f_y, "ContinuesLastPosition", i2);
    for (c_u = 0; c_u < 10; c_u++) {
      e_u[c_u] = r1->ContinuesLastTime[c_u];
    }

    g_y = NULL;
    m2 = emlrtCreateNumericArray(2, iv11, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m2);
    memcpy(&pData[0], &e_u[0], 10U * sizeof(double));
    emlrtAssign(&g_y, m2);
    emlrtAddField(y, g_y, "ContinuesLastTime", i2);
    for (c_u = 0; c_u < 10; c_u++) {
      e_u[c_u] = r1->ContinuesLastK[c_u];
    }

    h_y = NULL;
    m2 = emlrtCreateNumericArray(2, iv12, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m2);
    memcpy(&pData[0], &e_u[0], 10U * sizeof(double));
    emlrtAssign(&h_y, m2);
    emlrtAddField(y, h_y, "ContinuesLastK", i2);
  }

  return y;
}

/*
 * Arguments    : const struct2_T *u
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const struct2_T *u)
{
  const mxArray *y;
  const mxArray *b_y;
  static const int iv13[2] = { 4, 11520 };

  const mxArray *m3;
  double *pData;
  int i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  b_y = NULL;
  m3 = emlrtCreateNumericArray(2, iv13, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m3);
  for (i = 0; i < 46080; i++) {
    pData[i] = u->HeadQuaternion[i];
  }

  emlrtAssign(&b_y, m3);
  emlrtAddField(y, b_y, "HeadQuaternion", 0);
  return y;
}

/*
 * Arguments    : const struct1_T u_data[]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const struct1_T u_data[])
{
  const mxArray *y;
  int iv14[2];
  int i;
  const mxArray *b_y;
  static const int iv15[2] = { 3, 10 };

  const mxArray *m4;
  double *pData;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  static const int iv16[2] = { 1, 10 };

  const mxArray *f_y;
  static const int iv17[2] = { 3, 10 };

  const mxArray *g_y;
  static const int iv18[2] = { 1, 10 };

  const mxArray *h_y;
  static const int iv19[2] = { 1, 10 };

  y = NULL;
  for (i = 0; i < 2; i++) {
    iv14[i] = 1;
  }

  emlrtAssign(&y, emlrtCreateStructArray(2, iv14, 0, NULL));
  emlrtCreateField(y, "frequency");
  emlrtCreateField(y, "Position");
  emlrtCreateField(y, "otherMakersN");
  emlrtCreateField(y, "time");
  emlrtCreateField(y, "MarkerSet");
  emlrtCreateField(y, "ContinuesFlag");
  emlrtCreateField(y, "ContinuesLastPosition");
  emlrtCreateField(y, "ContinuesLastTime");
  emlrtCreateField(y, "ContinuesLastK");
  emlrtAddField(y, b_emlrt_marshallOut(u_data[0].frequency), "frequency", 0);
  b_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv15, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m4);
  for (i = 0; i < 30; i++) {
    pData[i] = u_data[0].Position[i];
  }

  emlrtAssign(&b_y, m4);
  emlrtAddField(y, b_y, "Position", 0);
  c_y = NULL;
  m4 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
  *(int *)mxGetData(m4) = u_data[0].otherMakersN;
  emlrtAssign(&c_y, m4);
  emlrtAddField(y, c_y, "otherMakersN", 0);
  emlrtAddField(y, b_emlrt_marshallOut(u_data[0].time), "time", 0);
  d_y = NULL;
  m4 = emlrtCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
  *(signed char *)mxGetData(m4) = u_data[0].MarkerSet;
  emlrtAssign(&d_y, m4);
  emlrtAddField(y, d_y, "MarkerSet", 0);
  e_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv16, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m4);
  for (i = 0; i < 10; i++) {
    pData[i] = u_data[0].ContinuesFlag[i];
  }

  emlrtAssign(&e_y, m4);
  emlrtAddField(y, e_y, "ContinuesFlag", 0);
  f_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv17, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m4);
  for (i = 0; i < 30; i++) {
    pData[i] = u_data[0].ContinuesLastPosition[i];
  }

  emlrtAssign(&f_y, m4);
  emlrtAddField(y, f_y, "ContinuesLastPosition", 0);
  g_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv18, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m4);
  for (i = 0; i < 10; i++) {
    pData[i] = u_data[0].ContinuesLastTime[i];
  }

  emlrtAssign(&g_y, m4);
  emlrtAddField(y, g_y, "ContinuesLastTime", 0);
  h_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv19, mxDOUBLE_CLASS, mxREAL);
  pData = (double *)mxGetPr(m4);
  for (i = 0; i < 10; i++) {
    pData[i] = u_data[0].ContinuesLastK[i];
  }

  emlrtAssign(&h_y, m4);
  emlrtAddField(y, h_y, "ContinuesLastK", 0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[11520]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[11520])
{
  int iv20[2];
  int i3;
  for (i3 = 0; i3 < 2; i3++) {
    iv20[i3] = 1 + 11519 * i3;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv20);
  for (i3 = 0; i3 < 11520; i3++) {
    ret[i3] = (*(double (*)[11520])mxGetData(src))[i3];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[46080]
 * Return Type  : void
 */
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[46080])
{
  int iv21[2];
  int i4;
  int i5;
  for (i4 = 0; i4 < 2; i4++) {
    iv21[i4] = 4 + 11516 * i4;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv21);
  for (i4 = 0; i4 < 11520; i4++) {
    for (i5 = 0; i5 < 4; i5++) {
      ret[i5 + (i4 << 2)] = (*(double (*)[46080])mxGetData(src))[i5 + (i4 << 2)];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[34560]
 * Return Type  : void
 */
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[34560])
{
  int iv22[2];
  int i6;
  int i7;
  for (i6 = 0; i6 < 2; i6++) {
    iv22[i6] = 3 + 11517 * i6;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv22);
  for (i6 = 0; i6 < 11520; i6++) {
    for (i7 = 0; i7 < 3; i7++) {
      ret[i7 + 3 * i6] = (*(double (*)[34560])mxGetData(src))[i7 + 3 * i6];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[3]
 * Return Type  : void
 */
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[3])
{
  int iv23[1];
  int i8;
  iv23[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv23);
  for (i8 = 0; i8 < 3; i8++) {
    ret[i8] = (*(double (*)[3])mxGetData(src))[i8];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double
 */
static double s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  double ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(double *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[30]
 * Return Type  : void
 */
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[30])
{
  int iv24[2];
  int i9;
  int i10;
  for (i9 = 0; i9 < 2; i9++) {
    iv24[i9] = 3 + 7 * i9;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv24);
  for (i9 = 0; i9 < 10; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      ret[i10 + 3 * i9] = (*(double (*)[30])mxGetData(src))[i10 + 3 * i9];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : int
 */
static int u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  int ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 0U, 0);
  ret = *(int *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : signed char
 */
static signed char v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  signed char ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int8", false, 0U, 0);
  ret = *(signed char *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[10]
 * Return Type  : void
 */
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[10])
{
  int iv25[2];
  int i11;
  for (i11 = 0; i11 < 2; i11++) {
    iv25[i11] = 1 + 9 * i11;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv25);
  for (i11 = 0; i11 < 10; i11++) {
    ret[i11] = (*(double (*)[10])mxGetData(src))[i11];
  }

  emlrtDestroyArray(&src);
}

/*
 * File trailer for _coder_GetINSCompensateFromVNS_api.c
 *
 * [EOF]
 */
