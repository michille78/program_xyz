/*
 * File: _coder_GetINSCompensateFromVNS_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 23-Jun-2015 17:08:58
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
  emlrtMsgIdentifier *parentId, int y[11520]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[46080]);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[34560]);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[3]);
static double h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char *identifier, struct1_T y[3600]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[3600]);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[30]);
static int l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static signed char m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[10]);
static unsigned char o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static double p_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char *identifier);
static struct2_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *CalculateOrder, const char *identifier);
static struct2_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const double u[3]);
static const mxArray *b_emlrt_marshallOut(const struct1_T u[3600]);
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[11520]);
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, int ret[11520]);
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[46080]);
static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[34560]);
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[3]);
static double x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[30]);
static int ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static signed char bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[10]);
static unsigned char db_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId);

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
 * Arguments    : const mxArray * const prhs[4]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void GetINSCompensateFromVNS_api(const mxArray * const prhs[4], const mxArray
  *plhs[2])
{
  double (*AccumulateCompensate_k_Out)[3];
  static struct0_T InertialData;
  static struct1_T otherMakers[3600];
  double compensateRate;
  struct2_T CalculateOrder;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  AccumulateCompensate_k_Out = (double (*)[3])mxMalloc(sizeof(double [3]));

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "InertialData",
                   &InertialData);
  i_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "otherMakers",
                     otherMakers);
  compensateRate = p_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "compensateRate");
  CalculateOrder = q_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]),
    "CalculateOrder");

  /* Invoke the target function */
  GetINSCompensateFromVNS(InertialData, otherMakers, compensateRate,
    CalculateOrder, *AccumulateCompensate_k_Out);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*AccumulateCompensate_k_Out);
  plhs[1] = b_emlrt_marshallOut(otherMakers);
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
  static const char * fieldNames[8] = { "time", "visual_k", "HipQuaternion",
    "HipPosition", "HeadQuaternion", "HeadPosition", "BodyDirection",
    "frequency" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(sp, parentId, u, 8, fieldNames, 0U, 0);
  thisId.fIdentifier = "time";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "time")),
                     &thisId, y->time);
  thisId.fIdentifier = "visual_k";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "visual_k")),
                     &thisId, y->visual_k);
  thisId.fIdentifier = "HipQuaternion";
  e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "HipQuaternion")), &thisId, y->HipQuaternion);
  thisId.fIdentifier = "HipPosition";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "HipPosition")),
                     &thisId, y->HipPosition);
  thisId.fIdentifier = "HeadQuaternion";
  e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "HeadQuaternion")), &thisId, y->HeadQuaternion);
  thisId.fIdentifier = "HeadPosition";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "HeadPosition")),
                     &thisId, y->HeadPosition);
  thisId.fIdentifier = "BodyDirection";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "BodyDirection")), &thisId, y->BodyDirection);
  thisId.fIdentifier = "frequency";
  y->frequency = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
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
  s_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                int y[11520]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, int y[11520])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[46080]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[46080])
{
  u_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[34560]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[34560])
{
  v_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                double y[3]
 * Return Type  : void
 */
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[3])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double
 */
static double h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
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
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char *identifier, struct1_T y[3600])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  j_emlrt_marshallIn(sp, emlrtAlias(otherMakers), &thisId, y);
  emlrtDestroyArray(&otherMakers);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct1_T y[3600]
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[3600])
{
  emlrtMsgIdentifier thisId;
  int iv0[2];
  int i0;
  static const char * fieldNames[12] = { "frequency", "Position", "otherMakersN",
    "time", "inertial_k", "MarkerSet", "trackedMakerPosition", "ContinuesFlag",
    "ContinuesLastPosition", "ContinuesLastTime", "ContinuesLastK",
    "CalculatedTime" };

  struct1_T (*r0)[3600];
  thisId.fParent = parentId;
  for (i0 = 0; i0 < 2; i0++) {
    iv0[i0] = 1 + 3599 * i0;
  }

  emlrtCheckStructR2012b(sp, parentId, u, 12, fieldNames, 2U, iv0);
  r0 = (struct1_T (*)[3600])y;
  for (i0 = 0; i0 < 3600; i0++) {
    thisId.fIdentifier = "frequency";
    (*r0)[i0].frequency = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a
      (sp, u, i0, "frequency")), &thisId);
    thisId.fIdentifier = "Position";
    k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0, "Position")),
                       &thisId, (*r0)[i0].Position);
    thisId.fIdentifier = "otherMakersN";
    (*r0)[i0].otherMakersN = l_emlrt_marshallIn(sp, emlrtAlias
      (emlrtGetFieldR2013a(sp, u, i0, "otherMakersN")), &thisId);
    thisId.fIdentifier = "time";
    (*r0)[i0].time = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u,
      i0, "time")), &thisId);
    thisId.fIdentifier = "inertial_k";
    (*r0)[i0].inertial_k = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a
      (sp, u, i0, "inertial_k")), &thisId);
    thisId.fIdentifier = "MarkerSet";
    (*r0)[i0].MarkerSet = m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a
      (sp, u, i0, "MarkerSet")), &thisId);
    thisId.fIdentifier = "trackedMakerPosition";
    g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0,
      "trackedMakerPosition")), &thisId, (*r0)[i0].trackedMakerPosition);
    thisId.fIdentifier = "ContinuesFlag";
    n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0,
      "ContinuesFlag")), &thisId, (*r0)[i0].ContinuesFlag);
    thisId.fIdentifier = "ContinuesLastPosition";
    k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0,
      "ContinuesLastPosition")), &thisId, (*r0)[i0].ContinuesLastPosition);
    thisId.fIdentifier = "ContinuesLastTime";
    n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0,
      "ContinuesLastTime")), &thisId, (*r0)[i0].ContinuesLastTime);
    thisId.fIdentifier = "ContinuesLastK";
    n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i0,
      "ContinuesLastK")), &thisId, (*r0)[i0].ContinuesLastK);
    thisId.fIdentifier = "CalculatedTime";
    (*r0)[i0].CalculatedTime = o_emlrt_marshallIn(sp, emlrtAlias
      (emlrtGetFieldR2013a(sp, u, i0, "CalculatedTime")), &thisId);
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
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[30])
{
  y_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : int
 */
static int l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int y;
  y = ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : signed char
 */
static signed char m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  signed char y;
  y = bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
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
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, double y[10])
{
  cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : unsigned char
 */
static unsigned char o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  unsigned char y;
  y = db_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *compensateRate
 *                const char *identifier
 * Return Type  : double
 */
static double p_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(compensateRate), &thisId);
  emlrtDestroyArray(&compensateRate);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *CalculateOrder
 *                const char *identifier
 * Return Type  : struct2_T
 */
static struct2_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *CalculateOrder, const char *identifier)
{
  struct2_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = r_emlrt_marshallIn(sp, emlrtAlias(CalculateOrder), &thisId);
  emlrtDestroyArray(&CalculateOrder);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : struct2_T
 */
static struct2_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  struct2_T y;
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[4] = { "CalStartVN", "CalEndVN", "CalStartIN",
    "CalEndIN" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(sp, parentId, u, 4, fieldNames, 0U, 0);
  thisId.fIdentifier = "CalStartVN";
  y.CalStartVN = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "CalStartVN")), &thisId);
  thisId.fIdentifier = "CalEndVN";
  y.CalEndVN = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "CalEndVN")), &thisId);
  thisId.fIdentifier = "CalStartIN";
  y.CalStartIN = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "CalStartIN")), &thisId);
  thisId.fIdentifier = "CalEndIN";
  y.CalEndIN = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "CalEndIN")), &thisId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const double u[3])
{
  const mxArray *y;
  static const int iv1[1] = { 0 };

  const mxArray *m0;
  static const int iv2[1] = { 3 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv1, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, *(int (*)[1])&iv2[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const struct1_T u[3600]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const struct1_T u[3600])
{
  const mxArray *y;
  int iv3[2];
  int i1;
  const struct1_T *r1;
  double b_u;
  const mxArray *b_y;
  const mxArray *m1;
  double c_u[30];
  int i;
  const mxArray *c_y;
  static const int iv4[2] = { 3, 10 };

  double *pData;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  signed char d_u;
  const mxArray *g_y;
  double e_u[3];
  const mxArray *h_y;
  static const int iv5[1] = { 3 };

  double f_u[10];
  const mxArray *i_y;
  static const int iv6[2] = { 1, 10 };

  const mxArray *j_y;
  static const int iv7[2] = { 3, 10 };

  const mxArray *k_y;
  static const int iv8[2] = { 1, 10 };

  const mxArray *l_y;
  static const int iv9[2] = { 1, 10 };

  unsigned char g_u;
  const mxArray *m_y;
  y = NULL;
  for (i1 = 0; i1 < 2; i1++) {
    iv3[i1] = 1 + 3599 * i1;
  }

  emlrtAssign(&y, emlrtCreateStructArray(2, iv3, 0, NULL));
  for (i1 = 0; i1 < 3600; i1++) {
    r1 = &u[i1];
    b_u = r1->frequency;
    b_y = NULL;
    m1 = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&b_y, m1);
    emlrtAddField(y, b_y, "frequency", i1);
    for (i = 0; i < 30; i++) {
      c_u[i] = r1->Position[i];
    }

    c_y = NULL;
    m1 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    memcpy(&pData[0], &c_u[0], 30U * sizeof(double));
    emlrtAssign(&c_y, m1);
    emlrtAddField(y, c_y, "Position", i1);
    i = r1->otherMakersN;
    d_y = NULL;
    m1 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    *(int *)mxGetData(m1) = i;
    emlrtAssign(&d_y, m1);
    emlrtAddField(y, d_y, "otherMakersN", i1);
    b_u = r1->time;
    e_y = NULL;
    m1 = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&e_y, m1);
    emlrtAddField(y, e_y, "time", i1);
    i = r1->inertial_k;
    f_y = NULL;
    m1 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    *(int *)mxGetData(m1) = i;
    emlrtAssign(&f_y, m1);
    emlrtAddField(y, f_y, "inertial_k", i1);
    d_u = r1->MarkerSet;
    g_y = NULL;
    m1 = emlrtCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
    *(signed char *)mxGetData(m1) = d_u;
    emlrtAssign(&g_y, m1);
    emlrtAddField(y, g_y, "MarkerSet", i1);
    for (i = 0; i < 3; i++) {
      e_u[i] = r1->trackedMakerPosition[i];
    }

    h_y = NULL;
    m1 = emlrtCreateNumericArray(1, iv5, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    for (i = 0; i < 3; i++) {
      pData[i] = e_u[i];
    }

    emlrtAssign(&h_y, m1);
    emlrtAddField(y, h_y, "trackedMakerPosition", i1);
    for (i = 0; i < 10; i++) {
      f_u[i] = r1->ContinuesFlag[i];
    }

    i_y = NULL;
    m1 = emlrtCreateNumericArray(2, iv6, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    memcpy(&pData[0], &f_u[0], 10U * sizeof(double));
    emlrtAssign(&i_y, m1);
    emlrtAddField(y, i_y, "ContinuesFlag", i1);
    for (i = 0; i < 30; i++) {
      c_u[i] = r1->ContinuesLastPosition[i];
    }

    j_y = NULL;
    m1 = emlrtCreateNumericArray(2, iv7, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    memcpy(&pData[0], &c_u[0], 30U * sizeof(double));
    emlrtAssign(&j_y, m1);
    emlrtAddField(y, j_y, "ContinuesLastPosition", i1);
    for (i = 0; i < 10; i++) {
      f_u[i] = r1->ContinuesLastTime[i];
    }

    k_y = NULL;
    m1 = emlrtCreateNumericArray(2, iv8, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    memcpy(&pData[0], &f_u[0], 10U * sizeof(double));
    emlrtAssign(&k_y, m1);
    emlrtAddField(y, k_y, "ContinuesLastTime", i1);
    for (i = 0; i < 10; i++) {
      f_u[i] = r1->ContinuesLastK[i];
    }

    l_y = NULL;
    m1 = emlrtCreateNumericArray(2, iv9, mxDOUBLE_CLASS, mxREAL);
    pData = (double *)mxGetPr(m1);
    memcpy(&pData[0], &f_u[0], 10U * sizeof(double));
    emlrtAssign(&l_y, m1);
    emlrtAddField(y, l_y, "ContinuesLastK", i1);
    g_u = r1->CalculatedTime;
    m_y = NULL;
    m1 = emlrtCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
    *(unsigned char *)mxGetData(m1) = g_u;
    emlrtAssign(&m_y, m1);
    emlrtAddField(y, m_y, "CalculatedTime", i1);
  }

  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                double ret[11520]
 * Return Type  : void
 */
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[11520])
{
  int iv10[2];
  int i2;
  for (i2 = 0; i2 < 2; i2++) {
    iv10[i2] = 1 + 11519 * i2;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv10);
  for (i2 = 0; i2 < 11520; i2++) {
    ret[i2] = (*(double (*)[11520])mxGetData(src))[i2];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                int ret[11520]
 * Return Type  : void
 */
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, int ret[11520])
{
  int iv11[2];
  int i3;
  for (i3 = 0; i3 < 2; i3++) {
    iv11[i3] = 1 + 11519 * i3;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 2U, iv11);
  for (i3 = 0; i3 < 11520; i3++) {
    ret[i3] = (*(int (*)[11520])mxGetData(src))[i3];
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
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[46080])
{
  int iv12[2];
  int i4;
  int i5;
  for (i4 = 0; i4 < 2; i4++) {
    iv12[i4] = 4 + 11516 * i4;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv12);
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
static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[34560])
{
  int iv13[2];
  int i6;
  int i7;
  for (i6 = 0; i6 < 2; i6++) {
    iv13[i6] = 3 + 11517 * i6;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv13);
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
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[3])
{
  int iv14[1];
  int i8;
  iv14[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv14);
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
static double x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[30])
{
  int iv15[2];
  int i9;
  int i10;
  for (i9 = 0; i9 < 2; i9++) {
    iv15[i9] = 3 + 7 * i9;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv15);
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
static int ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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
static signed char bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, double ret[10])
{
  int iv16[2];
  int i11;
  for (i11 = 0; i11 < 2; i11++) {
    iv16[i11] = 1 + 9 * i11;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv16);
  for (i11 = 0; i11 < 10; i11++) {
    ret[i11] = (*(double (*)[10])mxGetData(src))[i11];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : unsigned char
 */
static unsigned char db_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId)
{
  unsigned char ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 0U, 0);
  ret = *(unsigned char *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_GetINSCompensateFromVNS_api.c
 *
 * [EOF]
 */
