/*
 * testC.c
 *
 * Code generation for function 'testC'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "testC.h"

/* Variable Definitions */
static const volatile char_T *emlrtBreakCheckR2012bFlagVar;
static const mxArray *eml_mx;
static const mxArray *b_eml_mx;
static const mxArray *c_eml_mx;
emlrtRSInfo emlrtRSI = { 5, "testC",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\testC.m"
};

emlrtRSInfo b_emlrtRSI = { 82, "sqrt",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\fixedpoint\\@embedded\\@fi\\sqrt.m"
};

emlrtRSInfo c_emlrtRSI = { 214, "sqrt",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\fixedpoint\\@embedded\\@fi\\sqrt.m"
};

static emlrtMCInfo emlrtMCI = { -1, -1, "", "" };

/* Function Declarations */
static int16_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *b_numerictype(const emlrtStack *sp, const char * b, real_T
  c, const char * d, real_T e, const char * f, real_T g, const char * h, real_T
  i, const char * j, real_T k, emlrtMCInfo *location);
static int16_T c_emlrt_marshallIn(const mxArray *src);
static int16_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *B, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const int8_T u);
static const mxArray *fimath(const emlrtStack *sp, const char * b, const char
  * c, const char * d, const char * e, const char * f, const char * g, const
  char * h, const char * i, const char * j, const char * k, const char * l,
  real_T m, const char * n, real_T o, const char * p, real_T q, const char * r,
  real_T s, const char * t, real_T u, const char * v, real_T w, const char * x,
  real_T y, const char * ab, const char * bb, const char * cb, real_T db, const
  char * eb, real_T fb, const char * gb, real_T hb, const char * ib, real_T jb,
  const char * kb, real_T lb, const char * mb, real_T nb, const char * ob,
  real_T pb, const char * qb, boolean_T rb, emlrtMCInfo *location);
static const mxArray *numerictype(const emlrtStack *sp, emlrtMCInfo *location);
static void testC_once(const emlrtStack *sp);

/* Function Definitions */
static int16_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int16_T y;
  emlrtCheckFiR2012b(sp, parentId, u, false, 0U, 0, eml_mx, b_eml_mx);
  y = c_emlrt_marshallIn(emlrtAlias(u));
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_numerictype(const emlrtStack *sp, const char * b, real_T
  c, const char * d, real_T e, const char * f, real_T g, const char * h, real_T
  i, const char * j, real_T k, emlrtMCInfo *location)
{
  const mxArray *pArrays[10];
  const mxArray *m3;
  pArrays[0] = emlrtCreateString(b);
  pArrays[1] = emlrtCreateDoubleScalar(c);
  pArrays[2] = emlrtCreateString(d);
  pArrays[3] = emlrtCreateDoubleScalar(e);
  pArrays[4] = emlrtCreateString(f);
  pArrays[5] = emlrtCreateDoubleScalar(g);
  pArrays[6] = emlrtCreateString(h);
  pArrays[7] = emlrtCreateDoubleScalar(i);
  pArrays[8] = emlrtCreateString(j);
  pArrays[9] = emlrtCreateDoubleScalar(k);
  return emlrtCallMATLABR2012b(sp, 1, &m3, 10, pArrays, "numerictype", true,
    location);
}

static int16_T c_emlrt_marshallIn(const mxArray *src)
{
  int16_T ret;
  const mxArray *mxInt;
  mxInt = emlrtImportFiIntArrayR2008b(src);
  ret = *(int16_T *)mxGetData(mxInt);
  emlrtDestroyArray(&mxInt);
  emlrtDestroyArray(&src);
  return ret;
}

static int16_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *B, const
  char_T *identifier)
{
  int16_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(B), &thisId);
  emlrtDestroyArray(&B);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const int8_T u)
{
  const mxArray *y;
  const mxArray *b_y;
  const mxArray *m0;
  y = NULL;
  b_y = NULL;
  m0 = emlrtCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
  *(int8_T *)mxGetData(m0) = u;
  emlrtAssign(&b_y, m0);
  emlrtAssign(&y, emlrtCreateFIR2013b(sp, eml_mx, c_eml_mx, "simulinkarray", b_y,
    false, false));
  return y;
}

static const mxArray *fimath(const emlrtStack *sp, const char * b, const char
  * c, const char * d, const char * e, const char * f, const char * g, const
  char * h, const char * i, const char * j, const char * k, const char * l,
  real_T m, const char * n, real_T o, const char * p, real_T q, const char * r,
  real_T s, const char * t, real_T u, const char * v, real_T w, const char * x,
  real_T y, const char * ab, const char * bb, const char * cb, real_T db, const
  char * eb, real_T fb, const char * gb, real_T hb, const char * ib, real_T jb,
  const char * kb, real_T lb, const char * mb, real_T nb, const char * ob,
  real_T pb, const char * qb, boolean_T rb, emlrtMCInfo *location)
{
  const mxArray *pArrays[42];
  const mxArray *m1;
  pArrays[0] = emlrtCreateString(b);
  pArrays[1] = emlrtCreateString(c);
  pArrays[2] = emlrtCreateString(d);
  pArrays[3] = emlrtCreateString(e);
  pArrays[4] = emlrtCreateString(f);
  pArrays[5] = emlrtCreateString(g);
  pArrays[6] = emlrtCreateString(h);
  pArrays[7] = emlrtCreateString(i);
  pArrays[8] = emlrtCreateString(j);
  pArrays[9] = emlrtCreateString(k);
  pArrays[10] = emlrtCreateString(l);
  pArrays[11] = emlrtCreateDoubleScalar(m);
  pArrays[12] = emlrtCreateString(n);
  pArrays[13] = emlrtCreateDoubleScalar(o);
  pArrays[14] = emlrtCreateString(p);
  pArrays[15] = emlrtCreateDoubleScalar(q);
  pArrays[16] = emlrtCreateString(r);
  pArrays[17] = emlrtCreateDoubleScalar(s);
  pArrays[18] = emlrtCreateString(t);
  pArrays[19] = emlrtCreateDoubleScalar(u);
  pArrays[20] = emlrtCreateString(v);
  pArrays[21] = emlrtCreateDoubleScalar(w);
  pArrays[22] = emlrtCreateString(x);
  pArrays[23] = emlrtCreateDoubleScalar(y);
  pArrays[24] = emlrtCreateString(ab);
  pArrays[25] = emlrtCreateString(bb);
  pArrays[26] = emlrtCreateString(cb);
  pArrays[27] = emlrtCreateDoubleScalar(db);
  pArrays[28] = emlrtCreateString(eb);
  pArrays[29] = emlrtCreateDoubleScalar(fb);
  pArrays[30] = emlrtCreateString(gb);
  pArrays[31] = emlrtCreateDoubleScalar(hb);
  pArrays[32] = emlrtCreateString(ib);
  pArrays[33] = emlrtCreateDoubleScalar(jb);
  pArrays[34] = emlrtCreateString(kb);
  pArrays[35] = emlrtCreateDoubleScalar(lb);
  pArrays[36] = emlrtCreateString(mb);
  pArrays[37] = emlrtCreateDoubleScalar(nb);
  pArrays[38] = emlrtCreateString(ob);
  pArrays[39] = emlrtCreateDoubleScalar(pb);
  pArrays[40] = emlrtCreateString(qb);
  pArrays[41] = emlrtCreateLogicalScalar(rb);
  return emlrtCallMATLABR2012b(sp, 1, &m1, 42, pArrays, "fimath", true, location);
}

static const mxArray *numerictype(const emlrtStack *sp, emlrtMCInfo *location)
{
  const mxArray *m2;
  return emlrtCallMATLABR2012b(sp, 1, &m2, 0, NULL, "numerictype", true,
    location);
}

static void testC_once(const emlrtStack *sp)
{
  emlrtAssignP(&c_eml_mx, NULL);
  emlrtAssignP(&b_eml_mx, NULL);
  emlrtAssignP(&eml_mx, NULL);
  emlrtAssignP(&c_eml_mx, b_numerictype(sp, "WordLength", 8.0, "FractionLength",
    7.0, "BinaryPoint", 7.0, "Slope", 0.0078125, "FixedExponent", -7.0,
    &emlrtMCI));
  emlrtAssignP(&b_eml_mx, numerictype(sp, &emlrtMCI));
  emlrtAssignP(&eml_mx, fimath(sp, "RoundMode", "nearest", "RoundingMethod",
    "Nearest", "OverflowMode", "saturate", "OverflowAction", "Saturate",
    "ProductMode", "FullPrecision", "ProductWordLength", 32.0,
    "MaxProductWordLength", 65535.0, "ProductFractionLength", 30.0,
    "ProductFixedExponent", -30.0, "ProductSlope", 9.3132257461547852E-10,
    "ProductSlopeAdjustmentFactor", 1.0, "ProductBias", 0.0, "SumMode",
    "FullPrecision", "SumWordLength", 32.0, "MaxSumWordLength", 65535.0,
    "SumFractionLength", 30.0, "SumFixedExponent", -30.0, "SumSlope",
    9.3132257461547852E-10, "SumSlopeAdjustmentFactor", 1.0, "SumBias", 0.0,
    "CastBeforeSum", true, &emlrtMCI));
  emlrtCheckDefaultFimathR2008b(&eml_mx);
}

int8_T testC(const emlrtStack *sp, int16_T B)
{
  int8_T C;
  int32_T i;
  int8_T c;
  int32_T i0;
  (void)sp;

  /* codegen */
  /* TESTC 此处显示有关此函数的摘要 */
  /*    此处显示详细说明 */
  /*  coder.extrinsic( 'sin' ); */
  C = 0;
  if (B <= 0) {
  } else {
    for (i = 6; i > -1; i += -1) {
      c = (int8_T)(C | (int8_T)(1 << i));
      if ((c * c << 1) <= B) {
        C = c;
      }
    }

    if (C < 127) {
      i = C + 1;
      c = (int8_T)i;
      i = B + 1;
      i = c * c - (int32_T)((uint32_T)i >> 1);
      i0 = B + 1;
      i0 = (int32_T)((uint32_T)i0 >> 1) - C * C;
      if (i < i0) {
        C = c;
      }
    }
  }

  return C;
}

void testC_api(const mxArray * const prhs[1], const mxArray *plhs[1])
{
  int16_T B;
  int8_T C;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  B = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "B");

  /* Invoke the target function */
  C = testC(&st, B);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(&st, C);
}

void testC_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtDestroyArray(&eml_mx);
  emlrtDestroyArray(&b_eml_mx);
  emlrtDestroyArray(&c_eml_mx);
}

void testC_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 3U, "ForceOff");
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    testC_once(&st);
  }
}

void testC_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (testC.c) */
