/*
 * pythagoras.c
 *
 * Code generation for function 'pythagoras'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "pythagoras.h"
#include "eml_error.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 3, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

static emlrtRSInfo b_emlrtRSI = { 4, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

static emlrtRSInfo c_emlrtRSI = { 14, "sqrt",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"
};

static emlrtMCInfo emlrtMCI = { 11, 1, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

static emlrtMCInfo b_emlrtMCI = { 12, 1, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

static emlrtRSInfo d_emlrtRSI = { 12, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

static emlrtRSInfo e_emlrtRSI = { 11, "pythagoras",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform\\BVHTransformCTest\\pythagoras.m"
};

/* Function Declarations */
static void axis(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static void patch(const emlrtStack *sp, const mxArray *b, const mxArray *c,
                  const mxArray *d, emlrtMCInfo *location);

/* Function Definitions */
static void axis(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "axis", true, location);
}

static void patch(const emlrtStack *sp, const mxArray *b, const mxArray *c,
                  const mxArray *d, emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  pArrays[0] = b;
  pArrays[1] = c;
  pArrays[2] = d;
  emlrtCallMATLABR2012b(sp, 0, NULL, 3, pArrays, "patch", true, location);
}

real_T pythagoras(const emlrtStack *sp, real_T a, real_T b, char_T color)
{
  real_T c;
  real_T x;
  real_T b_x[3];
  real_T y[3];
  const mxArray *b_y;
  static const int32_T iv0[1] = { 3 };

  const mxArray *m0;
  real_T *pData;
  int32_T i;
  const mxArray *c_y;
  static const int32_T iv1[1] = { 3 };

  const mxArray *d_y;
  const mxArray *e_y;
  static const int32_T iv2[2] = { 1, 5 };

  char_T cv0[5];
  static const char_T cv1[5] = { 'e', 'q', 'u', 'a', 'l' };

  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  x = a * a + b * b;
  if (x < 0.0) {
    b_st.site = &c_emlrtRSI;
    eml_error(&b_st);
  }

  c = muDoubleScalarSqrt(x);
  st.site = &b_emlrtRSI;
  b_x[0] = 0.0;
  b_x[1] = a;
  b_x[2] = a;
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = b;
  b_y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m0);
  for (i = 0; i < 3; i++) {
    pData[i] = b_x[i];
  }

  emlrtAssign(&b_y, m0);
  c_y = NULL;
  m0 = emlrtCreateNumericArray(1, iv1, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m0);
  for (i = 0; i < 3; i++) {
    pData[i] = y[i];
  }

  emlrtAssign(&c_y, m0);
  d_y = NULL;
  m0 = emlrtCreateString1(color);
  emlrtAssign(&d_y, m0);
  b_st.site = &e_emlrtRSI;
  patch(&b_st, b_y, c_y, d_y, &emlrtMCI);
  e_y = NULL;
  m0 = emlrtCreateCharArray(2, iv2);
  for (i = 0; i < 5; i++) {
    cv0[i] = cv1[i];
  }

  emlrtInitCharArrayR2013a(&st, 5, m0, cv0);
  emlrtAssign(&e_y, m0);
  b_st.site = &d_emlrtRSI;
  axis(&b_st, e_y, &b_emlrtMCI);
  return c;
}

/* End of code generation (pythagoras.c) */
