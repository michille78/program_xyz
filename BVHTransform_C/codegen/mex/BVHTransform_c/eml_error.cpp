/*
 * eml_error.cpp
 *
 * Code generation for function 'eml_error'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "eml_error.h"

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 20, 5, "eml_error",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\matlab\\eml\\eml_error.m"
};

/* Function Definitions */

/*
 *
 */
void b_eml_error(const emlrtStack *sp)
{
  static char_T cv1[4][1] = { { 'a' }, { 'c' }, { 'o' }, { 's' } };

  emlrtErrorWithMessageIdR2012b(sp, &emlrtRTEI, "Coder:toolbox:ElFunDomainError",
    3, 4, 4, cv1);
}

/*
 *
 */
void eml_error(const emlrtStack *sp)
{
  static char_T cv0[4][1] = { { 'a' }, { 's' }, { 'i' }, { 'n' } };

  emlrtErrorWithMessageIdR2012b(sp, &emlrtRTEI, "Coder:toolbox:ElFunDomainError",
    3, 4, 4, cv0);
}

/* End of code generation (eml_error.cpp) */
