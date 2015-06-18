/*
 * _coder_DrawResult_InCpp_mex.c
 *
 * Code generation for function 'DrawResult_InCpp'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_DrawResult_InCpp_api.h"

/* Function Declarations */
static void DrawResult_InCpp_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "DrawResult_InCpp", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void DrawResult_InCpp_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *inputs[1];
  int n = 0;
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  DrawResult_InCpp_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 1, mxCHAR_CLASS, 16, "DrawResult_InCpp");
  } else if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 16, "DrawResult_InCpp");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  DrawResult_InCpp_api(inputs);
  /* Module finalization. */
  DrawResult_InCpp_terminate();
}

void DrawResult_InCpp_atexit_wrapper(void)
{
   DrawResult_InCpp_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(DrawResult_InCpp_atexit_wrapper);
  /* Dispatch the entry-point. */
  DrawResult_InCpp_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_DrawResult_InCpp_mex.c) */
