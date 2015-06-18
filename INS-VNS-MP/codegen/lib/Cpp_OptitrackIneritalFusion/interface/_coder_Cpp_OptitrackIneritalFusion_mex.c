/*
 * _coder_Cpp_OptitrackIneritalFusion_mex.c
 *
 * Code generation for function 'Cpp_OptitrackIneritalFusion'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_Cpp_OptitrackIneritalFusion_api.h"

/* Function Declarations */
static void Cpp_OptitrackIneritalFusion_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "Cpp_OptitrackIneritalFusion", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void Cpp_OptitrackIneritalFusion_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  Cpp_OptitrackIneritalFusion_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 0, mxCHAR_CLASS, 27, "Cpp_OptitrackIneritalFusion");
  } else if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 27, "Cpp_OptitrackIneritalFusion");
  }
  /* Call the function. */
  Cpp_OptitrackIneritalFusion_api();
  /* Module finalization. */
  Cpp_OptitrackIneritalFusion_terminate();
}

void Cpp_OptitrackIneritalFusion_atexit_wrapper(void)
{
   Cpp_OptitrackIneritalFusion_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(Cpp_OptitrackIneritalFusion_atexit_wrapper);
  /* Dispatch the entry-point. */
  Cpp_OptitrackIneritalFusion_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_Cpp_OptitrackIneritalFusion_mex.c) */
