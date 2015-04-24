/*
 * _coder_pythagoras_info.c
 *
 * Code generation for function 'pythagoras'
 *
 */

/* Include files */
#include "_coder_pythagoras_info.h"

/* Function Declarations */
static void info_helper(const mxArray **info);
static const mxArray *emlrt_marshallOut(const char * u);
static const mxArray *b_emlrt_marshallOut(const uint32_T u);

/* Function Definitions */
const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  nameCaptureInfo = NULL;
  emlrtAssign(&nameCaptureInfo, emlrtCreateStructMatrix(16, 1, 0, NULL));
  info_helper(&nameCaptureInfo);
  emlrtNameCapturePostProcessR2013b(&nameCaptureInfo);
  return nameCaptureInfo;
}

static void info_helper(const mxArray **info)
{
  const mxArray *rhs0 = NULL;
  const mxArray *lhs0 = NULL;
  const mxArray *rhs1 = NULL;
  const mxArray *lhs1 = NULL;
  const mxArray *rhs2 = NULL;
  const mxArray *lhs2 = NULL;
  const mxArray *rhs3 = NULL;
  const mxArray *lhs3 = NULL;
  const mxArray *rhs4 = NULL;
  const mxArray *lhs4 = NULL;
  const mxArray *rhs5 = NULL;
  const mxArray *lhs5 = NULL;
  const mxArray *rhs6 = NULL;
  const mxArray *lhs6 = NULL;
  const mxArray *rhs7 = NULL;
  const mxArray *lhs7 = NULL;
  const mxArray *rhs8 = NULL;
  const mxArray *lhs8 = NULL;
  const mxArray *rhs9 = NULL;
  const mxArray *lhs9 = NULL;
  const mxArray *rhs10 = NULL;
  const mxArray *lhs10 = NULL;
  const mxArray *rhs11 = NULL;
  const mxArray *lhs11 = NULL;
  const mxArray *rhs12 = NULL;
  const mxArray *lhs12 = NULL;
  const mxArray *rhs13 = NULL;
  const mxArray *lhs13 = NULL;
  const mxArray *rhs14 = NULL;
  const mxArray *lhs14 = NULL;
  const mxArray *rhs15 = NULL;
  const mxArray *lhs15 = NULL;
  emlrtAddField(*info, emlrt_marshallOut(
    "[E]E:/document and program/program_xyz_noitom/BVHTransform/BVHTransformCTest/pythagoras.m"),
                "context", 0);
  emlrtAddField(*info, emlrt_marshallOut("mpower"), "name", 0);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 0);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved", 0);
  emlrtAddField(*info, b_emlrt_marshallOut(1363717478U), "fileTimeLo", 0);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 0);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 0);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 0);
  emlrtAssign(&rhs0, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs0, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs0), "rhs", 0);
  emlrtAddField(*info, emlrtAliasP(lhs0), "lhs", 0);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context", 1);
  emlrtAddField(*info, emlrt_marshallOut("coder.internal.isBuiltInNumeric"),
                "name", 1);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 1);
  emlrtAddField(*info, emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                "resolved", 1);
  emlrtAddField(*info, b_emlrt_marshallOut(1363718156U), "fileTimeLo", 1);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 1);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 1);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 1);
  emlrtAssign(&rhs1, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs1, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs1), "rhs", 1);
  emlrtAddField(*info, emlrtAliasP(lhs1), "lhs", 1);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context", 2);
  emlrtAddField(*info, emlrt_marshallOut("ismatrix"), "name", 2);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 2);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved", 2);
  emlrtAddField(*info, b_emlrt_marshallOut(1331308458U), "fileTimeLo", 2);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 2);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 2);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 2);
  emlrtAssign(&rhs2, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs2, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs2), "rhs", 2);
  emlrtAddField(*info, emlrtAliasP(lhs2), "lhs", 2);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context", 3);
  emlrtAddField(*info, emlrt_marshallOut("power"), "name", 3);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 3);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved", 3);
  emlrtAddField(*info, b_emlrt_marshallOut(1363717480U), "fileTimeLo", 3);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 3);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 3);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 3);
  emlrtAssign(&rhs3, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs3, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs3), "rhs", 3);
  emlrtAddField(*info, emlrtAliasP(lhs3), "lhs", 3);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context", 4);
  emlrtAddField(*info, emlrt_marshallOut("coder.internal.isBuiltInNumeric"),
                "name", 4);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 4);
  emlrtAddField(*info, emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                "resolved", 4);
  emlrtAddField(*info, b_emlrt_marshallOut(1363718156U), "fileTimeLo", 4);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 4);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 4);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 4);
  emlrtAssign(&rhs4, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs4, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs4), "rhs", 4);
  emlrtAddField(*info, emlrtAliasP(lhs4), "lhs", 4);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                5);
  emlrtAddField(*info, emlrt_marshallOut("eml_scalar_eg"), "name", 5);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 5);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                5);
  emlrtAddField(*info, b_emlrt_marshallOut(1375987888U), "fileTimeLo", 5);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 5);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 5);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 5);
  emlrtAssign(&rhs5, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs5, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs5), "rhs", 5);
  emlrtAddField(*info, emlrtAliasP(lhs5), "lhs", 5);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                6);
  emlrtAddField(*info, emlrt_marshallOut("coder.internal.scalarEg"), "name", 6);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 6);
  emlrtAddField(*info, emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                "resolved", 6);
  emlrtAddField(*info, b_emlrt_marshallOut(1389311520U), "fileTimeLo", 6);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 6);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 6);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 6);
  emlrtAssign(&rhs6, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs6, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs6), "rhs", 6);
  emlrtAddField(*info, emlrtAliasP(lhs6), "lhs", 6);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                7);
  emlrtAddField(*info, emlrt_marshallOut("eml_scalexp_alloc"), "name", 7);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 7);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                "resolved", 7);
  emlrtAddField(*info, b_emlrt_marshallOut(1375987888U), "fileTimeLo", 7);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 7);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 7);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 7);
  emlrtAssign(&rhs7, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs7, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs7), "rhs", 7);
  emlrtAddField(*info, emlrtAliasP(lhs7), "lhs", 7);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                "context", 8);
  emlrtAddField(*info, emlrt_marshallOut("coder.internal.scalexpAlloc"), "name",
                8);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 8);
  emlrtAddField(*info, emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                "resolved", 8);
  emlrtAddField(*info, b_emlrt_marshallOut(1389311520U), "fileTimeLo", 8);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 8);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 8);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 8);
  emlrtAssign(&rhs8, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs8, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs8), "rhs", 8);
  emlrtAddField(*info, emlrtAliasP(lhs8), "lhs", 8);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                9);
  emlrtAddField(*info, emlrt_marshallOut("floor"), "name", 9);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 9);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved", 9);
  emlrtAddField(*info, b_emlrt_marshallOut(1363717454U), "fileTimeLo", 9);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 9);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 9);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 9);
  emlrtAssign(&rhs9, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs9, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs9), "rhs", 9);
  emlrtAddField(*info, emlrtAliasP(lhs9), "lhs", 9);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context", 10);
  emlrtAddField(*info, emlrt_marshallOut("coder.internal.isBuiltInNumeric"),
                "name", 10);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 10);
  emlrtAddField(*info, emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                "resolved", 10);
  emlrtAddField(*info, b_emlrt_marshallOut(1363718156U), "fileTimeLo", 10);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 10);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 10);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 10);
  emlrtAssign(&rhs10, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs10, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs10), "rhs", 10);
  emlrtAddField(*info, emlrtAliasP(lhs10), "lhs", 10);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context", 11);
  emlrtAddField(*info, emlrt_marshallOut("eml_scalar_floor"), "name", 11);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 11);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                "resolved", 11);
  emlrtAddField(*info, b_emlrt_marshallOut(1286825926U), "fileTimeLo", 11);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 11);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 11);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 11);
  emlrtAssign(&rhs11, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs11, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs11), "rhs", 11);
  emlrtAddField(*info, emlrtAliasP(lhs11), "lhs", 11);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                "context", 12);
  emlrtAddField(*info, emlrt_marshallOut("eml_scalar_eg"), "name", 12);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 12);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                12);
  emlrtAddField(*info, b_emlrt_marshallOut(1375987888U), "fileTimeLo", 12);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 12);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 12);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 12);
  emlrtAssign(&rhs12, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs12, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs12), "rhs", 12);
  emlrtAddField(*info, emlrtAliasP(lhs12), "lhs", 12);
  emlrtAddField(*info, emlrt_marshallOut(
    "[E]E:/document and program/program_xyz_noitom/BVHTransform/BVHTransformCTest/pythagoras.m"),
                "context", 13);
  emlrtAddField(*info, emlrt_marshallOut("sqrt"), "name", 13);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 13);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved", 13);
  emlrtAddField(*info, b_emlrt_marshallOut(1343837586U), "fileTimeLo", 13);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 13);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 13);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 13);
  emlrtAssign(&rhs13, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs13, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs13), "rhs", 13);
  emlrtAddField(*info, emlrtAliasP(lhs13), "lhs", 13);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context", 14);
  emlrtAddField(*info, emlrt_marshallOut("eml_error"), "name", 14);
  emlrtAddField(*info, emlrt_marshallOut("char"), "dominantType", 14);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved", 14);
  emlrtAddField(*info, b_emlrt_marshallOut(1343837558U), "fileTimeLo", 14);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 14);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 14);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 14);
  emlrtAssign(&rhs14, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs14, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs14), "rhs", 14);
  emlrtAddField(*info, emlrtAliasP(lhs14), "lhs", 14);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context", 15);
  emlrtAddField(*info, emlrt_marshallOut("eml_scalar_sqrt"), "name", 15);
  emlrtAddField(*info, emlrt_marshallOut("double"), "dominantType", 15);
  emlrtAddField(*info, emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                "resolved", 15);
  emlrtAddField(*info, b_emlrt_marshallOut(1286825938U), "fileTimeLo", 15);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "fileTimeHi", 15);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeLo", 15);
  emlrtAddField(*info, b_emlrt_marshallOut(0U), "mFileTimeHi", 15);
  emlrtAssign(&rhs15, emlrtCreateCellMatrix(0, 1));
  emlrtAssign(&lhs15, emlrtCreateCellMatrix(0, 1));
  emlrtAddField(*info, emlrtAliasP(rhs15), "rhs", 15);
  emlrtAddField(*info, emlrtAliasP(lhs15), "lhs", 15);
  emlrtDestroyArray(&rhs0);
  emlrtDestroyArray(&lhs0);
  emlrtDestroyArray(&rhs1);
  emlrtDestroyArray(&lhs1);
  emlrtDestroyArray(&rhs2);
  emlrtDestroyArray(&lhs2);
  emlrtDestroyArray(&rhs3);
  emlrtDestroyArray(&lhs3);
  emlrtDestroyArray(&rhs4);
  emlrtDestroyArray(&lhs4);
  emlrtDestroyArray(&rhs5);
  emlrtDestroyArray(&lhs5);
  emlrtDestroyArray(&rhs6);
  emlrtDestroyArray(&lhs6);
  emlrtDestroyArray(&rhs7);
  emlrtDestroyArray(&lhs7);
  emlrtDestroyArray(&rhs8);
  emlrtDestroyArray(&lhs8);
  emlrtDestroyArray(&rhs9);
  emlrtDestroyArray(&lhs9);
  emlrtDestroyArray(&rhs10);
  emlrtDestroyArray(&lhs10);
  emlrtDestroyArray(&rhs11);
  emlrtDestroyArray(&lhs11);
  emlrtDestroyArray(&rhs12);
  emlrtDestroyArray(&lhs12);
  emlrtDestroyArray(&rhs13);
  emlrtDestroyArray(&lhs13);
  emlrtDestroyArray(&rhs14);
  emlrtDestroyArray(&lhs14);
  emlrtDestroyArray(&rhs15);
  emlrtDestroyArray(&lhs15);
}

static const mxArray *emlrt_marshallOut(const char * u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateString(u);
  emlrtAssign(&y, m0);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const uint32_T u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *(uint32_T *)mxGetData(m1) = u;
  emlrtAssign(&y, m1);
  return y;
}

MEXFUNCTION_LINKAGE mxArray *emlrtMexFcnProperties(void);
mxArray *emlrtMexFcnProperties()
{
  const char *mexProperties[] = {
    "Version",
    "ResolvedFunctions",
    "EntryPoints",
    "CoverageInfo",
    NULL };

  const char *epProperties[] = {
    "Name",
    "NumberOfInputs",
    "NumberOfOutputs",
    "ConstantInputs" };

  mxArray *xResult = mxCreateStructMatrix(1,1,4,mexProperties);
  mxArray *xEntryPoints = mxCreateStructMatrix(1,1,4,epProperties);
  mxArray *xInputs = NULL;
  xInputs = mxCreateLogicalMatrix(1, 3);
  mxSetFieldByNumber(xEntryPoints, 0, 0, mxCreateString("pythagoras"));
  mxSetFieldByNumber(xEntryPoints, 0, 1, mxCreateDoubleScalar(3));
  mxSetFieldByNumber(xEntryPoints, 0, 2, mxCreateDoubleScalar(1));
  mxSetFieldByNumber(xEntryPoints, 0, 3, xInputs);
  mxSetFieldByNumber(xResult, 0, 0, mxCreateString("8.3.0.532 (R2014a)"));
  mxSetFieldByNumber(xResult, 0, 1, (mxArray*)emlrtMexFcnResolvedFunctionsInfo());
  mxSetFieldByNumber(xResult, 0, 2, xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_pythagoras_info.c) */
