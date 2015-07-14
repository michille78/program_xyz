/*
 * GetINSCompensateFromVNS.cpp
 *
 * Code generation for function 'GetINSCompensateFromVNS'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "GetINSCompensateFromVNS.h"

/* Type Definitions */
#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray__common*/

/* Variable Definitions */
static const volatile char_T *emlrtBreakCheckR2012bFlagVar;
static emlrtRTEInfo emlrtRTEI = { 43, 1, "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 33, 73, "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 1, 1, "_coder_GetINSCompensateFromVNS_api",
  "" };

static emlrtBCInfo emlrtBCI = { -1, -1, 53, 28, "InertialData.HipPosition",
  "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 54, 30, "InertialData.HipQuaternion",
  "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 57, 1, "otherMakers1",
  "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 60, 1, "otherMakers1",
  "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 61, 1, "otherMakers1",
  "GetINSCompensateFromVNS",
  "E:\\document and program\\program_xyz\\INS-VNS-MP\\test\\GetINSCompensateFromVNS.m",
  0 };

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *d_emlrt_marshallOut(const emxArray_real_T *u);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *e_emlrt_marshallOut(const real_T u);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *InertialData,
  const char_T *identifier, struct0_T *y);
static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const struct0_T *u);
static void emxCopyMatrix_real_T(real_T dst[3], const real_T src[3]);
static void emxCopyStruct_struct0_T(const emlrtStack *sp, struct0_T *dst, const
  struct0_T *src, const emlrtRTEInfo *srcLocation);
static void emxCopyStruct_struct1_T(const emlrtStack *sp, struct1_T *dst, const
  struct1_T *src, const emlrtRTEInfo *srcLocation);
static void emxCopy_char_T(const emlrtStack *sp, emxArray_char_T **dst,
  emxArray_char_T * const *src, const emlrtRTEInfo *srcLocation);
static void emxCopy_real_T(const emlrtStack *sp, emxArray_real_T **dst,
  emxArray_real_T * const *src, const emlrtRTEInfo *srcLocation);
static void emxEnsureCapacity(const emlrtStack *sp, emxArray__common *emxArray,
  int32_T oldNumel, int32_T elementSize, const emlrtRTEInfo *srcLocation);
static void emxEnsureCapacity_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
static void emxExpand_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  *emxArray, int32_T fromIndex, int32_T toIndex, const emlrtRTEInfo *srcLocation);
static void emxFreeStruct_struct0_T(struct0_T *pStruct);
static void emxFreeStruct_struct1_T(struct1_T *pStruct);
static void emxFree_char_T(emxArray_char_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxFree_struct1_T(emxArray_struct1_T **pEmxArray);
static void emxInitStruct_struct0_T(const emlrtStack *sp, struct0_T *pStruct,
  const emlrtRTEInfo *srcLocation, boolean_T doPush);
static void emxInitStruct_struct1_T(const emlrtStack *sp, struct1_T *pStruct,
  const emlrtRTEInfo *srcLocation, boolean_T doPush);
static void emxInit_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush);
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush);
static void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  **pEmxArray, int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T
  doPush);
static void emxTrim_struct1_T(emxArray_struct1_T *emxArray, int32_T fromIndex,
  int32_T toIndex);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_char_T *y);
static const mxArray *f_emlrt_marshallOut(const emlrtStack *sp, const
  emxArray_struct1_T *u);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static const mxArray *g_emlrt_marshallOut(const struct2_T *u);
static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char_T *identifier, emxArray_struct1_T *y);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_struct1_T *y);
static int32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char_T *identifier);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_char_T *ret);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);
static real_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static int32_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[8] = { "time", "HipQuaternion", "HipPosition",
    "HeadQuaternion", "HeadPosition", "DataStyle", "BodyDirection", "frequency"
  };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(sp, parentId, u, 8, fieldNames, 0U, 0);
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
  thisId.fIdentifier = "DataStyle";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "DataStyle")),
                     &thisId, y->DataStyle);
  thisId.fIdentifier = "BodyDirection";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "BodyDirection")), &thisId, y->BodyDirection);
  thisId.fIdentifier = "frequency";
  y->frequency = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "frequency")), &thisId);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m1;
  real_T *pData;
  int32_T i2;
  int32_T i;
  y = NULL;
  m1 = emlrtCreateNumericArray(2, *(int32_T (*)[2])u->size, mxDOUBLE_CLASS,
    mxREAL);
  pData = (real_T *)mxGetPr(m1);
  i2 = 0;
  for (i = 0; i < u->size[1]; i++) {
    pData[i2] = u->data[u->size[0] * i];
    i2++;
  }

  emlrtAssign(&y, m1);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m2;
  real_T *pData;
  int32_T i3;
  int32_T i;
  int32_T b_i;
  y = NULL;
  m2 = emlrtCreateNumericArray(2, *(int32_T (*)[2])u->size, mxDOUBLE_CLASS,
    mxREAL);
  pData = (real_T *)mxGetPr(m2);
  i3 = 0;
  for (i = 0; i < u->size[1]; i++) {
    for (b_i = 0; b_i < 4; b_i++) {
      pData[i3] = u->data[b_i + u->size[0] * i];
      i3++;
    }
  }

  emlrtAssign(&y, m2);
  return y;
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *d_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m3;
  real_T *pData;
  int32_T i4;
  int32_T i;
  int32_T b_i;
  y = NULL;
  m3 = emlrtCreateNumericArray(2, *(int32_T (*)[2])u->size, mxDOUBLE_CLASS,
    mxREAL);
  pData = (real_T *)mxGetPr(m3);
  i4 = 0;
  for (i = 0; i < u->size[1]; i++) {
    for (b_i = 0; b_i < 3; b_i++) {
      pData[i4] = u->data[b_i + u->size[0] * i];
      i4++;
    }
  }

  emlrtAssign(&y, m3);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *e_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m4;
  y = NULL;
  m4 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m4);
  return y;
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *InertialData,
  const char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  b_emlrt_marshallIn(sp, emlrtAlias(InertialData), &thisId, y);
  emlrtDestroyArray(&InertialData);
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const struct0_T *u)
{
  const mxArray *y;
  emxArray_char_T *b_u;
  int32_T i;
  int32_T loop_ub;
  const mxArray *b_y;
  const mxArray *m0;
  const mxArray *c_y;
  static const int32_T iv1[1] = { 3 };

  real_T *pData;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_char_T(sp, &b_u, 2, (emlrtRTEInfo *)NULL, true);
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  emlrtAddField(y, b_emlrt_marshallOut(u->time), "time", 0);
  emlrtAddField(y, c_emlrt_marshallOut(u->HipQuaternion), "HipQuaternion", 0);
  emlrtAddField(y, d_emlrt_marshallOut(u->HipPosition), "HipPosition", 0);
  emlrtAddField(y, c_emlrt_marshallOut(u->HeadQuaternion), "HeadQuaternion", 0);
  emlrtAddField(y, d_emlrt_marshallOut(u->HeadPosition), "HeadPosition", 0);
  i = b_u->size[0] * b_u->size[1];
  b_u->size[0] = 1;
  b_u->size[1] = u->DataStyle->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)b_u, i, (int32_T)sizeof(char_T),
                    (emlrtRTEInfo *)NULL);
  loop_ub = u->DataStyle->size[0] * u->DataStyle->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_u->data[i] = u->DataStyle->data[i];
  }

  b_y = NULL;
  m0 = emlrtCreateCharArray(2, *(int32_T (*)[2])b_u->size);
  emlrtInitCharArrayR2013a(sp, b_u->size[1], m0, b_u->data);
  emlrtAssign(&b_y, m0);
  emlrtAddField(y, b_y, "DataStyle", 0);
  c_y = NULL;
  m0 = emlrtCreateNumericArray(1, iv1, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m0);
  for (i = 0; i < 3; i++) {
    pData[i] = u->BodyDirection[i];
  }

  emlrtAssign(&c_y, m0);
  emlrtAddField(y, c_y, "BodyDirection", 0);
  emlrtAddField(y, e_emlrt_marshallOut(u->frequency), "frequency", 0);
  emxFree_char_T(&b_u);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return y;
}

static void emxCopyMatrix_real_T(real_T dst[3], const real_T src[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    dst[i] = src[i];
  }
}

static void emxCopyStruct_struct0_T(const emlrtStack *sp, struct0_T *dst, const
  struct0_T *src, const emlrtRTEInfo *srcLocation)
{
  emxCopy_real_T(sp, &dst->time, &src->time, srcLocation);
  emxCopy_real_T(sp, &dst->HipQuaternion, &src->HipQuaternion, srcLocation);
  emxCopy_real_T(sp, &dst->HipPosition, &src->HipPosition, srcLocation);
  emxCopy_real_T(sp, &dst->HeadQuaternion, &src->HeadQuaternion, srcLocation);
  emxCopy_real_T(sp, &dst->HeadPosition, &src->HeadPosition, srcLocation);
  emxCopy_char_T(sp, &dst->DataStyle, &src->DataStyle, srcLocation);
  emxCopyMatrix_real_T(dst->BodyDirection, src->BodyDirection);
  dst->frequency = src->frequency;
}

static void emxCopyStruct_struct1_T(const emlrtStack *sp, struct1_T *dst, const
  struct1_T *src, const emlrtRTEInfo *srcLocation)
{
  dst->frequency = src->frequency;
  emxCopy_real_T(sp, &dst->Position, &src->Position, srcLocation);
  dst->otherMakersN = src->otherMakersN;
  dst->time = src->time;
  emxCopy_char_T(sp, &dst->MarkerSet, &src->MarkerSet, srcLocation);
  emxCopy_real_T(sp, &dst->ContinuesFlag, &src->ContinuesFlag, srcLocation);
  emxCopy_real_T(sp, &dst->ContinuesLastPosition, &src->ContinuesLastPosition,
                 srcLocation);
  emxCopy_real_T(sp, &dst->ContinuesLastTime, &src->ContinuesLastTime,
                 srcLocation);
  emxCopy_real_T(sp, &dst->ContinuesLastK, &src->ContinuesLastK, srcLocation);
}

static void emxCopy_char_T(const emlrtStack *sp, emxArray_char_T **dst,
  emxArray_char_T * const *src, const emlrtRTEInfo *srcLocation)
{
  int32_T numElDst;
  int32_T numElSrc;
  int32_T i;
  numElDst = 1;
  numElSrc = 1;
  for (i = 0; i < (*dst)->numDimensions; i++) {
    numElDst *= (*dst)->size[i];
    numElSrc *= (*src)->size[i];
  }

  for (i = 0; i < (*dst)->numDimensions; i++) {
    (*dst)->size[i] = (*src)->size[i];
  }

  emxEnsureCapacity(sp, (emxArray__common *)*dst, numElDst, (int32_T)sizeof
                    (char_T), srcLocation);
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
  }
}

static void emxCopy_real_T(const emlrtStack *sp, emxArray_real_T **dst,
  emxArray_real_T * const *src, const emlrtRTEInfo *srcLocation)
{
  int32_T numElDst;
  int32_T numElSrc;
  int32_T i;
  numElDst = 1;
  numElSrc = 1;
  for (i = 0; i < (*dst)->numDimensions; i++) {
    numElDst *= (*dst)->size[i];
    numElSrc *= (*src)->size[i];
  }

  for (i = 0; i < (*dst)->numDimensions; i++) {
    (*dst)->size[i] = (*src)->size[i];
  }

  emxEnsureCapacity(sp, (emxArray__common *)*dst, numElDst, (int32_T)sizeof
                    (real_T), srcLocation);
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
  }
}

static void emxEnsureCapacity(const emlrtStack *sp, emxArray__common *emxArray,
  int32_T oldNumel, int32_T elementSize, const emlrtRTEInfo *srcLocation)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((uint32_T)newNumel, (uint32_T)
      emxArray->size[i], (emlrtRTEInfo *)srcLocation, sp);
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i = (int32_T)emlrtSizeMulR2012b((uint32_T)i, 2U, (emlrtRTEInfo *)
        srcLocation, sp);
    }

    newData = emlrtCallocMex((uint32_T)i, (uint32_T)elementSize);
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
    }

    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, (uint32_T)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxEnsureCapacity_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation)
{
  int32_T elementSize;
  int32_T newNumel;
  int32_T i;
  void *newData;
  elementSize = (int32_T)sizeof(struct1_T);
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((uint32_T)newNumel, (uint32_T)
      emxArray->size[i], (emlrtRTEInfo *)srcLocation, sp);
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i = (int32_T)emlrtSizeMulR2012b((uint32_T)i, 2U, (emlrtRTEInfo *)
        srcLocation, sp);
    }

    newData = emlrtCallocMex((uint32_T)i, (uint32_T)elementSize);
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
    }

    if (emxArray->data != NULL) {
      memcpy(newData, (void *)emxArray->data, (uint32_T)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        emlrtFreeMex((void *)emxArray->data);
      }
    }

    emxArray->data = (struct1_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }

  if (oldNumel > newNumel) {
    emxTrim_struct1_T(emxArray, newNumel, oldNumel);
  } else {
    if (oldNumel < newNumel) {
      emxExpand_struct1_T(sp, emxArray, oldNumel, newNumel, srcLocation);
    }
  }
}

static void emxExpand_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  *emxArray, int32_T fromIndex, int32_T toIndex, const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct1_T(sp, &emxArray->data[i], srcLocation, false);
  }
}

static void emxFreeStruct_struct0_T(struct0_T *pStruct)
{
  emxFree_real_T(&pStruct->time);
  emxFree_real_T(&pStruct->HipQuaternion);
  emxFree_real_T(&pStruct->HipPosition);
  emxFree_real_T(&pStruct->HeadQuaternion);
  emxFree_real_T(&pStruct->HeadPosition);
  emxFree_char_T(&pStruct->DataStyle);
}

static void emxFreeStruct_struct1_T(struct1_T *pStruct)
{
  emxFree_real_T(&pStruct->Position);
  emxFree_char_T(&pStruct->MarkerSet);
  emxFree_real_T(&pStruct->ContinuesFlag);
  emxFree_real_T(&pStruct->ContinuesLastPosition);
  emxFree_real_T(&pStruct->ContinuesLastTime);
  emxFree_real_T(&pStruct->ContinuesLastK);
}

static void emxFree_char_T(emxArray_char_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((void *)(*pEmxArray)->data);
    }

    emlrtFreeMex((void *)(*pEmxArray)->size);
    emlrtFreeMex((void *)*pEmxArray);
    *pEmxArray = (emxArray_char_T *)NULL;
  }
}

static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((void *)(*pEmxArray)->data);
    }

    emlrtFreeMex((void *)(*pEmxArray)->size);
    emlrtFreeMex((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

static void emxFree_struct1_T(emxArray_struct1_T **pEmxArray)
{
  int32_T numEl;
  int32_T i;
  if (*pEmxArray != (emxArray_struct1_T *)NULL) {
    if ((*pEmxArray)->data != (struct1_T *)NULL) {
      numEl = 1;
      for (i = 0; i < (*pEmxArray)->numDimensions; i++) {
        numEl *= (*pEmxArray)->size[i];
      }

      for (i = 0; i < numEl; i++) {
        emxFreeStruct_struct1_T(&(*pEmxArray)->data[i]);
      }

      if ((*pEmxArray)->canFreeData) {
        emlrtFreeMex((void *)(*pEmxArray)->data);
      }
    }

    emlrtFreeMex((void *)(*pEmxArray)->size);
    emlrtFreeMex((void *)*pEmxArray);
    *pEmxArray = (emxArray_struct1_T *)NULL;
  }
}

static void emxInitStruct_struct0_T(const emlrtStack *sp, struct0_T *pStruct,
  const emlrtRTEInfo *srcLocation, boolean_T doPush)
{
  emxInit_real_T(sp, &pStruct->time, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->HipQuaternion, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->HipPosition, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->HeadQuaternion, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->HeadPosition, 2, srcLocation, doPush);
  emxInit_char_T(sp, &pStruct->DataStyle, 2, srcLocation, doPush);
}

static void emxInitStruct_struct1_T(const emlrtStack *sp, struct1_T *pStruct,
  const emlrtRTEInfo *srcLocation, boolean_T doPush)
{
  emxInit_real_T(sp, &pStruct->Position, 2, srcLocation, doPush);
  emxInit_char_T(sp, &pStruct->MarkerSet, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->ContinuesFlag, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->ContinuesLastPosition, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->ContinuesLastTime, 2, srcLocation, doPush);
  emxInit_real_T(sp, &pStruct->ContinuesLastK, 2, srcLocation, doPush);
}

static void emxInit_char_T(const emlrtStack *sp, emxArray_char_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush)
{
  emxArray_char_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_char_T *)emlrtMallocMex(sizeof(emxArray_char_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void (*)(void *))
      emxFree_char_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex((uint32_T)(sizeof(int32_T)
    * numDimensions));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocMex(sizeof(emxArray_real_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void (*)(void *))
      emxFree_real_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex((uint32_T)(sizeof(int32_T)
    * numDimensions));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T
  **pEmxArray, int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T
  doPush)
{
  emxArray_struct1_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_struct1_T *)emlrtMallocMex(sizeof(emxArray_struct1_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void (*)(void *))
      emxFree_struct1_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (struct1_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex((uint32_T)(sizeof(int32_T)
    * numDimensions));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b((emlrtRTEInfo *)srcLocation, sp);
  }

  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxTrim_struct1_T(emxArray_struct1_T *emxArray, int32_T fromIndex,
  int32_T toIndex)
{
  int32_T i;
  for (i = fromIndex; i < toIndex; i++) {
    emxFreeStruct_struct1_T(&emxArray->data[i]);
  }
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_char_T *y)
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *f_emlrt_marshallOut(const emlrtStack *sp, const
  emxArray_struct1_T *u)
{
  const mxArray *y;
  emxArray_char_T *b_u;
  int32_T iv2[2];
  int32_T i5;
  int32_T i;
  int32_T b_j1;
  const mxArray *b_y;
  const mxArray *m5;
  int32_T loop_ub;
  const mxArray *c_y;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_char_T(sp, &b_u, 2, (emlrtRTEInfo *)NULL, true);
  y = NULL;
  for (i5 = 0; i5 < 2; i5++) {
    iv2[i5] = u->size[i5];
  }

  emlrtAssign(&y, emlrtCreateStructArray(2, iv2, 0, NULL));
  emlrtCreateField(y, "frequency");
  emlrtCreateField(y, "Position");
  emlrtCreateField(y, "otherMakersN");
  emlrtCreateField(y, "time");
  emlrtCreateField(y, "MarkerSet");
  emlrtCreateField(y, "ContinuesFlag");
  emlrtCreateField(y, "ContinuesLastPosition");
  emlrtCreateField(y, "ContinuesLastTime");
  emlrtCreateField(y, "ContinuesLastK");
  i = 0;
  for (b_j1 = 0; b_j1 < u->size[1U]; b_j1++) {
    emlrtAddField(y, e_emlrt_marshallOut(u->data[u->size[0] * b_j1].frequency),
                  "frequency", i);
    emlrtAddField(y, d_emlrt_marshallOut(u->data[u->size[0] * b_j1].Position),
                  "Position", i);
    b_y = NULL;
    m5 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    *(int32_T *)mxGetData(m5) = u->data[u->size[0] * b_j1].otherMakersN;
    emlrtAssign(&b_y, m5);
    emlrtAddField(y, b_y, "otherMakersN", i);
    emlrtAddField(y, e_emlrt_marshallOut(u->data[u->size[0] * b_j1].time),
                  "time", i);
    i5 = b_u->size[0] * b_u->size[1];
    b_u->size[0] = 1;
    b_u->size[1] = u->data[u->size[0] * b_j1].MarkerSet->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)b_u, i5, (int32_T)sizeof(char_T),
                      (emlrtRTEInfo *)NULL);
    loop_ub = u->data[u->size[0] * b_j1].MarkerSet->size[0] * u->data[u->size[0]
      * b_j1].MarkerSet->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_u->data[i5] = u->data[u->size[0] * b_j1].MarkerSet->data[i5];
    }

    c_y = NULL;
    m5 = emlrtCreateCharArray(2, *(int32_T (*)[2])b_u->size);
    emlrtInitCharArrayR2013a(sp, b_u->size[1], m5, b_u->data);
    emlrtAssign(&c_y, m5);
    emlrtAddField(y, c_y, "MarkerSet", i);
    emlrtAddField(y, b_emlrt_marshallOut(u->data[u->size[0] * b_j1].
      ContinuesFlag), "ContinuesFlag", i);
    emlrtAddField(y, d_emlrt_marshallOut(u->data[u->size[0] * b_j1].
      ContinuesLastPosition), "ContinuesLastPosition", i);
    emlrtAddField(y, b_emlrt_marshallOut(u->data[u->size[0] * b_j1].
      ContinuesLastTime), "ContinuesLastTime", i);
    emlrtAddField(y, b_emlrt_marshallOut(u->data[u->size[0] * b_j1].
      ContinuesLastK), "ContinuesLastK", i);
    i++;
  }

  emxFree_char_T(&b_u);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return y;
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *g_emlrt_marshallOut(const struct2_T *u)
{
  const mxArray *y;
  const mxArray *b_y;
  static const int32_T iv3[2] = { 1, 100 };

  const mxArray *m6;
  real_T *pData;
  int32_T i;
  const mxArray *c_y;
  static const int32_T iv4[2] = { 4, 120 };

  const mxArray *d_y;
  static const int32_T iv5[2] = { 3, 120 };

  const mxArray *e_y;
  static const int32_T iv6[2] = { 4, 120 };

  const mxArray *f_y;
  static const int32_T iv7[2] = { 3, 120 };

  const mxArray *g_y;
  static const int32_T iv8[1] = { 3 };

  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  emlrtAddField(y, e_emlrt_marshallOut(u->frequency), "frequency", 0);
  b_y = NULL;
  m6 = emlrtCreateNumericArray(2, iv3, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 100; i++) {
    pData[i] = u->time[i];
  }

  emlrtAssign(&b_y, m6);
  emlrtAddField(y, b_y, "time", 0);
  c_y = NULL;
  m6 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 480; i++) {
    pData[i] = u->HipQuaternio[i];
  }

  emlrtAssign(&c_y, m6);
  emlrtAddField(y, c_y, "HipQuaternio", 0);
  d_y = NULL;
  m6 = emlrtCreateNumericArray(2, iv5, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 360; i++) {
    pData[i] = u->HipPosition[i];
  }

  emlrtAssign(&d_y, m6);
  emlrtAddField(y, d_y, "HipPosition", 0);
  e_y = NULL;
  m6 = emlrtCreateNumericArray(2, iv6, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 480; i++) {
    pData[i] = u->HeadQuaternion[i];
  }

  emlrtAssign(&e_y, m6);
  emlrtAddField(y, e_y, "HeadQuaternion", 0);
  f_y = NULL;
  m6 = emlrtCreateNumericArray(2, iv7, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 360; i++) {
    pData[i] = u->HeadPosition[i];
  }

  emlrtAssign(&f_y, m6);
  emlrtAddField(y, f_y, "HeadPosition", 0);
  g_y = NULL;
  m6 = emlrtCreateNumericArray(1, iv8, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m6);
  for (i = 0; i < 3; i++) {
    pData[i] = u->BodyDirection[i];
  }

  emlrtAssign(&g_y, m6);
  emlrtAddField(y, g_y, "BodyDirection", 0);
  return y;
}

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *otherMakers,
  const char_T *identifier, emxArray_struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  j_emlrt_marshallIn(sp, emlrtAlias(otherMakers), &thisId, y);
  emlrtDestroyArray(&otherMakers);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  int32_T iv0[2];
  boolean_T bv0[2];
  int32_T i1;
  static const boolean_T bv1[2] = { false, true };

  int32_T sizes[2];
  static const char * fieldNames[9] = { "frequency", "Position", "otherMakersN",
    "time", "MarkerSet", "ContinuesFlag", "ContinuesLastPosition",
    "ContinuesLastTime", "ContinuesLastK" };

  int32_T i;
  int32_T b_j1;
  emxArray_real_T *t0_Position;
  emxArray_char_T *t0_MarkerSet;
  emxArray_real_T *t0_ContinuesFlag;
  emxArray_real_T *t0_ContinuesLastPosition;
  emxArray_real_T *t0_ContinuesLastTime;
  emxArray_real_T *t0_ContinuesLastK;
  struct1_T expl_temp;
  real_T t0_frequency;
  int32_T t0_otherMakersN;
  real_T t0_time;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  thisId.fParent = parentId;
  for (i1 = 0; i1 < 2; i1++) {
    iv0[i1] = 1 + -2 * i1;
    bv0[i1] = bv1[i1];
  }

  emlrtCheckVsStructR2012b(sp, parentId, u, 9, fieldNames, 2U, iv0, bv0, sizes);
  i1 = y->size[0] * y->size[1];
  y->size[1] = sizes[1];
  y->size[0] = 1;
  emxEnsureCapacity_struct1_T(sp, y, i1, (emlrtRTEInfo *)NULL);
  i = 0;
  b_j1 = 0;
  emxInit_real_T(sp, &t0_Position, 2, (emlrtRTEInfo *)NULL, true);
  emxInit_char_T(sp, &t0_MarkerSet, 2, (emlrtRTEInfo *)NULL, true);
  emxInit_real_T(sp, &t0_ContinuesFlag, 2, (emlrtRTEInfo *)NULL, true);
  emxInit_real_T(sp, &t0_ContinuesLastPosition, 2, (emlrtRTEInfo *)NULL, true);
  emxInit_real_T(sp, &t0_ContinuesLastTime, 2, (emlrtRTEInfo *)NULL, true);
  emxInit_real_T(sp, &t0_ContinuesLastK, 2, (emlrtRTEInfo *)NULL, true);
  emxInitStruct_struct1_T(sp, &expl_temp, (emlrtRTEInfo *)NULL, true);
  while (b_j1 < y->size[1U]) {
    thisId.fIdentifier = "frequency";
    t0_frequency = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u,
      i, "frequency")), &thisId);
    thisId.fIdentifier = "Position";
    e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i, "Position")),
                       &thisId, t0_Position);
    thisId.fIdentifier = "otherMakersN";
    t0_otherMakersN = k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp,
      u, i, "otherMakersN")), &thisId);
    thisId.fIdentifier = "time";
    t0_time = h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i,
      "time")), &thisId);
    thisId.fIdentifier = "MarkerSet";
    f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i, "MarkerSet")),
                       &thisId, t0_MarkerSet);
    thisId.fIdentifier = "ContinuesFlag";
    c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i,
      "ContinuesFlag")), &thisId, t0_ContinuesFlag);
    thisId.fIdentifier = "ContinuesLastPosition";
    e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i,
      "ContinuesLastPosition")), &thisId, t0_ContinuesLastPosition);
    thisId.fIdentifier = "ContinuesLastTime";
    c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i,
      "ContinuesLastTime")), &thisId, t0_ContinuesLastTime);
    thisId.fIdentifier = "ContinuesLastK";
    c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, i,
      "ContinuesLastK")), &thisId, t0_ContinuesLastK);
    expl_temp.frequency = t0_frequency;
    i1 = expl_temp.Position->size[0] * expl_temp.Position->size[1];
    expl_temp.Position->size[0] = 3;
    expl_temp.Position->size[1] = t0_Position->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.Position, i1, (int32_T)
                      sizeof(real_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_Position->size[0] * t0_Position->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.Position->data[i1] = t0_Position->data[i1];
    }

    expl_temp.otherMakersN = t0_otherMakersN;
    expl_temp.time = t0_time;
    i1 = expl_temp.MarkerSet->size[0] * expl_temp.MarkerSet->size[1];
    expl_temp.MarkerSet->size[0] = 1;
    expl_temp.MarkerSet->size[1] = t0_MarkerSet->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.MarkerSet, i1, (int32_T)
                      sizeof(char_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_MarkerSet->size[0] * t0_MarkerSet->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.MarkerSet->data[i1] = t0_MarkerSet->data[i1];
    }

    i1 = expl_temp.ContinuesFlag->size[0] * expl_temp.ContinuesFlag->size[1];
    expl_temp.ContinuesFlag->size[0] = 1;
    expl_temp.ContinuesFlag->size[1] = t0_ContinuesFlag->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.ContinuesFlag, i1,
                      (int32_T)sizeof(real_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_ContinuesFlag->size[0] * t0_ContinuesFlag->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.ContinuesFlag->data[i1] = t0_ContinuesFlag->data[i1];
    }

    i1 = expl_temp.ContinuesLastPosition->size[0] *
      expl_temp.ContinuesLastPosition->size[1];
    expl_temp.ContinuesLastPosition->size[0] = 3;
    expl_temp.ContinuesLastPosition->size[1] = t0_ContinuesLastPosition->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.ContinuesLastPosition,
                      i1, (int32_T)sizeof(real_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_ContinuesLastPosition->size[0] * t0_ContinuesLastPosition->
      size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.ContinuesLastPosition->data[i1] = t0_ContinuesLastPosition->
        data[i1];
    }

    i1 = expl_temp.ContinuesLastTime->size[0] *
      expl_temp.ContinuesLastTime->size[1];
    expl_temp.ContinuesLastTime->size[0] = 1;
    expl_temp.ContinuesLastTime->size[1] = t0_ContinuesLastTime->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.ContinuesLastTime, i1,
                      (int32_T)sizeof(real_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_ContinuesLastTime->size[0] * t0_ContinuesLastTime->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.ContinuesLastTime->data[i1] = t0_ContinuesLastTime->data[i1];
    }

    i1 = expl_temp.ContinuesLastK->size[0] * expl_temp.ContinuesLastK->size[1];
    expl_temp.ContinuesLastK->size[0] = 1;
    expl_temp.ContinuesLastK->size[1] = t0_ContinuesLastK->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)expl_temp.ContinuesLastK, i1,
                      (int32_T)sizeof(real_T), (emlrtRTEInfo *)NULL);
    loop_ub = t0_ContinuesLastK->size[0] * t0_ContinuesLastK->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      expl_temp.ContinuesLastK->data[i1] = t0_ContinuesLastK->data[i1];
    }

    emxCopyStruct_struct1_T(sp, &y->data[y->size[0] * b_j1], &expl_temp,
      (emlrtRTEInfo *)NULL);
    i++;
    b_j1++;
  }

  emxFreeStruct_struct1_T(&expl_temp);
  emxFree_real_T(&t0_ContinuesLastK);
  emxFree_real_T(&t0_ContinuesLastTime);
  emxFree_real_T(&t0_ContinuesLastPosition);
  emxFree_real_T(&t0_ContinuesFlag);
  emxFree_char_T(&t0_MarkerSet);
  emxFree_real_T(&t0_Position);
  emlrtDestroyArray(&u);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static int32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *compensateRate, const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(compensateRate), &thisId);
  emlrtDestroyArray(&compensateRate);
  return y;
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  int32_T iv9[2];
  boolean_T bv2[2];
  int32_T i6;
  static const boolean_T bv3[2] = { false, true };

  int32_T iv10[2];
  for (i6 = 0; i6 < 2; i6++) {
    iv9[i6] = 1 + -2 * i6;
    bv2[i6] = bv3[i6];
  }

  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv9, bv2, iv10);
  i6 = ret->size[0] * ret->size[1];
  ret->size[0] = iv10[0];
  ret->size[1] = iv10[1];
  emxEnsureCapacity(sp, (emxArray__common *)ret, i6, (int32_T)sizeof(real_T),
                    (emlrtRTEInfo *)NULL);
  emlrtImportArrayR2011b(src, ret->data, 8, false);
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  int32_T iv11[2];
  boolean_T bv4[2];
  int32_T i7;
  static const boolean_T bv5[2] = { false, true };

  int32_T iv12[2];
  for (i7 = 0; i7 < 2; i7++) {
    iv11[i7] = 4 + -5 * i7;
    bv4[i7] = bv5[i7];
  }

  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv11, bv4, iv12);
  i7 = ret->size[0] * ret->size[1];
  ret->size[0] = iv12[0];
  ret->size[1] = iv12[1];
  emxEnsureCapacity(sp, (emxArray__common *)ret, i7, (int32_T)sizeof(real_T),
                    (emlrtRTEInfo *)NULL);
  emlrtImportArrayR2011b(src, ret->data, 8, false);
  emlrtDestroyArray(&src);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  int32_T iv13[2];
  boolean_T bv6[2];
  int32_T i8;
  static const boolean_T bv7[2] = { false, true };

  int32_T iv14[2];
  for (i8 = 0; i8 < 2; i8++) {
    iv13[i8] = 3 + -4 * i8;
    bv6[i8] = bv7[i8];
  }

  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv13, bv6, iv14);
  i8 = ret->size[0] * ret->size[1];
  ret->size[0] = iv14[0];
  ret->size[1] = iv14[1];
  emxEnsureCapacity(sp, (emxArray__common *)ret, i8, (int32_T)sizeof(real_T),
                    (emlrtRTEInfo *)NULL);
  emlrtImportArrayR2011b(src, ret->data, 8, false);
  emlrtDestroyArray(&src);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_char_T *ret)
{
  int32_T iv15[2];
  boolean_T bv8[2];
  int32_T i9;
  static const boolean_T bv9[2] = { false, true };

  int32_T iv16[2];
  for (i9 = 0; i9 < 2; i9++) {
    iv15[i9] = 1 + -2 * i9;
    bv8[i9] = bv9[i9];
  }

  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "char", false, 2U, iv15, bv8, iv16);
  i9 = ret->size[0] * ret->size[1];
  ret->size[0] = iv16[0];
  ret->size[1] = iv16[1];
  emxEnsureCapacity(sp, (emxArray__common *)ret, i9, (int32_T)sizeof(char_T),
                    (emlrtRTEInfo *)NULL);
  emlrtImportArrayR2011b(src, ret->data, 1, false);
  emlrtDestroyArray(&src);
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  int32_T iv17[1];
  int32_T i10;
  iv17[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv17);
  for (i10 = 0; i10 < 3; i10++) {
    ret[i10] = (*(real_T (*)[3])mxGetData(src))[i10];
  }

  emlrtDestroyArray(&src);
}

static real_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static int32_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  int32_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 0U, 0);
  ret = *(int32_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void GetINSCompensateFromVNS(const emlrtStack *sp, const struct0_T *InertialData,
  const emxArray_struct1_T *otherMakers, real_T compensateRate, struct0_T
  *InertialData1, emxArray_struct1_T *otherMakers1, real_T *compensateRate1,
  struct2_T *InertialData2)
{
  int32_T i;
  int32_T loop_ub;
  struct1_T b_otherMakers1;
  struct1_T c_otherMakers1;
  static const real_T dv0[6] = { 1.0, 2.0, 3.0, 0.1, 0.2, 0.3 };

  struct1_T d_otherMakers1;
  struct1_T e_otherMakers1;
  static const char_T cv0[4] = { 'H', 'e', 'a', 'd' };

  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* % xyz 2015 5.25 */
  /* % otherMakers */
  /*  otherMakers(k).frequency [1] */
  /*  otherMakers(k).Position  [3*M] */
  /*  otherMakers(k).otherMakersN [1] */
  /* 　otherMakers(k).time [1] */
  /* 　otherMakers(k).MarkerSet "" */
  /*  记录每个马克点的连续特性 */
  /*  otherMakers(k).ContinuesFlag = zeros(1,M) ; % 不连续 */
  /*  otherMakers(k).ContinuesLastPosition = NaN(3,M)  ; */
  /*  otherMakers(k).ContinuesLastTime = NaN[1*M] ;  */
  /*  otherMakers(k).ContinuesLastK = NaN[1*M]; */
  /* % InertialData */
  /*  InertialData.frequency (k) */
  /*  InertialData.time (k) */
  /*  InertialData.HipQuaternion(k)  [4*N] */
  /*  InertialData.HipPosition (k)  [3*N] */
  /*  InertialData.HeadQuaternion (k)  [4*N] */
  /*  InertialData.HeadPosition (k)  [3*N] */
  /*  InertialData.BodyDirection(k)  [3*1] */
  /* % 用 Optitrack 的 OtherMarker 补偿惯性系统 */
  /* % 得到惯性 Hip 位置补偿量 InertialPositionCompensate */
  /*  InertialPositionCompensate [ 3*N ]  m  NED系 */
  /* % InertialData 格式数据创建 */
  /* % InertialData 格式数据创建 */
  InertialData2->frequency = 96.0;
  for (i = 0; i < 100; i++) {
    InertialData2->time[i] = 1.0 + (real_T)i;
  }

  for (i = 0; i < 480; i++) {
    InertialData2->HipQuaternio[i] = 0.0;
  }

  for (i = 0; i < 360; i++) {
    InertialData2->HipPosition[i] = 0.0;
  }

  for (i = 0; i < 480; i++) {
    InertialData2->HeadQuaternion[i] = 0.0;
  }

  for (i = 0; i < 360; i++) {
    InertialData2->HeadPosition[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    InertialData2->BodyDirection[i] = 0.0;
  }

  /* % InertialData 格式数据Copy */
  emxCopyStruct_struct0_T(sp, InertialData1, InertialData, &emlrtRTEI);

  /* % otherMakers 格式数据Copy */
  i = otherMakers1->size[0] * otherMakers1->size[1];
  otherMakers1->size[0] = 1;
  otherMakers1->size[1] = otherMakers->size[1];
  emxEnsureCapacity_struct1_T(sp, otherMakers1, i, &b_emlrtRTEI);
  loop_ub = otherMakers->size[0] * otherMakers->size[1];
  for (i = 0; i < loop_ub; i++) {
    emxCopyStruct_struct1_T(sp, &otherMakers1->data[i], &otherMakers->data[i],
      &b_emlrtRTEI);
  }

  emxInitStruct_struct1_T(sp, &b_otherMakers1, &b_emlrtRTEI, true);
  emxInitStruct_struct1_T(sp, &c_otherMakers1, &b_emlrtRTEI, true);

  /* %  */
  *compensateRate1 = compensateRate;

  /* % InertialData 数据赋值 */
  InertialData1->frequency = 96.0;
  i = InertialData->HipPosition->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i, (emlrtBCInfo *)&emlrtBCI, sp);
  i = InertialData->HipQuaternion->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i, (emlrtBCInfo *)&b_emlrtBCI, sp);

  /* % otherMakers 数据赋值 */
  i = otherMakers->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i, (emlrtBCInfo *)&c_emlrtBCI, sp);
  otherMakers1->data[0].frequency = 30.0;
  i = otherMakers->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i, (emlrtBCInfo *)&d_emlrtBCI, sp);
  i = otherMakers1->data[0].Position->size[0] * otherMakers1->data[0]
    .Position->size[1];
  otherMakers1->data[0].Position->size[0] = 3;
  emxEnsureCapacity(sp, (emxArray__common *)otherMakers1->data[0].Position, i,
                    (int32_T)sizeof(real_T), &b_emlrtRTEI);
  i = otherMakers1->data[0].Position->size[0] * otherMakers1->data[0]
    .Position->size[1];
  otherMakers1->data[0].Position->size[1] = 2;
  emxEnsureCapacity(sp, (emxArray__common *)otherMakers1->data[0].Position, i,
                    (int32_T)sizeof(real_T), &b_emlrtRTEI);
  emxCopyStruct_struct1_T(sp, &b_otherMakers1, &otherMakers1->data[0],
    &b_emlrtRTEI);
  emxCopyStruct_struct1_T(sp, &c_otherMakers1, &otherMakers1->data[0],
    &b_emlrtRTEI);
  emxFreeStruct_struct1_T(&c_otherMakers1);
  emxFreeStruct_struct1_T(&b_otherMakers1);
  for (i = 0; i < 6; i++) {
    otherMakers1->data[0].Position->data[i] = dv0[i];
  }

  emxInitStruct_struct1_T(sp, &d_otherMakers1, &b_emlrtRTEI, true);
  emxInitStruct_struct1_T(sp, &e_otherMakers1, &b_emlrtRTEI, true);
  i = otherMakers->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i, (emlrtBCInfo *)&e_emlrtBCI, sp);
  i = otherMakers1->data[0].MarkerSet->size[0] * otherMakers1->data[0].
    MarkerSet->size[1];
  otherMakers1->data[0].MarkerSet->size[0] = 1;
  emxEnsureCapacity(sp, (emxArray__common *)otherMakers1->data[0].MarkerSet, i,
                    (int32_T)sizeof(char_T), &b_emlrtRTEI);
  i = otherMakers1->data[0].MarkerSet->size[0] * otherMakers1->data[0].
    MarkerSet->size[1];
  otherMakers1->data[0].MarkerSet->size[1] = 4;
  emxEnsureCapacity(sp, (emxArray__common *)otherMakers1->data[0].MarkerSet, i,
                    (int32_T)sizeof(char_T), &b_emlrtRTEI);
  emxCopyStruct_struct1_T(sp, &d_otherMakers1, &otherMakers1->data[0],
    &b_emlrtRTEI);
  emxCopyStruct_struct1_T(sp, &e_otherMakers1, &otherMakers1->data[0],
    &b_emlrtRTEI);
  emxFreeStruct_struct1_T(&e_otherMakers1);
  emxFreeStruct_struct1_T(&d_otherMakers1);
  for (i = 0; i < 4; i++) {
    otherMakers1->data[0].MarkerSet->data[i] = cv0[i];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void GetINSCompensateFromVNS_api(const mxArray * const prhs[3], const mxArray
  *plhs[4])
{
  struct0_T InertialData;
  emxArray_struct1_T *otherMakers;
  struct0_T b_InertialData;
  emxArray_struct1_T *b_otherMakers;
  real_T compensateRate;
  int32_T i0;
  int32_T loop_ub;
  struct2_T InertialData2;
  real_T compensateRate1;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInitStruct_struct0_T(&st, &InertialData, &c_emlrtRTEI, true);
  emxInit_struct1_T(&st, &otherMakers, 2, &c_emlrtRTEI, true);
  emxInitStruct_struct0_T(&st, &b_InertialData, &c_emlrtRTEI, true);
  emxInit_struct1_T(&st, &b_otherMakers, 2, &c_emlrtRTEI, true);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "InertialData",
                   &InertialData);
  i_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "otherMakers",
                     otherMakers);
  compensateRate = l_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "compensateRate");

  /* Invoke the target function */
  emxCopyStruct_struct0_T(&st, &b_InertialData, &InertialData, &c_emlrtRTEI);
  i0 = b_otherMakers->size[0] * b_otherMakers->size[1];
  b_otherMakers->size[0] = 1;
  b_otherMakers->size[1] = otherMakers->size[1];
  emxEnsureCapacity_struct1_T(&st, b_otherMakers, i0, &c_emlrtRTEI);
  loop_ub = otherMakers->size[0] * otherMakers->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    emxCopyStruct_struct1_T(&st, &b_otherMakers->data[i0], &otherMakers->data[i0],
      &c_emlrtRTEI);
  }

  GetINSCompensateFromVNS(&st, &b_InertialData, b_otherMakers, compensateRate,
    &InertialData, otherMakers, &compensateRate1, &InertialData2);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(&st, &InertialData);
  plhs[1] = f_emlrt_marshallOut(&st, otherMakers);
  plhs[2] = e_emlrt_marshallOut(compensateRate1);
  plhs[3] = g_emlrt_marshallOut(&InertialData2);
  emxFree_struct1_T(&b_otherMakers);
  emxFree_struct1_T(&otherMakers);
  emxFreeStruct_struct0_T(&InertialData);
  emxFreeStruct_struct0_T(&b_InertialData);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

void GetINSCompensateFromVNS_atexit()
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void GetINSCompensateFromVNS_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void GetINSCompensateFromVNS_terminate()
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (GetINSCompensateFromVNS.cpp) */
