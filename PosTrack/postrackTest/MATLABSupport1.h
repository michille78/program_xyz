
#pragma once


#include "rt_nonfinite.h"
#include "GetINSCompensateFromVNS.h"

extern struct emxArray__common;
  void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
	elementSize);
 void emxEnsureCapacity_struct1_T(emxArray_struct1_T *emxArray, int
	oldNumel);

 extern void emxInitStruct_struct1_T(struct1_T *pStruct);
 void emxCopyStruct_struct1_T(struct1_T *dst, const struct1_T *src);
 void emxFreeStruct_struct1_T(struct1_T *pStruct);


 void c_Set_InertialData_otherMakers_(struct0_T *InertialData,
	emxArray_struct1_T *otherMakers, int I_N, int V_N);

 void Assign_otherMakers_1(emxArray_struct1_T *otherMakers, double
	visual_k, double frequency, double time, const emxArray_real_T *Position, int
	otherMakersN);


 void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);


