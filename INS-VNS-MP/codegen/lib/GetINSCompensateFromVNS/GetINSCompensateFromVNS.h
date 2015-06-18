//
// File: GetINSCompensateFromVNS.h
//
// MATLAB Coder version            : 2.6
// C/C++ source code generated on  : 18-Jun-2015 20:22:25
//
#ifndef __GETINSCOMPENSATEFROMVNS_H__
#define __GETINSCOMPENSATEFROMVNS_H__

// Include files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "GetINSCompensateFromVNS_types.h"

// Function Declarations
extern void GetINSCompensateFromVNS(const struct0_T *InertialData, struct1_T
  otherMakers[3600], double compensateRate, const struct2_T *CalculateOrder,
  double b_InertialPositionCompensate[34560], double b_HipDisplacementNew[34560]);
extern void GetINSCompensateFromVNS_initialize();
extern void GetINSCompensateFromVNS_terminate();

#endif

//
// File trailer for GetINSCompensateFromVNS.h
//
// [EOF]
//
