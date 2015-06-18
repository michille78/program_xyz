//
// File: GetINSCompensateFromVNS.h
//
// MATLAB Coder version            : 2.6
// C/C++ source code generated on  : 15-Jun-2015 10:51:25
//
#ifndef __GETINSCOMPENSATEFROMVNS_H__
#define __GETINSCOMPENSATEFROMVNS_H__

// Include files
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "GetINSCompensateFromVNS_types.h"

// Function Declarations
extern void GetINSCompensateFromVNS(const struct0_T *InertialData, const
  struct1_T otherMakers[3600], double compensateRate, struct0_T *InertialData1,
  struct1_T otherMakers1[3600], double *compensateRate1, struct2_T
  *InertialData2, struct1_T otherMakers2_data[], int otherMakers2_size[2]);
extern void GetINSCompensateFromVNS_initialize();
extern void GetINSCompensateFromVNS_terminate();

#endif

//
// File trailer for GetINSCompensateFromVNS.h
//
// [EOF]
//
