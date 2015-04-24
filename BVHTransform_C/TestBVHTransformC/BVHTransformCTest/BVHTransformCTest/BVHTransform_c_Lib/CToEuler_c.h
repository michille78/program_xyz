/*
 * File: CToEuler_c.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

#ifndef __CTOEULER_C_H__
#define __CTOEULER_C_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "BVHTransform_c_types.h"

/* Function Declarations */
extern void CToEuler_c(const double C[9], const double rotateDirection[3],
  double euler[3]);
extern void b_CToEuler_c(const double C[9], const double rotateDirection[3],
  double euler[3]);
extern void c_CToEuler_c(const double C[9], const double rotateDirection[3],
  double euler[3]);
extern void d_CToEuler_c(const double C[9], const double rotateDirection[3],
  double euler[3]);

#endif

/*
 * File trailer for CToEuler_c.h
 *
 * [EOF]
 */
