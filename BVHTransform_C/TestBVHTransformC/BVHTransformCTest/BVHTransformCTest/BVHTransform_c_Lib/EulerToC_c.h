/*
 * File: EulerToC_c.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

#ifndef __EULERTOC_C_H__
#define __EULERTOC_C_H__

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
extern void EulerToC_c(const double euler[6], const double rotateDirection[3],
  double C[9]);
extern void b_EulerToC_c(const double euler[3], const double rotateDirection[3],
  double C[9]);
extern void c_EulerToC_c(const double euler[3], const double rotateDirection[3],
  double C[9]);
extern void d_EulerToC_c(const double euler[3], const double rotateDirection[3],
  double C[9]);
extern void e_EulerToC_c(const double euler[3], const double rotateDirection[3],
  double C[9]);

#endif

/*
 * File trailer for EulerToC_c.h
 *
 * [EOF]
 */
