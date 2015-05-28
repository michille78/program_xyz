/*
 * File: GenThumbDirection.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 27-May-2015 16:37:00
 */

#ifndef __GENTHUMBDIRECTION_H__
#define __GENTHUMBDIRECTION_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "GenThumbDirection_types.h"

/* Function Declarations */
extern void GenThumbDirection(float rotateAngle[3], const float zd[3], char
  RightOrLeft, char pose, float bone_w[9]);
extern void GenThumbDirection_initialize(void);
extern void GenThumbDirection_terminate(void);

#endif

/*
 * File trailer for GenThumbDirection.h
 *
 * [EOF]
 */
