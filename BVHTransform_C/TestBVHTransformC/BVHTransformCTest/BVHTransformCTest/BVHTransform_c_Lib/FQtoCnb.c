/*
 * File: FQtoCnb.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "FQtoCnb.h"

/* Function Definitions */

/*
 * format long
 * Arguments    : double Q[4]
 *                double Cnb[9]
 * Return Type  : void
 */
void FQtoCnb(double Q[4], double Cnb[9])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;

  /*  buaa xyz 2014.1.10 */
  /*  四元数 -> 方向余弦矩阵 */
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = fabs(Q[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * sqrt(y);
  for (k = 0; k < 4; k++) {
    Q[k] /= y;
  }

  Cnb[0] = ((Q[0] * Q[0] + Q[1] * Q[1]) - Q[2] * Q[2]) - Q[3] * Q[3];
  Cnb[3] = 2.0 * (Q[1] * Q[2] + Q[0] * Q[3]);
  Cnb[6] = 2.0 * (Q[1] * Q[3] - Q[0] * Q[2]);
  Cnb[1] = 2.0 * (Q[1] * Q[2] - Q[0] * Q[3]);
  Cnb[4] = ((Q[0] * Q[0] - Q[1] * Q[1]) + Q[2] * Q[2]) - Q[3] * Q[3];
  Cnb[7] = 2.0 * (Q[2] * Q[3] + Q[0] * Q[1]);
  Cnb[2] = 2.0 * (Q[1] * Q[3] + Q[0] * Q[2]);
  Cnb[5] = 2.0 * (Q[2] * Q[3] - Q[0] * Q[1]);
  Cnb[8] = ((Q[0] * Q[0] - Q[1] * Q[1]) - Q[2] * Q[2]) + Q[3] * Q[3];
}

/*
 * File trailer for FQtoCnb.c
 *
 * [EOF]
 */
