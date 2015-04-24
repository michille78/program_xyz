/*
 * File: CToEuler_c.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "CToEuler_c.h"
#include "BVHTransform_c_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double C[9]
 *                const double rotateDirection[3]
 *                double euler[3]
 * Return Type  : void
 */
void CToEuler_c(const double C[9], const double rotateDirection[3], double
                euler[3])
{
  double C_K[9];
  double b_euler[3];
  int i;

  /* % xyz  2015.4.5 */
  /*  direction cosine matrix  to Euler angle  for multi C */
  /*  C: [N*3*3] */
  /*        euler = [eu_x,eu_y,eu_z]  ( Nframes*3 ) */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  the first and third euler angles scope: [-pi,pi] */
  /*  the second euler angles scope: [-pi/2,pi/2] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  memcpy(&C_K[0], &C[0], 9U * sizeof(double));

  /*  euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ; */
  b_euler[0] = rt_atan2d_snf(C_K[2], C_K[8]);
  b_euler[1] = asin(-C_K[5]);
  b_euler[2] = rt_atan2d_snf(C_K[3], C_K[4]);
  for (i = 0; i < 3; i++) {
    euler[i] = b_euler[i] * rotateDirection[i];
  }
}

/*
 * Arguments    : const double C[9]
 *                const double rotateDirection[3]
 *                double euler[3]
 * Return Type  : void
 */
void b_CToEuler_c(const double C[9], const double rotateDirection[3], double
                  euler[3])
{
  double C_K[9];
  double b_euler[3];
  int i;

  /* % xyz  2015.4.5 */
  /*  direction cosine matrix  to Euler angle  for multi C */
  /*  C: [N*3*3] */
  /*        euler = [eu_x,eu_y,eu_z]  ( Nframes*3 ) */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  the first and third euler angles scope: [-pi,pi] */
  /*  the second euler angles scope: [-pi/2,pi/2] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  memcpy(&C_K[0], &C[0], 9U * sizeof(double));

  /*  euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ; */
  b_euler[0] = rt_atan2d_snf(C_K[3], C_K[0]);
  b_euler[1] = asin(-C_K[6]);
  b_euler[2] = rt_atan2d_snf(C_K[7], C_K[8]);
  for (i = 0; i < 3; i++) {
    euler[i] = b_euler[i] * rotateDirection[i];
  }
}

/*
 * Arguments    : const double C[9]
 *                const double rotateDirection[3]
 *                double euler[3]
 * Return Type  : void
 */
void c_CToEuler_c(const double C[9], const double rotateDirection[3], double
                  euler[3])
{
  double C_K[9];
  double b_euler[3];
  int i;

  /* % xyz  2015.4.5 */
  /*  direction cosine matrix  to Euler angle  for multi C */
  /*  C: [N*3*3] */
  /*        euler = [eu_x,eu_y,eu_z]  ( Nframes*3 ) */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  the first and third euler angles scope: [-pi,pi] */
  /*  the second euler angles scope: [-pi/2,pi/2] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  memcpy(&C_K[0], &C[0], 9U * sizeof(double));
  b_euler[0] = rt_atan2d_snf(-C_K[5], C_K[8]);
  b_euler[1] = asin(C_K[2]);
  b_euler[2] = rt_atan2d_snf(-C_K[1], C_K[0]);
  for (i = 0; i < 3; i++) {
    euler[i] = b_euler[i] * rotateDirection[i];
  }
}

/*
 * Arguments    : const double C[9]
 *                const double rotateDirection[3]
 *                double euler[3]
 * Return Type  : void
 */
void d_CToEuler_c(const double C[9], const double rotateDirection[3], double
                  euler[3])
{
  double C_K[9];
  double b_euler[3];
  int i;

  /* % xyz  2015.4.5 */
  /*  direction cosine matrix  to Euler angle  for multi C */
  /*  C: [N*3*3] */
  /*        euler = [eu_x,eu_y,eu_z]  ( Nframes*3 ) */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  the first and third euler angles scope: [-pi,pi] */
  /*  the second euler angles scope: [-pi/2,pi/2] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  memcpy(&C_K[0], &C[0], 9U * sizeof(double));
  b_euler[0] = rt_atan2d_snf(C_K[7], C_K[4]);
  b_euler[1] = asin(-C_K[1]);
  b_euler[2] = rt_atan2d_snf(C_K[2], C_K[0]);
  for (i = 0; i < 3; i++) {
    euler[i] = b_euler[i] * rotateDirection[i];
  }
}

/*
 * File trailer for CToEuler_c.c
 *
 * [EOF]
 */
