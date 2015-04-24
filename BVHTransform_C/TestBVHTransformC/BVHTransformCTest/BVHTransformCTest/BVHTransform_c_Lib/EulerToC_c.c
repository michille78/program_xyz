/*
 * File: EulerToC_c.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "EulerToC_c.h"

/* Function Declarations */
static void OneEulerToC_c(double euler[6], const double rotateDirection[3],
  double C[9]);
static void b_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9]);
static void c_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9]);
static void d_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9]);
static void e_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9]);

/* Function Definitions */

/*
 * Arguments    : double euler[6]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
static void OneEulerToC_c(double euler[6], const double rotateDirection[3],
  double C[9])
{
  int i;
  double dv17[9];
  double dv18[9];
  static const signed char iv0[3] = { 0, 0, 1 };

  static const signed char iv1[3] = { 1, 0, 0 };

  double dv19[9];
  double dv20[9];
  int i3;
  int i4;
  static const signed char iv2[3] = { 0, 1, 0 };

  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  dv17[0] = cos(euler[2]);
  dv17[3] = sin(euler[2]);
  dv17[6] = 0.0;
  dv17[1] = -sin(euler[2]);
  dv17[4] = cos(euler[2]);
  dv17[7] = 0.0;
  for (i = 0; i < 3; i++) {
    dv17[2 + 3 * i] = iv0[i];
    dv18[3 * i] = iv1[i];
  }

  dv18[1] = 0.0;
  dv18[4] = cos(euler[1]);
  dv18[7] = sin(euler[1]);
  dv18[2] = 0.0;
  dv18[5] = -sin(euler[1]);
  dv18[8] = cos(euler[1]);
  dv20[0] = cos(euler[0]);
  dv20[3] = 0.0;
  dv20[6] = -sin(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i3 = 0; i3 < 3; i3++) {
      dv19[i + 3 * i3] = 0.0;
      for (i4 = 0; i4 < 3; i4++) {
        dv19[i + 3 * i3] += dv17[i + 3 * i4] * dv18[i4 + 3 * i3];
      }
    }

    dv20[1 + 3 * i] = iv2[i];
  }

  dv20[2] = sin(euler[0]);
  dv20[5] = 0.0;
  dv20[8] = cos(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i3 = 0; i3 < 3; i3++) {
      C[i + 3 * i3] = 0.0;
      for (i4 = 0; i4 < 3; i4++) {
        C[i + 3 * i3] += dv19[i + 3 * i4] * dv20[i4 + 3 * i3];
      }
    }
  }
}

/*
 * Arguments    : double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
static void b_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9])
{
  int i;
  double dv22[9];
  double dv23[9];
  static const signed char iv3[3] = { 0, 0, 1 };

  static const signed char iv4[3] = { 1, 0, 0 };

  double dv24[9];
  double dv25[9];
  int i6;
  int i7;
  static const signed char iv5[3] = { 0, 1, 0 };

  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  dv22[0] = cos(euler[2]);
  dv22[3] = sin(euler[2]);
  dv22[6] = 0.0;
  dv22[1] = -sin(euler[2]);
  dv22[4] = cos(euler[2]);
  dv22[7] = 0.0;
  for (i = 0; i < 3; i++) {
    dv22[2 + 3 * i] = iv3[i];
    dv23[3 * i] = iv4[i];
  }

  dv23[1] = 0.0;
  dv23[4] = cos(euler[1]);
  dv23[7] = sin(euler[1]);
  dv23[2] = 0.0;
  dv23[5] = -sin(euler[1]);
  dv23[8] = cos(euler[1]);
  dv25[0] = cos(euler[0]);
  dv25[3] = 0.0;
  dv25[6] = -sin(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i6 = 0; i6 < 3; i6++) {
      dv24[i + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        dv24[i + 3 * i6] += dv22[i + 3 * i7] * dv23[i7 + 3 * i6];
      }
    }

    dv25[1 + 3 * i] = iv5[i];
  }

  dv25[2] = sin(euler[0]);
  dv25[5] = 0.0;
  dv25[8] = cos(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i6 = 0; i6 < 3; i6++) {
      C[i + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        C[i + 3 * i6] += dv24[i + 3 * i7] * dv25[i7 + 3 * i6];
      }
    }
  }
}

/*
 * Arguments    : double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
static void c_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9])
{
  double dv34[9];
  double dv35[9];
  int i;
  static const signed char iv6[3] = { 1, 0, 0 };

  static const signed char iv7[3] = { 0, 1, 0 };

  double dv36[9];
  double dv37[9];
  int i19;
  int i20;
  static const signed char iv8[3] = { 0, 0, 1 };

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  for (i = 0; i < 3; i++) {
    dv34[3 * i] = iv6[i];
    euler[i] *= rotateDirection[i];
  }

  dv34[1] = 0.0;
  dv34[4] = cos(euler[2]);
  dv34[7] = sin(euler[2]);
  dv34[2] = 0.0;
  dv34[5] = -sin(euler[2]);
  dv34[8] = cos(euler[2]);
  dv35[0] = cos(euler[1]);
  dv35[3] = 0.0;
  dv35[6] = -sin(euler[1]);
  for (i = 0; i < 3; i++) {
    dv35[1 + 3 * i] = iv7[i];
  }

  dv35[2] = sin(euler[1]);
  dv35[5] = 0.0;
  dv35[8] = cos(euler[1]);
  dv37[0] = cos(euler[0]);
  dv37[3] = sin(euler[0]);
  dv37[6] = 0.0;
  dv37[1] = -sin(euler[0]);
  dv37[4] = cos(euler[0]);
  dv37[7] = 0.0;
  for (i = 0; i < 3; i++) {
    for (i19 = 0; i19 < 3; i19++) {
      dv36[i + 3 * i19] = 0.0;
      for (i20 = 0; i20 < 3; i20++) {
        dv36[i + 3 * i19] += dv34[i + 3 * i20] * dv35[i20 + 3 * i19];
      }
    }

    dv37[2 + 3 * i] = iv8[i];
  }

  for (i = 0; i < 3; i++) {
    for (i19 = 0; i19 < 3; i19++) {
      C[i + 3 * i19] = 0.0;
      for (i20 = 0; i20 < 3; i20++) {
        C[i + 3 * i19] += dv36[i + 3 * i20] * dv37[i20 + 3 * i19];
      }
    }
  }
}

/*
 * Arguments    : double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
static void d_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9])
{
  int i;
  double dv40[9];
  double dv41[9];
  static const signed char iv9[3] = { 0, 0, 1 };

  static const signed char iv10[3] = { 0, 1, 0 };

  double dv42[9];
  double dv43[9];
  int i24;
  int i25;
  static const signed char iv11[3] = { 1, 0, 0 };

  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  dv40[0] = cos(euler[2]);
  dv40[3] = sin(euler[2]);
  dv40[6] = 0.0;
  dv40[1] = -sin(euler[2]);
  dv40[4] = cos(euler[2]);
  dv40[7] = 0.0;
  dv41[0] = cos(euler[1]);
  dv41[3] = 0.0;
  dv41[6] = -sin(euler[1]);
  for (i = 0; i < 3; i++) {
    dv40[2 + 3 * i] = iv9[i];
    dv41[1 + 3 * i] = iv10[i];
  }

  dv41[2] = sin(euler[1]);
  dv41[5] = 0.0;
  dv41[8] = cos(euler[1]);
  for (i = 0; i < 3; i++) {
    for (i24 = 0; i24 < 3; i24++) {
      dv42[i + 3 * i24] = 0.0;
      for (i25 = 0; i25 < 3; i25++) {
        dv42[i + 3 * i24] += dv40[i + 3 * i25] * dv41[i25 + 3 * i24];
      }
    }

    dv43[3 * i] = iv11[i];
  }

  dv43[1] = 0.0;
  dv43[4] = cos(euler[0]);
  dv43[7] = sin(euler[0]);
  dv43[2] = 0.0;
  dv43[5] = -sin(euler[0]);
  dv43[8] = cos(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i24 = 0; i24 < 3; i24++) {
      C[i + 3 * i24] = 0.0;
      for (i25 = 0; i25 < 3; i25++) {
        C[i + 3 * i24] += dv42[i + 3 * i25] * dv43[i25 + 3 * i24];
      }
    }
  }
}

/*
 * Arguments    : double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
static void e_OneEulerToC_c(double euler[3], const double rotateDirection[3],
  double C[9])
{
  int i;
  double dv60[9];
  double dv61[9];
  static const signed char iv12[3] = { 0, 1, 0 };

  static const signed char iv13[3] = { 0, 0, 1 };

  double dv62[9];
  double dv63[9];
  int i40;
  int i41;
  static const signed char iv14[3] = { 1, 0, 0 };

  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive */
  dv60[0] = cos(euler[2]);
  dv60[3] = 0.0;
  dv60[6] = -sin(euler[2]);
  for (i = 0; i < 3; i++) {
    dv60[1 + 3 * i] = iv12[i];
  }

  dv60[2] = sin(euler[2]);
  dv60[5] = 0.0;
  dv60[8] = cos(euler[2]);
  dv61[0] = cos(euler[1]);
  dv61[3] = sin(euler[1]);
  dv61[6] = 0.0;
  dv61[1] = -sin(euler[1]);
  dv61[4] = cos(euler[1]);
  dv61[7] = 0.0;
  for (i = 0; i < 3; i++) {
    dv61[2 + 3 * i] = iv13[i];
  }

  for (i = 0; i < 3; i++) {
    for (i40 = 0; i40 < 3; i40++) {
      dv62[i + 3 * i40] = 0.0;
      for (i41 = 0; i41 < 3; i41++) {
        dv62[i + 3 * i40] += dv60[i + 3 * i41] * dv61[i41 + 3 * i40];
      }
    }

    dv63[3 * i] = iv14[i];
  }

  dv63[1] = 0.0;
  dv63[4] = cos(euler[0]);
  dv63[7] = sin(euler[0]);
  dv63[2] = 0.0;
  dv63[5] = -sin(euler[0]);
  dv63[8] = cos(euler[0]);
  for (i = 0; i < 3; i++) {
    for (i40 = 0; i40 < 3; i40++) {
      C[i + 3 * i40] = 0.0;
      for (i41 = 0; i41 < 3; i41++) {
        C[i + 3 * i40] += dv62[i + 3 * i41] * dv63[i41 + 3 * i40];
      }
    }
  }
}

/*
 * Arguments    : const double euler[6]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
void EulerToC_c(const double euler[6], const double rotateDirection[3], double
                C[9])
{
  double b_euler[6];
  int i2;
  double dv16[9];

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  for (i2 = 0; i2 < 6; i2++) {
    b_euler[i2] = euler[i2];
  }

  OneEulerToC_c(b_euler, rotateDirection, dv16);
  memcpy(&C[0], &dv16[0], 9U * sizeof(double));
}

/*
 * Arguments    : const double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
void b_EulerToC_c(const double euler[3], const double rotateDirection[3], double
                  C[9])
{
  double b_euler[3];
  int i5;
  double dv21[9];

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  for (i5 = 0; i5 < 3; i5++) {
    b_euler[i5] = euler[i5];
  }

  b_OneEulerToC_c(b_euler, rotateDirection, dv21);
  memcpy(&C[0], &dv21[0], 9U * sizeof(double));
}

/*
 * Arguments    : const double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
void c_EulerToC_c(const double euler[3], const double rotateDirection[3], double
                  C[9])
{
  double b_euler[3];
  int i18;
  double dv33[9];

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  for (i18 = 0; i18 < 3; i18++) {
    b_euler[i18] = euler[i18];
  }

  c_OneEulerToC_c(b_euler, rotateDirection, dv33);
  memcpy(&C[0], &dv33[0], 9U * sizeof(double));
}

/*
 * Arguments    : const double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
void d_EulerToC_c(const double euler[3], const double rotateDirection[3], double
                  C[9])
{
  double b_euler[3];
  int i23;
  double dv39[9];

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  for (i23 = 0; i23 < 3; i23++) {
    b_euler[i23] = euler[i23];
  }

  d_OneEulerToC_c(b_euler, rotateDirection, dv39);
  memcpy(&C[0], &dv39[0], 9U * sizeof(double));
}

/*
 * Arguments    : const double euler[3]
 *                const double rotateDirection[3]
 *                double C[9]
 * Return Type  : void
 */
void e_EulerToC_c(const double euler[3], const double rotateDirection[3], double
                  C[9])
{
  double b_euler[3];
  int i39;
  double dv59[9];

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  for (i39 = 0; i39 < 3; i39++) {
    b_euler[i39] = euler[i39];
  }

  e_OneEulerToC_c(b_euler, rotateDirection, dv59);
  memcpy(&C[0], &dv59[0], 9U * sizeof(double));
}

/*
 * File trailer for EulerToC_c.c
 *
 * [EOF]
 */
