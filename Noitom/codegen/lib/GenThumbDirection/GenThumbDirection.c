/*
 * File: GenThumbDirection.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 27-May-2015 16:37:00
 */

/* Include files */
#include "rt_nonfinite.h"
#include "GenThumbDirection.h"

/* Function Declarations */
static void Euler2C(const float euler[3], double C[9]);
static void Euler2C_One(const float euler[3], float C[9]);
static void RotateX(float angX, float Cx[9]);
static void RotateY(float angY, float Cy[9]);
static void RotateZ(float angZ, float Cz[9]);
static void b_RotateX(double angX, double Cx[9]);
static void b_RotateZ(double angZ, double Cz[9]);

/* Function Definitions */

/*
 * Arguments    : const float euler[3]
 *                double C[9]
 * Return Type  : void
 */
static void Euler2C(const float euler[3], double C[9])
{
  float fv2[9];
  int i2;

  /* % xyz 2015.4.8 */
  /*  Euler angle to direction cosine matrix */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Intput */
  /*  euler:  [3*N];   euler(k,:) = [eu_1,eu_2,eu_3];   rad    */
  /*        store order of euler angle: the rotate order */
  /*        eu_1,eu_3: [-pi,pi] */
  /*        eu_2:   CosBeita=1: [-pi/2,pi/2]       */
  /*                CosBeita=-1:[ -pi,-pi/2 ] or [ pi/2,pi ] */
  /*  rotateOrder:  XYZ  XZY  ... */
  /*  rotateDirection: anticlockwise 只运允许逆时针，如果是顺时针，先将该角度反号 */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Output */
  /*  C: [3*3*N] */
  /*  if ~exist('rotateOrder','var') */
  /*      rotateOrder = []; */
  /*  end */
  /* % xyz 2015 4.8 */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Intput */
  /*  data: [N*const] or [const*N] */
  /*  const */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Output */
  /*  data: [const*N] */
  Euler2C_One(euler, fv2);
  for (i2 = 0; i2 < 9; i2++) {
    C[i2] = fv2[i2];
  }
}

/*
 * Arguments    : const float euler[3]
 *                float C[9]
 * Return Type  : void
 */
static void Euler2C_One(const float euler[3], float C[9])
{
  float fv3[9];
  float fv4[9];
  float fv5[9];
  float fv6[9];
  int i3;
  int i4;
  int i5;

  /*  if ~exist('rotateOrder','var') || isempty(rotateOrder) */
  /*      rotateOrder = 'ZYX'; */
  /*  end */
  RotateX(euler[2], fv3);
  RotateZ(euler[1], fv4);
  RotateY(euler[0], fv5);
  for (i3 = 0; i3 < 3; i3++) {
    for (i4 = 0; i4 < 3; i4++) {
      fv6[i3 + 3 * i4] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        fv6[i3 + 3 * i4] += fv3[i3 + 3 * i5] * fv4[i5 + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      C[i3 + 3 * i4] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        C[i3 + 3 * i4] += fv6[i3 + 3 * i5] * fv5[i5 + 3 * i4];
      }
    }
  }
}

/*
 * Arguments    : float angX
 *                float Cx[9]
 * Return Type  : void
 */
static void RotateX(float angX, float Cx[9])
{
  int i6;
  static const signed char iv2[3] = { 1, 0, 0 };

  /* % xyz 2015.3.25 */
  /* % anticlockwise = positive  rad */
  for (i6 = 0; i6 < 3; i6++) {
    Cx[3 * i6] = iv2[i6];
  }

  Cx[1] = 0.0F;
  Cx[4] = (real32_T)cos(angX);
  Cx[7] = (real32_T)sin(angX);
  Cx[2] = 0.0F;
  Cx[5] = -(real32_T)sin(angX);
  Cx[8] = (real32_T)cos(angX);
}

/*
 * Arguments    : float angY
 *                float Cy[9]
 * Return Type  : void
 */
static void RotateY(float angY, float Cy[9])
{
  int i8;
  static const signed char iv4[3] = { 0, 1, 0 };

  /* % xyz 2015.3.25 */
  /*  anticlockwise = positive */
  /* % anticlockwise rotate is positive  rad */
  Cy[0] = (real32_T)cos(angY);
  Cy[3] = 0.0F;
  Cy[6] = -(real32_T)sin(angY);
  for (i8 = 0; i8 < 3; i8++) {
    Cy[1 + 3 * i8] = iv4[i8];
  }

  Cy[2] = (real32_T)sin(angY);
  Cy[5] = 0.0F;
  Cy[8] = (real32_T)cos(angY);
}

/*
 * Arguments    : float angZ
 *                float Cz[9]
 * Return Type  : void
 */
static void RotateZ(float angZ, float Cz[9])
{
  int i7;
  static const signed char iv3[3] = { 0, 0, 1 };

  /* % xyz 2015.3.25 */
  /* % anticlockwise = positive  rad */
  Cz[0] = (real32_T)cos(angZ);
  Cz[3] = (real32_T)sin(angZ);
  Cz[6] = 0.0F;
  Cz[1] = -(real32_T)sin(angZ);
  Cz[4] = (real32_T)cos(angZ);
  Cz[7] = 0.0F;
  for (i7 = 0; i7 < 3; i7++) {
    Cz[2 + 3 * i7] = iv3[i7];
  }
}

/*
 * Arguments    : double angX
 *                double Cx[9]
 * Return Type  : void
 */
static void b_RotateX(double angX, double Cx[9])
{
  int i10;
  static const signed char iv6[3] = { 1, 0, 0 };

  /* % xyz 2015.3.25 */
  /* % anticlockwise = positive  rad */
  for (i10 = 0; i10 < 3; i10++) {
    Cx[3 * i10] = iv6[i10];
  }

  Cx[1] = 0.0;
  Cx[4] = cos(angX);
  Cx[7] = sin(angX);
  Cx[2] = 0.0;
  Cx[5] = -sin(angX);
  Cx[8] = cos(angX);
}

/*
 * Arguments    : double angZ
 *                double Cz[9]
 * Return Type  : void
 */
static void b_RotateZ(double angZ, double Cz[9])
{
  int i9;
  static const signed char iv5[3] = { 0, 0, 1 };

  /* % xyz 2015.3.25 */
  /* % anticlockwise = positive  rad */
  Cz[0] = cos(angZ);
  Cz[3] = sin(angZ);
  Cz[6] = 0.0;
  Cz[1] = -sin(angZ);
  Cz[4] = cos(angZ);
  Cz[7] = 0.0;
  for (i9 = 0; i9 < 3; i9++) {
    Cz[2 + 3 * i9] = iv5[i9];
  }
}

/*
 * Arguments    : float rotateAngle[3]
 *                const float zd[3]
 *                char RightOrLeft
 *                char pose
 *                float bone_w[9]
 * Return Type  : void
 */
void GenThumbDirection(float rotateAngle[3], const float zd[3], char RightOrLeft,
  char pose, float bone_w[9])
{
  boolean_T b_bool;
  int i;
  double Crp[9];
  float b_rotateAngle[3];
  double dv0[9];
  float b_zd[3];
  float fv0[9];
  float c_zd[9];
  int i0;
  double d0;
  int i1;
  float fv1[9];
  static const signed char iv0[3] = { 0, 0, 1 };

  static const signed char iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  /* % xyz 2015.5.27 */
  /* % （右手）根据拇指的角度状态输出拇指坐标轴 各个Pose时， 在世界坐标系下的指向 */
  /*  拇指的骨骼系为“左上前”，旋转顺序为“YZX” */
  /*  w:当地北东地系 */
  /*  r:与w系仅相差航向信息的水平系（x指向人的前方） */
  /*  p：特定pose下手掌系 */
  /* %% Input: */
  /*  rotateAngle：拇指的“YZX”旋转角度,单位：度  [3*1]  */
  /*  zd： 人的前方在世界系下的表达（r系x轴在w系下表达）  [3*1] */
  /*  RightOrLeft： 左手（ 'L'）， 右手 （'R'） */
  /*  pose"：姿势 'T' 'A'  'S' */
  b_bool = false;
  if (RightOrLeft != 'L') {
  } else {
    b_bool = true;
  }

  if (b_bool) {
    for (i = 0; i < 2; i++) {
      rotateAngle[1 + i] = -rotateAngle[1 + i];
    }
  }

  /* % 当前pose下手掌系 p  -> 拇指系 */
  /* % 无航向的世界系 r -> 当前pose下手掌系 p */
  b_bool = false;
  if (pose != 'T') {
  } else {
    b_bool = true;
  }

  if (b_bool) {
    i = 0;
  } else {
    b_bool = false;
    if (pose != 'A') {
    } else {
      b_bool = true;
    }

    if (b_bool) {
      i = 1;
    } else {
      b_bool = false;
      if (pose != 'S') {
      } else {
        b_bool = true;
      }

      if (b_bool) {
        i = 2;
      } else {
        i = -1;
      }
    }
  }

  switch (i) {
   case 0:
    memset(&Crp[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      Crp[i + 3 * i] = 1.0;
    }
    break;

   case 1:
    b_bool = false;
    if (RightOrLeft != 'L') {
    } else {
      b_bool = true;
    }

    if (b_bool) {
      b_RotateX(-1.5707963267948966, Crp);
    } else {
      b_RotateX(1.5707963267948966, Crp);
    }
    break;

   case 2:
    b_bool = false;
    if (RightOrLeft != 'L') {
    } else {
      b_bool = true;
    }

    if (b_bool) {
      b_RotateZ(1.5707963267948966, Crp);
    } else {
      b_RotateX(-1.5707963267948966, Crp);
    }
    break;

   default:
    memset(&Crp[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      Crp[i + 3 * i] = 1.0;
    }
    break;
  }

  /* % Crw */
  /* % */
  for (i = 0; i < 3; i++) {
    b_rotateAngle[i] = rotateAngle[i] * 3.14159274F / 180.0F;
  }

  Euler2C(b_rotateAngle, dv0);
  b_zd[0] = zd[1] - zd[2] * 0.0F;
  b_zd[1] = zd[2] * 0.0F - zd[0];
  b_zd[2] = zd[0] * 0.0F - zd[1] * 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += dv0[i + 3 * i1] * Crp[i1 + 3 * i0];
      }

      fv0[i + 3 * i0] = (float)d0;
    }
  }

  for (i = 0; i < 3; i++) {
    c_zd[3 * i] = zd[i];
    c_zd[1 + 3 * i] = b_zd[i];
    c_zd[2 + 3 * i] = iv0[i];
    for (i0 = 0; i0 < 3; i0++) {
      fv1[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        fv1[i + 3 * i0] += fv0[i0 + 3 * i1] * c_zd[i1 + 3 * i];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      bone_w[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        bone_w[i + 3 * i0] += fv1[i + 3 * i1] * (float)iv1[i1 + 3 * i0];
      }
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void GenThumbDirection_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void GenThumbDirection_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for GenThumbDirection.c
 *
 * [EOF]
 */
