/*
 * File: BVHTransform_c.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 05-Apr-2015 19:50:56
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "CToEuler_c.h"
#include "EulerToC_c.h"
#include "FQtoCnb.h"
#include "eye.h"
#include "BVHTransform_c_rtwutil.h"

/* Function Declarations */
static void BVHDataToRotate_c(const double BVHData[354], double C_World_Hip[9],
  double C_Hip_Spine[9], double C_Spine_Spine1[9], double C_Spine1_Spine2[9],
  double C_Spine2_Spine3[9], double C_Spine3_Neck[9], double C_Neck_Head[9],
  double C_Spine3_LShoulder[9], double C_LShoulder_LArm[9], double
  C_LArm_LForeArm[9], double C_LForeArm_LHand[9], double C_Spine3_RShoulder[9],
  double C_RShoulder_RArm[9], double C_RArm_RForeArm[9], double
  C_RForeArm_RHand[9], double C_Hip_RUpLeg[9], double C_RUpLeg_RLeg[9], double
  C_RLeg_RFoot[9], double C_Hip_LUpLeg[9], double C_LUpLeg_LLeg[9], double
  C_LLeg_LFoot[9]);
static void CalculateC_Multi(const double C12[9], const double C23[9], double
  C13[9]);
static void CalculatedLLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_LUpLeg[9], const double C_LUpLeg_LLeg[9],
  double C_Hip_LLeg[9], double C_Spine2_LLeg[9], double C_Spine3_LLeg[9]);
static void CalculatedLeftArmRotate(const double C_Hip_Spine[9], const double
  C_Spine_Spine1[9], const double C_Spine1_Spine2[9], const double
  C_Spine2_Spine3[9], const double C_Spine3_LShoulder[9], const double
  C_LShoulder_LArm[9], const double C_LArm_LForeArm[9], const double
  C_LForeArm_LHand[9], double C_Hip_Spine2[9], double C_Hip_Spine3[9], double
  C_Spine_Spine3[9], double C_Hip_LArm[9], double C_Hip_LForeArm[9], double
  C_Hip_LHand[9], double C_Spine_LArm[9], double C_Spine_LForeArm[9], double
  C_Spine_LHand[9], double C_Spine2_LArm[9], double C_Spine2_LForeArm[9], double
  C_Spine2_LHand[9], double C_Spine3_LArm[9], double C_Spine3_LForeArm[9],
  double C_Spine3_LHand[9]);
static void CalculatedRLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_RUpLeg[9], const double C_RUpLeg_RLeg[9],
  double C_Hip_RLeg[9], double C_Spine2_RLeg[9], double C_Spine3_RLeg[9]);
static void CalculatedRightArmRotate(const double C_Hip_Spine3[9], const double
  C_Spine2_Spine3[9], const double C_Spine_Spine3[9], const double
  C_Spine3_RShoulder[9], const double C_RShoulder_RArm[9], const double
  C_RArm_RForeArm[9], const double C_RForeArm_RHand[9], double C_Hip_RArm[9],
  double C_Hip_RForeArm[9], double C_Hip_RHand[9], double C_Spine_RArm[9],
  double C_Spine_RForeArm[9], double C_Spine_RHand[9], double C_Spine2_RArm[9],
  double C_Spine2_RForeArm[9], double C_Spine2_RHand[9], double C_Spine3_RArm[9],
  double C_Spine3_RForeArm[9], double C_Spine3_RHand[9]);
static void InverseC_Multi(const double C12[9], double C21[9]);
static void TransformHead(const double C_Spine_Head[9], const double
  C_Spine_NeckNew[9], double *HeadYaw, double *HeadPitch, double Head_new[3]);
static void TransformLElbow(const double C_Spine_LForeArm[9], const double
  C_Spine_LArmNew[9], double LElbow_Nao[3], double LeftForeArm_new[3]);
static void TransformLFoot(const double C_LLeg_LFoot[9], double *LAnklePitch,
  double *LAnkleRoll, double LeftFoot_new[3]);
static void TransformLHand(const double C_Spine_LHand[9], const double
  C_Spine_LForeArmNew[9], double *LWristYaw, double LeftHand_new[3]);
static void TransformLLeg(const double C_Hip_LUpLegNew[9], const double
  C_Hip_LLeg[9], double *LKneePitch, double LeftLeg_New[3]);
static void TransformLShoulder(const double C_Spine3_LShoulderNew[9], const
  double C_Spine3_LArm[9], double *LShoulderPitch, double *LShoulderRoll, double
  LeftArm_new[3]);
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double HipYawPitch,
  double *LHipRoll, double *LHipPitch, double LeftUpLeg_new[3]);
static void TransformRElbow(const double C_Spine_RForeArm[9], const double
  C_Spine_RArmNew[9], double RElbow_Nao[3], double RightForeArm_new[3]);
static void TransformRFoot(const double C_RLeg_RFoot[9], double *RAnklePitch,
  double *RAnkleRoll, double RightFoot_new[3]);
static void TransformRHand(const double C_Spine_RHand[9], const double
  C_Spine_RForeArmNew[9], double *RWristYaw, double RightHand_new[3]);
static void TransformRLeg(const double C_Hip_RUpLegNew[9], const double
  C_Hip_RLeg[9], double *RKneePitch, double RightLeg_New[3]);
static void TransformRShoulder(const double C_Spine3_RShoulderNew[9], const
  double C_Spine3_RArm[9], double *RShoulderPitch, double *RShoulderRoll, double
  RightArm_new[3]);
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double HipYawPitch,
  double *RHipRoll, double *RHipPitch, double RightUpLeg_new[3]);

/* Function Definitions */

/*
 * coder.ceval('EulerToC_c');
 *  coder.inline('never');
 * Arguments    : const double BVHData[354]
 *                double C_World_Hip[9]
 *                double C_Hip_Spine[9]
 *                double C_Spine_Spine1[9]
 *                double C_Spine1_Spine2[9]
 *                double C_Spine2_Spine3[9]
 *                double C_Spine3_Neck[9]
 *                double C_Neck_Head[9]
 *                double C_Spine3_LShoulder[9]
 *                double C_LShoulder_LArm[9]
 *                double C_LArm_LForeArm[9]
 *                double C_LForeArm_LHand[9]
 *                double C_Spine3_RShoulder[9]
 *                double C_RShoulder_RArm[9]
 *                double C_RArm_RForeArm[9]
 *                double C_RForeArm_RHand[9]
 *                double C_Hip_RUpLeg[9]
 *                double C_RUpLeg_RLeg[9]
 *                double C_RLeg_RFoot[9]
 *                double C_Hip_LUpLeg[9]
 *                double C_LUpLeg_LLeg[9]
 *                double C_LLeg_LFoot[9]
 * Return Type  : void
 */
static void BVHDataToRotate_c(const double BVHData[354], double C_World_Hip[9],
  double C_Hip_Spine[9], double C_Spine_Spine1[9], double C_Spine1_Spine2[9],
  double C_Spine2_Spine3[9], double C_Spine3_Neck[9], double C_Neck_Head[9],
  double C_Spine3_LShoulder[9], double C_LShoulder_LArm[9], double
  C_LArm_LForeArm[9], double C_LForeArm_LHand[9], double C_Spine3_RShoulder[9],
  double C_RShoulder_RArm[9], double C_RArm_RForeArm[9], double
  C_RForeArm_RHand[9], double C_Hip_RUpLeg[9], double C_RUpLeg_RLeg[9], double
  C_RLeg_RFoot[9], double C_Hip_LUpLeg[9], double C_LUpLeg_LLeg[9], double
  C_LLeg_LFoot[9])
{
  double dv15[3];
  int i1;

  /*  BVHData: Nframes*(59*6)   [ 1*354 ] */
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  EulerToC_c(*(double (*)[6])&BVHData[0], dv15, C_World_Hip);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[45], dv15, C_Hip_Spine);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[51], dv15, C_Spine_Spine1);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[57], dv15, C_Spine1_Spine2);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[63], dv15, C_Spine2_Spine3);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[69], dv15, C_Spine3_Neck);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[75], dv15, C_Neck_Head);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[219], dv15, C_Spine3_LShoulder);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[225], dv15, C_LShoulder_LArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[231], dv15, C_LArm_LForeArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[237], dv15, C_LForeArm_LHand);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[81], dv15, C_Spine3_RShoulder);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[87], dv15, C_RShoulder_RArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[93], dv15, C_RArm_RForeArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[99], dv15, C_RForeArm_RHand);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[9], dv15, C_Hip_RUpLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[15], dv15, C_RUpLeg_RLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[21], dv15, C_RLeg_RFoot);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[27], dv15, C_Hip_LUpLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[33], dv15, C_LUpLeg_LLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv15[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[39], dv15, C_LLeg_LFoot);
}

/*
 * Arguments    : const double C12[9]
 *                const double C23[9]
 *                double C13[9]
 * Return Type  : void
 */
static void CalculateC_Multi(const double C12[9], const double C23[9], double
  C13[9])
{
  double C23_k[9];
  double C12_k[9];
  int k;
  double b_C23_k[9];
  int i8;
  int i9;
  for (k = 0; k < 9; k++) {
    C23_k[k] = C23[k];
    C12_k[k] = C12[k];
  }

  for (k = 0; k < 3; k++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_C23_k[k + 3 * i8] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        b_C23_k[k + 3 * i8] += C23_k[k + 3 * i9] * C12_k[i9 + 3 * i8];
      }
    }
  }

  for (k = 0; k < 3; k++) {
    for (i8 = 0; i8 < 3; i8++) {
      C13[i8 + 3 * k] = b_C23_k[i8 + 3 * k];
    }
  }
}

/*
 * Arguments    : const double C_Hip_Spine2[9]
 *                const double C_Hip_Spine3[9]
 *                const double C_Hip_LUpLeg[9]
 *                const double C_LUpLeg_LLeg[9]
 *                double C_Hip_LLeg[9]
 *                double C_Spine2_LLeg[9]
 *                double C_Spine3_LLeg[9]
 * Return Type  : void
 */
static void CalculatedLLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_LUpLeg[9], const double C_LUpLeg_LLeg[9],
  double C_Hip_LLeg[9], double C_Spine2_LLeg[9], double C_Spine3_LLeg[9])
{
  double C12_k[9];
  double b_C12_k[9];
  int k;
  double c_C12_k[9];
  double d_C12_k[9];
  int i10;
  CalculateC_Multi(C_Hip_LUpLeg, C_LUpLeg_LLeg, C_Hip_LLeg);
  for (k = 0; k < 9; k++) {
    C12_k[k] = C_Hip_Spine2[k];
    b_C12_k[k] = C_Hip_Spine3[k];
  }

  for (k = 0; k < 3; k++) {
    for (i10 = 0; i10 < 3; i10++) {
      c_C12_k[i10 + 3 * k] = C12_k[k + 3 * i10];
      d_C12_k[i10 + 3 * k] = b_C12_k[k + 3 * i10];
    }
  }

  CalculateC_Multi(c_C12_k, C_Hip_LLeg, C_Spine2_LLeg);
  CalculateC_Multi(d_C12_k, C_Hip_LLeg, C_Spine3_LLeg);
}

/*
 * Arguments    : const double C_Hip_Spine[9]
 *                const double C_Spine_Spine1[9]
 *                const double C_Spine1_Spine2[9]
 *                const double C_Spine2_Spine3[9]
 *                const double C_Spine3_LShoulder[9]
 *                const double C_LShoulder_LArm[9]
 *                const double C_LArm_LForeArm[9]
 *                const double C_LForeArm_LHand[9]
 *                double C_Hip_Spine2[9]
 *                double C_Hip_Spine3[9]
 *                double C_Spine_Spine3[9]
 *                double C_Hip_LArm[9]
 *                double C_Hip_LForeArm[9]
 *                double C_Hip_LHand[9]
 *                double C_Spine_LArm[9]
 *                double C_Spine_LForeArm[9]
 *                double C_Spine_LHand[9]
 *                double C_Spine2_LArm[9]
 *                double C_Spine2_LForeArm[9]
 *                double C_Spine2_LHand[9]
 *                double C_Spine3_LArm[9]
 *                double C_Spine3_LForeArm[9]
 *                double C_Spine3_LHand[9]
 * Return Type  : void
 */
static void CalculatedLeftArmRotate(const double C_Hip_Spine[9], const double
  C_Spine_Spine1[9], const double C_Spine1_Spine2[9], const double
  C_Spine2_Spine3[9], const double C_Spine3_LShoulder[9], const double
  C_LShoulder_LArm[9], const double C_LArm_LForeArm[9], const double
  C_LForeArm_LHand[9], double C_Hip_Spine2[9], double C_Hip_Spine3[9], double
  C_Spine_Spine3[9], double C_Hip_LArm[9], double C_Hip_LForeArm[9], double
  C_Hip_LHand[9], double C_Spine_LArm[9], double C_Spine_LForeArm[9], double
  C_Spine_LHand[9], double C_Spine2_LArm[9], double C_Spine2_LForeArm[9], double
  C_Spine2_LHand[9], double C_Spine3_LArm[9], double C_Spine3_LForeArm[9],
  double C_Spine3_LHand[9])
{
  double C_Spine_Spine2[9];

  /* % Analyse Left Arm BVH */
  CalculateC_Multi(C_Spine_Spine1, C_Spine1_Spine2, C_Spine_Spine2);
  CalculateC_Multi(C_Spine_Spine2, C_Spine2_Spine3, C_Spine_Spine3);
  CalculateC_Multi(C_Hip_Spine, C_Spine_Spine2, C_Hip_Spine2);
  CalculateC_Multi(C_Hip_Spine, C_Spine_Spine3, C_Hip_Spine3);
  CalculateC_Multi(C_Spine3_LShoulder, C_LShoulder_LArm, C_Spine3_LArm);
  CalculateC_Multi(C_Spine2_Spine3, C_Spine3_LArm, C_Spine2_LArm);
  CalculateC_Multi(C_Spine_Spine3, C_Spine3_LArm, C_Spine_LArm);
  CalculateC_Multi(C_Hip_Spine3, C_Spine3_LArm, C_Hip_LArm);
  CalculateC_Multi(C_Spine3_LArm, C_LArm_LForeArm, C_Spine3_LForeArm);
  CalculateC_Multi(C_Spine2_LArm, C_LArm_LForeArm, C_Spine2_LForeArm);
  CalculateC_Multi(C_Spine_LArm, C_LArm_LForeArm, C_Spine_LForeArm);
  CalculateC_Multi(C_Hip_LArm, C_LArm_LForeArm, C_Hip_LForeArm);
  CalculateC_Multi(C_Spine3_LForeArm, C_LForeArm_LHand, C_Spine3_LHand);
  CalculateC_Multi(C_Spine2_LForeArm, C_LForeArm_LHand, C_Spine2_LHand);
  CalculateC_Multi(C_Spine_LForeArm, C_LForeArm_LHand, C_Spine_LHand);
  CalculateC_Multi(C_Hip_LForeArm, C_LForeArm_LHand, C_Hip_LHand);
}

/*
 * Arguments    : const double C_Hip_Spine2[9]
 *                const double C_Hip_Spine3[9]
 *                const double C_Hip_RUpLeg[9]
 *                const double C_RUpLeg_RLeg[9]
 *                double C_Hip_RLeg[9]
 *                double C_Spine2_RLeg[9]
 *                double C_Spine3_RLeg[9]
 * Return Type  : void
 */
static void CalculatedRLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_RUpLeg[9], const double C_RUpLeg_RLeg[9],
  double C_Hip_RLeg[9], double C_Spine2_RLeg[9], double C_Spine3_RLeg[9])
{
  double C12_k[9];
  double b_C12_k[9];
  int k;
  double c_C12_k[9];
  double d_C12_k[9];
  int i13;
  CalculateC_Multi(C_Hip_RUpLeg, C_RUpLeg_RLeg, C_Hip_RLeg);
  for (k = 0; k < 9; k++) {
    C12_k[k] = C_Hip_Spine2[k];
    b_C12_k[k] = C_Hip_Spine3[k];
  }

  for (k = 0; k < 3; k++) {
    for (i13 = 0; i13 < 3; i13++) {
      c_C12_k[i13 + 3 * k] = C12_k[k + 3 * i13];
      d_C12_k[i13 + 3 * k] = b_C12_k[k + 3 * i13];
    }
  }

  CalculateC_Multi(c_C12_k, C_Hip_RLeg, C_Spine2_RLeg);
  CalculateC_Multi(d_C12_k, C_Hip_RLeg, C_Spine3_RLeg);
}

/*
 * Arguments    : const double C_Hip_Spine3[9]
 *                const double C_Spine2_Spine3[9]
 *                const double C_Spine_Spine3[9]
 *                const double C_Spine3_RShoulder[9]
 *                const double C_RShoulder_RArm[9]
 *                const double C_RArm_RForeArm[9]
 *                const double C_RForeArm_RHand[9]
 *                double C_Hip_RArm[9]
 *                double C_Hip_RForeArm[9]
 *                double C_Hip_RHand[9]
 *                double C_Spine_RArm[9]
 *                double C_Spine_RForeArm[9]
 *                double C_Spine_RHand[9]
 *                double C_Spine2_RArm[9]
 *                double C_Spine2_RForeArm[9]
 *                double C_Spine2_RHand[9]
 *                double C_Spine3_RArm[9]
 *                double C_Spine3_RForeArm[9]
 *                double C_Spine3_RHand[9]
 * Return Type  : void
 */
static void CalculatedRightArmRotate(const double C_Hip_Spine3[9], const double
  C_Spine2_Spine3[9], const double C_Spine_Spine3[9], const double
  C_Spine3_RShoulder[9], const double C_RShoulder_RArm[9], const double
  C_RArm_RForeArm[9], const double C_RForeArm_RHand[9], double C_Hip_RArm[9],
  double C_Hip_RForeArm[9], double C_Hip_RHand[9], double C_Spine_RArm[9],
  double C_Spine_RForeArm[9], double C_Spine_RHand[9], double C_Spine2_RArm[9],
  double C_Spine2_RForeArm[9], double C_Spine2_RHand[9], double C_Spine3_RArm[9],
  double C_Spine3_RForeArm[9], double C_Spine3_RHand[9])
{
  /* % Analyse Right Arm BVH */
  CalculateC_Multi(C_Spine3_RShoulder, C_RShoulder_RArm, C_Spine3_RArm);
  CalculateC_Multi(C_Spine2_Spine3, C_Spine3_RArm, C_Spine2_RArm);
  CalculateC_Multi(C_Spine_Spine3, C_Spine3_RArm, C_Spine_RArm);
  CalculateC_Multi(C_Hip_Spine3, C_Spine3_RArm, C_Hip_RArm);
  CalculateC_Multi(C_Spine3_RArm, C_RArm_RForeArm, C_Spine3_RForeArm);
  CalculateC_Multi(C_Spine2_RArm, C_RArm_RForeArm, C_Spine2_RForeArm);
  CalculateC_Multi(C_Spine_RArm, C_RArm_RForeArm, C_Spine_RForeArm);
  CalculateC_Multi(C_Hip_RArm, C_RArm_RForeArm, C_Hip_RForeArm);
  CalculateC_Multi(C_Spine3_RForeArm, C_RForeArm_RHand, C_Spine3_RHand);
  CalculateC_Multi(C_Spine2_RForeArm, C_RForeArm_RHand, C_Spine2_RHand);
  CalculateC_Multi(C_Spine_RForeArm, C_RForeArm_RHand, C_Spine_RHand);
  CalculateC_Multi(C_Hip_RForeArm, C_RForeArm_RHand, C_Hip_RHand);
}

/*
 * Arguments    : const double C12[9]
 *                double C21[9]
 * Return Type  : void
 */
static void InverseC_Multi(const double C12[9], double C21[9])
{
  double C12_k[9];
  int i11;
  int i12;
  memcpy(&C12_k[0], &C12[0], 9U * sizeof(double));
  for (i11 = 0; i11 < 3; i11++) {
    for (i12 = 0; i12 < 3; i12++) {
      C21[i12 + 3 * i11] = C12_k[i11 + 3 * i12];
    }
  }
}

/*
 * Arguments    : const double C_Spine_Head[9]
 *                const double C_Spine_NeckNew[9]
 *                double *HeadYaw
 *                double *HeadPitch
 *                double Head_new[3]
 * Return Type  : void
 */
static void TransformHead(const double C_Spine_Head[9], const double
  C_Spine_NeckNew[9], double *HeadYaw, double *HeadPitch, double Head_new[3])
{
  double C12_k[9];
  double C_NeckNew_Head[9];
  int i14;
  int i15;
  double dv26[3];
  double dv27[9];
  double E_Spine_Head[3];

  /* % TransformHead */
  /*  E_spine_head: euler from spine3 to head */
  /*  rotateOrder : rotate order of "E_spine_head" */
  memcpy(&C12_k[0], &C_Spine_NeckNew[0], 9U * sizeof(double));
  for (i14 = 0; i14 < 3; i14++) {
    for (i15 = 0; i15 < 3; i15++) {
      C_NeckNew_Head[i15 + 3 * i14] = C12_k[i14 + 3 * i15];
    }
  }

  for (i14 = 0; i14 < 3; i14++) {
    dv26[i14] = 1.0;
  }

  CalculateC_Multi(C_NeckNew_Head, C_Spine_Head, dv27);
  CToEuler_c(dv27, dv26, E_Spine_Head);
  E_Spine_Head[2] = 0.0;
  *HeadYaw = E_Spine_Head[0];
  *HeadPitch = E_Spine_Head[1];

  /* %% get new BVH */
  for (i14 = 0; i14 < 3; i14++) {
    dv26[i14] = 1.0;
  }

  b_EulerToC_c(E_Spine_Head, dv26, C_NeckNew_Head);
  for (i14 = 0; i14 < 3; i14++) {
    dv26[i14] = 1.0;
  }

  CToEuler_c(C_NeckNew_Head, dv26, Head_new);
}

/*
 * Arguments    : const double C_Spine_LForeArm[9]
 *                const double C_Spine_LArmNew[9]
 *                double LElbow_Nao[3]
 *                double LeftForeArm_new[3]
 * Return Type  : void
 */
static void TransformLElbow(const double C_Spine_LForeArm[9], const double
  C_Spine_LArmNew[9], double LElbow_Nao[3], double LeftForeArm_new[3])
{
  double C12_k[9];
  double b_C12_k[9];
  int i30;
  int i31;
  double C_LArmNew_LForeArm[9];
  double dv50[3];

  /*  %% LElbow */
  memcpy(&C12_k[0], &C_Spine_LArmNew[0], 9U * sizeof(double));
  for (i30 = 0; i30 < 3; i30++) {
    for (i31 = 0; i31 < 3; i31++) {
      b_C12_k[i31 + 3 * i30] = C12_k[i30 + 3 * i31];
    }
  }

  CalculateC_Multi(b_C12_k, C_Spine_LForeArm, C_LArmNew_LForeArm);

  /*      %%% LElbowRoll > 0  % 人很少做的动作：人左手不会这么转 */
  /*      LElbowRoll1 = acos(x) ; */
  /*      LElbowYaw1 = atan2( y,-z ); */
  /*      LElbow_Nao1 = [ LElbowYaw1 LElbowRoll1 0 ]; */
  /* %% LElbowRoll < 0  % 人正常动作 */
  LElbow_Nao[0] = rt_atan2d_snf(-C_LArmNew_LForeArm[3], C_LArmNew_LForeArm[6]);
  LElbow_Nao[1] = -acos(C_LArmNew_LForeArm[0]);
  LElbow_Nao[2] = 0.0;

  /* %% get new BVH */
  /* %  */
  for (i30 = 0; i30 < 3; i30++) {
    dv50[i30] = 1.0;
  }

  d_EulerToC_c(LElbow_Nao, dv50, C_LArmNew_LForeArm);
  for (i30 = 0; i30 < 3; i30++) {
    dv50[i30] = 1.0;
  }

  CToEuler_c(C_LArmNew_LForeArm, dv50, LeftForeArm_new);
}

/*
 * Arguments    : const double C_LLeg_LFoot[9]
 *                double *LAnklePitch
 *                double *LAnkleRoll
 *                double LeftFoot_new[3]
 * Return Type  : void
 */
static void TransformLFoot(const double C_LLeg_LFoot[9], double *LAnklePitch,
  double *LAnkleRoll, double LeftFoot_new[3])
{
  double dv69[3];
  int i46;
  double E_LLeg_LFoot[3];
  double C_RLeg_RFoot_New[9];

  /* % LFoot */
  for (i46 = 0; i46 < 3; i46++) {
    dv69[i46] = 1.0;
  }

  d_CToEuler_c(C_LLeg_LFoot, dv69, E_LLeg_LFoot);
  *LAnklePitch = E_LLeg_LFoot[0];
  *LAnkleRoll = E_LLeg_LFoot[1];

  /* %% get new BVH */
  E_LLeg_LFoot[2] = 0.0;
  for (i46 = 0; i46 < 3; i46++) {
    dv69[i46] = 1.0;
  }

  e_EulerToC_c(E_LLeg_LFoot, dv69, C_RLeg_RFoot_New);
  for (i46 = 0; i46 < 3; i46++) {
    dv69[i46] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, dv69, LeftFoot_new);
}

/*
 * Arguments    : const double C_Spine_LHand[9]
 *                const double C_Spine_LForeArmNew[9]
 *                double *LWristYaw
 *                double LeftHand_new[3]
 * Return Type  : void
 */
static void TransformLHand(const double C_Spine_LHand[9], const double
  C_Spine_LForeArmNew[9], double *LWristYaw, double LeftHand_new[3])
{
  double C12_k[9];
  double C_LForeArmNew_LHand[9];
  int i32;
  int i33;
  double dv51[3];
  double dv52[9];
  double E_LForeArmNew_LHand[3];
  double LHand_Nao[3];

  /* % LHand */
  memcpy(&C12_k[0], &C_Spine_LForeArmNew[0], 9U * sizeof(double));
  for (i32 = 0; i32 < 3; i32++) {
    for (i33 = 0; i33 < 3; i33++) {
      C_LForeArmNew_LHand[i33 + 3 * i32] = C12_k[i32 + 3 * i33];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    dv51[i32] = 1.0;
  }

  CalculateC_Multi(C_LForeArmNew_LHand, C_Spine_LHand, dv52);
  c_CToEuler_c(dv52, dv51, E_LForeArmNew_LHand);
  *LWristYaw = E_LForeArmNew_LHand[0];

  /* %% get new BVH */
  for (i32 = 0; i32 < 3; i32++) {
    LHand_Nao[i32] = 0.0;
  }

  LHand_Nao[0] = E_LForeArmNew_LHand[0];

  /* %  */
  for (i32 = 0; i32 < 3; i32++) {
    dv51[i32] = 1.0;
  }

  d_EulerToC_c(LHand_Nao, dv51, C_LForeArmNew_LHand);
  for (i32 = 0; i32 < 3; i32++) {
    dv51[i32] = 1.0;
  }

  CToEuler_c(C_LForeArmNew_LHand, dv51, LeftHand_new);
}

/*
 * Arguments    : const double C_Hip_LUpLegNew[9]
 *                const double C_Hip_LLeg[9]
 *                double *LKneePitch
 *                double LeftLeg_New[3]
 * Return Type  : void
 */
static void TransformLLeg(const double C_Hip_LUpLegNew[9], const double
  C_Hip_LLeg[9], double *LKneePitch, double LeftLeg_New[3])
{
  double C12_k[9];
  double C_LUpLegNew_LLeg[9];
  int i44;
  int i45;
  double dv67[3];
  double dv68[9];
  double E_LUpLegNew_LLeg[3];

  /* % LLeg */
  memcpy(&C12_k[0], &C_Hip_LUpLegNew[0], 9U * sizeof(double));
  for (i44 = 0; i44 < 3; i44++) {
    for (i45 = 0; i45 < 3; i45++) {
      C_LUpLegNew_LLeg[i45 + 3 * i44] = C12_k[i44 + 3 * i45];
    }
  }

  for (i44 = 0; i44 < 3; i44++) {
    dv67[i44] = 1.0;
  }

  CalculateC_Multi(C_LUpLegNew_LLeg, C_Hip_LLeg, dv68);
  c_CToEuler_c(dv68, dv67, E_LUpLegNew_LLeg);
  *LKneePitch = E_LUpLegNew_LLeg[0];

  /* %% get new BVH */
  for (i44 = 0; i44 < 2; i44++) {
    E_LUpLegNew_LLeg[1 + i44] = 0.0;
  }

  for (i44 = 0; i44 < 3; i44++) {
    dv67[i44] = 1.0;
  }

  d_EulerToC_c(E_LUpLegNew_LLeg, dv67, C_LUpLegNew_LLeg);
  for (i44 = 0; i44 < 3; i44++) {
    dv67[i44] = 1.0;
  }

  CToEuler_c(C_LUpLegNew_LLeg, dv67, LeftLeg_New);
}

/*
 * Arguments    : const double C_Spine3_LShoulderNew[9]
 *                const double C_Spine3_LArm[9]
 *                double *LShoulderPitch
 *                double *LShoulderRoll
 *                double LeftArm_new[3]
 * Return Type  : void
 */
static void TransformLShoulder(const double C_Spine3_LShoulderNew[9], const
  double C_Spine3_LArm[9], double *LShoulderPitch, double *LShoulderRoll, double
  LeftArm_new[3])
{
  double C12_k[9];
  double b_C12_k[9];
  int i28;
  int i29;
  double C_LShoulderNew_LArm[9];
  double b_C_LShoulderNew_LArm[9];
  double dv45[9];
  static const double dv46[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  double E_LShoulderNew_LArm_Nao[3];
  static const double dv47[3] = { -1.0, 1.0, 1.0 };

  double c_C_LShoulderNew_LArm[9];
  double dv48[9];
  static const double dv49[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  /* % LeftShoulder */
  memcpy(&C12_k[0], &C_Spine3_LShoulderNew[0], 9U * sizeof(double));
  for (i28 = 0; i28 < 3; i28++) {
    for (i29 = 0; i29 < 3; i29++) {
      b_C12_k[i29 + 3 * i28] = C12_k[i28 + 3 * i29];
    }
  }

  CalculateC_Multi(b_C12_k, C_Spine3_LArm, C_LShoulderNew_LArm);

  /* %% 对 C_LShoulderNew_LArm 进行零位补偿 */
  memcpy(&b_C_LShoulderNew_LArm[0], &C_LShoulderNew_LArm[0], 9U * sizeof(double));
  CalculateC_Multi(dv46, b_C_LShoulderNew_LArm, dv45);
  b_CToEuler_c(dv45, dv47, E_LShoulderNew_LArm_Nao);
  E_LShoulderNew_LArm_Nao[2] = 0.0;
  c_EulerToC_c(E_LShoulderNew_LArm_Nao, dv47, C_LShoulderNew_LArm);
  *LShoulderPitch = E_LShoulderNew_LArm_Nao[0];
  *LShoulderRoll = E_LShoulderNew_LArm_Nao[1];

  /* %% get new BVH */
  /*  零位补偿回去 */
  /*  'ZYX' */
  memcpy(&c_C_LShoulderNew_LArm[0], &C_LShoulderNew_LArm[0], 9U * sizeof(double));
  for (i28 = 0; i28 < 3; i28++) {
    E_LShoulderNew_LArm_Nao[i28] = 1.0;
  }

  CalculateC_Multi(dv49, c_C_LShoulderNew_LArm, dv48);
  CToEuler_c(dv48, E_LShoulderNew_LArm_Nao, LeftArm_new);
}

/*
 * Arguments    : const double C_Hip_LUpLeg[9]
 *                double HipYawPitch
 *                double *LHipRoll
 *                double *LHipPitch
 *                double LeftUpLeg_new[3]
 * Return Type  : void
 */
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double HipYawPitch,
  double *LHipRoll, double *LHipPitch, double LeftUpLeg_new[3])
{
  double C12_k[9];
  double x;
  double dv64[4];
  int i42;
  static const double a[3] = { 0.70710678118654746, -0.70710678118654746, 0.0 };

  double dv65[9];
  double C_HipYawPitch[9];
  double b_C12_k[9];
  int i43;
  double b_C_HipYawPitch[9];
  double dv66[3];

  /* % LUpLeg */
  /*  HipYawPitch：左右大腿的 HipYawPitch Angle  [ Nframes*1 ] */
  memcpy(&C12_k[0], &C_Hip_LUpLeg[0], 9U * sizeof(double));
  x = sin(HipYawPitch / 2.0);
  dv64[0] = cos(HipYawPitch / 2.0);
  for (i42 = 0; i42 < 3; i42++) {
    dv64[i42 + 1] = a[i42] * x;
  }

  FQtoCnb(dv64, dv65);
  memcpy(&C_HipYawPitch[0], &dv65[0], 9U * sizeof(double));
  for (i42 = 0; i42 < 3; i42++) {
    for (i43 = 0; i43 < 3; i43++) {
      b_C12_k[i43 + 3 * i42] = C12_k[i42 + 3 * i43];
    }
  }

  memcpy(&b_C_HipYawPitch[0], &C_HipYawPitch[0], 9U * sizeof(double));
  CalculateC_Multi(b_C12_k, b_C_HipYawPitch, C_HipYawPitch);
  memcpy(&C12_k[0], &C_HipYawPitch[0], 9U * sizeof(double));
  *LHipPitch = asin(C12_k[5]);
  *LHipRoll = atan(-C12_k[3] / C12_k[4]);

  /* %% get new BVH */
  memcpy(&C12_k[0], &C_HipYawPitch[0], 9U * sizeof(double));
  for (i42 = 0; i42 < 3; i42++) {
    for (i43 = 0; i43 < 3; i43++) {
      b_C12_k[i43 + 3 * i42] = C12_k[i42 + 3 * i43];
    }
  }

  for (i42 = 0; i42 < 3; i42++) {
    dv66[i42] = 1.0;
  }

  CToEuler_c(b_C12_k, dv66, LeftUpLeg_new);
}

/*
 * Arguments    : const double C_Spine_RForeArm[9]
 *                const double C_Spine_RArmNew[9]
 *                double RElbow_Nao[3]
 *                double RightForeArm_new[3]
 * Return Type  : void
 */
static void TransformRElbow(const double C_Spine_RForeArm[9], const double
  C_Spine_RArmNew[9], double RElbow_Nao[3], double RightForeArm_new[3])
{
  double C12_k[9];
  double b_C12_k[9];
  int i21;
  int i22;
  double C_RArmNew_RForeArm[9];
  static const double rotateDirection1[3] = { -1.0, 1.0, 1.0 };

  double dv38[3];

  /* % REblow */
  memcpy(&C12_k[0], &C_Spine_RArmNew[0], 9U * sizeof(double));
  for (i21 = 0; i21 < 3; i21++) {
    for (i22 = 0; i22 < 3; i22++) {
      b_C12_k[i22 + 3 * i21] = C12_k[i21 + 3 * i22];
    }
  }

  CalculateC_Multi(b_C12_k, C_Spine_RForeArm, C_RArmNew_RForeArm);

  /* %% RElbowRoll > 0  % 人正常动作 */
  RElbow_Nao[0] = rt_atan2d_snf(-C_RArmNew_RForeArm[3], -C_RArmNew_RForeArm[6]);
  RElbow_Nao[1] = acos(C_RArmNew_RForeArm[0]);
  RElbow_Nao[2] = 0.0;

  /*      %%% RElbowRoll < 0  % 人很少做的动作：人右手不会这么转 */
  /*      RElbowRoll2 = -acos(x) ; */
  /*      RElbowYaw2 = atan2( y,z ); */
  /*      RElbow_Nao2 = [ RElbowYaw2 RElbowRoll2 0 ]; */
  /* %% get new BVH */
  /* %  */
  d_EulerToC_c(RElbow_Nao, rotateDirection1, C_RArmNew_RForeArm);
  for (i21 = 0; i21 < 3; i21++) {
    dv38[i21] = 1.0;
  }

  CToEuler_c(C_RArmNew_RForeArm, dv38, RightForeArm_new);
}

/*
 * Arguments    : const double C_RLeg_RFoot[9]
 *                double *RAnklePitch
 *                double *RAnkleRoll
 *                double RightFoot_new[3]
 * Return Type  : void
 */
static void TransformRFoot(const double C_RLeg_RFoot[9], double *RAnklePitch,
  double *RAnkleRoll, double RightFoot_new[3])
{
  double dv58[3];
  int i38;
  double E_RLeg_RFoot[3];
  double C_RLeg_RFoot_New[9];

  /* % RFoot */
  for (i38 = 0; i38 < 3; i38++) {
    dv58[i38] = 1.0;
  }

  d_CToEuler_c(C_RLeg_RFoot, dv58, E_RLeg_RFoot);
  *RAnklePitch = E_RLeg_RFoot[0];
  *RAnkleRoll = E_RLeg_RFoot[1];

  /* %% get new BVH */
  E_RLeg_RFoot[2] = 0.0;
  for (i38 = 0; i38 < 3; i38++) {
    dv58[i38] = 1.0;
  }

  e_EulerToC_c(E_RLeg_RFoot, dv58, C_RLeg_RFoot_New);
  for (i38 = 0; i38 < 3; i38++) {
    dv58[i38] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, dv58, RightFoot_new);
}

/*
 * Arguments    : const double C_Spine_RHand[9]
 *                const double C_Spine_RForeArmNew[9]
 *                double *RWristYaw
 *                double RightHand_new[3]
 * Return Type  : void
 */
static void TransformRHand(const double C_Spine_RHand[9], const double
  C_Spine_RForeArmNew[9], double *RWristYaw, double RightHand_new[3])
{
  double C12_k[9];
  double C_RForeArmNew_RHand[9];
  double RHand_Nao[3];
  int i26;
  int i27;
  double dv44[9];
  double E_RForeArmNew_RHand[3];
  static const double rotateDirection1[3] = { -1.0, 1.0, 1.0 };

  /* % RHand */
  memcpy(&C12_k[0], &C_Spine_RForeArmNew[0], 9U * sizeof(double));
  for (i26 = 0; i26 < 3; i26++) {
    for (i27 = 0; i27 < 3; i27++) {
      C_RForeArmNew_RHand[i27 + 3 * i26] = C12_k[i26 + 3 * i27];
    }

    /* %% get new BVH */
    RHand_Nao[i26] = 0.0;
  }

  CalculateC_Multi(C_RForeArmNew_RHand, C_Spine_RHand, dv44);
  c_CToEuler_c(dv44, rotateDirection1, E_RForeArmNew_RHand);
  *RWristYaw = E_RForeArmNew_RHand[0];
  RHand_Nao[0] = E_RForeArmNew_RHand[0];

  /* %  */
  d_EulerToC_c(RHand_Nao, rotateDirection1, C_RForeArmNew_RHand);
  for (i26 = 0; i26 < 3; i26++) {
    RHand_Nao[i26] = 1.0;
  }

  CToEuler_c(C_RForeArmNew_RHand, RHand_Nao, RightHand_new);
}

/*
 * Arguments    : const double C_Hip_RUpLegNew[9]
 *                const double C_Hip_RLeg[9]
 *                double *RKneePitch
 *                double RightLeg_New[3]
 * Return Type  : void
 */
static void TransformRLeg(const double C_Hip_RUpLegNew[9], const double
  C_Hip_RLeg[9], double *RKneePitch, double RightLeg_New[3])
{
  double C12_k[9];
  double C_RUpLegNew_RLeg[9];
  int i36;
  int i37;
  double dv56[3];
  double dv57[9];
  double E_LUpLegNew_LLeg[3];

  /* % RLeg */
  memcpy(&C12_k[0], &C_Hip_RUpLegNew[0], 9U * sizeof(double));
  for (i36 = 0; i36 < 3; i36++) {
    for (i37 = 0; i37 < 3; i37++) {
      C_RUpLegNew_RLeg[i37 + 3 * i36] = C12_k[i36 + 3 * i37];
    }
  }

  for (i36 = 0; i36 < 3; i36++) {
    dv56[i36] = 1.0;
  }

  CalculateC_Multi(C_RUpLegNew_RLeg, C_Hip_RLeg, dv57);
  c_CToEuler_c(dv57, dv56, E_LUpLegNew_LLeg);
  *RKneePitch = E_LUpLegNew_LLeg[0];

  /*  E_UpLeg_Leg = CToEuler_c( C_RUpLeg_RLeg,'XYZ' ); */
  /*  RKneePitch = E_UpLeg_Leg(:,1)'; */
  /* %% get new BVH */
  for (i36 = 0; i36 < 2; i36++) {
    E_LUpLegNew_LLeg[1 + i36] = 0.0;
  }

  for (i36 = 0; i36 < 3; i36++) {
    dv56[i36] = 1.0;
  }

  d_EulerToC_c(E_LUpLegNew_LLeg, dv56, C_RUpLegNew_RLeg);
  for (i36 = 0; i36 < 3; i36++) {
    dv56[i36] = 1.0;
  }

  CToEuler_c(C_RUpLegNew_RLeg, dv56, RightLeg_New);
}

/*
 * Arguments    : const double C_Spine3_RShoulderNew[9]
 *                const double C_Spine3_RArm[9]
 *                double *RShoulderPitch
 *                double *RShoulderRoll
 *                double RightArm_new[3]
 * Return Type  : void
 */
static void TransformRShoulder(const double C_Spine3_RShoulderNew[9], const
  double C_Spine3_RArm[9], double *RShoulderPitch, double *RShoulderRoll, double
  RightArm_new[3])
{
  double C12_k[9];
  double b_C12_k[9];
  int i16;
  int i17;
  double C_RShoulderNew_RArm[9];
  double b_C_RShoulderNew_RArm[9];
  double dv28[3];
  double dv29[9];
  static const double dv30[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  double E_RShoulderNew_RArm_Nao[3];
  double c_C_RShoulderNew_RArm[9];
  double dv31[9];
  static const double dv32[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  /* % RightShoulder */
  memcpy(&C12_k[0], &C_Spine3_RShoulderNew[0], 9U * sizeof(double));
  for (i16 = 0; i16 < 3; i16++) {
    for (i17 = 0; i17 < 3; i17++) {
      b_C12_k[i17 + 3 * i16] = C12_k[i16 + 3 * i17];
    }
  }

  CalculateC_Multi(b_C12_k, C_Spine3_RArm, C_RShoulderNew_RArm);

  /* %% 对 C_RShoulderNew_RArm 进行零位补偿 */
  memcpy(&b_C_RShoulderNew_RArm[0], &C_RShoulderNew_RArm[0], 9U * sizeof(double));
  for (i16 = 0; i16 < 3; i16++) {
    dv28[i16] = 1.0;
  }

  CalculateC_Multi(dv30, b_C_RShoulderNew_RArm, dv29);
  b_CToEuler_c(dv29, dv28, E_RShoulderNew_RArm_Nao);
  E_RShoulderNew_RArm_Nao[2] = 0.0;
  for (i16 = 0; i16 < 3; i16++) {
    dv28[i16] = 1.0;
  }

  c_EulerToC_c(E_RShoulderNew_RArm_Nao, dv28, C_RShoulderNew_RArm);
  *RShoulderPitch = E_RShoulderNew_RArm_Nao[0];
  *RShoulderRoll = E_RShoulderNew_RArm_Nao[1];

  /* %% get new BVH */
  /*  零位补偿回去 */
  /*  'ZYX' */
  memcpy(&c_C_RShoulderNew_RArm[0], &C_RShoulderNew_RArm[0], 9U * sizeof(double));
  for (i16 = 0; i16 < 3; i16++) {
    dv28[i16] = 1.0;
  }

  CalculateC_Multi(dv32, c_C_RShoulderNew_RArm, dv31);
  CToEuler_c(dv31, dv28, RightArm_new);
}

/*
 * Arguments    : const double C_Hip_RUpLeg[9]
 *                double HipYawPitch
 *                double *RHipRoll
 *                double *RHipPitch
 *                double RightUpLeg_new[3]
 * Return Type  : void
 */
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double HipYawPitch,
  double *RHipRoll, double *RHipPitch, double RightUpLeg_new[3])
{
  double C12_k[9];
  double x;
  double dv53[4];
  int i34;
  static const double a[3] = { 0.70710678118654746, 0.70710678118654746, 0.0 };

  double dv54[9];
  double C_HipYawPitch[9];
  double b_C12_k[9];
  int i35;
  double b_C_HipYawPitch[9];
  double dv55[3];

  /* % RUpLeg */
  /*  HipYawPitch：左右大腿的 HipYawPitch Angle  [ 1*Nframes ] */
  memcpy(&C12_k[0], &C_Hip_RUpLeg[0], 9U * sizeof(double));
  x = sin(HipYawPitch / 2.0);
  dv53[0] = cos(HipYawPitch / 2.0);
  for (i34 = 0; i34 < 3; i34++) {
    dv53[i34 + 1] = a[i34] * x;
  }

  FQtoCnb(dv53, dv54);
  memcpy(&C_HipYawPitch[0], &dv54[0], 9U * sizeof(double));
  for (i34 = 0; i34 < 3; i34++) {
    for (i35 = 0; i35 < 3; i35++) {
      b_C12_k[i35 + 3 * i34] = C12_k[i34 + 3 * i35];
    }
  }

  memcpy(&b_C_HipYawPitch[0], &C_HipYawPitch[0], 9U * sizeof(double));
  CalculateC_Multi(b_C12_k, b_C_HipYawPitch, C_HipYawPitch);
  memcpy(&C12_k[0], &C_HipYawPitch[0], 9U * sizeof(double));
  *RHipPitch = asin(C12_k[5]);
  *RHipRoll = atan(-C12_k[3] / C12_k[4]);

  /* %% get new BVH */
  memcpy(&C12_k[0], &C_HipYawPitch[0], 9U * sizeof(double));
  for (i34 = 0; i34 < 3; i34++) {
    for (i35 = 0; i35 < 3; i35++) {
      b_C12_k[i35 + 3 * i34] = C12_k[i34 + 3 * i35];
    }
  }

  for (i34 = 0; i34 < 3; i34++) {
    dv55[i34] = 1.0;
  }

  CToEuler_c(b_C12_k, dv55, RightUpLeg_new);
}

/*
 * Arguments    : const double BVHData[354]
 *                double NaoData[25]
 * Return Type  : void
 */
void BVHTransform_c(const double BVHData[354], double NaoData[25])
{
  double b_BVHData[354];
  int i0;
  double C_LLeg_LFoot[9];
  double C_LUpLeg_LLeg[9];
  double C_Hip_LUpLeg[9];
  double C_RLeg_RFoot[9];
  double C_RUpLeg_RLeg[9];
  double C_Hip_RUpLeg[9];
  double C_RForeArm_RHand[9];
  double C_RArm_RForeArm[9];
  double C_RShoulder_RArm[9];
  double C_Spine3_RShoulder[9];
  double C_LForeArm_LHand[9];
  double C_LArm_LForeArm[9];
  double C_I[9];
  double C_World_Spine2[9];
  double C_Neck_Head[9];
  double C_Spine3_Neck[9];
  double C_Spine2_Spine3[9];
  double C_Spine2_Head[9];
  double C_Spine3_Head[9];
  double C_Spine3New_RArmNew[9];
  double C_World_Hip[9];
  double b_C_Spine3_Head[9];
  double b_C_Spine2_Head[9];
  double b_C_World_Spine2[9];
  double b_C_I[9];
  double b_C_LArm_LForeArm[9];
  double b_C_LForeArm_LHand[9];
  double b_C_Spine3_RShoulder[9];
  double b_C_RShoulder_RArm[9];
  double b_C_RArm_RForeArm[9];
  double C_Spine3_LHand[9];
  double C_Spine3_LForeArm[9];
  double C_Spine2_LHand[9];
  double C_Spine2_LForeArm[9];
  double C_Spine2_LArm[9];
  double C_Spine_Spine3[9];
  double C_Hip_Spine3[9];
  double C_Hip_Spine2[9];
  double dv0[9];
  double dv1[9];
  double dv2[9];
  double dv3[9];
  double Head_new[3];
  double HeadPitch;
  double HeadYaw;
  double RShoulderRoll;
  double RShoulderPitch;
  double dv4[3];
  double c_C_Spine2_Head[9];
  double RElbow_Nao[3];
  double c_C_World_Spine2[9];
  double dv5[9];
  double RWristYaw;
  double dv6[9];
  double LShoulderRoll;
  double LShoulderPitch;
  static const double dv7[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  double dv8[9];
  double LElbow_Nao[3];
  double d_C_World_Spine2[9];
  double dv9[9];
  double dv10[9];
  double LWristYaw;
  double dv11[9];
  double RHipPitch;
  double RHipRoll;
  double e_C_World_Spine2[9];
  double dv12[9];
  double RKneePitch;
  double RAnkleRoll;
  double RAnklePitch;
  double dv13[9];
  double LHipPitch;
  double LHipRoll;
  double f_C_World_Spine2[9];
  double dv14[9];
  double LKneePitch;
  double LAnkleRoll;
  double LAnklePitch;
  double b_HeadYaw[25];

  /* % xyz  2015.3.25 */
  /*  Transform BVH data Format */
  /*  BVHData: 1*354  degree */
  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = BVHData[i0] * 3.1415926535897931 / 180.0;
  }

  BVHDataToRotate_c(b_BVHData, C_World_Hip, C_Spine3New_RArmNew, C_Spine3_Head,
                    C_Spine2_Head, C_Spine2_Spine3, C_Spine3_Neck, C_Neck_Head,
                    C_World_Spine2, C_I, C_LArm_LForeArm, C_LForeArm_LHand,
                    C_Spine3_RShoulder, C_RShoulder_RArm, C_RArm_RForeArm,
                    C_RForeArm_RHand, C_Hip_RUpLeg, C_RUpLeg_RLeg, C_RLeg_RFoot,
                    C_Hip_LUpLeg, C_LUpLeg_LLeg, C_LLeg_LFoot);

  /* % 所有BVH的旋转数据 */
  for (i0 = 0; i0 < 9; i0++) {
    C_World_Hip[i0] = C_Spine3New_RArmNew[i0];
    b_C_Spine3_Head[i0] = C_Spine3_Head[i0];
    b_C_Spine2_Head[i0] = C_Spine2_Head[i0];
    b_C_World_Spine2[i0] = C_World_Spine2[i0];
    b_C_I[i0] = C_I[i0];
    b_C_LArm_LForeArm[i0] = C_LArm_LForeArm[i0];
    b_C_LForeArm_LHand[i0] = C_LForeArm_LHand[i0];
    b_C_Spine3_RShoulder[i0] = C_Spine3_RShoulder[i0];
    b_C_RShoulder_RArm[i0] = C_RShoulder_RArm[i0];
    b_C_RArm_RForeArm[i0] = C_RArm_RForeArm[i0];
  }

  CalculatedLeftArmRotate(C_World_Hip, b_C_Spine3_Head, b_C_Spine2_Head,
    C_Spine2_Spine3, b_C_World_Spine2, b_C_I, b_C_LArm_LForeArm,
    b_C_LForeArm_LHand, C_Hip_Spine2, C_Hip_Spine3, C_Spine_Spine3,
    C_Spine3New_RArmNew, C_Spine3_Head, C_Spine2_Head, C_World_Spine2, C_I,
    C_LArm_LForeArm, C_Spine2_LArm, C_Spine2_LForeArm, C_Spine2_LHand,
    C_LForeArm_LHand, C_Spine3_LForeArm, C_Spine3_LHand);
  CalculatedRightArmRotate(C_Hip_Spine3, C_Spine2_Spine3, C_Spine_Spine3,
    b_C_Spine3_RShoulder, b_C_RShoulder_RArm, b_C_RArm_RForeArm,
    C_RForeArm_RHand, C_Spine3New_RArmNew, C_Spine3_Head, C_Spine2_Head,
    C_World_Spine2, C_I, C_LArm_LForeArm, C_Spine3_RShoulder, C_RShoulder_RArm,
    C_RArm_RForeArm, C_LForeArm_LHand, C_Spine3_LForeArm, C_Spine3_LHand);
  CalculatedLLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_LUpLeg, C_LUpLeg_LLeg,
                       C_LForeArm_LHand, C_Spine3New_RArmNew, C_Spine3_Head);
  CalculatedRLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_RUpLeg, C_RUpLeg_RLeg,
                       C_LArm_LForeArm, C_Spine3New_RArmNew, C_Spine3_Head);

  /* % common parameters to set */
  /*  reference joint : 'Hip'  'SPine'  'Spine3' */
  eye(dv0);
  memcpy(&C_I[0], &dv0[0], 9U * sizeof(double));

  /* %% New Hip */
  /* % head */
  CalculateC_Multi(C_Spine3_Neck, C_Neck_Head, dv1);
  CalculateC_Multi(C_Spine2_Spine3, dv1, dv2);
  CalculateC_Multi(C_I, C_I, dv3);
  TransformHead(dv2, dv3, &HeadYaw, &HeadPitch, Head_new);

  /* % Right Shoulder */
  CalculateC_Multi(C_I, C_I, C_Spine3_Head);
  TransformRShoulder(C_Spine3_Head, C_Spine3_RShoulder, &RShoulderPitch,
                     &RShoulderRoll, Head_new);

  /* % Right Elbow */
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_Spine2_Head);
  memcpy(&c_C_Spine2_Head[0], &C_Spine2_Head[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, c_C_Spine2_Head, C_Spine2_Head);
  TransformRElbow(C_RShoulder_RArm, C_Spine2_Head, RElbow_Nao, Head_new);

  /* % RHand */
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_World_Spine2);
  memcpy(&c_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, c_C_World_Spine2, dv5);
  TransformRHand(C_RArm_RForeArm, dv5, &RWristYaw, Head_new);

  /* % Left Shoulder */
  CalculateC_Multi(C_I, C_I, dv6);
  TransformLShoulder(dv6, C_Spine2_LArm, &LShoulderPitch, &LShoulderRoll,
                     Head_new);

  /* % Left Elbow */
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_World_Spine2);
  CalculateC_Multi(dv7, C_World_Spine2, C_Spine2_Head);
  CalculateC_Multi(C_I, C_Spine2_Head, dv8);
  TransformLElbow(C_Spine2_LForeArm, dv8, LElbow_Nao, Head_new);

  /* % LHand */
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_World_Spine2);
  memcpy(&d_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, d_C_World_Spine2, dv9);
  CalculateC_Multi(C_I, dv9, dv10);
  TransformLHand(C_Spine2_LHand, dv10, &LWristYaw, Head_new);

  /* % RLeg */
  InverseC_Multi(C_Hip_Spine2, C_Spine3_Head);
  CalculateC_Multi(C_Spine3_Head, C_Hip_RUpLeg, dv11);
  TransformRUpLeg(dv11, 0.0, &RHipRoll, &RHipPitch, Head_new);
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_World_Spine2);
  memcpy(&e_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, e_C_World_Spine2, dv12);
  TransformRLeg(dv12, C_LArm_LForeArm, &RKneePitch, Head_new);
  TransformRFoot(C_RLeg_RFoot, &RAnklePitch, &RAnkleRoll, Head_new);

  /* % LLeg */
  CalculateC_Multi(C_Spine3_Head, C_Hip_LUpLeg, dv13);
  TransformLUpLeg(dv13, 0.0, &LHipRoll, &LHipPitch, Head_new);
  for (i0 = 0; i0 < 3; i0++) {
    dv4[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, dv4, C_World_Spine2);
  memcpy(&f_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, f_C_World_Spine2, dv14);
  TransformLLeg(dv14, C_LForeArm_LHand, &LKneePitch, Head_new);
  TransformLFoot(C_LLeg_LFoot, &LAnklePitch, &LAnkleRoll, Head_new);

  /* % Get Nao */
  b_HeadYaw[0] = HeadYaw;
  b_HeadYaw[1] = HeadPitch;
  b_HeadYaw[2] = LShoulderPitch;
  b_HeadYaw[3] = LShoulderRoll;
  b_HeadYaw[4] = LElbow_Nao[0];
  b_HeadYaw[5] = LElbow_Nao[1];
  b_HeadYaw[6] = RShoulderPitch;
  b_HeadYaw[7] = RShoulderRoll;
  b_HeadYaw[8] = RElbow_Nao[0];
  b_HeadYaw[9] = RElbow_Nao[1];
  b_HeadYaw[10] = 0.0;
  b_HeadYaw[11] = LHipRoll;
  b_HeadYaw[12] = LHipPitch;
  b_HeadYaw[13] = LKneePitch;
  b_HeadYaw[14] = RHipRoll;
  b_HeadYaw[15] = RHipPitch;
  b_HeadYaw[16] = RKneePitch;
  b_HeadYaw[17] = LAnklePitch;
  b_HeadYaw[18] = LAnkleRoll;
  b_HeadYaw[19] = RAnklePitch;
  b_HeadYaw[20] = RAnkleRoll;
  b_HeadYaw[21] = LWristYaw;
  b_HeadYaw[22] = RWristYaw;
  b_HeadYaw[23] = 0.0;
  b_HeadYaw[24] = 0.0;
  for (i0 = 0; i0 < 25; i0++) {
    NaoData[i0] = b_HeadYaw[i0] * 180.0 / 3.1415926535897931;
  }
}

/*
 * File trailer for BVHTransform_c.c
 *
 * [EOF]
 */
