//
// File: BVHTransform_c.cpp
//
// MATLAB Coder version            : 2.6
// C/C++ source code generated on  : 06-Apr-2015 14:16:46
//

// Include files
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"

// Function Declarations
static void BVHDataToRotate_c(const double BVHData[354], double C_World_Hip[9],
  double C_Hip_Spine[9], double C_Spine_Spine1[9], double C_Spine1_Spine2[9],
  double C_Spine2_Spine3[9], double C_Spine3_Neck[9], double C_Neck_Head[9],
  double C_Spine3_LShoulder[9], double C_LShoulder_LArm[9], double
  C_LArm_LForeArm[9], double C_LForeArm_LHand[9], double C_Spine3_RShoulder[9],
  double C_RShoulder_RArm[9], double C_RArm_RForeArm[9], double
  C_RForeArm_RHand[9], double C_Hip_RUpLeg[9], double C_RUpLeg_RLeg[9], double
  C_RLeg_RFoot[9], double C_Hip_LUpLeg[9], double C_LUpLeg_LLeg[9], double
  C_LLeg_LFoot[9]);
static void CToEuler_c(const double C[9], double rotateOrder, const double
  rotateDirection[3], double euler[3]);
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
static void ChangeEulerRotate_c(const double euler1[3], double rotateOrder1,
  double rotateOrder2, const double rotateDirection1[3], const double
  rotateDirection2[3], double euler2[3]);
static void EulerToC_c(const double euler[6], double rotateOrder, const double
  rotateDirection[3], double C[9]);
static void FQtoCnb(double Q[4], double Cnb[9]);
static void InverseC_Multi(const double C12[9], double C21[9]);
static void OneCToEuler_c(const double C[9], double rotateOrder, const double
  rotateDirection[3], double euler[3]);
static void OneEulerToC_c(double euler[6], double rotateOrder, const double
  rotateDirection[3], double C[9]);
static void RotateX(double angX, double Cx[9]);
static void RotateY(double angY, double Cy[9]);
static void RotateZ(double angZ, double Cz[9]);
static void TransformHead(const double C_Spine_Head[9], const double
  C_Spine_NeckNew[9], double rotateOrder_BVH, double *HeadYaw, double *HeadPitch,
  double Head_new[3]);
static void TransformLElbow(const double C_Spine_LForeArm[9], const double
  C_Spine_LArmNew[9], double rotateOrder_BVH, double LElbow_Nao[3], double
  LeftForeArm_new[3]);
static void TransformLFoot(const double C_LLeg_LFoot[9], double rotateOrder_BVH,
  double *LAnklePitch, double *LAnkleRoll, double LeftFoot_new[3]);
static void TransformLHand(const double C_Spine_LHand[9], const double
  C_Spine_LForeArmNew[9], double rotateOrder_BVH, double *LWristYaw, double
  LeftHand_new[3]);
static void TransformLLeg(const double C_Hip_LUpLegNew[9], const double
  C_Hip_LLeg[9], double rotateOrder_BVH, double *LKneePitch, double LeftLeg_New
  [3]);
static void TransformLShoulder(const double C_Spine3_LShoulderNew[9], const
  double C_Spine3_LArm[9], double rotateOrder_BVH, double *LShoulderPitch,
  double *LShoulderRoll, double LeftArm_new[3]);
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double HipYawPitch,
  double rotateOrder_BVH, double *LHipRoll, double *LHipPitch, double
  LeftUpLeg_new[3]);
static void TransformRElbow(const double C_Spine_RForeArm[9], const double
  C_Spine_RArmNew[9], double rotateOrder_BVH, double RElbow_Nao[3], double
  RightForeArm_new[3]);
static void TransformRFoot(const double C_RLeg_RFoot[9], double rotateOrder_BVH,
  double *RAnklePitch, double *RAnkleRoll, double RightFoot_new[3]);
static void TransformRHand(const double C_Spine_RHand[9], const double
  C_Spine_RForeArmNew[9], double rotateOrder_BVH, double *RWristYaw, double
  RightHand_new[3]);
static void TransformRLeg(const double C_Hip_RUpLegNew[9], const double
  C_Hip_RLeg[9], double rotateOrder_BVH, double *RKneePitch, double
  RightLeg_New[3]);
static void TransformRShoulder(const double C_Spine3_RShoulderNew[9], const
  double C_Spine3_RArm[9], double rotateOrder_BVH, double *RShoulderPitch,
  double *RShoulderRoll, double RightArm_new[3]);
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double HipYawPitch,
  double rotateOrder_BVH, double *RHipRoll, double *RHipPitch, double
  RightUpLeg_new[3]);
static void b_EulerToC_c(const double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9]);
static void b_OneEulerToC_c(double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9]);
static double eml_switch_helper(double expr, double varargin_1, double
  varargin_2, double varargin_3, double varargin_4, double varargin_5);
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : const double BVHData[354]
//                double C_World_Hip[9]
//                double C_Hip_Spine[9]
//                double C_Spine_Spine1[9]
//                double C_Spine1_Spine2[9]
//                double C_Spine2_Spine3[9]
//                double C_Spine3_Neck[9]
//                double C_Neck_Head[9]
//                double C_Spine3_LShoulder[9]
//                double C_LShoulder_LArm[9]
//                double C_LArm_LForeArm[9]
//                double C_LForeArm_LHand[9]
//                double C_Spine3_RShoulder[9]
//                double C_RShoulder_RArm[9]
//                double C_RArm_RForeArm[9]
//                double C_RForeArm_RHand[9]
//                double C_Hip_RUpLeg[9]
//                double C_RUpLeg_RLeg[9]
//                double C_RLeg_RFoot[9]
//                double C_Hip_LUpLeg[9]
//                double C_LUpLeg_LLeg[9]
//                double C_LLeg_LFoot[9]
// Return Type  : void
//
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
  double dv13[3];
  int i1;

  //  BVHData: Nframes*(59*6)   [ 1*354 ]
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  EulerToC_c(*(double (*)[6])&BVHData[0], 213.0, dv13, C_World_Hip);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[45], 213.0, dv13, C_Hip_Spine);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[51], 213.0, dv13, C_Spine_Spine1);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[57], 213.0, dv13, C_Spine1_Spine2);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[63], 213.0, dv13, C_Spine2_Spine3);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[69], 213.0, dv13, C_Spine3_Neck);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[75], 213.0, dv13, C_Neck_Head);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[219], 213.0, dv13, C_Spine3_LShoulder);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[225], 213.0, dv13, C_LShoulder_LArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[231], 213.0, dv13, C_LArm_LForeArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[237], 213.0, dv13, C_LForeArm_LHand);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[81], 213.0, dv13, C_Spine3_RShoulder);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[87], 213.0, dv13, C_RShoulder_RArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[93], 213.0, dv13, C_RArm_RForeArm);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[99], 213.0, dv13, C_RForeArm_RHand);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[9], 213.0, dv13, C_Hip_RUpLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[15], 213.0, dv13, C_RUpLeg_RLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[21], 213.0, dv13, C_RLeg_RFoot);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[27], 213.0, dv13, C_Hip_LUpLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[33], 213.0, dv13, C_LUpLeg_LLeg);
  for (i1 = 0; i1 < 3; i1++) {
    dv13[i1] = 1.0;
  }

  b_EulerToC_c(*(double (*)[3])&BVHData[39], 213.0, dv13, C_LLeg_LFoot);
}

//
// Arguments    : const double C[9]
//                double rotateOrder
//                const double rotateDirection[3]
//                double euler[3]
// Return Type  : void
//
static void CToEuler_c(const double C[9], double rotateOrder, const double
  rotateDirection[3], double euler[3])
{
  double C_K[9];
  double dv28[3];
  int i15;

  // % xyz  2015.4.5
  //  direction cosine matrix  to Euler angle  for multi C
  //  C: [N*3*3]
  //        euler = [eu_x,eu_y,eu_z]  ( Nframes*3 )
  //  store order of euler angle: the rotate order
  //  positive:anticlockwise rotate
  //  the first and third euler angles scope: [-pi,pi]
  //  the second euler angles scope: [-pi/2,pi/2]
  //  unit:rad
  //  rotateDirection: anticlockwise(1) or clockwise(-1)  111
  memcpy(&C_K[0], &C[0], 9U * sizeof(double));
  OneCToEuler_c(C_K, rotateOrder, rotateDirection, dv28);
  for (i15 = 0; i15 < 3; i15++) {
    euler[i15] = dv28[i15];
  }
}

//
// Arguments    : const double C12[9]
//                const double C23[9]
//                double C13[9]
// Return Type  : void
//
static void CalculateC_Multi(const double C12[9], const double C23[9], double
  C13[9])
{
  double C23_k[9];
  double C12_k[9];
  int k;
  double b_C23_k[9];
  int i11;
  int i12;
  for (k = 0; k < 9; k++) {
    C23_k[k] = C23[k];
    C12_k[k] = C12[k];
  }

  for (k = 0; k < 3; k++) {
    for (i11 = 0; i11 < 3; i11++) {
      b_C23_k[k + 3 * i11] = 0.0;
      for (i12 = 0; i12 < 3; i12++) {
        b_C23_k[k + 3 * i11] += C23_k[k + 3 * i12] * C12_k[i12 + 3 * i11];
      }
    }
  }

  for (k = 0; k < 3; k++) {
    for (i11 = 0; i11 < 3; i11++) {
      C13[i11 + 3 * k] = b_C23_k[i11 + 3 * k];
    }
  }
}

//
// Arguments    : const double C_Hip_Spine2[9]
//                const double C_Hip_Spine3[9]
//                const double C_Hip_LUpLeg[9]
//                const double C_LUpLeg_LLeg[9]
//                double C_Hip_LLeg[9]
//                double C_Spine2_LLeg[9]
//                double C_Spine3_LLeg[9]
// Return Type  : void
//
static void CalculatedLLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_LUpLeg[9], const double C_LUpLeg_LLeg[9],
  double C_Hip_LLeg[9], double C_Spine2_LLeg[9], double C_Spine3_LLeg[9])
{
  double dv24[9];
  double dv25[9];
  CalculateC_Multi(C_Hip_LUpLeg, C_LUpLeg_LLeg, C_Hip_LLeg);
  InverseC_Multi(C_Hip_Spine2, dv24);
  CalculateC_Multi(dv24, C_Hip_LLeg, C_Spine2_LLeg);
  InverseC_Multi(C_Hip_Spine3, dv25);
  CalculateC_Multi(dv25, C_Hip_LLeg, C_Spine3_LLeg);
}

//
// Arguments    : const double C_Hip_Spine[9]
//                const double C_Spine_Spine1[9]
//                const double C_Spine1_Spine2[9]
//                const double C_Spine2_Spine3[9]
//                const double C_Spine3_LShoulder[9]
//                const double C_LShoulder_LArm[9]
//                const double C_LArm_LForeArm[9]
//                const double C_LForeArm_LHand[9]
//                double C_Hip_Spine2[9]
//                double C_Hip_Spine3[9]
//                double C_Spine_Spine3[9]
//                double C_Hip_LArm[9]
//                double C_Hip_LForeArm[9]
//                double C_Hip_LHand[9]
//                double C_Spine_LArm[9]
//                double C_Spine_LForeArm[9]
//                double C_Spine_LHand[9]
//                double C_Spine2_LArm[9]
//                double C_Spine2_LForeArm[9]
//                double C_Spine2_LHand[9]
//                double C_Spine3_LArm[9]
//                double C_Spine3_LForeArm[9]
//                double C_Spine3_LHand[9]
// Return Type  : void
//
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

  // % Analyse Left Arm BVH
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

//
// Arguments    : const double C_Hip_Spine2[9]
//                const double C_Hip_Spine3[9]
//                const double C_Hip_RUpLeg[9]
//                const double C_RUpLeg_RLeg[9]
//                double C_Hip_RLeg[9]
//                double C_Spine2_RLeg[9]
//                double C_Spine3_RLeg[9]
// Return Type  : void
//
static void CalculatedRLegRotate(const double C_Hip_Spine2[9], const double
  C_Hip_Spine3[9], const double C_Hip_RUpLeg[9], const double C_RUpLeg_RLeg[9],
  double C_Hip_RLeg[9], double C_Spine2_RLeg[9], double C_Spine3_RLeg[9])
{
  double dv26[9];
  double dv27[9];
  CalculateC_Multi(C_Hip_RUpLeg, C_RUpLeg_RLeg, C_Hip_RLeg);
  InverseC_Multi(C_Hip_Spine2, dv26);
  CalculateC_Multi(dv26, C_Hip_RLeg, C_Spine2_RLeg);
  InverseC_Multi(C_Hip_Spine3, dv27);
  CalculateC_Multi(dv27, C_Hip_RLeg, C_Spine3_RLeg);
}

//
// Arguments    : const double C_Hip_Spine3[9]
//                const double C_Spine2_Spine3[9]
//                const double C_Spine_Spine3[9]
//                const double C_Spine3_RShoulder[9]
//                const double C_RShoulder_RArm[9]
//                const double C_RArm_RForeArm[9]
//                const double C_RForeArm_RHand[9]
//                double C_Hip_RArm[9]
//                double C_Hip_RForeArm[9]
//                double C_Hip_RHand[9]
//                double C_Spine_RArm[9]
//                double C_Spine_RForeArm[9]
//                double C_Spine_RHand[9]
//                double C_Spine2_RArm[9]
//                double C_Spine2_RForeArm[9]
//                double C_Spine2_RHand[9]
//                double C_Spine3_RArm[9]
//                double C_Spine3_RForeArm[9]
//                double C_Spine3_RHand[9]
// Return Type  : void
//
static void CalculatedRightArmRotate(const double C_Hip_Spine3[9], const double
  C_Spine2_Spine3[9], const double C_Spine_Spine3[9], const double
  C_Spine3_RShoulder[9], const double C_RShoulder_RArm[9], const double
  C_RArm_RForeArm[9], const double C_RForeArm_RHand[9], double C_Hip_RArm[9],
  double C_Hip_RForeArm[9], double C_Hip_RHand[9], double C_Spine_RArm[9],
  double C_Spine_RForeArm[9], double C_Spine_RHand[9], double C_Spine2_RArm[9],
  double C_Spine2_RForeArm[9], double C_Spine2_RHand[9], double C_Spine3_RArm[9],
  double C_Spine3_RForeArm[9], double C_Spine3_RHand[9])
{
  // % Analyse Right Arm BVH
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

//
// Arguments    : const double euler1[3]
//                double rotateOrder1
//                double rotateOrder2
//                const double rotateDirection1[3]
//                const double rotateDirection2[3]
//                double euler2[3]
// Return Type  : void
//
static void ChangeEulerRotate_c(const double euler1[3], double rotateOrder1,
  double rotateOrder2, const double rotateDirection1[3], const double
  rotateDirection2[3], double euler2[3])
{
  double C[9];

  // % Change Euler Rotate Order
  b_EulerToC_c(euler1, rotateOrder1, rotateDirection1, C);
  CToEuler_c(C, rotateOrder2, rotateDirection2, euler2);
}

//
// Arguments    : const double euler[6]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void EulerToC_c(const double euler[6], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  double b_euler[6];
  int i2;
  double dv14[9];

  // % xyz 2015.4.5
  //   EulerToC_Multi  新版 ： 可转化为 C++
  //    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]
  //  store order of euler angle: the rotate order
  //  positive:anticlockwise rotate
  //  euler angle scope: [-pi,pi]
  //  unit:rad
  //  rotateDirection: anticlockwise(1) or clockwise(-1)  111
  for (i2 = 0; i2 < 6; i2++) {
    b_euler[i2] = euler[i2];
  }

  OneEulerToC_c(b_euler, rotateOrder, rotateDirection, dv14);
  memcpy(&C[0], &dv14[0], 9U * sizeof(double));
}

//
// Arguments    : double Q[4]
//                double Cnb[9]
// Return Type  : void
//
static void FQtoCnb(double Q[4], double Cnb[9])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;

  // % 四元数 -> 方向余弦矩阵
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

//
// Arguments    : const double C12[9]
//                double C21[9]
// Return Type  : void
//
static void InverseC_Multi(const double C12[9], double C21[9])
{
  double C12_k[9];
  int i13;
  int i14;
  memcpy(&C12_k[0], &C12[0], 9U * sizeof(double));
  for (i13 = 0; i13 < 3; i13++) {
    for (i14 = 0; i14 < 3; i14++) {
      C21[i14 + 3 * i13] = C12_k[i13 + 3 * i14];
    }
  }
}

//
// Arguments    : const double C[9]
//                double rotateOrder
//                const double rotateDirection[3]
//                double euler[3]
// Return Type  : void
//
static void OneCToEuler_c(const double C[9], double rotateOrder, const double
  rotateDirection[3], double euler[3])
{
  double euler1;
  double euler2;
  double euler3;
  int i;
  euler1 = 0.0;
  euler2 = 0.0;
  euler3 = 0.0;
  switch ((int)rotateOrder) {
   case 321:
    euler1 = rt_atan2d_snf(C[3], C[0]);
    euler2 = asin(-C[6]);

    //  euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ;
    euler3 = rt_atan2d_snf(C[7], C[8]);
    break;

   case 312:
    euler1 = rt_atan2d_snf(-C[1], C[4]);
    euler2 = asin(C[7]);

    //  euler2 = atan2( C(2,3),sqrt( C(2,1)^2+C(2,2)^2 ) ) ;
    euler3 = rt_atan2d_snf(-C[6], C[8]);
    break;

   case 123:
    euler1 = rt_atan2d_snf(-C[5], C[8]);
    euler2 = asin(C[2]);
    euler3 = rt_atan2d_snf(-C[1], C[0]);
    break;

   case 132:
    euler1 = rt_atan2d_snf(C[7], C[4]);
    euler2 = asin(-C[1]);
    euler3 = rt_atan2d_snf(C[2], C[0]);
    break;

   case 213:
    euler1 = rt_atan2d_snf(C[2], C[8]);
    euler2 = asin(-C[5]);

    //  euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ;
    euler3 = rt_atan2d_snf(C[3], C[4]);
    break;

   case 231:
    euler1 = rt_atan2d_snf(-C[6], C[0]);
    euler2 = asin(C[3]);
    euler3 = rt_atan2d_snf(-C[5], C[4]);
    break;
  }

  euler[0] = euler1;
  euler[1] = euler2;
  euler[2] = euler3;
  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }
}

//
// Arguments    : double euler[6]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void OneEulerToC_c(double euler[6], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  int i;
  double dv15[9];
  double dv16[9];
  double dv17[9];
  double dv18[9];
  int i3;
  int i4;
  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  memset(&C[0], 0, 9U * sizeof(double));
  switch ((int)eml_switch_helper(rotateOrder, 321.0, 312.0, 123.0, 132.0, 213.0))
  {
   case 0:
    RotateX(euler[2], dv15);
    RotateY(euler[1], dv16);
    RotateZ(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;

   case 1:
    RotateY(euler[2], dv15);
    RotateX(euler[1], dv16);
    RotateZ(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;

   case 2:
    RotateZ(euler[2], dv15);
    RotateY(euler[1], dv16);
    RotateX(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;

   case 3:
    RotateY(euler[2], dv15);
    RotateZ(euler[1], dv16);
    RotateX(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;

   case 4:
    RotateZ(euler[2], dv15);
    RotateX(euler[1], dv16);
    RotateY(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;

   case 5:
    RotateX(euler[2], dv15);
    RotateZ(euler[1], dv16);
    RotateY(euler[0], dv17);
    for (i = 0; i < 3; i++) {
      for (i3 = 0; i3 < 3; i3++) {
        dv18[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          dv18[i + 3 * i3] += dv15[i + 3 * i4] * dv16[i4 + 3 * i3];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        C[i + 3 * i3] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          C[i + 3 * i3] += dv18[i + 3 * i4] * dv17[i4 + 3 * i3];
        }
      }
    }
    break;
  }
}

//
// Arguments    : double angX
//                double Cx[9]
// Return Type  : void
//
static void RotateX(double angX, double Cx[9])
{
  int i5;
  static const signed char iv0[3] = { 1, 0, 0 };

  // % anticlockwise = positive
  for (i5 = 0; i5 < 3; i5++) {
    Cx[3 * i5] = iv0[i5];
  }

  Cx[1] = 0.0;
  Cx[4] = cos(angX);
  Cx[7] = sin(angX);
  Cx[2] = 0.0;
  Cx[5] = -sin(angX);
  Cx[8] = cos(angX);
}

//
// Arguments    : double angY
//                double Cy[9]
// Return Type  : void
//
static void RotateY(double angY, double Cy[9])
{
  int i7;
  static const signed char iv2[3] = { 0, 1, 0 };

  // % anticlockwise rotate is positive
  Cy[0] = cos(angY);
  Cy[3] = 0.0;
  Cy[6] = -sin(angY);
  for (i7 = 0; i7 < 3; i7++) {
    Cy[1 + 3 * i7] = iv2[i7];
  }

  Cy[2] = sin(angY);
  Cy[5] = 0.0;
  Cy[8] = cos(angY);
}

//
// Arguments    : double angZ
//                double Cz[9]
// Return Type  : void
//
static void RotateZ(double angZ, double Cz[9])
{
  int i6;
  static const signed char iv1[3] = { 0, 0, 1 };

  // % anticlockwise = positive
  Cz[0] = cos(angZ);
  Cz[3] = sin(angZ);
  Cz[6] = 0.0;
  Cz[1] = -sin(angZ);
  Cz[4] = cos(angZ);
  Cz[7] = 0.0;
  for (i6 = 0; i6 < 3; i6++) {
    Cz[2 + 3 * i6] = iv1[i6];
  }
}

//
// Arguments    : const double C_Spine_Head[9]
//                const double C_Spine_NeckNew[9]
//                double rotateOrder_BVH
//                double *HeadYaw
//                double *HeadPitch
//                double Head_new[3]
// Return Type  : void
//
static void TransformHead(const double C_Spine_Head[9], const double
  C_Spine_NeckNew[9], double rotateOrder_BVH, double *HeadYaw, double *HeadPitch,
  double Head_new[3])
{
  double C_NeckNew_Spine[9];
  double b_C_NeckNew_Spine[9];
  double dv29[3];
  int i16;
  double dv30[9];
  double E_Spine_Head[3];

  // % TransformHead
  //  E_spine_head: euler from spine3 to head
  //  rotateOrder : rotate order of "E_spine_head"
  InverseC_Multi(C_Spine_NeckNew, C_NeckNew_Spine);
  memcpy(&b_C_NeckNew_Spine[0], &C_NeckNew_Spine[0], 9U * sizeof(double));
  for (i16 = 0; i16 < 3; i16++) {
    dv29[i16] = 1.0;
  }

  CalculateC_Multi(b_C_NeckNew_Spine, C_Spine_Head, dv30);
  CToEuler_c(dv30, 213.0, dv29, E_Spine_Head);
  E_Spine_Head[2] = 0.0;
  *HeadYaw = E_Spine_Head[0];
  *HeadPitch = E_Spine_Head[1];

  // %% get new BVH
  for (i16 = 0; i16 < 3; i16++) {
    dv29[i16] = 1.0;
  }

  b_EulerToC_c(E_Spine_Head, 213.0, dv29, C_NeckNew_Spine);
  for (i16 = 0; i16 < 3; i16++) {
    dv29[i16] = 1.0;
  }

  CToEuler_c(C_NeckNew_Spine, rotateOrder_BVH, dv29, Head_new);
}

//
// Arguments    : const double C_Spine_LForeArm[9]
//                const double C_Spine_LArmNew[9]
//                double rotateOrder_BVH
//                double LElbow_Nao[3]
//                double LeftForeArm_new[3]
// Return Type  : void
//
static void TransformLElbow(const double C_Spine_LForeArm[9], const double
  C_Spine_LArmNew[9], double rotateOrder_BVH, double LElbow_Nao[3], double
  LeftForeArm_new[3])
{
  double C_LArmNew_Spine[9];
  double b_C_LArmNew_Spine[9];
  double dv45[3];
  double dv46[3];
  int i21;

  //  %% LElbow
  InverseC_Multi(C_Spine_LArmNew, C_LArmNew_Spine);
  memcpy(&b_C_LArmNew_Spine[0], &C_LArmNew_Spine[0], 9U * sizeof(double));
  CalculateC_Multi(b_C_LArmNew_Spine, C_Spine_LForeArm, C_LArmNew_Spine);

  //      %%% LElbowRoll > 0  % 人很少做的动作：人左手不会这么转
  //      LElbowRoll1 = acos(x) ;
  //      LElbowYaw1 = atan2( y,-z );
  //      LElbow_Nao1 = [ LElbowYaw1 LElbowRoll1 0 ];
  // %% LElbowRoll < 0  % 人正常动作
  LElbow_Nao[0] = rt_atan2d_snf(-C_LArmNew_Spine[3], C_LArmNew_Spine[6]);
  LElbow_Nao[1] = -acos(C_LArmNew_Spine[0]);
  LElbow_Nao[2] = 0.0;

  // %% get new BVH
  for (i21 = 0; i21 < 3; i21++) {
    dv45[i21] = 1.0;
    dv46[i21] = 1.0;
  }

  ChangeEulerRotate_c(LElbow_Nao, 123.0, rotateOrder_BVH, dv45, dv46,
                      LeftForeArm_new);
}

//
// Arguments    : const double C_LLeg_LFoot[9]
//                double rotateOrder_BVH
//                double *LAnklePitch
//                double *LAnkleRoll
//                double LeftFoot_new[3]
// Return Type  : void
//
static void TransformLFoot(const double C_LLeg_LFoot[9], double rotateOrder_BVH,
  double *LAnklePitch, double *LAnkleRoll, double LeftFoot_new[3])
{
  double dv60[3];
  int i26;
  double E_LLeg_LFoot[3];
  double C_RLeg_RFoot_New[9];

  // % LFoot
  for (i26 = 0; i26 < 3; i26++) {
    dv60[i26] = 1.0;
  }

  CToEuler_c(C_LLeg_LFoot, 132.0, dv60, E_LLeg_LFoot);
  *LAnklePitch = E_LLeg_LFoot[0];
  *LAnkleRoll = E_LLeg_LFoot[1];

  // %% get new BVH
  E_LLeg_LFoot[2] = 0.0;
  for (i26 = 0; i26 < 3; i26++) {
    dv60[i26] = 1.0;
  }

  b_EulerToC_c(E_LLeg_LFoot, 132.0, dv60, C_RLeg_RFoot_New);
  for (i26 = 0; i26 < 3; i26++) {
    dv60[i26] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, rotateOrder_BVH, dv60, LeftFoot_new);
}

//
// Arguments    : const double C_Spine_LHand[9]
//                const double C_Spine_LForeArmNew[9]
//                double rotateOrder_BVH
//                double *LWristYaw
//                double LeftHand_new[3]
// Return Type  : void
//
static void TransformLHand(const double C_Spine_LHand[9], const double
  C_Spine_LForeArmNew[9], double rotateOrder_BVH, double *LWristYaw, double
  LeftHand_new[3])
{
  double C_LForeArmNew_Spine[9];
  double b_C_LForeArmNew_Spine[9];
  double dv47[3];
  int i22;
  double dv48[9];
  double E_LForeArmNew_LHand[3];
  double LHand_Nao[3];

  // % LHand
  InverseC_Multi(C_Spine_LForeArmNew, C_LForeArmNew_Spine);
  memcpy(&b_C_LForeArmNew_Spine[0], &C_LForeArmNew_Spine[0], 9U * sizeof(double));
  for (i22 = 0; i22 < 3; i22++) {
    dv47[i22] = 1.0;
  }

  CalculateC_Multi(b_C_LForeArmNew_Spine, C_Spine_LHand, dv48);
  CToEuler_c(dv48, 123.0, dv47, E_LForeArmNew_LHand);
  *LWristYaw = E_LForeArmNew_LHand[0];

  // %% get new BVH
  for (i22 = 0; i22 < 3; i22++) {
    LHand_Nao[i22] = 0.0;
  }

  LHand_Nao[0] = E_LForeArmNew_LHand[0];
  for (i22 = 0; i22 < 3; i22++) {
    dv47[i22] = 1.0;
    E_LForeArmNew_LHand[i22] = 1.0;
  }

  ChangeEulerRotate_c(LHand_Nao, 123.0, rotateOrder_BVH, dv47,
                      E_LForeArmNew_LHand, LeftHand_new);
}

//
// Arguments    : const double C_Hip_LUpLegNew[9]
//                const double C_Hip_LLeg[9]
//                double rotateOrder_BVH
//                double *LKneePitch
//                double LeftLeg_New[3]
// Return Type  : void
//
static void TransformLLeg(const double C_Hip_LUpLegNew[9], const double
  C_Hip_LLeg[9], double rotateOrder_BVH, double *LKneePitch, double LeftLeg_New
  [3])
{
  double C_LUpLegNew_Hip[9];
  double b_C_LUpLegNew_Hip[9];
  double dv58[3];
  int i25;
  double dv59[9];
  double E_LUpLegNew_LLeg[3];

  // % LLeg
  InverseC_Multi(C_Hip_LUpLegNew, C_LUpLegNew_Hip);
  memcpy(&b_C_LUpLegNew_Hip[0], &C_LUpLegNew_Hip[0], 9U * sizeof(double));
  for (i25 = 0; i25 < 3; i25++) {
    dv58[i25] = 1.0;
  }

  CalculateC_Multi(b_C_LUpLegNew_Hip, C_Hip_LLeg, dv59);
  CToEuler_c(dv59, 123.0, dv58, E_LUpLegNew_LLeg);
  *LKneePitch = E_LUpLegNew_LLeg[0];

  //  figure('name','LeftLeg_Nao')
  //  subplot(3,1,1)
  //  plot(E_LUpLegNew_LLeg(:,1)*180/pi,'r'  )
  //  ylabel('x')
  //  subplot(3,1,2)
  //  plot(E_LUpLegNew_LLeg(:,2)*180/pi,'b'  )
  //  ylabel('y')
  //  subplot(3,1,3)
  //  plot(E_LUpLegNew_LLeg(:,3)*180/pi,'k'  )
  //  ylabel('z')
  // %% get new BVH
  for (i25 = 0; i25 < 2; i25++) {
    E_LUpLegNew_LLeg[1 + i25] = 0.0;
  }

  for (i25 = 0; i25 < 3; i25++) {
    dv58[i25] = 1.0;
  }

  b_EulerToC_c(E_LUpLegNew_LLeg, 123.0, dv58, C_LUpLegNew_Hip);
  for (i25 = 0; i25 < 3; i25++) {
    dv58[i25] = 1.0;
  }

  CToEuler_c(C_LUpLegNew_Hip, rotateOrder_BVH, dv58, LeftLeg_New);
}

//
// Arguments    : const double C_Spine3_LShoulderNew[9]
//                const double C_Spine3_LArm[9]
//                double rotateOrder_BVH
//                double *LShoulderPitch
//                double *LShoulderRoll
//                double LeftArm_new[3]
// Return Type  : void
//
static void TransformLShoulder(const double C_Spine3_LShoulderNew[9], const
  double C_Spine3_LArm[9], double rotateOrder_BVH, double *LShoulderPitch,
  double *LShoulderRoll, double LeftArm_new[3])
{
  double C_LShoulderNew_Spine3[9];
  double b_C_LShoulderNew_Spine3[9];
  double c_C_LShoulderNew_Spine3[9];
  double dv40[9];
  static const double dv41[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  double E_LShoulderNew_LArm_Nao[3];
  static const double dv42[3] = { -1.0, 1.0, 1.0 };

  double d_C_LShoulderNew_Spine3[9];
  int i20;
  double dv43[9];
  static const double dv44[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  // % LeftShoulder
  InverseC_Multi(C_Spine3_LShoulderNew, C_LShoulderNew_Spine3);
  memcpy(&b_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(b_C_LShoulderNew_Spine3, C_Spine3_LArm, C_LShoulderNew_Spine3);

  // %% 对 C_LShoulderNew_LArm 进行零位补偿
  memcpy(&c_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(dv41, c_C_LShoulderNew_Spine3, dv40);
  CToEuler_c(dv40, 321.0, dv42, E_LShoulderNew_LArm_Nao);
  E_LShoulderNew_LArm_Nao[2] = 0.0;
  b_EulerToC_c(E_LShoulderNew_LArm_Nao, 321.0, dv42, C_LShoulderNew_Spine3);
  *LShoulderPitch = E_LShoulderNew_LArm_Nao[0];
  *LShoulderRoll = E_LShoulderNew_LArm_Nao[1];

  // %% get new BVH
  //  零位补偿回去
  //  'ZYX'
  memcpy(&d_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i20 = 0; i20 < 3; i20++) {
    E_LShoulderNew_LArm_Nao[i20] = 1.0;
  }

  CalculateC_Multi(dv44, d_C_LShoulderNew_Spine3, dv43);
  CToEuler_c(dv43, rotateOrder_BVH, E_LShoulderNew_LArm_Nao, LeftArm_new);
}

//
// Arguments    : const double C_Hip_LUpLeg[9]
//                double HipYawPitch
//                double rotateOrder_BVH
//                double *LHipRoll
//                double *LHipPitch
//                double LeftUpLeg_new[3]
// Return Type  : void
//
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double HipYawPitch,
  double rotateOrder_BVH, double *LHipRoll, double *LHipPitch, double
  LeftUpLeg_new[3])
{
  double C_LUpLeg_Hip[9];
  double x;
  double dv55[4];
  int k;
  static const double a[3] = { 0.70710678118654746, -0.70710678118654746, 0.0 };

  double C_LUpLeg_Hip_New_k[9];
  double b_C_LUpLeg_Hip[9];
  double C_HipYawPitch[9];
  double dv56[3];
  double dv57[9];

  // % LUpLeg
  //  HipYawPitch：左右大腿的 HipYawPitch Angle  [ Nframes*1 ]
  InverseC_Multi(C_Hip_LUpLeg, C_LUpLeg_Hip);
  x = sin(HipYawPitch / 2.0);
  dv55[0] = cos(HipYawPitch / 2.0);
  for (k = 0; k < 3; k++) {
    dv55[k + 1] = a[k] * x;
  }

  FQtoCnb(dv55, C_LUpLeg_Hip_New_k);
  for (k = 0; k < 9; k++) {
    C_HipYawPitch[k] = C_LUpLeg_Hip_New_k[k];
    b_C_LUpLeg_Hip[k] = C_LUpLeg_Hip[k];
  }

  CalculateC_Multi(b_C_LUpLeg_Hip, C_HipYawPitch, C_LUpLeg_Hip);
  for (k = 0; k < 9; k++) {
    C_LUpLeg_Hip_New_k[k] = C_LUpLeg_Hip[k];
    b_C_LUpLeg_Hip[k] = C_LUpLeg_Hip[k];
  }

  *LHipPitch = asin(C_LUpLeg_Hip_New_k[5]);
  *LHipRoll = atan(-C_LUpLeg_Hip_New_k[3] / C_LUpLeg_Hip_New_k[4]);

  // %% get new BVH
  for (k = 0; k < 3; k++) {
    dv56[k] = 1.0;
  }

  InverseC_Multi(b_C_LUpLeg_Hip, dv57);
  CToEuler_c(dv57, rotateOrder_BVH, dv56, LeftUpLeg_new);
}

//
// Arguments    : const double C_Spine_RForeArm[9]
//                const double C_Spine_RArmNew[9]
//                double rotateOrder_BVH
//                double RElbow_Nao[3]
//                double RightForeArm_new[3]
// Return Type  : void
//
static void TransformRElbow(const double C_Spine_RForeArm[9], const double
  C_Spine_RArmNew[9], double rotateOrder_BVH, double RElbow_Nao[3], double
  RightForeArm_new[3])
{
  double C_RArmNew_Spine[9];
  double b_C_RArmNew_Spine[9];
  double dv36[3];
  int i18;
  static const double dv37[3] = { -1.0, 1.0, 1.0 };

  // % REblow
  InverseC_Multi(C_Spine_RArmNew, C_RArmNew_Spine);
  memcpy(&b_C_RArmNew_Spine[0], &C_RArmNew_Spine[0], 9U * sizeof(double));
  CalculateC_Multi(b_C_RArmNew_Spine, C_Spine_RForeArm, C_RArmNew_Spine);

  // %% RElbowRoll > 0  % 人正常动作
  RElbow_Nao[0] = rt_atan2d_snf(-C_RArmNew_Spine[3], -C_RArmNew_Spine[6]);
  RElbow_Nao[1] = acos(C_RArmNew_Spine[0]);
  RElbow_Nao[2] = 0.0;

  //      %%% RElbowRoll < 0  % 人很少做的动作：人右手不会这么转
  //      RElbowRoll2 = -acos(x) ;
  //      RElbowYaw2 = atan2( y,z );
  //      RElbow_Nao2 = [ RElbowYaw2 RElbowRoll2 0 ];
  // %% get new BVH
  for (i18 = 0; i18 < 3; i18++) {
    dv36[i18] = 1.0;
  }

  ChangeEulerRotate_c(RElbow_Nao, 123.0, rotateOrder_BVH, dv37, dv36,
                      RightForeArm_new);
}

//
// Arguments    : const double C_RLeg_RFoot[9]
//                double rotateOrder_BVH
//                double *RAnklePitch
//                double *RAnkleRoll
//                double RightFoot_new[3]
// Return Type  : void
//
static void TransformRFoot(const double C_RLeg_RFoot[9], double rotateOrder_BVH,
  double *RAnklePitch, double *RAnkleRoll, double RightFoot_new[3])
{
  double dv54[3];
  int i24;
  double E_RLeg_RFoot[3];
  double C_RLeg_RFoot_New[9];

  // % RFoot
  for (i24 = 0; i24 < 3; i24++) {
    dv54[i24] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot, 132.0, dv54, E_RLeg_RFoot);
  *RAnklePitch = E_RLeg_RFoot[0];
  *RAnkleRoll = E_RLeg_RFoot[1];

  // %% get new BVH
  E_RLeg_RFoot[2] = 0.0;
  for (i24 = 0; i24 < 3; i24++) {
    dv54[i24] = 1.0;
  }

  b_EulerToC_c(E_RLeg_RFoot, 132.0, dv54, C_RLeg_RFoot_New);
  for (i24 = 0; i24 < 3; i24++) {
    dv54[i24] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, rotateOrder_BVH, dv54, RightFoot_new);
}

//
// Arguments    : const double C_Spine_RHand[9]
//                const double C_Spine_RForeArmNew[9]
//                double rotateOrder_BVH
//                double *RWristYaw
//                double RightHand_new[3]
// Return Type  : void
//
static void TransformRHand(const double C_Spine_RHand[9], const double
  C_Spine_RForeArmNew[9], double rotateOrder_BVH, double *RWristYaw, double
  RightHand_new[3])
{
  double C_RForeArmNew_Spine[9];
  double b_C_RForeArmNew_Spine[9];
  double dv38[9];
  double E_RForeArmNew_RHand[3];
  static const double dv39[3] = { -1.0, 1.0, 1.0 };

  double RHand_Nao[3];
  int i19;

  // % RHand
  InverseC_Multi(C_Spine_RForeArmNew, C_RForeArmNew_Spine);
  memcpy(&b_C_RForeArmNew_Spine[0], &C_RForeArmNew_Spine[0], 9U * sizeof(double));
  CalculateC_Multi(b_C_RForeArmNew_Spine, C_Spine_RHand, dv38);
  CToEuler_c(dv38, 123.0, dv39, E_RForeArmNew_RHand);
  *RWristYaw = E_RForeArmNew_RHand[0];

  // %% get new BVH
  for (i19 = 0; i19 < 3; i19++) {
    RHand_Nao[i19] = 0.0;
  }

  RHand_Nao[0] = E_RForeArmNew_RHand[0];
  for (i19 = 0; i19 < 3; i19++) {
    E_RForeArmNew_RHand[i19] = 1.0;
  }

  ChangeEulerRotate_c(RHand_Nao, 123.0, rotateOrder_BVH, dv39,
                      E_RForeArmNew_RHand, RightHand_new);
}

//
// Arguments    : const double C_Hip_RUpLegNew[9]
//                const double C_Hip_RLeg[9]
//                double rotateOrder_BVH
//                double *RKneePitch
//                double RightLeg_New[3]
// Return Type  : void
//
static void TransformRLeg(const double C_Hip_RUpLegNew[9], const double
  C_Hip_RLeg[9], double rotateOrder_BVH, double *RKneePitch, double
  RightLeg_New[3])
{
  double C_RUpLegNew_Hip[9];
  double b_C_RUpLegNew_Hip[9];
  double dv52[3];
  int i23;
  double dv53[9];
  double E_LUpLegNew_LLeg[3];

  // % RLeg
  InverseC_Multi(C_Hip_RUpLegNew, C_RUpLegNew_Hip);
  memcpy(&b_C_RUpLegNew_Hip[0], &C_RUpLegNew_Hip[0], 9U * sizeof(double));
  for (i23 = 0; i23 < 3; i23++) {
    dv52[i23] = 1.0;
  }

  CalculateC_Multi(b_C_RUpLegNew_Hip, C_Hip_RLeg, dv53);
  CToEuler_c(dv53, 123.0, dv52, E_LUpLegNew_LLeg);
  *RKneePitch = E_LUpLegNew_LLeg[0];

  //  E_UpLeg_Leg = CToEuler_c( C_RUpLeg_RLeg,123 );
  //  RKneePitch = E_UpLeg_Leg(:,1)';
  // %% get new BVH
  for (i23 = 0; i23 < 2; i23++) {
    E_LUpLegNew_LLeg[1 + i23] = 0.0;
  }

  for (i23 = 0; i23 < 3; i23++) {
    dv52[i23] = 1.0;
  }

  b_EulerToC_c(E_LUpLegNew_LLeg, 123.0, dv52, C_RUpLegNew_Hip);
  for (i23 = 0; i23 < 3; i23++) {
    dv52[i23] = 1.0;
  }

  CToEuler_c(C_RUpLegNew_Hip, rotateOrder_BVH, dv52, RightLeg_New);
}

//
// Arguments    : const double C_Spine3_RShoulderNew[9]
//                const double C_Spine3_RArm[9]
//                double rotateOrder_BVH
//                double *RShoulderPitch
//                double *RShoulderRoll
//                double RightArm_new[3]
// Return Type  : void
//
static void TransformRShoulder(const double C_Spine3_RShoulderNew[9], const
  double C_Spine3_RArm[9], double rotateOrder_BVH, double *RShoulderPitch,
  double *RShoulderRoll, double RightArm_new[3])
{
  double C_RShoulderNew_Spine3[9];
  double b_C_RShoulderNew_Spine3[9];
  double c_C_RShoulderNew_Spine3[9];
  double dv31[3];
  int i17;
  double dv32[9];
  static const double dv33[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  double E_RShoulderNew_RArm_Nao[3];
  double d_C_RShoulderNew_Spine3[9];
  double dv34[9];
  static const double dv35[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  // % RightShoulder
  InverseC_Multi(C_Spine3_RShoulderNew, C_RShoulderNew_Spine3);
  memcpy(&b_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(b_C_RShoulderNew_Spine3, C_Spine3_RArm, C_RShoulderNew_Spine3);

  // %% 对 C_RShoulderNew_RArm 进行零位补偿
  memcpy(&c_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i17 = 0; i17 < 3; i17++) {
    dv31[i17] = 1.0;
  }

  CalculateC_Multi(dv33, c_C_RShoulderNew_Spine3, dv32);
  CToEuler_c(dv32, 321.0, dv31, E_RShoulderNew_RArm_Nao);
  E_RShoulderNew_RArm_Nao[2] = 0.0;
  for (i17 = 0; i17 < 3; i17++) {
    dv31[i17] = 1.0;
  }

  b_EulerToC_c(E_RShoulderNew_RArm_Nao, 321.0, dv31, C_RShoulderNew_Spine3);
  *RShoulderPitch = E_RShoulderNew_RArm_Nao[0];
  *RShoulderRoll = E_RShoulderNew_RArm_Nao[1];

  // %% get new BVH
  //  零位补偿回去
  //  321
  memcpy(&d_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i17 = 0; i17 < 3; i17++) {
    dv31[i17] = 1.0;
  }

  CalculateC_Multi(dv35, d_C_RShoulderNew_Spine3, dv34);
  CToEuler_c(dv34, rotateOrder_BVH, dv31, RightArm_new);
}

//
// Arguments    : const double C_Hip_RUpLeg[9]
//                double HipYawPitch
//                double rotateOrder_BVH
//                double *RHipRoll
//                double *RHipPitch
//                double RightUpLeg_new[3]
// Return Type  : void
//
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double HipYawPitch,
  double rotateOrder_BVH, double *RHipRoll, double *RHipPitch, double
  RightUpLeg_new[3])
{
  double C_RUpLeg_Hip[9];
  double x;
  double dv49[4];
  int k;
  static const double a[3] = { 0.70710678118654746, 0.70710678118654746, 0.0 };

  double C_RUpLeg_Hip_New_k[9];
  double b_C_RUpLeg_Hip[9];
  double C_HipYawPitch[9];
  double dv50[3];
  double dv51[9];

  // % RUpLeg
  //  HipYawPitch：左右大腿的 HipYawPitch Angle  [ 1*Nframes ]
  InverseC_Multi(C_Hip_RUpLeg, C_RUpLeg_Hip);
  x = sin(HipYawPitch / 2.0);
  dv49[0] = cos(HipYawPitch / 2.0);
  for (k = 0; k < 3; k++) {
    dv49[k + 1] = a[k] * x;
  }

  FQtoCnb(dv49, C_RUpLeg_Hip_New_k);
  for (k = 0; k < 9; k++) {
    C_HipYawPitch[k] = C_RUpLeg_Hip_New_k[k];
    b_C_RUpLeg_Hip[k] = C_RUpLeg_Hip[k];
  }

  CalculateC_Multi(b_C_RUpLeg_Hip, C_HipYawPitch, C_RUpLeg_Hip);
  for (k = 0; k < 9; k++) {
    C_RUpLeg_Hip_New_k[k] = C_RUpLeg_Hip[k];
    b_C_RUpLeg_Hip[k] = C_RUpLeg_Hip[k];
  }

  *RHipPitch = asin(C_RUpLeg_Hip_New_k[5]);
  *RHipRoll = atan(-C_RUpLeg_Hip_New_k[3] / C_RUpLeg_Hip_New_k[4]);

  // %% get new BVH
  for (k = 0; k < 3; k++) {
    dv50[k] = 1.0;
  }

  InverseC_Multi(b_C_RUpLeg_Hip, dv51);
  CToEuler_c(dv51, rotateOrder_BVH, dv50, RightUpLeg_new);
}

//
// Arguments    : const double euler[3]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void b_EulerToC_c(const double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  double b_euler[3];
  int i8;
  double dv19[9];

  // % xyz 2015.4.5
  //   EulerToC_Multi  新版 ： 可转化为 C++
  //    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]
  //  store order of euler angle: the rotate order
  //  positive:anticlockwise rotate
  //  euler angle scope: [-pi,pi]
  //  unit:rad
  //  rotateDirection: anticlockwise(1) or clockwise(-1)  111
  for (i8 = 0; i8 < 3; i8++) {
    b_euler[i8] = euler[i8];
  }

  b_OneEulerToC_c(b_euler, rotateOrder, rotateDirection, dv19);
  memcpy(&C[0], &dv19[0], 9U * sizeof(double));
}

//
// Arguments    : double euler[3]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void b_OneEulerToC_c(double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  int i;
  double dv20[9];
  double dv21[9];
  double dv22[9];
  double dv23[9];
  int i9;
  int i10;
  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  memset(&C[0], 0, 9U * sizeof(double));
  switch ((int)eml_switch_helper(rotateOrder, 321.0, 312.0, 123.0, 132.0, 213.0))
  {
   case 0:
    RotateX(euler[2], dv20);
    RotateY(euler[1], dv21);
    RotateZ(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;

   case 1:
    RotateY(euler[2], dv20);
    RotateX(euler[1], dv21);
    RotateZ(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;

   case 2:
    RotateZ(euler[2], dv20);
    RotateY(euler[1], dv21);
    RotateX(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;

   case 3:
    RotateY(euler[2], dv20);
    RotateZ(euler[1], dv21);
    RotateX(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;

   case 4:
    RotateZ(euler[2], dv20);
    RotateX(euler[1], dv21);
    RotateY(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;

   case 5:
    RotateX(euler[2], dv20);
    RotateZ(euler[1], dv21);
    RotateY(euler[0], dv22);
    for (i = 0; i < 3; i++) {
      for (i9 = 0; i9 < 3; i9++) {
        dv23[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          dv23[i + 3 * i9] += dv20[i + 3 * i10] * dv21[i10 + 3 * i9];
        }
      }

      for (i9 = 0; i9 < 3; i9++) {
        C[i + 3 * i9] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          C[i + 3 * i9] += dv23[i + 3 * i10] * dv22[i10 + 3 * i9];
        }
      }
    }
    break;
  }
}

//
// Arguments    : double expr
//                double varargin_1
//                double varargin_2
//                double varargin_3
//                double varargin_4
//                double varargin_5
// Return Type  : double
//
static double eml_switch_helper(double expr, double varargin_1, double
  varargin_2, double varargin_3, double varargin_4, double varargin_5)
{
  double b_index;
  if (varargin_1 == expr) {
    b_index = 0.0;
  } else if (varargin_2 == expr) {
    b_index = 1.0;
  } else if (varargin_3 == expr) {
    b_index = 2.0;
  } else if (varargin_4 == expr) {
    b_index = 3.0;
  } else if (varargin_5 == expr) {
    b_index = 4.0;
  } else {
    b_index = -1.0;
  }

  return b_index;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(double)(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : const double BVHData[354]
//                double NaoData[25]
// Return Type  : void
//
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
  double C_LShoulder_LArm[9];
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
  double b_C_LShoulder_LArm[9];
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
  static const double C_I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  double Head_new[3];
  double HeadPitch;
  double HeadYaw;
  double RShoulderRoll;
  double RShoulderPitch;
  double dv3[3];
  double c_C_Spine2_Head[9];
  double RElbow_Nao[3];
  double c_C_World_Spine2[9];
  double dv4[9];
  double RWristYaw;
  double dv5[9];
  double LShoulderRoll;
  double LShoulderPitch;
  double dv6[9];
  double LElbow_Nao[3];
  double d_C_World_Spine2[9];
  double dv7[9];
  double dv8[9];
  double LWristYaw;
  double dv9[9];
  double RHipPitch;
  double RHipRoll;
  double e_C_World_Spine2[9];
  double dv10[9];
  double RKneePitch;
  double RAnkleRoll;
  double RAnklePitch;
  double dv11[9];
  double LHipPitch;
  double LHipRoll;
  double f_C_World_Spine2[9];
  double dv12[9];
  double LKneePitch;
  double LAnkleRoll;
  double LAnklePitch;
  double b_HeadYaw[25];

  // % xyz  2015.3.25
  //  Transform BVH data Format
  //  BVHData: 1*354  degree
  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = BVHData[i0] * 3.1415926535897931 / 180.0;
  }

  BVHDataToRotate_c(b_BVHData, C_World_Hip, C_Spine3New_RArmNew, C_Spine3_Head,
                    C_Spine2_Head, C_Spine2_Spine3, C_Spine3_Neck, C_Neck_Head,
                    C_World_Spine2, C_LShoulder_LArm, C_LArm_LForeArm,
                    C_LForeArm_LHand, C_Spine3_RShoulder, C_RShoulder_RArm,
                    C_RArm_RForeArm, C_RForeArm_RHand, C_Hip_RUpLeg,
                    C_RUpLeg_RLeg, C_RLeg_RFoot, C_Hip_LUpLeg, C_LUpLeg_LLeg,
                    C_LLeg_LFoot);

  // % 所有BVH的旋转数据
  for (i0 = 0; i0 < 9; i0++) {
    C_World_Hip[i0] = C_Spine3New_RArmNew[i0];
    b_C_Spine3_Head[i0] = C_Spine3_Head[i0];
    b_C_Spine2_Head[i0] = C_Spine2_Head[i0];
    b_C_World_Spine2[i0] = C_World_Spine2[i0];
    b_C_LShoulder_LArm[i0] = C_LShoulder_LArm[i0];
    b_C_LArm_LForeArm[i0] = C_LArm_LForeArm[i0];
    b_C_LForeArm_LHand[i0] = C_LForeArm_LHand[i0];
    b_C_Spine3_RShoulder[i0] = C_Spine3_RShoulder[i0];
    b_C_RShoulder_RArm[i0] = C_RShoulder_RArm[i0];
    b_C_RArm_RForeArm[i0] = C_RArm_RForeArm[i0];
  }

  CalculatedLeftArmRotate(C_World_Hip, b_C_Spine3_Head, b_C_Spine2_Head,
    C_Spine2_Spine3, b_C_World_Spine2, b_C_LShoulder_LArm, b_C_LArm_LForeArm,
    b_C_LForeArm_LHand, C_Hip_Spine2, C_Hip_Spine3, C_Spine_Spine3,
    C_Spine3New_RArmNew, C_Spine3_Head, C_Spine2_Head, C_World_Spine2,
    C_LShoulder_LArm, C_LArm_LForeArm, C_Spine2_LArm, C_Spine2_LForeArm,
    C_Spine2_LHand, C_LForeArm_LHand, C_Spine3_LForeArm, C_Spine3_LHand);
  CalculatedRightArmRotate(C_Hip_Spine3, C_Spine2_Spine3, C_Spine_Spine3,
    b_C_Spine3_RShoulder, b_C_RShoulder_RArm, b_C_RArm_RForeArm,
    C_RForeArm_RHand, C_Spine3New_RArmNew, C_Spine3_Head, C_Spine2_Head,
    C_World_Spine2, C_LShoulder_LArm, C_LArm_LForeArm, C_Spine3_RShoulder,
    C_RShoulder_RArm, C_RArm_RForeArm, C_LForeArm_LHand, C_Spine3_LForeArm,
    C_Spine3_LHand);
  CalculatedLLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_LUpLeg, C_LUpLeg_LLeg,
                       C_LArm_LForeArm, C_Spine3New_RArmNew, C_Spine3_Head);
  CalculatedRLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_RUpLeg, C_RUpLeg_RLeg,
                       C_LShoulder_LArm, C_Spine3New_RArmNew, C_Spine3_Head);

  // % common parameters to set
  //  reference joint : 'Hip'  'SPine'  'Spine3'
  // %% New Hip
  // % head
  CalculateC_Multi(C_Spine3_Neck, C_Neck_Head, dv0);
  CalculateC_Multi(C_Spine2_Spine3, dv0, dv1);
  CalculateC_Multi(C_I, C_I, dv2);
  TransformHead(dv1, dv2, 213.0, &HeadYaw, &HeadPitch, Head_new);

  // % Right Shoulder
  CalculateC_Multi(C_I, C_I, C_Spine3_Head);
  TransformRShoulder(C_Spine3_Head, C_Spine3_RShoulder, 213.0, &RShoulderPitch,
                     &RShoulderRoll, Head_new);

  // % Right Elbow
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_Spine2_Head);
  memcpy(&c_C_Spine2_Head[0], &C_Spine2_Head[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, c_C_Spine2_Head, C_Spine2_Head);
  TransformRElbow(C_RShoulder_RArm, C_Spine2_Head, 213.0, RElbow_Nao, Head_new);

  // % RHand
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_World_Spine2);
  memcpy(&c_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, c_C_World_Spine2, dv4);
  TransformRHand(C_RArm_RForeArm, dv4, 213.0, &RWristYaw, Head_new);

  // % Left Shoulder
  CalculateC_Multi(C_I, C_I, dv5);
  TransformLShoulder(dv5, C_Spine2_LArm, 213.0, &LShoulderPitch, &LShoulderRoll,
                     Head_new);

  // % Left Elbow
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_World_Spine2);
  CalculateC_Multi(C_I, C_World_Spine2, C_Spine2_Head);
  CalculateC_Multi(C_I, C_Spine2_Head, dv6);
  TransformLElbow(C_Spine2_LForeArm, dv6, 213.0, LElbow_Nao, Head_new);

  // % LHand
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_World_Spine2);
  memcpy(&d_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, d_C_World_Spine2, dv7);
  CalculateC_Multi(C_I, dv7, dv8);
  TransformLHand(C_Spine2_LHand, dv8, 213.0, &LWristYaw, Head_new);

  // % RLeg
  InverseC_Multi(C_Hip_Spine2, C_Spine3_Head);
  CalculateC_Multi(C_Spine3_Head, C_Hip_RUpLeg, dv9);
  TransformRUpLeg(dv9, 0.0, 213.0, &RHipRoll, &RHipPitch, Head_new);
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_World_Spine2);
  memcpy(&e_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, e_C_World_Spine2, dv10);
  TransformRLeg(dv10, C_LShoulder_LArm, 213.0, &RKneePitch, Head_new);
  TransformRFoot(C_RLeg_RFoot, 213.0, &RAnklePitch, &RAnkleRoll, Head_new);

  // % LLeg
  CalculateC_Multi(C_Spine3_Head, C_Hip_LUpLeg, dv11);
  TransformLUpLeg(dv11, 0.0, 213.0, &LHipRoll, &LHipPitch, Head_new);
  for (i0 = 0; i0 < 3; i0++) {
    dv3[i0] = 1.0;
  }

  b_EulerToC_c(Head_new, 213.0, dv3, C_World_Spine2);
  memcpy(&f_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, f_C_World_Spine2, dv12);
  TransformLLeg(dv12, C_LArm_LForeArm, 213.0, &LKneePitch, Head_new);
  TransformLFoot(C_LLeg_LFoot, 213.0, &LAnklePitch, &LAnkleRoll, Head_new);

  // % Get Nao
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

//
// Arguments    : void
// Return Type  : void
//
void BVHTransform_c_initialize()
{
  rt_InitInfAndNaN(8U);
}

//
// Arguments    : void
// Return Type  : void
//
void BVHTransform_c_terminate()
{
  // (no terminate code required)
}

//
// File trailer for BVHTransform_c.cpp
//
// [EOF]
//
