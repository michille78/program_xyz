//
// File: BVHTransform_c.cpp
//
// MATLAB Coder version            : 2.6
// C/C++ source code generated on  : 07-Apr-2015 20:00:35
//

// Include files
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"

// Function Declarations
static void BVHDataToRotate_c(const double BVHData[354], double rotateOrder_BVH,
  const double rotateDirection_BVH[3], double C_World_Hip[9], double
  C_Hip_Spine[9], double C_Spine_Spine1[9], double C_Spine1_Spine2[9], double
  C_Spine2_Spine3[9], double C_Spine3_Neck[9], double C_Neck_Head[9], double
  C_Spine3_LShoulder[9], double C_LShoulder_LArm[9], double C_LArm_LForeArm[9],
  double C_LForeArm_LHand[9], double C_Spine3_RShoulder[9], double
  C_RShoulder_RArm[9], double C_RArm_RForeArm[9], double C_RForeArm_RHand[9],
  double C_Hip_RUpLeg[9], double C_RUpLeg_RLeg[9], double C_RLeg_RFoot[9],
  double C_Hip_LUpLeg[9], double C_LUpLeg_LLeg[9], double C_LLeg_LFoot[9]);
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
static void EulerToC_c(const double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9]);
static void InverseC_Multi(const double C12[9], double C21[9]);
static void OneCToEuler_c(const double C[9], double rotateOrder, const double
  rotateDirection[3], double euler[3]);
static void OneEulerToC_c(double euler[3], double rotateOrder, const double
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
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double rotateOrder_BVH,
  double *LHipRoll, double *LHipPitch, double LeftUpLeg_new[3]);
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
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double rotateOrder_BVH,
  double *RHipRoll, double *RHipPitch, double RightUpLeg_new[3]);
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : const double BVHData[354]
//                double rotateOrder_BVH
//                const double rotateDirection_BVH[3]
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
static void BVHDataToRotate_c(const double BVHData[354], double rotateOrder_BVH,
  const double rotateDirection_BVH[3], double C_World_Hip[9], double
  C_Hip_Spine[9], double C_Spine_Spine1[9], double C_Spine1_Spine2[9], double
  C_Spine2_Spine3[9], double C_Spine3_Neck[9], double C_Neck_Head[9], double
  C_Spine3_LShoulder[9], double C_LShoulder_LArm[9], double C_LArm_LForeArm[9],
  double C_LForeArm_LHand[9], double C_Spine3_RShoulder[9], double
  C_RShoulder_RArm[9], double C_RArm_RForeArm[9], double C_RForeArm_RHand[9],
  double C_Hip_RUpLeg[9], double C_RUpLeg_RLeg[9], double C_RLeg_RFoot[9],
  double C_Hip_LUpLeg[9], double C_LUpLeg_LLeg[9], double C_LLeg_LFoot[9])
{
  //  BVHData: Nframes*(59*6)   [ 1*354 ]
  EulerToC_c(*(double (*)[3])&BVHData[3], rotateOrder_BVH, rotateDirection_BVH,
             C_World_Hip);
  EulerToC_c(*(double (*)[3])&BVHData[45], rotateOrder_BVH, rotateDirection_BVH,
             C_Hip_Spine);
  EulerToC_c(*(double (*)[3])&BVHData[51], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine_Spine1);
  EulerToC_c(*(double (*)[3])&BVHData[57], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine1_Spine2);
  EulerToC_c(*(double (*)[3])&BVHData[63], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine2_Spine3);
  EulerToC_c(*(double (*)[3])&BVHData[69], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine3_Neck);
  EulerToC_c(*(double (*)[3])&BVHData[75], rotateOrder_BVH, rotateDirection_BVH,
             C_Neck_Head);
  EulerToC_c(*(double (*)[3])&BVHData[219], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine3_LShoulder);
  EulerToC_c(*(double (*)[3])&BVHData[225], rotateOrder_BVH, rotateDirection_BVH,
             C_LShoulder_LArm);
  EulerToC_c(*(double (*)[3])&BVHData[231], rotateOrder_BVH, rotateDirection_BVH,
             C_LArm_LForeArm);
  EulerToC_c(*(double (*)[3])&BVHData[237], rotateOrder_BVH, rotateDirection_BVH,
             C_LForeArm_LHand);
  EulerToC_c(*(double (*)[3])&BVHData[81], rotateOrder_BVH, rotateDirection_BVH,
             C_Spine3_RShoulder);
  EulerToC_c(*(double (*)[3])&BVHData[87], rotateOrder_BVH, rotateDirection_BVH,
             C_RShoulder_RArm);
  EulerToC_c(*(double (*)[3])&BVHData[93], rotateOrder_BVH, rotateDirection_BVH,
             C_RArm_RForeArm);
  EulerToC_c(*(double (*)[3])&BVHData[99], rotateOrder_BVH, rotateDirection_BVH,
             C_RForeArm_RHand);
  EulerToC_c(*(double (*)[3])&BVHData[9], rotateOrder_BVH, rotateDirection_BVH,
             C_Hip_RUpLeg);
  EulerToC_c(*(double (*)[3])&BVHData[15], rotateOrder_BVH, rotateDirection_BVH,
             C_RUpLeg_RLeg);
  EulerToC_c(*(double (*)[3])&BVHData[21], rotateOrder_BVH, rotateDirection_BVH,
             C_RLeg_RFoot);
  EulerToC_c(*(double (*)[3])&BVHData[27], rotateOrder_BVH, rotateDirection_BVH,
             C_Hip_LUpLeg);
  EulerToC_c(*(double (*)[3])&BVHData[33], rotateOrder_BVH, rotateDirection_BVH,
             C_LUpLeg_LLeg);
  EulerToC_c(*(double (*)[3])&BVHData[39], rotateOrder_BVH, rotateDirection_BVH,
             C_LLeg_LFoot);
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
  double dv22[3];
  int i11;

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
  OneCToEuler_c(C_K, rotateOrder, rotateDirection, dv22);
  for (i11 = 0; i11 < 3; i11++) {
    euler[i11] = dv22[i11];
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
  int i7;
  int i8;
  for (k = 0; k < 9; k++) {
    C23_k[k] = C23[k];
    C12_k[k] = C12[k];
  }

  for (k = 0; k < 3; k++) {
    for (i7 = 0; i7 < 3; i7++) {
      b_C23_k[k + 3 * i7] = 0.0;
      for (i8 = 0; i8 < 3; i8++) {
        b_C23_k[k + 3 * i7] += C23_k[k + 3 * i8] * C12_k[i8 + 3 * i7];
      }
    }
  }

  for (k = 0; k < 3; k++) {
    for (i7 = 0; i7 < 3; i7++) {
      C13[i7 + 3 * k] = b_C23_k[i7 + 3 * k];
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
  double dv18[9];
  double dv19[9];
  CalculateC_Multi(C_Hip_LUpLeg, C_LUpLeg_LLeg, C_Hip_LLeg);
  InverseC_Multi(C_Hip_Spine2, dv18);
  CalculateC_Multi(dv18, C_Hip_LLeg, C_Spine2_LLeg);
  InverseC_Multi(C_Hip_Spine3, dv19);
  CalculateC_Multi(dv19, C_Hip_LLeg, C_Spine3_LLeg);
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
  double dv20[9];
  double dv21[9];
  CalculateC_Multi(C_Hip_RUpLeg, C_RUpLeg_RLeg, C_Hip_RLeg);
  InverseC_Multi(C_Hip_Spine2, dv20);
  CalculateC_Multi(dv20, C_Hip_RLeg, C_Spine2_RLeg);
  InverseC_Multi(C_Hip_Spine3, dv21);
  CalculateC_Multi(dv21, C_Hip_RLeg, C_Spine3_RLeg);
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
  EulerToC_c(euler1, rotateOrder1, rotateDirection1, C);
  CToEuler_c(C, rotateOrder2, rotateDirection2, euler2);
}

//
// Arguments    : const double euler[3]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void EulerToC_c(const double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  double b_euler[3];
  int i1;
  double dv13[9];

  // % xyz 2015.4.5
  //   EulerToC_Multi  新版 ： 可转化为 C++
  //    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]
  //  store order of euler angle: the rotate order
  //  positive:anticlockwise rotate
  //  euler angle scope: [-pi,pi]
  //  unit:rad
  //  rotateDirection: anticlockwise(1) or clockwise(-1)  111
  for (i1 = 0; i1 < 3; i1++) {
    b_euler[i1] = euler[i1];
  }

  OneEulerToC_c(b_euler, rotateOrder, rotateDirection, dv13);
  memcpy(&C[0], &dv13[0], 9U * sizeof(double));
}

//
// Arguments    : const double C12[9]
//                double C21[9]
// Return Type  : void
//
static void InverseC_Multi(const double C12[9], double C21[9])
{
  double C12_k[9];
  int i9;
  int i10;
  memcpy(&C12_k[0], &C12[0], 9U * sizeof(double));
  for (i9 = 0; i9 < 3; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      C21[i10 + 3 * i9] = C12_k[i9 + 3 * i10];
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
// Arguments    : double euler[3]
//                double rotateOrder
//                const double rotateDirection[3]
//                double C[9]
// Return Type  : void
//
static void OneEulerToC_c(double euler[3], double rotateOrder, const double
  rotateDirection[3], double C[9])
{
  int i;
  double dv14[9];
  double dv15[9];
  double dv16[9];
  double dv17[9];
  int i2;
  int i3;
  for (i = 0; i < 3; i++) {
    euler[i] *= rotateDirection[i];
  }

  memset(&C[0], 0, 9U * sizeof(double));
  if (321.0 == rotateOrder) {
    i = 0;
  } else if (312.0 == rotateOrder) {
    i = 1;
  } else if (123.0 == rotateOrder) {
    i = 2;
  } else if (132.0 == rotateOrder) {
    i = 3;
  } else if (213.0 == rotateOrder) {
    i = 4;
  } else {
    i = -1;
  }

  switch (i) {
   case 0:
    RotateX(euler[2], dv14);
    RotateY(euler[1], dv15);
    RotateZ(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
        }
      }
    }
    break;

   case 1:
    RotateY(euler[2], dv14);
    RotateX(euler[1], dv15);
    RotateZ(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
        }
      }
    }
    break;

   case 2:
    RotateZ(euler[2], dv14);
    RotateY(euler[1], dv15);
    RotateX(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
        }
      }
    }
    break;

   case 3:
    RotateY(euler[2], dv14);
    RotateZ(euler[1], dv15);
    RotateX(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
        }
      }
    }
    break;

   case 4:
    RotateZ(euler[2], dv14);
    RotateX(euler[1], dv15);
    RotateY(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
        }
      }
    }
    break;

   case 5:
    RotateX(euler[2], dv14);
    RotateZ(euler[1], dv15);
    RotateY(euler[0], dv16);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv17[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv17[i + 3 * i2] += dv14[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv17[i + 3 * i3] * dv16[i3 + 3 * i2];
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
  int i4;
  static const signed char iv0[3] = { 1, 0, 0 };

  // % anticlockwise = positive
  for (i4 = 0; i4 < 3; i4++) {
    Cx[3 * i4] = iv0[i4];
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
  int i6;
  static const signed char iv2[3] = { 0, 1, 0 };

  // % anticlockwise rotate is positive
  Cy[0] = cos(angY);
  Cy[3] = 0.0;
  Cy[6] = -sin(angY);
  for (i6 = 0; i6 < 3; i6++) {
    Cy[1 + 3 * i6] = iv2[i6];
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
  int i5;
  static const signed char iv1[3] = { 0, 0, 1 };

  // % anticlockwise = positive
  Cz[0] = cos(angZ);
  Cz[3] = sin(angZ);
  Cz[6] = 0.0;
  Cz[1] = -sin(angZ);
  Cz[4] = cos(angZ);
  Cz[7] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    Cz[2 + 3 * i5] = iv1[i5];
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
  double dv23[3];
  int i12;
  double dv24[9];
  double E_Spine_Head[3];

  // % TransformHead
  //  E_spine_head: euler from spine3 to head
  //  rotateOrder : rotate order of "E_spine_head"
  InverseC_Multi(C_Spine_NeckNew, C_NeckNew_Spine);
  memcpy(&b_C_NeckNew_Spine[0], &C_NeckNew_Spine[0], 9U * sizeof(double));
  for (i12 = 0; i12 < 3; i12++) {
    dv23[i12] = 1.0;
  }

  CalculateC_Multi(b_C_NeckNew_Spine, C_Spine_Head, dv24);
  CToEuler_c(dv24, 213.0, dv23, E_Spine_Head);
  E_Spine_Head[2] = 0.0;
  *HeadYaw = E_Spine_Head[0];
  *HeadPitch = E_Spine_Head[1];

  // %% get new BVH
  for (i12 = 0; i12 < 3; i12++) {
    dv23[i12] = 1.0;
  }

  EulerToC_c(E_Spine_Head, 213.0, dv23, C_NeckNew_Spine);
  for (i12 = 0; i12 < 3; i12++) {
    dv23[i12] = 1.0;
  }

  CToEuler_c(C_NeckNew_Spine, rotateOrder_BVH, dv23, Head_new);
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
  double dv39[3];
  double dv40[3];
  int i17;

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
  for (i17 = 0; i17 < 3; i17++) {
    dv39[i17] = 1.0;
    dv40[i17] = 1.0;
  }

  ChangeEulerRotate_c(LElbow_Nao, 123.0, rotateOrder_BVH, dv39, dv40,
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
  double dv50[3];
  int i24;
  double E_LLeg_LFoot[3];
  double C_RLeg_RFoot_New[9];

  // % LFoot
  for (i24 = 0; i24 < 3; i24++) {
    dv50[i24] = 1.0;
  }

  CToEuler_c(C_LLeg_LFoot, 132.0, dv50, E_LLeg_LFoot);
  *LAnklePitch = E_LLeg_LFoot[0];
  *LAnkleRoll = E_LLeg_LFoot[1];

  // %% get new BVH
  E_LLeg_LFoot[2] = 0.0;
  for (i24 = 0; i24 < 3; i24++) {
    dv50[i24] = 1.0;
  }

  EulerToC_c(E_LLeg_LFoot, 132.0, dv50, C_RLeg_RFoot_New);
  for (i24 = 0; i24 < 3; i24++) {
    dv50[i24] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, rotateOrder_BVH, dv50, LeftFoot_new);
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
  double dv41[3];
  int i18;
  double dv42[9];
  double E_LForeArmNew_LHand[3];
  double LHand_Nao[3];

  // % LHand
  InverseC_Multi(C_Spine_LForeArmNew, C_LForeArmNew_Spine);
  memcpy(&b_C_LForeArmNew_Spine[0], &C_LForeArmNew_Spine[0], 9U * sizeof(double));
  for (i18 = 0; i18 < 3; i18++) {
    dv41[i18] = 1.0;
  }

  CalculateC_Multi(b_C_LForeArmNew_Spine, C_Spine_LHand, dv42);
  CToEuler_c(dv42, 123.0, dv41, E_LForeArmNew_LHand);
  *LWristYaw = E_LForeArmNew_LHand[0];

  // %% get new BVH
  for (i18 = 0; i18 < 3; i18++) {
    LHand_Nao[i18] = 0.0;
  }

  LHand_Nao[0] = E_LForeArmNew_LHand[0];
  for (i18 = 0; i18 < 3; i18++) {
    dv41[i18] = 1.0;
    E_LForeArmNew_LHand[i18] = 1.0;
  }

  ChangeEulerRotate_c(LHand_Nao, 123.0, rotateOrder_BVH, dv41,
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
  double dv48[3];
  int i23;
  double dv49[9];
  double E_LUpLegNew_LLeg[3];

  // % LLeg
  InverseC_Multi(C_Hip_LUpLegNew, C_LUpLegNew_Hip);
  memcpy(&b_C_LUpLegNew_Hip[0], &C_LUpLegNew_Hip[0], 9U * sizeof(double));
  for (i23 = 0; i23 < 3; i23++) {
    dv48[i23] = 1.0;
  }

  CalculateC_Multi(b_C_LUpLegNew_Hip, C_Hip_LLeg, dv49);
  CToEuler_c(dv49, 123.0, dv48, E_LUpLegNew_LLeg);
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
  for (i23 = 0; i23 < 2; i23++) {
    E_LUpLegNew_LLeg[1 + i23] = 0.0;
  }

  for (i23 = 0; i23 < 3; i23++) {
    dv48[i23] = 1.0;
  }

  EulerToC_c(E_LUpLegNew_LLeg, 123.0, dv48, C_LUpLegNew_Hip);
  for (i23 = 0; i23 < 3; i23++) {
    dv48[i23] = 1.0;
  }

  CToEuler_c(C_LUpLegNew_Hip, rotateOrder_BVH, dv48, LeftLeg_New);
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
  double dv34[9];
  static const double dv35[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  double E_LShoulderNew_LArm_Nao[3];
  static const double dv36[3] = { -1.0, 1.0, 1.0 };

  double d_C_LShoulderNew_Spine3[9];
  int i16;
  double dv37[9];
  static const double dv38[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  // % LeftShoulder
  InverseC_Multi(C_Spine3_LShoulderNew, C_LShoulderNew_Spine3);
  memcpy(&b_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(b_C_LShoulderNew_Spine3, C_Spine3_LArm, C_LShoulderNew_Spine3);

  // %% 对 C_LShoulderNew_LArm 进行零位补偿
  memcpy(&c_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(dv35, c_C_LShoulderNew_Spine3, dv34);
  CToEuler_c(dv34, 321.0, dv36, E_LShoulderNew_LArm_Nao);
  E_LShoulderNew_LArm_Nao[2] = 0.0;
  EulerToC_c(E_LShoulderNew_LArm_Nao, 321.0, dv36, C_LShoulderNew_Spine3);
  *LShoulderPitch = E_LShoulderNew_LArm_Nao[0];
  *LShoulderRoll = E_LShoulderNew_LArm_Nao[1];

  // %% get new BVH
  //  零位补偿回去
  //  'ZYX'
  memcpy(&d_C_LShoulderNew_Spine3[0], &C_LShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i16 = 0; i16 < 3; i16++) {
    E_LShoulderNew_LArm_Nao[i16] = 1.0;
  }

  CalculateC_Multi(dv38, d_C_LShoulderNew_Spine3, dv37);
  CToEuler_c(dv37, rotateOrder_BVH, E_LShoulderNew_LArm_Nao, LeftArm_new);
}

//
// Arguments    : const double C_Hip_LUpLeg[9]
//                double rotateOrder_BVH
//                double *LHipRoll
//                double *LHipPitch
//                double LeftUpLeg_new[3]
// Return Type  : void
//
static void TransformLUpLeg(const double C_Hip_LUpLeg[9], double rotateOrder_BVH,
  double *LHipRoll, double *LHipPitch, double LeftUpLeg_new[3])
{
  double C_LUpLeg_Hip[9];
  double C_LUpLeg_SecondHip_k[9];
  double b_LHipRoll[3];
  double dv47[3];
  int i22;

  // % LUpLeg
  //  HipYawPitch：左右大腿的 HipYawPitch Angle  [ Nframes*1 ]
  InverseC_Multi(C_Hip_LUpLeg, C_LUpLeg_Hip);
  memcpy(&C_LUpLeg_SecondHip_k[0], &C_LUpLeg_Hip[0], 9U * sizeof(double));
  *LHipPitch = asin(C_LUpLeg_SecondHip_k[5]);
  *LHipRoll = atan(-C_LUpLeg_SecondHip_k[3] / C_LUpLeg_SecondHip_k[4]);
  b_LHipRoll[0] = *LHipRoll;
  b_LHipRoll[1] = *LHipPitch;
  b_LHipRoll[2] = 0.0;
  for (i22 = 0; i22 < 3; i22++) {
    dv47[i22] = 1.0;
  }

  EulerToC_c(b_LHipRoll, 312.0, dv47, C_LUpLeg_Hip);

  //  if pos_err_sum>1e-1
  //      for k=1:Nframes
  //          LHipPitch(k) = pi-LHipPitch(k) ;
  //      end
  //      E_Hip_LUpLeg_ZXY = [ LHipRoll' LHipPitch' zeros(Nframes,1) ];
  //      C_Hip_LUpLeg_ZXY = EulerToC_c( E_Hip_LUpLeg_ZXY,312,[1,1,1] );
  //  end
  // %% get new BVH
  for (i22 = 0; i22 < 3; i22++) {
    dv47[i22] = 1.0;
  }

  CToEuler_c(C_LUpLeg_Hip, rotateOrder_BVH, dv47, LeftUpLeg_new);
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
  double dv30[3];
  int i14;
  static const double dv31[3] = { -1.0, 1.0, 1.0 };

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
  for (i14 = 0; i14 < 3; i14++) {
    dv30[i14] = 1.0;
  }

  ChangeEulerRotate_c(RElbow_Nao, 123.0, rotateOrder_BVH, dv31, dv30,
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
  double dv46[3];
  int i21;
  double E_RLeg_RFoot[3];
  double C_RLeg_RFoot_New[9];

  // % RFoot
  for (i21 = 0; i21 < 3; i21++) {
    dv46[i21] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot, 132.0, dv46, E_RLeg_RFoot);
  *RAnklePitch = E_RLeg_RFoot[0];
  *RAnkleRoll = E_RLeg_RFoot[1];

  // %% get new BVH
  E_RLeg_RFoot[2] = 0.0;
  for (i21 = 0; i21 < 3; i21++) {
    dv46[i21] = 1.0;
  }

  EulerToC_c(E_RLeg_RFoot, 132.0, dv46, C_RLeg_RFoot_New);
  for (i21 = 0; i21 < 3; i21++) {
    dv46[i21] = 1.0;
  }

  CToEuler_c(C_RLeg_RFoot_New, rotateOrder_BVH, dv46, RightFoot_new);
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
  double dv32[9];
  double E_RForeArmNew_RHand[3];
  static const double dv33[3] = { -1.0, 1.0, 1.0 };

  double RHand_Nao[3];
  int i15;

  // % RHand
  InverseC_Multi(C_Spine_RForeArmNew, C_RForeArmNew_Spine);
  memcpy(&b_C_RForeArmNew_Spine[0], &C_RForeArmNew_Spine[0], 9U * sizeof(double));
  CalculateC_Multi(b_C_RForeArmNew_Spine, C_Spine_RHand, dv32);
  CToEuler_c(dv32, 123.0, dv33, E_RForeArmNew_RHand);
  *RWristYaw = E_RForeArmNew_RHand[0];

  // %% get new BVH
  for (i15 = 0; i15 < 3; i15++) {
    RHand_Nao[i15] = 0.0;
  }

  RHand_Nao[0] = E_RForeArmNew_RHand[0];
  for (i15 = 0; i15 < 3; i15++) {
    E_RForeArmNew_RHand[i15] = 1.0;
  }

  ChangeEulerRotate_c(RHand_Nao, 123.0, rotateOrder_BVH, dv33,
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
  double dv44[3];
  int i20;
  double dv45[9];
  double E_RUpLegNew_RLeg[3];

  // % RLeg
  InverseC_Multi(C_Hip_RUpLegNew, C_RUpLegNew_Hip);
  memcpy(&b_C_RUpLegNew_Hip[0], &C_RUpLegNew_Hip[0], 9U * sizeof(double));
  for (i20 = 0; i20 < 3; i20++) {
    dv44[i20] = 1.0;
  }

  CalculateC_Multi(b_C_RUpLegNew_Hip, C_Hip_RLeg, dv45);
  CToEuler_c(dv45, 123.0, dv44, E_RUpLegNew_RLeg);
  *RKneePitch = E_RUpLegNew_RLeg[0];

  //  E_UpLeg_Leg = CToEuler_c( C_RUpLeg_RLeg,123 );
  //  RKneePitch = E_UpLeg_Leg(:,1)';
  // %% get new BVH
  for (i20 = 0; i20 < 2; i20++) {
    E_RUpLegNew_RLeg[1 + i20] = 0.0;
  }

  for (i20 = 0; i20 < 3; i20++) {
    dv44[i20] = 1.0;
  }

  EulerToC_c(E_RUpLegNew_RLeg, 123.0, dv44, C_RUpLegNew_Hip);
  for (i20 = 0; i20 < 3; i20++) {
    dv44[i20] = 1.0;
  }

  CToEuler_c(C_RUpLegNew_Hip, rotateOrder_BVH, dv44, RightLeg_New);
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
  double dv25[3];
  int i13;
  double dv26[9];
  static const double dv27[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  double E_RShoulderNew_RArm_Nao[3];
  double d_C_RShoulderNew_Spine3[9];
  double dv28[9];
  static const double dv29[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  // % RightShoulder
  InverseC_Multi(C_Spine3_RShoulderNew, C_RShoulderNew_Spine3);
  memcpy(&b_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  CalculateC_Multi(b_C_RShoulderNew_Spine3, C_Spine3_RArm, C_RShoulderNew_Spine3);

  // %% 对 C_RShoulderNew_RArm 进行零位补偿
  memcpy(&c_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i13 = 0; i13 < 3; i13++) {
    dv25[i13] = 1.0;
  }

  CalculateC_Multi(dv27, c_C_RShoulderNew_Spine3, dv26);
  CToEuler_c(dv26, 321.0, dv25, E_RShoulderNew_RArm_Nao);
  E_RShoulderNew_RArm_Nao[2] = 0.0;
  for (i13 = 0; i13 < 3; i13++) {
    dv25[i13] = 1.0;
  }

  EulerToC_c(E_RShoulderNew_RArm_Nao, 321.0, dv25, C_RShoulderNew_Spine3);
  *RShoulderPitch = E_RShoulderNew_RArm_Nao[0];
  *RShoulderRoll = E_RShoulderNew_RArm_Nao[1];

  // %% get new BVH
  //  零位补偿回去
  //  321
  memcpy(&d_C_RShoulderNew_Spine3[0], &C_RShoulderNew_Spine3[0], 9U * sizeof
         (double));
  for (i13 = 0; i13 < 3; i13++) {
    dv25[i13] = 1.0;
  }

  CalculateC_Multi(dv29, d_C_RShoulderNew_Spine3, dv28);
  CToEuler_c(dv28, rotateOrder_BVH, dv25, RightArm_new);
}

//
// Arguments    : const double C_Hip_RUpLeg[9]
//                double rotateOrder_BVH
//                double *RHipRoll
//                double *RHipPitch
//                double RightUpLeg_new[3]
// Return Type  : void
//
static void TransformRUpLeg(const double C_Hip_RUpLeg[9], double rotateOrder_BVH,
  double *RHipRoll, double *RHipPitch, double RightUpLeg_new[3])
{
  double C_RUpLeg_Hip[9];
  double C_RUpLeg_SecondHip_k[9];
  double b_RHipRoll[3];
  double dv43[3];
  int k;
  double C_HipYawPitch[9];
  double C_Hip_RUpLeg_ZXY_k[9];
  double pos[3];
  int i19;
  static const signed char b[3] = { 0, 1, 0 };

  double posNew[3];
  double y;
  double c_RHipRoll[3];

  // % RUpLeg
  //  HipYawPitch：左右大腿的 HipYawPitch Angle  [ 1*Nframes ]
  InverseC_Multi(C_Hip_RUpLeg, C_RUpLeg_Hip);
  memcpy(&C_RUpLeg_SecondHip_k[0], &C_RUpLeg_Hip[0], 9U * sizeof(double));
  *RHipPitch = asin(C_RUpLeg_SecondHip_k[5]);
  *RHipRoll = atan(-C_RUpLeg_SecondHip_k[3] / C_RUpLeg_SecondHip_k[4]);
  b_RHipRoll[0] = *RHipRoll;
  b_RHipRoll[1] = *RHipPitch;
  b_RHipRoll[2] = 0.0;
  for (k = 0; k < 3; k++) {
    dv43[k] = 1.0;
  }

  EulerToC_c(b_RHipRoll, 312.0, dv43, C_HipYawPitch);
  for (k = 0; k < 9; k++) {
    C_RUpLeg_SecondHip_k[k] = C_RUpLeg_Hip[k];
    C_Hip_RUpLeg_ZXY_k[k] = C_HipYawPitch[k];
  }

  for (k = 0; k < 3; k++) {
    pos[k] = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      pos[k] += C_RUpLeg_SecondHip_k[k + 3 * i19] * (double)b[i19];
    }

    posNew[k] = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      posNew[k] += C_Hip_RUpLeg_ZXY_k[i19 + 3 * k] * (double)b[i19];
    }

    pos[k] -= posNew[k];
    posNew[k] = fabs(pos[k]);
  }

  y = posNew[0];
  for (k = 0; k < 2; k++) {
    y += posNew[k + 1];
  }

  if (y > 0.1) {
    *RHipPitch = 3.1415926535897931 - *RHipPitch;
    c_RHipRoll[0] = *RHipRoll;
    c_RHipRoll[1] = *RHipPitch;
    c_RHipRoll[2] = 0.0;
    for (k = 0; k < 3; k++) {
      dv43[k] = 1.0;
    }

    EulerToC_c(c_RHipRoll, 312.0, dv43, C_HipYawPitch);
  }

  // %% get new BVH
  for (k = 0; k < 3; k++) {
    dv43[k] = 1.0;
  }

  CToEuler_c(C_HipYawPitch, rotateOrder_BVH, dv43, RightUpLeg_new);
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
// function [ NaoData,ROOT_Hips_RotationNew ] = BVHTransform_c( BVHData )
// Arguments    : const float BVHData[354]
//                double NaoData[25]
// Return Type  : void
//
void BVHTransform_c(const float BVHData[354], double NaoData[25])
{
  double b_BVHData[354];
  int i0;
  double ROOT_Hips_Position[3];
  double dv0[3];
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
  double dv1[9];
  double dv2[9];
  double dv3[9];
  static const double C_I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  double HeadPitch;
  double HeadYaw;
  double RShoulderRoll;
  double RShoulderPitch;
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

  // % xyz  2015.3.25
  //  Transform BVH data Format
  //  BVHData: 1*354  degree
  //  NaoData: 25*1 degree
  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = BVHData[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    ROOT_Hips_Position[i0] = b_BVHData[i0];
  }

  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = b_BVHData[i0] * 3.1415926535897931 / 180.0;
  }

  for (i0 = 0; i0 < 3; i0++) {
    b_BVHData[i0] = ROOT_Hips_Position[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  BVHDataToRotate_c(b_BVHData, 213.0, dv0, C_World_Hip, C_Spine3New_RArmNew,
                    C_Spine3_Head, C_Spine2_Head, C_Spine2_Spine3, C_Spine3_Neck,
                    C_Neck_Head, C_World_Spine2, C_LShoulder_LArm,
                    C_LArm_LForeArm, C_LForeArm_LHand, C_Spine3_RShoulder,
                    C_RShoulder_RArm, C_RArm_RForeArm, C_RForeArm_RHand,
                    C_Hip_RUpLeg, C_RUpLeg_RLeg, C_RLeg_RFoot, C_Hip_LUpLeg,
                    C_LUpLeg_LLeg, C_LLeg_LFoot);

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
  CalculateC_Multi(C_Spine3_Neck, C_Neck_Head, dv1);
  CalculateC_Multi(C_Spine2_Spine3, dv1, dv2);
  CalculateC_Multi(C_I, C_I, dv3);
  TransformHead(dv2, dv3, 213.0, &HeadYaw, &HeadPitch, ROOT_Hips_Position);

  // % Right Shoulder
  CalculateC_Multi(C_I, C_I, C_Spine3_Head);

  //  dbstop in TransformRShoulder
  TransformRShoulder(C_Spine3_Head, C_Spine3_RShoulder, 213.0, &RShoulderPitch,
                     &RShoulderRoll, ROOT_Hips_Position);

  // % Right Elbow
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_Spine2_Head);
  memcpy(&c_C_Spine2_Head[0], &C_Spine2_Head[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, c_C_Spine2_Head, C_Spine2_Head);
  TransformRElbow(C_RShoulder_RArm, C_Spine2_Head, 213.0, RElbow_Nao,
                  ROOT_Hips_Position);

  // % RHand
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_World_Spine2);
  memcpy(&c_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, c_C_World_Spine2, dv4);
  TransformRHand(C_RArm_RForeArm, dv4, 213.0, &RWristYaw, ROOT_Hips_Position);

  // % Left Shoulder
  CalculateC_Multi(C_I, C_I, dv5);
  TransformLShoulder(dv5, C_Spine2_LArm, 213.0, &LShoulderPitch, &LShoulderRoll,
                     ROOT_Hips_Position);

  // % Left Elbow
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_World_Spine2);
  CalculateC_Multi(C_I, C_World_Spine2, C_Spine2_Head);
  CalculateC_Multi(C_I, C_Spine2_Head, dv6);
  TransformLElbow(C_Spine2_LForeArm, dv6, 213.0, LElbow_Nao, ROOT_Hips_Position);

  // % LHand
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_World_Spine2);
  memcpy(&d_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine2_Head, d_C_World_Spine2, dv7);
  CalculateC_Multi(C_I, dv7, dv8);
  TransformLHand(C_Spine2_LHand, dv8, 213.0, &LWristYaw, ROOT_Hips_Position);

  // % RLeg
  InverseC_Multi(C_Hip_Spine2, C_Spine3_Head);

  //  dbstop in TransformRUpLeg
  CalculateC_Multi(C_Spine3_Head, C_Hip_RUpLeg, dv9);
  TransformRUpLeg(dv9, 213.0, &RHipRoll, &RHipPitch, ROOT_Hips_Position);
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_World_Spine2);
  memcpy(&e_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, e_C_World_Spine2, dv10);
  TransformRLeg(dv10, C_LShoulder_LArm, 213.0, &RKneePitch, ROOT_Hips_Position);
  TransformRFoot(C_RLeg_RFoot, 213.0, &RAnklePitch, &RAnkleRoll,
                 ROOT_Hips_Position);

  // % LLeg
  CalculateC_Multi(C_Spine3_Head, C_Hip_LUpLeg, dv11);
  TransformLUpLeg(dv11, 213.0, &LHipRoll, &LHipPitch, ROOT_Hips_Position);
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  EulerToC_c(ROOT_Hips_Position, 213.0, dv0, C_World_Spine2);
  memcpy(&f_C_World_Spine2[0], &C_World_Spine2[0], 9U * sizeof(double));
  CalculateC_Multi(C_Spine3_Head, f_C_World_Spine2, dv12);
  TransformLLeg(dv12, C_LArm_LForeArm, 213.0, &LKneePitch, ROOT_Hips_Position);
  TransformLFoot(C_LLeg_LFoot, 213.0, &LAnklePitch, &LAnkleRoll,
                 ROOT_Hips_Position);

  // % Get Nao
  //  LHipRoll = LHipRoll*0 ;
  //  LHipPitch = LHipPitch*0 ;
  //  LKneePitch = LKneePitch*0 ;
  //  RHipRoll = RHipRoll*0 ;
  //  RHipPitch = RHipPitch*0;
  //
  //  RKneePitch = RKneePitch*0 ;
  //  LAnklePitch = LAnklePitch*0 ;
  //  LAnkleRoll = LAnkleRoll*0 ;
  //  RAnklePitch = RAnklePitch*0 ;
  //  RAnkleRoll = RAnkleRoll*0;
  NaoData[0] = HeadYaw;
  NaoData[1] = HeadPitch;
  NaoData[2] = LShoulderPitch;
  NaoData[3] = LShoulderRoll;
  NaoData[4] = LElbow_Nao[0];
  NaoData[5] = LElbow_Nao[1];
  NaoData[6] = RShoulderPitch;
  NaoData[7] = RShoulderRoll;
  NaoData[8] = RElbow_Nao[0];
  NaoData[9] = RElbow_Nao[1];
  NaoData[10] = 0.0;
  NaoData[11] = LHipRoll;
  NaoData[12] = LHipPitch;
  NaoData[13] = LKneePitch;
  NaoData[14] = RHipRoll;
  NaoData[15] = RHipPitch;
  NaoData[16] = RKneePitch;
  NaoData[17] = LAnklePitch;
  NaoData[18] = LAnkleRoll;
  NaoData[19] = RAnklePitch;
  NaoData[20] = RAnkleRoll;
  NaoData[21] = LWristYaw;
  NaoData[22] = RWristYaw;
  NaoData[23] = 0.0;
  NaoData[24] = 0.0;
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
