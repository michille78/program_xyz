/*
 * BVHTransform_c.cpp
 *
 * Code generation for function 'BVHTransform_c'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "BVHTransform_c.h"
#include "eml_error.h"
#include "BVHTransform_c_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 17, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo b_emlrtRSI = { 61, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo c_emlrtRSI = { 76, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo d_emlrtRSI = { 87, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo e_emlrtRSI = { 90, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo f_emlrtRSI = { 99, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo g_emlrtRSI = { 103, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo h_emlrtRSI = { 112, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo i_emlrtRSI = { 122, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo j_emlrtRSI = { 126, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo k_emlrtRSI = { 135, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo l_emlrtRSI = { 139, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo m_emlrtRSI = { 148, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo n_emlrtRSI = { 154, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo o_emlrtRSI = { 156, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo p_emlrtRSI = { 158, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo q_emlrtRSI = { 159, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo r_emlrtRSI = { 162, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo s_emlrtRSI = { 164, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo t_emlrtRSI = { 166, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo u_emlrtRSI = { 168, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo v_emlrtRSI = { 757, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo w_emlrtRSI = { 758, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo x_emlrtRSI = { 759, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo y_emlrtRSI = { 760, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ab_emlrtRSI = { 761, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo bb_emlrtRSI = { 762, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo cb_emlrtRSI = { 763, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo db_emlrtRSI = { 764, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo eb_emlrtRSI = { 765, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo fb_emlrtRSI = { 766, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo gb_emlrtRSI = { 767, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo hb_emlrtRSI = { 768, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ib_emlrtRSI = { 769, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo jb_emlrtRSI = { 770, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo kb_emlrtRSI = { 771, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo lb_emlrtRSI = { 772, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo mb_emlrtRSI = { 773, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo nb_emlrtRSI = { 774, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ob_emlrtRSI = { 775, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo pb_emlrtRSI = { 776, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo qb_emlrtRSI = { 777, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo rb_emlrtRSI = { 798, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo sb_emlrtRSI = { 851, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo tb_emlrtRSI = { 887, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ub_emlrtRSI = { 882, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo vb_emlrtRSI = { 878, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo wb_emlrtRSI = { 874, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo xb_emlrtRSI = { 869, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo yb_emlrtRSI = { 864, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ac_emlrtRSI = { 14, "asin",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\matlab\\elfun\\asin.m"
};

static emlrtRSInfo bc_emlrtRSI = { 312, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo cc_emlrtRSI = { 318, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo dc_emlrtRSI = { 319, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ec_emlrtRSI = { 337, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo fc_emlrtRSI = { 339, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo gc_emlrtRSI = { 350, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo hc_emlrtRSI = { 395, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ic_emlrtRSI = { 412, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo jc_emlrtRSI = { 14, "acos",
  "D:\\Program Files\\MATLAB\\R2014a\\toolbox\\eml\\lib\\matlab\\elfun\\acos.m"
};

static emlrtRSInfo kc_emlrtRSI = { 900, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo lc_emlrtRSI = { 901, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo mc_emlrtRSI = { 451, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo nc_emlrtRSI = { 463, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo oc_emlrtRSI = { 366, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo pc_emlrtRSI = { 368, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo qc_emlrtRSI = { 380, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo rc_emlrtRSI = { 432, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo sc_emlrtRSI = { 442, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo tc_emlrtRSI = { 473, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo uc_emlrtRSI = { 483, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo vc_emlrtRSI = { 498, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo wc_emlrtRSI = { 509, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo xc_emlrtRSI = { 514, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo yc_emlrtRSI = { 536, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ad_emlrtRSI = { 540, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo gd_emlrtRSI = { 613, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo hd_emlrtRSI = { 622, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo id_emlrtRSI = { 623, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo jd_emlrtRSI = { 658, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo kd_emlrtRSI = { 664, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ld_emlrtRSI = { 665, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo md_emlrtRSI = { 562, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo nd_emlrtRSI = { 573, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo od_emlrtRSI = { 578, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo pd_emlrtRSI = { 604, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo qd_emlrtRSI = { 632, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo rd_emlrtRSI = { 649, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo sd_emlrtRSI = { 651, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo td_emlrtRSI = { 672, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo ud_emlrtRSI = { 678, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtRSInfo vd_emlrtRSI = { 679, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m"
};

static emlrtDCInfo emlrtDCI = { 861, 8, "BVHTransform_c",
  "E:\\document and program\\program_xyz_noitom\\BVHTransform_C\\BVHTransform_c.m",
  1 };

/* Function Declarations */
static void BVHDataToRotate_c(const emlrtStack *sp, const real_T BVHData[354],
  real_T rotateOrder_BVH, const real_T rotateDirection_BVH[3], real_T
  C_World_Hip[9], real_T C_Hip_Spine[9], real_T C_Spine_Spine1[9], real_T
  C_Spine1_Spine2[9], real_T C_Spine2_Spine3[9], real_T C_Spine3_Neck[9], real_T
  C_Neck_Head[9], real_T C_Spine3_LShoulder[9], real_T C_LShoulder_LArm[9],
  real_T C_LArm_LForeArm[9], real_T C_LForeArm_LHand[9], real_T
  C_Spine3_RShoulder[9], real_T C_RShoulder_RArm[9], real_T C_RArm_RForeArm[9],
  real_T C_RForeArm_RHand[9], real_T C_Hip_RUpLeg[9], real_T C_RUpLeg_RLeg[9],
  real_T C_RLeg_RFoot[9], real_T C_Hip_LUpLeg[9], real_T C_LUpLeg_LLeg[9],
  real_T C_LLeg_LFoot[9]);
static void CToEuler_c(const emlrtStack *sp, const real_T C[9], real_T
  rotateOrder, const real_T rotateDirection[3], real_T euler[3]);
static void CalculateC_Multi(const real_T C12[9], const real_T C23[9], real_T
  C13[9]);
static void CalculatedLLegRotate(const real_T C_Hip_Spine2[9], const real_T
  C_Hip_Spine3[9], const real_T C_Hip_LUpLeg[9], const real_T C_LUpLeg_LLeg[9],
  real_T C_Hip_LLeg[9], real_T C_Spine2_LLeg[9], real_T C_Spine3_LLeg[9]);
static void CalculatedLeftArmRotate(const real_T C_Hip_Spine[9], const real_T
  C_Spine_Spine1[9], const real_T C_Spine1_Spine2[9], const real_T
  C_Spine2_Spine3[9], const real_T C_Spine3_LShoulder[9], const real_T
  C_LShoulder_LArm[9], const real_T C_LArm_LForeArm[9], const real_T
  C_LForeArm_LHand[9], real_T C_Hip_Spine2[9], real_T C_Hip_Spine3[9], real_T
  C_Spine_Spine3[9], real_T C_Hip_LArm[9], real_T C_Hip_LForeArm[9], real_T
  C_Hip_LHand[9], real_T C_Spine_LArm[9], real_T C_Spine_LForeArm[9], real_T
  C_Spine_LHand[9], real_T C_Spine2_LArm[9], real_T C_Spine2_LForeArm[9], real_T
  C_Spine2_LHand[9], real_T C_Spine3_LArm[9], real_T C_Spine3_LForeArm[9],
  real_T C_Spine3_LHand[9]);
static void CalculatedRLegRotate(const real_T C_Hip_Spine2[9], const real_T
  C_Hip_Spine3[9], const real_T C_Hip_RUpLeg[9], const real_T C_RUpLeg_RLeg[9],
  real_T C_Hip_RLeg[9], real_T C_Spine2_RLeg[9], real_T C_Spine3_RLeg[9]);
static void CalculatedRightArmRotate(const real_T C_Hip_Spine3[9], const real_T
  C_Spine2_Spine3[9], const real_T C_Spine_Spine3[9], const real_T
  C_Spine3_RShoulder[9], const real_T C_RShoulder_RArm[9], const real_T
  C_RArm_RForeArm[9], const real_T C_RForeArm_RHand[9], real_T C_Hip_RArm[9],
  real_T C_Hip_RForeArm[9], real_T C_Hip_RHand[9], real_T C_Spine_RArm[9],
  real_T C_Spine_RForeArm[9], real_T C_Spine_RHand[9], real_T C_Spine2_RArm[9],
  real_T C_Spine2_RForeArm[9], real_T C_Spine2_RHand[9], real_T C_Spine3_RArm[9],
  real_T C_Spine3_RForeArm[9], real_T C_Spine3_RHand[9]);
static void ChangeEulerRotate_c(const emlrtStack *sp, const real_T euler1[3],
  real_T rotateOrder1, real_T rotateOrder2, const real_T rotateDirection1[3],
  const real_T rotateDirection2[3], real_T euler2[3]);
static void EulerToC_c(const emlrtStack *sp, const real_T euler[3], real_T
  rotateOrder, const real_T rotateDirection[3], real_T C[9]);
static void FQtoCnb(real_T Q[4], real_T Cnb[9]);
static void InverseC_Multi(const real_T C12[9], real_T C21[9]);
static void OneCToEuler_c(const emlrtStack *sp, const real_T C[9], real_T
  rotateOrder, const real_T rotateDirection[3], real_T euler[3]);
static void OneEulerToC_c(const emlrtStack *sp, real_T euler[3], real_T
  rotateOrder, const real_T rotateDirection[3], real_T C[9]);
static void RotateX(real_T angX, real_T Cx[9]);
static void RotateY(real_T angY, real_T Cy[9]);
static void RotateZ(real_T angZ, real_T Cz[9]);
static void TransformHead(const emlrtStack *sp, const real_T C_Spine_Head[9],
  const real_T C_Spine_NeckNew[9], real_T rotateOrder_BVH, real_T *HeadYaw,
  real_T *HeadPitch, real_T Head_new[3]);
static void TransformLElbow(const emlrtStack *sp, const real_T C_Spine_LForeArm
  [9], const real_T C_Spine_LArmNew[9], real_T rotateOrder_BVH, real_T
  LElbow_Nao[3], real_T LeftForeArm_new[3]);
static void TransformLFoot(const emlrtStack *sp, const real_T C_LLeg_LFoot[9],
  real_T rotateOrder_BVH, real_T *LAnklePitch, real_T *LAnkleRoll, real_T
  LeftFoot_new[3]);
static void TransformLHand(const emlrtStack *sp, const real_T C_Spine_LHand[9],
  const real_T C_Spine_LForeArmNew[9], real_T rotateOrder_BVH, real_T *LWristYaw,
  real_T LeftHand_new[3]);
static void TransformLLeg(const emlrtStack *sp, const real_T C_Hip_LUpLegNew[9],
  const real_T C_Hip_LLeg[9], real_T rotateOrder_BVH, real_T *LKneePitch, real_T
  LeftLeg_New[3]);
static void TransformLShoulder(const emlrtStack *sp, const real_T
  C_Spine3_LShoulderNew[9], const real_T C_Spine3_LArm[9], real_T
  rotateOrder_BVH, real_T *LShoulderPitch, real_T *LShoulderRoll, real_T
  LeftArm_new[3]);
static void TransformLUpLeg(const emlrtStack *sp, const real_T C_Hip_LUpLeg[9],
  real_T HipYawPitch, real_T rotateOrder_BVH, real_T *LHipRoll, real_T
  *LHipPitch, real_T LeftUpLeg_new[3]);
static void TransformRElbow(const emlrtStack *sp, const real_T C_Spine_RForeArm
  [9], const real_T C_Spine_RArmNew[9], real_T rotateOrder_BVH, real_T
  RElbow_Nao[3], real_T RightForeArm_new[3]);
static void TransformRFoot(const emlrtStack *sp, const real_T C_RLeg_RFoot[9],
  real_T rotateOrder_BVH, real_T *RAnklePitch, real_T *RAnkleRoll, real_T
  RightFoot_new[3]);
static void TransformRHand(const emlrtStack *sp, const real_T C_Spine_RHand[9],
  const real_T C_Spine_RForeArmNew[9], real_T rotateOrder_BVH, real_T *RWristYaw,
  real_T RightHand_new[3]);
static void TransformRLeg(const emlrtStack *sp, const real_T C_Hip_RUpLegNew[9],
  const real_T C_Hip_RLeg[9], real_T rotateOrder_BVH, real_T *RKneePitch, real_T
  RightLeg_New[3]);
static void TransformRShoulder(const emlrtStack *sp, const real_T
  C_Spine3_RShoulderNew[9], const real_T C_Spine3_RArm[9], real_T
  rotateOrder_BVH, real_T *RShoulderPitch, real_T *RShoulderRoll, real_T
  RightArm_new[3]);
static void TransformRUpLeg(const emlrtStack *sp, const real_T C_Hip_RUpLeg[9],
  real_T HipYawPitch, real_T rotateOrder_BVH, real_T *RHipRoll, real_T
  *RHipPitch, real_T RightUpLeg_new[3]);

/* Function Definitions */

/*
 * function [ C_World_Hip,C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_Neck,C_Neck_Head,C_Spine3_LShoulder,...
 * C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand,...
 * C_Hip_RUpLeg,C_RUpLeg_RLeg,C_RLeg_RFoot,C_Hip_LUpLeg,C_LUpLeg_LLeg,C_LLeg_LFoot ] = BVHDataToRotate_c( BVHData,rotateOrder_BVH,rotateDirection_BVH )
 */
static void BVHDataToRotate_c(const emlrtStack *sp, const real_T BVHData[354],
  real_T rotateOrder_BVH, const real_T rotateDirection_BVH[3], real_T
  C_World_Hip[9], real_T C_Hip_Spine[9], real_T C_Spine_Spine1[9], real_T
  C_Spine1_Spine2[9], real_T C_Spine2_Spine3[9], real_T C_Spine3_Neck[9], real_T
  C_Neck_Head[9], real_T C_Spine3_LShoulder[9], real_T C_LShoulder_LArm[9],
  real_T C_LArm_LForeArm[9], real_T C_LForeArm_LHand[9], real_T
  C_Spine3_RShoulder[9], real_T C_RShoulder_RArm[9], real_T C_RArm_RForeArm[9],
  real_T C_RForeArm_RHand[9], real_T C_Hip_RUpLeg[9], real_T C_RUpLeg_RLeg[9],
  real_T C_RLeg_RFoot[9], real_T C_Hip_LUpLeg[9], real_T C_LUpLeg_LLeg[9],
  real_T C_LLeg_LFoot[9])
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /*  BVHData: Nframes*(59*6)   [ 1*354 ] */
  /* 'BVHTransform_c:686' coder.inline('never'); */
  /* 'BVHTransform_c:688' Nframes = size(BVHData,1); */
  /* 'BVHTransform_c:691' if size(BVHData,2)==180 */
  /* 'BVHTransform_c:693' else */
  /* 'BVHTransform_c:694' step_k = 6 ; */
  /* 'BVHTransform_c:696' i = 0 ; */
  /* 'BVHTransform_c:697' ROOT_Hips = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:697' i = i+1 ; */
  /* 'BVHTransform_c:698' RightUpLeg = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:698' i = i+1 ; */
  /* 'BVHTransform_c:699' RightLeg = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:699' i = i+1 ; */
  /* 'BVHTransform_c:700' RightFoot = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:700' i = i+1 ; */
  /* 'BVHTransform_c:701' LeftUpLeg = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:701' i = i+1 ; */
  /* 'BVHTransform_c:702' LeftLeg = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:702' i = i+1 ; */
  /* 'BVHTransform_c:703' LeftFoot = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:703' i = i+1 ; */
  /* 'BVHTransform_c:704' Spine = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:704' i = i+1 ; */
  /* 'BVHTransform_c:705' Spine1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:705' i = i+1 ; */
  /* 'BVHTransform_c:706' Spine2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:706' i = i+1 ; */
  /* 'BVHTransform_c:707' Spine3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:707' i = i+1 ; */
  /* 'BVHTransform_c:708' Neck = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:708' i = i+1 ; */
  /* 'BVHTransform_c:709' Head = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:709' i = i+1 ; */
  /* 'BVHTransform_c:710' RightShoulder = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:710' i = i+1 ; */
  /* 'BVHTransform_c:711' RightArm = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:711' i = i+1 ; */
  /* 'BVHTransform_c:712' RightForeArm = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:712' i = i+1 ; */
  /* 'BVHTransform_c:713' RightHand = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:713' i = i+1 ; */
  /* 'BVHTransform_c:714' RightHandThumb1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:714' i = i+1 ; */
  /* 'BVHTransform_c:715' RightHandThumb2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:715' i = i+1 ; */
  /* 'BVHTransform_c:716' RightHandThumb3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:716' i = i+1 ; */
  /* 'BVHTransform_c:717' RightInHandIndex = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:717' i = i+1 ; */
  /* 'BVHTransform_c:718' RightHandIndex1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:718' i = i+1 ; */
  /* 'BVHTransform_c:719' RightHandIndex2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:719' i = i+1 ; */
  /* 'BVHTransform_c:720' RightHandIndex3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:720' i = i+1 ; */
  /* 'BVHTransform_c:721' RightInHandMiddle = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:721' i = i+1 ; */
  /* 'BVHTransform_c:722' RightHandMiddle1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:722' i = i+1 ; */
  /* 'BVHTransform_c:723' RightHandMiddle2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:723' i = i+1 ; */
  /* 'BVHTransform_c:724' RightHandMiddle3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:724' i = i+1 ; */
  /* 'BVHTransform_c:725' RightInHandRing = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:725' i = i+1 ; */
  /* 'BVHTransform_c:726' RightHandRing1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:726' i = i+1 ; */
  /* 'BVHTransform_c:727' RightHandRing2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:727' i = i+1 ; */
  /* 'BVHTransform_c:728' RightHandRing3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:728' i = i+1 ; */
  /* 'BVHTransform_c:729' RightInHandPinky = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:729' i = i+1 ; */
  /* 'BVHTransform_c:730' RightHandPinky1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:730' i = i+1 ; */
  /* 'BVHTransform_c:731' RightHandPinky2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:731' i = i+1 ; */
  /* 'BVHTransform_c:732' RightHandPinky3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:732' i = i+1 ; */
  /* 'BVHTransform_c:733' LeftShoulder = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:733' i = i+1 ; */
  /* 'BVHTransform_c:734' LeftArm = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:734' i = i+1 ; */
  /* 'BVHTransform_c:735' LeftForeArm = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:735' i = i+1 ; */
  /* 'BVHTransform_c:736' LeftHand = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:736' i = i+1 ; */
  /* 'BVHTransform_c:737' LeftHandThumb1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:737' i = i+1 ; */
  /* 'BVHTransform_c:738' LeftHandThumb2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:738' i = i+1 ; */
  /* 'BVHTransform_c:739' LeftHandThumb3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:739' i = i+1 ; */
  /* 'BVHTransform_c:740' LeftInHandIndex = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:740' i = i+1 ; */
  /* 'BVHTransform_c:741' LeftHandIndex1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:741' i = i+1 ; */
  /* 'BVHTransform_c:742' LeftHandIndex2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:742' i = i+1 ; */
  /* 'BVHTransform_c:743' LeftHandIndex3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:743' i = i+1 ; */
  /* 'BVHTransform_c:744' LeftInHandMiddle = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:744' i = i+1 ; */
  /* 'BVHTransform_c:745' LeftHandMiddle1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:745' i = i+1 ; */
  /* 'BVHTransform_c:746' LeftHandMiddle2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:746' i = i+1 ; */
  /* 'BVHTransform_c:747' LeftHandMiddle3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:747' i = i+1 ; */
  /* 'BVHTransform_c:748' LeftInHandRing = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:748' i = i+1 ; */
  /* 'BVHTransform_c:749' LeftHandRing1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:749' i = i+1 ; */
  /* 'BVHTransform_c:750' LeftHandRing2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:750' i = i+1 ; */
  /* 'BVHTransform_c:751' LeftHandRing3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:751' i = i+1 ; */
  /* 'BVHTransform_c:752' LeftInHandPinky = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:752' i = i+1 ; */
  /* 'BVHTransform_c:753' LeftHandPinky1 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:753' i = i+1 ; */
  /* 'BVHTransform_c:754' LeftHandPinky2 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:754' i = i+1 ; */
  /* 'BVHTransform_c:755' LeftHandPinky3 = BVHData( :,4+step_k*i:6+step_k*i ); */
  /* 'BVHTransform_c:757' C_World_Hip = EulerToC_c( ROOT_Hips,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &v_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[3], rotateOrder_BVH,
             rotateDirection_BVH, C_World_Hip);

  /* 'BVHTransform_c:758' C_Hip_Spine = EulerToC_c( Spine,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &w_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[45], rotateOrder_BVH,
             rotateDirection_BVH, C_Hip_Spine);

  /* 'BVHTransform_c:759' C_Spine_Spine1 = EulerToC_c( Spine1,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &x_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[51], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine_Spine1);

  /* 'BVHTransform_c:760' C_Spine1_Spine2 = EulerToC_c( Spine2,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &y_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[57], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine1_Spine2);

  /* 'BVHTransform_c:761' C_Spine2_Spine3 = EulerToC_c( Spine3,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &ab_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[63], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine2_Spine3);

  /* 'BVHTransform_c:762' C_Spine3_Neck = EulerToC_c( Neck,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &bb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[69], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine3_Neck);

  /* 'BVHTransform_c:763' C_Neck_Head = EulerToC_c( Head,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &cb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[75], rotateOrder_BVH,
             rotateDirection_BVH, C_Neck_Head);

  /* 'BVHTransform_c:764' C_Spine3_LShoulder = EulerToC_c( LeftShoulder,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &db_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[219], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine3_LShoulder);

  /* 'BVHTransform_c:765' C_LShoulder_LArm = EulerToC_c( LeftArm,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &eb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[225], rotateOrder_BVH,
             rotateDirection_BVH, C_LShoulder_LArm);

  /* 'BVHTransform_c:766' C_LArm_LForeArm = EulerToC_c( LeftForeArm,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &fb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[231], rotateOrder_BVH,
             rotateDirection_BVH, C_LArm_LForeArm);

  /* 'BVHTransform_c:767' C_LForeArm_LHand = EulerToC_c( LeftHand,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &gb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[237], rotateOrder_BVH,
             rotateDirection_BVH, C_LForeArm_LHand);

  /* 'BVHTransform_c:768' C_Spine3_RShoulder = EulerToC_c( RightShoulder,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &hb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[81], rotateOrder_BVH,
             rotateDirection_BVH, C_Spine3_RShoulder);

  /* 'BVHTransform_c:769' C_RShoulder_RArm = EulerToC_c( RightArm,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &ib_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[87], rotateOrder_BVH,
             rotateDirection_BVH, C_RShoulder_RArm);

  /* 'BVHTransform_c:770' C_RArm_RForeArm = EulerToC_c( RightForeArm,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &jb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[93], rotateOrder_BVH,
             rotateDirection_BVH, C_RArm_RForeArm);

  /* 'BVHTransform_c:771' C_RForeArm_RHand = EulerToC_c( RightHand,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &kb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[99], rotateOrder_BVH,
             rotateDirection_BVH, C_RForeArm_RHand);

  /* 'BVHTransform_c:772' C_Hip_RUpLeg = EulerToC_c( RightUpLeg,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &lb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[9], rotateOrder_BVH,
             rotateDirection_BVH, C_Hip_RUpLeg);

  /* 'BVHTransform_c:773' C_RUpLeg_RLeg = EulerToC_c( RightLeg,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &mb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[15], rotateOrder_BVH,
             rotateDirection_BVH, C_RUpLeg_RLeg);

  /* 'BVHTransform_c:774' C_RLeg_RFoot = EulerToC_c( RightFoot,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &nb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[21], rotateOrder_BVH,
             rotateDirection_BVH, C_RLeg_RFoot);

  /* 'BVHTransform_c:775' C_Hip_LUpLeg = EulerToC_c( LeftUpLeg,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &ob_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[27], rotateOrder_BVH,
             rotateDirection_BVH, C_Hip_LUpLeg);

  /* 'BVHTransform_c:776' C_LUpLeg_LLeg = EulerToC_c( LeftLeg,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &pb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[33], rotateOrder_BVH,
             rotateDirection_BVH, C_LUpLeg_LLeg);

  /* 'BVHTransform_c:777' C_LLeg_LFoot = EulerToC_c( LeftFoot,rotateOrder_BVH,rotateDirection_BVH ) ; */
  st.site = &qb_emlrtRSI;
  EulerToC_c(&st, *(real_T (*)[3])&BVHData[39], rotateOrder_BVH,
             rotateDirection_BVH, C_LLeg_LFoot);
}

/*
 * function euler = CToEuler_c( C,rotateOrder,rotateDirection )
 */
static void CToEuler_c(const emlrtStack *sp, const real_T C[9], real_T
  rotateOrder, const real_T rotateDirection[3], real_T euler[3])
{
  real_T C_K[9];
  real_T dv21[3];
  int32_T i11;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

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
  /* 'BVHTransform_c:842' coder.inline('never'); */
  /* 'BVHTransform_c:844' if size(C,3) == 1 */
  /* 'BVHTransform_c:846' else */
  /* 'BVHTransform_c:847' Nframes = size( C,1 ); */
  /* 'BVHTransform_c:848' euler = zeros( Nframes,3 ); */
  /* 'BVHTransform_c:849' for k=1:Nframes */
  /* 'BVHTransform_c:850' C_K = permute( C(k,:,:),[2,3,1] ); */
  memcpy(&C_K[0], &C[0], 9U * sizeof(real_T));

  /* 'BVHTransform_c:851' euler(k,:) = OneCToEuler_c( C_K,rotateOrder,rotateDirection ); */
  st.site = &sb_emlrtRSI;
  OneCToEuler_c(&st, C_K, rotateOrder, rotateDirection, dv21);
  for (i11 = 0; i11 < 3; i11++) {
    euler[i11] = dv21[i11];
  }
}

/*
 * function C13 = CalculateC_Multi( C12,C23 )
 */
static void CalculateC_Multi(const real_T C12[9], const real_T C23[9], real_T
  C13[9])
{
  real_T C12_k[9];
  real_T C23_k[9];
  int32_T k;
  real_T b_C23_k[9];
  int32_T i7;
  int32_T i8;

  /* 'BVHTransform_c:260' coder.inline('never'); */
  /* 'BVHTransform_c:262' C13 = coder.nullcopy(zeros(size(C12))); */
  /* 'BVHTransform_c:264' if size(C12,3)==1 */
  /* 'BVHTransform_c:269' Nframes = size( C12,1 ); */
  /* 'BVHTransform_c:270' C13 = zeros(Nframes,3,3); */
  /* 'BVHTransform_c:271' for k=1:Nframes */
  /* 'BVHTransform_c:272' C23_k = permute( C23(k,:,:),[2,3,1] ); */
  /* 'BVHTransform_c:273' C12_k = permute( C12(k,:,:),[2,3,1] ); */
  for (k = 0; k < 9; k++) {
    C23_k[k] = C23[k];
    C12_k[k] = C12[k];
  }

  /* 'BVHTransform_c:274' C13(k,:,:) = C23_k*C12_k ; */
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

/*
 * function [ C_Hip_LLeg,C_Spine2_LLeg,C_Spine3_LLeg ] = CalculatedLLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_LUpLeg,C_LUpLeg_LLeg )
 */
static void CalculatedLLegRotate(const real_T C_Hip_Spine2[9], const real_T
  C_Hip_Spine3[9], const real_T C_Hip_LUpLeg[9], const real_T C_LUpLeg_LLeg[9],
  real_T C_Hip_LLeg[9], real_T C_Spine2_LLeg[9], real_T C_Spine3_LLeg[9])
{
  real_T dv17[9];
  real_T dv18[9];

  /* 'BVHTransform_c:242' coder.inline('never'); */
  /* 'BVHTransform_c:244' C_Hip_LLeg = CalculateC_Multi( C_Hip_LUpLeg,C_LUpLeg_LLeg ); */
  CalculateC_Multi(C_Hip_LUpLeg, C_LUpLeg_LLeg, C_Hip_LLeg);

  /* 'BVHTransform_c:245' C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2); */
  /* 'BVHTransform_c:246' C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3); */
  /* 'BVHTransform_c:247' C_Spine2_LLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_LLeg ); */
  InverseC_Multi(C_Hip_Spine2, dv17);
  CalculateC_Multi(dv17, C_Hip_LLeg, C_Spine2_LLeg);

  /* 'BVHTransform_c:248' C_Spine3_LLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_LLeg ); */
  InverseC_Multi(C_Hip_Spine3, dv18);
  CalculateC_Multi(dv18, C_Hip_LLeg, C_Spine3_LLeg);
}

/*
 * function [ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
 *     CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand )
 */
static void CalculatedLeftArmRotate(const real_T C_Hip_Spine[9], const real_T
  C_Spine_Spine1[9], const real_T C_Spine1_Spine2[9], const real_T
  C_Spine2_Spine3[9], const real_T C_Spine3_LShoulder[9], const real_T
  C_LShoulder_LArm[9], const real_T C_LArm_LForeArm[9], const real_T
  C_LForeArm_LHand[9], real_T C_Hip_Spine2[9], real_T C_Hip_Spine3[9], real_T
  C_Spine_Spine3[9], real_T C_Hip_LArm[9], real_T C_Hip_LForeArm[9], real_T
  C_Hip_LHand[9], real_T C_Spine_LArm[9], real_T C_Spine_LForeArm[9], real_T
  C_Spine_LHand[9], real_T C_Spine2_LArm[9], real_T C_Spine2_LForeArm[9], real_T
  C_Spine2_LHand[9], real_T C_Spine3_LArm[9], real_T C_Spine3_LForeArm[9],
  real_T C_Spine3_LHand[9])
{
  real_T C_Spine_Spine2[9];

  /* % Analyse Left Arm BVH */
  /* 'BVHTransform_c:199' coder.inline('never'); */
  /* 'BVHTransform_c:201' C_Spine_Spine2 = CalculateC_Multi( C_Spine_Spine1,C_Spine1_Spine2 ); */
  CalculateC_Multi(C_Spine_Spine1, C_Spine1_Spine2, C_Spine_Spine2);

  /* 'BVHTransform_c:202' C_Spine_Spine3 = CalculateC_Multi( C_Spine_Spine2,C_Spine2_Spine3 ); */
  CalculateC_Multi(C_Spine_Spine2, C_Spine2_Spine3, C_Spine_Spine3);

  /* 'BVHTransform_c:203' C_Hip_Spine2 = CalculateC_Multi( C_Hip_Spine,C_Spine_Spine2 ); */
  CalculateC_Multi(C_Hip_Spine, C_Spine_Spine2, C_Hip_Spine2);

  /* 'BVHTransform_c:204' C_Hip_Spine3 = CalculateC_Multi( C_Hip_Spine,C_Spine_Spine3 ); */
  CalculateC_Multi(C_Hip_Spine, C_Spine_Spine3, C_Hip_Spine3);

  /* 'BVHTransform_c:206' C_Spine3_LArm = CalculateC_Multi( C_Spine3_LShoulder,C_LShoulder_LArm ); */
  CalculateC_Multi(C_Spine3_LShoulder, C_LShoulder_LArm, C_Spine3_LArm);

  /* 'BVHTransform_c:207' C_Spine2_LArm = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_LArm ); */
  CalculateC_Multi(C_Spine2_Spine3, C_Spine3_LArm, C_Spine2_LArm);

  /* 'BVHTransform_c:208' C_Spine_LArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_LArm ); */
  CalculateC_Multi(C_Spine_Spine3, C_Spine3_LArm, C_Spine_LArm);

  /* 'BVHTransform_c:209' C_Hip_LArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_LArm ); */
  CalculateC_Multi(C_Hip_Spine3, C_Spine3_LArm, C_Hip_LArm);

  /* 'BVHTransform_c:211' C_Spine3_LForeArm = CalculateC_Multi( C_Spine3_LArm,C_LArm_LForeArm ); */
  CalculateC_Multi(C_Spine3_LArm, C_LArm_LForeArm, C_Spine3_LForeArm);

  /* 'BVHTransform_c:212' C_Spine2_LForeArm = CalculateC_Multi( C_Spine2_LArm,C_LArm_LForeArm ); */
  CalculateC_Multi(C_Spine2_LArm, C_LArm_LForeArm, C_Spine2_LForeArm);

  /* 'BVHTransform_c:213' C_Spine_LForeArm = CalculateC_Multi( C_Spine_LArm,C_LArm_LForeArm ); */
  CalculateC_Multi(C_Spine_LArm, C_LArm_LForeArm, C_Spine_LForeArm);

  /* 'BVHTransform_c:214' C_Hip_LForeArm = CalculateC_Multi( C_Hip_LArm,C_LArm_LForeArm ); */
  CalculateC_Multi(C_Hip_LArm, C_LArm_LForeArm, C_Hip_LForeArm);

  /* 'BVHTransform_c:216' C_Spine3_LHand = CalculateC_Multi( C_Spine3_LForeArm,C_LForeArm_LHand ); */
  CalculateC_Multi(C_Spine3_LForeArm, C_LForeArm_LHand, C_Spine3_LHand);

  /* 'BVHTransform_c:217' C_Spine2_LHand = CalculateC_Multi( C_Spine2_LForeArm,C_LForeArm_LHand ); */
  CalculateC_Multi(C_Spine2_LForeArm, C_LForeArm_LHand, C_Spine2_LHand);

  /* 'BVHTransform_c:218' C_Spine_LHand = CalculateC_Multi( C_Spine_LForeArm,C_LForeArm_LHand ); */
  CalculateC_Multi(C_Spine_LForeArm, C_LForeArm_LHand, C_Spine_LHand);

  /* 'BVHTransform_c:219' C_Hip_LHand = CalculateC_Multi( C_Hip_LForeArm,C_LForeArm_LHand ); */
  CalculateC_Multi(C_Hip_LForeArm, C_LForeArm_LHand, C_Hip_LHand);
}

/*
 * function [ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg )
 */
static void CalculatedRLegRotate(const real_T C_Hip_Spine2[9], const real_T
  C_Hip_Spine3[9], const real_T C_Hip_RUpLeg[9], const real_T C_RUpLeg_RLeg[9],
  real_T C_Hip_RLeg[9], real_T C_Spine2_RLeg[9], real_T C_Spine3_RLeg[9])
{
  real_T dv19[9];
  real_T dv20[9];

  /* 'BVHTransform_c:251' coder.inline('never'); */
  /* 'BVHTransform_c:253' C_Hip_RLeg = CalculateC_Multi( C_Hip_RUpLeg,C_RUpLeg_RLeg ); */
  CalculateC_Multi(C_Hip_RUpLeg, C_RUpLeg_RLeg, C_Hip_RLeg);

  /* 'BVHTransform_c:254' C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2); */
  /* 'BVHTransform_c:255' C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3); */
  /* 'BVHTransform_c:256' C_Spine2_RLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_RLeg ); */
  InverseC_Multi(C_Hip_Spine2, dv19);
  CalculateC_Multi(dv19, C_Hip_RLeg, C_Spine2_RLeg);

  /* 'BVHTransform_c:257' C_Spine3_RLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_RLeg ); */
  InverseC_Multi(C_Hip_Spine3, dv20);
  CalculateC_Multi(dv20, C_Hip_RLeg, C_Spine3_RLeg);
}

/*
 * function [ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine2_RArm,C_Spine2_RForeArm,C_Spine2_RHand,C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
 *     CalculatedRightArmRotate( C_Hip_Spine3,C_Spine2_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand )
 */
static void CalculatedRightArmRotate(const real_T C_Hip_Spine3[9], const real_T
  C_Spine2_Spine3[9], const real_T C_Spine_Spine3[9], const real_T
  C_Spine3_RShoulder[9], const real_T C_RShoulder_RArm[9], const real_T
  C_RArm_RForeArm[9], const real_T C_RForeArm_RHand[9], real_T C_Hip_RArm[9],
  real_T C_Hip_RForeArm[9], real_T C_Hip_RHand[9], real_T C_Spine_RArm[9],
  real_T C_Spine_RForeArm[9], real_T C_Spine_RHand[9], real_T C_Spine2_RArm[9],
  real_T C_Spine2_RForeArm[9], real_T C_Spine2_RHand[9], real_T C_Spine3_RArm[9],
  real_T C_Spine3_RForeArm[9], real_T C_Spine3_RHand[9])
{
  /* % Analyse Right Arm BVH */
  /* 'BVHTransform_c:224' coder.inline('never'); */
  /* 'BVHTransform_c:226' C_Spine3_RArm = CalculateC_Multi( C_Spine3_RShoulder,C_RShoulder_RArm ); */
  CalculateC_Multi(C_Spine3_RShoulder, C_RShoulder_RArm, C_Spine3_RArm);

  /* 'BVHTransform_c:227' C_Spine2_RArm = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_RArm ); */
  CalculateC_Multi(C_Spine2_Spine3, C_Spine3_RArm, C_Spine2_RArm);

  /* 'BVHTransform_c:228' C_Spine_RArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_RArm ); */
  CalculateC_Multi(C_Spine_Spine3, C_Spine3_RArm, C_Spine_RArm);

  /* 'BVHTransform_c:229' C_Hip_RArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_RArm ); */
  CalculateC_Multi(C_Hip_Spine3, C_Spine3_RArm, C_Hip_RArm);

  /* 'BVHTransform_c:231' C_Spine3_RForeArm = CalculateC_Multi( C_Spine3_RArm,C_RArm_RForeArm ); */
  CalculateC_Multi(C_Spine3_RArm, C_RArm_RForeArm, C_Spine3_RForeArm);

  /* 'BVHTransform_c:232' C_Spine2_RForeArm = CalculateC_Multi( C_Spine2_RArm,C_RArm_RForeArm ); */
  CalculateC_Multi(C_Spine2_RArm, C_RArm_RForeArm, C_Spine2_RForeArm);

  /* 'BVHTransform_c:233' C_Spine_RForeArm = CalculateC_Multi( C_Spine_RArm,C_RArm_RForeArm ); */
  CalculateC_Multi(C_Spine_RArm, C_RArm_RForeArm, C_Spine_RForeArm);

  /* 'BVHTransform_c:234' C_Hip_RForeArm = CalculateC_Multi( C_Hip_RArm,C_RArm_RForeArm ); */
  CalculateC_Multi(C_Hip_RArm, C_RArm_RForeArm, C_Hip_RForeArm);

  /* 'BVHTransform_c:236' C_Spine3_RHand  = CalculateC_Multi( C_Spine3_RForeArm,C_RForeArm_RHand ); */
  CalculateC_Multi(C_Spine3_RForeArm, C_RForeArm_RHand, C_Spine3_RHand);

  /* 'BVHTransform_c:237' C_Spine2_RHand  = CalculateC_Multi( C_Spine2_RForeArm,C_RForeArm_RHand ); */
  CalculateC_Multi(C_Spine2_RForeArm, C_RForeArm_RHand, C_Spine2_RHand);

  /* 'BVHTransform_c:238' C_Spine_RHand  = CalculateC_Multi( C_Spine_RForeArm,C_RForeArm_RHand ); */
  CalculateC_Multi(C_Spine_RForeArm, C_RForeArm_RHand, C_Spine_RHand);

  /* 'BVHTransform_c:239' C_Hip_RHand  = CalculateC_Multi(C_Hip_RForeArm ,C_RForeArm_RHand ); */
  CalculateC_Multi(C_Hip_RForeArm, C_RForeArm_RHand, C_Hip_RHand);
}

/*
 * function euler2 = ChangeEulerRotate_c( euler1,rotateOrder1,rotateOrder2,rotateDirection1,rotateDirection2 )
 */
static void ChangeEulerRotate_c(const emlrtStack *sp, const real_T euler1[3],
  real_T rotateOrder1, real_T rotateOrder2, const real_T rotateDirection1[3],
  const real_T rotateDirection2[3], real_T euler2[3])
{
  real_T C[9];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % Change Euler Rotate Order */
  /* 'BVHTransform_c:899' coder.inline('never'); */
  /* 'BVHTransform_c:900' C = EulerToC_c( euler1,rotateOrder1,rotateDirection1 ); */
  st.site = &kc_emlrtRSI;
  EulerToC_c(&st, euler1, rotateOrder1, rotateDirection1, C);

  /* 'BVHTransform_c:901' euler2 = CToEuler_c( C,rotateOrder2,rotateDirection2 ) ; */
  st.site = &lc_emlrtRSI;
  CToEuler_c(&st, C, rotateOrder2, rotateDirection2, euler2);
}

/*
 * function C =  EulerToC_c( euler,rotateOrder,rotateDirection )
 */
static void EulerToC_c(const emlrtStack *sp, const real_T euler[3], real_T
  rotateOrder, const real_T rotateDirection[3], real_T C[9])
{
  real_T b_euler[3];
  int32_T i1;
  real_T dv12[9];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % xyz 2015.4.5 */
  /*   EulerToC_Multi  新版 ： 可转化为 C++ */
  /*    euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z]  */
  /*  store order of euler angle: the rotate order */
  /*  positive:anticlockwise rotate  */
  /*  euler angle scope: [-pi,pi] */
  /*  unit:rad  */
  /*  rotateDirection: anticlockwise(1) or clockwise(-1)  111 */
  /* 'BVHTransform_c:793' coder.inline('never'); */
  /* 'BVHTransform_c:795' Nframes = size( euler,1 ); */
  /* 'BVHTransform_c:796' C = zeros( Nframes,3,3 ); */
  /* 'BVHTransform_c:797' for k=1:Nframes */
  /* 'BVHTransform_c:798' C( k,:,: )  = OneEulerToC_c( euler(k,:),rotateOrder,rotateDirection ); */
  for (i1 = 0; i1 < 3; i1++) {
    b_euler[i1] = euler[i1];
  }

  st.site = &rb_emlrtRSI;
  OneEulerToC_c(&st, b_euler, rotateOrder, rotateDirection, dv12);
  memcpy(&C[0], &dv12[0], 9U * sizeof(real_T));

  /* 'BVHTransform_c:800' if Nframes==1 */
  /* 'BVHTransform_c:801' C = C( 1,:,: ) ; */
}

/*
 * function Cnb = FQtoCnb(Q)
 */
static void FQtoCnb(real_T Q[4], real_T Cnb[9])
{
  real_T y;
  real_T scale;
  int32_T k;
  real_T absxk;
  real_T t;

  /* % 四元数 -> 方向余弦矩阵 */
  /* 'BVHTransform_c:932' coder.inline('never'); */
  /* 'BVHTransform_c:934' Q=Q/norm(Q); */
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = muDoubleScalarAbs(Q[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * muDoubleScalarSqrt(y);
  for (k = 0; k < 4; k++) {
    Q[k] /= y;
  }

  /* 'BVHTransform_c:935' q0 = Q(1) ; */
  /* 'BVHTransform_c:936' q1 = Q(2) ; */
  /* 'BVHTransform_c:937' q2 = Q(3) ; */
  /* 'BVHTransform_c:938' q3 = Q(4) ; */
  /* 'BVHTransform_c:939' Cnb=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2); */
  /* 'BVHTransform_c:940'         2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1); */
  /* 'BVHTransform_c:941'         2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ]; */
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
 * function C21 = InverseC_Multi( C12 )
 */
static void InverseC_Multi(const real_T C12[9], real_T C21[9])
{
  real_T C12_k[9];
  int32_T i9;
  int32_T i10;

  /* 'BVHTransform_c:278' coder.inline('never'); */
  /* 'BVHTransform_c:280' if size(C12,3)==1 */
  /* 'BVHTransform_c:285' Nframes = size( C12,1 ); */
  /* 'BVHTransform_c:286' C21 = zeros(Nframes,3,3); */
  /* 'BVHTransform_c:287' for k=1:Nframes */
  /* 'BVHTransform_c:288' C12_k = permute( C12(k,:,:),[2,3,1] ); */
  memcpy(&C12_k[0], &C12[0], 9U * sizeof(real_T));

  /* 'BVHTransform_c:289' C21(k,:,:) = C12_k' ; */
  for (i9 = 0; i9 < 3; i9++) {
    for (i10 = 0; i10 < 3; i10++) {
      C21[i10 + 3 * i9] = C12_k[i9 + 3 * i10];
    }
  }
}

/*
 * function euler = OneCToEuler_c( C,rotateOrder,rotateDirection )
 */
static void OneCToEuler_c(const emlrtStack *sp, const real_T C[9], real_T
  rotateOrder, const real_T rotateDirection[3], real_T euler[3])
{
  real_T euler1;
  real_T euler2;
  real_T euler3;
  int32_T i;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* 'BVHTransform_c:857' coder.inline('never'); */
  /* 'BVHTransform_c:858' euler1 = 0; */
  euler1 = 0.0;

  /* 'BVHTransform_c:859' euler2 = 0; */
  euler2 = 0.0;

  /* 'BVHTransform_c:860' euler3 = 0 ; */
  euler3 = 0.0;

  /* 'BVHTransform_c:861' switch rotateOrder */
  switch ((int32_T)emlrtIntegerCheckFastR2012b(rotateOrder, (emlrtDCInfo *)
           &emlrtDCI, sp)) {
   case 321:
    /* 'BVHTransform_c:862' case 321  */
    /* 'BVHTransform_c:863' euler1 = atan2( C(1,2),C(1,1) ) ; */
    euler1 = muDoubleScalarAtan2(C[3], C[0]);

    /* 'BVHTransform_c:864' euler2 = asin( -C(1,3) ) ; */
    st.site = &yb_emlrtRSI;
    if ((-C[6] < -1.0) || (1.0 < -C[6])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(-C[6]);

    /*  euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ; */
    /* 'BVHTransform_c:866' euler3 = atan2( C(2,3),C(3,3) ) ; */
    euler3 = muDoubleScalarAtan2(C[7], C[8]);
    break;

   case 312:
    /* 'BVHTransform_c:867' case 312 */
    /* 'BVHTransform_c:868' euler1 = atan2( -C(2,1),C(2,2) ) ; */
    euler1 = muDoubleScalarAtan2(-C[1], C[4]);

    /* 'BVHTransform_c:869' euler2 = asin( C(2,3) ) ; */
    st.site = &xb_emlrtRSI;
    if ((C[7] < -1.0) || (1.0 < C[7])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(C[7]);

    /*  euler2 = atan2( C(2,3),sqrt( C(2,1)^2+C(2,2)^2 ) ) ; */
    /* 'BVHTransform_c:871' euler3 = atan2( -C(1,3),C(3,3) ) ; */
    euler3 = muDoubleScalarAtan2(-C[6], C[8]);
    break;

   case 123:
    /* 'BVHTransform_c:872' case 123 */
    /* 'BVHTransform_c:873' euler1 = atan2( -C(3,2),C(3,3) ) ; */
    euler1 = muDoubleScalarAtan2(-C[5], C[8]);

    /* 'BVHTransform_c:874' euler2 = asin(C(3,1)) ; */
    st.site = &wb_emlrtRSI;
    if ((C[2] < -1.0) || (1.0 < C[2])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(C[2]);

    /* 'BVHTransform_c:875' euler3 = atan2( -C(2,1),C(1,1) ) ; */
    euler3 = muDoubleScalarAtan2(-C[1], C[0]);
    break;

   case 132:
    /* 'BVHTransform_c:876' case 132 */
    /* 'BVHTransform_c:877' euler1 =  atan2(C(2,3),C(2,2)) ; */
    euler1 = muDoubleScalarAtan2(C[7], C[4]);

    /* 'BVHTransform_c:878' euler2 =  asin(-C(2,1)) ; */
    st.site = &vb_emlrtRSI;
    if ((-C[1] < -1.0) || (1.0 < -C[1])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(-C[1]);

    /* 'BVHTransform_c:879' euler3 =  atan2(C(3,1),C(1,1)) ; */
    euler3 = muDoubleScalarAtan2(C[2], C[0]);
    break;

   case 213:
    /* 'BVHTransform_c:880' case 213 */
    /* 'BVHTransform_c:881' euler1 = atan2( C(3,1),C(3,3) ) ; */
    euler1 = muDoubleScalarAtan2(C[2], C[8]);

    /* 'BVHTransform_c:882' euler2 = asin( -C(3,2) ) ; */
    st.site = &ub_emlrtRSI;
    if ((-C[5] < -1.0) || (1.0 < -C[5])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(-C[5]);

    /*  euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ; */
    /* 'BVHTransform_c:884' euler3 = atan2( C(1,2),C(2,2) ) ; */
    euler3 = muDoubleScalarAtan2(C[3], C[4]);
    break;

   case 231:
    /* 'BVHTransform_c:885' case 231  */
    /* 'BVHTransform_c:886' euler1 = atan2(-C(1,3),C(1,1))  ; */
    euler1 = muDoubleScalarAtan2(-C[6], C[0]);

    /* 'BVHTransform_c:887' euler2 = asin(C(1,2)) ; */
    st.site = &tb_emlrtRSI;
    if ((C[3] < -1.0) || (1.0 < C[3])) {
      b_st.site = &ac_emlrtRSI;
      eml_error(&b_st);
    }

    euler2 = muDoubleScalarAsin(C[3]);

    /* 'BVHTransform_c:888' euler3 = atan2(-C(3,2),C(2,2)) ; */
    euler3 = muDoubleScalarAtan2(-C[5], C[4]);
    break;
  }

  /* 'BVHTransform_c:890' euler = [ euler1; euler2; euler3 ]; */
  euler[0] = euler1;
  euler[1] = euler2;
  euler[2] = euler3;

  /* 'BVHTransform_c:892' for i=1:3 */
  for (i = 0; i < 3; i++) {
    /* 'BVHTransform_c:893' euler(i) = euler(i)*rotateDirection(i) ; */
    emlrtBreakCheckFastR2012b((const volatile char_T *)
      emlrtBreakCheckR2012bFlagVar, sp);
    euler[i] *= rotateDirection[i];
  }
}

/*
 * function C =  OneEulerToC_c( euler,rotateOrder,rotateDirection )
 */
static void OneEulerToC_c(const emlrtStack *sp, real_T euler[3], real_T
  rotateOrder, const real_T rotateDirection[3], real_T C[9])
{
  int32_T i;
  real_T dv13[9];
  real_T dv14[9];
  real_T dv15[9];
  real_T dv16[9];
  int32_T i2;
  int32_T i3;

  /* 'BVHTransform_c:806' coder.inline('never'); */
  /* 'BVHTransform_c:808' for i=1:3 */
  for (i = 0; i < 3; i++) {
    /* 'BVHTransform_c:809' euler(i) = euler(i)*rotateDirection(i) ; */
    emlrtBreakCheckFastR2012b((const volatile char_T *)
      emlrtBreakCheckR2012bFlagVar, sp);
    euler[i] *= rotateDirection[i];
  }

  /* 'BVHTransform_c:811' C = zeros(3,3); */
  memset(&C[0], 0, 9U * sizeof(real_T));

  /* 'BVHTransform_c:813' switch rotateOrder */
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
    /* 'BVHTransform_c:814' case 321  */
    /* 'BVHTransform_c:815' C = RotateX(euler(3)) * RotateY(euler(2)) * RotateZ(euler(1)) ; */
    RotateX(euler[2], dv13);
    RotateY(euler[1], dv14);
    RotateZ(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;

   case 1:
    /* 'BVHTransform_c:816' case 312 */
    /* 'BVHTransform_c:817' C = RotateY(euler(3)) * RotateX(euler(2)) * RotateZ(euler(1)) ; */
    RotateY(euler[2], dv13);
    RotateX(euler[1], dv14);
    RotateZ(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;

   case 2:
    /* 'BVHTransform_c:818' case 123 */
    /* 'BVHTransform_c:819' C = RotateZ(euler(3)) * RotateY(euler(2)) * RotateX(euler(1)) ; */
    RotateZ(euler[2], dv13);
    RotateY(euler[1], dv14);
    RotateX(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;

   case 3:
    /* 'BVHTransform_c:820' case 132 */
    /* 'BVHTransform_c:821' C = RotateY(euler(3)) * RotateZ(euler(2)) * RotateX(euler(1)) ; */
    RotateY(euler[2], dv13);
    RotateZ(euler[1], dv14);
    RotateX(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;

   case 4:
    /* 'BVHTransform_c:822' case 213 */
    /* 'BVHTransform_c:823' C = RotateZ(euler(3)) * RotateX(euler(2)) * RotateY(euler(1)) ; */
    RotateZ(euler[2], dv13);
    RotateX(euler[1], dv14);
    RotateY(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;

   case 5:
    /* 'BVHTransform_c:824' case 'YZX' */
    /* 'BVHTransform_c:825' C = RotateX(euler(3)) * RotateZ(euler(2)) * RotateY(euler(1)) ; */
    RotateX(euler[2], dv13);
    RotateZ(euler[1], dv14);
    RotateY(euler[0], dv15);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv16[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          dv16[i + 3 * i2] += dv13[i + 3 * i3] * dv14[i3 + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        C[i + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          C[i + 3 * i2] += dv16[i + 3 * i3] * dv15[i3 + 3 * i2];
        }
      }
    }
    break;
  }
}

/*
 * function Cx = RotateX( angX )
 */
static void RotateX(real_T angX, real_T Cx[9])
{
  int32_T i4;
  static const int8_T iv0[3] = { 1, 0, 0 };

  /* % anticlockwise = positive */
  /* 'BVHTransform_c:905' coder.inline('never'); */
  /* 'BVHTransform_c:906' Cx = [ */
  /* 'BVHTransform_c:907'     1       0           0 */
  /* 'BVHTransform_c:908'     0       cos(angX)   sin(angX) */
  /* 'BVHTransform_c:909'     0       -sin(angX)  cos(angX) */
  /* 'BVHTransform_c:910' ]; */
  for (i4 = 0; i4 < 3; i4++) {
    Cx[3 * i4] = iv0[i4];
  }

  Cx[1] = 0.0;
  Cx[4] = muDoubleScalarCos(angX);
  Cx[7] = muDoubleScalarSin(angX);
  Cx[2] = 0.0;
  Cx[5] = -muDoubleScalarSin(angX);
  Cx[8] = muDoubleScalarCos(angX);
}

/*
 * function Cy = RotateY( angY )
 */
static void RotateY(real_T angY, real_T Cy[9])
{
  int32_T i6;
  static const int8_T iv2[3] = { 0, 1, 0 };

  /* % anticlockwise rotate is positive */
  /* 'BVHTransform_c:914' coder.inline('never'); */
  /* 'BVHTransform_c:915' Cy = [ */
  /* 'BVHTransform_c:916'     cos(angY)       0       -sin(angY) */
  /* 'BVHTransform_c:917'     0               1       0 */
  /* 'BVHTransform_c:918'     sin(angY)       0       cos(angY) */
  /* 'BVHTransform_c:919' ]; */
  Cy[0] = muDoubleScalarCos(angY);
  Cy[3] = 0.0;
  Cy[6] = -muDoubleScalarSin(angY);
  for (i6 = 0; i6 < 3; i6++) {
    Cy[1 + 3 * i6] = iv2[i6];
  }

  Cy[2] = muDoubleScalarSin(angY);
  Cy[5] = 0.0;
  Cy[8] = muDoubleScalarCos(angY);
}

/*
 * function Cz = RotateZ( angZ )
 */
static void RotateZ(real_T angZ, real_T Cz[9])
{
  int32_T i5;
  static const int8_T iv1[3] = { 0, 0, 1 };

  /* % anticlockwise = positive */
  /* 'BVHTransform_c:923' coder.inline('never'); */
  /* 'BVHTransform_c:924' Cz = [ */
  /* 'BVHTransform_c:925'     cos(angZ)       sin(angZ)       0 */
  /* 'BVHTransform_c:926'     -sin(angZ)      cos(angZ)       0 */
  /* 'BVHTransform_c:927'     0               0               1 */
  /* 'BVHTransform_c:928' ]; */
  Cz[0] = muDoubleScalarCos(angZ);
  Cz[3] = muDoubleScalarSin(angZ);
  Cz[6] = 0.0;
  Cz[1] = -muDoubleScalarSin(angZ);
  Cz[4] = muDoubleScalarCos(angZ);
  Cz[7] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    Cz[2 + 3 * i5] = iv1[i5];
  }
}

/*
 * function [ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_Spine_Head,C_Spine_NeckNew,rotateOrder_BVH )
 */
static void TransformHead(const emlrtStack *sp, const real_T C_Spine_Head[9],
  const real_T C_Spine_NeckNew[9], real_T rotateOrder_BVH, real_T *HeadYaw,
  real_T *HeadPitch, real_T Head_new[3])
{
  real_T C_Spine_HeadNew[9];
  real_T b_C_Spine_HeadNew[9];
  real_T dv22[3];
  int32_T i12;
  real_T dv23[9];
  real_T E_Spine_Head[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % TransformHead */
  /*  E_spine_head: euler from spine3 to head */
  /*  rotateOrder : rotate order of "E_spine_head" */
  /* 'BVHTransform_c:306' coder.inline('never'); */
  /* 'BVHTransform_c:308' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:309' Head_rotateOrder = 213; */
  /* 'BVHTransform_c:310' C_NeckNew_Spine = InverseC_Multi( C_Spine_NeckNew ); */
  /* 'BVHTransform_c:311' C_NeckNew_Head = CalculateC_Multi( C_NeckNew_Spine,C_Spine_Head ); */
  /* 'BVHTransform_c:312' E_Spine_Head = CToEuler_c( C_NeckNew_Head,Head_rotateOrder,rotateDirection_BVH ); */
  InverseC_Multi(C_Spine_NeckNew, C_Spine_HeadNew);
  memcpy(&b_C_Spine_HeadNew[0], &C_Spine_HeadNew[0], 9U * sizeof(real_T));
  for (i12 = 0; i12 < 3; i12++) {
    dv22[i12] = 1.0;
  }

  CalculateC_Multi(b_C_Spine_HeadNew, C_Spine_Head, dv23);
  st.site = &bc_emlrtRSI;
  CToEuler_c(&st, dv23, 213.0, dv22, E_Spine_Head);

  /* 'BVHTransform_c:313' E_Spine_Head(:,3) = 0; */
  E_Spine_Head[2] = 0.0;

  /* 'BVHTransform_c:314' HeadYaw = E_Spine_Head(:,1)' ; */
  *HeadYaw = E_Spine_Head[0];

  /* 'BVHTransform_c:315' HeadPitch = E_Spine_Head(:,2)' ; */
  *HeadPitch = E_Spine_Head[1];

  /* %% get new BVH */
  /* 'BVHTransform_c:318' C_Spine_HeadNew = EulerToC_c( E_Spine_Head,Head_rotateOrder ,[1,1,1] ) ; */
  for (i12 = 0; i12 < 3; i12++) {
    dv22[i12] = 1.0;
  }

  st.site = &cc_emlrtRSI;
  EulerToC_c(&st, E_Spine_Head, 213.0, dv22, C_Spine_HeadNew);

  /* 'BVHTransform_c:319' Head_new = CToEuler_c( C_Spine_HeadNew,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i12 = 0; i12 < 3; i12++) {
    dv22[i12] = 1.0;
  }

  st.site = &dc_emlrtRSI;
  CToEuler_c(&st, C_Spine_HeadNew, rotateOrder_BVH, dv22, Head_new);
}

/*
 * function [ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_Spine_LForeArm,C_Spine_LArmNew,rotateOrder_BVH )
 */
static void TransformLElbow(const emlrtStack *sp, const real_T C_Spine_LForeArm
  [9], const real_T C_Spine_LArmNew[9], real_T rotateOrder_BVH, real_T
  LElbow_Nao[3], real_T LeftForeArm_new[3])
{
  real_T C_LArmNew_LForeArm[9];
  real_T b_C_LArmNew_LForeArm[9];
  real_T dv40[3];
  real_T dv41[3];
  int32_T i17;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /*  %% LElbow */
  /* 'BVHTransform_c:418' coder.inline('never'); */
  /* 'BVHTransform_c:420' C_LArmNew_Spine = InverseC_Multi( C_Spine_LArmNew ); */
  /* 'BVHTransform_c:421' C_LArmNew_LForeArm = CalculateC_Multi( C_LArmNew_Spine,C_Spine_LForeArm ); */
  InverseC_Multi(C_Spine_LArmNew, C_LArmNew_LForeArm);
  memcpy(&b_C_LArmNew_LForeArm[0], &C_LArmNew_LForeArm[0], 9U * sizeof(real_T));
  CalculateC_Multi(b_C_LArmNew_LForeArm, C_Spine_LForeArm, C_LArmNew_LForeArm);

  /* 'BVHTransform_c:423' Nframes = size( C_Spine_LForeArm,1 ); */
  /* 'BVHTransform_c:424' LElbow_Nao = zeros(Nframes,3); */
  /* 'BVHTransform_c:425' for k=1:Nframes */
  /* 'BVHTransform_c:426' [x,y,z] = deal( C_LArmNew_LForeArm(k,1,1),C_LArmNew_LForeArm(k,1,2),C_LArmNew_LForeArm(k,1,3) ); */
  /*      %%% LElbowRoll > 0  % 人很少做的动作：人左手不会这么转 */
  /*      LElbowRoll1 = acos(x) ; */
  /*      LElbowYaw1 = atan2( y,-z ); */
  /*      LElbow_Nao1 = [ LElbowYaw1 LElbowRoll1 0 ]; */
  /* %% LElbowRoll < 0  % 人正常动作 */
  /* 'BVHTransform_c:432' LElbowRoll2 = -acos(x) ; */
  st.site = &rc_emlrtRSI;
  if ((C_LArmNew_LForeArm[0] < -1.0) || (1.0 < C_LArmNew_LForeArm[0])) {
    b_st.site = &jc_emlrtRSI;
    b_eml_error(&b_st);
  }

  /* 'BVHTransform_c:433' LElbowYaw2 = atan2( -y,z ); */
  /* 'BVHTransform_c:434' LElbow_Nao2 = [ LElbowYaw2 LElbowRoll2 0 ]; */
  LElbow_Nao[0] = muDoubleScalarAtan2(-C_LArmNew_LForeArm[3],
    C_LArmNew_LForeArm[6]);
  LElbow_Nao[1] = -muDoubleScalarAcos(C_LArmNew_LForeArm[0]);
  LElbow_Nao[2] = 0.0;

  /* 'BVHTransform_c:436' LElbow_Nao(k,:) = LElbow_Nao2 ; */
  /* %% get new BVH */
  /* 'BVHTransform_c:439' LElbow_rotateOrder = 123; */
  /* 'BVHTransform_c:440' LeftForeArm_new = zeros(Nframes,3); */
  /* 'BVHTransform_c:441' for k=1:Nframes */
  /* 'BVHTransform_c:442' LeftForeArm_new(k,:) = ChangeEulerRotate_c(  LElbow_Nao( k,: ),LElbow_rotateOrder,rotateOrder_BVH ,[1,1,1],[1,1,1] ) ; */
  for (i17 = 0; i17 < 3; i17++) {
    dv40[i17] = 1.0;
    dv41[i17] = 1.0;
  }

  st.site = &sc_emlrtRSI;
  ChangeEulerRotate_c(&st, LElbow_Nao, 123.0, rotateOrder_BVH, dv40, dv41,
                      LeftForeArm_new);
}

/*
 * function [ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH )
 */
static void TransformLFoot(const emlrtStack *sp, const real_T C_LLeg_LFoot[9],
  real_T rotateOrder_BVH, real_T *LAnklePitch, real_T *LAnkleRoll, real_T
  LeftFoot_new[3])
{
  real_T dv55[3];
  int32_T i24;
  real_T E_LLeg_LFoot[3];
  real_T C_RLeg_RFoot_New[9];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % LFoot */
  /* 'BVHTransform_c:669' coder.inline('never'); */
  /* 'BVHTransform_c:671' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:672' E_LLeg_LFoot = CToEuler_c( C_LLeg_LFoot,132,rotateDirection_BVH ); */
  for (i24 = 0; i24 < 3; i24++) {
    dv55[i24] = 1.0;
  }

  st.site = &td_emlrtRSI;
  CToEuler_c(&st, C_LLeg_LFoot, 132.0, dv55, E_LLeg_LFoot);

  /* 'BVHTransform_c:673' LAnklePitch = E_LLeg_LFoot( :,1 )'; */
  *LAnklePitch = E_LLeg_LFoot[0];

  /* 'BVHTransform_c:674' LAnkleRoll = E_LLeg_LFoot( :,2 )'; */
  *LAnkleRoll = E_LLeg_LFoot[1];

  /* %% get new BVH */
  /* 'BVHTransform_c:677' E_LLeg_LFoot(:,3) = 0; */
  E_LLeg_LFoot[2] = 0.0;

  /* 'BVHTransform_c:678' C_RLeg_RFoot_New = EulerToC_c( E_LLeg_LFoot,132,[1,1,1] ); */
  for (i24 = 0; i24 < 3; i24++) {
    dv55[i24] = 1.0;
  }

  st.site = &ud_emlrtRSI;
  EulerToC_c(&st, E_LLeg_LFoot, 132.0, dv55, C_RLeg_RFoot_New);

  /* 'BVHTransform_c:679' LeftFoot_new = CToEuler_c( C_RLeg_RFoot_New,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i24 = 0; i24 < 3; i24++) {
    dv55[i24] = 1.0;
  }

  st.site = &vd_emlrtRSI;
  CToEuler_c(&st, C_RLeg_RFoot_New, rotateOrder_BVH, dv55, LeftFoot_new);
}

/*
 * function [ LWristYaw,LeftHand_new ] = TransformLHand( C_Spine_LHand,C_Spine_LForeArmNew,rotateOrder_BVH )
 */
static void TransformLHand(const emlrtStack *sp, const real_T C_Spine_LHand[9],
  const real_T C_Spine_LForeArmNew[9], real_T rotateOrder_BVH, real_T *LWristYaw,
  real_T LeftHand_new[3])
{
  real_T dv42[9];
  real_T dv43[9];
  real_T dv44[3];
  int32_T i18;
  real_T dv45[9];
  real_T E_LForeArmNew_LHand[3];
  real_T LHand_Nao[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % LHand */
  /* 'BVHTransform_c:468' coder.inline('never'); */
  /* 'BVHTransform_c:470' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:471' C_LForeArmNew_Spine = InverseC_Multi( C_Spine_LForeArmNew ); */
  /* 'BVHTransform_c:472' C_LForeArmNew_LHand = CalculateC_Multi( C_LForeArmNew_Spine,C_Spine_LHand ); */
  /* 'BVHTransform_c:473' E_LForeArmNew_LHand = CToEuler_c( C_LForeArmNew_LHand,123 ,rotateDirection_BVH); */
  InverseC_Multi(C_Spine_LForeArmNew, dv42);
  memcpy(&dv43[0], &dv42[0], 9U * sizeof(real_T));
  for (i18 = 0; i18 < 3; i18++) {
    dv44[i18] = 1.0;
  }

  CalculateC_Multi(dv43, C_Spine_LHand, dv45);
  st.site = &tc_emlrtRSI;
  CToEuler_c(&st, dv45, 123.0, dv44, E_LForeArmNew_LHand);

  /* 'BVHTransform_c:474' LWristYaw = E_LForeArmNew_LHand(:,1)' ; */
  *LWristYaw = E_LForeArmNew_LHand[0];

  /* %% get new BVH */
  /* 'BVHTransform_c:477' LHand_Nao = zeros( size(E_LForeArmNew_LHand) ); */
  for (i18 = 0; i18 < 3; i18++) {
    LHand_Nao[i18] = 0.0;
  }

  /* 'BVHTransform_c:478' LHand_Nao(:,1) = E_LForeArmNew_LHand(:,1) ; */
  LHand_Nao[0] = E_LForeArmNew_LHand[0];

  /* 'BVHTransform_c:479' Nframes = size(C_Spine_LHand,1); */
  /* 'BVHTransform_c:480' LHand_rotateOrder = 123; */
  /* 'BVHTransform_c:481' LeftHand_new = zeros(Nframes,3); */
  /* 'BVHTransform_c:482' for k=1:Nframes */
  /* 'BVHTransform_c:483' LeftHand_new(k,:) = ChangeEulerRotate_c(  LHand_Nao( k,: ),LHand_rotateOrder,rotateOrder_BVH ,[1,1,1],[1,1,1] ) ; */
  for (i18 = 0; i18 < 3; i18++) {
    dv44[i18] = 1.0;
    E_LForeArmNew_LHand[i18] = 1.0;
  }

  st.site = &uc_emlrtRSI;
  ChangeEulerRotate_c(&st, LHand_Nao, 123.0, rotateOrder_BVH, dv44,
                      E_LForeArmNew_LHand, LeftHand_new);
}

/*
 * function [ LKneePitch,LeftLeg_New ] = TransformLLeg( C_Hip_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH )
 */
static void TransformLLeg(const emlrtStack *sp, const real_T C_Hip_LUpLegNew[9],
  const real_T C_Hip_LLeg[9], real_T rotateOrder_BVH, real_T *LKneePitch, real_T
  LeftLeg_New[3])
{
  real_T C_LUpLegNew_LLegNew[9];
  real_T b_C_LUpLegNew_LLegNew[9];
  real_T dv53[3];
  int32_T i23;
  real_T dv54[9];
  real_T E_LUpLegNew_LLeg[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % LLeg */
  /* 'BVHTransform_c:627' coder.inline('never'); */
  /* 'BVHTransform_c:629' C_LUpLegNew_Hip = InverseC_Multi( C_Hip_LUpLegNew ); */
  /* 'BVHTransform_c:630' C_LUpLegNew_LLeg = CalculateC_Multi( C_LUpLegNew_Hip,C_Hip_LLeg ); */
  /* 'BVHTransform_c:631' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:632' E_LUpLegNew_LLeg = CToEuler_c( C_LUpLegNew_LLeg,123,rotateDirection_BVH ); */
  InverseC_Multi(C_Hip_LUpLegNew, C_LUpLegNew_LLegNew);
  memcpy(&b_C_LUpLegNew_LLegNew[0], &C_LUpLegNew_LLegNew[0], 9U * sizeof(real_T));
  for (i23 = 0; i23 < 3; i23++) {
    dv53[i23] = 1.0;
  }

  CalculateC_Multi(b_C_LUpLegNew_LLegNew, C_Hip_LLeg, dv54);
  st.site = &qd_emlrtRSI;
  CToEuler_c(&st, dv54, 123.0, dv53, E_LUpLegNew_LLeg);

  /* 'BVHTransform_c:633' LKneePitch = E_LUpLegNew_LLeg(:,1)'; */
  *LKneePitch = E_LUpLegNew_LLeg[0];

  /*  figure('name','LeftLeg_Nao') */
  /*  subplot(3,1,1) */
  /*  plot(E_LUpLegNew_LLeg(:,1)*180/pi,'r'  ) */
  /*  ylabel('x') */
  /*  subplot(3,1,2) */
  /*  plot(E_LUpLegNew_LLeg(:,2)*180/pi,'b'  ) */
  /*  ylabel('y') */
  /*  subplot(3,1,3) */
  /*  plot(E_LUpLegNew_LLeg(:,3)*180/pi,'k'  ) */
  /*  ylabel('z') */
  /* %% get new BVH */
  /* 'BVHTransform_c:647' E_LUpLegNew_LLegNew = E_LUpLegNew_LLeg ; */
  /* 'BVHTransform_c:648' E_LUpLegNew_LLegNew(:,2:3) = 0; */
  for (i23 = 0; i23 < 2; i23++) {
    E_LUpLegNew_LLeg[1 + i23] = 0.0;
  }

  /* 'BVHTransform_c:649' C_LUpLegNew_LLegNew = EulerToC_c( E_LUpLegNew_LLegNew,123,[1,1,1] ); */
  for (i23 = 0; i23 < 3; i23++) {
    dv53[i23] = 1.0;
  }

  st.site = &rd_emlrtRSI;
  EulerToC_c(&st, E_LUpLegNew_LLeg, 123.0, dv53, C_LUpLegNew_LLegNew);

  /* 'BVHTransform_c:650' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:651' LeftLeg_New = CToEuler_c( C_LUpLegNew_LLegNew,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i23 = 0; i23 < 3; i23++) {
    dv53[i23] = 1.0;
  }

  st.site = &sd_emlrtRSI;
  CToEuler_c(&st, C_LUpLegNew_LLegNew, rotateOrder_BVH, dv53, LeftLeg_New);
}

/*
 * function [ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_Spine3_LShoulderNew,C_Spine3_LArm,rotateOrder_BVH )
 */
static void TransformLShoulder(const emlrtStack *sp, const real_T
  C_Spine3_LShoulderNew[9], const real_T C_Spine3_LArm[9], real_T
  rotateOrder_BVH, real_T *LShoulderPitch, real_T *LShoulderRoll, real_T
  LeftArm_new[3])
{
  real_T C_LShoulderNew_LArm_NaoNew[9];
  real_T b_C_LShoulderNew_LArm_NaoNew[9];
  real_T c_C_LShoulderNew_LArm_NaoNew[9];
  real_T dv35[9];
  static const real_T dv36[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  real_T E_LShoulderNew_LArm_Nao[3];
  static const real_T dv37[3] = { -1.0, 1.0, 1.0 };

  real_T d_C_LShoulderNew_LArm_NaoNew[9];
  int32_T i16;
  real_T dv38[9];
  static const real_T dv39[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % LeftShoulder */
  /* 'BVHTransform_c:354' coder.inline('never'); */
  /* 'BVHTransform_c:356' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:357' C_LShoulderNew_Spine3 = InverseC_Multi( C_Spine3_LShoulderNew ); */
  /* 'BVHTransform_c:358' C_LShoulderNew_LArm = CalculateC_Multi( C_LShoulderNew_Spine3,C_Spine3_LArm ); */
  /* %% 对 C_LShoulderNew_LArm 进行零位补偿 */
  /* 'BVHTransform_c:360' Nframes = size(C_Spine3_LShoulderNew,1); */
  /* 'BVHTransform_c:361' E_naoZero_NoitomZero_YXZ = [pi/2,0,0]; */
  /* 'BVHTransform_c:362' E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 ); */
  /* 'BVHTransform_c:363' C_naoZero_NoitomZero_YXZ = EulerToC_c( E_naoZero_NoitomZero_YXZ,213,[1,1,1] ); */
  /* 'BVHTransform_c:365' C_LShoulderNew_LArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_LShoulderNew_LArm ); */
  /* 'BVHTransform_c:366' E_LShoulderNew_LArm_Nao = CToEuler_c( C_LShoulderNew_LArm_Nao,321,[-1,1,1] ); */
  InverseC_Multi(C_Spine3_LShoulderNew, C_LShoulderNew_LArm_NaoNew);
  memcpy(&b_C_LShoulderNew_LArm_NaoNew[0], &C_LShoulderNew_LArm_NaoNew[0], 9U *
         sizeof(real_T));
  CalculateC_Multi(b_C_LShoulderNew_LArm_NaoNew, C_Spine3_LArm,
                   C_LShoulderNew_LArm_NaoNew);
  memcpy(&c_C_LShoulderNew_LArm_NaoNew[0], &C_LShoulderNew_LArm_NaoNew[0], 9U *
         sizeof(real_T));
  CalculateC_Multi(dv36, c_C_LShoulderNew_LArm_NaoNew, dv35);
  st.site = &oc_emlrtRSI;
  CToEuler_c(&st, dv35, 321.0, dv37, E_LShoulderNew_LArm_Nao);

  /* 'BVHTransform_c:367' E_LShoulderNew_LArm_Nao(:,3) = 0; */
  E_LShoulderNew_LArm_Nao[2] = 0.0;

  /* 'BVHTransform_c:368' C_LShoulderNew_LArm_NaoNew = EulerToC_c( E_LShoulderNew_LArm_Nao,321,[-1,1,1] ); */
  st.site = &pc_emlrtRSI;
  EulerToC_c(&st, E_LShoulderNew_LArm_Nao, 321.0, dv37,
             C_LShoulderNew_LArm_NaoNew);

  /* 'BVHTransform_c:370' LShoulderPitch = E_LShoulderNew_LArm_Nao(:,1)'; */
  *LShoulderPitch = E_LShoulderNew_LArm_Nao[0];

  /* 'BVHTransform_c:371' LShoulderRoll = E_LShoulderNew_LArm_Nao(:,2)'; */
  *LShoulderRoll = E_LShoulderNew_LArm_Nao[1];

  /* %% get new BVH */
  /*  零位补偿回去 */
  /* 'BVHTransform_c:375' E_NoitomZero_naoZero_ZYX = [0,-pi/2,0]; */
  /*  'ZYX' */
  /* 'BVHTransform_c:376' E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 ); */
  /* 'BVHTransform_c:377' C_NoitomZero_naoZero_ZYX = EulerToC_c( E_NoitomZero_naoZero_ZYX,321,[1,1,1] ); */
  /* 'BVHTransform_c:378' C_LShoulderNew_LArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_LShoulderNew_LArm_NaoNew ); */
  /* 'BVHTransform_c:380' LeftArm_new = CToEuler_c( C_LShoulderNew_LArm_NoitomNew,rotateOrder_BVH,rotateDirection_BVH ); */
  memcpy(&d_C_LShoulderNew_LArm_NaoNew[0], &C_LShoulderNew_LArm_NaoNew[0], 9U *
         sizeof(real_T));
  for (i16 = 0; i16 < 3; i16++) {
    E_LShoulderNew_LArm_Nao[i16] = 1.0;
  }

  CalculateC_Multi(dv39, d_C_LShoulderNew_LArm_NaoNew, dv38);
  st.site = &qc_emlrtRSI;
  CToEuler_c(&st, dv38, rotateOrder_BVH, E_LShoulderNew_LArm_Nao, LeftArm_new);
}

/*
 * function [ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Hip_LUpLeg,HipYawPitch,rotateOrder_BVH )
 */
static void TransformLUpLeg(const emlrtStack *sp, const real_T C_Hip_LUpLeg[9],
  real_T HipYawPitch, real_T rotateOrder_BVH, real_T *LHipRoll, real_T
  *LHipPitch, real_T LeftUpLeg_new[3])
{
  real_T C_LUpLeg_Hip[9];
  real_T x;
  real_T dv51[4];
  int32_T i22;
  static const real_T a[3] = { 0.70710678118654746, -0.70710678118654746, 0.0 };

  real_T unusedExpr[9];
  real_T C_LUpLeg_SecondHip_k[9];
  real_T b_LHipRoll[3];
  real_T dv52[3];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* % LUpLeg */
  /*  HipYawPitch：左右大腿的 HipYawPitch Angle  [ Nframes*1 ] */
  /* 'BVHTransform_c:553' C_LUpLeg_Hip = InverseC_Multi( C_Hip_LUpLeg ); */
  InverseC_Multi(C_Hip_LUpLeg, C_LUpLeg_Hip);

  /* 'BVHTransform_c:554' AxisL = [1,-1,0] ; */
  /* 'BVHTransform_c:555' AxisL = AxisL/normest(AxisL) ; */
  /* 'BVHTransform_c:556' Nframes = size(C_Hip_LUpLeg,1); */
  /* 'BVHTransform_c:557' C_HipYawPitch = zeros(Nframes,3,3); */
  /* 'BVHTransform_c:558' for k=1:Nframes */
  /* 'BVHTransform_c:559' HipYawPitch_k = HipYawPitch(k); */
  /* 'BVHTransform_c:560' Q_AxisL = AxisL*sin( HipYawPitch_k/2 ); */
  x = muDoubleScalarSin(HipYawPitch / 2.0);

  /* 'BVHTransform_c:561' Q_HipYawPitch_k = [ cos( HipYawPitch_k/2 ) Q_AxisL ] ; */
  /* 'BVHTransform_c:562' C_HipYawPitch(k,:,:) = FQtoCnb( Q_HipYawPitch_k ); */
  dv51[0] = muDoubleScalarCos(HipYawPitch / 2.0);
  for (i22 = 0; i22 < 3; i22++) {
    dv51[i22 + 1] = a[i22] * x;
  }

  st.site = &md_emlrtRSI;
  FQtoCnb(dv51, unusedExpr);

  /* 'BVHTransform_c:564' C_LUpLeg_SecondHip = CalculateC_Multi( C_LUpLeg_Hip,C_HipYawPitch ); */
  /* 'BVHTransform_c:565' C_LUpLeg_SecondHip = C_LUpLeg_Hip ; */
  /* 'BVHTransform_c:567' LHipPitch = zeros(1,Nframes); */
  /* 'BVHTransform_c:568' LHipRoll = zeros(1,Nframes); */
  /* 'BVHTransform_c:569' for k=1:Nframes */
  /* 'BVHTransform_c:570' C_LUpLeg_SecondHip_k = permute( C_LUpLeg_SecondHip(k,:,:),[2,3,1] ); */
  memcpy(&C_LUpLeg_SecondHip_k[0], &C_LUpLeg_Hip[0], 9U * sizeof(real_T));

  /* 'BVHTransform_c:571' [x,y,z] = deal( C_LUpLeg_SecondHip_k(1,2),C_LUpLeg_SecondHip_k(2,2),C_LUpLeg_SecondHip_k(3,2) ); */
  /* 'BVHTransform_c:573' LHipPitch(k) = asin( z ); */
  st.site = &nd_emlrtRSI;
  if ((C_LUpLeg_SecondHip_k[5] < -1.0) || (1.0 < C_LUpLeg_SecondHip_k[5])) {
    b_st.site = &ac_emlrtRSI;
    eml_error(&b_st);
  }

  *LHipPitch = muDoubleScalarAsin(C_LUpLeg_SecondHip_k[5]);

  /* 'BVHTransform_c:574' LHipRoll(k) = atan( -x/y ); */
  *LHipRoll = muDoubleScalarAtan(-C_LUpLeg_SecondHip_k[3] /
    C_LUpLeg_SecondHip_k[4]);

  /* 'BVHTransform_c:577' E_Hip_LUpLeg_ZXY = [ LHipRoll' LHipPitch' zeros(Nframes,1) ]; */
  /* 'BVHTransform_c:578' C_Hip_LUpLeg_ZXY = EulerToC_c( E_Hip_LUpLeg_ZXY,312,[1,1,1] ); */
  b_LHipRoll[0] = *LHipRoll;
  b_LHipRoll[1] = *LHipPitch;
  b_LHipRoll[2] = 0.0;
  for (i22 = 0; i22 < 3; i22++) {
    dv52[i22] = 1.0;
  }

  st.site = &od_emlrtRSI;
  EulerToC_c(&st, b_LHipRoll, 312.0, dv52, C_LUpLeg_Hip);

  /* 'BVHTransform_c:580' pos = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:581' posNew = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:582' pos_err = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:583' for k=1:Nframes */
  /* 'BVHTransform_c:586' C_LUpLeg_SecondHip_k = permute( C_LUpLeg_SecondHip(k,:,:),[2 3 1] ); */
  /* 'BVHTransform_c:587' C_Hip_LUpLeg_ZXY_k = permute( C_Hip_LUpLeg_ZXY(k,:,:),[2 3 1] ); */
  /* 'BVHTransform_c:589' pos(:,k) = C_LUpLeg_SecondHip_k*[0;1;0]; */
  /* 'BVHTransform_c:590' posNew(:,k) = C_Hip_LUpLeg_ZXY_k'*[0;1;0]; */
  /* 'BVHTransform_c:591' pos_err(:,k) = pos(:,k)-posNew(:,k) ; */
  /* 'BVHTransform_c:594' pos_err_sum = sum( sum( abs( pos_err ) ) ); */
  /*  if pos_err_sum>1e-1 */
  /*      for k=1:Nframes */
  /*          LHipPitch(k) = pi-LHipPitch(k) ; */
  /*      end     */
  /*      E_Hip_LUpLeg_ZXY = [ LHipRoll' LHipPitch' zeros(Nframes,1) ]; */
  /*      C_Hip_LUpLeg_ZXY = EulerToC_c( E_Hip_LUpLeg_ZXY,312,[1,1,1] ); */
  /*  end */
  /* %% get new BVH */
  /* 'BVHTransform_c:604' LeftUpLeg_new = CToEuler_c( C_Hip_LUpLeg_ZXY,rotateOrder_BVH,[1,1,1] ); */
  for (i22 = 0; i22 < 3; i22++) {
    dv52[i22] = 1.0;
  }

  st.site = &pd_emlrtRSI;
  CToEuler_c(&st, C_LUpLeg_Hip, rotateOrder_BVH, dv52, LeftUpLeg_new);
}

/*
 * function [ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_Spine_RForeArm,C_Spine_RArmNew,rotateOrder_BVH )
 */
static void TransformRElbow(const emlrtStack *sp, const real_T C_Spine_RForeArm
  [9], const real_T C_Spine_RArmNew[9], real_T rotateOrder_BVH, real_T
  RElbow_Nao[3], real_T RightForeArm_new[3])
{
  real_T C_RArmNew_RForeArm[9];
  real_T b_C_RArmNew_RForeArm[9];
  real_T dv29[3];
  int32_T i14;
  static const real_T dv30[3] = { -1.0, 1.0, 1.0 };

  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* % REblow */
  /* 'BVHTransform_c:385' coder.inline('never'); */
  /* 'BVHTransform_c:387' C_RArmNew_Spine = InverseC_Multi( C_Spine_RArmNew ); */
  /* 'BVHTransform_c:388' C_RArmNew_RForeArm = CalculateC_Multi( C_RArmNew_Spine,C_Spine_RForeArm ); */
  InverseC_Multi(C_Spine_RArmNew, C_RArmNew_RForeArm);
  memcpy(&b_C_RArmNew_RForeArm[0], &C_RArmNew_RForeArm[0], 9U * sizeof(real_T));
  CalculateC_Multi(b_C_RArmNew_RForeArm, C_Spine_RForeArm, C_RArmNew_RForeArm);

  /* 'BVHTransform_c:389' Nframes = size(C_RArmNew_RForeArm,1); */
  /* 'BVHTransform_c:390' RElbow_Nao = zeros(Nframes,3); */
  /* 'BVHTransform_c:391' for k=1:Nframes */
  /* 'BVHTransform_c:392' [x,y,z] = deal( C_RArmNew_RForeArm(k,1,1),C_RArmNew_RForeArm(k,1,2),C_RArmNew_RForeArm(k,1,3) ); */
  /* %% RElbowRoll > 0  % 人正常动作 */
  /* 'BVHTransform_c:395' RElbowRoll1 = acos(x) ; */
  st.site = &hc_emlrtRSI;
  if ((C_RArmNew_RForeArm[0] < -1.0) || (1.0 < C_RArmNew_RForeArm[0])) {
    b_st.site = &jc_emlrtRSI;
    b_eml_error(&b_st);
  }

  /* 'BVHTransform_c:396' RElbowYaw1 = atan2( -y,-z ); */
  /* 'BVHTransform_c:397' RElbow_Nao1 = [ RElbowYaw1 RElbowRoll1 0 ]; */
  RElbow_Nao[0] = muDoubleScalarAtan2(-C_RArmNew_RForeArm[3],
    -C_RArmNew_RForeArm[6]);
  RElbow_Nao[1] = muDoubleScalarAcos(C_RArmNew_RForeArm[0]);
  RElbow_Nao[2] = 0.0;

  /*      %%% RElbowRoll < 0  % 人很少做的动作：人右手不会这么转 */
  /*      RElbowRoll2 = -acos(x) ; */
  /*      RElbowYaw2 = atan2( y,z ); */
  /*      RElbow_Nao2 = [ RElbowYaw2 RElbowRoll2 0 ]; */
  /* 'BVHTransform_c:403' RElbow_Nao(k,:) = RElbow_Nao1 ; */
  /* 'BVHTransform_c:405' RElbow_Nao_degree = RElbow_Nao*180/pi ; */
  /* %% get new BVH */
  /* 'BVHTransform_c:407' RElbow_rotateOrder = 123; */
  /* 'BVHTransform_c:408' RElbow_rotateDirection = [-1,1,1]; */
  /* 'BVHTransform_c:409' BVH_rotateDirection = [1,1,1]; */
  /* 'BVHTransform_c:410' RightForeArm_new = zeros(Nframes,3); */
  /* 'BVHTransform_c:411' for k=1:Nframes */
  /* 'BVHTransform_c:412' RightForeArm_new(k,:) = ChangeEulerRotate_c(  RElbow_Nao( k,: ),RElbow_rotateOrder,... */
  /* 'BVHTransform_c:413'         rotateOrder_BVH,RElbow_rotateDirection,BVH_rotateDirection ) ; */
  for (i14 = 0; i14 < 3; i14++) {
    dv29[i14] = 1.0;
  }

  st.site = &ic_emlrtRSI;
  ChangeEulerRotate_c(&st, RElbow_Nao, 123.0, rotateOrder_BVH, dv30, dv29,
                      RightForeArm_new);
}

/*
 * function [ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH )
 */
static void TransformRFoot(const emlrtStack *sp, const real_T C_RLeg_RFoot[9],
  real_T rotateOrder_BVH, real_T *RAnklePitch, real_T *RAnkleRoll, real_T
  RightFoot_new[3])
{
  real_T dv50[3];
  int32_T i21;
  real_T E_RLeg_RFoot[3];
  real_T C_RLeg_RFoot_New[9];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % RFoot */
  /* 'BVHTransform_c:655' coder.inline('never'); */
  /* 'BVHTransform_c:657' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:658' E_RLeg_RFoot = CToEuler_c( C_RLeg_RFoot,132,rotateDirection_BVH ); */
  for (i21 = 0; i21 < 3; i21++) {
    dv50[i21] = 1.0;
  }

  st.site = &jd_emlrtRSI;
  CToEuler_c(&st, C_RLeg_RFoot, 132.0, dv50, E_RLeg_RFoot);

  /* 'BVHTransform_c:659' RAnklePitch = E_RLeg_RFoot( :,1 )'; */
  *RAnklePitch = E_RLeg_RFoot[0];

  /* 'BVHTransform_c:660' RAnkleRoll = E_RLeg_RFoot( :,2 )'; */
  *RAnkleRoll = E_RLeg_RFoot[1];

  /* %% get new BVH */
  /* 'BVHTransform_c:663' E_RLeg_RFoot(:,3) = 0; */
  E_RLeg_RFoot[2] = 0.0;

  /* 'BVHTransform_c:664' C_RLeg_RFoot_New = EulerToC_c( E_RLeg_RFoot,132,[1,1,1] ); */
  for (i21 = 0; i21 < 3; i21++) {
    dv50[i21] = 1.0;
  }

  st.site = &kd_emlrtRSI;
  EulerToC_c(&st, E_RLeg_RFoot, 132.0, dv50, C_RLeg_RFoot_New);

  /* 'BVHTransform_c:665' RightFoot_new = CToEuler_c( C_RLeg_RFoot_New,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i21 = 0; i21 < 3; i21++) {
    dv50[i21] = 1.0;
  }

  st.site = &ld_emlrtRSI;
  CToEuler_c(&st, C_RLeg_RFoot_New, rotateOrder_BVH, dv50, RightFoot_new);
}

/*
 * function [ RWristYaw,RightHand_new ] = TransformRHand( C_Spine_RHand,C_Spine_RForeArmNew,rotateOrder_BVH )
 */
static void TransformRHand(const emlrtStack *sp, const real_T C_Spine_RHand[9],
  const real_T C_Spine_RForeArmNew[9], real_T rotateOrder_BVH, real_T *RWristYaw,
  real_T RightHand_new[3])
{
  real_T dv31[9];
  real_T dv32[9];
  real_T dv33[9];
  real_T E_RForeArmNew_RHand[3];
  static const real_T dv34[3] = { -1.0, 1.0, 1.0 };

  real_T RHand_Nao[3];
  int32_T i15;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % RHand */
  /* 'BVHTransform_c:447' coder.inline('never'); */
  /* 'BVHTransform_c:449' C_RForeArmNew_Spine = InverseC_Multi( C_Spine_RForeArmNew ); */
  /* 'BVHTransform_c:450' C_RForeArmNew_RHand = CalculateC_Multi( C_RForeArmNew_Spine,C_Spine_RHand ); */
  /* 'BVHTransform_c:451' E_RForeArmNew_RHand = CToEuler_c( C_RForeArmNew_RHand,123,[-1,1,1] ); */
  InverseC_Multi(C_Spine_RForeArmNew, dv31);
  memcpy(&dv32[0], &dv31[0], 9U * sizeof(real_T));
  CalculateC_Multi(dv32, C_Spine_RHand, dv33);
  st.site = &mc_emlrtRSI;
  CToEuler_c(&st, dv33, 123.0, dv34, E_RForeArmNew_RHand);

  /* 'BVHTransform_c:452' RWristYaw = E_RForeArmNew_RHand(:,1)'; */
  *RWristYaw = E_RForeArmNew_RHand[0];

  /* %% get new BVH */
  /* 'BVHTransform_c:455' RHand_Nao = zeros( size(E_RForeArmNew_RHand) ); */
  for (i15 = 0; i15 < 3; i15++) {
    RHand_Nao[i15] = 0.0;
  }

  /* 'BVHTransform_c:456' RHand_Nao(:,1) = E_RForeArmNew_RHand(:,1) ; */
  RHand_Nao[0] = E_RForeArmNew_RHand[0];

  /* 'BVHTransform_c:457' Nframes = size(C_Spine_RHand,1); */
  /* 'BVHTransform_c:458' RHand_rotateOrder = 123; */
  /* 'BVHTransform_c:459' RHand_rotateDirection = [-1,1,1]; */
  /* 'BVHTransform_c:460' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:461' RightHand_new = zeros(Nframes,3); */
  /* 'BVHTransform_c:462' for k=1:Nframes */
  /* 'BVHTransform_c:463' RightHand_new(k,:) = ChangeEulerRotate_c(  RHand_Nao( k,: ),RHand_rotateOrder,rotateOrder_BVH,RHand_rotateDirection,rotateDirection_BVH ) ; */
  for (i15 = 0; i15 < 3; i15++) {
    E_RForeArmNew_RHand[i15] = 1.0;
  }

  st.site = &nc_emlrtRSI;
  ChangeEulerRotate_c(&st, RHand_Nao, 123.0, rotateOrder_BVH, dv34,
                      E_RForeArmNew_RHand, RightHand_new);
}

/*
 * function [ RKneePitch,RightLeg_New ] = TransformRLeg( C_Hip_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH )
 */
static void TransformRLeg(const emlrtStack *sp, const real_T C_Hip_RUpLegNew[9],
  const real_T C_Hip_RLeg[9], real_T rotateOrder_BVH, real_T *RKneePitch, real_T
  RightLeg_New[3])
{
  real_T C_RUpLegNew_RLegNew[9];
  real_T b_C_RUpLegNew_RLegNew[9];
  real_T dv48[3];
  int32_T i20;
  real_T dv49[9];
  real_T E_RUpLegNew_RLeg[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % RLeg */
  /* 'BVHTransform_c:608' coder.inline('never'); */
  /* 'BVHTransform_c:610' C_RUpLegNew_Hip = InverseC_Multi( C_Hip_RUpLegNew ); */
  /* 'BVHTransform_c:611' C_RUpLegNew_RLeg = CalculateC_Multi( C_RUpLegNew_Hip,C_Hip_RLeg ); */
  /* 'BVHTransform_c:612' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:613' E_RUpLegNew_RLeg = CToEuler_c( C_RUpLegNew_RLeg,123,rotateDirection_BVH ); */
  InverseC_Multi(C_Hip_RUpLegNew, C_RUpLegNew_RLegNew);
  memcpy(&b_C_RUpLegNew_RLegNew[0], &C_RUpLegNew_RLegNew[0], 9U * sizeof(real_T));
  for (i20 = 0; i20 < 3; i20++) {
    dv48[i20] = 1.0;
  }

  CalculateC_Multi(b_C_RUpLegNew_RLegNew, C_Hip_RLeg, dv49);
  st.site = &gd_emlrtRSI;
  CToEuler_c(&st, dv49, 123.0, dv48, E_RUpLegNew_RLeg);

  /* 'BVHTransform_c:614' RKneePitch = E_RUpLegNew_RLeg(:,1)'; */
  *RKneePitch = E_RUpLegNew_RLeg[0];

  /*  E_UpLeg_Leg = CToEuler_c( C_RUpLeg_RLeg,123 ); */
  /*  RKneePitch = E_UpLeg_Leg(:,1)'; */
  /* %% get new BVH */
  /* 'BVHTransform_c:620' E_RUpLegNew_RLegNew = E_RUpLegNew_RLeg ; */
  /* 'BVHTransform_c:621' E_RUpLegNew_RLegNew(:,2:3) = 0; */
  for (i20 = 0; i20 < 2; i20++) {
    E_RUpLegNew_RLeg[1 + i20] = 0.0;
  }

  /* 'BVHTransform_c:622' C_RUpLegNew_RLegNew = EulerToC_c( E_RUpLegNew_RLegNew,123,[1,1,1] ); */
  for (i20 = 0; i20 < 3; i20++) {
    dv48[i20] = 1.0;
  }

  st.site = &hd_emlrtRSI;
  EulerToC_c(&st, E_RUpLegNew_RLeg, 123.0, dv48, C_RUpLegNew_RLegNew);

  /* 'BVHTransform_c:623' RightLeg_New = CToEuler_c( C_RUpLegNew_RLegNew,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i20 = 0; i20 < 3; i20++) {
    dv48[i20] = 1.0;
  }

  st.site = &id_emlrtRSI;
  CToEuler_c(&st, C_RUpLegNew_RLegNew, rotateOrder_BVH, dv48, RightLeg_New);
}

/*
 * function [ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_Spine3_RShoulderNew,C_Spine3_RArm,rotateOrder_BVH )
 */
static void TransformRShoulder(const emlrtStack *sp, const real_T
  C_Spine3_RShoulderNew[9], const real_T C_Spine3_RArm[9], real_T
  rotateOrder_BVH, real_T *RShoulderPitch, real_T *RShoulderRoll, real_T
  RightArm_new[3])
{
  real_T C_RShoulderNew_RArm_NaoNew[9];
  real_T b_C_RShoulderNew_RArm_NaoNew[9];
  real_T c_C_RShoulderNew_RArm_NaoNew[9];
  real_T dv24[3];
  int32_T i13;
  real_T dv25[9];
  static const real_T dv26[9] = { 6.123233995736766E-17, 0.0, -1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 6.123233995736766E-17 };

  real_T E_RShoulderNew_RArm_Nao[3];
  real_T d_C_RShoulderNew_RArm_NaoNew[9];
  real_T dv27[9];
  static const real_T dv28[9] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 6.123233995736766E-17 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % RightShoulder */
  /* 'BVHTransform_c:324' coder.inline('never'); */
  /* 'BVHTransform_c:326' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:327' C_RShoulderNew_Spine3 = InverseC_Multi( C_Spine3_RShoulderNew ); */
  /* 'BVHTransform_c:328' C_RShoulderNew_RArm = CalculateC_Multi( C_RShoulderNew_Spine3,C_Spine3_RArm ); */
  /* %% 对 C_RShoulderNew_RArm 进行零位补偿 */
  /* 'BVHTransform_c:331' Nframes = size(C_Spine3_RShoulderNew,1); */
  /* 'BVHTransform_c:332' E_naoZero_NoitomZero_YXZ = [-pi/2,0,0]; */
  /* 'BVHTransform_c:333' E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 ); */
  /* 'BVHTransform_c:334' C_naoZero_NoitomZero_YXZ = EulerToC_c( E_naoZero_NoitomZero_YXZ,213,[1,1,1] ); */
  /* 'BVHTransform_c:336' C_RShoulderNew_RArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_RShoulderNew_RArm ); */
  /* 'BVHTransform_c:337' E_RShoulderNew_RArm_Nao = CToEuler_c( C_RShoulderNew_RArm_Nao,321,rotateDirection_BVH ); */
  InverseC_Multi(C_Spine3_RShoulderNew, C_RShoulderNew_RArm_NaoNew);
  memcpy(&b_C_RShoulderNew_RArm_NaoNew[0], &C_RShoulderNew_RArm_NaoNew[0], 9U *
         sizeof(real_T));
  CalculateC_Multi(b_C_RShoulderNew_RArm_NaoNew, C_Spine3_RArm,
                   C_RShoulderNew_RArm_NaoNew);
  memcpy(&c_C_RShoulderNew_RArm_NaoNew[0], &C_RShoulderNew_RArm_NaoNew[0], 9U *
         sizeof(real_T));
  for (i13 = 0; i13 < 3; i13++) {
    dv24[i13] = 1.0;
  }

  CalculateC_Multi(dv26, c_C_RShoulderNew_RArm_NaoNew, dv25);
  st.site = &ec_emlrtRSI;
  CToEuler_c(&st, dv25, 321.0, dv24, E_RShoulderNew_RArm_Nao);

  /* 'BVHTransform_c:338' E_RShoulderNew_RArm_Nao(:,3) = 0; */
  E_RShoulderNew_RArm_Nao[2] = 0.0;

  /* 'BVHTransform_c:339' C_RShoulderNew_RArm_NaoNew = EulerToC_c( E_RShoulderNew_RArm_Nao,321,[1,1,1] ); */
  for (i13 = 0; i13 < 3; i13++) {
    dv24[i13] = 1.0;
  }

  st.site = &fc_emlrtRSI;
  EulerToC_c(&st, E_RShoulderNew_RArm_Nao, 321.0, dv24,
             C_RShoulderNew_RArm_NaoNew);

  /* 'BVHTransform_c:340' RShoulderPitch = E_RShoulderNew_RArm_Nao(:,1)'; */
  *RShoulderPitch = E_RShoulderNew_RArm_Nao[0];

  /* 'BVHTransform_c:341' RShoulderRoll = E_RShoulderNew_RArm_Nao(:,2)'; */
  *RShoulderRoll = E_RShoulderNew_RArm_Nao[1];

  /* %% get new BVH */
  /*  零位补偿回去 */
  /* 'BVHTransform_c:345' E_NoitomZero_naoZero_ZYX = [0,pi/2,0]; */
  /*  321 */
  /* 'BVHTransform_c:346' E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 ); */
  /* 'BVHTransform_c:347' C_NoitomZero_naoZero_ZYX = EulerToC_c( E_NoitomZero_naoZero_ZYX,321 ,[1,1,1]); */
  /* 'BVHTransform_c:348' C_RShoulderNew_RArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_RShoulderNew_RArm_NaoNew ); */
  /* 'BVHTransform_c:350' RightArm_new = CToEuler_c( C_RShoulderNew_RArm_NoitomNew,rotateOrder_BVH,rotateDirection_BVH ); */
  memcpy(&d_C_RShoulderNew_RArm_NaoNew[0], &C_RShoulderNew_RArm_NaoNew[0], 9U *
         sizeof(real_T));
  for (i13 = 0; i13 < 3; i13++) {
    dv24[i13] = 1.0;
  }

  CalculateC_Multi(dv28, d_C_RShoulderNew_RArm_NaoNew, dv27);
  st.site = &gc_emlrtRSI;
  CToEuler_c(&st, dv27, rotateOrder_BVH, dv24, RightArm_new);
}

/*
 * function [ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Hip_RUpLeg,HipYawPitch,rotateOrder_BVH )
 */
static void TransformRUpLeg(const emlrtStack *sp, const real_T C_Hip_RUpLeg[9],
  real_T HipYawPitch, real_T rotateOrder_BVH, real_T *RHipRoll, real_T
  *RHipPitch, real_T RightUpLeg_new[3])
{
  real_T C_RUpLeg_Hip[9];
  real_T x;
  real_T dv46[4];
  int32_T k;
  static const real_T a[3] = { 0.70710678118654746, 0.70710678118654746, 0.0 };

  real_T unusedExpr[9];
  real_T C_RUpLeg_SecondHip_k[9];
  real_T b_RHipRoll[3];
  real_T dv47[3];
  real_T C_Hip_RUpLeg_ZXY[9];
  real_T C_Hip_RUpLeg_ZXY_k[9];
  real_T pos[3];
  int32_T i19;
  static const int8_T b[3] = { 0, 1, 0 };

  real_T posNew[3];
  real_T c_RHipRoll[3];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* % RUpLeg */
  /*  HipYawPitch：左右大腿的 HipYawPitch Angle  [ 1*Nframes ] */
  /* 'BVHTransform_c:489' C_RUpLeg_Hip = InverseC_Multi( C_Hip_RUpLeg ); */
  InverseC_Multi(C_Hip_RUpLeg, C_RUpLeg_Hip);

  /* 'BVHTransform_c:490' AxisR = [1,1,0] ; */
  /* 'BVHTransform_c:491' AxisR = AxisR/normest(AxisR) ; */
  /* 'BVHTransform_c:492' Nframes = size(C_Hip_RUpLeg,1); */
  /* 'BVHTransform_c:493' C_HipYawPitch = zeros(Nframes,3,3); */
  /* 'BVHTransform_c:494' for k=1:Nframes */
  /* 'BVHTransform_c:495' HipYawPitch_k = HipYawPitch(k); */
  /* 'BVHTransform_c:496' Q_AxisR = AxisR*sin( HipYawPitch_k/2 ); */
  x = muDoubleScalarSin(HipYawPitch / 2.0);

  /* 'BVHTransform_c:497' Q_HipYawPitch_k = [ cos( HipYawPitch_k/2 ) Q_AxisR ] ; */
  /* 'BVHTransform_c:498' C_HipYawPitch(k,:,:) = FQtoCnb( Q_HipYawPitch_k ); */
  dv46[0] = muDoubleScalarCos(HipYawPitch / 2.0);
  for (k = 0; k < 3; k++) {
    dv46[k + 1] = a[k] * x;
  }

  st.site = &vc_emlrtRSI;
  FQtoCnb(dv46, unusedExpr);

  /* 'BVHTransform_c:500' C_RUpLeg_SecondHip = CalculateC_Multi( C_RUpLeg_Hip,C_HipYawPitch ); */
  /* 'BVHTransform_c:501' C_RUpLeg_SecondHip = C_RUpLeg_Hip ; */
  /* 'BVHTransform_c:503' RHipPitch = zeros(1,Nframes); */
  /* 'BVHTransform_c:504' RHipRoll = zeros(1,Nframes); */
  /* 'BVHTransform_c:505' for k=1:Nframes */
  /* 'BVHTransform_c:506' C_RUpLeg_SecondHip_k = permute( C_RUpLeg_SecondHip(k,:,:),[2,3,1] ); */
  memcpy(&C_RUpLeg_SecondHip_k[0], &C_RUpLeg_Hip[0], 9U * sizeof(real_T));

  /* 'BVHTransform_c:507' [x,y,z] = deal( C_RUpLeg_SecondHip_k(1,2),C_RUpLeg_SecondHip_k(2,2),C_RUpLeg_SecondHip_k(3,2) ); */
  /* 'BVHTransform_c:509' RHipPitch(k) = asin( z ); */
  st.site = &wc_emlrtRSI;
  if ((C_RUpLeg_SecondHip_k[5] < -1.0) || (1.0 < C_RUpLeg_SecondHip_k[5])) {
    b_st.site = &ac_emlrtRSI;
    eml_error(&b_st);
  }

  *RHipPitch = muDoubleScalarAsin(C_RUpLeg_SecondHip_k[5]);

  /* 'BVHTransform_c:510' RHipRoll(k) = atan( -x/y ); */
  *RHipRoll = muDoubleScalarAtan(-C_RUpLeg_SecondHip_k[3] /
    C_RUpLeg_SecondHip_k[4]);

  /* 'BVHTransform_c:513' E_Hip_RUpLeg_ZXY = [ RHipRoll' RHipPitch' zeros(Nframes,1) ]; */
  /* 'BVHTransform_c:514' C_Hip_RUpLeg_ZXY = EulerToC_c( E_Hip_RUpLeg_ZXY,312,[1,1,1] ); */
  b_RHipRoll[0] = *RHipRoll;
  b_RHipRoll[1] = *RHipPitch;
  b_RHipRoll[2] = 0.0;
  for (k = 0; k < 3; k++) {
    dv47[k] = 1.0;
  }

  st.site = &xc_emlrtRSI;
  EulerToC_c(&st, b_RHipRoll, 312.0, dv47, C_Hip_RUpLeg_ZXY);

  /* 'BVHTransform_c:516' pos = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:517' posNew = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:518' pos_err = zeros( 3,Nframes ); */
  /* 'BVHTransform_c:519' for k=1:Nframes */
  /* 'BVHTransform_c:522' C_RUpLeg_SecondHip_k = permute( C_RUpLeg_SecondHip(k,:,:),[2 3 1] ); */
  /* 'BVHTransform_c:523' C_Hip_RUpLeg_ZXY_k = permute( C_Hip_RUpLeg_ZXY(k,:,:),[2 3 1] ); */
  for (k = 0; k < 9; k++) {
    C_RUpLeg_SecondHip_k[k] = C_RUpLeg_Hip[k];
    C_Hip_RUpLeg_ZXY_k[k] = C_Hip_RUpLeg_ZXY[k];
  }

  /* 'BVHTransform_c:525' pos(:,k) = C_RUpLeg_SecondHip_k*[0;1;0]; */
  /* 'BVHTransform_c:530' pos_err_sum = sum( sum( abs( pos_err ) ) ); */
  for (k = 0; k < 3; k++) {
    pos[k] = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      pos[k] += C_RUpLeg_SecondHip_k[k + 3 * i19] * (real_T)b[i19];
    }

    /* 'BVHTransform_c:526' posNew(:,k) = C_Hip_RUpLeg_ZXY_k'*[0;1;0]; */
    posNew[k] = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      posNew[k] += C_Hip_RUpLeg_ZXY_k[i19 + 3 * k] * (real_T)b[i19];
    }

    /* 'BVHTransform_c:527' pos_err(:,k) = pos(:,k)-posNew(:,k) ; */
    pos[k] -= posNew[k];
    posNew[k] = muDoubleScalarAbs(pos[k]);
  }

  x = posNew[0];
  for (k = 0; k < 2; k++) {
    x += posNew[k + 1];
  }

  /* 'BVHTransform_c:531' if pos_err_sum>1e-1 */
  if (x > 0.1) {
    /* 'BVHTransform_c:532' for k=1:Nframes */
    /* 'BVHTransform_c:533' RHipPitch(k) = pi-RHipPitch(k) ; */
    *RHipPitch = 3.1415926535897931 - *RHipPitch;

    /* 'BVHTransform_c:535' E_Hip_RUpLeg_ZXY = [ RHipRoll' RHipPitch' zeros(Nframes,1) ]; */
    /* 'BVHTransform_c:536' C_Hip_RUpLeg_ZXY = EulerToC_c( E_Hip_RUpLeg_ZXY,312,[1,1,1] ); */
    c_RHipRoll[0] = *RHipRoll;
    c_RHipRoll[1] = *RHipPitch;
    c_RHipRoll[2] = 0.0;
    for (k = 0; k < 3; k++) {
      dv47[k] = 1.0;
    }

    st.site = &yc_emlrtRSI;
    EulerToC_c(&st, c_RHipRoll, 312.0, dv47, C_Hip_RUpLeg_ZXY);
  }

  /* %% get new BVH */
  /* 'BVHTransform_c:540' RightUpLeg_new = CToEuler_c( C_Hip_RUpLeg_ZXY,rotateOrder_BVH,[1,1,1] ); */
  for (k = 0; k < 3; k++) {
    dv47[k] = 1.0;
  }

  st.site = &ad_emlrtRSI;
  CToEuler_c(&st, C_Hip_RUpLeg_ZXY, rotateOrder_BVH, dv47, RightUpLeg_new);
}

/*
 * function [ NaoData,ROOT_Hips_RotationNew ] = BVHTransform_c( BVHData )
 */
void BVHTransform_c(const emlrtStack *sp, const real32_T BVHData[354], real_T
                    NaoData[25], real_T ROOT_Hips_RotationNew[3])
{
  real_T b_BVHData[354];
  int32_T i0;
  real_T ROOT_Hips_Position[3];
  real_T dv0[3];
  real_T C_LLeg_LFoot[9];
  real_T C_LUpLeg_LLeg[9];
  real_T C_Hip_LUpLeg[9];
  real_T C_RLeg_RFoot[9];
  real_T C_RUpLeg_RLeg[9];
  real_T C_Hip_RUpLeg[9];
  real_T C_RForeArm_RHand[9];
  real_T C_RArm_RForeArm[9];
  real_T C_RShoulder_RArm[9];
  real_T C_Spine3_RShoulder[9];
  real_T C_LForeArm_LHand[9];
  real_T C_LArm_LForeArm[9];
  real_T C_LShoulder_LArm[9];
  real_T C_Spine3_LShoulder[9];
  real_T C_Neck_Head[9];
  real_T C_Spine3_Neck[9];
  real_T C_Spine2_Spine3[9];
  real_T C_Spine1_Spine2[9];
  real_T C_Spine2New_RShoulderNew[9];
  real_T C_RShoulderNew_RArmNew[9];
  real_T C_World_Hip[9];
  real_T b_C_RShoulderNew_RArmNew[9];
  real_T b_C_Spine2New_RShoulderNew[9];
  real_T b_C_Spine1_Spine2[9];
  real_T b_C_Spine3_LShoulder[9];
  real_T b_C_LShoulder_LArm[9];
  real_T b_C_LArm_LForeArm[9];
  real_T b_C_LForeArm_LHand[9];
  real_T C_Spine3_LHand[9];
  real_T C_Spine3_LForeArm[9];
  real_T C_Spine3_LArm[9];
  real_T C_Spine2_LHand[9];
  real_T C_Spine2_LForeArm[9];
  real_T C_Spine2_LArm[9];
  real_T C_Hip_Spine3[9];
  real_T C_Hip_Spine2[9];
  real_T dv1[9];
  real_T c_C_Spine1_Spine2[9];
  real_T dv2[9];
  real_T dv3[9];
  static const real_T C_I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  real_T HeadPitch;
  real_T HeadYaw;
  real_T RShoulderRoll;
  real_T RShoulderPitch;
  real_T c_C_RShoulderNew_RArmNew[9];
  real_T RElbow_Nao[3];
  real_T dv4[9];
  real_T RWristYaw;
  real_T dv5[9];
  real_T LShoulderRoll;
  real_T LShoulderPitch;
  real_T dv6[9];
  real_T LElbow_Nao[3];
  real_T d_C_Spine1_Spine2[9];
  real_T dv7[9];
  real_T LWristYaw;
  real_T dv8[9];
  real_T RHipPitch;
  real_T RHipRoll;
  real_T dv9[9];
  real_T RKneePitch;
  real_T RAnkleRoll;
  real_T RAnklePitch;
  real_T dv10[9];
  real_T LHipPitch;
  real_T LHipRoll;
  real_T dv11[9];
  real_T LKneePitch;
  real_T LAnkleRoll;
  real_T LAnklePitch;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* % xyz  2015.3.25 */
  /*  Transform BVH data Format */
  /*  BVHData: 1*354  degree */
  /*  NaoData: 25*1 degree */
  /*    function NaoData = BVHTransform_c( BVHData ) */
  /* 'BVHTransform_c:8' BVHData = double(BVHData); */
  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = BVHData[i0];
  }

  /* 'BVHTransform_c:9' ROOT_Hips_Position = BVHData(:,1:3); */
  for (i0 = 0; i0 < 3; i0++) {
    ROOT_Hips_Position[i0] = b_BVHData[i0];
  }

  /* 'BVHTransform_c:10' BVHData = BVHData*pi/180 ; */
  for (i0 = 0; i0 < 354; i0++) {
    b_BVHData[i0] = b_BVHData[i0] * 3.1415926535897931 / 180.0;
  }

  /* 'BVHTransform_c:11' BVHData(:,1:3) = ROOT_Hips_Position ; */
  for (i0 = 0; i0 < 3; i0++) {
    b_BVHData[i0] = ROOT_Hips_Position[i0];
  }

  /* 'BVHTransform_c:13' Nframes = size(BVHData,1); */
  /* 'BVHTransform_c:14' rotateOrder_BVH = 213; */
  /* 'BVHTransform_c:15' rotateDirection_BVH = [1,1,1]; */
  /* 'BVHTransform_c:17' [ C_World_Hip,C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_Neck,C_Neck_Head,C_Spine3_LShoulder,... */
  /* 'BVHTransform_c:18' C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand,... */
  /* 'BVHTransform_c:19' C_Hip_RUpLeg,C_RUpLeg_RLeg,C_RLeg_RFoot,C_Hip_LUpLeg,C_LUpLeg_LLeg,C_LLeg_LFoot ] = BVHDataToRotate_c( BVHData,rotateOrder_BVH,rotateDirection_BVH ) ; */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &emlrtRSI;
  BVHDataToRotate_c(&st, b_BVHData, 213.0, dv0, C_World_Hip,
                    C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew,
                    C_Spine1_Spine2, C_Spine2_Spine3, C_Spine3_Neck, C_Neck_Head,
                    C_Spine3_LShoulder, C_LShoulder_LArm, C_LArm_LForeArm,
                    C_LForeArm_LHand, C_Spine3_RShoulder, C_RShoulder_RArm,
                    C_RArm_RForeArm, C_RForeArm_RHand, C_Hip_RUpLeg,
                    C_RUpLeg_RLeg, C_RLeg_RFoot, C_Hip_LUpLeg, C_LUpLeg_LLeg,
                    C_LLeg_LFoot);

  /* % 所有BVH的旋转数据 */
  /* 'BVHTransform_c:22' [ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ... */
  /* 'BVHTransform_c:23'     CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand ) ; */
  for (i0 = 0; i0 < 9; i0++) {
    b_C_RShoulderNew_RArmNew[i0] = C_RShoulderNew_RArmNew[i0];
    b_C_Spine2New_RShoulderNew[i0] = C_Spine2New_RShoulderNew[i0];
    b_C_Spine1_Spine2[i0] = C_Spine1_Spine2[i0];
    b_C_Spine3_LShoulder[i0] = C_Spine3_LShoulder[i0];
    b_C_LShoulder_LArm[i0] = C_LShoulder_LArm[i0];
    b_C_LArm_LForeArm[i0] = C_LArm_LForeArm[i0];
    b_C_LForeArm_LHand[i0] = C_LForeArm_LHand[i0];
  }

  CalculatedLeftArmRotate(b_C_RShoulderNew_RArmNew, b_C_Spine2New_RShoulderNew,
    b_C_Spine1_Spine2, C_Spine2_Spine3, b_C_Spine3_LShoulder, b_C_LShoulder_LArm,
    b_C_LArm_LForeArm, b_C_LForeArm_LHand, C_Hip_Spine2, C_Hip_Spine3,
    C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew, C_Spine1_Spine2,
    C_Spine3_LShoulder, C_LShoulder_LArm, C_LArm_LForeArm, C_LForeArm_LHand,
    C_Spine2_LArm, C_Spine2_LForeArm, C_Spine2_LHand, C_Spine3_LArm,
    C_Spine3_LForeArm, C_Spine3_LHand);

  /* 'BVHTransform_c:25' [ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine2_RArm,C_Spine2_RForeArm,C_Spine2_RHand,C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ... */
  /* 'BVHTransform_c:26'     CalculatedRightArmRotate( C_Hip_Spine3,C_Spine2_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand ) ; */
  for (i0 = 0; i0 < 9; i0++) {
    b_C_RShoulderNew_RArmNew[i0] = C_RShoulderNew_RArmNew[i0];
    b_C_Spine2New_RShoulderNew[i0] = C_Spine3_RShoulder[i0];
    b_C_Spine1_Spine2[i0] = C_RShoulder_RArm[i0];
  }

  CalculatedRightArmRotate(C_Hip_Spine3, C_Spine2_Spine3,
    b_C_RShoulderNew_RArmNew, b_C_Spine2New_RShoulderNew, b_C_Spine1_Spine2,
    C_RArm_RForeArm, C_RForeArm_RHand, C_RShoulderNew_RArmNew,
    C_Spine2New_RShoulderNew, C_Spine1_Spine2, C_Spine3_LShoulder,
    C_LShoulder_LArm, C_LArm_LForeArm, C_Spine3_LHand, C_Spine3_RShoulder,
    C_RShoulder_RArm, C_LForeArm_LHand, C_Spine3_LArm, C_Spine3_LForeArm);

  /* 'BVHTransform_c:28' [ C_Hip_LLeg,C_Spine2_LLeg,C_Spine3_LLeg ] = CalculatedLLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_LUpLeg,C_LUpLeg_LLeg ) ; */
  CalculatedLLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_LUpLeg, C_LUpLeg_LLeg,
                       C_LShoulder_LArm, C_RShoulderNew_RArmNew,
                       C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:29' [ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg ) ; */
  CalculatedRLegRotate(C_Hip_Spine2, C_Hip_Spine3, C_Hip_RUpLeg, C_RUpLeg_RLeg,
                       C_Spine3_LShoulder, C_RShoulderNew_RArmNew,
                       C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:31' HipYawPitch = zeros(1,size(C_Hip_Spine,1)); */
  /* % common parameters to set */
  /*  reference joint : 'Hip'  'SPine'  'Spine3' */
  /* 'BVHTransform_c:34' refJoint1 = 'Spine2'; */
  /* 'BVHTransform_c:35' referenceOfHead = refJoint1 ; */
  /* 'BVHTransform_c:36' referenceOfArm = refJoint1; */
  /* 'BVHTransform_c:37' referenceOfForeArm = refJoint1; */
  /* 'BVHTransform_c:38' referenceOfHand =  refJoint1; */
  /* 'BVHTransform_c:40' E_0 = zeros( Nframes,3 ); */
  /* 'BVHTransform_c:41' RightShoulder_new = E_0; */
  /* 'BVHTransform_c:42' LeftShoulder_new = E_0; */
  /* 'BVHTransform_c:43' Spine3_new = E_0 ; */
  /* 'BVHTransform_c:44' Spine2_new = E_0 ; */
  /* 'BVHTransform_c:45' Spine_new = E_0 ; */
  /* 'BVHTransform_c:46' Neck_new = E_0; */
  /* 'BVHTransform_c:48' C_I = zeros( Nframes,3,3 ); */
  /* 'BVHTransform_c:49' for k=1:Nframes */
  /* 'BVHTransform_c:50' C_I(k,:,:) = [1 0 0; 0 1 0; 0 0 1 ]; */
  /* 'BVHTransform_c:52' C_Spine3New_NeckNew = C_I; */
  /* 'BVHTransform_c:53' C_Spine2New_Spine3New = C_I ; */
  /* 'BVHTransform_c:54' C_SpineNew_Spine3New = C_I ; */
  /* 'BVHTransform_c:55' C_HipNew_Spine3New = C_I ; */
  /* 'BVHTransform_c:56' C_Spine3New_LShoulderNew = C_I ; */
  /* 'BVHTransform_c:57' C_Spine3New_RShoulderNew = C_I ; */
  /* %% New Hip */
  /* 'BVHTransform_c:60' C_World_Spine2 = CalculateC_Multi( C_World_Hip,C_Hip_Spine2 ); */
  /* 'BVHTransform_c:61' E_World_Spine2 = CToEuler_c( C_World_Spine2,rotateOrder_BVH,rotateDirection_BVH ); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  CalculateC_Multi(C_World_Hip, C_Hip_Spine2, dv1);
  st.site = &b_emlrtRSI;
  CToEuler_c(&st, dv1, 213.0, dv0, ROOT_Hips_RotationNew);

  /* 'BVHTransform_c:62' ROOT_Hips_RotationNew = E_World_Spine2 ; */
  /* % head */
  /* 'BVHTransform_c:64' C_Spine3_Head = CalculateC_Multi( C_Spine3_Neck,C_Neck_Head ); */
  /* 'BVHTransform_c:65' C_Spine2_Head = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_Head ); */
  /* 'BVHTransform_c:66' C_Spine_Head = CalculateC_Multi( C_Spine_Spine3,C_Spine3_Head ); */
  /* 'BVHTransform_c:67' C_Hip_Head = CalculateC_Multi( C_Hip_Spine3,C_Spine3_Head ); */
  /* 'BVHTransform_c:69' C_Spine2New_NeckNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_NeckNew ); */
  /* 'BVHTransform_c:70' C_SpineNew_NeckNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_NeckNew ); */
  /* 'BVHTransform_c:71' C_HipNew_NeckNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_NeckNew ); */
  /* 'BVHTransform_c:73' C_RefHead_Head = C_Spine2_Head ; */
  /* 'BVHTransform_c:74' C_RefHeadNew_NeckNew = C_Spine2New_NeckNew; */
  /* 'BVHTransform_c:76' [ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_RefHead_Head,C_RefHeadNew_NeckNew,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_Spine3_Neck, C_Neck_Head, C_Spine1_Spine2);
  memcpy(&c_C_Spine1_Spine2[0], &C_Spine1_Spine2[0], 9U * sizeof(real_T));
  CalculateC_Multi(C_Spine2_Spine3, c_C_Spine1_Spine2, dv2);
  CalculateC_Multi(C_I, C_I, dv3);
  st.site = &c_emlrtRSI;
  TransformHead(&st, dv2, dv3, 213.0, &HeadYaw, &HeadPitch, ROOT_Hips_Position);

  /* % Right Shoulder */
  /* 'BVHTransform_c:79' C_Spine2New_RShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_RShoulderNew ); */
  CalculateC_Multi(C_I, C_I, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:80' C_SpineNew_RShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RShoulderNew ); */
  /* 'BVHTransform_c:81' C_HipNew_RShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RShoulderNew ); */
  /* 'BVHTransform_c:83' C_RefArm_RArm = C_Spine2_RArm; */
  /* 'BVHTransform_c:84' C_RefArmNew_RShoulderNew = C_Spine2New_RShoulderNew ; */
  /*  dbstop in TransformRShoulder */
  /* 'BVHTransform_c:87' [ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_RefArmNew_RShoulderNew,C_RefArm_RArm,rotateOrder_BVH ) ; */
  st.site = &d_emlrtRSI;
  TransformRShoulder(&st, C_Spine2New_RShoulderNew, C_Spine3_LHand, 213.0,
                     &RShoulderPitch, &RShoulderRoll, ROOT_Hips_Position);

  /* % Right Elbow */
  /* 'BVHTransform_c:90' C_RShoulderNew_RArmNew = EulerToC_c( RightArm_new,rotateOrder_BVH,[1,1,1] ); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &e_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_RShoulderNew_RArmNew);

  /* 'BVHTransform_c:91' C_Spine3New_RArmNew = CalculateC_Multi( C_Spine3New_RShoulderNew,C_RShoulderNew_RArmNew ); */
  /* 'BVHTransform_c:92' C_Spine2New_RArmNew = CalculateC_Multi( C_Spine2New_RShoulderNew,C_RShoulderNew_RArmNew ); */
  memcpy(&c_C_RShoulderNew_RArmNew[0], &C_RShoulderNew_RArmNew[0], 9U * sizeof
         (real_T));
  CalculateC_Multi(C_Spine2New_RShoulderNew, c_C_RShoulderNew_RArmNew,
                   C_RShoulderNew_RArmNew);

  /* 'BVHTransform_c:93' C_SpineNew_RArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RArmNew ); */
  /* 'BVHTransform_c:94' C_HipNew_RArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RArmNew ); */
  /* 'BVHTransform_c:96' C_RefArm_RForeArm = C_Spine2_RForeArm ; */
  /* 'BVHTransform_c:97' C_RefArmNew_RArmNew = C_Spine2New_RArmNew; */
  /* 'BVHTransform_c:99' [ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_RefArm_RForeArm,C_RefArmNew_RArmNew,rotateOrder_BVH ) ; */
  st.site = &f_emlrtRSI;
  TransformRElbow(&st, C_Spine3_RShoulder, C_RShoulderNew_RArmNew, 213.0,
                  RElbow_Nao, ROOT_Hips_Position);

  /* 'BVHTransform_c:100' [ RElbowYaw,RElbowRoll ] = deal( RElbow_Nao(:,1)',RElbow_Nao(:,2)' ); */
  /* % RHand */
  /* 'BVHTransform_c:103' C_RArmNew_RForeArmNew = EulerToC_c( RightForeArm_new,rotateOrder_BVH,[1,1,1] ); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &g_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:104' C_Spine3New_RForeArmNew = CalculateC_Multi( C_Spine3New_RArmNew,C_RArmNew_RForeArmNew ); */
  /* 'BVHTransform_c:105' C_Spine2New_RForeArmNew = CalculateC_Multi( C_Spine2New_RArmNew,C_RArmNew_RForeArmNew ); */
  /* 'BVHTransform_c:106' C_SpineNew_RForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RForeArmNew ); */
  /* 'BVHTransform_c:107' C_HipNew_RForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RForeArmNew ); */
  /* 'BVHTransform_c:109' C_RefArm_RHand = C_Spine2_RHand; */
  /* 'BVHTransform_c:110' C_RefArmNew_RForeArmNew = C_Spine2New_RForeArmNew; */
  /* 'BVHTransform_c:112' [ RWristYaw,RightHand_new ] = TransformRHand( C_RefArm_RHand,C_RefArmNew_RForeArmNew,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew, dv4);
  st.site = &h_emlrtRSI;
  TransformRHand(&st, C_RShoulder_RArm, dv4, 213.0, &RWristYaw,
                 ROOT_Hips_Position);

  /* % Left Shoulder */
  /* 'BVHTransform_c:115' C_Spine2New_LShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LShoulderNew ); */
  /* 'BVHTransform_c:116' C_SpineNew_LShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LShoulderNew ); */
  /* 'BVHTransform_c:117' C_HipNew_LShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LShoulderNew ); */
  /* 'BVHTransform_c:119' C_RefArm_LArm = C_Spine2_LArm; */
  /* 'BVHTransform_c:120' C_RefArmNew_LShoulderNew = C_Spine2New_LShoulderNew; */
  /* 'BVHTransform_c:122' [ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_RefArmNew_LShoulderNew,C_RefArm_LArm,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_I, C_I, dv5);
  st.site = &i_emlrtRSI;
  TransformLShoulder(&st, dv5, C_Spine2_LArm, 213.0, &LShoulderPitch,
                     &LShoulderRoll, ROOT_Hips_Position);

  /* % Left Elbow */
  /* 'BVHTransform_c:125' C_Spine3New_LShoulderNew = EulerToC_c( LeftShoulder_new,rotateOrder_BVH,[1,1,1] ); */
  /* 'BVHTransform_c:126' C_LShoulderNew_LArmNew = EulerToC_c( LeftArm_new,rotateOrder_BVH,[1,1,1] ); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &j_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:127' C_Spine3New_LArmNew = CalculateC_Multi( C_Spine3New_LShoulderNew,C_LShoulderNew_LArmNew ); */
  CalculateC_Multi(C_I, C_Spine2New_RShoulderNew, C_RShoulderNew_RArmNew);

  /* 'BVHTransform_c:128' C_Spine2New_LArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LArmNew ); */
  /* 'BVHTransform_c:129' C_SpineNew_LArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LArmNew ); */
  /* 'BVHTransform_c:130' C_HipNew_LArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LArmNew ); */
  /* 'BVHTransform_c:132' C_RefArm_LForeArm = C_Spine2_LForeArm; */
  /* 'BVHTransform_c:133' C_RefArmNew_LArmNew = C_Spine2New_LArmNew; */
  /* 'BVHTransform_c:135' [ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_RefArm_LForeArm,C_RefArmNew_LArmNew,rotateOrder_BVH ); */
  CalculateC_Multi(C_I, C_RShoulderNew_RArmNew, dv6);
  st.site = &k_emlrtRSI;
  TransformLElbow(&st, C_Spine2_LForeArm, dv6, 213.0, LElbow_Nao,
                  ROOT_Hips_Position);

  /* 'BVHTransform_c:136' [ LElbowYaw,LElbowRoll ] = deal( LElbow_Nao(:,1)',LElbow_Nao(:,2)' ); */
  /* % LHand */
  /* 'BVHTransform_c:139' C_LArmNew_LForeArmNew = EulerToC_c( LeftForeArm_new,rotateOrder_BVH ,[1,1,1]); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &l_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:140' C_Spine3New_LForeArmNew = CalculateC_Multi( C_Spine3New_LArmNew,C_LArmNew_LForeArmNew ); */
  /* 'BVHTransform_c:141' C_Spine2New_LForeArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LForeArmNew ); */
  /* 'BVHTransform_c:142' C_SpineNew_LForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LForeArmNew ); */
  /* 'BVHTransform_c:143' C_HipNew_LForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LForeArmNew ); */
  /* 'BVHTransform_c:145' C_RefArm_LHand = C_Spine2_LHand ; */
  /* 'BVHTransform_c:146' C_RefArmNew_LForeArmNew = C_Spine2New_LForeArmNew ; */
  /* 'BVHTransform_c:148' [ LWristYaw,LeftHand_new ] = TransformLHand( C_RefArm_LHand,C_RefArmNew_LForeArmNew,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew,
                   C_Spine1_Spine2);
  memcpy(&d_C_Spine1_Spine2[0], &C_Spine1_Spine2[0], 9U * sizeof(real_T));
  CalculateC_Multi(C_I, d_C_Spine1_Spine2, dv7);
  st.site = &m_emlrtRSI;
  TransformLHand(&st, C_Spine2_LHand, dv7, 213.0, &LWristYaw, ROOT_Hips_Position);

  /* % RLeg */
  /* 'BVHTransform_c:151' C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2); */
  InverseC_Multi(C_Hip_Spine2, C_RShoulderNew_RArmNew);

  /* 'BVHTransform_c:152' C_Spine2_RUpLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_RUpLeg ); */
  /*  dbstop in TransformRUpLeg */
  /* 'BVHTransform_c:154' [ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Spine2_RUpLeg,HipYawPitch,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Hip_RUpLeg, dv8);
  st.site = &n_emlrtRSI;
  TransformRUpLeg(&st, dv8, 0.0, 213.0, &RHipRoll, &RHipPitch,
                  ROOT_Hips_Position);

  /* 'BVHTransform_c:156' C_Hip_RUpLegNew = EulerToC_c( RightUpLeg_new,rotateOrder_BVH ,[1,1,1]); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &o_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:157' C_Spine2_RUpLegNew = CalculateC_Multi( C_Spine2_Hip,C_Hip_RUpLegNew ); */
  /* 'BVHTransform_c:158' [ RKneePitch,RightLeg_new ] = TransformRLeg( C_Spine2_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew, dv9);
  st.site = &p_emlrtRSI;
  TransformRLeg(&st, dv9, C_Spine3_LShoulder, 213.0, &RKneePitch,
                ROOT_Hips_Position);

  /* 'BVHTransform_c:159' [ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH ) ; */
  st.site = &q_emlrtRSI;
  TransformRFoot(&st, C_RLeg_RFoot, 213.0, &RAnklePitch, &RAnkleRoll,
                 ROOT_Hips_Position);

  /* % LLeg */
  /* 'BVHTransform_c:161' C_Spine2_LUpLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_LUpLeg ); */
  /* 'BVHTransform_c:162' [ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Spine2_LUpLeg,HipYawPitch,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Hip_LUpLeg, dv10);
  st.site = &r_emlrtRSI;
  TransformLUpLeg(&st, dv10, 0.0, 213.0, &LHipRoll, &LHipPitch,
                  ROOT_Hips_Position);

  /* 'BVHTransform_c:164' C_Hip_LUpLegNew = EulerToC_c( LeftUpLeg_new,rotateOrder_BVH,[1,1,1] ); */
  for (i0 = 0; i0 < 3; i0++) {
    dv0[i0] = 1.0;
  }

  st.site = &s_emlrtRSI;
  EulerToC_c(&st, ROOT_Hips_Position, 213.0, dv0, C_Spine2New_RShoulderNew);

  /* 'BVHTransform_c:165' C_Spine2_LUpLegNew = CalculateC_Multi( C_Spine2_Hip,C_Hip_LUpLegNew ); */
  /* 'BVHTransform_c:166' [ LKneePitch,LeftLeg_new ] = TransformLLeg( C_Spine2_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH ) ; */
  CalculateC_Multi(C_RShoulderNew_RArmNew, C_Spine2New_RShoulderNew, dv11);
  st.site = &t_emlrtRSI;
  TransformLLeg(&st, dv11, C_LShoulder_LArm, 213.0, &LKneePitch,
                ROOT_Hips_Position);

  /* 'BVHTransform_c:168' [ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH ) ; */
  st.site = &u_emlrtRSI;
  TransformLFoot(&st, C_LLeg_LFoot, 213.0, &LAnklePitch, &LAnkleRoll,
                 ROOT_Hips_Position);

  /* % Get Nao */
  /* 'BVHTransform_c:172' Nframes = size(HeadYaw,2); */
  /* 'BVHTransform_c:173' LHand = zeros( 1,Nframes ); */
  /* 'BVHTransform_c:174' RHand = zeros( 1,Nframes ); */
  /*  LHipRoll = LHipRoll*0 ;  */
  /*  LHipPitch = LHipPitch*0 ;  */
  /*  LKneePitch = LKneePitch*0 ;  */
  /*  RHipRoll = RHipRoll*0 ;  */
  /*  RHipPitch = RHipPitch*0;  */
  /*              */
  /*  RKneePitch = RKneePitch*0 ;  */
  /*  LAnklePitch = LAnklePitch*0 ;  */
  /*  LAnkleRoll = LAnkleRoll*0 ;  */
  /*  RAnklePitch = RAnklePitch*0 ;  */
  /*  RAnkleRoll = RAnkleRoll*0; */
  /* 'BVHTransform_c:188' NaoData = [ HeadYaw; HeadPitch; LShoulderPitch; LShoulderRoll; LElbowYaw;  LElbowRoll; ... */
  /* 'BVHTransform_c:189'            RShoulderPitch; RShoulderRoll; RElbowYaw ; RElbowRoll; ... */
  /* 'BVHTransform_c:190'            HipYawPitch; LHipRoll ; LHipPitch ; LKneePitch ; RHipRoll ; RHipPitch; ... */
  /* 'BVHTransform_c:191'            RKneePitch ; LAnklePitch ; LAnkleRoll ; RAnklePitch ; RAnkleRoll; ... */
  /* 'BVHTransform_c:192'            LWristYaw ; RWristYaw ; LHand ; RHand   ]; */
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

/* End of code generation (BVHTransform_c.cpp) */
