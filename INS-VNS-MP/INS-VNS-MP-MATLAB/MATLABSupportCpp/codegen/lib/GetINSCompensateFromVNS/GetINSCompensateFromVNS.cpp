//
// File: GetINSCompensateFromVNS.cpp
//
// MATLAB Coder version            : 2.6
// C/C++ source code generated on  : 15-Jun-2015 10:51:25
//

// Include files
#include "rt_nonfinite.h"
#include "GetINSCompensateFromVNS.h"

// Function Declarations
static void c_Set_InertialData_otherMakers_(struct0_T *InertialData, struct1_T
  otherMakers[3600]);

// Function Definitions

//
// Arguments    : struct0_T *InertialData
//                struct1_T otherMakers[3600]
// Return Type  : void
//
static void c_Set_InertialData_otherMakers_(struct0_T *InertialData, struct1_T
  otherMakers[3600])
{
  int i;
  int k;

  //  %% 给 otherMakers 赋值函数 1
  //  frequency = otherMakers(2).frequency;
  //  time = otherMakers(2).time;
  //  v_Position = zeros(3,20);
  //  v_Position(:,32) = [1;3;-2];
  //  otherMakersN = otherMakers(6).otherMakersN;
  //
  //  for visual_k=1:100
  //      otherMakers1 = Assign_otherMakers_1( otherMakers1,visual_k,frequency,time,v_Position,otherMakersN ); 
  //  end
  // % InertialData  和 otherMakers 置 NaN
  // % 用于给 C++ 自动生成 初始化 函数
  InertialData->frequency = 96.0;
  for (i = 0; i < 46080; i++) {
    InertialData->HipQuaternion[i] = rtNaN;
  }

  for (i = 0; i < 11520; i++) {
    InertialData->time[i] = rtNaN;
  }

  for (i = 0; i < 34560; i++) {
    InertialData->HipPosition[i] = rtNaN;
  }

  for (i = 0; i < 46080; i++) {
    InertialData->HeadQuaternion[i] = rtNaN;
  }

  for (i = 0; i < 34560; i++) {
    InertialData->HeadPosition[i] = rtNaN;
  }

  for (i = 0; i < 3; i++) {
    InertialData->BodyDirection[i] = rtNaN;
  }

  //  InertialData.DataStyle =  'GlobalBoneQuat';
  for (k = 0; k < 3600; k++) {
    otherMakers[k].frequency = 30.0;
    for (i = 0; i < 30; i++) {
      otherMakers[k].Position[i] = rtNaN;
    }

    otherMakers[k].otherMakersN = 0;
    otherMakers[k].time = rtNaN;
    otherMakers[k].MarkerSet = 16;

    //  'head';
    for (i = 0; i < 10; i++) {
      otherMakers[k].ContinuesFlag[i] = rtNaN;
    }

    for (i = 0; i < 30; i++) {
      otherMakers[k].ContinuesLastPosition[i] = rtNaN;
    }

    for (i = 0; i < 10; i++) {
      otherMakers[k].ContinuesLastTime[i] = rtNaN;
    }

    for (i = 0; i < 10; i++) {
      otherMakers[k].ContinuesLastK[i] = rtNaN;
    }
  }
}

//
// Arguments    : const struct0_T *InertialData
//                const struct1_T otherMakers[3600]
//                double compensateRate
//                struct0_T *InertialData1
//                struct1_T otherMakers1[3600]
//                double *compensateRate1
//                struct2_T *InertialData2
//                struct1_T otherMakers2_data[]
//                int otherMakers2_size[2]
// Return Type  : void
//
void GetINSCompensateFromVNS(const struct0_T *InertialData, const struct1_T
  otherMakers[3600], double compensateRate, struct0_T *InertialData1, struct1_T
  otherMakers1[3600], double *compensateRate1, struct2_T *InertialData2,
  struct1_T otherMakers2_data[], int otherMakers2_size[2])
{
  int i0;
  int i1;
  static const double dv0[6] = { 1.0, 2.0, 3.0, 0.1, 0.2, 0.3 };

  // % xyz 2015 5.25
  // % otherMakers
  //  otherMakers(k).frequency [1]
  //  otherMakers(k).Position  [3*M]
  //  otherMakers(k).otherMakersN [1]
  // 　otherMakers(k).time [1]
  // 　otherMakers(k).MarkerSet ""
  //  记录每个马克点的连续特性
  //  otherMakers(k).ContinuesFlag = zeros(1,M) ; % 不连续
  //  otherMakers(k).ContinuesLastPosition = NaN(3,M)  ;
  //  otherMakers(k).ContinuesLastTime = NaN[1*M] ;
  //  otherMakers(k).ContinuesLastK = NaN[1*M];
  // % InertialData
  //  InertialData.frequency (k)
  //  InertialData.time (k)
  //  InertialData.HipQuaternion(k)  [4*N]
  //  InertialData.HipPosition (k)  [3*N]
  //  InertialData.HeadQuaternion (k)  [4*N]
  //  InertialData.HeadPosition (k)  [3*N]
  //  InertialData.BodyDirection(k)  [3*1]
  //  InertialData.DataStyle(k)  'GlobalBoneQuat'
  // % 用 Optitrack 的 OtherMarker 补偿惯性系统
  // % 得到惯性 Hip 位置补偿量 InertialPositionCompensate
  //  InertialPositionCompensate [ 3*N ]  m  NED系
  // %
  // % InertialData  和 otherMakers 置 NaN
  *InertialData1 = *InertialData;
  for (i0 = 0; i0 < 3600; i0++) {
    otherMakers1[i0] = otherMakers[i0];
  }

  c_Set_InertialData_otherMakers_(InertialData1, otherMakers1);

  // % InertialData 格式数据创建
  for (i0 = 0; i0 < 46080; i0++) {
    InertialData2->HeadQuaternion[i0] = InertialData1->HeadQuaternion[i0];
  }

  // % InertialData 格式数据Copy
  // % otherMakers 格式数据创建
  otherMakers2_size[0] = 1;
  otherMakers2_size[1] = 1;
  otherMakers2_data[0] = otherMakers1[0];
  otherMakers2_data[14] = otherMakers1[4];

  // % otherMakers 格式数据Copy
  // % compensateRate
  *compensateRate1 = compensateRate;

  // % InertialData 数据赋值
  InertialData1->frequency = 96.0;

  // % otherMakers 数据赋值
  otherMakers1[0].frequency = 30.0;
  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      otherMakers1[0].Position[i1 + 3 * i0] = dv0[i1 + 3 * i0];
    }
  }

  otherMakers1[0].MarkerSet = 16;
}

//
// Arguments    : void
// Return Type  : void
//
void GetINSCompensateFromVNS_initialize()
{
  rt_InitInfAndNaN(8U);
}

//
// Arguments    : void
// Return Type  : void
//
void GetINSCompensateFromVNS_terminate()
{
  // (no terminate code required)
}

//
// File trailer for GetINSCompensateFromVNS.cpp
//
// [EOF]
//
