
#include "stdafx.h"
#include "MATLABSupport.h"


//
// Arguments    : emxArray_char_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
 void emxInit_char_T(emxArray_char_T **pEmxArray, int b_numDimensions)
{
	emxArray_char_T *emxArray;
	int i;
	*pEmxArray = (emxArray_char_T *)malloc(sizeof(emxArray_char_T));
	emxArray = *pEmxArray;
	emxArray->data = (char *)NULL;
	emxArray->numDimensions = b_numDimensions;
	emxArray->size = (int *)malloc((unsigned int)(sizeof(int)* b_numDimensions));
	emxArray->allocatedSize = 0;
	emxArray->canFreeData = true;
	for (i = 0; i < b_numDimensions; i++) {
		emxArray->size[i] = 0;
	}
}

//
// Arguments    : struct1_T *pStruct
// Return Type  : void
//
 void emxInitStruct_struct1_T(struct1_T *pStruct)
{
	emxInit_real_T(&pStruct->Position, 2);
	emxInit_char_T(&pStruct->MarkerSet, 2);
	emxInit_real_T(&pStruct->ContinuesFlag, 2);
	emxInit_real_T(&pStruct->ContinuesLastPosition, 2);
	emxInit_real_T(&pStruct->ContinuesLastTime, 2);
	emxInit_real_T(&pStruct->ContinuesLastK, 2);
}


//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
	emxArray_real_T *emxArray;
	int i;
	*pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
	emxArray = *pEmxArray;
	emxArray->data = (double *)NULL;
	emxArray->numDimensions = b_numDimensions;
	emxArray->size = (int *)malloc((unsigned int)(sizeof(int)* b_numDimensions));
	emxArray->allocatedSize = 0;
	emxArray->canFreeData = true;
	for (i = 0; i < b_numDimensions; i++) {
		emxArray->size[i] = 0;
	}
}


// Arguments    : emxArray_struct1_T *otherMakers
//                double visual_k
//                double frequency
//                double time
//                const emxArray_real_T *Position
//                int otherMakersN
// Return Type  : void
//%% 给 otherMakers 赋值函数 1
// % 给第 visual_k 个值的 frequency, time, Position, otherMakersN 赋值

 void Assign_otherMakers_1(emxArray_struct1_T *otherMakers, double
	visual_k, double frequency, double time, const emxArray_real_T *Position, int
	otherMakersN)
{
	struct1_T b_otherMakers;
	struct1_T c_otherMakers;
	int i1;
	int loop_ub;
	emxInitStruct_struct1_T(&b_otherMakers);
	emxInitStruct_struct1_T(&c_otherMakers);

	// % 给 otherMakers 赋值函数 1
	otherMakers->data[(int)visual_k - 1].frequency = frequency;
	otherMakers->data[(int)visual_k - 1].time = time;
	i1 = otherMakers->data[(int)visual_k - 1].Position->size[0] *
		otherMakers->data[(int)visual_k - 1].Position->size[1];
	otherMakers->data[(int)visual_k - 1].Position->size[0] = 3;
	emxEnsureCapacity((emxArray__common *)otherMakers->data[(int)visual_k - 1].
		Position, i1, (int)sizeof(double));
	i1 = otherMakers->data[(int)visual_k - 1].Position->size[0] *
		otherMakers->data[(int)visual_k - 1].Position->size[1];
	otherMakers->data[(int)visual_k - 1].Position->size[1] = Position->size[1];
	emxEnsureCapacity((emxArray__common *)otherMakers->data[(int)visual_k - 1].
		Position, i1, (int)sizeof(double));
	emxCopyStruct_struct1_T(&b_otherMakers, &otherMakers->data[(int)visual_k - 1]);
	emxCopyStruct_struct1_T(&c_otherMakers, &otherMakers->data[(int)visual_k - 1]);
	loop_ub = Position->size[0] * Position->size[1];
	emxFreeStruct_struct1_T(&c_otherMakers);
	emxFreeStruct_struct1_T(&b_otherMakers);
	for (i1 = 0; i1 < loop_ub; i1++) {
		otherMakers->data[(int)visual_k - 1].Position->data[i1] = Position->data[i1];
	}

	otherMakers->data[(int)visual_k - 1].otherMakersN = otherMakersN;
}


//
// Arguments    : struct0_T *InertialData
//                emxArray_struct1_T *otherMakers
//                int I_N
//                int V_N
// Return Type  : void
//
/// 调用 MATLAB自动生成的函数，对 M_InertialData 和 M_otherMakers 进行初始化
// 将大小分别设置为 I_BufferN 和 V_BufferN， 同时将初值设为 NaN
// 惯性频率为96HZ，视觉频率为 30HZ
// 此函数由MATLAB自动生成，具体查阅相关MATLAB代码

 void c_Set_InertialData_otherMakers_( struct0_T *InertialData,
	emxArray_struct1_T *otherMakers, int I_N, int V_N)
{
	int i1;
	int i;
	static const char cv1[14] = { 'G', 'l', 'o', 'b', 'a', 'l', 'B', 'o', 'n', 'e',
		'Q', 'u', 'a', 't' };

	struct1_T b_otherMakers;
	struct1_T c_otherMakers;
	struct1_T d_otherMakers;
	struct1_T e_otherMakers;
	struct1_T f_otherMakers;
	struct1_T g_otherMakers;
	struct1_T h_otherMakers;
	struct1_T i_otherMakers;
	struct1_T j_otherMakers;
	struct1_T k_otherMakers;
	struct1_T l_otherMakers;
	struct1_T m_otherMakers;
	static const char cv2[4] = { 'h', 'e', 'a', 'd' };

	// % InertialData  和 otherMakers 置 NaN
	InertialData->frequency = 96.0;
	i1 = InertialData->time->size[0] * InertialData->time->size[1];
	InertialData->time->size[0] = 1;
	InertialData->time->size[1] = 1;
	emxEnsureCapacity((emxArray__common *)InertialData->time, i1, (int)sizeof
		(double));
	InertialData->time->data[0] = rtNaN;
	i1 = InertialData->HipQuaternion->size[0] * InertialData->HipQuaternion->size
		[1];
	InertialData->HipQuaternion->size[0] = 4;
	InertialData->HipQuaternion->size[1] = I_N;
	emxEnsureCapacity((emxArray__common *)InertialData->HipQuaternion, i1, (int)
		sizeof(double));
	i = I_N << 2;
	for (i1 = 0; i1 < i; i1++) {
		InertialData->HipQuaternion->data[i1] = rtNaN;
	}

	i1 = InertialData->HipPosition->size[0] * InertialData->HipPosition->size[1];
	InertialData->HipPosition->size[0] = 3;
	InertialData->HipPosition->size[1] = I_N;
	emxEnsureCapacity((emxArray__common *)InertialData->HipPosition, i1, (int)
		sizeof(double));
	i = 3 * I_N;
	for (i1 = 0; i1 < i; i1++) {
		InertialData->HipPosition->data[i1] = rtNaN;
	}

	i1 = InertialData->HeadQuaternion->size[0] * InertialData->
		HeadQuaternion->size[1];
	InertialData->HeadQuaternion->size[0] = 4;
	InertialData->HeadQuaternion->size[1] = I_N;
	emxEnsureCapacity((emxArray__common *)InertialData->HeadQuaternion, i1, (int)
		sizeof(double));
	i = I_N << 2;
	for (i1 = 0; i1 < i; i1++) {
		InertialData->HeadQuaternion->data[i1] = rtNaN;
	}

	i1 = InertialData->HeadPosition->size[0] * InertialData->HeadPosition->size[1];
	InertialData->HeadPosition->size[0] = 3;
	InertialData->HeadPosition->size[1] = I_N;
	emxEnsureCapacity((emxArray__common *)InertialData->HeadPosition, i1, (int)
		sizeof(double));
	i = 3 * I_N;
	for (i1 = 0; i1 < i; i1++) {
		InertialData->HeadPosition->data[i1] = rtNaN;
	}

	for (i = 0; i < 3; i++) {
		InertialData->BodyDirection[i] = rtNaN;
	}

	i1 = InertialData->DataStyle->size[0] * InertialData->DataStyle->size[1];
	InertialData->DataStyle->size[0] = 1;
	InertialData->DataStyle->size[1] = 14;
	emxEnsureCapacity((emxArray__common *)InertialData->DataStyle, i1, (int)sizeof
		(char));
	for (i1 = 0; i1 < 14; i1++) {
		InertialData->DataStyle->data[i1] = cv1[i1];
	}

	i = 0;
	emxInitStruct_struct1_T(&b_otherMakers);
	emxInitStruct_struct1_T(&c_otherMakers);
	emxInitStruct_struct1_T(&d_otherMakers);
	emxInitStruct_struct1_T(&e_otherMakers);
	emxInitStruct_struct1_T(&f_otherMakers);
	emxInitStruct_struct1_T(&g_otherMakers);
	emxInitStruct_struct1_T(&h_otherMakers);
	emxInitStruct_struct1_T(&i_otherMakers);
	emxInitStruct_struct1_T(&j_otherMakers);
	emxInitStruct_struct1_T(&k_otherMakers);
	emxInitStruct_struct1_T(&l_otherMakers);
	emxInitStruct_struct1_T(&m_otherMakers);
	while (i + 1 <= V_N) {
		otherMakers->data[i].frequency = 30.0;
		i1 = otherMakers->data[i].Position->size[0] * otherMakers->data[i].
			Position->size[1];
		otherMakers->data[i].Position->size[0] = 3;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].Position, i1,
			(int)sizeof(double));
		i1 = otherMakers->data[i].Position->size[0] * otherMakers->data[i].
			Position->size[1];
		otherMakers->data[i].Position->size[1] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].Position, i1,
			(int)sizeof(double));
		emxCopyStruct_struct1_T(&b_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&c_otherMakers, &otherMakers->data[i]);
		for (i1 = 0; i1 < 3; i1++) {
			otherMakers->data[i].Position->data[i1] = rtNaN;
		}

		otherMakers->data[i].otherMakersN = 0;
		otherMakers->data[i].time = rtNaN;
		i1 = otherMakers->data[i].MarkerSet->size[0] * otherMakers->data[i].
			MarkerSet->size[1];
		otherMakers->data[i].MarkerSet->size[0] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].MarkerSet, i1,
			(int)sizeof(char));
		i1 = otherMakers->data[i].MarkerSet->size[0] * otherMakers->data[i].
			MarkerSet->size[1];
		otherMakers->data[i].MarkerSet->size[1] = 4;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].MarkerSet, i1,
			(int)sizeof(char));
		emxCopyStruct_struct1_T(&d_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&e_otherMakers, &otherMakers->data[i]);
		for (i1 = 0; i1 < 4; i1++) {
			otherMakers->data[i].MarkerSet->data[i1] = cv2[i1];
		}

		i1 = otherMakers->data[i].ContinuesFlag->size[0] * otherMakers->data[i].
			ContinuesFlag->size[1];
		otherMakers->data[i].ContinuesFlag->size[0] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesFlag, i1,
			(int)sizeof(double));
		i1 = otherMakers->data[i].ContinuesFlag->size[0] * otherMakers->data[i].
			ContinuesFlag->size[1];
		otherMakers->data[i].ContinuesFlag->size[1] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesFlag, i1,
			(int)sizeof(double));
		emxCopyStruct_struct1_T(&f_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&g_otherMakers, &otherMakers->data[i]);
		otherMakers->data[i].ContinuesFlag->data[0] = rtNaN;
		i1 = otherMakers->data[i].ContinuesLastPosition->size[0] * otherMakers->
			data[i].ContinuesLastPosition->size[1];
		otherMakers->data[i].ContinuesLastPosition->size[0] = 3;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].
			ContinuesLastPosition, i1, (int)sizeof(double));
		i1 = otherMakers->data[i].ContinuesLastPosition->size[0] * otherMakers->
			data[i].ContinuesLastPosition->size[1];
		otherMakers->data[i].ContinuesLastPosition->size[1] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].
			ContinuesLastPosition, i1, (int)sizeof(double));
		emxCopyStruct_struct1_T(&h_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&i_otherMakers, &otherMakers->data[i]);
		for (i1 = 0; i1 < 3; i1++) {
			otherMakers->data[i].ContinuesLastPosition->data[i1] = rtNaN;
		}

		i1 = otherMakers->data[i].ContinuesLastTime->size[0] * otherMakers->data[i].
			ContinuesLastTime->size[1];
		otherMakers->data[i].ContinuesLastTime->size[0] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesLastTime,
			i1, (int)sizeof(double));
		i1 = otherMakers->data[i].ContinuesLastTime->size[0] * otherMakers->data[i].
			ContinuesLastTime->size[1];
		otherMakers->data[i].ContinuesLastTime->size[1] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesLastTime,
			i1, (int)sizeof(double));
		emxCopyStruct_struct1_T(&j_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&k_otherMakers, &otherMakers->data[i]);
		otherMakers->data[i].ContinuesLastTime->data[0] = rtNaN;
		i1 = otherMakers->data[i].ContinuesLastK->size[0] * otherMakers->data[i].
			ContinuesLastK->size[1];
		otherMakers->data[i].ContinuesLastK->size[0] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesLastK,
			i1, (int)sizeof(double));
		i1 = otherMakers->data[i].ContinuesLastK->size[0] * otherMakers->data[i].
			ContinuesLastK->size[1];
		otherMakers->data[i].ContinuesLastK->size[1] = 1;
		emxEnsureCapacity((emxArray__common *)otherMakers->data[i].ContinuesLastK,
			i1, (int)sizeof(double));
		emxCopyStruct_struct1_T(&l_otherMakers, &otherMakers->data[i]);
		emxCopyStruct_struct1_T(&m_otherMakers, &otherMakers->data[i]);
		otherMakers->data[i].ContinuesLastK->data[0] = rtNaN;
		i++;
	}

	emxFreeStruct_struct1_T(&m_otherMakers);
	emxFreeStruct_struct1_T(&l_otherMakers);
	emxFreeStruct_struct1_T(&k_otherMakers);
	emxFreeStruct_struct1_T(&j_otherMakers);
	emxFreeStruct_struct1_T(&i_otherMakers);
	emxFreeStruct_struct1_T(&h_otherMakers);
	emxFreeStruct_struct1_T(&g_otherMakers);
	emxFreeStruct_struct1_T(&f_otherMakers);
	emxFreeStruct_struct1_T(&e_otherMakers);
	emxFreeStruct_struct1_T(&d_otherMakers);
	emxFreeStruct_struct1_T(&c_otherMakers);
	emxFreeStruct_struct1_T(&b_otherMakers);
}
