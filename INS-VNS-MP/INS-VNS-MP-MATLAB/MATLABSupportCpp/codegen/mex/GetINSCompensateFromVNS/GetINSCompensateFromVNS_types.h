/*
 * GetINSCompensateFromVNS_types.h
 *
 * Code generation for function 'GetINSCompensateFromVNS'
 *
 */

#ifndef __GETINSCOMPENSATEFROMVNS_TYPES_H__
#define __GETINSCOMPENSATEFROMVNS_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T
struct emxArray_char_T
{
    char_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_char_T*/
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T
{
    real_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_real_T*/
typedef struct
{
    real_T frequency;
    emxArray_real_T *Position;
    int32_T otherMakersN;
    real_T time;
    emxArray_char_T *MarkerSet;
    emxArray_real_T *ContinuesFlag;
    emxArray_real_T *ContinuesLastPosition;
    emxArray_real_T *ContinuesLastTime;
    emxArray_real_T *ContinuesLastK;
} struct1_T;
#ifndef struct_emxArray_struct1_T
#define struct_emxArray_struct1_T
struct emxArray_struct1_T
{
    struct1_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_struct1_T*/
typedef struct
{
    emxArray_real_T *time;
    emxArray_real_T *HipQuaternion;
    emxArray_real_T *HipPosition;
    emxArray_real_T *HeadQuaternion;
    emxArray_real_T *HeadPosition;
    emxArray_char_T *DataStyle;
    real_T BodyDirection[3];
    real_T frequency;
} struct0_T;
typedef struct
{
    real_T frequency;
    real_T time[100];
    real_T HipQuaternio[480];
    real_T HipPosition[360];
    real_T HeadQuaternion[480];
    real_T HeadPosition[360];
    real_T BodyDirection[3];
} struct2_T;

#endif
/* End of code generation (GetINSCompensateFromVNS_types.h) */
