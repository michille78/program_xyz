#pragma once

#ifdef __APPLE__
    #define __stdcall   // '__stdcall' is ignored because of '__stdcall' equalls to default Mach -O
#endif

/********************************************
 *          Callbacks for data outputs      *
 ********************************************/
// Calibration progress callback
typedef void (__stdcall *PNEventCalibrationProgressCallback)(void* customObject, int avatarIndex, float percent);
// Raw file playing callback
typedef void (__stdcall *PNEventPlayProgressCallback)(void* customObject, int currentFrame, int totalFrames);
// Contact event callback
typedef void (__stdcall *PNEventContactNotificationCallback)(void* customObject, int boneId);
// Parsed raw data callback
typedef void (__stdcall *PNEventRawDataParsedCallback)(void* customObject, int sensorId, Quaternion4_t* quat, Vector3_t* acc, Vector3_t* gyro);
// Calculated frame data callback
typedef void (__stdcall *PNEventCalculatedStringDataCallback)(void* customObject, char* calculationData);
// Calculated frame binary data callback
typedef void (__stdcall *PNEventCalculatedBinaryDataCallback)(void* customObject, int avatarIndex, CalculationDataHeader* cbp, int packLen);
// BVH string data boradcast callback
typedef void (__stdcall *PNEventBVHStringDataBoardcastCallback)(void* customObject, int avatarIndex, char* bvhData);
// BVH binary data boradcast callback
typedef void (__stdcall *PNEventBVHBinaryDataBoardcastCallback)(void* customObject, int avatarIndex, BvhOutputBinaryHeader* bbp, int packLen);
// BVH matrix string data callback
typedef void (__stdcall *PNEventBVHMatrixDataBoardcastCallback)(void* customObject, int avatarIndex, char* matrixData);
// Constraint data callback
typedef void (__stdcall *PNEventConstraintDataCallback)(void* customObject, int avatarIndex, FrameContactData* contactData);

/****************Recognition action data callback************/
// boneId: 发出动作的骨骼Id
// eventCode: 动作代码：
//	8: P0, Single tapping
//	1: P1, Double tapping
//	2: P2, Fire
//	3: P3, In-ward flipping
//	4: P4, Out-ward flipping
// level: range1~10, undefined
typedef void(__stdcall *PNEventActionRecognitionDataStringStreamCallback)(void* customObject, int avatarIndex, char* actionData);
//typedef void(__stdcall *PNEventActionRecognizedEvent)(void* customObject, int avatarIndex, int boneId, int eventCode, int level);
// 动作识别：重心矢量
typedef void(__stdcall *PNEventBodyMassVectorStringCallback)(void* customObject, int avatarIndex, char* data);
// 动作识别：身体倾斜矢量
typedef void(__stdcall *PNEventBodySwingVectorStringCallback)(void* customObject, int avatarIndex, char* data);

/***********************************************/