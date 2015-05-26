/*
* Copyright: Copyright 2014 Beijing Noitom Technology Ltd. All Rights reserved.
* Pending Patents: PCT/CN2014/085659 PCT/CN2014/071006
* 
* Licensed under the Perception Neuron SDK License Beta Version (the “License");
* You may only use the Perception Neuron SDK when in compliance with the License,
* which is provided at the time of installation or download, or which
* otherwise accompanies this software in the form of either an electronic or a hard copy.
* 
* Unless required by applicable law or agreed to in writing, the Perception Neuron SDK
* distributed under the License is provided on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing conditions and
* limitations under the License.
*/

#pragma once

/**********************************************************
 *               Symbol import definition                 *
 *   If you are using this PNLib sdk in Mac OS X, Linux or*
 * unix like OS, you should define '__OS_XUN__' symbol in *
 * your 'Preprocessing' config segment.                   *
 **********************************************************/
#ifdef PNLIB_EXPORTS
#ifdef __OS_XUN__
#define PNLIB_API __attribute__((visibility("default")))
#else
#define PNLIB_API __declspec(dllexport)
#endif
#else
#ifdef __OS_XUN__
#define PNLIB_API // EMPTY
#else
#define PNLIB_API __declspec(dllimport)
#endif
#endif

/**********************************************************
 *          Include customed data/callbak types           *
 **********************************************************/
#include "PNDataTypes.h"
#include "PNCallbacks.h"

/**********************************************************
 *          Export pure carbon intface/functions          *
 *  So that C# or Unity can use it directly               *
 **********************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************
 *          Initialize and config PNLib environment      *
 **********************************************************/
// Get this running PNLib version number
PNLIB_API PNLibVersion PNGetLibVersion();

// Initialize library
// Must initialize the library env by calling this function before using PNLib
PNLIB_API void PNLibInit();

// Get error code to find more informations if called a function failed
PNLIB_API const PNSTATUS PNGetLastErrorCode();

// Get error informations by error code
PNLIB_API const char* PNGetLastErrorMessage();

// Set sensor suit type: Neuron suit or Wireless legacy sensor type
PNLIB_API void PNSetSensorSuitType(SensorSuitTypes modes);

// Get standard bone system table of PNLib
// return the array point of BoneMap, the count is defined by 'FULL_BODY_BONE_COUNT'
PNLIB_API const BoneMap* PNGetStandardBoneTable();

// Set node combination mode
PNLIB_API void PNSetNodeCombinationMode(int avatarIndex, NodeCombinationModes mode);

// Set temporary folder or data export default folder.
// Calibration data will save to the temporary folder.
// Exoprted data file will be saved to the 'workingFolder'.
PNLIB_API void PNSetDataFolders(char* appDataFolder, char* workingFolder);
    
// Set data acquisition frequency. Unit: Hz
PNLIB_API void PNSetDataAcquisitionFrequency(int freq);

// Get data acquisition frequency.
PNLIB_API int PNGetDataAcquisitionFrequency();

// Set or switch run mode between realtime mode and raw file playing mode
// (打开文件前必须显式的指定为播放模式
PNLIB_API void PNSetRunningMode(RunningMode runMode);

// Get current running mode
PNLIB_API RunningMode PNGetRunningMode();

// 装载上次校准完毕后缓冲的校准数据，以便即使程序重启也无需重新校准（传感器相对安装位置未做过调整的情况下）
PNLIB_API void PNLoadCalibrationData();

// 激活上台阶功能（关闭地面接触功能）
PNLIB_API void PNEnableClimbContact(int avatarIndex, PNBOOL enable);

// 重置上台阶参数
PNLIB_API void PNResetClimbContact(int avatarIndex);

// 是否激活磁免疫功能
PNLIB_API void PNEnableMagneticImmune(int avatarIndex, MagneticImmuneLevel level);

// 设置脊柱平滑系数, breastbone:胸椎平滑系数, vertebra:脊椎平滑系数
PNLIB_API void PNSetSpineSmoothFactors(int avatarIndex, double breastbone, double vertebra);

// The default accelerator type of sensors is SA_LegacySmallRange, modify it if using different sensor(s)
PNLIB_API void PNSetSensorAcceleratorType(int sensorId, SensorAcceleratorTypes type);

// Get sensor type
PNLIB_API SensorAcceleratorTypes PNGetSensorAcceleratorType(int sensorId);

// Release PNLib
PNLIB_API void PNLibRelease();
    

/***********************************************************
 *        Register and config data output callbacks        *
 *                                                         *
 *    There are lots of levels to support diferent users   *
 * to use the data:                                        *
 * Level 1: Use a few sensors to capture and use raw sensor*
 *          data directly.                                 *
 * Level 2: Use a suit sensor, and want to use raw frame   *
 *          data to implementate thire arithmetic          *
 * Level 3: Use a suit sensor, and want to use calculated  *
 *          frame data to do something                     *
 * Level 4: Use a suit sensor, and want to use BVH data to *
 *          drive thire model directly                     *
 ***********************************************************/
// Register calibration progress callback
PNLIB_API void PNRegisterCalibrationProgressHandle(void* customObject, PNEventCalibrationProgressCallback handle);

// Register BVH string data callback handle if you want to deal with string type
PNLIB_API void PNRegisterBvhStringDataBoardcastHandle(void* customObject, PNEventBVHStringDataBoardcastCallback handle);

// Register BVH binary data callback handle if you want to deal with binary type
PNLIB_API void PNRegisterBvhBinaryDataBoardcastHandle(void* customObject, PNEventBVHBinaryDataBoardcastCallback handle);

// Register matrix type of BVH data callback handle if you want to get matrix string data
PNLIB_API void PNRegisterBvhMatrixDataBoardcastHandle(void* customObject, PNEventBVHMatrixDataBoardcastCallback handle);

// Change output type of BVH data. You must register the relevant callback handle before change to a certain one.
PNLIB_API void PNSetBvhDataBlockBoardcastType(BvhDataStreamTypes type);

// Disable BVH callback if you do not need it. Because BVH conve
PNLIB_API void PNEnableBvhDataBoardcast(PNBOOL isEnable);

// Change output type of Calculated data.
PNLIB_API void PNSetCalculatedDataBlockBoardcastType(CalculatedDataStreamTypes type);

// Register calculation data callback handle
PNLIB_API void PNRegisterCalculatedStringDataBoardcastHandle(void* customObject, PNEventCalculatedStringDataCallback handle);

// Register calculation data callback handle
PNLIB_API void PNRegisterCalculatedBinaryDataBoardcastHandle(void* customObject, PNEventCalculatedBinaryDataCallback handle);

// Change quaternion type in calculation frame data
PNLIB_API void PNSetCalculatedQuaternionDataType(OutputQuaternionTypes type);

// Change Acceleration type in calculation frame data
PNLIB_API void PNSetCalculatedAccelerationDataType(OutputAccelerationTypes type);

// Change Gyro type in calculation frame data
PNLIB_API void PNSetCalculatedGyroDataType(OutputGyroType type);

// Enable/disable calculation data callback
PNLIB_API void PNEnableCalculationDataBoardcast(PNBOOL enable);

// Register raw data playing progress callback
PNLIB_API void PNRegisterRawDataPlayingProgressHandle(void* customObject, PNEventPlayProgressCallback handle);

// Register raw data parsed data callback
PNLIB_API void PNRegisterPlayingRawDataParsedHandle(void* customObject, PNEventRawDataParsedCallback handle);

// 接触事件回调
PNLIB_API void PNRegisterContactNotification(void* customObject, PNEventContactNotificationCallback handle);
    
// 设置接触编辑回调函数（函数名称或功能会做比较大的调整）
PNLIB_API void PNRegisterContactEditCallback(void* customObject, PNEventConstraintDataCallback editHandler);

// 激活/取消接触事件通知功能
PNLIB_API void PNEnableContactNotification(PNBOOL enable);

// 设置动作识别事件回调函数
PNLIB_API void PNRegisterActionRecognizationStringDataBoardcastHandle(void* customObject, PNEventActionRecognitionDataStringStreamCallback handle);

// 设置动作识别，重心矢量回调函数
PNLIB_API void PNRegisterBodyMassVectorStringCallback(void* customObject, PNEventBodyMassVectorStringCallback handle);

// 身体倾斜矢量回调函数
PNLIB_API void PNRegisterBodySwingVectorStringCallback(void* customObject, PNEventBodySwingVectorStringCallback handle);

/**********************************************************
 *                     Avatar management                  *
 **********************************************************/
// Create a avatar in scene
PNLIB_API int PNCreateAvatar();

// Delete a avatar from scene
PNLIB_API int PNRemoveAvatar(int avatarIndex);

// Get total avatars count in current scene
PNLIB_API int PNGetAvatarCount();

// Update BVH head of all avatar(s)
PNLIB_API void PNUpdateBvhDataHeader();

// Set a name for avatar
PNLIB_API void PNSetAvatarName(int avatarIndex, char* name);

// Get avatar's name 
PNLIB_API void PNGetAvatarName(int avatarIndex, char* name, int buffLen);

// Set avatar's bone dimensions
PNLIB_API void PNSetBoneDimensions(int avatarIndex, BoneDimension* dimensions);

// Get avatar's bone dimensions
PNLIB_API void PNGetBoneDimensions(int avatarIndex, BoneDimension* dimsBuffer);

// Get avatar's bone length
PNLIB_API float PNGetBoneLength(int avatarIndex, int boneIndex);

// Binding a sensor to bone. Return FALSE if failed.
PNLIB_API PNBOOL PNBindSensor(int avatarIndex, int boneIndex, int sensorId);

// Remove sensor from bone. Return FALSE if failed.
PNLIB_API PNBOOL PNRemoveSensor(int avatarIndex, int boneIndex);

// Reset sensors binding to default relationship with bones. Return FALSE if failed.
PNLIB_API PNBOOL PNResetBoneMapping(int avatarIndex);

// boneIndex从0开始
PNLIB_API char* PNGetBoneName(int avatarIndex, int boneIndex);

// sensorId从1开始
PNLIB_API char* PNGetBoneNameBySensorId(int avatarIndex, int sensorId);

// boneIndex从0开始，sensorId从1开始
// 如果骨骼上未绑定传感器，则返回0，否则返回一个大于0的数
PNLIB_API int PNGetSensorId(int avatarIndex, int boneIndex);

// Get bone id by sensor id. Return -1 if this sensor was not binded to a bone.
PNLIB_API int PNGetBoneIdBySensor(int avatarIndex, int sensorId);

// Get hip width of avatar
PNLIB_API float PNGetHipWidth(int avatarIndex);

// Get hip hight of avatar, the hight is the length of hip from ground
PNLIB_API float PNGetHipHight(int avatarIndex);

// Get shoulder width of avatar
PNLIB_API float PNGetShoulderWidth(int avatarIndex);

// Get heel hight of avatar
PNLIB_API float PNGetHeelHight(int avatarIndex);

// Get direction of avatar at calibrated time
PNLIB_API void PNGetInitiationDirection(int avatarIndex, Vector3_t* zd);

// Get the left direction of avatar at calibrated time
PNLIB_API void PNGetInitiationLeftDirection(int avatarIndex, Vector3_t* xd);

// 基于现有传感器，是否能完成CrouchingPose校准
PNLIB_API PNBOOL PNCanCalibratePose(int avatarIndex, CalibrationTypes type);
// 是否上半身
PNLIB_API PNBOOL PNIsUpperBody(int avatarIndex);
// 是否下半身
PNLIB_API PNBOOL PNIsLowBody(int avatarIndex);
// 是否全身
PNLIB_API PNBOOL PNIsFullBody(int avatarIndex);

// Get or set calibration data
PNLIB_API void PNGetCalibrationData(int avatarIndex, CalibrationData* data);
PNLIB_API void PNSetCalibrationData(int avatarIndex, CalibrationData* data);

// 清除积分状态
PNLIB_API void PNClearIntegralState(int avatarIndex);

// 数据输出频率的比率
// ratio: 1=1/1、2=1/2、4=1/4、8=1/8
PNLIB_API void PNSetDataOutputFrequencyRatio(int ratio);

// Get ratio of output data frequency
PNLIB_API int PNGetDataOutputFrequencyRatio();


/**********************************************************
 *             Calculation parameters settings            *
 **********************************************************/
// 获取Kalman参数
PNLIB_API KalmanParams* PNGetKalmanParams(int avatarIndex);
// 设置Kalman参数
PNLIB_API void PNSetKalmanParams(int avatarIndex, KalmanParams params);
// 重置Kalman参数
PNLIB_API void PNResetKalmanParams(int avatarIndex);

// 阻止P发散的阻尼系数
PNLIB_API void PNSetPDamping(int avatarIndex, double pdaming);

// 
PNLIB_API double PNGetPDamping(int avatarIndex);

// 设置jointStiffness
PNLIB_API void PNSetJointStiffness(int avatarIndex, double percent);    
// 设置StepStiffness
PNLIB_API void PNSetStepStiffness(int avatarIndex, double percent);    
// 设置StepConstraint
PNLIB_API void PNSetStepConstraint(int avatarIndex, double percent);
// 获取kalm因子
PNLIB_API void PNGetStiffnessPercent(int avatarIndex, double* jointStiffnessPercent, 
                                                        double* stepStiffnessPercent, 
                                                        double* stepConstraintPercent);


/**********************************************************
 *                    Smooth settings                     *
 **********************************************************/
// 激活平滑功能
PNLIB_API void PNEnableSmoothFilter(int avatarIndex, PNBOOL enable);
// 检查是否开启smooth功能
PNLIB_API PNBOOL PNIsSmoothFilterEnabled(int avatarIndex);
// 设置平滑系数
PNLIB_API void PNSetSmoothFactor(int avatarIndex, SmoothFactors smoothFactors);
PNLIB_API void PNGetSmoothFactor(int avatarIndex, SmoothFactors* smoothFactors);



/**********************************************************
 *                  Mamange drawing scene                 *
 **********************************************************/
// Create and manage drawing scene of model(s)
PNLIB_API void* PNCreateBvhPlayer(BVHWindowContainerRef hWndParent);

// Splite and duplicate drawing scene
PNLIB_API void PNSetBvhPlayerCameras(int cameras);

// Bind/unbind camera to an avatar
PNLIB_API void PNEnableBvhPlayerCameraBind(int camera, int avatarIndex, PNBOOL enableBind);

// Resize drawing window to fill the containner
PNLIB_API void PNBvhPlayerResizeToParent();

// Release drawing scene
PNLIB_API void PNCloseBvhPlayer();

// Enable/disable mass showing for avatar
PNLIB_API void PNEnableMassShowing(int camera, int avatarIndex, PNBOOL enable);

// 是否打开绘图功能
PNLIB_API void PNEnableRendering(PNBOOL enable);


/**********************************************************
 *                   Realtime data interface              *
 **********************************************************/
// Sensor check
PNLIB_API void PNAvailableSensorCheck();

// Get check result
// return: 1 is available, 0 is unavailable, -1 is out of range
PNLIB_API int PNIsSensorAvailable(int sensorId);

// 将从串口读取的数据包压入kalman库
PNLIB_API void PNPushData(unsigned char* data, int len);
    
// 从串口录入对应角色的数据
PNLIB_API void PNPushDataForAvatar(int avatarIndex, unsigned char* data, int len);

// 是否激活丢包补偿
PNLIB_API void PNEnableLostDataFitting(PNBOOL enable);

// 单个角色姿态校准
PNLIB_API void PNCalibrateAvatar(int avatarIndex, CalibrationTypes type);
    
// 所有角色姿态校准
PNLIB_API void PNCalibrateAllAvatars(CalibrationTypes type);

// 获取某个角色的所有传感器的得包率，返回传感器数（命名待定）
PNLIB_API int PNGetSensorReceivingStatus(int avatarIndex, double bufferForPecentData[FULL_BODY_BONE_COUNT]);


/**********************************************************
 *                 Raw data playing interface             *
 **********************************************************/
// 打开原始文件，返回文件中的角色数
PNLIB_API int PNOpenRawDataFile(char* filename);

// 获取文件总帧数
PNLIB_API int PNRawDataPlayGetTotalFrames();

// 获取文件中当前的节点模式
PNLIB_API SensorSuitTypes PNGetSensorSuitType();

// Get node combination mode
PNLIB_API NodeCombinationModes PNGetNodeCombinationMode(int avatarIndex);

// 是否使用真实时间（高精度计时器）播放raw文件
PNLIB_API void PNUseRealTimeToPlayRawFile(PNBOOL enable);

PNLIB_API int PNRawDataPlayGetCurrentPlayingPosition();
// 将播放位置移动到pos，如果此帧已经经过积分，则返回TRUE，否则返回FALSE
// pos 允许从0 到 totalFrames-1
PNLIB_API PNBOOL PNRawDataPlaySetPlayingPosition(int pos);

// 返回从0 到 totalFrames-1的帧序号，如果不存在则返回-1
PNLIB_API int PNRawDataPlayGetPlayingPosition();

// 设置播放速度倍数
PNLIB_API void PNRawDataPlaySetSpeed(double ratio);

// 开始数据播放
PNLIB_API void PNRawDataPlayStart();
    
// 暂停原始数据播放
PNLIB_API void PNRawDataPlayPaus();
    
// 停止原始数据播放
PNLIB_API void PNRawDataPlayStop();

// 开启数据倒序播放
PNLIB_API void PNRawDataPlayEnableReversePlaying(PNBOOL enable);

// 移到前一帧，返回前一帧的帧序号
PNLIB_API int PNRawDataPlaySetToPrev();

// 移到下一帧，返回下一帧的帧序号
PNLIB_API int PNRawDataPlaySetToNext();

// 关闭原始文件并转换到实时模式
PNLIB_API void PNCloseRawDataFile();


/**********************************************************
 *                   Constraint editing                   *
 **********************************************************/
PNLIB_API PNSTATUS PNEditContact(int avatarIndex, int frameIndex , ConstraintPoint point, PNBOOL isContact);
PNLIB_API PNSTATUS PNGetContactStatus(int avatarIndex, int frameIndex, ConstraintPoint point, PNBOOL* isContact);
PNLIB_API PNSTATUS PNBatchEditContact(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point, PNBOOL isContact);
PNLIB_API PNSTATUS PNBatchCancelEdit(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point);
PNLIB_API PNSTATUS PNCancelContactEdit(int avatarIndex, int frameIndex, ConstraintPoint point);
PNLIB_API PNSTATUS PNRefreshContactEdit(int avatarIndex, int frameIndex);


/**********************************************************
 *                 Data export operations                 *
 **********************************************************/
// 将原始数据导出到文件（默认导出一个总的raw文件及各个角色的分离raw文件）
PNLIB_API char* PNExportRawData();
PNLIB_API void  PNStopExportRawData();

// 导出中间数据格式文件
PNLIB_API char* PNExportCalculationData(int avatarIndex);
PNLIB_API void  PNStopExportCalculationData(int avatarIndex);

// Export for BVH data, return file name
PNLIB_API char* PNExportBvhData(int avatarIndex);
PNLIB_API void  PNStopExportBvhData(int avatarIndex);

// 导出fbx格式数据
PNLIB_API char* PNExportFbxData(int avatarIndex);
PNLIB_API void  PNStopExportFbxData(int avatarIndex);

// 设置bvh数据格式
PNLIB_API void PNSetBvhDataFormat(PNBOOL isWithDisp, RotateOrders order);
    
// 输出bvh是否前缀一个reference
PNLIB_API void PNSetBvhDataWithReference(PNBOOL withReference);

// 带位移的BVH输出的位移为世界坐标系的还是局部坐标系的
PNLIB_API void PNEnableBvhDataGlobalDisplacement(int avatarIndex, PNBOOL isGlobalDips);

PNLIB_API void PNRotateFaceDirection(int avatarIndex, double yaw);

// 调节模型的倾斜度（左右、前后）
PNLIB_API void PNRotateModel(int avatarIndex, double pitch, double roll);// 调节模型朝向

PNLIB_API void PNZeroOutAllAvatar();                    // XY平面上归零

PNLIB_API void PNZeroOutPosition(int avatarIndex);      // XY平面上归零


/**********************************************************
 *                    Action Recognition                  *
 **********************************************************/
// 是否激活动作识别
PNLIB_API void PNEnableActionRecognition(PNBOOL enable);

// 设置脚跟高度系数
PNLIB_API void PNSetHeelHRatio(int avatarIndex, double percent);


//==========以下为定制函数，后期可能去除==========

// 获取骨骼某时刻的位置和朝向矢量
// 获取动作识别时发起动作的骨骼位置（可能废弃或做额外的功能划分，按用户需求确定是否发布）
PNLIB_API void PNGetBonePosition(int avatarIndex, int boneId, int eventCode, double* posX, double* posY, double* posZ, double* vectX, double* vectY, double* vectZ);

// 是否激活bvh输出的hip位移匀速运动的功能
PNLIB_API void PNEnableBvhMoving(PNBOOL enable);

// 匀速运动倍速
PNLIB_API void PNSetBvhMovingV(double v);

// 是否激活高尔夫球杆绘图
PNLIB_API void PNEnableGolfCue(PNBOOL enable);

// 将修改后的Xt加入迭代
PNLIB_API void PNUpdateXt(int avatarIndex, float Xt[126]);
//===============================================


/**********************************************************
 *                     Prob management                    *
 **********************************************************/
PNLIB_API void PNSetProbInfo(int avatarIndex, int probId, int sensorId, int parentId, double probLength);

// BVH捏脚测试
PNLIB_API void PN_TEST_EnableBVHFootLock(PNBOOL enable);
PNLIB_API void PN_TEST_Set_h0(double h0);

#ifdef __cplusplus
};
#endif