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
PNLIB_API const BoneInfo* PNGetStandardBoneTable();

// Set sensor combination mode. PNCheckSensorBindingMode will be called in this function to check the current sensor binding if is compfortable with the refered mode.
PNLIB_API PNBOOL PNSetSensorCombinationMode(int avatarIndex, SensorCombinationModes mode);

// Set temporary folder or data export default folder.
// Calibration data will save to the temporary folder.
// Exoprted data file will be saved to the 'workingFolder'.
PNLIB_API void PNSetDataFolders(char* appDataFolder, char* workingFolder);
    
// Set data acquisition frequency. Unit: Hz
PNLIB_API void PNSetDataAcquisitionFrequency(int freq);

// Get data acquisition frequency.
PNLIB_API int PNGetDataAcquisitionFrequency();

// Set or switch run mode between realtime mode and raw file playing mode
// (打开文件前必须显式的指定为播放模式)
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
PNLIB_API void PNEnableMagneticImmune(int avatarIndex, MagneticImmunityLevel level);

// 设置脊柱平滑系数, breastbone:胸椎平滑系数(0~1), vertebra:脊椎平滑系数(0~1)
PNLIB_API void PNSetSpineSmoothFactors(int avatarIndex, float breastbone, float vertebra);

// The default accelerator type of bones is SmallRange, modify it if using different binding of sensor(s)
PNLIB_API void PNSetSensorAcceleratorType(int avatarIndex, int boneId, SensorAcceleratorTypes type);

// Get sensor type
PNLIB_API SensorAcceleratorTypes PNGetSensorAcceleratorType(int avatarIndex, int boneId);

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
    
// 设置接触信息回调函数
PNLIB_API void PNRegisterContactDataCallback(void* customObject, PNEventContactDataCallback handler);

// 设置动作识别事件回调函数
PNLIB_API void PNRegisterActionRecognitionStringDataBoardcastHandle(void* customObject, PNEventActionRecognitionDataStringStreamCallback handle);

// 设置重心矢量回调函数
PNLIB_API void PNRegisterBodyCenterMassCallback(void* customObject, PNEventBodyCenterMassCallback handle);

// Enable/disable center of mass vector callback
PNLIB_API void PNEnableBodyCenterMassDataBoardcast(PNBOOL enable);

// 身体倾斜矢量回调函数
PNLIB_API void PNRegisterBodySwingVectorStringCallback(void* customObject, PNEventBodySwingVectorStringCallback handle);

// Register prop data callback handle if prop node enabled
PNLIB_API void PNRegisterPropDataCallbackHandle(void* customObject, PNEventPropDataCallback handle);

/**********************************************************
 *                     Avatar management                  *
 **********************************************************/
// Create a avatar in scene
PNLIB_API int PNCreateAvatar();

// Delete a avatar from scene
PNLIB_API int PNRemoveAvatar(int avatarIndex);

// Get total avatars count in current scene
PNLIB_API int PNGetAvatarCount();

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

// Get avatar's bvh inheritance
PNLIB_API const char* PNGetBvhInheritanceTxt(int avatarIndex);

// Get avatar's bone length with name
PNLIB_API void PNGetBVHBoneInfo(int avatarIndex, int bvhBoneIndex, BoneInfo* boneInfo);

// Binding a sensor to bone. Return FALSE if failed.
PNLIB_API PNBOOL PNBindSensor(int avatarIndex, int boneIndex, int sensorId);

// Remove sensor from refered bone
PNLIB_API PNBOOL PNRemoveSensor(int avatarIndex, int boneIndex);

// Check the refered bone if is bound sensor
PNLIB_API PNBOOL PNIsBindingSensor(int avatarIndex, int boneIndex);

// Check the current sensor binding if is comfortable with the refered sensor combination mode
// For SC_ArmOnly mode, left upper arm and left forearm is nessory, or, right upper arm and right forearm is nessory. 2 nodes.
// For SC_UpperBody mode, chest, hip, left upper arm and left forearm is nessory, or, chest, hip, right upper arm and right forearm is nessory. 4 nodes.
// For SC_LowerBody mode, hip, double leg is nessory.
// For SC_FullBody mode, left upper leg, left leg, right upper leg, right leg, hip and chest sensor is necessory. 6 nodes.
PNLIB_API PNBOOL PNCheckSensorBindingMode(int avatarIndex, SensorCombinationModes mod);

// Reset sensors binding to default relationship with bones. Return FALSE if failed.
PNLIB_API PNBOOL PNResetBoneMapping(int avatarIndex);

// boneIndex从0开始
PNLIB_API char* PNGetBoneName(int avatarIndex, int boneIndex);

// sensorId从1开始
PNLIB_API char* PNGetBoneNameBySensorId(int avatarIndex, int sensorId);

// Get sensor ID based on 1 from refered bone. Return 0 means no sensor bound on the refered bone, -1 means error occured. 
PNLIB_API int PNGetSensorId(int avatarIndex, int boneIndex);

// Get bone id by sensor id. Return -1 if this sensor was not binded to a bone.
PNLIB_API int PNGetBoneIndexBySensorId(int avatarIndex, int sensorId);

// Get hip width of avatar
PNLIB_API float PNGetHipWidth(int avatarIndex);

// Get hip height of avatar, the height is the length of hip from ground
PNLIB_API float PNGetHipHeight(int avatarIndex);

// Get shoulder width of avatar
PNLIB_API float PNGetShoulderWidth(int avatarIndex);

// Get heel height of avatar
PNLIB_API float PNGetHeelHeight(int avatarIndex);

// Get direction of avatar at calibrated time
PNLIB_API Vector3_t PNGetCalibratedBodyDirection(int avatarIndex);

// Get the left direction of avatar at calibrated time
PNLIB_API Vector3_t PNGetCalibratedBodyLeftDirection(int avatarIndex);

// Check the current sensor binding if is comfortable with the refered calibration type
PNLIB_API PNBOOL PNCanCalibratePose(int avatarIndex, CalibrationTypes type);

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
PNLIB_API void PNSetPDamping(int avatarIndex, float pdamping);

// 
PNLIB_API float PNGetPDamping(int avatarIndex);

// 设置地面高度，单位：米
PNLIB_API void PNSetGroundHeight(int avatarIndex, float height);

// 获取地面高度，单位：米
PNLIB_API float PNGetGroundHeight(int avatarIndex);

// 设置jointStiffness
PNLIB_API void PNSetJointStiffness(int avatarIndex, float percent);    
// 设置StepStiffness
PNLIB_API void PNSetStepStiffness(int avatarIndex, float percent);    
// 设置StepConstraint
PNLIB_API void PNSetStepConstraint(int avatarIndex, float percent);
// 获取kalm因子
PNLIB_API void PNGetStiffnessPercent(int avatarIndex,
	float* jointStiffnessPercent,
	float* stepStiffnessPercent, 
	float* stepConstraintPercent);

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

// 是否在脚面接触地面时捏住不动
PNLIB_API void PNEnableFootLock(PNBOOL enable);
// 获取当前脚部是否锁定的状态
PNLIB_API PNBOOL PNIsEnabledFootLock();

// 是否启用手指反关节处理
PNLIB_API void PNEnableFingerAntiJoint(PNBOOL enable);
// 获取当前手指反关节处理开关状态
PNLIB_API PNBOOL PNIsEnableFingerAntiJoint();

/**********************************************************
 *                  Mamange drawing scene                 *
 **********************************************************/
// Create and manage drawing scene of model(s)
PNLIB_API void* PNCreateBvhPlayer(BVHWindowContainerRef hWndParent);

// Splite and duplicate drawing scene
PNLIB_API void PNSetBvhPlayerCameras(int cameras);

// Bind/unbind camera to an avatar
PNLIB_API void PNEnableBvhPlayerCameraBind(int cameraIndex, int avatarIndex, PNBOOL enableBind);

// Resize drawing window to fill the containner
PNLIB_API void PNBvhPlayerResizeToParent();

// Release drawing scene
PNLIB_API void PNCloseBvhPlayer();

// Enable/disable mass showing for avatar
PNLIB_API void PNEnableMassShowing(int avatarIndex, PNBOOL enable);

// 是否打开绘图功能
PNLIB_API void PNEnableRendering(PNBOOL enable);


/**********************************************************
 *                   Realtime data interface              *
 **********************************************************/
// Push raw data into PNLib
PNLIB_API void PNPushData(unsigned char* data, int len);
    
// Push avatar's raw data into PNLib
PNLIB_API void PNPushDataForAvatar(int avatarIndex, unsigned char* data, int len);

// Enable/disable fitting of lost data
PNLIB_API void PNEnableLostDataFitting(PNBOOL enable);

// Clear the buffered data used to calibration
// Before a new calibration, clear the preview buffered data by this function is required
PNLIB_API void PNResetCalibrationSteps(int avatarIndex);

// Start a calibrating action of avatar
PNLIB_API void PNCalibrateAvatar(int avatarIndex, CalibrationTypes type);

// Start a calibrating action for all avatar
PNLIB_API void PNCalibrateAllAvatars(CalibrationTypes type);

// Get receiving percent of sensor data
PNLIB_API int PNGetSensorReceivingStatus(int avatarIndex, float bufferForPercentData[FULL_BODY_BONE_COUNT]);

/**********************************************************
 *                     Raw file interface                 *
 **********************************************************/
// Get the refered raw file information. Such as recode time, sensor type, freq, avatar count, etc.
PNLIB_API RawFileInfo* PNGetRawFileInfo(char* filename);

// Open raw file to replay the captured motion data, return the avatar number in this motion data file
PNLIB_API int PNOpenRawDataFile(char* filename);

// Get total frames in opened raw file
PNLIB_API unsigned long PNRawDataPlayGetTotalFrames();

// Get sensor suit type in opened raw file
PNLIB_API SensorSuitTypes PNGetSensorSuitType();

// Get node combination mode
PNLIB_API SensorCombinationModes PNGetSensorCombinationMode(int avatarIndex);

// 将播放位置移动到pos，如果超出范围返回FALSE
// pos 允许从0 到 totalFrames-1
PNLIB_API PNBOOL PNRawDataPlaySetPlayingPosition(int pos);

// 返回从0 到 totalFrames-1的帧序号，如果不存在则返回-1
PNLIB_API unsigned long PNRawDataPlayGetPlayingPosition();

// 设置播放速度倍数
PNLIB_API void PNRawDataPlaySetSpeed(float ratio);

// 开始数据播放
PNLIB_API void PNRawDataPlayStart();
    
// 暂停原始数据播放
PNLIB_API void PNRawDataPlayPause();
    
// 停止原始数据播放
PNLIB_API void PNRawDataPlayStop();

// 控制播放顺序（正播为FALSE或者倒播TRUE）
PNLIB_API void PNRawDataEnableReversePlay(PNBOOL enable);

// 移到前一帧，返回前一帧的帧序号
PNLIB_API unsigned long PNRawDataPlaySetToPrev();

// 移到下一帧，返回下一帧的帧序号
PNLIB_API unsigned long PNRawDataPlaySetToNext();

// 关闭原始文件
PNLIB_API void PNCloseRawDataFile();


/**********************************************************
 *                   Constraint editing                   *
 **********************************************************/
PNLIB_API PNSTATUS PNEditContact(int avatarIndex, int frameIndex , ConstraintPoint point, PNBOOL isContact);
PNLIB_API PNSTATUS PNGetContactStatus(int avatarIndex, int frameIndex, ConstraintPoint point, PNBOOL* isContact);
PNLIB_API PNSTATUS PNBatchEditContact(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point, PNBOOL isContact);
PNLIB_API PNSTATUS PNBatchResetContactEditStatus(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point);
PNLIB_API PNSTATUS PNResetContactEditStatus(int avatarIndex, int frameIndex, ConstraintPoint point);

// Optimization feet constraint with the specified level. 0 means reset to no optimized status. 10 is the max level can be used
PNLIB_API void PNSetFeetContactOptimizationLevel(int level);

PNLIB_API int PNGetFeetContactOptimizationLevel();
/**********************************************************
 *                 Data export operations                 *
 **********************************************************/
// 将原始数据导出到文件（默认导出一个总的raw文件及各个角色的分离raw文件）
PNLIB_API char* PNExportRawData();
PNLIB_API void  PNStopExportRawData();

// 导出某个人的某个传感器对应的中间数据
PNLIB_API char* PNExportRawDataTxt(int avatarIndex, int sensorId);
PNLIB_API void  PNStopExportRawDataTxt(int avatarIndex);

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

// BVH data output compression flag
PNLIB_API void PNBvhBinaryDataOutputIsCompression(PNBOOL isCompression);

// 带位移的BVH输出的位移为世界坐标系的还是局部坐标系的
PNLIB_API void PNEnableBvhDataGlobalDisplacement(int avatarIndex, PNBOOL isGlobalDips);

// 调节每个模型的在绘图中的位置
PNLIB_API void PNAdjustPosition(int avatarIndex, float x, float y, float z);

// 
PNLIB_API void PNRotateFaceDirection(int avatarIndex, float yaw);

// 当调节模型朝向时，是否绕原点或者自身旋转，默认绕自身旋转
PNLIB_API void PNEnableSpinWithOrigin(int avatarIndex, PNBOOL enable);

// 调节模型的倾斜度（左右、前后）
PNLIB_API void PNRotateModel(int avatarIndex, float pitch, float roll);// 调节模型朝向

PNLIB_API void PNZeroOutAllAvatar();                    // XY平面上归零

PNLIB_API void PNZeroOutPosition(int avatarIndex);      // XY平面上归零


/**********************************************************
 *                     Prop management                    *
 **********************************************************/
// Enable/Disable refered prop.
PNLIB_API PNBOOL PNPropEnable(int avatarIndex, PropIdentity propId, PNBOOL enable);
// Set parent of prop.
PNLIB_API PNBOOL PNPropSetParent(int avatarIndex, PropIdentity propId, int parentBoneId);
// Bind sensor onto prop.
PNLIB_API PNBOOL PNPropBindSensor(int avatarIndex, PropIdentity propId, int sensorId);
// Remove sensor from prop.
PNLIB_API PNBOOL PNPropRemoveSensor(int avatarIndex, PropIdentity propId);
// Set length of prop.
PNLIB_API PNBOOL PNPropSetLength(int avatarIndex, PropIdentity propId, float probLength);
// Set initial offset of prop from parent end point.
PNLIB_API PNBOOL PNPropSetEndOffset(int avatarIndex, PropIdentity propId, Vector3_t offset);
// Set initial direction of prop.
PNLIB_API PNBOOL PNPropSetDirection(int avatarIndex, PropIdentity propId, Vector3_t direction);
// Set all parameters of prop.
PNLIB_API PNBOOL PNPropSetInfo(int avatarIndex, PropIdentity propId, PropInformation* pi);
// Get all parameters of prop.
PNLIB_API PNBOOL PNPropGetInfo(int avatarIndex, PropIdentity propId, PropInformation* piBuff);
// Handle prop node as a BVH child node. Default is 'true'.
PNLIB_API PNBOOL PNPropHandleNodeAsBVHPart(int avatarIndex, PropIdentity propId, PNBOOL enable);


/**********************************************************
 *                    Action Recognition                  *
 **********************************************************/
// 是否激活动作识别
PNLIB_API void PNEnableActionRecognition(PNBOOL enable);

// 获取世界坐标系下骨骼的位置和朝向矢量
PNLIB_API void PNGetBonePosition(int avatarIndex, int boneId, int eventCode, Vector3_t* bonePos, Vector3_t* boneDirection);

// 是否激活手指游戏模式
PNLIB_API void PNEnableGameModeForFingers(PNBOOL enable);

// 查询当前是否激活了手指游戏模式
PNLIB_API PNBOOL PNIsEnableGameModeForFingers();


//==========以下为定制函数，后期可能去除==========

// 是否激活bvh输出的hip位移匀速运动的功能
PNLIB_API void PNEnableBvhMoving(int avatarIndex, PNBOOL enable);

// 匀速运动倍速
PNLIB_API void PNSetBvhMovingV(int avatarIndex, float v);

// 是否激活高尔夫球杆绘图
PNLIB_API void PNEnableGolfCue(PNBOOL enable);

// 将修改后的Xt加入迭代
PNLIB_API void PNUpdateXt(int avatarIndex, float Xt[126]);

PNLIB_API void PN_TEST_Set_h0(float h0);
//===============================================
#ifdef __cplusplus
};
#endif
