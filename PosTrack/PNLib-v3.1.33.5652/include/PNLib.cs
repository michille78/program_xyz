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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Runtime.InteropServices;

// Wrapper of PNLib SDK for C#
namespace PNLibCSharpImporter
{
    #region delegates
    public delegate void PNEventBVHStringDataBoardcastCallback(IntPtr customObject, int avatarID, string data);
    public delegate void PNEventBVHBinaryDataBoardcastCallback(IntPtr customObject, int avatarID, IntPtr pack, int packLen);
    public delegate void PNCalculationDataCallback(IntPtr customObject, string data); // 中间数据导出 callback
    public delegate void PNCalculatedBinaryDataCallback(IntPtr customObject, int avatarIndex, IntPtr pack, int packLen);
    public delegate void PNCalibrationCallback(IntPtr customObject, int avatarIndex, float percent);
    public delegate void PNMatrixDataCallback(IntPtr customObject, int avatarIndex, string matData); // 实时M矩阵数据callback
    public delegate void PNEventPropDataCallback(IntPtr customObject, int avatarIndex, ref PropData propData);
    // 播放进度
    public delegate void PNPlayProgressCallback(IntPtr customObject, int currentFrame, int totalFrames);

    // 接触信息回调函数
    public delegate void PNEventContactDataCallback(IntPtr customObject, int avatarIndex, ref FrameContactData statusData);

    public delegate void PNRawDataParsedCallback(IntPtr customObject, int moduleId, [MarshalAs(UnmanagedType.LPArray, SizeConst = 4)]float[] q, [MarshalAs(UnmanagedType.LPArray, SizeConst = 3)]float[] w, [MarshalAs(UnmanagedType.LPArray, SizeConst = 3)]float[] a);

    /// <summary>
    /// 识别事件的回调函数
    /// </summary>
    /// <param name="eventCode">事件代码</param>
    /// <param name="direction">发射朝向</param>
    /// <param name="position">发射位置</param>
    /// <param name="level">力度等级</param>
    public delegate void PNRecognizedEvent(IntPtr customObject, int avatarIndex, int boneId, int eventCode, int level);
    // 字符串方式
    public delegate void PNEventActionRecognitionDataStringStreamCallback(IntPtr customObject, int avatarIndex, [MarshalAs(UnmanagedType.LPStr)]string data);
    // 
    public delegate void PNBodySwingVectorCallback(IntPtr customObject, int avatarIndex, int moduleId, [MarshalAs(UnmanagedType.LPArray, SizeConst = 3)]float[] vect);
    public delegate void PNBodySwingVectorStringCallback(IntPtr customObject, int avatarIndex, [MarshalAs(UnmanagedType.LPStr)]string data);

    public delegate void PNBodyMassVectorCallback(IntPtr customObject, int avatarIndex, float x, float y, float z);
    public delegate void PNBodyMassVectorStringCallback(IntPtr customObject, int avatarIndex, [MarshalAs(UnmanagedType.LPStr)]string data);
    #endregion

    #region Data types
    public enum RunningMode
    {
        RM_Realtime,
        RM_RawPlaying,
        RM_Unknown,           // Unknown type
    }

    // 传感器套件模式
    public enum SensorSuitTypes
    {
        SS_LegacyNodes,
        SS_NeuronNodes,
    };

    // Node combination types
    public enum SensorCombinationModes
    {
        SC_ArmOnly,              // Left arm or right arm only
        SC_UpperBody,            // Upper body, include one arm or both arm, must have chest node
        SC_FullBody,             // Full body mode
        SC_LowerBody,            // ower body, sensors of legs and hips are necessory
        SC_Unknown,              // Unknown mode
    };
    // bvh rotate order
    public enum RotateOrders
    {
        XZY,
        YXZ,
        XYZ,
        YZX,
        ZXY,
        ZYX,
    };
    
    public enum ExportDataFormat
    {
        fbx = 0,
    }

    public enum NetDataOutputServiceTypes
    {
        TCP,
        UDP,
    };

    public enum NetDataOutputBlockTypes
    {
        BinaryType,
        StringType,
    };

    // 校准动作类型定义
    public enum CalibrationTypes
    {
        Cali_TPose,  // T pose
        Cali_APose,  // A pose
        Cali_SPose,  // 屈膝校准
        Cali_NPose,  // 点头校准
    };
    
    // Quaternion types of output stream
    public enum OutputQuaternionTypes
    {
        QT_GlobalRawQuat,        // Module Q, by global system
        QT_GlobalBoneQuat,       // Calibrated module Q(Bone Q) 
        QT_LocalBoneQuat,        // Quaternion with parent
    };

    // Acceleration data types of output stream
    public enum OutputAccelerationTypes
    {
        AT_ModuleRawData,        // Module A, by global system
        AT_GlobalData,           // Calibrated module A(Bone A) 
    };

    // Gyro data types of output stream
    public enum OutputGyroType
    {
        GY_ModuleRawData,        // Module W, by global system
        GY_GlobalData,           // Calibrated module W(Bone W) 
    };

    // Prop identities
    public enum PropIdentity
    {
        PI_Prop0,                // Prop0
	    PI_Prop1,                // Prop1
	    PI_Prop2,                // Prop2
	    PI_Prop3,                // Prop3
	    PI_Prop4,                // Prop4
	    PI_RightShoulderExtra,   // Right shoulder extra
	    PI_LeftShoulderExtra,    // Left shoulder extra
        PI_Unknown,              // Unknown type
    };
        

    public enum MagneticImmunityLevel
    {
        DisableMI, // 不使用磁免疫
        WeakMI,    // 弱磁免疫（只用在脚上）
        StrongMI,  // 强磁免疫（用在脚和小腿上）
    };
    
    // 接触点枚举
    public enum ConstraintPoint
    {
        Hip = 0, // Hip
        FR = 3,  // Right foot
        FL = 6,  // Left foot
        HR = 10, // Right hand
        HL = 14, // Left hand
    };

    public struct Vector3_t
    {
        public float x;
        public float y;
        public float z;
    };

    public struct Quaternion4_t
    {
        public float s;
        public float x;
        public float y;
        public float z;
    };

    // 身体骨骼尺寸（单位：厘米）
    public struct BoneDimension
    {
        public float Head;          
        public float Neck;          
        public float Body;          
        public float ShoulderWidth; 
        public float UpperArm;      
        public float Forearm;       
        public float Palm;          
        public float HipWidth;      
        public float UpperLeg;      
        public float LowerLeg;      
        public float HeelHeight;    
        public float FootLength;    
    };

    // Bone size
    public struct BVHBoneInfo
    {
        public int Index;             // Bone Index in standard bone table
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 60)]
        public string Name;           // Bone name
        public float Length;          // Bone length
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct EditingPoint
    {
        [FieldOffset(0)]
        public ConstraintPoint Point;
        [FieldOffset(4)]
        public bool IsEdit;
        [FieldOffset(8)]
        public bool IsContact;
        [FieldOffset(12)]
        public bool IsUseDefault;
    }

    // dynamic library version info
    [StructLayout(LayoutKind.Sequential)]
    public struct DylibVersion
    {
        public ushort Major;            // Major number
        public ushort Minor;            // Minor number
        public ushort Revision;         // Revision number
        public ushort Build;            // Build number
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct ContactStatus
    {
        public int PointId;
        public bool IsEdited;
        public bool IsContact;
    };

    public struct FrameContactData
    {
        public int frameIndex;
        public ContactStatus Hip;
        public ContactStatus RightFoot;
        public ContactStatus LeftFoot;
        public ContactStatus RightHand;
        public ContactStatus LeftHand;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct SmoothFactors
    {
        public int GlobalDisplacement;    // Smooth factor of Displacement for all bones
        public int GlobalRotation;      // Smooth factor of rotation for all bones
        public int HipDisplacement;       // Smooth factor of Displacement specially for hip, global value is override by this value
        public int FeetDisplacement;      // Smooth factor of rotation specially for feet, global value is override by this value
        public int HipRotation;
        public int FeetRotation;
    };


    [StructLayout(LayoutKind.Sequential)]
    public struct PropInformation
    {
	    public bool IsEnable;          // Whether enabled
	    public int ParentBoneId;       // Parent bone id of prop
	    public int SensorId;           // Sensor Id bound to this prop
	    public float Length;           // Prop length, unit: meter
	    public Vector3_t Rotation;     // Prop initial face direction
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct PropData
    {
        int           BoneId;          // Id of prop in standard bone table
        int           SensorId;        // Sensor bound to this prop bone
        Quaternion4_t Rotation;        // Ratation of sensor
        Vector3_t     Acc;             // Acceleration of senosr
        Vector3_t     Gyro;            // Gyro of sensor
        Vector3_t     BoneStart;       // Start position of prop in parent coordinate system
        Vector3_t     BoneEnd;         // End position of prop
        Vector3_t     BoneAngle;       // Angle relative to the parent node
        Vector3_t     SensorPosition;  // 
        Vector3_t     SensorVelocity;  // 
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct Params
    {
        // 帧间隔
        public float dt;
        public float wERR;

        // 加速度计的误差协方差矩阵初始化;根据加速度传感器datasheet，noise density换算得出
        public float aERR;

        // 设角度观测误差级别为7度
        public float alpha;

        public float dERR;

        // 重力投影所引入的误差，该误差与投影的角度测量相关，而角度测量与q相关，此处假定角度测量误差较大时有alpha度的测量偏差
        public float gERR;

        // 速度衰减系数
        public float damp;

        // 位移积分时的速度衰减系数,由于速度不足，这里做了个补偿
        public float damp1;


        // 增益调整系数;
        public float c;

        // 重力加速度大小,用一个略大的重力加速度压住模型不让丫飘起来
        public float g;

        // 状态误差协方差矩阵初始化；P矩阵会迅速迭代收敛，因此初始化无需精确，随意即可，设置好一点收敛快一些罢了
        public float Perr;

        public int freq;

        // 粘连
        public int stick;

        public float tcrrr;

        public float tol1;

        public float tol2;

        public float tol3;

        public float atol;

        public float vtol;

        // 全身位移k
        public float kBodyX;

        // 全身速度k
        public float kBodyV;

        // 脚部位移k
        public float kFeetX;

        // 脚部速度k
        public float kFeetV;

        // 脚部位移z方向分段下捏系数（kFeetX的一个百分系数）
        public float kFeetZ;

        // 臀部接触高度
        public float hipH;

        // Center of mass
        public float acmm;

        // 是否打开重心判断
        public int massSwitch;

        // 是否捏双脚
        public int constrain;
        // 手部捏与不捏
        public int constrainHands;
        // 臀部捏与不捏
        public int constrainHip;
    }

    // test...
    public struct FootContactData
    {
        public bool IsContact;
        public float FootHeight;
    };

    #endregion

    #region PNLib API Wraper
    public class PNLib
    {
        // Get library's version information
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        [return: MarshalAs(UnmanagedType.Struct)]
        public static extern DylibVersion PNGetLibVersion();

        // Initialize library
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNLibInit();
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNLibRelease();

        /// <summary>
        /// 是否启用传感器数据交换
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void EnableSensorDataExchange(bool enable);
        
        /// <summary>
        /// 设置角色名
        /// </summary>
        /// <param name="name"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetAvatarName(int avatar, string name);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetAvatarName(int avatar, StringBuilder name, int len = 100);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetDataFolders(string appDataDir, string workingFolder);
        
        // 激活上台阶功能
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableClimbContact(int avatarIndex, bool enable);
        // 重置上台阶参数
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNResetClimbContact(int avatarIndex);
        // 激活磁免疫功能
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableMagneticImmune(int avatar, MagneticImmunityLevel level);
        
        // 根据boneId获取sensorId
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNGetSensorId(int avatarIndex, int boneIndex);
        /// <summary;
        /// 根据sensorId返回所在的骨骼编号
        /// </summary>
        /// <param name="sensorId">传感器Id</param>
        /// <returns>传感器所在的骨骼编号，如果传感器未使用则返回-1</returns>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNGetBoneIndexBySensorId(int avatar, int sensorId);
        /// <summary>
        /// 绑定传感器
        /// </summary>
        /// <param name="boneIndex">骨骼编号</param>
        /// <param name="sensorId">传感器编号</param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNBindSensor(int avatar, int boneIndex, int sensorId);
        /// <summary>
        /// 移除传感器
        /// </summary>
        /// <param name="boneIndex">骨骼编号</param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNRemoveSensor(int avatarIndex, int boneIndex);        
        // 重置传感器与骨骼的映射关系
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNResetBoneMapping(int avatarIndex);
        // 基于现有传感器，是否能完成相应动作校准
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNCanCalibratePose(int avatarIndex, CalibrationTypes type);

         // boneIndex从0开始
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        private static extern IntPtr PNGetBoneName(int avatarIndex, int boneIndex);
        public static string GetBoneName(int avatarIndex, int boneIndex)
        {
            IntPtr addr = PNGetBoneName(avatarIndex, boneIndex);
            return Marshal.PtrToStringAnsi(addr);
        }

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        private static extern IntPtr PNGetBoneNameBySensorId(int avatarIndex, int sensorId);
        public static string GetBoneNameBySensorId(int avatarIndex, int sensorId)
        {
            IntPtr addr = PNGetBoneNameBySensorId(avatarIndex, sensorId);
            return Marshal.PtrToStringAnsi(addr);
        }

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNLoadCalibrationData();

        /// <summary>
        /// 设置动作识别事件回调函数
        /// </summary>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void PNRegisterActionRecognitionStringDataBoardcastHandle(IntPtr customObject, PNEventActionRecognitionDataStringStreamCallback handle);


        /// <summary>
        /// 设置动作识别事件回调函数
        /// </summary>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void PNRegisterActionEventBoardcastHandle(IntPtr customObject, PNRecognizedEvent handle);
        
        /// <summary>
        /// 是否激活动作识别
        /// </summary>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void PNEnableActionRecognition(bool enable);

        /// <summary>
        /// 设置采集频率
        /// </summary>
        /// <param name="freq"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetDataAcquisitionFrequency(int freq);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNGetDataAcquisitionFrequency();

        /// <summary>
        /// 数据输出频率的比率
        /// </summary>
        /// <param name="ratio">
        /// 1=1/1
        /// 2=1/2
        /// 4=1/4
        /// 8=1/8
        /// </param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetDataOutputFrequencyRatio(int ratio);
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNPushData(byte[] data, int len);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNPushDataForAvatar(int avatarIndex, byte[] data, int len);

        // 是否激活丢包补偿
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableLostDataFitting(bool enable);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterCalibrationProgressHandle(IntPtr customObject, PNCalibrationCallback PCFn);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNCalibrateAllAvatars(CalibrationTypes type);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNCalibrateAvatar(int avatarIndex, CalibrationTypes type);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNResetCalibrationSteps(int avatarIndex);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNClearAllAvatarCalibrationBufferedData();
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNGetSensorReceivingStatus(int avatar, float[] bufferOfRatio);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Auto, ExactSpelling = true)]
        private static extern IntPtr PNGetKalmanParams(int avatar);
        public static Params GetKalmanParams(int avatarId)
        {
           IntPtr ptr = PNGetKalmanParams(avatarId);
           return (Params)(Marshal.PtrToStructure(ptr, typeof(Params)));
        }

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetKalmanParams(int avatarIndex, Params p);

        // 设置jointStiffness
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetJointStiffness(int avatarIndex, float percent);    
        // 设置StepStiffness
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetStepStiffness(int avatarIndex, float percent);    
        // 设置StepConstraint
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetStepConstraint(int avatarIndex, float percent);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetStiffnessPercent(int avatarIndex, ref float jointStiffnessPercent, 
                                                         ref float stepStiffnessPercent, 
                                                         ref float setpConstraintPercent);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNExportRawData();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNStopExportRawData();
        
        // 导出某个人的某个传感器对应的中间数据
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNExportRawDataTxt(int avatarIndex, int sensorId);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNStopExportRawDataTxt(int avatarIndex);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNExportCalculationData(int avatarIndex);// 导出中间数据
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNStopExportCalculationData(int avatarIndex);// 导出中间数据

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNExportBvhData(int avatarIndex);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNStopExportBvhData(int avatarIndex);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNExportFbxData(int avatarIndex);// 导出fbx格式数据
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNStopExportFbxData(int avatarIndex);// 导出fbx格式数据

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterRawDataPlayingProgressHandle(IntPtr customObject, PNPlayProgressCallback handle);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNOpenRawDataFile(string filename);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNCloseRawDataFile();
        
        // 是否使用真实时间（高精度计时器）播放raw文件
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNUseRealTimeToPlayRawFile(bool enable);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern UInt32 PNRawDataPlayGetTotalFrames();
   
        // 将播放位置移动到pos，如果此帧已经经过积分，则返回true，否则返回false
        // pos 从0 到 totalFrames-1
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNRawDataPlaySetPlayingPosition(int pos);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern UInt32 PNRawDataPlayGetPlayingPosition();

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRawDataPlaySetSpeed(float percent);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern UInt32 PNRawDataPlaySetToPrev();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern UInt32 PNRawDataPlaySetToNext();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRawDataPlayStart();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRawDataEnableReversePlay(bool enable);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRawDataPlayPause();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRawDataPlayStop();

        // 设置单个原始数据包解压回调函数指针
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterPlayingRawDataParsedHandle(IntPtr customObject, PNRawDataParsedCallback handle);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        private static extern void PNSetBoneDimensions(int avatarIndex, IntPtr boneDimensions);
        public static void SetBoneDimensions(int avatarIndex, BoneDimension dimsBuffer)
        {
            int size = Marshal.SizeOf(typeof(BoneDimension));
            IntPtr buffer = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(dimsBuffer, buffer, true); 
            PNSetBoneDimensions(avatarIndex, buffer);
            Marshal.FreeHGlobal(buffer);  
        }

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetBoneDimensions(int avatarIndex, ref BoneDimension dimsBuffer);
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        private static extern  IntPtr PNGetBvhInheritanceTxt(int avatarIndex);
        public static string GetBvhInheritanceTxt(int avatarIndex)
        {
            IntPtr addr = PNGetBvhInheritanceTxt(avatarIndex);
            return Marshal.PtrToStringAnsi(addr);
        }
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetBVHBoneInfo(int avatarIndex, int bvhBoneIndex, ref BVHBoneInfo bvhBoneInfo);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void SetAccOverflow(float reverseValue, float overValue);

        // 设置BVH回调
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterBvhStringDataBoardcastHandle(IntPtr customObject, PNEventBVHStringDataBoardcastCallback handle);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterBvhBinaryDataBoardcastHandle(IntPtr customObject, PNEventBVHBinaryDataBoardcastCallback handle);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetBvhDataBlockBoardcastType(NetDataOutputBlockTypes type);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetBvhDataFormat(bool isWithDisp, RotateOrders order);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetBvhDataWithReference(bool withReference);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNBvhBinaryDataOutputIsCompression(bool isCompression);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableBvhDataBoardcast(bool enable);
        // 带位移的BVH输出的位移为世界坐标系的还是局部坐标系的
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableBvhDataGlobalDisplacement(int avatarIndex, bool isGlobalDips);
        // 设置中间数据的格式
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetCalculatedDataBlockBoardcastType(NetDataOutputBlockTypes type);
        // 设置中间数据输出的对应个类型
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetCalculatedQuaternionDataType(OutputQuaternionTypes type);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetCalculatedAccelerationDataType(OutputAccelerationTypes type);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetCalculatedGyroDataType(OutputGyroType type);

        // 初始化实时中间数据callback
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterCalculatedStringDataBoardcastHandle(IntPtr customObject, PNCalculationDataCallback handle);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterCalculatedBinaryDataBoardcastHandle(IntPtr customObject, PNCalculatedBinaryDataCallback handle);
        // 激活实时中间数据callback
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableCalculationDataBoardcast(bool enable);
        
        
        // 设置Prop数据输出回调函数
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterPropDataCallbackHandle(IntPtr customObject, PNEventPropDataCallback handle);


        // 初始化实时M矩阵数据callback
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterBvhMatrixDataBoardcastHandle(IntPtr customObject, PNMatrixDataCallback handle);
        // 激活实时M矩阵数据callback
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableRealtimeMatrixData();
        // 禁用实时M矩阵数据callback
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNDisableRealtimeMatrixData();

        /// <summary>
        /// 旋转模型
        /// </summary>
        /// <param name="pitch">前后仰角</param>
        /// <param name="roll">左右倾斜角</param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRotateModel(int avatar, float pitch, float roll); // 调节模型朝向
        /// <summary>
        /// 调节模型朝向
        /// </summary>
        /// <param name="yaw">左右朝向</param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRotateFaceDirection(int avatar, float yaw);
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNAdjustPosition(int avatarIndex, float x, float y, float z);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNZeroOutPosition(int avatar);             // 位置归零
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNZeroOutAllAvatar();             // 位置归零
        

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterContactDataCallback(IntPtr customObject, PNEventContactDataCallback handler);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEditContact(int avatarIndex, int frameIndex, ConstraintPoint point, int isConstrain);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetContactStatus(int avatarIndex, int frameIndex, ConstraintPoint point, ref bool isContact);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNBatchEditContact(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point, bool isConstrain);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNBatchResetContactEditStatus(int avatarIndex, int startFrameIndex, int endFrameIndex, ConstraintPoint point);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNResetContactEditStatus(int avatarIndex, int frameIndex, ConstraintPoint point);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNFeetConstraintOptimization(int avatarIndex, uint level);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetSmoothFactor(int avatarIndex, SmoothFactors facters);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableSmoothFilter(int avatarIndex, bool enable);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetSmoothFactor(int avatarIndex, ref SmoothFactors facters);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNIsSmoothFilterEnabled(int avatarIndex);
        /// <summary>
        /// // 是否在脚面接触地面时捏住不动
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableFootLock(bool enable);

        /// <summary>
        /// // 是否处理手指反关节
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableFingerAntiJoint(bool enable);

        /// <summary>
        /// // 是否处理手指反关节
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNIsEnableFingerAntiJoint();

        /// <summary>
        /// 是否激活高尔夫球杆绘图
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableGolfCue(bool enable);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetPDamping(int avatarIndex, float pdaming);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern float PNGetPDamping(int avatarIndex);

        // 设置地面高度，单位：米
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetGroundHeight(int avatarIndex, float height);

        // 获取地面高度，单位：米
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern float PNGetGroundHeight(int avatarIndex);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void RestrictArmRotation(bool be);
        
        /// <summary>
        /// 设置脊柱平滑系数
        /// </summary>
        /// <param name="breastbone">胸椎平滑系数</param>
        /// <param name="vertebra">脊椎平滑系数</param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetSpineSmoothFactors(int avatarIndex, float breastbone, float vertebra);
                
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNGetBonePosition(int avatarIndex, int boneId, int eventCode, ref float posX, ref float posY, ref float posZ, ref float vectX, ref float vectY, ref float vectZ);
        
        /// <summary>
        /// 是否激活bvh输出的hip位移匀速运动的功能
        /// </summary>
        /// <param name="enable"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableBvhMoving(int avatarIndex, bool enable);

        /// <summary>
        /// 匀速运动倍速
        /// </summary>
        /// <param name="v"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetBvhMovingV(int avatarIndex, float v);

        /// <summary>
        /// 身体倾斜矢量回调函数
        /// </summary>
        /// <param name="handle"></param>
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterBodySwingVectorStringCallback(IntPtr customObject, PNBodySwingVectorStringCallback handle);        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNRegisterBodyCenterMassCallback(IntPtr customObject, PNBodyMassVectorStringCallback handle);    

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern IntPtr PNCreateBvhPlayer(IntPtr hWndParent);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetBvhPlayerCameras(int cameras);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNGetAvatarCount();
        // 增加一个角色，返回角色Id
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNCreateAvatar();
        // 删除指定角色，返回剩余的总角色数
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern int PNRemoveAvatar(int avatar);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableMassShowing(int avatarIndex, bool enable);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNBvhPlayerResizeToParent();
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableBvhPlayerCameraBind(int camera, int avatar, bool enable);
    
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNCloseBvhPlayer();

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableRendering(bool enable);

        // 设置模式是全全身模式还是普通的模式
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetSensorSuitType(SensorSuitTypes modes);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern SensorSuitTypes PNGetSensorSuitType();

        // 设置节点组合模式
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetSensorCombinationMode(int avatarIndex, SensorCombinationModes mode);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern SensorCombinationModes PNGetSensorCombinationMode(int avatarIndex);

        // 设置播放模式还是实时模式
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
         public static extern void PNSetRunningMode(RunningMode runMode);
        // 得到当前的模式
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern RunningMode PNGetRunningMode();
        // 设置道具的信息
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNSetProbInfo(int avatarIndex, int probId, int sensorId, int parentId, float probLength);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableGameModeForFingers(bool enable);
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNIsEnableGameModeForFingers();

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PNEnableSpinWithOrigin(int avatarIndex, bool enable);

        
        /**********************************************************
         *                     Prop management                    *
         **********************************************************/
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropEnable(int avatarIndex, PropIdentity propId, bool enable);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropSetParent(int avatarIndex, PropIdentity propId, int parentBoneId);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropBindSensor(int avatarIndex, PropIdentity propId, int sensorId);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropRemoveSensor(int avatarIndex, PropIdentity propId);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropSetLength(int avatarIndex, PropIdentity propId, float probLength);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropSetEndOffset(int avatarIndex, PropIdentity propId, Vector3_t offset);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropSetDirection(int avatarIndex, PropIdentity propId, Vector3_t rotate);
        
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropSetInfo(int avatarIndex, PropIdentity propId, PropInformation pi);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropGetInfo(int avatarIndex, PropIdentity propId, ref PropInformation piBuff);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern bool PNPropHandleNodeAsBVHPart(int avatarIndex, PropIdentity propId, bool enable);
        
        //===================================================
        // 测试用
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PN_TEST_Set_h0(float h0);
        //===================================================
        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern void PN_TEST_EableOpenPerfTest(bool enable);


        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        // vector3_t x:Y y:Z z:X (按照YZX进行旋转)
        public static extern void PNSetThumbAngle(int avatarIndex, Vector3_t angle);

        [DllImport("PNLib.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, ExactSpelling = true)]
        public static extern Vector3_t PNGetThumbAngle(int avatarIndex);

    }
    #endregion
}
