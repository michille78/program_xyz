#pragma  once

/********************************************
 *   Cross platform data types definitions  *
 ********************************************/
#ifdef __OS_XUN__
#include <Cocoa/Cocoa.h>     // Mac OS X basic file
typedef NSView*              BVHWindowContainerRef;

typedef unsigned short       UINT16;
typedef unsigned int         UINT32;
typedef unsigned long long   UINT64;
typedef unsigned short       USHORT;
typedef unsigned char        UCHAR;
typedef unsigned char        BYTE;
typedef wchar_t              WCHAR;

#else
#include <windows.h>         // Windows Heade File
typedef HWND                 BVHWindowContainerRef;
#endif

typedef int                  PNBOOL;   // There lots of different 'boolean' definition in different platform or compiler
typedef unsigned int         PNSTATUS; // Status code returned from PNLib functions. '0' means successfully implemented, otherwise some error(0) occured when calling PNLib function.
#define PN_SUCCESS(statusCode) (0==statusCode)   // A MICRO used to check return code
#define PN_ISTRUE(PNBoolean)   (0!=PNBoolean)    // A MICRO used to check a PNBOOL is 'true'
/********************************************/

// Number of bones in standard bone system of PNLib
#define FULL_BODY_BONE_COUNT 167

/********************************************
 *               Enum types                 *
 ********************************************/

// Suit mode types of sensor at realtime mode
typedef enum _SensorSuitTypes
{
    SS_LegacySensors,   // Wireless legacy suit
    SS_NeuronSensors,   // Neuron nodes suit
    SS_Unknow,          // Unknow type
}SensorSuitTypes;

// Node combination types
typedef enum _NodeCombinationModes
{
    NC_ArmOnly,              // Left arm or right arm only
    NC_UpperBody,            // Upper body, include one arm or both arm, must have chest node
    NC_FullBody,             // Full body mode
    NC_Unknow,               // Unknow type
}NodeCombinationModes;

// Running mode
typedef enum _RunningMode
{
    RM_Realtime,
    RM_RawPlaying,
    RM_Unknow,           // Unknow type
}RunningMode;

//// Glove types
//typedef enum _GloveTypes
//{
//    GT_RightHand,            // Glove of right hand
//    GT_LeftHand              // Glove of left hand
//    GT_Unknow,           // Unknow type
//}GloveTypes;
//
//// Glove node modes
//typedef enum _GloveNodeModes
//{
//    GN_ThreeFingers,         // Three fingers mode
//    GN_FiveFingers,          // Five fingers mode
//    GN_FullHand,             // Full hand mode
//    GN_Unknow,               // Unknow type
//}GloveNodeModes;

// Accelerator type of sensor
typedef enum _SensorAcceleratorTypes
{
    SA_Legacy8G,             // Legacy small range
    SA_Legacy24G,            // Legacy larg range
    SA_Neuron8G,             // Neuron small range
    SA_Neuron16G,            // Neuron small range
    SA_Unknow,               // Unknow type
}SensorAcceleratorTypes;

// BVH rotate orders
typedef enum _RotateOrders
{
    RO_XZY,
    RO_YXZ,
    RO_XYZ,
    RO_YZX,
    RO_ZXY,
    RO_ZYX,
    RO_Unknow,           // Unknow type
}RotateOrders;

// Quaternion types of output stream
typedef enum _OutputQuaternionTypes
{
    QT_GlobalRawQuat,        // Module Q, by global system
    QT_GlobalBoneQuat,       // Calibrated module Q(Bone Q) 
    QT_LocalBoneQuat,        // Quaternion with parent
    QT_Unknow,               // Unknow type
}OutputQuaternionTypes;

// Acceleration data types of output stream
typedef enum _OutputAccelerationTypes
{
    AT_ModelRawData,         // Module A, by global system
    AT_GlobalData,           // Calibrated module A(Bone A) 
    AT_Unknow,               // Unknow type
}OutputAccelerationTypes;

// Gyro data types of output stream
typedef enum _OutputGyroType
{
    GY_ModelRawData,         // Module W, by global system
    GY_GlobalData,           // Calibrated module W(Bone W) 
    GY_Unknow,               // Unknow type
}OutputGyroType;

// BVH output stream types
typedef enum _BvhDataStreamTypes
{
    BO_BinaryType,           // Binary type
    BO_StringType,           // Sring type
    BO_MatrixStringType,     // Matrix string type
    BO_Unknow,               // Unknow type
}BvhDataStreamTypes;

typedef enum _CalculatedDataStreamTypes
{
    CS_BinaryType,
    CS_StringType,
    CS_Unknow,
}CalculatedDataStreamTypes;

// Bones can used as constraint 
typedef enum _ConstraintPoint
{
    CP_Hip       = 0,        // Hip
    CP_RightFoot = 3,        // Right foot
    CP_LeftFoot  = 6,        // Left foot
    CP_RightHand = 10,       // Right hand
    CP_LeftHand  = 14,       // Left hand
    CP_Unknow,               // Unknow type
}ConstraintPoint;

// Magnetic immune levels
typedef enum _MagneticImmuneLevel
{
    MI_Disable,              // Disable magnetic immune function
    MI_Weak,                 // Low level  magnetic immune
    MI_Strong,               // Strong level  magnetic immune
    MI_Unknow,               // Unknow type
}MagneticImmuneLevel;

// Poses for calibration
typedef enum _CalibrationTypes
{
    Cali_TPose,              // T pose
    Cali_APose,              // A pose
    Cali_SPose,              // Crouching pose
    Cali_NPose,              // Nod pos
    Cali_Unknow,             // Unknow type
}CalibrationTypes;

// Recognized action data types
typedef enum _ActionRecognizationDataTypes
{
    Action_SingleTapping,    // Hand single tapping
    Action_DoubleTapping,    // Hand double tapping
    Action_Fire,             // Fire
    Action_InwardFlipping,   // Hand in-ward flipping
    Action_OutwardFlipping,  // Hand out-ward flipping
    Action_MassVector,       // Vector data of center of gravity
    Action_SwingVector,      // Vector data of body swing
    Action_Unknow,           // Unknow type
}ActionRecognizationDataTypes;
/********************************************/


/********************************************
 *       Struct data type definitions       *
 ********************************************/
#pragma pack(push, 1)
// dynamic library version info
typedef struct _PNLibVersion
{
	USHORT Major;            // Major number
	USHORT Minor;            // Minor number
	USHORT Revision;         // Revision number
	USHORT BuildNumb;        // Build number
}PNLibVersion;

// BVH data stream version number
typedef union OutputDataVersion
{
    UINT32 _VersionMask;
    struct
    {
        UCHAR BuildNumb;     // Build number
        UCHAR Revision;      // Revision number
        UCHAR Minor;         // Subversion number
        UCHAR Major;         // Major version number
    };
} DATA_VER;

// Header format of BVH data in output stream
typedef struct _BvhOutputBinaryHeader
{
    UINT16 BvhHeaderToken1;  // Package start token: 0xDDFF
    DATA_VER DataVersion;    // Version of community data format. e.g.: 1.0.0.2
    UINT32 DataCount;        // Values count, 180 for without disp data
    PNBOOL WithDisp;         // With/out dispement
    PNBOOL WithReference;    // With/out reference bone data at first
    UINT32 AvatarIndex;      // Avatar index
    UCHAR  AvatarName[32];   // Avatar name
    UINT32 Reserved1;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved2;        // Reserved, only enable this package has 64bytes length
    UINT16 BvhHeaderToken2;  // Package end token: 0xEEFF
}BvhOutputBinaryHeader;

typedef struct _CalculationDataHeader
{
    UINT16 HeaderToken1;     // Package start token: 0xDDFF
    DATA_VER DataVersion;    // Version of community data format. e.g.: 1.0.0.2
    UINT32 DataCount;        // Values count
    UINT32 AvatarIndex;      // Avatar index
    UCHAR  AvatarName[32];   // Avatar name
    UINT32 Reserved1;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved2;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved3;        // Reserved, only enable this package has 64bytes length
    UINT32 Reserved4;        // Reserved, only enable this package has 64bytes length
    UINT16 HeaderToken2;     // Package end token: 0xEEFF
}CalculationDataHeader;


// Quaternion
typedef struct _Quaternion4_t
{
    float s;
    float x;
    float y;
    float z;
}Quaternion4_t;

// Vector
typedef struct _Vector3_t
{
    float x;
    float y;
    float z;
}Vector3_t;

// Standard bone system table
typedef struct _BoneMap
{
    int Index;               // The bone index, also treaded as standard bone Id.
    char Name[32];           // Bone name
    int SensorId;            // Sensor Id bound to this bone
}BoneMap;


// Bone dimensions, unit: meter
typedef struct _BoneDimension
{
    float Head;              // Bone length of head,      eg.: 0.20
    float Neck;              // Bone length of neck,      eg.: 0.08
    float Body;              // Length of body,           eg.: 0.60
    float ShoulderWidth;     // Width of shoulder,        eg.: 0.41
    float UpperArm;          // Bone length of upper arm, eg.: 0.26
    float Forearm;           // Bone length of forearm,   eg.: 0.28
    float Palm;              // Bone length of hand,      eg.: 0.20
    float HipWidth;          // Width of hip,             eg.: 0.26
    float UpperLeg;          // Bone length of upper leg, eg.: 0.49
    float LowerLeg;          // Bone length of lower leg, eg.: 0.48
    float HeelHeight;        // Heel height,              eg.: 0.05
    float FootLength;        // Foot length,              eg.: 0.28
}BoneDimension;

// Status of constraint point
typedef struct _ContactStatus
{
    ConstraintPoint Point;   // PointId
    PNBOOL IsEdited;         // Is edited tag
    PNBOOL IsContact;        // Is contact tag
}ContactStatus;

// All status of constraint points in one frame
typedef struct _FrameContactData
{
    int FrameIndex;          // Index of frame data
    ContactStatus ContactInfo[5]; // Status of contraints
}FrameContactData;

// Raw file created time
typedef struct _RawFileTime
{
    UCHAR  Reserved;         // 
    UCHAR  Second;           // second
    UCHAR  Minute;           // minute
    UCHAR  Hour;             // hour
    UCHAR  Day;              // day
    UCHAR  Month;            // month
    USHORT Year;             // year
}RawFileTime;

typedef struct _RawDataFileInformation
{
    RawFileTime Time;          // Created time
    int         AvatarCount;   // Avatar count
    int         DataFrequency; // Data acquisition frequency, Hz
}RawDataFileInformation;

// Action recognization data
typedef struct _ActionRecognizationData
{
    ActionRecognizationDataTypes DataType; // Type of action
    int         DataLength;  // Data length
    int         TotalLength; // Total length
    void*       DataPtr;     // Address of data, always be the next address of 'DataPtr'
}ActionRecognizationData;

// Calibrated data
typedef struct _CalibrationData
{
    int AvatarIndex;         // Avatar index in avatar list
    char AvatarName[64];     // Avatar name
    BoneDimension BoneDim;   // Bone dimension of avatar

    Vector3_t FaceDirection; // Initial face direction at calibrated time
    Vector3_t LeftDirection; // Initial left direction at calibrated time

    Vector3_t BoneDirections[FULL_BODY_BONE_COUNT]; // Bone data of calibration
    Vector3_t BoneLeft[FULL_BODY_BONE_COUNT];       // Bone data of calibration
    Vector3_t AccData[FULL_BODY_BONE_COUNT];        // Bone data of calibration
}CalibrationData;

typedef struct _SmoothFactors
{
    int GlobalDisplement;    // Smooth factor of displement for all bones
    int GlobalRotation;      // Smooth factor of rotation for all bones
    int HipDisplement;       // Smooth factor of displement specially for hip, global value is override by this value
    int FeetDisplement;      // Smooth factor of rotation specially for feet, global value is override by this value
    int HipRotation;
    int FeetRotation;
}SmoothFactors;

// kalm params (后期将会被封闭到库中，不再对外开放)
typedef struct _KalmanParams
{
    float dt;         // 帧间隔
    float wERR;
    float aERR;       // 加速度计的误差协方差矩阵初始化;根据加速度传感器datasheet，noise density换算得出
    float alpha;      // 设角度观测误差级别为7度
    float dERR;
    float gERR;       // 重力投影所引入的误差，该误差与投影的角度测量相关，而角度测量与q相关，此处假定角度测量误差较大时有alpha度的测量偏差
    float damp;       // 速度衰减系数
    float damp1;      // 位移积分时的速度衰减系数,由于速度不足，这里做了个补偿
    float c;          // 增益调整系数;
    float g;          // 重力加速度大小,用一个略大的重力加速度压住模型不让丫飘起来
    float Perr;       // 状态误差协方差矩阵初始化；P矩阵会迅速迭代收敛，因此初始化无需精确，随意即可，设置好一点收敛快一些罢了
    int freq;
    int stick;         // 粘连
    float tcrrr;
    float tol1;
    float tol2;
    float tol3;
    float atol;
    float vtol;
    float kBodyX;      // 全身位移k
    float kBodyV;      // 全身速度k
    float kFeetX;      // 脚部位移k
    float kFeetV;      // 脚部速度k
    float kFeetZ;      // 脚部位移z方向分段下捏系数（kFeetX的一个百分系数）
    float hipH;        // 臀部接触高度
    float acmm;        // Center of mass
    PNBOOL massSwitch;    // 是否打开重心判断
    PNBOOL constrain;     // 是否捏双脚
    PNBOOL constrainHands;// 手部捏与不捏
    PNBOOL constrainHip;  // 臀部捏与不捏
}KalmanParams;
/********************************************/

#pragma pack(pop)

