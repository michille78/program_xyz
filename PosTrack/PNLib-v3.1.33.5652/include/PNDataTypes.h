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
    SS_Unknown,          // Unknown type
}SensorSuitTypes;

// Sensor binding combination mode
typedef enum _SensorCombinationModes
{
    SC_ArmOnly,              // Left arm or right arm only
    SC_UpperBody,            // Upper body, include one arm or both arm, must have chest node
    SC_FullBody,             // Full body mode
	SC_LowerBody,            // Lower body, sensors of legs and hips are necessory
    SC_Unknown,              // Unknown type
}SensorCombinationModes;

// Running mode
typedef enum _RunningMode
{
    RM_Realtime,
    RM_RawPlaying,
    RM_Unknown,           // Unknown type
}RunningMode;

// Accelerator type of sensor
typedef enum _SensorAcceleratorTypes
{
    SA_Range8G,                 // small range
	SA_Range16G,                // small range
	SA_Range24G,                // larg range
    SA_Unknown,                 // Unknown type
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
    RO_Unknown,           // Unknown type
}RotateOrders;

// Quaternion types of output stream
typedef enum _OutputQuaternionTypes
{
    QT_GlobalRawQuat,        // Module Q, by global system
    QT_GlobalBoneQuat,       // Calibrated module Q(Bone Q) 
    QT_LocalBoneQuat,        // Quaternion with parent
    QT_Unknown,              // Unknown type
}OutputQuaternionTypes;

// Acceleration data types of output stream
typedef enum _OutputAccelerationTypes
{
    AT_ModuleRawData,         // Module A, by global system
    AT_GlobalData,            // Calibrated module A(Bone A) 
    AT_Unknown,               // Unknown type
}OutputAccelerationTypes;

// Gyro data types of output stream
typedef enum _OutputGyroType
{
    GY_ModuleRawData,         // Module W, by global system
    GY_GlobalData,            // Calibrated module W(Bone W) 
    GY_Unknown,               // Unknown type
}OutputGyroType;

// BVH output stream types
typedef enum _BvhDataStreamTypes
{
    BO_BinaryType,           // Binary type
    BO_StringType,           // Sring type
    BO_MatrixStringType,     // Matrix string type
    BO_Unknown,              // Unknown type
}BvhDataStreamTypes;

typedef enum _CalculatedDataStreamTypes
{
    CS_BinaryType,
    CS_StringType,
    CS_Unknown,
}CalculatedDataStreamTypes;

// Bones can used as constraint 
typedef enum _ConstraintPoint
{
    CP_Hip       = 0,        // Hip
    CP_RightFoot = 3,        // Right foot
    CP_LeftFoot  = 6,        // Left foot
    CP_RightHand = 10,       // Right hand
    CP_LeftHand  = 14,       // Left hand
    CP_Unknown,              // Unknown type
}ConstraintPoint;

// Magnetic immune levels
typedef enum _MagneticImmunityLevel
{
    MI_Disable,              // Disable magnetic immune function
    MI_Weak,                 // Low level  magnetic immune
    MI_Strong,               // Strong level  magnetic immune
    MI_Unknown,              // Unknown type
}MagneticImmunityLevel;

// Poses for calibration
typedef enum _CalibrationTypes
{
    Cali_TPose,              // T pose
    Cali_APose,              // A pose
    Cali_SPose,              // Crouching pose
    Cali_NPose,              // Nod pos
    Cali_Unknown,            // Unknown type
}CalibrationTypes;

// Prop identities
typedef enum _PropIdentity
{
    PI_Prop0,                // Prop0
	PI_Prop1,                // Prop1
	PI_Prop2,                // Prop2
	PI_Prop3,                // Prop3
	PI_Prop4,                // Prop4
	PI_RightShoulderExtra,   // Right shoulder extra
	PI_LeftShoulderExtra,    // Left shoulder extra
    PI_Unknown,              // Unknown type
}PropIdentity;

// Recognized action data types
typedef enum _ActionRecognitionDataTypes
{
    Action_SingleTapping,    // Hand single tapping
    Action_DoubleTapping,    // Hand double tapping
    Action_Fire,             // Fire
    Action_InwardFlipping,   // Hand in-ward flipping
    Action_OutwardFlipping,  // Hand out-ward flipping
    Action_MassVector,       // Vector data of center of gravity
    Action_SwingVector,      // Vector data of body swing
    Action_Unknown,          // Unknown type
}ActionRecognitionDataTypes;
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
    UINT16   BvhHeaderToken1;  // Package start token: 0xDDFF
    DATA_VER DataVersion;      // Version of community data format. e.g.: 1.0.0.2
    UINT32   DataCount;        // Values count, 180 for without disp data
    PNBOOL   WithDisp;         // With/out dispement
	PNBOOL   WithReference;    // With/out reference bone data at first
	UINT32   AvatarIndex;      // Avatar index
	UCHAR    AvatarName[32];   // Avatar name
	UINT32   Reserved1;        // Reserved, only enable this package has 64bytes length
	UINT32   Reserved2;        // Reserved, only enable this package has 64bytes length
	UINT16   BvhHeaderToken2;  // Package end token: 0xEEFF
}BvhOutputBinaryHeader;

// Header format of compressed BVH data
typedef struct _BvhDataHeaderEx
{
	UINT16   BvhHeaderToken1;  // Package start token: 0xDDFF
	DATA_VER DataVersion;      // Version of community data format. e.g.: 1.0.0.2
	UINT32   DataCount;        // Values count
	PNBOOL   WithDisp;         // With/out dispement
	PNBOOL   WithReference;    // With/out reference bone data at first
	UINT32   AvatarIndex;      // Avatar index
	UCHAR    AvatarName[32];   // Avatar name
	UINT32   IsCompressed;     // Wether BVH data is compressed and compressed data check code
	UINT32   Reserved1;        // Reserved, only enable this package has 64bytes length
	UINT16   BvhHeaderToken2;  // Package end token: 0xEEFF
}BvhOutputBinaryHeaderEx;

typedef struct _CalculationDataHeader
{
    UINT16   HeaderToken1;     // Package start token: 0xDDFF
    DATA_VER DataVersion;      // Version of community data format. e.g.: 1.0.0.2
    UINT32   DataCount;        // Values count
    UINT32   AvatarIndex;      // Avatar index
    UCHAR    AvatarName[32];   // Avatar name
    UINT32   Reserved1;        // Reserved, only enable this package has 64bytes length
    UINT32   Reserved2;        // Reserved, only enable this package has 64bytes length
    UINT32   Reserved3;        // Reserved, only enable this package has 64bytes length
    UINT32   Reserved4;        // Reserved, only enable this package has 64bytes length
    UINT16   HeaderToken2;     // Package end token: 0xEEFF
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


// Bone dimensions, unit: meter
typedef struct _BoneDimension
{
    float Head;              // Bone length of head
    float Neck;              // Bone length of neck
    float Body;              // Length of body
    float ShoulderWidth;     // Width of shoulder
    float UpperArm;          // Bone length of upper arm
    float Forearm;           // Bone length of forearm
    float Palm;              // Bone length of hand
    float HipWidth;          // Width of hip
    float UpperLeg;          // Bone length of upper leg
    float LowerLeg;          // Bone length of lower leg
    float HeelHeight;        // Heel height
    float FootLength;        // Foot length
}BoneDimension;

// Bone information
typedef struct _BoneInfo
{
	int   Index;             // Bone Index in standard bone table
	char  Name[60];          // Bone name
	float Length;            // Bone length
}BoneInfo;

// Status of constraint point
typedef struct _ContactStatus
{
    ConstraintPoint Point;            // PointId
    PNBOOL          IsEdited;         // Is edited tag
    PNBOOL          IsContact;        // Is contact tag
}ContactStatus;

// All status of constraint points in one frame
typedef struct _FrameContactData
{
    int           FrameIndex;          // Index of frame data
    ContactStatus ContactInfo[5];      // Status of contraints
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

// 文件中单个Avatar的信息
typedef struct _RawFileAvatarInfo
{
	char AvatarName[32];                                              // Avatar's name
	SensorCombinationModes CombMode;                                  // Combination modes
	float                  FrontDirection[3];                         // Front direction
	BoneDimension          boneDimension;                             // boneDimension
	PNBOOL                 SensorBindingList[FULL_BODY_BONE_COUNT];   // Sensor bind 
}RawFileAvatarInfo;

// 文件信息
typedef struct _RawFileInfo
{
    RawFileTime        DateTime;      // Created time
	PNLibVersion       LibVersion;    // Library version
	SensorSuitTypes    SuitType;      // Sensor suit type: Neuron or Legacy
    int                DataFrequency; // Data acquisition frequency, Hz
	int                TotalFrames;   // Total frames in raw file 
	int                TotalTime;     // Second
    int                AvatarCount;   // Avatar count
	RawFileAvatarInfo* AvatarInfoList;
}RawFileInfo;

// Action Recognition data
typedef struct _ActionRecognitionData
{
    ActionRecognitionDataTypes DataType;    // Type of action
    int                        DataLength;  // Data length
    int                        TotalLength; // Total length
    void*                      DataPtr;     // Address of data, always be the next address of 'DataPtr'
}ActionRecognitionData;

// Calibrated data
typedef struct _CalibrationData
{
    int           AvatarIndex;                          // Avatar index in avatar list
    char          AvatarName[64];                       // Avatar name
    BoneDimension BoneDim;                              // Bone dimension of avatar
    Vector3_t     BodyDirection;                        // Initial body direction at calibrated time
    Vector3_t     LeftDirection;                        // Initial left direction at calibrated time
	Vector3_t     BoneDirections[FULL_BODY_BONE_COUNT]; // Bone data of calibration
	Vector3_t     BoneLeft[FULL_BODY_BONE_COUNT];       // Bone data of calibration
	Vector3_t     AccData[FULL_BODY_BONE_COUNT];        // Bone data of calibration
}CalibrationData;

typedef struct _SmoothFactors
{
	int GlobalDisplacement;  // Smooth factor of Displacement for all bones
	int GlobalRotation;      // Smooth factor of rotation for all bones
	int HipDisplacement;     // Smooth factor of Displacement specially for hip, global value is override by this value
	int FeetDisplacement;    // Smooth factor of rotation specially for feet, global value is override by this value
	int HipRotation;
	int FeetRotation;
}SmoothFactors;

typedef struct _PropInformation
{
    int          BoneId;     // Bone index in standard bone table
    char         Name[32];   // Prop name
	PNBOOL       IsEnable;   // Whether enabled
    int          SensorId;   // Sensor Id bound to this prop
	int          ParentBoneId; // Parent bone id of prop
	float        Length;     // Prop length
	Vector3_t    Direction;  // Prop initial face direction
}PropInformation;

typedef struct _PropData
{
    int           BoneId;    // 
    int           SensorId;  // 
    Quaternion4_t Rotation;  // 
    Vector3_t     Acc;       // 
    Vector3_t     Gyro;      // 
    Vector3_t     BoneStart; // 
    Vector3_t     BoneEnd;   // 
    Vector3_t     BoneAngle; // 
    Vector3_t     SensorPosition; // 
    Vector3_t     SensorVelocity; // 
}TEST_PropData;

// kalm params (后期将会被封闭到库中，不再对外开放)
typedef struct _KalmanParams
{
    float dt;             // 帧间隔
    float wERR;
    float aERR;           // 误差协方差
    float alpha;          // 观测误差
    float dERR;
    float gERR;           // 重力投影误差
    float damp;           // 速度衰减系数
    float damp1;          // 速度衰减系数
    float c;              // 增益调整系数
    float g;              // 重力加速度
    float Perr;           // 状态误差协方差
    int   freq;
    int   stick;          // 粘连
    float tcrrr;
    float tol1;
    float tol2;
    float tol3;
    float atol;
    float vtol;
    float kBodyX;         // 全身位移k
    float kBodyV;         // 全身速度k
    float kFeetX;         // 脚部位移k
    float kFeetV;         // 脚部速度k
    float kFeetZ;         // 脚部位移z方向下捏系数
    float hipH;           // 臀部接触高度
    float acmm;           // Center of mass
    PNBOOL massSwitch;    // 是否打开重心判断
    PNBOOL constrain;     // 是否捏双脚
    PNBOOL constrainHands;// 手部捏与不捏
    PNBOOL constrainHip;  // 臀部捏与不捏
}KalmanParams;
/********************************************/

#pragma pack(pop)

