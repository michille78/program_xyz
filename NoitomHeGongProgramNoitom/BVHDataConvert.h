#pragma once
#include "./IO/dataexport.h"
#include "BvhDataSmoother.h"
#include "AntiJointCompensation.h"

struct FrameCalculatedData;
class Kalmansteps;
class Calibration;
class BoneMapping;
class Avatar;
class Prop;
class BVHDataConvert 
{
protected:
	Avatar* avatarRef;
	Kalmansteps* kalmRef;
	Calibration* caliRef;
    BoneMapping* boneMapRef;
	
	int   FrameDataBuffLen;
	char* FrameDataBuff;
	char  boneDataBuff[100];
		
	Vector3d vErr;
	MatrixXd ret;
	
	QUATERNION_t parentQ;
	QUATERNION_t hipQ;
		
	double dt;
	long   nframe;
	
	int Qlen;
	int Xlen;
	int BoneEndLen;
	
    // Yaw是否改变的控制量
    bool  yawchanged; // 在界面上调了朝向，则更新Offset，从而对hip做位移补偿，使之绕着自身旋转
    float XOffset;
    float YOffset;
    float ZOffset;

	float* bvhFloatBuff;

	// 防抖动处理
	PNBOOL useL;
	PNBOOL useR;
	Point3D_t calcNewZ (int boneId, QUATERNION_t* ql);
	Point3D_t calcEndPt(int boneId, double* x);
	Point3D_t calcDtemp(int boneId, QUATERNION_t* ql);

	// 平滑处理缓冲区
	list<Point3D_t>    smoothnessBufferX;
	list<QUATERNION_t> smoothnessBufferQ;
    
	// 平滑处理
	void smoothness(double* x, QUATERNION_t* ql);

	BvhDataSmoother dispSmoother;
	BvhDataSmoother angleSmoother;

	QUATERNION_t smoothness(QUATERNION_t* q1, QUATERNION_t* q2);

	vector<MatrixXd> TPoseMetrix;
	void initTPoseMetrix();

	// 调节朝向(弧度)
	double       faceAngle;
	QUATERNION_t faceRot;
	QUATERNION_t FaceRotFromAngle(double angle);

	// 模型一个偏移量
	Point3D_t positionOffset;
	
	//Point3D_t X0;
	Point3D_t rootX;
	
	Matrix3d getRot(int boneId, QUATERNION_t* ql, PNBOOL isSubNode);
	// 计算Q 此处Q为Global Q, Rot1包含校准信息, TPoseMetrix包含初始姿态
	void getRot1(Matrix3d& rot1, int boneId, QUATERNION_t* ql, PNBOOL isSubNode);
    // 计算初始骨骼朝向
	Matrix3d getRot2(int boneId, PNBOOL isSubNode);    
	void getSmoothMatrix(Matrix3d& rot1, int boneId, int boneId1, int boneId2, QUATERNION_t* ql, PNBOOL isNearBone1); // 对没有模块的骨骼插值

	// 获取子节点相对于父节点的局部位移（偏移量）
	Point3D_t getLocalBoneStart(int boneIndex, double* X, QUATERNION_t* ql, Point3D_t* BoneStart, Point3D_t* BoneEnd);
	int getParentId(int boneId);

	// 是否输出世界坐标系下的骨骼起始位置
	PNBOOL isGlobalDisp;

	// 取得世界坐标系下的骨骼起始端点
	Point3D_t getGlobalBoneStart(int boneIndex, Point3D_t* BoneStart, Point3D_t* BoneEnd);
	
	// 计算每根骨骼的位置
	void calcBonePos(double *x, QUATERNION_t* qList);
	
	// 对手指间的数据扩展
	void fingersExpansion();

	// 手指自身扩展
	void fingerExpandItself();

	//基于BVH的YXZ的旋转顺序进行的数据插值
	void fingerDataExpandWithFingerId(int fingerID);

	// 手指反关节补偿
	AntiJointCompensation antiJoint;
	void fingerAntiJoint();

	void gameModeExchanged();
	// 基于BVH的YXZ的旋转顺序进行的游戏模式插值
	void gameModeExpend(int fingerID, double xOffset, double yOffset);


	// 在大地坐标系下的骨骼的模块安装位置
	Point3D_t bonesSensorGlobalPos[FULL_BODY_BONE_COUNT];

	// 在大地坐标系下的骨骼朝向
	Point3D_t bonesSensorGlobalDir[FULL_BODY_BONE_COUNT];

	// 通过身体前倾、后仰等动作控制输出模型的移动
	PNBOOL    isEnableBvhMoving;
	Point3D_t deltaVect;
	double    movingV;

	void bvhMoving(Point3D_t* root, Point3D_t* bodyVector);

	// 切换欧拉角或者四元数输出
	PNBOOL isEnableQuatOutput;
	
	Point3D_t V1;
	Point3D_t AD1;
	Point3D_t DB1;

	Point3D_t V2;
	Point3D_t AD2;
	Point3D_t DB2;

    void wipeoffWobble(double* x, QUATERNION_t* ql);

    RotateOrders RotOrder;
    PNBOOL       IsWithDisp;
/*
	vector<MatrixXd> retrivedM;
	void updateMetrix(Calibration* cali);
*/
public:
	BVHDataConvert(Avatar* avatarRef);
	virtual ~BVHDataConvert(void);

	Point3D_t GetBonePosition(int boneIndex);
	Point3D_t GetBoneDirection(int boneIndex);

    void WithDisp(PNBOOL isWithDisp);
    void SetRotOrder(RotateOrders rotOrder);

	void   AddToBuffer(double* X, QUATERNION_t* qList, Point3D_t* BoneStart, Point3D_t* BoneEnd, PNBOOL contactL, PNBOOL contactR);
	void   CalcBvh(FrameCalculatedData* frameData);
    void   CalcBvhByQuat(FrameCalculatedData* frameData);

	void SetFaceAngle(double angle);
	void SetAdjustPos(float x, float y, float z);
	Point3D_t GetAdjustPos();


	void EnableSmooth(bool enable)
	{
		dispSmoother.IsEnabled = enable;
		angleSmoother.IsEnabled = enable;
	};

    // 绕原点还是自身旋转
    PNBOOL IsSpinWithOrigin;

    // 获取XOffset 和 YOffset 用于旋转模型
    void GetOffset(float* xOffset, float* yOffset, QUATERNION_t* facerot);

	// 人人网，手臂限制功能
	PNBOOL IsRestrictArmRotation;
	// 人人网，手臂限制功能
	Point3D_t getRestrictArmNewZ(int boneId, QUATERNION_t* ql);		
	Point3D_t rightArmZ0;
	Point3D_t leftArmZ0;
	PNBOOL    IsFirstRight;
	PNBOOL    IsFirstLeft;

 //   // 获取叠加校准的具有父子关系的全局 Q
 //   QUATERNION_t* GetGlobalBoneQ(){ return bvhGlobalQuat.data();}
 //   // 获取叠加校准的具有父子关系的本地 Q
 //   QUATERNION_t* GetLocalBoneQ(){ return bvhLocalQuat.data();}
 //   // 获取具有父子关系的Local姿态
 //   Point3D_t* GetLocalBoneAngle(){ return bvhAngle.data();}
 //   // 获取具有父子关系的位移
 //   Point3D_t* GetBoneDisp(){ return bvhDisp.data();}
	
	// 具有父子关系的Global Q
	vector<QUATERNION_t> bvhGlobalQuat;
	// 具有父子关系的Local Q
	vector<QUATERNION_t> bvhLocalQuat;
	// 具有父子关系的Local姿态
	vector<Point3D_t>    bvhAngle;
	// 具有父子关系的位移
	vector<Point3D_t>    bvhDisp;

	// 将道具数据放入相应的bvhExporter中，方便后面统一导出
	void ImportPropData(int propBoneId, Point3D_t propBvhAngle, Point3D_t propBvhDisp);

	// 手指反关节处理类
	AntiJointCompensation* GetFingerAntiJointor(){ return &antiJoint; };

	// 清理某根骨骼angle
    void CleanBoneData(int boneId)
    {
        Point3D_t* addr = bvhAngle.data();
        int len = boneId * sizeof(Point3D_t);
        memset(((char*)addr) + len, 0, sizeof(Point3D_t));
    }

    // 对于Legacy，没有手部数据，可调用此函数直接抹掉
    void CleanHandData()
	{
		static int lenR1 = 25 * sizeof(Point3D_t);
		static int lenR2 = 49 * sizeof(Point3D_t);
		static int lenL1 = 56 * sizeof(Point3D_t);
		static int lenL2 = 80 * sizeof(Point3D_t);

		// 只清除左右手
		Point3D_t* addr = bvhAngle.data();
		memset(((char*)addr) + lenR1, 0, lenR2 - lenR1);
		memset(((char*)addr) + lenL1, 0, lenL2 - lenL1);
    };

    // 清理缓存的所有节点的角度数据
    void ClearData()
    {
        static int len = FULL_BODY_BONE_COUNT*sizeof(Point3D_t);
        Point3D_t* addr = bvhAngle.data();
        memset((char*)addr, 0, len);
        addr = bvhDisp.data();
        memset((char*)addr, 0, len);
    }
	
	// 重置Hip位置
	void ResetHipPos()
	{
		XOffset = 0;
		YOffset = 0;
		ZOffset = 0;
	};

	void EnableBvhMoving(PNBOOL enable)
	{
		deltaVect = Point3D_t();
		isEnableBvhMoving = enable;
	};
	// 匀速运动倍速
	void SetBvhMovingV(double v)
	{
		movingV = v;
	};

	PNBOOL isOnlyOffset;

	Point3D_t* getBVHAngle();

	void EnableQuatOutput(PNBOOL enable)
	{
		isEnableQuatOutput = enable;
	};

	void EnableGlobalDisp(PNBOOL enable)
	{
		isGlobalDisp = enable;
	};
};

