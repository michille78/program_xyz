#pragma once
#include "dataexport.h"

struct FrameCalculatedData;
class Kalmansteps;
class Calibration;
class BvhHeader;
class BoneMapping;
class Avatar;

class BVHDataConvert 
{
protected:
	Kalmansteps* kalmRef;
	Calibration* caliRef;
	BvhHeader*   bvhHeaderRef;
    BoneMapping* boneMapRef;
	
	int   FrameDataBuffLen;
	char* FrameDataBuff;
	char  boneDataBuff[100];
    
    // 右手指骨长度：26~49
    Point3D_t FingersOffsetR[24];
    // 左手指骨长度：57~80
    Point3D_t FingersOffsetL[24];
		
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


	vector<QUATERNION_t> bvhGlobalQuat;// 具有父子关系的Global Q
	vector<QUATERNION_t> bvhLocalQuat; // 具有父子关系的Local Q
	vector<Point3D_t>    bvhAngle;     // 具有父子关系的Local姿态
	vector<Point3D_t>    bvhDisp;      // 具有父子关系的位移

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

	QUATERNION_t smoothness(QUATERNION_t* q1, QUATERNION_t* q2);

	vector<MatrixXd> TPoseMetrix;
	void initTPoseMetrix();

	// 调节朝向(弧度)
	double       faceAngle;
	QUATERNION_t faceRot;
	QUATERNION_t FaceRotFromAngle(double angle);
	
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
	void calcBonePos(double *x);

	void fingerDataExchange();

	void fingerDataExpend(int fingerID, double xOffset=0, double yOffset=0);

    // 在大地坐标系下的骨骼的模块安装位置
    Point3D_t bonesSensorGlobalPos[BODY_BONE_COUNT];

	// 通过身体前倾、后仰等动作控制输出模型的移动
	PNBOOL    isEnableBvhMoving;
	Point3D_t deltaVect;
	double    movingV;

	void bvhMoving(Point3D_t* root, Point3D_t* bodyVector);

	// 切换欧拉角或者四元数输出
	PNBOOL isEnableQuatOutput;

	// 设置包好中的下压长度，只在无位移bvh中消除抖动的函数wipeoffWobble使用
	double expLen;

	Point3D_t V1;
	Point3D_t AD1;
	Point3D_t DB1;

	Point3D_t V2;
	Point3D_t AD2;
	Point3D_t DB2;

    void wipeoffWobble(double* x, QUATERNION_t* ql);

    RotateOrders RotOrder;
    PNBOOL         IsWithDisp;
/*
	vector<MatrixXd> retrivedM;
	void updateMetrix(Calibration* cali);
*/
public:
	BVHDataConvert(void);
	virtual ~BVHDataConvert(void);

	void Init(Avatar* avatar);

    void WithDisp(PNBOOL isWithDisp);
    void SetRotOrder(RotateOrders rotOrder);

	void   AddToBuffer(double* X, QUATERNION_t* qList, Point3D_t* BoneStart, Point3D_t* BoneEnd, PNBOOL contactL, PNBOOL contactR);
	void   CalcBvh(FrameCalculatedData* frameData);
    void   CalcBvhByQuat(FrameCalculatedData* frameData);
	float* FillBuffer(float* buffer);

	void SetFaceAngle(double angle);

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
	PNBOOL      IsFirstRight;
	PNBOOL      IsFirstLeft;

    // 获取叠加校准的具有父子关系的全局 Q
    QUATERNION_t* GetGlobalBoneQ(){ return bvhGlobalQuat.data();}
    // 获取叠加校准的具有父子关系的本地 Q
    QUATERNION_t* GetLocalBoneQ(){ return bvhLocalQuat.data();}
    // 获取具有父子关系的Local姿态
    Point3D_t* GetLocalBoneAngle(){ return bvhAngle.data();}
    // 获取具有父子关系的位移
    Point3D_t* GetBoneDisp(){ return bvhDisp.data();}

    void CleanBoneData(int boneId)
    {
        Point3D_t* addr = bvhAngle.data();
        int len = boneId * sizeof(Point3D_t);
        memset(((char*)addr) + len, 0, sizeof(Point3D_t));
    }

    // 对于Legacy，没有手部数据，可调用此函数直接抹掉
    void CleanHandData()
    {
        static int len1 = 21*sizeof(Point3D_t);
        static int len2 = BODY_BONE_COUNT*sizeof(Point3D_t);

        Point3D_t* addr = bvhAngle.data();
        memset(((char*)addr)+len1, 0, len2-len1);
    };

    // 清理缓存的所有节点的角度数据
    void ClearData()
    {
        static int len = BODY_BONE_COUNT*sizeof(Point3D_t);
        Point3D_t* addr = bvhAngle.data();
        memset((char*)addr, 0, len);
        addr = bvhDisp.data();
        memset((char*)addr, 0, len);
    }

	// 设置包好中的下压长度(只在无位移bvh中需要)
	void SetExpLen(double len) { expLen = len; };

	void EnableBvhMoving(PNBOOL enable)
	{
		deltaVect = Point3D_t();
		isEnableBvhMoving = enable;
	};
	
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

