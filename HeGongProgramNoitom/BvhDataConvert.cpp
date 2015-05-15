#include "stdafx.h"
#include "BVHDataConvert.h"

#include "Kalmansteps.h"
#include "Calibration.h"
#include "BvhHeader.h"
#include "FrameCalculatedData.h"
#include "./ErrorManage/ErrorCode.h"
#include "./AvatarManagement/MultiAvatarManager.h"
#include "./AvatarManagement/Avatar.h"

#include "BoneMapping.h"
BVHDataConvert::BVHDataConvert(void) :
vErr(3),
ret(3, 3)
{
    IsFirstRight = TRUE;
    IsFirstLeft = TRUE;

    faceAngle = 0;
    isOnlyOffset = FALSE;

    yawchanged = FALSE;
    XOffset = 0;
    YOffset = 0;
    ZOffset = 0;

    // 不带位移的bvh防抖下压量
    expLen = 2;
    useR = FALSE;
    useL = FALSE;

    bvhHeaderRef = NULL;
    caliRef = NULL;
    boneMapRef = NULL;
    kalmRef = NULL;

    isEnableBvhMoving = FALSE;
    deltaVect = Point3D_t();
    movingV = 2;

    IsWithDisp = FALSE;
    RotOrder = RO_YXZ;

    IsRestrictArmRotation = FALSE;

    isEnableQuatOutput = FALSE;
    isGlobalDisp = FALSE;
    bvhFloatBuff = NULL;

    FrameDataBuffLen = 3000;
    FrameDataBuff = new char[FrameDataBuffLen];
    IsSpinWithOrigin = 0; // 默认是绕自身旋转
    ASSERT(FrameDataBuff);
}


BVHDataConvert::~BVHDataConvert(void)
{
    if (FrameDataBuff)
    {
        FrameDataBuffLen = 0;
        delete[] FrameDataBuff;
        FrameDataBuff = NULL;
    }

    if (this->bvhFloatBuff)
    {
        delete[] this->bvhFloatBuff;
    }

    bvhGlobalQuat.clear();
    bvhLocalQuat.clear();
    bvhAngle.clear();
    bvhDisp.clear();
}

void BVHDataConvert::WithDisp(PNBOOL isWithDisp)
{
    this->IsWithDisp = isWithDisp;
    if (this->IsWithDisp)
    {
        // 当时带位移的时候则不进行防抖处理
        useL = FALSE;
        useR = FALSE;
    }
}

void BVHDataConvert::SetRotOrder(RotateOrders rotOrder)
{
    this->RotOrder = rotOrder;
}

void BVHDataConvert::Init(Avatar* avatar)
{
    this->kalmRef = avatar->GetKalmFilter();
    this->caliRef = kalmRef->GetCalibrator();
    this->dt = kalmRef->GetDt();
    this->bvhHeaderRef = avatar->GetBvhHeaderRef();
    this->boneMapRef = avatar->GetBoneMapping();

    // 获取指节骨骼长度数据
    memset(FingersOffsetR, 0, sizeof(FingersOffsetR));
    for (int i = 25; i < 49; i++)
    {
        FingersOffsetR[i - 25] = bvhHeaderRef->getOFFSET(i);
    }
    memset(FingersOffsetL, 0, sizeof(FingersOffsetL));
    for (int i = 56; i < 80; i++)
    {
        FingersOffsetL[i - 56] = bvhHeaderRef->getOFFSET(i);
    }

    this->bvhGlobalQuat.resize(BODY_BONE_COUNT, QUATERNION_t());
    this->bvhLocalQuat.resize(BODY_BONE_COUNT, QUATERNION_t());
    this->bvhAngle.resize(BODY_BONE_COUNT, Point3D_t());
    this->bvhDisp.resize(BODY_BONE_COUNT, Point3D_t());

    this->IsRestrictArmRotation = FALSE;

    if (this->bvhFloatBuff)
    {
        delete[] this->bvhFloatBuff;
    }
    this->bvhFloatBuff = new float[22 * 6];

    this->faceRot = FaceRotFromAngle(faceAngle);

    this->Qlen = sizeof(QUATERNION_t) * kalmRef->qList.size();
    this->Xlen = sizeof(double) * 126;
    this->BoneEndLen = (BODY_BONE_COUNT + 2) * sizeof(Point3D_t) * 2;

    // Zero out
    rootX = Point3D_t();

    initTPoseMetrix();
}

void BVHDataConvert::initTPoseMetrix()
{
    TPoseMetrix.resize(BODY_BONE_COUNT, MatrixXd(3, 3));

    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        TPoseMetrix[i] = getRot2(i, i != 0).inverse();
    }
}

QUATERNION_t BVHDataConvert::FaceRotFromAngle(double angle)
{
    // 解决Yaw绕圆心旋转
    //if (faceAngle != angle)
    {
        yawchanged = TRUE;
    }

    // 旋转人体朝向至模型方向
    Point3D_t faceDirection = caliRef->Zd;

    QUATERNION_t faceQ = QUATERNION_t(cos(angle / 2.0), 0.0, 0.0, sin(angle / 2.0));

    Point3D_t y1;
    q_v_qc(&faceQ, &faceDirection, &y1);

    if (abs(y1.dot(Point3D_t(0, 1, 0)) - 1) > 1e-3)
    {
        faceQ = QUATERNION_t(cos(-angle / 2.0), 0.0, 0.0, sin(-angle / 2.0));
    }

    // save angle
    faceAngle = angle;

    return faceQ;
}

// 通过身体前倾、后仰等动作控制输出模型的移动
void BVHDataConvert::bvhMoving(Point3D_t* root, Point3D_t* bodyVector)
{
    Point3D_t vect(bodyVector->get());
    if (vect.mod() != 0)
    {
        vect.Normalize(); // 归一化
    }

    // 应用face rot
    //q_v_qc(&faceRot, &vect, &vect);

    vect.Z = 0; // 在平面上投影

    // 前倾到一定程度才算是要前移的动作
    if (vect.mod() > 0.25)
    {
        vect = vect * dt * movingV;

        deltaVect = deltaVect + vect;

        //deltaVect = deltaVect + Point3D_t(vect.X, -vect.Z, vect.Y) * 100; // 转换成厘米，xzy格式
    }
    root->X -= deltaVect.X;
    root->Y -= deltaVect.Y;
}

void BVHDataConvert::CalcBvhByQuat(FrameCalculatedData* frameData)
{
    QUATERNION_t* smoothQ = frameData->qList;
    Matrix3d rot1;
    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        if ((i < 21) ||            // 身体部分
            (i >= 25 && i < 49) ||   // 右手
            (i >= 56 && i < 80))     // 左手
        {
            getRot1(rot1, i, smoothQ, i != 0);
            bvhGlobalQuat[i] = MatrixToQuat(rot1*TPoseMetrix[i]);
        }
    }
    //=========== 几根没有模块的节点，用相邻骨骼插值 =======================
    // Neck:16
    getSmoothMatrix(rot1, 16, 15, 18, smoothQ, TRUE);
    bvhGlobalQuat[16] = MatrixToQuat(rot1*TPoseMetrix[16]);

    // quatSpine3:17
    getSmoothMatrix(rot1, 17, 18, 15, smoothQ, TRUE);
    bvhGlobalQuat[17] = MatrixToQuat(rot1*TPoseMetrix[17]);

    // quatSpine1:19
    getSmoothMatrix(rot1, 19, 18, 0, smoothQ, TRUE);
    bvhGlobalQuat[19] = MatrixToQuat(rot1*TPoseMetrix[19]);

    // quatSpine0:20
    getSmoothMatrix(rot1, 20, 0, 18, smoothQ, TRUE);
    bvhGlobalQuat[20] = MatrixToQuat(rot1*TPoseMetrix[20]);
    //========================================

    // 由Q计算Angle
    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        if (i == 0)             // hip
        {
            bvhAngle[0] = QuatToAng(RotOrder, bvhGlobalQuat[0]);
        }
        else if (i == 1 || i == 4) // 大腿
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]));
        }
        else if (i == 7 || i == 11) // 双肩
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[17]));
        }
        else if (i == 20)          // Spine0
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]));
        }
        else if (i < 15)           // 双手双脚
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]));
        }
        else if (i >= 15 && i < 21)    // 脊柱
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i + 1]));
        }
        else                    // 手指
        {
            if (i == 25 || i == 29 || i == 34 || i == 39 || i == 44)       // 右手指根
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[10]));
            }
            else if (i == 56 || i == 60 || i == 65 || i == 70 || i == 75)   // 左手指根
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[14]));
            }
            // 其他指节
            else if ((i>25 && i < 49) || (i>56 && i < 80))
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]));
            }
        }
    }

    // 五指模式对手指bvh角度进行插值
    fingerDataExchange();

    // 计算位移
    // 身体部分(非全局位移）
    for (int i = 0; i < 21; i++)
    {
        bvhDisp[i] = (bvhHeaderRef->getOFFSET(i)) * 100; //将单位从m转成cm
    }
    // 右手
    for (int i = 25; i < 49; i++)
    {
        // 对于手部骨节长度，0是大拇指骨节
        bvhDisp[i] = FingersOffsetR[i - 25] * 100; //将单位从m转成cm
    }
    // 左手
    for (int i = 56; i < 80; i++)
    {
        // 对于手部骨节长度，0是大拇指骨节
        bvhDisp[i] = FingersOffsetL[i - 56] * 100; //将单位从m转成cm
    }
}


void BVHDataConvert::CalcBvh(FrameCalculatedData* frameData)
{
    //static HighPerfTimer hpt;

    // 从第一帧初始化X0
    //if(nframe==0)
    //{
    //	X0 = Point3D_t(x[0], x[1], 0);
    //}

    double* x = frameData->x;
    QUATERNION_t* smoothQ = frameData->qList;
    Point3D_t* BoneStart = frameData->boneStart;
    Point3D_t* BoneEnd = frameData->boneEnd;

    Point3D_t hipPos = Point3D_t(x[0], x[1], x[2]);// - X0;

    float X0 = hipPos.X;
    float Y0 = hipPos.Y;
    float Z0 = hipPos.Z;

    // 对于固定模式，给Hip加一个位移，使之可以移动
    if (isEnableBvhMoving)
    {
        bvhMoving(&hipPos, &kalmRef->ldList[18]);
    }

    // 按照调整后的朝向旋转Hip，从而达到旋转模型朝向的目的
    q_v_qc(&faceRot, &hipPos, &hipPos);

    // 在界面上调了朝向，则更新Offset，从而对hip做位移补偿，使之绕着自身旋转
    if (yawchanged)
    {
        // 如果绕Bvh播放器中的中心转则不加偏移量，否则消除偏移量
        if (IsSpinWithOrigin)
        {
            XOffset = 0;
            YOffset = 0;
            ZOffset = 0;
        }
        else
        {
            XOffset = hipPos.X - X0;
            YOffset = hipPos.Y - Y0;
            ZOffset = hipPos.Z - Z0;
        }
        yawchanged = FALSE;
    }

    // l=ll[i]*LU[i]，挪到LU一端
    hipPos = hipPos - kalmRef->lList[0];

    // 抬高dZ+heelHeight
    hipPos.Z -= kalmRef->dZ + kalmRef->heelHeight;

    // fu
    if (MultiAvatarManager::Instance->IsZeroOutPosition)
    {
        XOffset = 0;
        YOffset = 0;
        ZOffset = 0;
        MultiAvatarManager::Instance->IsZeroOutPosition = FALSE;
    }
    // 转成bvh坐标系
    rootX = Point3D_t(hipPos.X - XOffset, -hipPos.Z, hipPos.Y - YOffset) * 100;

    // 保存
    bvhDisp[0] = rootX;

    // 计算每根骨骼的位置
    calcBonePos(x);

    //hpt.CounterStart();

    // 不带位移的bvh数据需要做防抖动处理
    if (!IsWithDisp)
    {
        wipeoffWobble(x, smoothQ);
    }

    // 计算Q
    // 此处Q为Global Q, Rot1包含校准信息, TPoseMetrix包含初始姿态
    Matrix3d rot1;
    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        if ((i < 21) ||            // 身体部分
            (i >= 25 && i < 49) ||   // 右手
            (i >= 56 && i < 80))     // 左手
        {
            getRot1(rot1, i, smoothQ, i != 0);
            bvhGlobalQuat[i] = MatrixToQuat(rot1*TPoseMetrix[i]);
        }
    }
    //=========== 几根没有模块的节点，用相邻骨骼插值 =======================
    // Neck:16
    getSmoothMatrix(rot1, 16, 15, 18, smoothQ, TRUE);
    bvhGlobalQuat[16] = MatrixToQuat(rot1*TPoseMetrix[16]);

    // quatSpine3:17
    getSmoothMatrix(rot1, 17, 18, 15, smoothQ, TRUE);
    bvhGlobalQuat[17] = MatrixToQuat(rot1*TPoseMetrix[17]);

    // quatSpine1:19
    getSmoothMatrix(rot1, 19, 18, 0, smoothQ, TRUE);
    bvhGlobalQuat[19] = MatrixToQuat(rot1*TPoseMetrix[19]);

    // quatSpine0:20
    getSmoothMatrix(rot1, 20, 0, 18, smoothQ, TRUE);
    bvhGlobalQuat[20] = MatrixToQuat(rot1*TPoseMetrix[20]);
    //========================================

    // 由Q计算Angle
    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        if (i == 0)                 // hip
        {
            bvhAngle[0] = QuatToAng(RotOrder, bvhGlobalQuat[0]);
        }
        else if (i == 1 || i == 4)  // 大腿
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]));
        }
        else if (i == 7 || i == 11) // 双肩
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[17]));
        }
        else if (i == 20)           // Spine0
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]));
        }
        else if (i < 15)            // 双手双脚
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]));
        }
        else if (i >= 15 && i < 21) // 脊柱
        {
            bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i + 1]));
        }
        else                        // 双手手指
        {
            if (i == 25 || i == 29 || i == 34 || i == 39 || i == 44)        // 右手指根
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[10]));
            }
            else if (i == 56 || i == 60 || i == 65 || i == 70 || i == 75)   // 左手指根
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[14]));
            }
            else if ((i>25 && i < 49) || (i>56 && i < 80))                  // 其他指节
            {
                bvhAngle[i] = QuatToAng(RotOrder, qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]));
            }
        }
    }

    // 按照父子关系转成本地Q
    for (int i = 0; i < BODY_BONE_COUNT; i++)
    {
        if (i == 0)                         // hip
        {
            bvhLocalQuat[i] = bvhGlobalQuat[0];
        }
        else if (i == 1 || i == 4)          // 大腿
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]);
        }
        else if (i == 7 || i == 11)         // 肩膀 
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[17]);
        }
        else if (i == 20)                   // Spine0 
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[0]);
        }
        else if (i < 15)                    // 双手双脚
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]);
        }
        else if (i >= 15 && i < 21)         // 脊柱
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[i + 1]);
        }
        else if (i == 25 || i == 29 || i == 34 || i == 39 || i == 44)   // 右手指根
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[10]);
        }
        else if (i == 56 || i == 60 || i == 65 || i == 70 || i == 75)   // 左手指根
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[14]);
        }
        else if ((i>25 && i < 49) || (i>56 && i < 80))   // 其他指节，包括 指甲骨节 部分
        {
            bvhLocalQuat[i] = qconj(bvhGlobalQuat[i], bvhGlobalQuat[i - 1]);
        }
    }

    // 五指模式对手指bvh角度进行插值
    fingerDataExchange();

    // disp
    if (!isGlobalDisp)
    {
        // 身体部分
        for (int i = 1; i < 21; i++)
        {
            bvhDisp[i] = getLocalBoneStart(i, x, smoothQ, BoneStart, BoneEnd);
        }
        // 右手
        for (int i = 25; i < 49; i++)
        {
            // 对于手部骨节长度，0是大拇指骨节
            bvhDisp[i] = FingersOffsetR[i - 25] * 100; //将单位从m转成cm
        }
        // 左手
        for (int i = 56; i < 80; i++)
        {
            // 对于手部骨节长度，0是大拇指骨节
            bvhDisp[i] = FingersOffsetL[i - 56] * 100; //将单位从m转成cm
        }
    }
    else
    {
        // 身体部分
        for (int i = 1; i < 21; i++)
        {
            bvhDisp[i] = getGlobalBoneStart(i, BoneStart, BoneEnd);
        }

        // 手部Global位移+自身的offset
        for (int i = 25; i < 49; i++) // 右手
        {
            // 手部骨节长度
            bvhDisp[i] = FingersOffsetL[i - 25] * 100;
            if (i == 25 || i == 29 || i == 34 || i == 39 || i == 44)
            {
                bvhDisp[i] += bvhDisp[10];
            }
            else
            {
                bvhDisp[i] += FingersOffsetL[i - 1];
            }
        }
        for (int i = 56; i < 80; i++) // 左手
        {
            // 手部骨节长度
            bvhDisp[i] = FingersOffsetL[i - 56] * 100;
            if (i == 56 || i == 60 || i == 65 || i == 70 || i == 75)
            {
                bvhDisp[i] += bvhDisp[14];
            }
            else
            {
                bvhDisp[i] += FingersOffsetL[i - 1];
            }
        }
    }
}

float* BVHDataConvert::FillBuffer(float* buffer)
{
    static int len = sizeof(float) * 3;
    float* tmpbuffer = buffer;

    if (IsWithDisp)
    {
        // 下半身
        for (int i = 0; i < 7; i++)
        {
            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 脊柱、neck、头
        for (int i = 20; i > 14; i--)
        {
            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 右手手臂
        for (int i = 7; i < 11; i++)
        {
            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 右手指
        for (int i = 25; i < 49; i++)
        {
            if (i == 28 || i == 33 || i == 38 || i == 43 || i == 48) continue; // 跳过指甲部分

            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 左手手臂(跳过右手手指:19根指骨骼=3根大拇指骨骼+4根手指*每根手指4根骨骼，每根骨骼3个角度三个位移114=19*6)
        //tmpbuffer += 114;
        for (int i = 11; i < 15; i++)
        {
            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 左手指
        for (int i = 56; i < 80; i++)
        {
            if (i == 59 || i == 64 || i == 69 || i == 74 || i == 79) continue; // 跳过指甲部分

            memcpy(tmpbuffer, bvhDisp[i].get(), len);
            tmpbuffer += 3;
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
    }
    else
    {
        // hip位移
        memcpy(tmpbuffer, bvhDisp[0].get(), len);
        tmpbuffer += 3;

        // 下半身
        for (int i = 0; i < 7; i++)
        {
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 脊柱、neck、头
        for (int i = 20; i > 14; i--)
        {
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 右手手臂
        for (int i = 7; i < 11; i++)
        {
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 右手指
        for (int i = 25; i < 48; i++)
        {
            if (i == 28 || i == 33 || i == 38 || i == 43 || i == 48) continue;

            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 左手手臂(跳过右手手指:19根指骨骼=3根大拇指骨骼+4根手指*每根手指4根骨骼，每根骨骼3个角度57=19*3)
        //tmpbuffer += 57;
        for (int i = 11; i < 15; i++)
        {
            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
        // 左手指
        for (int i = 56; i < 79; i++)
        {
            if (i == 59 || i == 64 || i == 69 || i == 74 || i == 79) continue;

            memcpy(tmpbuffer, bvhAngle[i].get(), len);
            tmpbuffer += 3;
        }
    }

    return buffer;
}

// 获取子节点相对于父节点的局部位移（偏移量）
Point3D_t BVHDataConvert::getLocalBoneStart(int boneIndex, double* X, QUATERNION_t* ql, Point3D_t* BoneStart, Point3D_t* BoneEnd)
{
    // 模型坐标系（左上前）下的骨骼偏移量
    Point3D_t offset = bvhHeaderRef->getOFFSET(boneIndex);

    // 世界坐标系下的骨骼，父节点末端、子节点的起始位置
    Point3D_t parentEnd;
    Point3D_t childStart;
    if (boneIndex == 4)
    {
        parentEnd = BoneEnd[1];
        childStart = BoneStart[6];
    }
    else if (boneIndex == 7)
    {
        parentEnd = BoneStart[19];
        childStart = BoneStart[9];
    }
    else if (boneIndex == 11)
    {
        parentEnd = BoneStart[19];
        childStart = BoneStart[13];
    }
    else if (boneIndex >= 15 && boneIndex < 20)
    {
        parentEnd = BoneStart[boneIndex + 3];
        childStart = BoneEnd[boneIndex + 2];
    }
    else if (boneIndex == 20)
    {
        parentEnd = BoneEnd[0];
        childStart = BoneEnd[22];
    }
    else
    {
        parentEnd = BoneEnd[boneIndex + 1];
        childStart = BoneStart[boneIndex + 2];
    }

    // 世界坐标系下的骨骼错位矢量
    Point3D_t delta = childStart - parentEnd;

    // wpf绘图坐标系转成世界坐标系
    delta = Point3D_t(delta.X, -delta.Y, -delta.Z);

    //MatrixXd vdeta(3,1);
    Vector3d vdeta;
    vdeta = Vector3d(delta.X, delta.Y, delta.Z);
    //ToMatrix(vdeta, delta, TRUE);

    // 构建相对于父节点的旋转矩阵
    int parentId = getParentId(boneIndex);
    Point3D_t newX;
    Point3D_t newY;
    Point3D_t newZ;
    q_v_qc(&ql[parentId], &caliRef->VboneList[parentId], &newX); // 下
    q_v_qc(&ql[parentId], &caliRef->Vbone1List[parentId], &newY); // 左

    // 三个基矢量构建旋转矩阵（左x上y前z）
    //rmatrix m(3, 3);
    Matrix3d m;
    if (boneIndex > 7 && boneIndex <= 10) // 右手臂(除去肩，因为肩的父节点是脊柱)
    {
        //MatrixXd tmpM(1,3);
        Vector3d tmpM;
        // [-b, -b1 x b, b1]
        newZ = cross(newY, newX);	// 前

        tmpM = Vector3d(-newX.X, -newX.Y, -newX.Z);
        m.row(0) = tmpM;
        tmpM = Vector3d(newZ.X, newZ.Y, newZ.Z);
        m.row(1) = tmpM;
        tmpM = Vector3d(-newY.X, -newY.Y, -newY.Z);
        m.row(2) = tmpM;

    }
    else if (boneIndex > 11 && boneIndex <= 14) // 左手臂(除去肩，因为肩的父节点是脊柱)
    {
        //MatrixXd tmpM(1,3);
        Vector3d tmpM;
        // [b, -b1 x b, -b1]
        newZ = cross(newY*(-1), newX);	// 前

        tmpM = Vector3d(newX.X, newX.Y, newX.Z);
        m.row(0) = tmpM;
        tmpM = Vector3d(newZ.X, newZ.Y, newZ.Z);
        m.row(1) = tmpM;
        tmpM = Vector3d(-newY.X, -newY.Y, -newY.Z);
        m.row(2) = tmpM;

    }
    else // 其他（脚掌不需特殊处理，因为旋转点用的是父节点的）
    {
        //MatrixXd tmpM(1,3);
        Vector3d tmpM;
        // [b1, -b, b x b1]
        newZ = cross(newX, newY);	// 前

        tmpM = Vector3d(newY.X, newY.Y, newY.Z);
        m.row(0) = tmpM;
        tmpM = Vector3d(-newX.X, -newX.Y, -newX.Z);
        m.row(1) = tmpM;
        tmpM = Vector3d(newZ.X, newZ.Y, newZ.Z);
        m.row(2) = tmpM;

    }

    // 将世界坐标矢量vdeta转换到模型坐标系（左上前）
    //rmatrix v = m * vdeta;
    Vector3d v = m * vdeta;

    delta = Point3D_t(v(0), v(1), v(2));

    // 在标准矢量上补上错位矢量
    offset = offset + delta;
    offset = offset * 100.0; // m to cm

    return offset;
}

Point3D_t BVHDataConvert::getGlobalBoneStart(int boneIndex, Point3D_t* BoneStart, Point3D_t* BoneEnd)
{
#if (0)
    Point3D_t boneStart;
    if(boneIndex==0)
    {
        boneStart = BoneEnd[0];
    }
    else
    {
        boneStart = BoneStart[boneIndex+2]; // 自己的开始端
    }

    //boneStart = Point3D_t(boneStart.X, boneStart.Z, -boneStart.Y) * 100; // m to cm
    boneStart = Point3D_t(boneStart.X, boneStart.Z, -boneStart.Y) * 100; // BoneStart、BoneEnd为(X, -Y, dZ-Z)，输出应该是(X, dZ-Z, Y)

    q_v_qc(&faceRot, &boneStart, &boneStart); // hip做了旋转，其他骨骼也得做旋转

    return boneStart;
#endif

    Point3D_t boneStart;
    Point3D_t boneEnd = BoneEnd[boneIndex + 2];

    if (boneIndex == 0)                       // Hip：上端为BoneStart、下端为BoneEnd
    {
        boneStart = BoneEnd[0];
        boneEnd = BoneStart[0];
    }
    else if (boneIndex >= 15 && boneIndex < 21) // 脊柱：上端为BoneStart、下端为BoneEnd
    {
        boneStart = BoneEnd[boneIndex + 2];
        boneEnd = BoneStart[boneIndex + 2];
    }
    else
    {
        boneStart = BoneStart[boneIndex + 2]; // 自己的开始端
        boneEnd = BoneEnd[boneIndex + 2];
    }

    boneStart = Point3D_t(boneStart.X, boneStart.Z, -boneStart.Y) * 100; // BoneStart、BoneEnd为(X, -Y, dZ-Z)，输出应该是(X, dZ-Z, Y)

    q_v_qc(&faceRot, &boneStart, &boneStart); // hip做了旋转，其他骨骼也得做旋转

    return boneStart;
}

int BVHDataConvert::getParentId(int boneId)
{
    int id = 0;
    if (boneId <= 0 || boneId == 1 || boneId == 4 || boneId == 20)
    {
        id = 0;
    }
    else if (boneId == 7 || boneId == 11)
    {
        id = 17;
    }
    else if (boneId >= 15 && boneId < 20)
    {
        id = boneId + 1;
    }
    else
    {
        id = boneId - 1;
    }

    return id;
}

// 平滑处理
void BVHDataConvert::smoothness(double* x, QUATERNION_t* ql)
{
    Point3D_t xhip(x[0], x[1], x[2]);
    QUATERNION_t qhip = ql[0];

    Point3D_t Xhip;
    QUATERNION_t Qhip;

    smoothnessBufferX.push_back(xhip);
    if (smoothnessBufferX.size() > 10)
    {
        smoothnessBufferX.pop_front();
    }

    // Hip 位移平均值
    int count = smoothnessBufferX.size();
    list<Point3D_t>::iterator itor = smoothnessBufferX.begin();
    while (itor != smoothnessBufferX.end())
    {
        Xhip += *itor;
        itor++;
    }
    Xhip = Xhip / count;

    x[0] = Xhip.X;
    x[1] = Xhip.Y;
    x[2] = Xhip.Z;

    smoothnessBufferQ.push_back(qhip);
    if (smoothnessBufferQ.size() > 10)
    {
        smoothnessBufferQ.pop_front();
    }

    // Hip 四元数平均值
    list<QUATERNION_t>::iterator itorQ = smoothnessBufferQ.begin();
    count = 0;
    while (itorQ != smoothnessBufferQ.end())
    {
        count++;

        if (count == smoothnessBufferQ.size()) break; // 最后一个是当前点，不处理

        QUATERNION_t buffQ = *itorQ;

        QUATERNION_t qd1;
        QUATERNION_t qhipc = qhip.Conjuct();
        Quat_Mult(&buffQ, &qhipc, &qd1);

        if (qd1.qs < 0) qd1.qs = -qd1.qs;
        if (qd1.qs > 0.9999) qd1.qs = 0.9999f;

        double theta = acos(qd1.qs / count);
        double stheta = sin(theta);

        QUATERNION_t qd2(cos(theta), stheta*qd1.qx, stheta*qd1.qy, stheta*qd1.qz);
        //qd2.Normalize();
        Quat_Mult(&qd2, &qhip, &Qhip);

        itorQ++;
    }

    ql[0] = Qhip;
}

// 平滑处理
QUATERNION_t BVHDataConvert::smoothness(QUATERNION_t* q1, QUATERNION_t* q2)
{
    double cosOmega = q1->qs*q2->qs + q1->qx*q2->qx + q1->qy*q2->qy + q1->qz*q2->qz;
    if (cosOmega < 0.0f)
    {
        q1->qs *= -1;
        q1->qx *= -1;
        q1->qy *= -1;
        q1->qz *= -1;
        cosOmega *= -1;
    }
    double t = 0.1;
    double k1, k2;
    if (cosOmega > 0.9999)
    {
        k1 = 1.0 - t;
        k2 = t;
    }
    else
    {
        double sinOmega = sqrt(1.0 - cosOmega*cosOmega);
        double omega = atan2(sinOmega, cosOmega);
        double oneOverSinOmega = 1.0 / sinOmega;
        k1 = sin((1.0 - t) * omega) * oneOverSinOmega;
        k2 = sin(t * omega) * oneOverSinOmega;
    }
    //QUATERNION_t a(q1->qs*k1 + q2->qs*k2, q1->qx*k1 + q2->qx*k2, q1->qy*k1 + q2->qy*k2, q1->qz*k1 + q2->qz*k2);
    //a.Normalize();
    //return a;


    return 	QUATERNION_t(q1->qs*k1 + q2->qs*k2, q1->qx*k1 + q2->qx*k2, q1->qy*k1 + q2->qy*k2, q1->qz*k1 + q2->qz*k2);
}

// 计算每根骨骼的位置
void BVHDataConvert::calcBonePos(double *x)
{
    Point3D_t pos;
    for (int i = 0; i < 21; i++) // 身体的21根骨骼的积分数据，所以这里必须是21
    {
        // 模块的位置
        pos = Point3D_t(x[i * 6], x[i * 6 + 1], x[i * 6 + 2]);// - X0;
        // 根据FaceRot做朝向转换
        q_v_qc(&faceRot, &pos, &pos);
        // 骨骼的模块位置
        bonesSensorGlobalPos[i] = Point3D_t(pos.X, kalmRef->dZ - pos.Z, pos.Y) * 100.0;
    }
}

Point3D_t BVHDataConvert::calcNewZ(int boneId, QUATERNION_t* ql)
{
    QUATERNION_t q;
    Quat_Mult(&faceRot, &ql[boneId], &q);

    Point3D_t newX;
    q_v_qc(&q, &caliRef->VboneList[boneId], &newX);
    newX = Point3D_t(newX.X, -newX.Z, newX.Y);

    Point3D_t newY;
    q_v_qc(&q, &caliRef->Vbone1List[boneId], &newY);
    newY = Point3D_t(newY.X, -newY.Z, newY.Y);

    return cross(newX, newY);
}

Point3D_t BVHDataConvert::calcEndPt(int boneId, double* x)
{
    Point3D_t startP = Point3D_t(x[boneId * 6], x[boneId * 6 + 1], x[boneId * 6 + 2]);// - X0;
    q_v_qc(&faceRot, &startP, &startP);

    Point3D_t ep(startP.X, kalmRef->dZ - startP.Z, startP.Y);
    ep = ep * 100.0;
    ep.Y += 1.7f / 2;

    return ep;
}

Point3D_t BVHDataConvert::calcDtemp(int boneId, QUATERNION_t* ql)
{
    QUATERNION_t q;
    Quat_Mult(&faceRot, &ql[boneId], &q);

    Point3D_t vbone;
    q_v_qc(&q, &caliRef->VboneList[boneId], &vbone);

    Point3D_t dtemp;
    if (boneId == 3 || boneId == 6)
    {
        dtemp = vbone * 100 * (-kalmRef->LD[boneId]);
    }
    else
    {
        dtemp = vbone * 100 * kalmRef->LU[boneId];
    }
    dtemp = Point3D_t(dtemp.X, -dtemp.Z, dtemp.Y) * (-1);

    return dtemp;
}

void BVHDataConvert::getSmoothMatrix(Matrix3d& rot1, int boneId, int boneId1, int boneId2, QUATERNION_t* ql, PNBOOL isNearBone1)
{
    QUATERNION_t q;

    Quat_Mult(&faceRot, &ql[boneId1], &q);

    Point3D_t newX1;
    q_v_qc(&q, &caliRef->VboneList[boneId1], &newX1);
    newX1 = Point3D_t(newX1.X, -newX1.Z, newX1.Y);

    Point3D_t newY1;
    q_v_qc(&q, &caliRef->Vbone1List[boneId1], &newY1);
    newY1 = Point3D_t(newY1.X, -newY1.Z, newY1.Y);


    Quat_Mult(&faceRot, &ql[boneId2], &q);

    Point3D_t newX2;
    q_v_qc(&q, &caliRef->VboneList[boneId2], &newX2);
    newX2 = Point3D_t(newX2.X, -newX2.Z, newX2.Y);

    Point3D_t newY2;
    q_v_qc(&q, &caliRef->Vbone1List[boneId2], &newY2);
    newY2 = Point3D_t(newY2.X, -newY2.Z, newY2.Y);

    // 取相邻两根脊柱的算术平均数
    Point3D_t newX;
    Point3D_t newY;
    if (boneId == 16)
    {
        newX = smoothRatio(newX1, newX2);
        newY = smoothRatio(newY1, newY2);
    }
    else if (boneId == 17)
    {
        newX = newX1 * 0.95 + newX2 * 0.05;
        newY = newY1 * 0.95 + newY2 * 0.05;
    }
    else if (boneId == 19)
    {
        newX = newX1 * 0.75 + newX2 * 0.25;
        newY = newY1 * 0.75 + newY2 * 0.25;
    }
    else if (boneId == 20)
    {
        newX = newX1 * 0.4 + newX2 * 0.6;
        newY = newY1 * 0.4 + newY2 * 0.6;
    }
    newX.Normalize();
    newY.Normalize();

    /*
    double radio = 0;
    if(boneId==16 || boneId==17)
    {
    radio = isNearBone1?kalmRef->breastbone:(1.0-kalmRef->breastbone);
    }
    else
    {
    radio = isNearBone1?kalmRef->vertebra:(1.0-kalmRef->vertebra);
    }

    Point3D_t newX = newX1 * radio + newX2 * (1.0-radio);
    Point3D_t newY = newY1 * radio + newY2 * (1.0-radio);
    */

    Point3D_t newZ = cross(newX, newY);
    rot1.col(0) = Vector3d(newX.X, newX.Y, newX.Z);
    rot1.col(1) = Vector3d(newY.X, newY.Y, newY.Z);
    rot1.col(2) = Vector3d(newZ.X, newZ.Y, newZ.Z);

}

Matrix3d BVHDataConvert::getRot(int boneId, QUATERNION_t* ql, PNBOOL isSubNode)
{
    Matrix3d rot1;
    getRot1(rot1, boneId, ql, isSubNode);

    Matrix3d rot2 = getRot2(boneId, isSubNode);
    return rot1*rot2.inverse();

    //ret.gels(FALSE, rot2, rot1, vErr);
    //return ret;
}

void BVHDataConvert::getRot1(Matrix3d& rot1, int boneId, QUATERNION_t* ql, PNBOOL isSubNode)
{
    QUATERNION_t q;

    // 将调节角度叠加到输入Q上
    Quat_Mult(&faceRot, &ql[boneId], &q);

    // 将骨骼朝向从模块坐标转为大地坐标
    Point3D_t newX;
    q_v_qc(&q, &caliRef->VboneList[boneId], &newX);
    // 转为bvh坐标系
    newX = Point3D_t(newX.X, -newX.Z, newX.Y) * (isSubNode ? 1 : -1);

    // 将骨骼垂直朝向从模块坐标转为大地坐标（双手朝后，其他都朝左，包括双脚）
    Point3D_t newY;
    q_v_qc(&q, &caliRef->Vbone1List[boneId], &newY);
    // 转为bvh坐标系
    newY = Point3D_t(newY.X, -newY.Z, newY.Y) * (isSubNode ? 1 : -1);

    // 不带位移的bvh的防抖处理
    if (boneId == 1 && useR)
    {
        newX = AD1;
        newY = V1;
    }
    else if (boneId == 2 && useR)
    {
        newX = DB1;
        newY = V1;
    }
    else if (boneId == 4 && useL)
    {
        newX = AD2;
        newY = V2;
    }
    else if (boneId == 5 && useL)
    {
        newX = DB2;
        newY = V2;
    }

    Point3D_t newZ = cross(newX, newY);

    // 人人网，手臂限制功能
    if (IsRestrictArmRotation)
    {
        static Point3D_t rightArmZ;
        static Point3D_t leftArmZ;

        // 右臂
        if (boneId == 8)
        {
            rightArmZ = getRestrictArmNewZ(8, ql);
            newZ = rightArmZ;
            newY = cross(newZ, newX);
        }
        else if (boneId == 9)
        {
            newZ = rightArmZ;
            newY = cross(newZ, newX);
        }

        // 左臂
        if (boneId == 12)
        {
            leftArmZ = getRestrictArmNewZ(12, ql);
            newZ = leftArmZ;
            newY = cross(newZ, newX);
        }
        else if (boneId == 13)
        {
            newZ = leftArmZ;
            newY = cross(newZ, newX);
        }
    }

    rot1.col(0) = Vector3d(newX.X, newX.Y, newX.Z);
    rot1.col(1) = Vector3d(newY.X, newY.Y, newY.Z);
    rot1.col(2) = Vector3d(newZ.X, newZ.Y, newZ.Z);
}

Matrix3d BVHDataConvert::getRot2(int boneId, PNBOOL isSubNode)
{
    // 构造矩阵的三个基矢量
    Point3D_t oldX(0, 1, 0);
    Point3D_t oldY(-1, 0, 0);
    Point3D_t oldZ = cross(oldX, oldY);
    if (isSubNode)
    {
        oldX = Point3D_t(0, -1, 0);
        oldY = Point3D_t(1, 0, 0);
        oldZ = cross(oldX, oldY);
    }

    if (boneId == 3)
    {
        double footLen = (kalmRef->LU[3] + kalmRef->LD[3]) * 100.0;
        oldX = Point3D_t(0, -(footLen / 10.0), footLen);
        oldX = oldX / sqrt(oldX.dot());
        oldY = Point3D_t(1, 0, 0);
        oldZ = cross(oldX, oldY);
    }
    else if (boneId == 6)
    {
        double footLen = (kalmRef->LU[6] + kalmRef->LD[6]) * 100.0;
        oldX = Point3D_t(0, -(footLen / 10.0), footLen);
        oldX = oldX / sqrt(oldX.dot());
        oldY = Point3D_t(1, 0, 0);
        oldZ = cross(oldX, oldY);
    }
    else if (boneId >= 7 && boneId < 11)
    {
        oldX = Point3D_t(-1, 0, 0);
        oldX = oldX / sqrt(oldX.dot());
        oldY = Point3D_t(0, 0, -1);
        oldZ = cross(oldX, oldY);
    }
    else if (boneId >= 11 && boneId < 15)
    {
        oldX = Point3D_t(1, 0, 0);
        oldX = oldX / sqrt(oldX.dot());
        oldY = Point3D_t(0, 0, -1);
        oldZ = cross(oldX, oldY);
    }

    // 双手
    if ((boneId >= 25 && boneId < 50) || (boneId >= 56 && boneId < 80))
    {
        if (boneId >= 25 && boneId < 50)  // 右手
        {
            oldX = Point3D_t(-1, 0, 0);
        }
        else                              // 左手
        {
            oldX = Point3D_t(1, 0, 0);
        }

        // 初始Y和Z
        oldY = Point3D_t(0, 0, 1);
        oldZ = cross(oldX, oldY);

        // 修正大拇指姿态
        if ((boneId >= 25 && boneId <= 28) || (boneId >= 56 && boneId <= 59))
        {
            // save oldX
            Point3D_t Rv = oldX;
			
			//double theta = 45 * PI / 180;
			//double sqrt2 = sin(45 * PI / 180);
			//Point3D_t bone = Zd*(sqrt2*cos(theta)) + Xd*(-1)*(sqrt2*cos(theta)) + Point3D_t(0, 0, 1)*sin(theta);
			//Point3D_t bone2 = Zd*(sqrt2*sin(theta)) + Xd*(-1)*(sqrt2*sin(theta)) + Point3D_t(0, 0, 1)*-cos(theta);
			//Point3D_t bone1 = cross(bone2, bone);

			if ((boneId >= 25 && boneId <= 28)) // 右手
			{
				oldX = Rv * 0.5 + oldY * 0.5 + oldZ * (-0.25);
				oldX.Normalize();

				//oldZ = Rv * (-0.5) + oldY * 0.5 + oldZ * 0.25;
				//oldZ.Normalize();
				
				// 叉乘的到Y
				oldY = cross(oldZ, oldX);
				oldY.Normalize();
			}
			else  // 左手
			{
				oldX = Rv * 0.5 + oldY * 0.5 + oldZ * (0.25);
				oldX.Normalize();

				//oldZ = Rv *(0.5) + oldY * (-0.5) + oldZ * (0.25);
				//oldZ.Normalize();

				// 叉乘的到Y
				oldY = cross(oldZ, oldX);
				oldY.Normalize();
			}
			
            // 重新计算Z，确保正交
            oldZ = cross(oldX, oldY);


			/*
			// save oldX
            Point3D_t Rv = oldX;

            oldX = oldX * 0.5 + oldY * 0.5 + oldZ * (-0.25);
            oldX.Normalize();

            oldZ = Rv * 0.5 + oldY * (-0.5) + oldZ * (-0.25);
            if ((boneId >= 25 && boneId <= 28)) // 右手
            {
                oldZ = oldZ * (-1);
            }
            oldZ.Normalize();

            // 叉乘的到Y
            oldY = cross(oldZ, oldX);
            oldY.Normalize();

            // 重新计算Z，确保正交
            oldZ = cross(oldX, oldY);
			*/
			
			/*
			double theta = 45 * PI / 180;
			double sqrt2 = sin(60 * PI / 180);
			Point3D_t bone = oldZ*(sqrt2*cos(theta)) + oldX*(sqrt2*cos(theta)) + Point3D_t(0, 0, 1)*sin(theta);
			Point3D_t bone2 = oldZ*(sqrt2*sin(theta)) + oldX*(sqrt2*sin(theta)) + Point3D_t(0, 0, 1)*-cos(theta);
			Point3D_t bone1 = cross(bone2, bone);

			oldX = bone;
			oldY = bone2;
			oldZ = bone1;
			*/
        }
        else if (boneId == 29 || boneId == 60) // 食指指根
        {
            oldX = oldX * cos(InHandFingerAngle * PI / 180) + oldY * sin(InHandFingerAngle * PI / 180);
            oldX = oldX / sqrt(oldX.dot());
            oldY = cross(oldZ, oldX);
            oldY = oldY / sqrt(oldY.dot());
        }
        else if (boneId == 39 || boneId == 70) // 无名指指根
        {
            oldX = oldX * cos(InHandFingerAngle * PI / 180) + oldY * -sin(InHandFingerAngle * PI / 180);
            oldX = oldX / sqrt(oldX.dot());
            oldY = cross(oldZ, oldX);
            oldY = oldY / sqrt(oldY.dot());
        }
        else if (boneId == 44 || boneId == 75) // 小指指根
        {
            oldX = oldX * cos(InHandFingerAngle * PI / 180) + oldY * -sin(InHandFingerAngle * PI / 180);
            oldX = oldX / sqrt(oldX.dot());
            oldY = cross(oldZ, oldX);
            oldY = oldY / sqrt(oldY.dot());
        }
    }

    Matrix3d rot2;
    rot2.col(0) = Vector3d(oldX.X, oldX.Y, oldX.Z);
    rot2.col(1) = Vector3d(oldY.X, oldY.Y, oldY.Z);
    rot2.col(2) = Vector3d(oldZ.X, oldZ.Y, oldZ.Z);

    return rot2;
}

// 人人网，手臂限制功能
Point3D_t BVHDataConvert::getRestrictArmNewZ(int boneId, QUATERNION_t* ql)
{
    QUATERNION_t q;

    // 上臂的轴线
    Quat_Mult(&faceRot, &ql[boneId], &q);
    Point3D_t newX1;
    q_v_qc(&q, &caliRef->VboneList[boneId], &newX1);
    newX1 = Point3D_t(newX1.X, -newX1.Z, newX1.Y);
    // 上臂的Y分量
    Point3D_t newY1;
    q_v_qc(&q, &caliRef->Vbone1List[boneId], &newY1);
    newY1 = Point3D_t(newY1.X, -newY1.Z, newY1.Y);

    // 前臂的轴线
    Quat_Mult(&faceRot, &ql[boneId + 1], &q);
    Point3D_t newX2;
    q_v_qc(&q, &caliRef->VboneList[boneId + 1], &newX2);
    newX2 = Point3D_t(newX2.X, -newX2.Z, newX2.Y);

    // 求newZ
    Point3D_t armZ = cross(newX1, newX2) * (-1);

    // 如果上臂与前臂平行（手臂伸直）
    if (armZ.dot() < 1e-6)
    {
        // use
        if (boneId == 8)
        {
            if (!IsFirstRight)
            {
                armZ = rightArmZ0;
            }
            else
            {
                armZ = newY1;
                IsFirstRight = FALSE;
            }
        }
        else if (boneId == 12)
        {
            if (!IsFirstLeft)
            {
                armZ = leftArmZ0;
            }
            else
            {
                armZ = newY1;
                IsFirstLeft = FALSE;
            }
        }
    }

    armZ.Normalize();

    // save
    if (boneId == 8)
    {
        rightArmZ0 = armZ;
    }
    else if (boneId == 12)
    {
        leftArmZ0 = armZ;
    }

    return armZ;
}

#if 0
Point3D_t BVHDataConvert::eulerAngle(int boneId, QUATERNION_t* ql /*Point3D_t boneX, QUATERNION_t boneQ, Point3D_t vbone, Point3D_t vbone1*/)
{
    PNBOOL isSubNode = FALSE;
    if(boneId!=0 && boneId!=20)
    {
        isSubNode = TRUE;
    }

    QUATERNION_t q;
    Quat_Mult(&faceRot, &(ql[boneId]), &q);

    Point3D_t newX;
    q_v_qc(&q, &caliRef->VboneList[boneId], &newX);
    newX = Point3D_t(newX.X, -newX.Z, newX.Y) * (isSubNode?1:-1);

    Point3D_t newY;
    q_v_qc(&q, &caliRef->Vbone1List[boneId], &newY);
    newY = Point3D_t(newY.X, -newY.Z, newY.Y) * (isSubNode?1:-1);

    Point3D_t newZ = cross(newX, newY);

    Matrix3d rot1;
    rot1.col(0) = Vector3d (newX.X, newX.Y, newX.Z);
    rot1.col(1) = Vector3d (newY.X, newY.Y, newY.Z);
    rot1.col(2) = Vector3d (newZ.X, newZ.Y, newZ.Z);

    Point3D_t oldX(0, 1, 0);
    Point3D_t oldY(-1, 0, 0);
    Point3D_t oldZ = cross(oldX, oldY);
    if(isSubNode)
    {
        oldX = Point3D_t(0, -1, 0);
        oldY = Point3D_t(1, 0, 0);
        oldZ = cross(oldX, oldY);
    }

    Matrix3d rot2;
    rot2.col(0) = Vector3d (oldX.X, oldX.Y, oldX.Z);
    rot2.col(1) = Vector3d (oldY.X, oldY.Y, oldY.Z);
    rot2.col(2) = Vector3d (oldZ.X, oldZ.Y, oldZ.Z);

    MatrixXd rot = rot1 * rot2.inverse();

    QUATERNION_t rotQ = MatrixToQuat(rot);
    if(isSubNode)
    {
        rotQ.Conjuct();
        Quat_Mult(&rotQ, &parentQ, &rotQ);
        rotQ.Conjuct();
    }
    parentQ = rotQ;

    if(boneId==0)
    {
        hipQ = rotQ;
    }
    else if(boneId==1 || boneId==4 || boneId==20)
    {
        parentQ = hipQ;
    }

    // 计算三个转角
    return QuatToAng(RotOrder, rotQ);
}
#endif

void BVHDataConvert::GetOffset(float* xOffset, float* yOffset, QUATERNION_t* facerot)
{
    *xOffset = XOffset;
    *yOffset = YOffset;
    *facerot = faceRot;
}

void BVHDataConvert::SetFaceAngle(double angle)
{
    if (kalmRef == NULL) return;

    faceRot = FaceRotFromAngle(angle);

    //	
    //// 获取hip矢量
    //Point3D_t hipVect;
    //hipVect.X = kalmRef->X(1,1);	
    //hipVect.Y = kalmRef->X(1,1);	
    //hipVect.Z = kalmRef->X(1,1);	

    //// 归一化
    //hipVect.Normalize();

    //// 垂直方向上的分量
    //double tmp = hipVect.dot(Point3D_t(0, 1, 0));
    //if (tmp>1 )
    //{
    //	tmp = 1;
    //}
    //else if(tmp < -1)
    //{
    //	tmp = -1;
    //}

    //// 使w在0~2PI之间
    //int d = 10000000000;// 小数位精度
    //int wTmp = w * d;
    //wTmp %= (int)(2*PI * d);
    //w = wTmp/(double)d;

    //double ang = acos(tmp) + w;

    //QUATERNION_t faceQ = QUATERNION_t(cos(w/2.0), 0.0, 0.0, sin(w/2.0));	

    //Point3D_t y1;
    //q_v_qc(&faceQ, &hipVect, &y1);
    //
    //if(abs(y1.dot(Point3D_t(0, 1, 0)) - 1) > 1e-3)
    //{
    //	faceQ = QUATERNION_t(cos(-w/2.0), 0.0, 0.0, sin(-w/2.0));
    //}

    ////faceRot = faceQ;

    //Quat_Mult(&faceRot, &faceQ, &faceRot);

    //return;
}

void BVHDataConvert::wipeoffWobble(double* x, QUATERNION_t* ql)
{
    useL = TRUE;
    useR = TRUE;

    /*
    // 四元数插值
    for(int i=0; i< boneCount; i++)
    {
    ql[i] = smoothness(&preQ[i], &ql[i]);
    preQ[i] = ql[i];
    }
    */

    //smoothness(x, ql);

    // 记录髋关节点
    Point3D_t A = calcEndPt(0, x) + Point3D_t(0, -expLen, 0); // 把髋关节降5cm避免骨节分开;

    Point3D_t hipVbone;
    Point3D_t hipVbone1;
    QUATERNION_t q;
    Quat_Mult(&faceRot, &ql[0], &q);
    q_v_qc(&q, &caliRef->VboneList[0], &hipVbone);
    q_v_qc(&q, &caliRef->Vbone1List[0], &hipVbone1);

    Point3D_t dtemp;
    dtemp = (hipVbone * (-kalmRef->LU[0]) + hipVbone1 * kalmRef->LW / 2.0) * 100; // 位移量
    dtemp = Point3D_t(dtemp.X, -dtemp.Z, dtemp.Y) * (-1);
    Point3D_t A1 = A + dtemp; // 记录左髋关节点 髋宽0.2m

    dtemp = (hipVbone * (-kalmRef->LU[0]) - hipVbone1 * kalmRef->LW / 2.0) * 100; // 位移量
    dtemp = Point3D_t(dtemp.X, -dtemp.Z, dtemp.Y) * (-1);
    Point3D_t A2 = A + dtemp; // 记录右髋关节点 髋宽0.2m

    // 右膝关节节点获取
    Point3D_t startP = calcEndPt(1, x);
    Point3D_t C1 = startP - calcDtemp(1, ql); // 记录右膝关节点
    Point3D_t F2 = calcNewZ(1, ql);

    // Leg2
    Point3D_t F3 = calcNewZ(2, ql);

    // 右脚节点获取
    startP = calcEndPt(3, x);
    Point3D_t B1 = startP - calcDtemp(3, ql); // 记录右脚关节点

    // 左膝关节节点获取
    startP = calcEndPt(4, x);
    Point3D_t C2 = startP - calcDtemp(4, ql);
    Point3D_t F5 = calcNewZ(4, ql); // 记录左膝关节点

    // Leg4	
    Point3D_t F6 = calcNewZ(5, ql);

    // 左脚节点获取
    startP = calcEndPt(6, x);
    Point3D_t B2 = startP - calcDtemp(6, ql); // 记录左脚关节点

    // right
    double L2 = (kalmRef->LU[1] + kalmRef->LD[1]) * 100;
    double L3 = (kalmRef->LU[2] + kalmRef->LD[2]) * 100;

    C1 = C1 + (F2 + F3) * 30;  //把膝盖往前方移动，防止那啥
    C2 = C2 + (F5 + F6) * 30;  //把膝盖往前方移动，防止那啥

    double powX = pow((A1.X - B1.X), 2);
    double powY = pow((A1.Y - B1.Y), 2);
    double powZ = pow((A1.Z - B1.Z), 2);
    double tot = powX + powY + powZ;

    double cosA1 = (pow(L2, 2) - pow(L3, 2) + tot) / (2 * L2*pow(tot, 0.5));
    double cosB1 = (pow(L3, 2) - pow(L2, 2) + tot) / (2 * L3*pow(tot, 0.5));
    Point3D_t V((A1.Z - C1.Z)*(B1.Y - C1.Y) - (A1.Y - C1.Y)*(B1.Z - C1.Z), (A1.X - C1.X)*(B1.Z - C1.Z) - (A1.Z - C1.Z)*(B1.X - C1.X), (A1.Y - C1.Y)*(B1.X - C1.X) - (A1.X - C1.X)*(B1.Y - C1.Y));
    V1 = V / sqrt(V.dot());
    if (abs(cosA1) > 1 || abs(cosB1) > 1)
    {
        useR = FALSE;
    }

    // 求AD,DB矢量
    double AA1 = acos(cosA1);
    double BB1 = acos(cosB1);

    Point3D_t dq2tmp = V1 * (-sin(AA1 / 2.0));
    Point3D_t dq3tmp = V1 * sin(BB1 / 2.0);
    QUATERNION_t dq2(cos(AA1 / 2.0), dq2tmp.X, dq2tmp.Y, dq2tmp.Z);
    QUATERNION_t dq3(cos(BB1 / 2.0), dq3tmp.X, dq3tmp.Y, dq3tmp.Z);
    Point3D_t AB1 = (A1 - B1) / sqrt((A1 - B1).dot()) * (-1);

    q_v_qc(&dq2, &AB1, &AD1);
    q_v_qc(&dq3, &AB1, &DB1);

    // left
    L2 = (kalmRef->LU[4] + kalmRef->LD[4]) * 100;
    L3 = (kalmRef->LU[5] + kalmRef->LD[5]) * 100;

    powX = pow((A2.X - B2.X), 2);
    powY = pow((A2.Y - B2.Y), 2);
    powZ = pow((A2.Z - B2.Z), 2);
    tot = powX + powY + powZ;

    double cosA2 = (pow(L2, 2) - pow(L3, 2) + tot) / (2 * L2*pow(tot, 0.5));
    double cosB2 = (pow(L3, 2) - pow(L2, 2) + tot) / (2 * L3*pow(tot, 0.5));
    V = Point3D_t((A2.Z - C2.Z)*(B2.Y - C2.Y) - (A2.Y - C2.Y)*(B2.Z - C2.Z), (A2.X - C2.X)*(B2.Z - C2.Z) - (A2.Z - C2.Z)*(B2.X - C2.X), (A2.Y - C2.Y)*(B2.X - C2.X) - (A2.X - C2.X)*(B2.Y - C2.Y));
    V2 = V / sqrt(V.dot());

    if (abs(cosA2) > 1 || abs(cosB2) > 1)
    {
        useL = FALSE;
    }

    // 求AD,DB矢量
    double AA2 = acos(cosA2);
    double BB2 = acos(cosB2);

    Point3D_t dq5tmp = V2 * (-sin(AA2 / 2.0));
    Point3D_t dq6tmp = V2 * sin(BB2 / 2.0);
    QUATERNION_t dq5(cos(AA2 / 2), dq5tmp.X, dq5tmp.Y, dq5tmp.Z);
    QUATERNION_t dq6(cos(BB2 / 2), dq6tmp.X, dq6tmp.Y, dq6tmp.Z);
    Point3D_t AB2 = (A2 - B2) / sqrt((A2 - B2).dot()) * (-1);

    q_v_qc(&dq5, &AB2, &AD2);
    q_v_qc(&dq6, &AB2, &DB2);
}

void BVHDataConvert::fingerDataExchange()
{
    static int len = sizeof(float) * 3; 

	// 大拇指处理
    //bvhAngle[25] = bvhAngle[26] / 5; // 右大拇指指根 
    //bvhAngle[25].X += 20; // 外张  左右和上下
    //bvhAngle[25].Y += 15; // 外拧
    //bvhAngle[25].Z += 15; // 前倾

	//bvhAngle[26] = bvhAngle[26] * 4 / 5; // 右大拇指指根
	
	//bvhAngle[56] = bvhAngle[57] / 3; // 右大拇指指根  
	//bvhAngle[56].X += -20; // 外张  左右和上下
	//bvhAngle[56].Y += 15;  // 外拧
	//bvhAngle[56].Z += -15; // 前倾
	//bvhAngle[57] = bvhAngle[57] * 2 / 3;       // 右大拇指指根  


	// 其余四个指头处理
	if (!MultiAvatarManager::Instance->IsGameMode)
    {
		// 正常模式
        fingerDataExpend(31);            // 右食指	
        fingerDataExpend(36);	         // 右中指	
        fingerDataExpend(41);            // 右无名指	
        fingerDataExpend(46);            // 右小指

        fingerDataExpend(62);            // 左食指	
        fingerDataExpend(67);	         // 左中指	
        fingerDataExpend(72);            // 左无名指	
        fingerDataExpend(77);            // 左小指
    }
    else
    {
		// 游戏模式
        fingerDataExpend(31, 10, 3);               // 右食指	
        fingerDataExpend(36, 0, -8);	           // 右中指	
        fingerDataExpend(41, -10, -20);            // 右无名指	
        fingerDataExpend(46, -15, -35);            // 右小指

        fingerDataExpend(62, -10, 3);              // 左食指	
        fingerDataExpend(67, 0, -8);	           // 左中指	
        fingerDataExpend(72, 10, -20);             // 左无名指	
        fingerDataExpend(77, 15, -35);             // 左小指
   }
}

void BVHDataConvert::fingerDataExpend(int index, double xOffset, double yOffset)
{
    // 由于三个骨节都是用的一个模块的四元数，而四元数是大地坐标系的
    // 所以算出来的角度，只有指根与掌骨间有角度，而各指节间无角度

    // 补偿系数
    static float ratio = 1.0 / 2.0;
    
    // 如果指根绑定了则只有系数
    if (boneMapRef->IsBindSensor(index-1))
    {
        bvhAngle[index].Z *= ratio;

        //  下一根 Interpolation
        bvhAngle[index + 1] = Point3D_t(0, 0, bvhAngle[index].Z);
    }
    else
    {
        // 给前一根
        // (为什么只有指根有数据:三根骨节的数据都一样，所以算出来的相对姿态为0，只有指根与掌骨姿态有差异，能算出角度来)
		bvhAngle[index - 1] = bvhAngle[index - 1];
        
		// 游戏模式限位
		if (MultiAvatarManager::Instance->IsGameMode)
        {
            bvhAngle[index - 1].X = xOffset;
            bvhAngle[index - 1].Y = yOffset;
        }

		bvhAngle[index - 1].Z *= ratio;

        // 自己反向补偿
        bvhAngle[index].X = 0;
        bvhAngle[index].Y = 0;
        bvhAngle[index].Z = bvhAngle[index - 1].Z;

        // 给下一根骨骼
        bvhAngle[index + 1].X = 0;
        bvhAngle[index + 1].Y = 0;
        bvhAngle[index + 1].Z = bvhAngle[index].Z;
    }
}

Point3D_t* BVHDataConvert::getBVHAngle()
{
    return bvhAngle.data();
}