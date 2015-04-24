%% xyz  2015.3.25
% Transform BVH data Format
% BVHData: 1*354  degree
% NaoData: 25*1 degree
%   function NaoData = BVHTransform_c( BVHData )
function [ NaoData,ROOT_Hips_RotationNew ] = BVHTransform_c( BVHData )

BVHData = double(BVHData);
ROOT_Hips_Position = BVHData(:,1:3);
BVHData = BVHData*pi/180 ; 
BVHData(:,1:3) = ROOT_Hips_Position ;

Nframes = size(BVHData,1);
rotateOrder_BVH = 213;
rotateDirection_BVH = [1,1,1];

[ C_World_Hip,C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_Neck,C_Neck_Head,C_Spine3_LShoulder,...
C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand,...
C_Hip_RUpLeg,C_RUpLeg_RLeg,C_RLeg_RFoot,C_Hip_LUpLeg,C_LUpLeg_LLeg,C_LLeg_LFoot ] = BVHDataToRotate_c( BVHData,rotateOrder_BVH,rotateDirection_BVH ) ;

%% 所有BVH的旋转数据
[ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand ) ;

[ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine2_RArm,C_Spine2_RForeArm,C_Spine2_RHand,C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
    CalculatedRightArmRotate( C_Hip_Spine3,C_Spine2_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand ) ;

[ C_Hip_LLeg,C_Spine2_LLeg,C_Spine3_LLeg ] = CalculatedLLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_LUpLeg,C_LUpLeg_LLeg ) ;
[ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg ) ;

HipYawPitch = zeros(1,size(C_Hip_Spine,1));
%% common parameters to set
% reference joint : 'Hip'  'SPine'  'Spine3'
refJoint1 = 'Spine2';
referenceOfHead = refJoint1 ;
referenceOfArm = refJoint1;
referenceOfForeArm = refJoint1;
referenceOfHand =  refJoint1;

E_0 = zeros( Nframes,3 );
RightShoulder_new = E_0;
LeftShoulder_new = E_0;
Spine3_new = E_0 ;
Spine2_new = E_0 ;
Spine_new = E_0 ;
Neck_new = E_0;

C_I = zeros( Nframes,3,3 );
for k=1:Nframes
    C_I(k,:,:) = [1 0 0; 0 1 0; 0 0 1 ];
end
C_Spine3New_NeckNew = C_I;
C_Spine2New_Spine3New = C_I ;
C_SpineNew_Spine3New = C_I ;
C_HipNew_Spine3New = C_I ;
C_Spine3New_LShoulderNew = C_I ;
C_Spine3New_RShoulderNew = C_I ;
%%% New Hip

C_World_Spine2 = CalculateC_Multi( C_World_Hip,C_Hip_Spine2 );
E_World_Spine2 = CToEuler_c( C_World_Spine2,rotateOrder_BVH,rotateDirection_BVH );
ROOT_Hips_RotationNew = E_World_Spine2 ;
%% head
C_Spine3_Head = CalculateC_Multi( C_Spine3_Neck,C_Neck_Head );
C_Spine2_Head = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_Head );
C_Spine_Head = CalculateC_Multi( C_Spine_Spine3,C_Spine3_Head );
C_Hip_Head = CalculateC_Multi( C_Hip_Spine3,C_Spine3_Head );

C_Spine2New_NeckNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_NeckNew );
C_SpineNew_NeckNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_NeckNew );
C_HipNew_NeckNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_NeckNew );

C_RefHead_Head = C_Spine2_Head ;
C_RefHeadNew_NeckNew = C_Spine2New_NeckNew;

[ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_RefHead_Head,C_RefHeadNew_NeckNew,rotateOrder_BVH ) ;

%% Right Shoulder
C_Spine2New_RShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_RShoulderNew );
C_SpineNew_RShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RShoulderNew );
C_HipNew_RShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RShoulderNew );

C_RefArm_RArm = C_Spine2_RArm;
C_RefArmNew_RShoulderNew = C_Spine2New_RShoulderNew ;

% dbstop in TransformRShoulder
[ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_RefArmNew_RShoulderNew,C_RefArm_RArm,rotateOrder_BVH ) ;

%% Right Elbow
C_RShoulderNew_RArmNew = EulerToC_c( RightArm_new,rotateOrder_BVH,[1,1,1] );
C_Spine3New_RArmNew = CalculateC_Multi( C_Spine3New_RShoulderNew,C_RShoulderNew_RArmNew );
C_Spine2New_RArmNew = CalculateC_Multi( C_Spine2New_RShoulderNew,C_RShoulderNew_RArmNew );
C_SpineNew_RArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RArmNew );
C_HipNew_RArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RArmNew );

C_RefArm_RForeArm = C_Spine2_RForeArm ;
C_RefArmNew_RArmNew = C_Spine2New_RArmNew;

[ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_RefArm_RForeArm,C_RefArmNew_RArmNew,rotateOrder_BVH ) ;
[ RElbowYaw,RElbowRoll ] = deal( RElbow_Nao(:,1)',RElbow_Nao(:,2)' );

%% RHand
C_RArmNew_RForeArmNew = EulerToC_c( RightForeArm_new,rotateOrder_BVH,[1,1,1] );
C_Spine3New_RForeArmNew = CalculateC_Multi( C_Spine3New_RArmNew,C_RArmNew_RForeArmNew );
C_Spine2New_RForeArmNew = CalculateC_Multi( C_Spine2New_RArmNew,C_RArmNew_RForeArmNew );
C_SpineNew_RForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RForeArmNew );
C_HipNew_RForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RForeArmNew );

C_RefArm_RHand = C_Spine2_RHand;
C_RefArmNew_RForeArmNew = C_Spine2New_RForeArmNew;

[ RWristYaw,RightHand_new ] = TransformRHand( C_RefArm_RHand,C_RefArmNew_RForeArmNew,rotateOrder_BVH ) ;

%% Left Shoulder
C_Spine2New_LShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LShoulderNew );
C_SpineNew_LShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LShoulderNew );
C_HipNew_LShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LShoulderNew );

C_RefArm_LArm = C_Spine2_LArm;
C_RefArmNew_LShoulderNew = C_Spine2New_LShoulderNew;

[ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_RefArmNew_LShoulderNew,C_RefArm_LArm,rotateOrder_BVH ) ;

%% Left Elbow
C_Spine3New_LShoulderNew = EulerToC_c( LeftShoulder_new,rotateOrder_BVH,[1,1,1] );
C_LShoulderNew_LArmNew = EulerToC_c( LeftArm_new,rotateOrder_BVH,[1,1,1] );
C_Spine3New_LArmNew = CalculateC_Multi( C_Spine3New_LShoulderNew,C_LShoulderNew_LArmNew );
C_Spine2New_LArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LArmNew );
C_SpineNew_LArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LArmNew );
C_HipNew_LArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LArmNew );

C_RefArm_LForeArm = C_Spine2_LForeArm;
C_RefArmNew_LArmNew = C_Spine2New_LArmNew;

[ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_RefArm_LForeArm,C_RefArmNew_LArmNew,rotateOrder_BVH );
[ LElbowYaw,LElbowRoll ] = deal( LElbow_Nao(:,1)',LElbow_Nao(:,2)' );

%% LHand
C_LArmNew_LForeArmNew = EulerToC_c( LeftForeArm_new,rotateOrder_BVH ,[1,1,1]);
C_Spine3New_LForeArmNew = CalculateC_Multi( C_Spine3New_LArmNew,C_LArmNew_LForeArmNew );
C_Spine2New_LForeArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LForeArmNew );
C_SpineNew_LForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LForeArmNew );
C_HipNew_LForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LForeArmNew );

C_RefArm_LHand = C_Spine2_LHand ;
C_RefArmNew_LForeArmNew = C_Spine2New_LForeArmNew ;

[ LWristYaw,LeftHand_new ] = TransformLHand( C_RefArm_LHand,C_RefArmNew_LForeArmNew,rotateOrder_BVH ) ;

%% RLeg
C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2);
C_Spine2_RUpLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_RUpLeg );
% dbstop in TransformRUpLeg
[ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Spine2_RUpLeg,HipYawPitch,rotateOrder_BVH ) ;

C_Hip_RUpLegNew = EulerToC_c( RightUpLeg_new,rotateOrder_BVH ,[1,1,1]);
C_Spine2_RUpLegNew = CalculateC_Multi( C_Spine2_Hip,C_Hip_RUpLegNew );
[ RKneePitch,RightLeg_new ] = TransformRLeg( C_Spine2_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH ) ;
[ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH ) ;
%% LLeg
C_Spine2_LUpLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_LUpLeg );
[ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Spine2_LUpLeg,HipYawPitch,rotateOrder_BVH ) ;

C_Hip_LUpLegNew = EulerToC_c( LeftUpLeg_new,rotateOrder_BVH,[1,1,1] );
C_Spine2_LUpLegNew = CalculateC_Multi( C_Spine2_Hip,C_Hip_LUpLegNew );
[ LKneePitch,LeftLeg_new ] = TransformLLeg( C_Spine2_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH ) ;

[ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH ) ;
%% Get Nao


Nframes = size(HeadYaw,2);
LHand = zeros( 1,Nframes );
RHand = zeros( 1,Nframes );

% LHipRoll = LHipRoll*0 ; 
% LHipPitch = LHipPitch*0 ; 
% LKneePitch = LKneePitch*0 ; 
% RHipRoll = RHipRoll*0 ; 
% RHipPitch = RHipPitch*0; 
%            
% RKneePitch = RKneePitch*0 ; 
% LAnklePitch = LAnklePitch*0 ; 
% LAnkleRoll = LAnkleRoll*0 ; 
% RAnklePitch = RAnklePitch*0 ; 
% RAnkleRoll = RAnkleRoll*0;

NaoData = [ HeadYaw; HeadPitch; LShoulderPitch; LShoulderRoll; LElbowYaw;  LElbowRoll; ...
           RShoulderPitch; RShoulderRoll; RElbowYaw ; RElbowRoll; ...
           HipYawPitch; LHipRoll ; LHipPitch ; LKneePitch ; RHipRoll ; RHipPitch; ...
           RKneePitch ; LAnklePitch ; LAnkleRoll ; RAnklePitch ; RAnkleRoll; ...
           LWristYaw ; RWristYaw ; LHand ; RHand   ];



%% Analyse Left Arm BVH
function [ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand )
coder.inline('never');

C_Spine_Spine2 = CalculateC_Multi( C_Spine_Spine1,C_Spine1_Spine2 );
C_Spine_Spine3 = CalculateC_Multi( C_Spine_Spine2,C_Spine2_Spine3 );
C_Hip_Spine2 = CalculateC_Multi( C_Hip_Spine,C_Spine_Spine2 );
C_Hip_Spine3 = CalculateC_Multi( C_Hip_Spine,C_Spine_Spine3 );

C_Spine3_LArm = CalculateC_Multi( C_Spine3_LShoulder,C_LShoulder_LArm );
C_Spine2_LArm = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_LArm );
C_Spine_LArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_LArm );
C_Hip_LArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_LArm );

C_Spine3_LForeArm = CalculateC_Multi( C_Spine3_LArm,C_LArm_LForeArm );
C_Spine2_LForeArm = CalculateC_Multi( C_Spine2_LArm,C_LArm_LForeArm );
C_Spine_LForeArm = CalculateC_Multi( C_Spine_LArm,C_LArm_LForeArm );
C_Hip_LForeArm = CalculateC_Multi( C_Hip_LArm,C_LArm_LForeArm );

C_Spine3_LHand = CalculateC_Multi( C_Spine3_LForeArm,C_LForeArm_LHand );
C_Spine2_LHand = CalculateC_Multi( C_Spine2_LForeArm,C_LForeArm_LHand );
C_Spine_LHand = CalculateC_Multi( C_Spine_LForeArm,C_LForeArm_LHand );
C_Hip_LHand = CalculateC_Multi( C_Hip_LForeArm,C_LForeArm_LHand );

%% Analyse Right Arm BVH
function [ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine2_RArm,C_Spine2_RForeArm,C_Spine2_RHand,C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
    CalculatedRightArmRotate( C_Hip_Spine3,C_Spine2_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand )
coder.inline('never');

C_Spine3_RArm = CalculateC_Multi( C_Spine3_RShoulder,C_RShoulder_RArm );
C_Spine2_RArm = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_RArm );
C_Spine_RArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_RArm );
C_Hip_RArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_RArm );

C_Spine3_RForeArm = CalculateC_Multi( C_Spine3_RArm,C_RArm_RForeArm );
C_Spine2_RForeArm = CalculateC_Multi( C_Spine2_RArm,C_RArm_RForeArm );
C_Spine_RForeArm = CalculateC_Multi( C_Spine_RArm,C_RArm_RForeArm );
C_Hip_RForeArm = CalculateC_Multi( C_Hip_RArm,C_RArm_RForeArm );
 
C_Spine3_RHand  = CalculateC_Multi( C_Spine3_RForeArm,C_RForeArm_RHand );
C_Spine2_RHand  = CalculateC_Multi( C_Spine2_RForeArm,C_RForeArm_RHand );
C_Spine_RHand  = CalculateC_Multi( C_Spine_RForeArm,C_RForeArm_RHand );
C_Hip_RHand  = CalculateC_Multi(C_Hip_RForeArm ,C_RForeArm_RHand );

function [ C_Hip_LLeg,C_Spine2_LLeg,C_Spine3_LLeg ] = CalculatedLLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_LUpLeg,C_LUpLeg_LLeg )
coder.inline('never');

C_Hip_LLeg = CalculateC_Multi( C_Hip_LUpLeg,C_LUpLeg_LLeg );
C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2);
C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3);
C_Spine2_LLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_LLeg );
C_Spine3_LLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_LLeg );

function [ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg )
coder.inline('never');

C_Hip_RLeg = CalculateC_Multi( C_Hip_RUpLeg,C_RUpLeg_RLeg );
C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2);
C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3);
C_Spine2_RLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_RLeg );
C_Spine3_RLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_RLeg );

function C13 = CalculateC_Multi( C12,C23 )
coder.inline('never');

C13 = coder.nullcopy(zeros(size(C12)));

if size(C12,3)==1
    C13 = C23*C12 ;
    return;
end

Nframes = size( C12,1 );
C13 = zeros(Nframes,3,3);
for k=1:Nframes
    C23_k = permute( C23(k,:,:),[2,3,1] );
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C13(k,:,:) = C23_k*C12_k ;
end

function C21 = InverseC_Multi( C12 )
coder.inline('never');

if size(C12,3)==1
    C21 = C12' ;
    return;
end

Nframes = size( C12,1 );
C21 = zeros(Nframes,3,3);
for k=1:Nframes
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C21(k,:,:) = C12_k' ;
end

%% 使 Fun_Multi 能执行 Fun 的功能
function C_new = permuteC_xyz( C,order )
coder.inline('never');

if size(C,3)>1
    C_new = permute( C,order );
else  % C [3,3]
    C_new = C ;
end

%% TransformHead
% E_spine_head: euler from spine3 to head
% rotateOrder : rotate order of "E_spine_head"
function [ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_Spine_Head,C_Spine_NeckNew,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
Head_rotateOrder = 213;
C_NeckNew_Spine = InverseC_Multi( C_Spine_NeckNew );
C_NeckNew_Head = CalculateC_Multi( C_NeckNew_Spine,C_Spine_Head );
E_Spine_Head = CToEuler_c( C_NeckNew_Head,Head_rotateOrder,rotateDirection_BVH );
E_Spine_Head(:,3) = 0;
HeadYaw = E_Spine_Head(:,1)' ;
HeadPitch = E_Spine_Head(:,2)' ;

%%% get new BVH
C_Spine_HeadNew = EulerToC_c( E_Spine_Head,Head_rotateOrder ,[1,1,1] ) ;
Head_new = CToEuler_c( C_Spine_HeadNew,rotateOrder_BVH,rotateDirection_BVH );


%% RightShoulder
function [ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_Spine3_RShoulderNew,C_Spine3_RArm,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
C_RShoulderNew_Spine3 = InverseC_Multi( C_Spine3_RShoulderNew );
C_RShoulderNew_RArm = CalculateC_Multi( C_RShoulderNew_Spine3,C_Spine3_RArm );

%%% 对 C_RShoulderNew_RArm 进行零位补偿
Nframes = size(C_Spine3_RShoulderNew,1);
E_naoZero_NoitomZero_YXZ = [-pi/2,0,0];
E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 );
C_naoZero_NoitomZero_YXZ = EulerToC_c( E_naoZero_NoitomZero_YXZ,213,[1,1,1] );

C_RShoulderNew_RArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_RShoulderNew_RArm );
E_RShoulderNew_RArm_Nao = CToEuler_c( C_RShoulderNew_RArm_Nao,321,rotateDirection_BVH );
E_RShoulderNew_RArm_Nao(:,3) = 0;
C_RShoulderNew_RArm_NaoNew = EulerToC_c( E_RShoulderNew_RArm_Nao,321,[1,1,1] );
RShoulderPitch = E_RShoulderNew_RArm_Nao(:,1)';
RShoulderRoll = E_RShoulderNew_RArm_Nao(:,2)';

%%% get new BVH
% 零位补偿回去
E_NoitomZero_naoZero_ZYX = [0,pi/2,0];  % 321
E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 );
C_NoitomZero_naoZero_ZYX = EulerToC_c( E_NoitomZero_naoZero_ZYX,321 ,[1,1,1]);
C_RShoulderNew_RArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_RShoulderNew_RArm_NaoNew );

RightArm_new = CToEuler_c( C_RShoulderNew_RArm_NoitomNew,rotateOrder_BVH,rotateDirection_BVH );

%% LeftShoulder
function [ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_Spine3_LShoulderNew,C_Spine3_LArm,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
C_LShoulderNew_Spine3 = InverseC_Multi( C_Spine3_LShoulderNew );
C_LShoulderNew_LArm = CalculateC_Multi( C_LShoulderNew_Spine3,C_Spine3_LArm );
%%% 对 C_LShoulderNew_LArm 进行零位补偿
Nframes = size(C_Spine3_LShoulderNew,1);
E_naoZero_NoitomZero_YXZ = [pi/2,0,0];
E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 );
C_naoZero_NoitomZero_YXZ = EulerToC_c( E_naoZero_NoitomZero_YXZ,213,[1,1,1] );

C_LShoulderNew_LArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_LShoulderNew_LArm );
E_LShoulderNew_LArm_Nao = CToEuler_c( C_LShoulderNew_LArm_Nao,321,[-1,1,1] );
E_LShoulderNew_LArm_Nao(:,3) = 0;
C_LShoulderNew_LArm_NaoNew = EulerToC_c( E_LShoulderNew_LArm_Nao,321,[-1,1,1] );

LShoulderPitch = E_LShoulderNew_LArm_Nao(:,1)';
LShoulderRoll = E_LShoulderNew_LArm_Nao(:,2)';

%%% get new BVH
% 零位补偿回去
E_NoitomZero_naoZero_ZYX = [0,-pi/2,0];  % 'ZYX'
E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 );
C_NoitomZero_naoZero_ZYX = EulerToC_c( E_NoitomZero_naoZero_ZYX,321,[1,1,1] );
C_LShoulderNew_LArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_LShoulderNew_LArm_NaoNew );

LeftArm_new = CToEuler_c( C_LShoulderNew_LArm_NoitomNew,rotateOrder_BVH,rotateDirection_BVH );


%% REblow
function [ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_Spine_RForeArm,C_Spine_RArmNew,rotateOrder_BVH )
coder.inline('never');

C_RArmNew_Spine = InverseC_Multi( C_Spine_RArmNew );
C_RArmNew_RForeArm = CalculateC_Multi( C_RArmNew_Spine,C_Spine_RForeArm );
Nframes = size(C_RArmNew_RForeArm,1);
RElbow_Nao = zeros(Nframes,3);
for k=1:Nframes         
    [x,y,z] = deal( C_RArmNew_RForeArm(k,1,1),C_RArmNew_RForeArm(k,1,2),C_RArmNew_RForeArm(k,1,3) );
    
    %%% RElbowRoll > 0  % 人正常动作
    RElbowRoll1 = acos(x) ;
    RElbowYaw1 = atan2( -y,-z );
    RElbow_Nao1 = [ RElbowYaw1 RElbowRoll1 0 ];
%     %%% RElbowRoll < 0  % 人很少做的动作：人右手不会这么转
%     RElbowRoll2 = -acos(x) ;
%     RElbowYaw2 = atan2( y,z );
%     RElbow_Nao2 = [ RElbowYaw2 RElbowRoll2 0 ];
            
    RElbow_Nao(k,:) = RElbow_Nao1 ;
end
RElbow_Nao_degree = RElbow_Nao*180/pi ;
%%% get new BVH
RElbow_rotateOrder = 123;
RElbow_rotateDirection = [-1,1,1];
BVH_rotateDirection = [1,1,1];
RightForeArm_new = zeros(Nframes,3);
for k=1:Nframes
    RightForeArm_new(k,:) = ChangeEulerRotate_c(  RElbow_Nao( k,: ),RElbow_rotateOrder,...
        rotateOrder_BVH,RElbow_rotateDirection,BVH_rotateDirection ) ;
end

 %% LElbow
function [ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_Spine_LForeArm,C_Spine_LArmNew,rotateOrder_BVH )
coder.inline('never');

C_LArmNew_Spine = InverseC_Multi( C_Spine_LArmNew );
C_LArmNew_LForeArm = CalculateC_Multi( C_LArmNew_Spine,C_Spine_LForeArm );

Nframes = size( C_Spine_LForeArm,1 );
LElbow_Nao = zeros(Nframes,3);
for k=1:Nframes
    [x,y,z] = deal( C_LArmNew_LForeArm(k,1,1),C_LArmNew_LForeArm(k,1,2),C_LArmNew_LForeArm(k,1,3) );
    %     %%% LElbowRoll > 0  % 人很少做的动作：人左手不会这么转
    %     LElbowRoll1 = acos(x) ;
    %     LElbowYaw1 = atan2( y,-z );
    %     LElbow_Nao1 = [ LElbowYaw1 LElbowRoll1 0 ];
    %%% LElbowRoll < 0  % 人正常动作
    LElbowRoll2 = -acos(x) ;
    LElbowYaw2 = atan2( -y,z );
    LElbow_Nao2 = [ LElbowYaw2 LElbowRoll2 0 ];   

    LElbow_Nao(k,:) = LElbow_Nao2 ;
end
%%% get new BVH
LElbow_rotateOrder = 123;
LeftForeArm_new = zeros(Nframes,3);
for k=1:Nframes
    LeftForeArm_new(k,:) = ChangeEulerRotate_c(  LElbow_Nao( k,: ),LElbow_rotateOrder,rotateOrder_BVH ,[1,1,1],[1,1,1] ) ;
end

%% RHand
function [ RWristYaw,RightHand_new ] = TransformRHand( C_Spine_RHand,C_Spine_RForeArmNew,rotateOrder_BVH ) 
coder.inline('never');

C_RForeArmNew_Spine = InverseC_Multi( C_Spine_RForeArmNew );
C_RForeArmNew_RHand = CalculateC_Multi( C_RForeArmNew_Spine,C_Spine_RHand );
E_RForeArmNew_RHand = CToEuler_c( C_RForeArmNew_RHand,123,[-1,1,1] );
RWristYaw = E_RForeArmNew_RHand(:,1)';

%%% get new BVH
RHand_Nao = zeros( size(E_RForeArmNew_RHand) );
RHand_Nao(:,1) = E_RForeArmNew_RHand(:,1) ;
Nframes = size(C_Spine_RHand,1);
RHand_rotateOrder = 123;
RHand_rotateDirection = [-1,1,1];
rotateDirection_BVH = [1,1,1];
RightHand_new = zeros(Nframes,3);
for k=1:Nframes
    RightHand_new(k,:) = ChangeEulerRotate_c(  RHand_Nao( k,: ),RHand_rotateOrder,rotateOrder_BVH,RHand_rotateDirection,rotateDirection_BVH ) ;
end

%% LHand
function [ LWristYaw,LeftHand_new ] = TransformLHand( C_Spine_LHand,C_Spine_LForeArmNew,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
C_LForeArmNew_Spine = InverseC_Multi( C_Spine_LForeArmNew );
C_LForeArmNew_LHand = CalculateC_Multi( C_LForeArmNew_Spine,C_Spine_LHand );
E_LForeArmNew_LHand = CToEuler_c( C_LForeArmNew_LHand,123 ,rotateDirection_BVH);
LWristYaw = E_LForeArmNew_LHand(:,1)' ;

%%% get new BVH
LHand_Nao = zeros( size(E_LForeArmNew_LHand) );
LHand_Nao(:,1) = E_LForeArmNew_LHand(:,1) ;
Nframes = size(C_Spine_LHand,1);
LHand_rotateOrder = 123;
LeftHand_new = zeros(Nframes,3);
for k=1:Nframes
    LeftHand_new(k,:) = ChangeEulerRotate_c(  LHand_Nao( k,: ),LHand_rotateOrder,rotateOrder_BVH ,[1,1,1],[1,1,1] ) ;
end

%% RUpLeg
% HipYawPitch：左右大腿的 HipYawPitch Angle  [ 1*Nframes ]
function [ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Hip_RUpLeg,HipYawPitch,rotateOrder_BVH )
C_RUpLeg_Hip = InverseC_Multi( C_Hip_RUpLeg );
AxisR = [1,1,0] ;
AxisR = AxisR/normest(AxisR) ;
Nframes = size(C_Hip_RUpLeg,1);
C_HipYawPitch = zeros(Nframes,3,3);
for k=1:Nframes
    HipYawPitch_k = HipYawPitch(k);   
    Q_AxisR = AxisR*sin( HipYawPitch_k/2 );
    Q_HipYawPitch_k = [ cos( HipYawPitch_k/2 ) Q_AxisR ] ;
    C_HipYawPitch(k,:,:) = FQtoCnb( Q_HipYawPitch_k );
end
C_RUpLeg_SecondHip = CalculateC_Multi( C_RUpLeg_Hip,C_HipYawPitch );
C_RUpLeg_SecondHip = C_RUpLeg_Hip ;

RHipPitch = zeros(1,Nframes);
RHipRoll = zeros(1,Nframes);
for k=1:Nframes
    C_RUpLeg_SecondHip_k = permute( C_RUpLeg_SecondHip(k,:,:),[2,3,1] );
    [x,y,z] = deal( C_RUpLeg_SecondHip_k(1,2),C_RUpLeg_SecondHip_k(2,2),C_RUpLeg_SecondHip_k(3,2) );
    
    RHipPitch(k) = asin( z );
    RHipRoll(k) = atan( -x/y );    
end

E_Hip_RUpLeg_ZXY = [ RHipRoll' RHipPitch' zeros(Nframes,1) ];
C_Hip_RUpLeg_ZXY = EulerToC_c( E_Hip_RUpLeg_ZXY,312,[1,1,1] );

pos = zeros( 3,Nframes );
posNew = zeros( 3,Nframes );
pos_err = zeros( 3,Nframes );
for k=1:Nframes
    
    
    C_RUpLeg_SecondHip_k = permute( C_RUpLeg_SecondHip(k,:,:),[2 3 1] );
    C_Hip_RUpLeg_ZXY_k = permute( C_Hip_RUpLeg_ZXY(k,:,:),[2 3 1] );
    
    pos(:,k) = C_RUpLeg_SecondHip_k*[0;1;0];
    posNew(:,k) = C_Hip_RUpLeg_ZXY_k'*[0;1;0];
    pos_err(:,k) = pos(:,k)-posNew(:,k) ;
end

pos_err_sum = sum( sum( abs( pos_err ) ) );
if pos_err_sum>1e-1
    for k=1:Nframes
        RHipPitch(k) = pi-RHipPitch(k) ;
    end    
    E_Hip_RUpLeg_ZXY = [ RHipRoll' RHipPitch' zeros(Nframes,1) ];
    C_Hip_RUpLeg_ZXY = EulerToC_c( E_Hip_RUpLeg_ZXY,312,[1,1,1] );
end

%%% get new BVH
RightUpLeg_new = CToEuler_c( C_Hip_RUpLeg_ZXY,rotateOrder_BVH,[1,1,1] );

function C_new = permute_xyz( C,order )
if size(C,3)==1
    C_new = permute(C,order);
else
    C_new = C ;
end

%% LUpLeg
% HipYawPitch：左右大腿的 HipYawPitch Angle  [ Nframes*1 ]

function [ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Hip_LUpLeg,HipYawPitch,rotateOrder_BVH )
C_LUpLeg_Hip = InverseC_Multi( C_Hip_LUpLeg );
AxisL = [1,-1,0] ;
AxisL = AxisL/normest(AxisL) ;
Nframes = size(C_Hip_LUpLeg,1);
C_HipYawPitch = zeros(Nframes,3,3);
for k=1:Nframes
    HipYawPitch_k = HipYawPitch(k);   
    Q_AxisL = AxisL*sin( HipYawPitch_k/2 );
    Q_HipYawPitch_k = [ cos( HipYawPitch_k/2 ) Q_AxisL ] ;
    C_HipYawPitch(k,:,:) = FQtoCnb( Q_HipYawPitch_k );
end
C_LUpLeg_SecondHip = CalculateC_Multi( C_LUpLeg_Hip,C_HipYawPitch );
C_LUpLeg_SecondHip = C_LUpLeg_Hip ;

LHipPitch = zeros(1,Nframes);
LHipRoll = zeros(1,Nframes);
for k=1:Nframes
    C_LUpLeg_SecondHip_k = permute( C_LUpLeg_SecondHip(k,:,:),[2,3,1] );
    [x,y,z] = deal( C_LUpLeg_SecondHip_k(1,2),C_LUpLeg_SecondHip_k(2,2),C_LUpLeg_SecondHip_k(3,2) );
    
    LHipPitch(k) = asin( z );
    LHipRoll(k) = atan( -x/y );    
end

E_Hip_LUpLeg_ZXY = [ LHipRoll' LHipPitch' zeros(Nframes,1) ];
C_Hip_LUpLeg_ZXY = EulerToC_c( E_Hip_LUpLeg_ZXY,312,[1,1,1] );

pos = zeros( 3,Nframes );
posNew = zeros( 3,Nframes );
pos_err = zeros( 3,Nframes );
for k=1:Nframes
    
    
    C_LUpLeg_SecondHip_k = permute( C_LUpLeg_SecondHip(k,:,:),[2 3 1] );
    C_Hip_LUpLeg_ZXY_k = permute( C_Hip_LUpLeg_ZXY(k,:,:),[2 3 1] );
    
    pos(:,k) = C_LUpLeg_SecondHip_k*[0;1;0];
    posNew(:,k) = C_Hip_LUpLeg_ZXY_k'*[0;1;0];
    pos_err(:,k) = pos(:,k)-posNew(:,k) ;
end

pos_err_sum = sum( sum( abs( pos_err ) ) );
% if pos_err_sum>1e-1
%     for k=1:Nframes
%         LHipPitch(k) = pi-LHipPitch(k) ;
%     end    
%     E_Hip_LUpLeg_ZXY = [ LHipRoll' LHipPitch' zeros(Nframes,1) ];
%     C_Hip_LUpLeg_ZXY = EulerToC_c( E_Hip_LUpLeg_ZXY,312,[1,1,1] );
% end

%%% get new BVH
LeftUpLeg_new = CToEuler_c( C_Hip_LUpLeg_ZXY,rotateOrder_BVH,[1,1,1] );

%% RLeg
function [ RKneePitch,RightLeg_New ] = TransformRLeg( C_Hip_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH )
coder.inline('never');

C_RUpLegNew_Hip = InverseC_Multi( C_Hip_RUpLegNew );
C_RUpLegNew_RLeg = CalculateC_Multi( C_RUpLegNew_Hip,C_Hip_RLeg );
rotateDirection_BVH = [1,1,1];
E_RUpLegNew_RLeg = CToEuler_c( C_RUpLegNew_RLeg,123,rotateDirection_BVH );
RKneePitch = E_RUpLegNew_RLeg(:,1)';

% E_UpLeg_Leg = CToEuler_c( C_RUpLeg_RLeg,123 );
% RKneePitch = E_UpLeg_Leg(:,1)';

%%% get new BVH
E_RUpLegNew_RLegNew = E_RUpLegNew_RLeg ;
E_RUpLegNew_RLegNew(:,2:3) = 0;
C_RUpLegNew_RLegNew = EulerToC_c( E_RUpLegNew_RLegNew,123,[1,1,1] );
RightLeg_New = CToEuler_c( C_RUpLegNew_RLegNew,rotateOrder_BVH,rotateDirection_BVH );

%% LLeg
function [ LKneePitch,LeftLeg_New ] = TransformLLeg( C_Hip_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH )
coder.inline('never');

C_LUpLegNew_Hip = InverseC_Multi( C_Hip_LUpLegNew );
C_LUpLegNew_LLeg = CalculateC_Multi( C_LUpLegNew_Hip,C_Hip_LLeg );
rotateDirection_BVH = [1,1,1];
E_LUpLegNew_LLeg = CToEuler_c( C_LUpLegNew_LLeg,123,rotateDirection_BVH );
LKneePitch = E_LUpLegNew_LLeg(:,1)';

% figure('name','LeftLeg_Nao')
% subplot(3,1,1)
% plot(E_LUpLegNew_LLeg(:,1)*180/pi,'r'  )
% ylabel('x')
% subplot(3,1,2)
% plot(E_LUpLegNew_LLeg(:,2)*180/pi,'b'  )
% ylabel('y')
% subplot(3,1,3)
% plot(E_LUpLegNew_LLeg(:,3)*180/pi,'k'  )
% ylabel('z')

%%% get new BVH
E_LUpLegNew_LLegNew = E_LUpLegNew_LLeg ;
E_LUpLegNew_LLegNew(:,2:3) = 0;
C_LUpLegNew_LLegNew = EulerToC_c( E_LUpLegNew_LLegNew,123,[1,1,1] );
rotateDirection_BVH = [1,1,1];
LeftLeg_New = CToEuler_c( C_LUpLegNew_LLegNew,rotateOrder_BVH,rotateDirection_BVH );

%% RFoot
function [ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
E_RLeg_RFoot = CToEuler_c( C_RLeg_RFoot,132,rotateDirection_BVH );
RAnklePitch = E_RLeg_RFoot( :,1 )';
RAnkleRoll = E_RLeg_RFoot( :,2 )';

%%% get new BVH
E_RLeg_RFoot(:,3) = 0;
C_RLeg_RFoot_New = EulerToC_c( E_RLeg_RFoot,132,[1,1,1] );
RightFoot_new = CToEuler_c( C_RLeg_RFoot_New,rotateOrder_BVH,rotateDirection_BVH );

%% LFoot
function [ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH )
coder.inline('never');

rotateDirection_BVH = [1,1,1];
E_LLeg_LFoot = CToEuler_c( C_LLeg_LFoot,132,rotateDirection_BVH );
LAnklePitch = E_LLeg_LFoot( :,1 )';
LAnkleRoll = E_LLeg_LFoot( :,2 )';

%%% get new BVH
E_LLeg_LFoot(:,3) = 0;
C_RLeg_RFoot_New = EulerToC_c( E_LLeg_LFoot,132,[1,1,1] );
LeftFoot_new = CToEuler_c( C_RLeg_RFoot_New,rotateOrder_BVH,rotateDirection_BVH );


% BVHData: Nframes*(59*6)   [ 1*354 ]
function [ C_World_Hip,C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_Neck,C_Neck_Head,C_Spine3_LShoulder,...
C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand,...
C_Hip_RUpLeg,C_RUpLeg_RLeg,C_RLeg_RFoot,C_Hip_LUpLeg,C_LUpLeg_LLeg,C_LLeg_LFoot ] = BVHDataToRotate_c( BVHData,rotateOrder_BVH,rotateDirection_BVH ) %#codegen
coder.inline('never');

Nframes = size(BVHData,1);


if size(BVHData,2)==180
    step_k = 3 ;    
else
    step_k = 6 ;
end
i = 0 ;
ROOT_Hips = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightUpLeg = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightLeg = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightFoot = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftUpLeg = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftLeg = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftFoot = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Spine = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Spine1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Spine2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Spine3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Neck = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
Head = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightShoulder = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightArm = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightForeArm = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHand = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandThumb1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandThumb2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandThumb3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightInHandIndex = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandIndex1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandIndex2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandIndex3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightInHandMiddle = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandMiddle1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandMiddle2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandMiddle3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightInHandRing = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandRing1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandRing2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandRing3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightInHandPinky = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandPinky1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandPinky2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
RightHandPinky3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftShoulder = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftArm = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftForeArm = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHand = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandThumb1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandThumb2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandThumb3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftInHandIndex = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandIndex1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandIndex2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandIndex3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftInHandMiddle = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandMiddle1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandMiddle2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandMiddle3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftInHandRing = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandRing1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandRing2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandRing3 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftInHandPinky = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandPinky1 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandPinky2 = BVHData( :,4+step_k*i:6+step_k*i );  i = i+1 ;
LeftHandPinky3 = BVHData( :,4+step_k*i:6+step_k*i ); 

C_World_Hip = EulerToC_c( ROOT_Hips,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Hip_Spine = EulerToC_c( Spine,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine_Spine1 = EulerToC_c( Spine1,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine1_Spine2 = EulerToC_c( Spine2,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine2_Spine3 = EulerToC_c( Spine3,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine3_Neck = EulerToC_c( Neck,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Neck_Head = EulerToC_c( Head,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine3_LShoulder = EulerToC_c( LeftShoulder,rotateOrder_BVH,rotateDirection_BVH ) ;
C_LShoulder_LArm = EulerToC_c( LeftArm,rotateOrder_BVH,rotateDirection_BVH ) ;
C_LArm_LForeArm = EulerToC_c( LeftForeArm,rotateOrder_BVH,rotateDirection_BVH ) ;
C_LForeArm_LHand = EulerToC_c( LeftHand,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Spine3_RShoulder = EulerToC_c( RightShoulder,rotateOrder_BVH,rotateDirection_BVH ) ;
C_RShoulder_RArm = EulerToC_c( RightArm,rotateOrder_BVH,rotateDirection_BVH ) ;
C_RArm_RForeArm = EulerToC_c( RightForeArm,rotateOrder_BVH,rotateDirection_BVH ) ;
C_RForeArm_RHand = EulerToC_c( RightHand,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Hip_RUpLeg = EulerToC_c( RightUpLeg,rotateOrder_BVH,rotateDirection_BVH ) ;
C_RUpLeg_RLeg = EulerToC_c( RightLeg,rotateOrder_BVH,rotateDirection_BVH ) ;
C_RLeg_RFoot = EulerToC_c( RightFoot,rotateOrder_BVH,rotateDirection_BVH ) ;
C_Hip_LUpLeg = EulerToC_c( LeftUpLeg,rotateOrder_BVH,rotateDirection_BVH ) ;
C_LUpLeg_LLeg = EulerToC_c( LeftLeg,rotateOrder_BVH,rotateDirection_BVH ) ;
C_LLeg_LFoot = EulerToC_c( LeftFoot,rotateOrder_BVH,rotateDirection_BVH ) ;



%% xyz 2015.4.5
%  EulerToC_Multi  新版 ： 可转化为 C++

%   euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z] 
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% euler angle scope: [-pi,pi]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111


function C =  EulerToC_c( euler,rotateOrder,rotateDirection )
coder.inline('never');

Nframes = size( euler,1 );
C = zeros( Nframes,3,3 );
for k=1:Nframes
   C( k,:,: )  = OneEulerToC_c( euler(k,:),rotateOrder,rotateDirection );
end
if Nframes==1
    C = C( 1,:,: ) ;
end


function C =  OneEulerToC_c( euler,rotateOrder,rotateDirection )
coder.inline('never');

for i=1:3
    euler(i) = euler(i)*rotateDirection(i) ;
end
C = zeros(3,3);

switch rotateOrder
    case 321 
        C = RotateX(euler(3)) * RotateY(euler(2)) * RotateZ(euler(1)) ;
    case 312
        C = RotateY(euler(3)) * RotateX(euler(2)) * RotateZ(euler(1)) ;
    case 123
        C = RotateZ(euler(3)) * RotateY(euler(2)) * RotateX(euler(1)) ;
    case 132
        C = RotateY(euler(3)) * RotateZ(euler(2)) * RotateX(euler(1)) ;
    case 213
        C = RotateZ(euler(3)) * RotateX(euler(2)) * RotateY(euler(1)) ;
    case 'YZX'
        C = RotateX(euler(3)) * RotateZ(euler(2)) * RotateY(euler(1)) ;
end


%% xyz  2015.4.5

% direction cosine matrix  to Euler angle  for multi C
% C: [N*3*3]
%       euler = [eu_x,eu_y,eu_z]  ( Nframes*3 )
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% the first and third euler angles scope: [-pi,pi]
% the second euler angles scope: [-pi/2,pi/2]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111

function euler = CToEuler_c( C,rotateOrder,rotateDirection )
coder.inline('never');

if size(C,3) == 1
    euler = OneCToEuler_c( C,rotateOrder,rotateDirection );
else
    Nframes = size( C,1 );
    euler = zeros( Nframes,3 );
    for k=1:Nframes
        C_K = permute( C(k,:,:),[2,3,1] );
        euler(k,:) = OneCToEuler_c( C_K,rotateOrder,rotateDirection );
    end
end


function euler = OneCToEuler_c( C,rotateOrder,rotateDirection )
coder.inline('never');
euler1 = 0;
euler2 = 0;
euler3 = 0 ;
switch rotateOrder
    case 321 
        euler1 = atan2( C(1,2),C(1,1) ) ;
        euler2 = asin( -C(1,3) ) ;
        % euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ;
        euler3 = atan2( C(2,3),C(3,3) ) ;
    case 312
        euler1 = atan2( -C(2,1),C(2,2) ) ;
        euler2 = asin( C(2,3) ) ;
        % euler2 = atan2( C(2,3),sqrt( C(2,1)^2+C(2,2)^2 ) ) ;
        euler3 = atan2( -C(1,3),C(3,3) ) ;
    case 123
        euler1 = atan2( -C(3,2),C(3,3) ) ;
        euler2 = asin(C(3,1)) ;
        euler3 = atan2( -C(2,1),C(1,1) ) ;
    case 132
        euler1 =  atan2(C(2,3),C(2,2)) ;
        euler2 =  asin(-C(2,1)) ;
        euler3 =  atan2(C(3,1),C(1,1)) ;
    case 213
        euler1 = atan2( C(3,1),C(3,3) ) ;
        euler2 = asin( -C(3,2) ) ;
        % euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ;
        euler3 = atan2( C(1,2),C(2,2) ) ;        
    case 231 
        euler1 = atan2(-C(1,3),C(1,1))  ;
        euler2 = asin(C(1,2)) ;
        euler3 = atan2(-C(3,2),C(2,2)) ;
end
euler = [ euler1; euler2; euler3 ];

for i=1:3
    euler(i) = euler(i)*rotateDirection(i) ;
end

%% Change Euler Rotate Order

function euler2 = ChangeEulerRotate_c( euler1,rotateOrder1,rotateOrder2,rotateDirection1,rotateDirection2 )
coder.inline('never');
C = EulerToC_c( euler1,rotateOrder1,rotateDirection1 );
euler2 = CToEuler_c( C,rotateOrder2,rotateDirection2 ) ;

%% anticlockwise = positive
function Cx = RotateX( angX )
coder.inline('never');
Cx = [
    1       0           0
    0       cos(angX)   sin(angX)
    0       -sin(angX)  cos(angX)
];

%% anticlockwise rotate is positive
function Cy = RotateY( angY )
coder.inline('never');
Cy = [
    cos(angY)       0       -sin(angY)
    0               1       0
    sin(angY)       0       cos(angY)
];

%% anticlockwise = positive
function Cz = RotateZ( angZ )
coder.inline('never');
Cz = [
    cos(angZ)       sin(angZ)       0
    -sin(angZ)      cos(angZ)       0
    0               0               1
];

%% 四元数 -> 方向余弦矩阵
function Cnb = FQtoCnb(Q) %#codegen
coder.inline('never');

Q=Q/norm(Q);
q0 = Q(1) ;
q1 = Q(2) ;
q2 = Q(3) ;
q3 = Q(4) ;
Cnb=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ];