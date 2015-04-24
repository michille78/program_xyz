%% xyz  2015.3.25
% Transform BVH data Format
% rad

function BVHTransform(  )

clc
clear all
close all
%% parameters to set
dataName = 'NaoTestBVH1';  % 无后缀 .bvh
% dataFolder = 'E:\data_xyz_noitom\BVHTransformTestData\BVHtransform';
dataFolder = 'E:\data_xyz_noitom\NaoTestBVH\NaoTestBVH1';
rotateOrder_BVH = 'YXZ' ;

global resPath isCheck
isCheck = 0 ;
resPath = [dataFolder,'\',dataName,'_result'];
if ~isdir(resPath)
    mkdir(resPath)
else
    delete([resPath,'\*'])
end
%% load data
BVHFilePath = [dataFolder,'\',dataName,'.bvh'] ;
newBVHFilePath = [dataFolder,'\new_',dataName,'.bvh'] ;
%    dbstop in readBVHData
BVHStruct = readBVHData ( dataFolder,dataName ) ;

WriteBVH( BVHStruct,resPath,[dataName,'_NoDisplacement'] );

BVHStruct = degreeBVH_To_rad( BVHStruct ) ;
BVHStruct_new = BVHStruct ;
% Test( BVHStruct );
%% calculate
%%% BVH Input data
BVHFormat = BVHStruct.BVHFormat ;
N_BVHFormat = length(BVHFormat);
for k=1:N_BVHFormat
   eval( sprintf('%s = BVHStruct.data.%s ;',BVHFormat{k},BVHFormat{k}) );   % 从前一节骨骼到所命名骨骼的旋转： Spine为从Hip到SDpine的旋转。
   Joint_M = eval( sprintf(' size(%s,2) ;',BVHFormat{k}) ) ; % 3 或 6
   eval( sprintf(' %s = %s(:,%d-2:%d) ;',BVHFormat{k},BVHFormat{k},Joint_M,Joint_M) );  % 只取后3列旋转数据
   eval( sprintf(' C_%s = EulerToC_Multi( %s,rotateOrder_BVH ); ',BVHFormat{k},BVHFormat{k}) );  % 获取所有方向余弦矩阵
end
% 重新命名以清晰旋转意义
Nframes = size(C_Spine,1);
C_Hip_Spine = C_Spine ;
C_Spine_Spine1 = C_Spine1 ;
C_Spine1_Spine2 = C_Spine2 ;
C_Spine2_Spine3 = C_Spine3 ;
C_Spine3_Neck = C_Neck ;
C_Neck_Head = C_Head ;
C_Spine3_LShoulder = C_LeftShoulder ;
C_LShoulder_LArm = C_LeftArm ;
C_LArm_LForeArm = C_LeftForeArm ;
C_LForeArm_LHand = C_LeftHand ;
C_Spine3_RShoulder = C_RightShoulder ;
C_RShoulder_RArm = C_RightArm ;
C_RArm_RForeArm = C_RightForeArm ;
C_RForeArm_RHand = C_RightHand ;
C_Hip_RUpLeg = C_RightUpLeg ;
C_RUpLeg_RLeg = C_RightLeg ;
C_RLeg_RFoot = C_RightFoot ;
C_Hip_LUpLeg = C_LeftUpLeg ;
C_LUpLeg_LLeg = C_LeftLeg ;
C_LLeg_LFoot = C_LeftFoot ;

for k=1:N_BVHFormat
    eval( sprintf(' clear C_%s ;',BVHFormat{k}) );
end

%% 所有BVH的旋转数据
[ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand ) ;

[ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine2_RArm,C_Spine2_RForeArm,C_Spine2_RHand,C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
    CalculatedRightArmRotate( C_Hip_Spine3,C_Spine2_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand ) ;

[ C_Hip_LLeg,C_Spine2_LLeg,C_Spine3_LLeg ] = CalculatedLLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_LUpLeg,C_LUpLeg_LLeg ) ;
[ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg ) ;

%% common parameters to set
% reference joint : 'Hip'  'SPine'  'Spine3'
refJoint1 = 'Spine3';
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
    C_I(k,:,:) = eye(3);
end
C_Spine3New_NeckNew = C_I;
C_Spine2New_Spine3New = C_I ;
C_SpineNew_Spine3New = C_I ;
C_HipNew_Spine3New = C_I ;
C_Spine3New_LShoulderNew = C_I ;
C_Spine3New_RShoulderNew = C_I ;
%% head
C_Spine3_Head = CalculateC_Multi( C_Spine3_Neck,C_Neck_Head );
C_Spine2_Head = CalculateC_Multi( C_Spine2_Spine3,C_Spine3_Head );
C_Spine_Head = CalculateC_Multi( C_Spine_Spine3,C_Spine3_Head );
C_Hip_Head = CalculateC_Multi( C_Hip_Spine3,C_Spine3_Head );

C_Spine2New_NeckNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_NeckNew );
C_SpineNew_NeckNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_NeckNew );
C_HipNew_NeckNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_NeckNew );

C_RefHead_Head = eval( sprintf(' C_%s_Head  ',referenceOfHead) );
C_RefHeadNew_NeckNew = eval( sprintf(' C_%sNew_NeckNew ',referenceOfHead) );

[ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_RefHead_Head,C_RefHeadNew_NeckNew,rotateOrder_BVH ) ;

BVHStruct_new.data.Neck = Neck_new ;
BVHStruct_new.data.Head = Head_new ;
%% Right Shoulder
C_Spine2New_RShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_RShoulderNew );
C_SpineNew_RShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RShoulderNew );
C_HipNew_RShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RShoulderNew );

C_RefArm_RArm = eval( sprintf('C_%s_RArm',referenceOfArm) );
C_RefArmNew_RShoulderNew = eval( sprintf('C_%sNew_RShoulderNew',referenceOfArm) );

[ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_RefArmNew_RShoulderNew,C_RefArm_RArm,rotateOrder_BVH ) ;

%% Right Elbow
C_RShoulderNew_RArmNew = EulerToC_Multi( RightArm_new,rotateOrder_BVH );
C_Spine3New_RArmNew = CalculateC_Multi( C_Spine3New_RShoulderNew,C_RShoulderNew_RArmNew );
C_Spine2New_RArmNew = CalculateC_Multi( C_Spine2New_RShoulderNew,C_RShoulderNew_RArmNew );
C_SpineNew_RArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RArmNew );
C_HipNew_RArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RArmNew );

C_RefArm_RForeArm = eval( sprintf('C_%s_RForeArm',referenceOfForeArm) );
C_RefArmNew_RArmNew = eval( sprintf('C_%sNew_RArmNew',referenceOfForeArm) );

[ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_RefArm_RForeArm,C_RefArmNew_RArmNew,rotateOrder_BVH ) ;
[ RElbowYaw,RElbowRoll ] = deal( RElbow_Nao(:,1)',RElbow_Nao(:,2)' );

%% RHand
C_RArmNew_RForeArmNew = EulerToC_Multi( RightForeArm_new,rotateOrder_BVH );
C_Spine3New_RForeArmNew = CalculateC_Multi( C_Spine3New_RArmNew,C_RArmNew_RForeArmNew );
C_Spine2New_RForeArmNew = CalculateC_Multi( C_Spine2New_RArmNew,C_RArmNew_RForeArmNew );
C_SpineNew_RForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_RForeArmNew );
C_HipNew_RForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_RForeArmNew );

C_RefArm_RHand = eval( sprintf('C_%s_RHand',referenceOfHand) );
C_RefArmNew_RForeArmNew = eval( sprintf('C_%sNew_RForeArmNew',referenceOfHand) );

[ RWristYaw,RightHand_new ] = TransformRHand( C_RefArm_RHand,C_RefArmNew_RForeArmNew,rotateOrder_BVH ) ;

%% Left Shoulder
C_Spine2New_LShoulderNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LShoulderNew );
C_SpineNew_LShoulderNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LShoulderNew );
C_HipNew_LShoulderNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LShoulderNew );

C_RefArm_LArm = eval( sprintf('C_%s_LArm',referenceOfArm) );
C_RefArmNew_LShoulderNew = eval( sprintf('C_%sNew_LShoulderNew',referenceOfArm) );

[ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_RefArmNew_LShoulderNew,C_RefArm_LArm,rotateOrder_BVH ) ;

%% Left Elbow
C_Spine3New_LShoulderNew = EulerToC_Multi( LeftShoulder_new,rotateOrder_BVH );
C_LShoulderNew_LArmNew = EulerToC_Multi( LeftArm_new,rotateOrder_BVH );
C_Spine3New_LArmNew = CalculateC_Multi( C_Spine3New_LShoulderNew,C_LShoulderNew_LArmNew );
C_Spine2New_LArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LArmNew );
C_SpineNew_LArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LArmNew );
C_HipNew_LArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LArmNew );

C_RefArm_LForeArm = eval( sprintf('C_%s_LForeArm',referenceOfForeArm) );
C_RefArmNew_LArmNew = eval( sprintf('C_%sNew_LArmNew',referenceOfForeArm) );

[ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_RefArm_LForeArm,C_RefArmNew_LArmNew,rotateOrder_BVH );
[ LElbowYaw,LElbowRoll ] = deal( LElbow_Nao(:,1)',LElbow_Nao(:,2)' );

%% LHand
C_LArmNew_LForeArmNew = EulerToC_Multi( LeftForeArm_new,rotateOrder_BVH );
C_Spine3New_LForeArmNew = CalculateC_Multi( C_Spine3New_LArmNew,C_LArmNew_LForeArmNew );
C_Spine2New_LForeArmNew = CalculateC_Multi( C_Spine2New_Spine3New,C_Spine3New_LForeArmNew );
C_SpineNew_LForeArmNew = CalculateC_Multi( C_SpineNew_Spine3New,C_Spine3New_LForeArmNew );
C_HipNew_LForeArmNew = CalculateC_Multi( C_HipNew_Spine3New,C_Spine3New_LForeArmNew );

C_RefArm_LHand = eval( sprintf('C_%s_LHand',referenceOfHand) );
C_RefArmNew_LForeArmNew = eval( sprintf('C_%sNew_LForeArmNew',referenceOfHand) );

[ LWristYaw,LeftHand_new ] = TransformLHand( C_RefArm_LHand,C_RefArmNew_LForeArmNew,rotateOrder_BVH ) ;

%% RLeg
HipYawPitch = zeros(1,size(C_Hip_Spine,1));
[ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Hip_RUpLeg,HipYawPitch,rotateOrder_BVH ) ;

C_Hip_RUpLegNew = EulerToC_Multi( RightUpLeg_new,rotateOrder_BVH );
[ RKneePitch,RightLeg_new ] = TransformRLeg( C_Hip_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH ) ;
[ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH ) ;
%% LLeg
[ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Hip_LUpLeg,HipYawPitch,rotateOrder_BVH ) ;

C_Hip_LUpLegNew = EulerToC_Multi( LeftUpLeg_new,rotateOrder_BVH );
[ LKneePitch,LeftLeg_new ] = TransformLLeg( C_Hip_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH ) ;

[ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH ) ;
%% BVHStruct_new
BVHStruct_new.data.RightShoulder = RightShoulder_new ;
BVHStruct_new.data.RightArm = RightArm_new ;
BVHStruct_new.data.RightForeArm = RightForeArm_new ;
BVHStruct_new.data.RightHand = RightHand_new ;
BVHStruct_new.data.LeftShoulder = LeftShoulder_new ;
BVHStruct_new.data.LeftArm = LeftArm_new ;
BVHStruct_new.data.LeftForeArm = LeftForeArm_new ;
BVHStruct_new.data.LeftHand = LeftHand_new ;
BVHStruct_new.data.RightUpLeg = RightUpLeg_new ;
BVHStruct_new.data.LeftUpLeg = LeftUpLeg_new ;
BVHStruct_new.data.RightLeg = RightLeg_new  ;
BVHStruct_new.data.LeftLeg = LeftLeg_new ;
BVHStruct_new.data.RightFoot = RightFoot_new ;
BVHStruct_new.data.LeftFoot = LeftFoot_new ;
BVHStruct_new.data.Spine3 = Spine3_new ;
BVHStruct_new.data.Spine2 = Spine2_new ;
BVHStruct_new.data.Spine = Spine_new ;
BVHStruct_new.data.Neck = Neck_new ;

%% record Nao
Nframes = size(HeadYaw,2);
LHand = zeros( 1,Nframes );
RHand = zeros( 1,Nframes );
NaoData = [ HeadYaw; HeadPitch; LShoulderPitch; LShoulderRoll; LElbowYaw;  LElbowRoll; ...
           RShoulderPitch; RShoulderRoll; RElbowYaw ; RElbowRoll; ...
           HipYawPitch; LHipRoll ; LHipPitch ; LKneePitch ; RHipRoll ; RHipPitch; ...
           RKneePitch ; LAnklePitch ; LAnkleRoll ; RAnklePitch ; RAnkleRoll; ...
           LWristYaw ; RWristYaw ; LHand ; RHand   ];
NaoData =  NaoData *180/pi  ;

Frame_Time_ms = BVHStruct.Frame_Time*100 ;
%%%  reduce the frequency
reduceFre = 5 ;
NaoDataNew = zeros(25,10);
n = 0;
for k=1:reduceFre:size(NaoData,2)
    n = n+1;
    NaoDataNew(:,n) = NaoData(:,k);
end
NaoData = NaoDataNew(:,1:n);

%%
% dbstop in NoitomBVH_To_NaoBVH
BVH_Nao = NoitomBVH_To_NaoBVH(  HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw,  LElbowRoll, ...
           RShoulderPitch, RShoulderRoll, RElbowYaw , RElbowRoll, ...
           HipYawPitch, LHipRoll , LHipPitch , LKneePitch , RHipRoll , RHipPitch, ...
           RKneePitch , LAnklePitch , LAnkleRoll , RAnklePitch , RAnkleRoll, ...
           LWristYaw , RWristYaw , LHand , RHand, BVHStruct_new.data.ROOT_Hips,...
           BVHStruct_new.BVHFormat,BVHStruct_new.BVHHeadStr   ) ;   %  全部是  rad
BVH_Nao = radBVH_To_degree( BVH_Nao ) ;       
WriteBVH( BVH_Nao,dataFolder,[dataName,'_NaoRotationBVH'] );
uiopen([dataFolder,'\',dataName,'_NaoRotationBVH.bvh']);
%% record BVHStruct_new
% dbstop in UpdateBVHStruct
BVHStruct_new = UpdateBVHStruct( BVHStruct_new ) ;
BVHStruct_new = radBVH_To_degree( BVHStruct_new ) ;
% dbstop in WriteBVH
WriteBVH( BVHStruct_new,dataFolder,[dataName,'_NaoBVH'] );
uiopen([dataFolder,'\',dataName,'_NaoBVH.bvh']);
%%       

Frame_Time_ms = Frame_Time_ms*reduceFre ;
Frame_Time_ms_Str = sprintf( 'FrameNumber(%0.1ms)',Frame_Time_ms );
%%%
Nframes = size(NaoData,2);
numdata = 0:(Nframes-1);
NaoData = [ numdata;NaoData ];
for k=1:26
    NaoDataStr( k,: ) = sprintf( '%8.2f ',NaoData( k,: ) );
end

NaoName = { Frame_Time_ms_Str,'HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw'...
            'LElbowRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll'...
            'HipYawPitch','LHipRoll','LHipPitch','LKneePitch','RHipRoll','RHipPitch'...
            'RKneePitch','LAnklePitch','LAnkleRoll','RAnklePitch','RAnkleRoll'...
            'LWristYaw','RWristYaw','LHand','RHand'};
N_name = length(NaoName) ;
for k=1:N_name
    NaoNameStr(k,:) = sprintf( '%20s',NaoName{k} );
end

NaoStr = [ NaoNameStr  NaoDataStr ];
%% Output
% DrawNaoData( NaoData,NaoName );

fID = fopen( [ resPath,'\',dataName,'_NaoData.txt' ],'w' );

for k=1:size(NaoStr,1)
    fprintf( fID,'%s\n',NaoStr(k,:) );
end
fprintf( fID,'%s','JOINTANGLES_END' );
fclose(fID);
uiopen([ resPath,'\',dataName,'_NaoData.txt' ]);

for k=2:N_name
   eval( sprintf( ' %s = %s*180/pi ; ' ,NaoName{k},NaoName{k}) ); 
end
disp('OK')


function Test( BVHStruct )
data = BVHStruct.data ;
LeftLeg = data.LeftLeg  ;
figure
plot( LeftLeg(:,1)*180/pi,'r' );
hold on
plot( LeftLeg(:,2)*180/pi,'b' );
plot( LeftLeg(:,3)*180/pi,'k' );
legend( 'y','x','z' );

C_LeftLeg = EulerToC_Multi( LeftLeg,'YXZ' );
LeftLeg_XYZ = CToEuler_Multi( C_LeftLeg,'XYZ' );
figure
plot( LeftLeg_XYZ(:,1)*180/pi,'r' );
hold on
plot( LeftLeg_XYZ(:,2)*180/pi,'b' );
plot( LeftLeg_XYZ(:,3)*180/pi,'k' );
legend( 'x','y','z' );

disp('')


%% Analyse Left Arm BVH
function [ C_Hip_Spine2,C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine2_LArm,C_Spine2_LForeArm,C_Spine2_LHand,C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand )

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
C_Hip_LLeg = CalculateC_Multi( C_Hip_LUpLeg,C_LUpLeg_LLeg );
C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2);
C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3);
C_Spine2_LLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_LLeg );
C_Spine3_LLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_LLeg );

function [ C_Hip_RLeg,C_Spine2_RLeg,C_Spine3_RLeg ] = CalculatedRLegRotate( C_Hip_Spine2,C_Hip_Spine3,C_Hip_RUpLeg,C_RUpLeg_RLeg )
C_Hip_RLeg = CalculateC_Multi( C_Hip_RUpLeg,C_RUpLeg_RLeg );
C_Spine2_Hip = InverseC_Multi(C_Hip_Spine2);
C_Spine3_Hip = InverseC_Multi(C_Hip_Spine3);
C_Spine2_RLeg = CalculateC_Multi( C_Spine2_Hip,C_Hip_RLeg );
C_Spine3_RLeg = CalculateC_Multi( C_Spine3_Hip,C_Hip_RLeg );

function C13 = CalculateC_Multi( C12,C23 )
Nframes = size( C12,1 );
C13 = zeros(Nframes,3,3);
for k=1:Nframes
    C23_k = permute( C23(k,:,:),[2,3,1] );
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C13(k,:,:) = C23_k*C12_k ;
end

function C21 = InverseC_Multi( C12 )
Nframes = size( C12,1 );
C21 = zeros(Nframes,3,3);
for k=1:Nframes
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C21(k,:,:) = C12_k' ;
end

%% draw

function  DrawNaoData( NaoData,NaoName )
DrawOrder = { [1,2],[3,4],[5,6],[7,8],[9,10],[11],[12,13],[14],[15,16],[17],[18,19],[20,21],[22,23],[24,25] } ;        
global resPath
NaoData(1,:) = [];
NaoName(1) = [];

figureN = length(DrawOrder);
for k=1:figureN
   data_ks =  DrawOrder{k};
   names = NaoName{ DrawOrder{k}(1) } ;   
   data_k_N = length(data_ks) ;
   for i=2:data_k_N
       names = sprintf( '%s_%s',names,NaoName{ DrawOrder{k}(i) } );
   end
   
   Maker = { 'r','b','g','k'};
   figure('name',names)
   for i=1:data_k_N
      subplot(data_k_N,1,i) ;
      plot( NaoData( DrawOrder{k}(i),: ),Maker{i} );
      ylabel( NaoName{ DrawOrder{k}(i) } );
   end
   
   saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
    saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])
end


%% chang BVH unit from degree to rad

function BVHStruct = degreeBVH_To_rad( BVHStruct )
BVHStruct.BVHData = BVHStruct.BVHData*pi/180 ;
data = BVHStruct.data ;
data_names = fieldnames(data);
N = length(data_names);
for k=1:N
    data_names_k = data_names{k} ;
    eval( sprintf(' data.%s=data.%s* pi/180 ; ',data_names_k,data_names_k) );
end
BVHStruct.data = data ;

%% chang BVH unit from rad to degree

function BVHStruct = radBVH_To_degree( BVHStruct )
BVHStruct.BVHData = BVHStruct.BVHData*180/pi ;
BVHStruct.BVHDataRotation = BVHStruct.BVHDataRotation*180/pi ;
data = BVHStruct.data ;
data_names = fieldnames(data);
N = length(data_names);
for k=1:N
    data_names_k = data_names{k} ;
    eval( sprintf(' data.%s=data.%s*180/pi ; ',data_names_k,data_names_k) );
end
BVHStruct.data = data ;
%% TransformHead
% E_spine_head: euler from spine3 to head
% rotateOrder : rotate order of "E_spine_head"
function [ HeadYaw,HeadPitch,Head_new ] = TransformHead(  C_Spine_Head,C_Spine_NeckNew,rotateOrder_BVH )

Head_rotateOrder = 'YXZ';
C_NeckNew_Spine = InverseC_Multi( C_Spine_NeckNew );
C_NeckNew_Head = CalculateC_Multi( C_NeckNew_Spine,C_Spine_Head );
E_Spine_Head = CToEuler_Multi( C_NeckNew_Head,Head_rotateOrder );
E_Spine_Head(:,3) = 0;
HeadYaw = E_Spine_Head(:,1)' ;
HeadPitch = E_Spine_Head(:,2)' ;

%%% get new BVH
C_Spine_HeadNew = EulerToC_Multi( E_Spine_Head,Head_rotateOrder ) ;
Head_new = CToEuler_Multi( C_Spine_HeadNew,rotateOrder_BVH );



function [ RShoulder_Nao,RightShoulder_new,RightArm_new ] = Old_TransformRShoulder( RightShoulder,RightArm,rotateOrder_BVH )
RShoulder_rotateOrder = 'ZYX';
N = size(RightShoulder,1);
% 零位补偿
E_naoZero_NoitomZero_YXZ = [-pi/2,0,0];
for k=1:N
    RightArm(k,:) = CalculateEuler( E_naoZero_NoitomZero_YXZ,RightArm(k,:),rotateOrder_BVH );
end

RShoulder_abandon_num = 1 ;

RShoulder_Nao = zeros(N,3);
E_spine_arm = zeros(N,3);
for k=1:N
    E_spine_arm(k,:) = CalculateEuler( RightShoulder( k,: ),RightArm( k,: ),rotateOrder_BVH ) ; % from spine3 to arm
    RShoulder_Nao( k,: ) = DirectAbandonMethod( E_spine_arm(k,:),rotateOrder_BVH,RShoulder_rotateOrder,RShoulder_abandon_num ) ;
end
RShoulder_Nao_degree = RShoulder_Nao*180/pi;
%%% get new BVH
RightShoulder_new = zeros(N,3);
RightArm_new = zeros(N,3);
% 零位补偿
RShoulder_Nao_New = RShoulder_Nao ;
E_NoitomZero_naoZero_ZYX = [0,pi/2,0];  % 'ZYX'
for k=1:N
    RShoulder_Nao_New(k,:) = CalculateEuler( E_NoitomZero_naoZero_ZYX,RShoulder_Nao_New(k,:),RShoulder_rotateOrder );
end

for k=1:N
    E_spine_arm_new = ChangeEulerRotate(  RShoulder_Nao_New( k,: ),RShoulder_rotateOrder,rotateOrder_BVH ) ;
    RightArm_new(k,:) = E_spine_arm_new ;  % 全部放在 RightArm_new
end

%% RightShoulder
function [ RShoulderPitch,RShoulderRoll,RightArm_new ] = TransformRShoulder(  C_Spine3_RShoulderNew,C_Spine3_RArm,rotateOrder_BVH )

C_RShoulderNew_Spine3 = InverseC_Multi( C_Spine3_RShoulderNew );
C_RShoulderNew_RArm = CalculateC_Multi( C_RShoulderNew_Spine3,C_Spine3_RArm );

%%% 对 C_RShoulderNew_RArm 进行零位补偿
Nframes = size(C_Spine3_RShoulderNew,1);
E_naoZero_NoitomZero_YXZ = [-pi/2,0,0];
E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 );
C_naoZero_NoitomZero_YXZ = EulerToC_Multi( E_naoZero_NoitomZero_YXZ,'YXZ' );

C_RShoulderNew_RArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_RShoulderNew_RArm );
E_RShoulderNew_RArm_Nao = CToEuler_Multi( C_RShoulderNew_RArm_Nao,'ZYX' );
E_RShoulderNew_RArm_Nao(:,3) = 0;
C_RShoulderNew_RArm_NaoNew = EulerToC_Multi( E_RShoulderNew_RArm_Nao,'ZYX' );
RShoulderPitch = E_RShoulderNew_RArm_Nao(:,1)';
RShoulderRoll = E_RShoulderNew_RArm_Nao(:,2)';

%%% get new BVH
% 零位补偿回去
E_NoitomZero_naoZero_ZYX = [0,pi/2,0];  % 'ZYX'
E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 );
C_NoitomZero_naoZero_ZYX = EulerToC_Multi( E_NoitomZero_naoZero_ZYX,'ZYX' );
C_RShoulderNew_RArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_RShoulderNew_RArm_NaoNew );

RightArm_new = CToEuler_Multi( C_RShoulderNew_RArm_NoitomNew,rotateOrder_BVH );

%% LeftShoulder
function [ LShoulderPitch,LShoulderRoll,LeftArm_new ] = TransformLShoulder(  C_Spine3_LShoulderNew,C_Spine3_LArm,rotateOrder_BVH )
C_LShoulderNew_Spine3 = InverseC_Multi( C_Spine3_LShoulderNew );
C_LShoulderNew_LArm = CalculateC_Multi( C_LShoulderNew_Spine3,C_Spine3_LArm );
%%% 对 C_LShoulderNew_LArm 进行零位补偿
Nframes = size(C_Spine3_LShoulderNew,1);
E_naoZero_NoitomZero_YXZ = [pi/2,0,0];
E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 );
C_naoZero_NoitomZero_YXZ = EulerToC_Multi( E_naoZero_NoitomZero_YXZ,'YXZ' );

C_LShoulderNew_LArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_LShoulderNew_LArm );
E_LShoulderNew_LArm_Nao = CToEuler_Multi( C_LShoulderNew_LArm_Nao,'ZYX',[-1,1,1] );
E_LShoulderNew_LArm_Nao(:,3) = 0;
C_LShoulderNew_LArm_NaoNew = EulerToC_Multi( E_LShoulderNew_LArm_Nao,'ZYX',[-1,1,1] );

LShoulderPitch = E_LShoulderNew_LArm_Nao(:,1)';
LShoulderRoll = E_LShoulderNew_LArm_Nao(:,2)';

%%% get new BVH
% 零位补偿回去
E_NoitomZero_naoZero_ZYX = [0,-pi/2,0];  % 'ZYX'
E_NoitomZero_naoZero_ZYX = repmat( E_NoitomZero_naoZero_ZYX,Nframes,1 );
C_NoitomZero_naoZero_ZYX = EulerToC_Multi( E_NoitomZero_naoZero_ZYX,'ZYX' );
C_LShoulderNew_LArm_NoitomNew = CalculateC_Multi( C_NoitomZero_naoZero_ZYX,C_LShoulderNew_LArm_NaoNew );

LeftArm_new = CToEuler_Multi( C_LShoulderNew_LArm_NoitomNew,rotateOrder_BVH );

function [ LShoulderPitch,LShoulderRoll,LeftArm_new ] = Old_TransformLShoulder(  C_Spine3_LShoulderNew,C_Spine3_LArm,rotateOrder_BVH )

C_LShoulderNew_Spine3 = InverseC_Multi( C_Spine3_LShoulderNew );
C_LShoulderNew_LArm = CalculateC_Multi( C_LShoulderNew_Spine3,C_Spine3_LArm );

E_LShoulderNew_LArm_NoConpansateZero = CToEuler_Multi( C_LShoulderNew_LArm,'ZYX',[-1,1,1] );

%%% 对 C_LShoulderNew_LArm 进行零位补偿
Nframes = size(C_Spine3_LShoulderNew,1);
E_naoZero_NoitomZero_YXZ = [pi/2,0,0];
E_naoZero_NoitomZero_YXZ = repmat( E_naoZero_NoitomZero_YXZ,Nframes,1 );
C_naoZero_NoitomZero_YXZ = EulerToC_Multi( E_naoZero_NoitomZero_YXZ,'YXZ' );
C_LShoulderNew_LArm_Nao = CalculateC_Multi( C_naoZero_NoitomZero_YXZ,C_LShoulderNew_LArm );
E_LShoulderNew_LArm_Nao = CToEuler_Multi( C_LShoulderNew_LArm_Nao,'ZYX',[-1,1,1] );

LShoulderPitch = E_LShoulderNew_LArm_Nao(:,1)';
LShoulderRoll = E_LShoulderNew_LArm_Nao(:,2)';

%%% get new BVH
E_LShoulderNew_LArmNew = E_LShoulderNew_LArm_NoConpansateZero ;
E_LShoulderNew_LArmNew(:,3) = 0;
C_LShoulderNew_LArmNew = EulerToC_Multi( E_LShoulderNew_LArmNew,'ZYX',[-1,1,1] );
LeftArm_new = CToEuler_Multi( C_LShoulderNew_LArmNew,rotateOrder_BVH );

return;

LShoulder_rotateOrder = 'ZYX';
LShoulder_rotateDirection = [-1,1,1];
BVH_rotateDirection = [1,1,1];

% LeftArm 零位补偿
Nframes = size(LeftArm,1);
E_naoZero_NoitomZero_YXZ = [pi/2,0,0];
for k=1:Nframes
    LeftArm(k,:) = CalculateEuler( E_naoZero_NoitomZero_YXZ,LeftArm(k,:),rotateOrder_BVH );
end

LShoulder_abandon_num = 1 ;

LShoulder_Nao = zeros(N,3);
E_spine_Larm = zeros(N,3);
for k=1:N
    E_spine_Larm( k,: ) = CalculateEuler( LeftShoulder( k,: ),LeftArm( k,: ),rotateOrder_BVH ) ; % from spine3 to arm
    LShoulder_Nao( k,: ) = DirectAbandonMethod( E_spine_Larm( k,: ),rotateOrder_BVH,...
    LShoulder_rotateOrder,LShoulder_abandon_num,BVH_rotateDirection,LShoulder_rotateDirection ) ;
end
LShoulder_Nao_degree = LShoulder_Nao*180/pi ;
E_spine_Larm_degree = E_spine_Larm*180/pi ;
%%% get new BVH
% 零位补偿
LShoulder_NaoNew = LShoulder_Nao ;
E_NoitomZero_naoZero_nZYX = [0,-pi/2,0]; % -Z,Y,X
for k=1:N
    LShoulder_NaoNew(k,:) = CalculateEuler( E_NoitomZero_naoZero_nZYX,LShoulder_NaoNew(k,:),LShoulder_rotateOrder,LShoulder_rotateDirection );
end

LeftShoulder_new = zeros(N,3);
LeftArm_new = zeros(N,3);
for k=1:N
    E_spine_Larm_new = ChangeEulerRotate(  LShoulder_NaoNew( k,: ),LShoulder_rotateOrder,rotateOrder_BVH,...
         LShoulder_rotateDirection,BVH_rotateDirection ) ;
    LeftArm_new(k,:) = E_spine_Larm_new ;  % 全部放在 RightArm_new
end

%% REblow
function [ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_Spine_RForeArm,C_Spine_RArmNew,rotateOrder_BVH )
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
RElbow_rotateOrder = 'XYZ';
RElbow_rotateDirection = [-1,1,1];
BVH_rotateDirection = [1,1,1];
RightForeArm_new = zeros(Nframes,3);
for k=1:Nframes
    RightForeArm_new(k,:) = ChangeEulerRotate(  RElbow_Nao( k,: ),RElbow_rotateOrder,...
        rotateOrder_BVH,RElbow_rotateDirection,BVH_rotateDirection ) ;
end

 %% LElbow
function [ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_Spine_LForeArm,C_Spine_LArmNew,rotateOrder_BVH )
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
LElbow_rotateOrder = 'XYZ';
LeftForeArm_new = zeros(Nframes,3);
for k=1:Nframes
    LeftForeArm_new(k,:) = ChangeEulerRotate(  LElbow_Nao( k,: ),LElbow_rotateOrder,rotateOrder_BVH ) ;
end

%% RHand
function [ RWristYaw,RightHand_new ] = TransformRHand( C_Spine_RHand,C_Spine_RForeArmNew,rotateOrder_BVH ) 
C_RForeArmNew_Spine = InverseC_Multi( C_Spine_RForeArmNew );
C_RForeArmNew_RHand = CalculateC_Multi( C_RForeArmNew_Spine,C_Spine_RHand );
E_RForeArmNew_RHand = CToEuler_Multi( C_RForeArmNew_RHand,'XYZ',[-1,1,1] );
RWristYaw = E_RForeArmNew_RHand(:,1)';

%%% get new BVH
RHand_Nao = zeros( size(E_RForeArmNew_RHand) );
RHand_Nao(:,1) = E_RForeArmNew_RHand(:,1) ;
Nframes = size(C_Spine_RHand,1);
RHand_rotateOrder = 'XYZ';
RHand_rotateDirection = [-1,1,1];
rotateDirection_BVH = [1,1,1];
RightHand_new = zeros(Nframes,3);
for k=1:Nframes
    RightHand_new(k,:) = ChangeEulerRotate(  RHand_Nao( k,: ),RHand_rotateOrder,rotateOrder_BVH,RHand_rotateDirection,rotateDirection_BVH ) ;
end

%% LHand
function [ LWristYaw,LeftHand_new ] = TransformLHand( C_Spine_LHand,C_Spine_LForeArmNew,rotateOrder_BVH )
C_LForeArmNew_Spine = InverseC_Multi( C_Spine_LForeArmNew );
C_LForeArmNew_LHand = CalculateC_Multi( C_LForeArmNew_Spine,C_Spine_LHand );
E_LForeArmNew_LHand = CToEuler_Multi( C_LForeArmNew_LHand,'XYZ' );
LWristYaw = E_LForeArmNew_LHand(:,1)' ;

%%% get new BVH
LHand_Nao = zeros( size(E_LForeArmNew_LHand) );
LHand_Nao(:,1) = E_LForeArmNew_LHand(:,1) ;
Nframes = size(C_Spine_LHand,1);
LHand_rotateOrder = 'XYZ';
LeftHand_new = zeros(Nframes,3);
for k=1:Nframes
    LeftHand_new(k,:) = ChangeEulerRotate(  LHand_Nao( k,: ),LHand_rotateOrder,rotateOrder_BVH ) ;
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
C_RUpLeg_Hip_New = CalculateC_Multi( C_RUpLeg_Hip,C_HipYawPitch );

RHipPitch = zeros(1,Nframes);
RHipRoll = zeros(1,Nframes);
for k=1:Nframes
    C_RUpLeg_Hip_New_k = permute( C_RUpLeg_Hip_New(k,:,:),[2,3,1] );
    [x,y,z] = deal( C_RUpLeg_Hip_New_k(1,2),C_RUpLeg_Hip_New_k(2,2),C_RUpLeg_Hip_New_k(3,2) );
    
    RHipPitch(k) = asin( z );
    RHipRoll(k) = atan( -x/y );    
end

%%% get new BVH
C_Hip_RUpLeg_New = InverseC_Multi( C_RUpLeg_Hip_New ); 
RightUpLeg_new = CToEuler_Multi( C_Hip_RUpLeg_New,rotateOrder_BVH );

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
C_LUpLeg_Hip_New = CalculateC_Multi( C_LUpLeg_Hip,C_HipYawPitch );

LHipPitch = zeros(1,Nframes);
LHipRoll = zeros(1,Nframes);
for k=1:Nframes
    C_LUpLeg_Hip_New_k = permute( C_LUpLeg_Hip_New(k,:,:),[2,3,1] );
    [x,y,z] = deal( C_LUpLeg_Hip_New_k(1,2),C_LUpLeg_Hip_New_k(2,2),C_LUpLeg_Hip_New_k(3,2) );
    
    LHipPitch(k) = asin( z );
    LHipRoll(k) = atan( -x/y );    
end

%%% get new BVH
C_Hip_LUpLeg_New = InverseC_Multi( C_LUpLeg_Hip_New ); 
LeftUpLeg_new = CToEuler_Multi( C_Hip_LUpLeg_New,rotateOrder_BVH );

%% RLeg
function [ RKneePitch,RightLeg_New ] = TransformRLeg( C_Hip_RUpLegNew,C_Hip_RLeg,rotateOrder_BVH )
C_RUpLegNew_Hip = InverseC_Multi( C_Hip_RUpLegNew );
C_RUpLegNew_RLeg = CalculateC_Multi( C_RUpLegNew_Hip,C_Hip_RLeg );

E_LUpLegNew_LLeg = CToEuler_Multi( C_RUpLegNew_RLeg,'XYZ' );
RKneePitch = E_LUpLegNew_LLeg(:,1)';

% E_UpLeg_Leg = CToEuler_Multi( C_RUpLeg_RLeg,'XYZ' );
% RKneePitch = E_UpLeg_Leg(:,1)';

%%% get new BVH
E_LUpLegNew_LLegNew = E_LUpLegNew_LLeg ;
E_LUpLegNew_LLegNew(:,2:3) = 0;
C_LUpLegNew_LLegNew = EulerToC_Multi( E_LUpLegNew_LLegNew,'XYZ' );
RightLeg_New = CToEuler_Multi( C_LUpLegNew_LLegNew,rotateOrder_BVH );

%% LLeg
function [ LKneePitch,LeftLeg_New ] = TransformLLeg( C_Hip_LUpLegNew,C_Hip_LLeg,rotateOrder_BVH )
C_LUpLegNew_Hip = InverseC_Multi( C_Hip_LUpLegNew );
C_LUpLegNew_LLeg = CalculateC_Multi( C_LUpLegNew_Hip,C_Hip_LLeg );

E_LUpLegNew_LLeg = CToEuler_Multi( C_LUpLegNew_LLeg,'XYZ' );
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
C_LUpLegNew_LLegNew = EulerToC_Multi( E_LUpLegNew_LLegNew,'XYZ' );
LeftLeg_New = CToEuler_Multi( C_LUpLegNew_LLegNew,rotateOrder_BVH );

%% RFoot
function [ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH )
E_RLeg_RFoot = CToEuler_Multi( C_RLeg_RFoot,'XZY' );
RAnklePitch = E_RLeg_RFoot( :,1 )';
RAnkleRoll = E_RLeg_RFoot( :,2 )';

%%% get new BVH
E_RLeg_RFoot(:,3) = 0;
C_RLeg_RFoot_New = EulerToC_Multi( E_RLeg_RFoot,'XZY' );
RightFoot_new = CToEuler_Multi( C_RLeg_RFoot_New,rotateOrder_BVH );

%% LFoot
function [ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH )
E_LLeg_LFoot = CToEuler_Multi( C_LLeg_LFoot,'XZY' );
LAnklePitch = E_LLeg_LFoot( :,1 )';
LAnkleRoll = E_LLeg_LFoot( :,2 )';

%%% get new BVH
E_LLeg_LFoot(:,3) = 0;
C_RLeg_RFoot_New = EulerToC_Multi( E_LLeg_LFoot,'XZY' );
LeftFoot_new = CToEuler_Multi( C_RLeg_RFoot_New,rotateOrder_BVH );

%% E_BVH： BVH 欧拉角
% rotateOrder_BVH： E_BVH 的旋转顺序
% Nao_rotateOrder：Nao关节的旋转顺序，E_Nao的最后abandon_num位是被限制的，输出时置0
% E_Nao：Nao关节的旋转顺序，E_BVH 转换为 Nao_rotateOrder 顺序，并将最后abandon_num位是被限制的，输出时置0
function E_Nao = DirectAbandonMethod( E_BVH,rotateOrder_BVH,Nao_rotateOrder,abandon_num,...
    BVH_rotateDirection,Nao_rotateDirection )

if ~exist('Nao_rotateDirection','var')
    Nao_rotateDirection = [1,1,1];
end
if ~exist('BVH_rotateDirection','var')
    BVH_rotateDirection = [1,1,1];
end
E_Nao = ChangeEulerRotate( E_BVH,rotateOrder_BVH,Nao_rotateOrder,BVH_rotateDirection,Nao_rotateDirection ) ;
for k=1:abandon_num
    E_Nao( 4-k ) = 0 ;
end

function E13 = CalculateEuler( E12,E23,rotateOrder,rotateDirection )
if ~exist('rotateDirection','var')
   rotateDirection = [1,1,1]; 
end
C12 =  EulerToC( E12,rotateOrder,rotateDirection ) ;
C23 =  EulerToC( E23,rotateOrder,rotateDirection ) ;
C13 = C23*C12 ;
E13 = CToEuler( C13,rotateOrder,rotateDirection ) ;

function E13 = CalculateEuler_Multi( E12,E23,rotateOrder,rotateDirection )
if ~exist('rotateDirection','var')
   rotateDirection = [1,1,1]; 
end
N=size( E12,1 );
E13 = zeros( size(E12) );
for k=1:N
    E13(k,:) = CalculateEuler( E12(k,:),E23(k,:),rotateOrder,rotateDirection ) ;
end

function E21 = InverseEuler( E12 )
E21 = zeros(1,3);
E21(1) = -E12(3) ;
E21(2) = -E12(2) ;
E21(3) = -E12(1) ;

function BVH_Nao = NoitomBVH_To_NaoBVH(  HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw,  LElbowRoll, ...
           RShoulderPitch, RShoulderRoll, RElbowYaw , RElbowRoll, ...
           HipYawPitch, LHipRoll , LHipPitch , LKneePitch , RHipRoll , RHipPitch, ...
           RKneePitch , LAnklePitch , LAnkleRoll , RAnklePitch , RAnkleRoll, ...
           LWristYaw , RWristYaw , LHand , RHand, ROOT_Hips, BVHFormat,BVHHeadStr   )
       
Nframes = size( HeadYaw,2 );  

LShoulderPitch = -LShoulderPitch ;
RElbowYaw = -RElbowYaw ;
RWristYaw = -RWristYaw ;

% shoulder zero reference conpansate
E_RShoulderRoll_Zero_ZYX = [ 0,pi/2,0 ];
E_RShoulderRoll_Zero_ZYX = repmat( E_RShoulderRoll_Zero_ZYX,Nframes,1 );
C_RShoulderRoll_Zero_ZYX  =EulerToC_Multi( E_RShoulderRoll_Zero_ZYX,'ZYX' );
E_RShoulder = [ RShoulderPitch',RShoulderRoll',zeros(Nframes,1) ];
C_RShoulder = EulerToC_Multi( E_RShoulder,'ZYX'  );
C_RShoulder_New = CalculateC_Multi(  C_RShoulderRoll_Zero_ZYX,C_RShoulder );
E_RShoulder_New = CToEuler_Multi( C_RShoulder_New,'ZYX' );

RightArm  = E_RShoulder_New ;
% RShoulderPitch = E_RShoulder_New(:,1)' ;
% RShoulderRoll = E_RShoulder_New(:,2)' ;

%%%
E_LShoulderRoll_Zero_ZYX = [ 0,-pi/2,0 ];
E_LShoulderRoll_Zero_ZYX = repmat( E_LShoulderRoll_Zero_ZYX,Nframes,1 );
C_LShoulderRoll_Zero_ZYX  =EulerToC_Multi( E_LShoulderRoll_Zero_ZYX,'ZYX' );
E_LShoulder = [ LShoulderPitch',LShoulderRoll',zeros(Nframes,1) ];
C_LShoulder = EulerToC_Multi( E_LShoulder,'ZYX'  );
C_LShoulder_New = CalculateC_Multi( C_LShoulderRoll_Zero_ZYX,C_LShoulder );
E_LShoulder_New = CToEuler_Multi( C_LShoulder_New,'ZYX' );

% LShoulderPitch = E_LShoulder_New(:,1)' ;
% LShoulderRoll = E_LShoulder_New(:,2)' ;

LeftArm = E_LShoulder_New ;

EZeros = zeros( Nframes,1 );
Head = [ HeadYaw' HeadPitch' EZeros ];
% LeftArm = [ LShoulderPitch'  LShoulderRoll'  EZeros  ]; % -ZY
LeftForeArm = [ LElbowYaw' LElbowRoll'  EZeros ];
% RightArm = [ RShoulderPitch', RShoulderRoll',EZeros ];
RightForeArm = [ RElbowYaw' , RElbowRoll',EZeros ]; % -XY
LeftUpLeg = [ LHipRoll' , LHipPitch' ,EZeros  ];
LeftLeg = [ LKneePitch' ,EZeros,EZeros ];
RightUpLeg = [ RHipRoll' , RHipPitch' ,EZeros ];
RightLeg = [ RKneePitch',EZeros,EZeros ];
LeftFoot = [ LAnklePitch' , LAnkleRoll' ,EZeros ];
RightFoot = [ RAnklePitch' , RAnkleRoll' ,EZeros ];
LeftHand = [ LWristYaw', EZeros,EZeros];
RightHand = [ RWristYaw',EZeros,EZeros ];  % -X

BVHdataN = length( BVHFormat );
for k=1:BVHdataN
   if exist( BVHFormat{k},'var' ) 
       eval( sprintf( 'data.%s=%s ;',BVHFormat{k},BVHFormat{k} ) );
   else
       eval( sprintf( 'data.%s=zeros(%d,3);',BVHFormat{k},Nframes ) );
   end
end

NaoRotateOrder.Head = 'YXZ';
NaoRotateOrder.LeftArm = 'ZYX';
NaoRotateOrder.LeftForeArm = 'XYZ';
NaoRotateOrder.RightArm = 'ZYX';
NaoRotateOrder.RightForeArm = 'XYZ';
NaoRotateOrder.LeftUpLeg = 'ZXY';
NaoRotateOrder.LeftLeg = 'XYZ';
NaoRotateOrder.RightUpLeg = 'ZXY';
NaoRotateOrder.RightLeg = 'XYZ';
NaoRotateOrder.LeftFoot = 'XZY';
NaoRotateOrder.RightFoot = 'XZY';
NaoRotateOrder.LeftHand = 'XZY';
NaoRotateOrder.RightHand = 'XZY';

field_NaoData = fieldnames( NaoRotateOrder );
N = length(field_NaoData);
for k=1:N
   order_k = eval( sprintf( 'NaoRotateOrder.%s ;',field_NaoData{k} ) );
   order_k_new = sprintf( 'CHANNELS 3 %srotation %srotation %srotation',order_k(1),order_k(2),order_k(3) );
   eval( sprintf( 'NaoRotateOrder.%s = order_k_new ;',field_NaoData{k} ) );
end

NaoJOINTS_OrderNum = zeros(1,N);  % Nao 关节点在 BVHHeadStr 的序号（不包括Hip）
for k=1:N
    str = sprintf('JOINT %s',field_NaoData{k} ) ;
    temp = strfind( BVHHeadStr,str );
    NaoJOINTS_OrderNum(k)= temp(1);
end
JOINTS_OrderNum = strfind( BVHHeadStr,'JOINT' );
CHANNELS_OrderNum = strfind( BVHHeadStr,'CHANNELS 3' );

[ NaoJOINTS_OrderNum_Sorted, NaoJointSort] = sort( NaoJOINTS_OrderNum ); % NaoJointSort:Nao 关节点在BVH中58个关节点的顺序（不包括Hip）


for k=1:N
    CHANNELS_k =  find(JOINTS_OrderNum==NaoJOINTS_OrderNum(k))  ;
    start_k = CHANNELS_OrderNum( CHANNELS_k );
    L = eval( sprintf('length( NaoRotateOrder.%s );',field_NaoData{k}) );
    BVHHeadStr( start_k:start_k+L-1 ) = eval(sprintf(' NaoRotateOrder.%s ',field_NaoData{k}))   ;
end

BVH_Nao.data = data ;
BVH_Nao.BVHHeadStr = BVHHeadStr ;
BVH_Nao.BVHFormat = BVHFormat ;
BVH_Nao = UpdateBVHStruct( BVH_Nao ) ;