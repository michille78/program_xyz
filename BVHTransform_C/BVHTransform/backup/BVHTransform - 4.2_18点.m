%% xyz  2015.3.25
% Transform BVH data Format
% rad

function BVHTransform(  )

clc
clear all
close all
%% parameters to set
dataName = 'NaoTestBVH4';  % 无后缀 .bvh
% dataFolder = 'E:\data_xyz_noitom\BVHTransformTestData\BVHtransform';
dataFolder = 'E:\data_xyz_noitom\NaoTestBVH\NaoTestBVH4';
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
%   dbstop in readBVHData
BVHStruct = readBVHData ( dataFolder,dataName ) ;

WriteBVH( BVHStruct,resPath,[dataName,'_NoDisplacement'] );

BVHStruct = degreeBVH_To_rad( BVHStruct ) ;
BVHStruct_new = BVHStruct ;
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

[ C_Hip_Spine3,C_Spine_Spine3,C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand );

[ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
    CalculatedRightArmRotate( C_Hip_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand );


%% head
[ Head_Nao,neck_new,head_new ] = TransformHead( BVHStruct.data.Neck(:,1:3),BVHStruct.data.Head(:,1:3),rotateOrder_BVH ) ;

[ HeadYaw,HeadPitch ] = deal( Head_Nao(:,1)',Head_Nao(:,2)' );

BVHStruct_new.data.Neck = neck_new ;
BVHStruct_new.data.Head = head_new ;
%% RightArm
[ RShoulder_Nao,RightShoulder_new,RightArm_new ] = TransformRShoulder( RightShoulder,RightArm,rotateOrder_BVH ) ;
[ RShoulderPitch,RShoulderRoll ] = deal( RShoulder_Nao(:,1)',RShoulder_Nao(:,2)' );

C_Spine3_RShoulderNew = EulerToC_Multi( RightShoulder_new,rotateOrder_BVH );
C_RShoulderNew_RArmNew = EulerToC_Multi( RightArm_new,rotateOrder_BVH );
C_Spine3_RArmNew = CalculateC_Multi( C_Spine3_RShoulderNew,C_RShoulderNew_RArmNew );
C_Spine_RArmNew = CalculateC_Multi( C_Spine_Spine3,C_Spine3_RArmNew );
C_Hip_RArmNew = CalculateC_Multi( C_Hip_Spine3,C_Spine3_RArmNew );


[ RElbow_Nao,RightForeArm_new ] = TransformRElbow( C_Spine_RForeArm,C_Spine_RArmNew,rotateOrder_BVH ) ;
% [ RElbow_Nao,RightForeArm_new ] = TransformRElbow( RightShoulder,RightArm,RightForeArm,RightShoulder_new,RightArm_new,rotateOrder_BVH ) ;
[ RElbowYaw,RElbowRoll ] = deal( RElbow_Nao(:,1)',RElbow_Nao(:,2)' );

%% RHand
C_RArmNew_RForeArmNew = EulerToC_Multi( RightForeArm_new,rotateOrder_BVH );
C_Spine3_RForeArmNew = CalculateC_Multi( C_Spine3_RArmNew,C_RArmNew_RForeArmNew );
C_Spine_RForeArmNew = CalculateC_Multi( C_Spine_Spine3,C_Spine3_RForeArmNew );
C_Hip_RForeArmNew = CalculateC_Multi( C_Hip_Spine3,C_Spine3_RForeArmNew );

[ RWristYaw,RightHand_new ] = TransformRHand( C_Spine_RHand,C_Spine_RForeArmNew,rotateOrder_BVH ) ;


%% LeftArm
[ LShoulder_Nao,LeftShoulder_new,LeftArm_new ] = TransformLShoulder( LeftShoulder,LeftArm,rotateOrder_BVH );
[ LShoulderPitch,LShoulderRoll ] = deal( LShoulder_Nao(:,1)',LShoulder_Nao(:,2)' );

C_Spine3_LShoulderNew = EulerToC_Multi( LeftShoulder_new,rotateOrder_BVH );
C_LShoulderNew_LArmNew = EulerToC_Multi( LeftArm_new,rotateOrder_BVH );
C_Spine3_LArmNew = CalculateC_Multi( C_Spine3_LShoulderNew,C_LShoulderNew_LArmNew );
C_Spine_LArmNew = CalculateC_Multi( C_Spine_Spine3,C_Spine3_LArmNew );
C_Hip_LArmNew = CalculateC_Multi( C_Hip_Spine3,C_Spine3_LArmNew );

[ LElbow_Nao,LeftForeArm_new ] = TransformLElbow( C_Spine_LForeArm,C_Spine_LArmNew,rotateOrder_BVH );
% [ LElbow_Nao,LeftForeArm_new ] = Old_TransformLElbow( LeftShoulder,LeftArm,LeftForeArm,LeftShoulder_new,LeftArm_new,rotateOrder_BVH ) ;
[ LElbowYaw,LElbowRoll ] = deal( LElbow_Nao(:,1)',LElbow_Nao(:,2)' );

%% LHand
C_LArmNew_LForeArmNew = EulerToC_Multi( LeftForeArm_new,rotateOrder_BVH );
C_Spine3_LForeArmNew = CalculateC_Multi( C_Spine3_LArmNew,C_LArmNew_LForeArmNew );
C_Spine_LForeArmNew = CalculateC_Multi( C_Spine_Spine3,C_Spine3_LForeArmNew );
C_Hip_LForeArmNew = CalculateC_Multi( C_Hip_Spine3,C_Spine3_LForeArmNew );

[ LWristYaw,LeftHand_new ] = TransformLHand( C_Spine_LHand,C_Spine_LForeArmNew,rotateOrder_BVH ) ;

%% RLeg
HipYawPitch = zeros(1,size(C_Hip_Spine,1));
[ RHipRoll,RHipPitch,RightUpLeg_new ] = TransformRUpLeg( C_Hip_RUpLeg,HipYawPitch,rotateOrder_BVH ) ;
[ RKneePitch,RightLeg_new ] = TransformRLeg( C_RUpLeg_RLeg,rotateOrder_BVH ) ;
[ RAnklePitch,RAnkleRoll,RightFoot_new ] = TransformRFoot( C_RLeg_RFoot,rotateOrder_BVH ) ;
%% LLeg
[ LHipRoll,LHipPitch,LeftUpLeg_new ] = TransformLUpLeg( C_Hip_LUpLeg,HipYawPitch,rotateOrder_BVH ) ;
[ LKneePitch,LeftLeg_new ] = TransformLLeg( C_LUpLeg_LLeg,rotateOrder_BVH ) ;
[ LAnklePitch,LAnkleRoll,LeftFoot_new ] = TransformLFoot( C_LLeg_LFoot,rotateOrder_BVH ) ;
%% BVHStruct_new
BVHStruct_new.data.RightShoulder_new = RightShoulder_new ;
BVHStruct_new.data.RightArm_new = RightArm_new ;
BVHStruct_new.data.RightForeArm_new = RightForeArm_new ;
BVHStruct_new.data.RightHand = RightHand_new ;
BVHStruct_new.data.LeftShoulder = LeftShoulder_new ;
BVHStruct_new.data.LeftArm = LeftArm_new ;
BVHStruct_new.data.LeftForeArm = LeftForeArm_new ;
BVHStruct_new.data.LeftHand = LeftHand_new ;
BVHStruct_new.data.RightUpLeg = RightUpLeg_new ;
BVHStruct_new.data.LeftUpLeg = LeftUpLeg_new ;
BVHStruct_new.data.RightUpLeg = RightUpLeg_new  ;
BVHStruct_new.data.LeftLeg = LeftLeg_new ;
BVHStruct_new.data.RightFoot = RightFoot_new ;
BVHStruct_new.data.LeftFoot = LeftFoot_new ;
%% record BVHStruct_new
% dbstop in UpdateBVHStruct
BVHStruct_new = UpdateBVHStruct( BVHStruct_new ) ;
BVHStruct_new = radBVH_To_degree( BVHStruct_new ) ;
WriteBVH( BVHStruct_new,resPath,[dataName,'_NaoBVH'] );
uiopen([resPath,'\',dataName,'_NaoBVH.bvh']);
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

DrawNaoData( NaoData );
%%%  reduce the frequency
NaoDataNew = zeros(25,10);
n = 0;
for k=1:5:size(NaoData,2)
    n = n+1;
    NaoDataNew(:,n) = NaoData(:,k);
end
NaoData = NaoDataNew(:,1:n);
%%%
Nframes = size(NaoData,2);
numdata = 0:(Nframes-1);
NaoData = [ numdata;NaoData ];
for k=1:26
    NaoDataStr( k,: ) = sprintf( '%8.2f ',NaoData( k,: ) );
end

NaoName = { 'FrameNumber(100ms)','HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw'...
            'LElbowRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll'...
            'HipYawPitch','LHipRoll','LHipPitch','LKneePitch','RHipRoll','RHipPitch'...
            'RKneePitch','LAnklePitch','LAnkleRoll','RAnklePitch','RAnkleRoll'...
            'LWristYaw','RWristYaw','LHand','RHand'};
N_name = length(NaoName) ;
for k=1:N_name
    NaoNameStr(k,:) = sprintf( '%20s',NaoName{k} );
end

NaoStr = [ NaoNameStr  NaoDataStr ];

fID = fopen( [ resPath,'\',dataName,'_NaoData.txt' ],'w' );

for k=1:size(NaoStr,1)
    fprintf( fID,'%s\n',NaoStr(k,:) );
end
fprintf( fID,'%s','JOINTANGLES_END' );
fclose(fID);
uiopen([ resPath,'\',dataName,'_NaoData.txt' ]);
disp('OK')
return;
%%
% CheckSinB( Eac2,Eab3 ) ;

BVH_ab_new = BVH_ab ;
BVH_bc_new = BVH_bc ;
BVH_cd_new = BVH_cd ;    

N = size( BVH_ab,1 );
wh = waitbar( 0,'BVHTransform...' );
for k=1:N
    [ BVH_ab_new(k,4:6),BVH_bc_new(k,4:6),BVH_cd_new(k,4:6) ] = BVHTransformOne( BVH_ab(k,4:6),BVH_bc(k,4:6),BVH_cd(k,4:6),rotateOrder ) ;
    waitbar(k/N);
end
close(wh);

BVH_new = BVH ;
BVH_new( :,BVH_ab_Order(1):BVH_ab_Order(2) ) = BVH_ab_new ;
BVH_new( :,BVH_bc_Order(1):BVH_bc_Order(2) ) = BVH_bc_new ;
BVH_new( :,BVH_cd_Order(1):BVH_cd_Order(2) ) = BVH_cd_new ;

save( [resPath,'\',dataName,'_new.mat'],'BVH_new' )

% copyfile( BVHFilePath,newBVHFilePath )
fID = fopen( newBVHFilePath,'w' );
BVH_Head = readBVH_Head( BVHFilePath ) ;
fprintf( fID,'%s\n',BVH_Head );
for k=1:N
    str_k  = sprintf( '%0.2f ',BVH_new(k,:) );
    fprintf( fID,'%s \n',str_k );
end
fclose(fID)

%% Analyse Left Arm BVH
function [ C_Hip_Spine3,C_Spine_Spine3, C_Hip_LArm,C_Hip_LForeArm,C_Hip_LHand, C_Spine_LArm,C_Spine_LForeArm,C_Spine_LHand, C_Spine3_LArm,C_Spine3_LForeArm,C_Spine3_LHand ]  = ...
    CalculatedLeftArmRotate( C_Hip_Spine,C_Spine_Spine1,C_Spine1_Spine2,C_Spine2_Spine3,C_Spine3_LShoulder,C_LShoulder_LArm,C_LArm_LForeArm,C_LForeArm_LHand )

C_Spine_Spine2 = CalculateC_Multi( C_Spine_Spine1,C_Spine1_Spine2 );
C_Spine_Spine3 = CalculateC_Multi( C_Spine_Spine2,C_Spine2_Spine3 );
C_Hip_Spine3 = CalculateC_Multi( C_Hip_Spine,C_Spine_Spine3 );

C_Spine3_LArm = CalculateC_Multi( C_Spine3_LShoulder,C_LShoulder_LArm );
C_Spine_LArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_LArm );
C_Hip_LArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_LArm );

C_Spine3_LForeArm = CalculateC_Multi( C_Spine3_LArm,C_LArm_LForeArm );
C_Spine_LForeArm = CalculateC_Multi( C_Spine_LArm,C_LArm_LForeArm );
C_Hip_LForeArm = CalculateC_Multi( C_Hip_LArm,C_LArm_LForeArm );

C_Spine3_LHand = CalculateC_Multi( C_Spine3_LForeArm,C_LForeArm_LHand );
C_Spine_LHand = CalculateC_Multi( C_Spine_LForeArm,C_LForeArm_LHand );
C_Hip_LHand = CalculateC_Multi( C_Hip_LForeArm,C_LForeArm_LHand );

%% Analyse Right Arm BVH
function [ C_Hip_RArm,C_Hip_RForeArm,C_Hip_RHand, C_Spine_RArm,C_Spine_RForeArm,C_Spine_RHand, C_Spine3_RArm,C_Spine3_RForeArm,C_Spine3_RHand ]  = ...
    CalculatedRightArmRotate( C_Hip_Spine3,C_Spine_Spine3,C_Spine3_RShoulder,C_RShoulder_RArm,C_RArm_RForeArm,C_RForeArm_RHand )

C_Spine3_RArm = CalculateC_Multi( C_Spine3_RShoulder,C_RShoulder_RArm );
C_Spine_RArm = CalculateC_Multi( C_Spine_Spine3,C_Spine3_RArm );
C_Hip_RArm = CalculateC_Multi( C_Hip_Spine3,C_Spine3_RArm );

C_Spine3_RForeArm = CalculateC_Multi( C_Spine3_RArm,C_RArm_RForeArm );
C_Spine_RForeArm = CalculateC_Multi( C_Spine_RArm,C_RArm_RForeArm );
C_Hip_RForeArm = CalculateC_Multi( C_Hip_RArm,C_RArm_RForeArm );
 
C_Spine3_RHand  = CalculateC_Multi( C_Spine3_RForeArm,C_RForeArm_RHand );
C_Spine_RHand  = CalculateC_Multi( C_Spine_RForeArm,C_RForeArm_RHand );
C_Hip_RHand  = CalculateC_Multi(C_Hip_RForeArm ,C_RForeArm_RHand );


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

function  DrawNaoData( NaoData )
global resPath

figure( 'name','Head' )
plot( NaoData(1,:),'b' )
hold on
plot( NaoData(2,:),'r' )
legend( 'HeadYaw','HeadPitch' )
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])

figure( 'name','LShoulder' )
plot( NaoData(3,:),'b' )
hold on
plot( NaoData(4,:),'r' )
legend( 'LShoulderPitch','LShoulderRoll' )
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])

figure( 'name','LElbow' )
plot( NaoData(5,:),'b' )
hold on
plot( NaoData(6,:),'r' )
legend( 'LElbowYaw','LElbowRoll' )
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])

figure( 'name','RShoulder' )
plot( NaoData(7,:),'b' )
hold on
plot( NaoData(8,:),'r' )
legend( 'RShoulderPitch','RShoulderRoll' )
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])

figure( 'name','RElbow' )
plot( NaoData(9,:),'b' )
hold on
plot( NaoData(10,:),'r' )
legend( 'RElbowYaw','RElbowRoll' )
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.fig' ])
saveas(gcf,[ resPath,'\',get(gcf,'name'),'.emf' ])


disp(' OK ')

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
function [ Head_Nao,neck_new,head_new ] = TransformHead( neck,head,rotateOrder_BVH )

Head_rotateOrder = 'YXZ';
Head_abandon_num = 1 ;
N = size(neck,1);
Head_Nao = zeros(N,3);
E_spine_head = zeros(N,3);
for k=1:N
    E_spine_head( k,: ) = CalculateEuler( neck( k,: ),head( k,: ),rotateOrder_BVH ) ; % from spine3 to head
    Head_Nao( k,: ) = DirectAbandonMethod( E_spine_head( k,: ),rotateOrder_BVH,Head_rotateOrder,Head_abandon_num ) ;
end
%%% get new BVH
neck_new = zeros(N,3);
head_new = zeros(N,3);
for k=1:N
    E_spine_head_new = ChangeEulerRotate(  Head_Nao( k,: ),Head_rotateOrder,rotateOrder_BVH ) ;
    neck_new(k,:) = E_spine_head_new ;  % 全部放在 neck
end

%% RightShoulder
function [ RShoulder_Nao,RightShoulder_new,RightArm_new ] = TransformRShoulder( RightShoulder,RightArm,rotateOrder_BVH )
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

%% LeftShoulder
function [ LShoulder_Nao,LeftShoulder_new,LeftArm_new ] = TransformLShoulder( LeftShoulder,LeftArm,rotateOrder_BVH )
LShoulder_rotateOrder = 'ZYX';
LShoulder_rotateDirection = [-1,1,1];
BVH_rotateDirection = [1,1,1];
N = size(LeftShoulder,1);
% 零位补偿
E_naoZero_NoitomZero_YXZ = [pi/2,0,0];
for k=1:N
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
function [ RKneePitch,RightLeg_New ] = TransformRLeg( C_RUpLeg_RLeg,rotateOrder_BVH )

E_UpLeg_Leg = CToEuler_Multi( C_RUpLeg_RLeg,'XYZ' );
RKneePitch = E_UpLeg_Leg(:,1)';

%%% get new BVH
E_UpLeg_Leg_New = E_UpLeg_Leg ;
E_UpLeg_Leg_New(:,2:3) = 0;
C_UpLeg_Leg_New = EulerToC_Multi( E_UpLeg_Leg_New,'XYZ' );
RightLeg_New = CToEuler_Multi( C_UpLeg_Leg_New,rotateOrder_BVH );

%% LLeg
function [ LKneePitch,LeftLeg_New ] = TransformLLeg( C_LUpLeg_LLeg,rotateOrder_BVH )

E_UpLeg_Leg = CToEuler_Multi( C_LUpLeg_LLeg,'XYZ' );
LKneePitch = E_UpLeg_Leg(:,1)';

%%% get new BVH
E_UpLeg_Leg_New = E_UpLeg_Leg ;
E_UpLeg_Leg_New(:,2:3) = 0;
C_UpLeg_Leg_New = EulerToC_Multi( E_UpLeg_Leg_New,'XYZ' );
LeftLeg_New = CToEuler_Multi( C_UpLeg_Leg_New,rotateOrder_BVH );

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




















%% RElbow
function [ RElbow_Nao,RightForeArm_new ] = Old_TransformRElbow( RightShoulder,RightArm,RightForeArm,RightShoulder_new,RightArm_new,rotateOrder_BVH )
N = size(RightForeArm,1);
RElbow_Nao = zeros(N,3);
for k=1:N      
    E_Spine_ArmNew = CalculateEuler( RightShoulder_new(k,:),RightArm_new(k,:),rotateOrder_BVH );
    C_Spine_ArmNew = EulerToC( E_Spine_ArmNew,rotateOrder_BVH );
    C_ArmNew_Spine = C_Spine_ArmNew' ;
           
    E_Spine_Arm = CalculateEuler( RightShoulder(k,:),RightArm(k,:),rotateOrder_BVH );
    C_Spine_Arm = EulerToC( E_Spine_Arm,rotateOrder_BVH );
    C_Arm_Forearm = EulerToC( RightForeArm(k,:),rotateOrder_BVH );
    
    C_ArmNew_Forearm = C_Arm_Forearm * C_Spine_Arm * C_ArmNew_Spine ;   
   
    [x,y,z] = deal( C_ArmNew_Forearm(1,1),C_ArmNew_Forearm(1,2),C_ArmNew_Forearm(1,3) );
    
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
RightForeArm_new = zeros(N,3);
for k=1:N
    RightForeArm_new(k,:) = ChangeEulerRotate(  RElbow_Nao( k,: ),RElbow_rotateOrder,...
        rotateOrder_BVH,RElbow_rotateDirection,BVH_rotateDirection ) ;
end

function [ LElbow_Nao,LeftForeArm_new ] = Old_TransformLElbow( LeftShoulder,LeftArm,LeftForeArm,LeftShoulder_new,LeftArm_new,rotateOrder_BVH )
N = size(LeftForeArm,1);
LElbow_Nao = zeros(N,3);
for k=1:N
    E_Spine_ArmNew = CalculateEuler( LeftShoulder_new(k,:),LeftArm_new(k,:),rotateOrder_BVH );
    C_Spine_ArmNew = EulerToC( E_Spine_ArmNew,rotateOrder_BVH );
    C_ArmNew_Spine = C_Spine_ArmNew' ;
           
    E_Spine_Arm = CalculateEuler( LeftShoulder(k,:),LeftArm(k,:),rotateOrder_BVH );
    C_Spine_Arm = EulerToC( E_Spine_Arm,rotateOrder_BVH );
    C_Arm_Forearm = EulerToC( LeftForeArm(k,:),rotateOrder_BVH );
    
    C_ArmNew_Forearm = C_Arm_Forearm * C_Spine_Arm * C_ArmNew_Spine ;
    [x,y,z] = deal( C_ArmNew_Forearm(1,1),C_ArmNew_Forearm(1,2),C_ArmNew_Forearm(1,3) );
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
LElbow_Nao_degree = LElbow_Nao*180/pi ;
%%% get new BVH
LElbow_rotateOrder = 'XYZ';
LeftForeArm_new = zeros(N,3);
for k=1:N
    LeftForeArm_new(k,:) = ChangeEulerRotate(  LElbow_Nao( k,: ),LElbow_rotateOrder,rotateOrder_BVH ) ;
end

function [ Eab_new ] = TransformBVH_RightArm_ab( Eab,rotateOrder )
format long
Cab = EulerToC( Eab,rotateOrder );
Eab_ZYX = CToEuler( Cab,'YZX' ) ;
Eab_ZYX_new = Eab_ZYX ;
Eab_ZYX_new(3) = 0 ;

Eab_new = ChangeEulerRotate( Eab_ZYX_new,'YZX',rotateOrder ) ;

% 肘关节
function [ Ebc_new ] = TransformBVH_RightArm_bc( Eac,Eab_new,rotateOrder )
format long
Cac = EulerToC( Eac,rotateOrder );
Cab_new = EulerToC( Eab_new,rotateOrder );
Cbc_new = Cac * Cab_new' ;
X_CB_bnew = Cbc_new' * [1;0;0];
[x,y,z] = deal( X_CB_bnew(1),X_CB_bnew(2),X_CB_bnew(3) );
% Ebc_ZXY_new

% 1) beita > 0
beita1 = acos(x);
afa1 = atan2(z,y);
Ebc_XZY_new1 = [ afa1;beita1;0 ];
Ebc_new1 = ChangeEulerRotate( Ebc_XZY_new1,'XZY',rotateOrder ) ;
% 2) beita < 0
beita2 = -acos(x);
afa2 = atan2(-z,-y);
Ebc_XZY_new2 = [ afa2;beita2;0 ];
Ebc_new2 = ChangeEulerRotate( Ebc_XZY_new2,'XZY',rotateOrder ) ;
% check

Cbc_new1 = EulerToC( Ebc_new1,rotateOrder );
X_BC_b1 = Cbc_new1'*[-1;0;0] ;
X_BC_a1 = Cab_new'*X_BC_b1 ;

Cbc_new2 = EulerToC( Ebc_new2,rotateOrder );
X_BC_b2 = Cbc_new2'*[-1;0;0] ;
X_BC_a2 = Cab_new'*X_BC_b2 ;

X_BC_a0 = Cac'*[-1;0;0] ;

X_BC_a1_err = sum( abs(X_BC_a1-X_BC_a0) );
X_BC_a2_err = sum( abs(X_BC_a2-X_BC_a0) );
if X_BC_a1_err>1e-5
    errordlg('X_BC_a1_err')
end
if X_BC_a2_err>1e-5
    errordlg('X_BC_a2_err')
end

Ebc_new = Ebc_new1 ;


% 肘关节 old
function [ Ebc_new ] = TransformBVH_RightArm_bc_old( Eac,Eab_new,rotateOrder )
format long
Cac = EulerToC( Eac,rotateOrder );
Cab_new = EulerToC( Eab_new,rotateOrder );
Cbc_new = Cac * Cab_new' ;
Ebc_ZXY_new = CToEuler( Cbc_new,'XZY' ) ;
Ebc_ZXY_new(3) = 0 ;
Ebc_new = ChangeEulerRotate( Ebc_ZXY_new,'XZY',rotateOrder ) ;

% 腕关节：只有X
function [ Ecd_new ] = TransformBVH_RightArm_cd( Ead,Eac_new,rotateOrder )
format long
Cad = EulerToC( Ead,rotateOrder );
Cac_new = EulerToC( Eac_new,rotateOrder );
Ccd_new = Cad * Cac_new' ;
Ecd_ZXY_new = CToEuler( Ccd_new,'XZY' ) ;
Ecd_ZXY_new(2) = 0 ;
Ecd_ZXY_new(3) = 0 ;
Ecd_new = ChangeEulerRotate( Ecd_ZXY_new,'XZY',rotateOrder ) ;



return ;

FR = RA' * RotateX(Eac(2))' * RotateZ(Eac(3))' * RC * RB * RotateZ(Eab(3)) * RotateY(Eab(1))-eye(3) ;
FR = simplify(FR) ;
save FR FR
% temp1 = RotateX(Eac(2))' * RotateZ(Eac(3))' ;
% temp2 = RotateZ(Eab(3)) * RotateY(Eab(1)) ;
% 
% FR = RA' * temp1 * RC * RB * temp2-eye(3) ;

% FREQ = eval( 'FR==0' );

FREQ1 = eval( 'FR(1,1)==0' );
FREQ2 = eval( 'FR(2,2)==0' );
FREQ3 = eval( 'FR(3,3)==0' );
FREQ123 = [ FREQ1,FREQ2,FREQ3 ];

[A,B,C ] = solve( FREQ123 ) ;


FREQ = sym( [FR,'=zeros(3)'] );
ABC = feval( symengine,'solve',FR,'[ A,B,C ]' ) ; 


function aE = AgainstEuler( E,rotateOrder )
C =  EulerToC( E,rotateOrder ) ;
aE = CToEuler( C',rotateOrder ) ;

function FR = mufun( A,B,C )
syms A B C real
RA = RotateY( A ) ;
RB = RotateX( B ) ;
RC = RotateZ( C ) ;

FR = RA' * RotateX(Eac(2))' * RotateZ(Eac(3))' * RC * RB * RotateZ(Eab(3)) * RotateY(Eab(1))-eye(3) ;



function E13 = CalculateEuler( E12,E23,rotateOrder,rotateDirection )
if ~exist('rotateDirection','var')
   rotateDirection = [1,1,1]; 
end
C12 =  EulerToC( E12,rotateOrder,rotateDirection ) ;
C23 =  EulerToC( E23,rotateOrder,rotateDirection ) ;
C13 = C23*C12 ;
E13 = CToEuler( C13,rotateOrder,rotateDirection ) ;

function E21 = InverseEuler( E12 )
E21 = zeros(1,3);
E21(1) = -E12(3) ;
E21(2) = -E12(2) ;
E21(3) = -E12(1) ;

function BVH_Head = readBVH_Head( filePath,numberStartLine )

numberStartLine = 353 ;

fid = fopen(filePath,'r' ) ;
BVH_Head = '' ;  % N*3

for k=1:numberStartLine-1
    tline = fgetl(fid) ; 
%     lineData = textscan( tline,'%f' ) ;
    if k==1
        BVH_Head = tline ;
    else
        BVH_Head = sprintf( '%s\n%s',BVH_Head,tline );
    end
    
end

fclose(fid);


function [ Eab_new,Ebc_new,Ecd_new ] = BVHTransformOne( Eab,Ebc,Ecd,rotateOrder )

E_Head_Nao = TransformHead( neck,head,rotateOrder ) ;


Eab = Eab*pi/180 ;
Ebc = Ebc*pi/180 ;
Ecd = Ecd*pi/180 ;

Eac = CalculateEuler( Eab,Ebc,rotateOrder ) ;
Ead = CalculateEuler( Eac,Ecd,rotateOrder ) ;

[ Eab_new ] = TransformBVH_RightArm_ab( Eab,rotateOrder ) ;
[ Ebc_new ] = TransformBVH_RightArm_bc( Eac,Eab_new,rotateOrder ) ;
Eac_new = CalculateEuler( Eab_new,Ebc_new,rotateOrder ) ;
[ Ecd_new ] = TransformBVH_RightArm_cd( Ead,Eac_new,rotateOrder ) ;

Eab_new = EularScopeLimit( Eab_new,'-pi to pi') ;
Ebc_new = EularScopeLimit( Ebc_new,'-pi to pi') ;
Ecd_new = EularScopeLimit( Ecd_new,'-pi to pi') ;

Eab_new = Eab_new*180/pi ;
Ebc_new = Ebc_new*180/pi ;
Ecd_new = Ecd_new*180/pi ;


% Ead:原始的
% Eac_new:上一步算出来的新的
function [ Ead_new,Ecd_new ] = TransformBVH_RightArm_acd( Eac_new,Ead,rotateOrder )
format long
global  isCheck
Ecd_new = zeros(3,1);  % Ecd_new(1)=0


temp1 = cos(Eac_new(3))*cos(Eac_new(2)) ;
K1 = sin(Ead(2))/temp1 ;
K2 = -sin(Eac_new(2))/temp1 ;
temp2 = sqrt( (2*K1*K2)^2-4*(1+K2^2)*(K1^2-1) ) ;
CosB1 = ( -2*K1*K2+temp2 )/(1+K2^2)/2 ;
CosB2 = ( -2*K1*K2-temp2 )/(1+K2^2)/2 ;
CosB = CosB1 ;
SinB = K1+K2*CosB ;
SinA = SinB*sin(Eac_new(3))/cos(Ead(2)) ;
CosA = ( -SinB * cos(Eac_new(3)) * sin(Eac_new(2)) + CosB * cos(Eac_new(2)) )/cos(Ead(2)) ;
K3 = sin(Eac_new(3))*cos(Eac_new(2)) ;
K4 = SinB*sin(Eac_new(2))-CosB*cos(Eac_new(3))*cos(Eac_new(2)) ;
CosC = CosA/( cos(Eac_new(3)) - K3/K4*CosB*sin(Eac_new(3)) ) ;
SinC = CosC*K3/K4 ;

A = atan2(SinA,CosA);
B = asin(SinB);
C = atan2(SinC,CosC);

Ead_new = Ead ;
Ead_new(1) = A+Eac_new(1);
Ecd_new(2) = B ;
Ecd_new(3) = C+Ead(3) ;
% Ead OK    Ecd OK

%% check
if isCheck
    Edc_new = AgainstEuler( Ecd_new,rotateOrder ) ;
    Eac_check = CalculateEuler( Ead_new,Edc_new,rotateOrder ) ;
    if sum(abs( Eac_check-Eac_new ))>1e-10
        errordlg( 'TransformBVHArm_acd Eac_check error!' );
    end
    Rad = RotateZ(Ead_new(3))*RotateX(Ead_new(2))*RotateY(Ead_new(1)) ;
    Raccd = RotateZ(Ecd_new(3))*RotateX(Ecd_new(2))*RotateZ(Eac_new(3))*RotateX(Eac_new(2))*RotateY(Eac_new(1)) ;
    Raderr = Rad-Raccd ;
    if sum(sum(abs(Raderr)))>1e-10
       errordlg('TransformBVHArm Error: R12>0')  ;
    end
end

function [ Eab_new,Ebc_new,Eac_new ] = TransformBVH_RightArm_abc( Eab,Eac,rotateOrder )
format long
global isCheck

Cab = EulerToC( Eab,rotateOrder );
Eab_ZYX = CToEuler( Cab,'ZYX' ) ;

Eab_ZYX_new = Eab_ZYX ;
Eab_ZYX_new(3) = 0 ;
% Eab_ZYX_new OK
Ebc_ZYX_new = zeros(3,1);  % Ebc_new(1)=0

Eac_new = Eac ;

SinB = sin(Eac(2))/cos(Eab(3)) ;
if SinB>1
    SinB=1;
end
if SinB<-1
    SinB=-1;
end

SinA = SinB*sin(Eab(3))/cos(Eac(2)) ;
SinC = -SinA/SinB ;
CosB = sqrt(1-SinB^2);
CosA = CosB/cos(Eac(2));
CosC = sin(Eac(2))*CosA/SinB ;

A = atan2(SinA,CosA);
B = asin(SinB);
C = atan2(SinC,CosC);

Eac_new(1) = A+Eab(1) ;
Ebc_new(2) = B ;
Ebc_new(3) = Eac(3)+C ;

%% check
if isCheck
    Ecb_new = AgainstEuler( Ebc_new,rotateOrder ) ;
    Eab_new = CalculateEuler( Eac_new,Ecb_new,rotateOrder ) ;

    % 应该有：Eab_new(2) = 0 
    if abs( Eab_new(2) )<1e-10
        Eab_new(2) = 0;
    else
        errordlg('TransformBVHArm Error:Eab_new(2)>0 ')  ;
    end

    tA  = SinA^2+CosA^2 ;
    tB  = SinB^2+CosB^2 ;
    tC  = SinC^2+CosC^2 ;

    R1 = RotateX(Eac(2))*RotateY(A) ;
    R2 = RotateZ(C)*RotateX(B)*RotateZ(Eab(3)) ;
    R12 = R1-R2 ;
    if sum(sum(abs(R12)))>1e-10
       errordlg('TransformBVHArm Error: R12>0')  ;
    end
end
return;

syms A B C real
RA = RotateY( A ) ;
RB = RotateX( B ) ;
RC = RotateZ( C ) ;

% FR = RotateZ(Eac(3))*RotateX(Eac(2))*RA - RC * RB * RotateZ(Eab(3)) * RotateY(Eab(1)) ;

FR = RotateX(Eac(2))*RA - RC * RB * RotateZ(Eab(3)) ;

FREQ = eval( 'FR==0' );


FREQ11 = eval( 'FR(1,1)==0' );
FREQ12 = eval( 'FR(1,2)==0' );
FREQ13 = eval( 'FR(1,3)==0' );
FREQ21 = eval( 'FR(2,1)==0' );
FREQ22 = eval( 'FR(2,2)==0' );
FREQ23 = eval( 'FR(2,3)==0' );
FREQ33 = eval( 'FR(3,3)==0' );

FREQ123 = [ FREQ11,FREQ22,FREQ33 ];

ABC = feval( symengine,'solve',[ FREQ21,FREQ12,FREQ23 ],'[A,B,C]' ) ; 

[A,B,C ] = solve( FREQ ) ;
[A,B,C ] = solve( FREQ123 ) ;
[A1,B1,C1 ] = solve( [ FREQ11,FREQ12,FREQ13 ] ) ;

