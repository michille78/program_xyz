%% xyz 2015 5.25

%% otherMakers
% otherMakers(k).Position
% otherMakers(k).otherMakersN
%　otherMakers(k).time

% otherMakers(k).frequency
% otherMakers(k).MarkerSet

%     otherMakers(k).ContinuesFlag = NaN;
%     otherMakers(k).ContinuesLastPosition = NaN(3,1);
%     otherMakers(k).ContinuesLastTime = NaN;
%     otherMakers(k).ContinuesLastK = NaN;

%     otherMakers(k).CalculatedTime  计算次数 初始=0

%% InertialData

% InertialData.time (k)
% InertialData.HipQuaternion(k)
% InertialData.HipPosition (k)  [3*N]
% InertialData.HeadQuaternion (k)
% InertialData.HeadPosition (k)

% InertialData.BodyDirection(k)
% InertialData.frequency (k)
% 弃 % InertialData.DataStyle(k)  'GlobalBoneQuat'

%     InertialData.CalculatedTime(k)  计算次数 初始=0

%% 用 Optitrack 的 OtherMarker 补偿惯性系统
% 第一步： 补偿 Hip  或 reference 的位置

function OptitrackInertialFusion(  )
clc
clear all
close all
global dataFolder 
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head1';

%% load data
CalStruct = ReadCalData ( dataFolder,'CalData_Avatar00' ) ;

BVHStruct = readBVHData ( dataFolder,'BVHData_Avatar00' );
% HipDisplacement = GetJointDisplacement( BVHStruct,'ROOT_Hips' );
% HipDisplacement = Hip_NUE2NED( HipDisplacement ) ;
% HeadDisplacement = GetJointDisplacement( BVHStruct,'Head' );

InertialData1 = importdata( [dataFolder,'\InertialData.mat'] );
otherMakers = importdata( [dataFolder,'\otherMakers.mat'] );


% 用 CalStruct 替代 InertialData
InertialData.HipPosition = CalStruct.ROOT_Hips.X ; 
InertialData.HeadPosition = CalStruct.Head.X ; 
InertialData.HipQuaternion = CalStruct.ROOT_Hips.Q ;
InertialData.HeadQuaternion = CalStruct.Head.Q ;
InertialN = size(InertialData.HipPosition,2);
InertialData.time = (0:InertialN-1)/96;
InertialData.frequency = 96;
InertialData.BodyDirection = InertialData1.BodyDirection;

% InertialData = rmfield(InertialData,'HeadHipLength');
% InertialData = rmfield(InertialData,'DataStyle');

[ otherMakers,InertialData ] = CalInertialVisualSyc( otherMakers,InertialData ) ;
inertialN = size(InertialData.HipPosition,2);
visualN = length(otherMakers);
for k=1:visualN
    otherMakers(k).frequency = otherMakers(1).frequency;
    otherMakers(k).MarkerSet = otherMakers(1).MarkerSet;
    
    otherMakers(k).ContinuesFlag = NaN;
    otherMakers(k).ContinuesLastPosition = NaN(3,1);
    otherMakers(k).ContinuesLastTime = NaN;
    otherMakers(k).ContinuesLastK = NaN;
    
    otherMakers(k).CalculatedTime = 0;
    
    otherMakers(k).MarkerSet = 16 ; % head
    
    otherMakersNew(k) = orderfields( otherMakers(k),{'frequency','Position','otherMakersN','time','inertial_k','MarkerSet','ContinuesFlag','ContinuesLastPosition','ContinuesLastTime','ContinuesLastK','CalculatedTime'} );
end
INbuf = 11520;
VNbuf = 3600;
% [ otherMakersNew,InertialData ] = MakeSize( otherMakersNew,InertialData,INbuf,VNbuf );

InertialData = orderfields(InertialData,{ 'time','visual_k','HipQuaternion','HipPosition','HeadQuaternion','HeadPosition','BodyDirection','frequency','CalculatedTime' });

compensateRate = 1 ;
visualN  = size(otherMakersNew,2);

otherMakersOld = otherMakersNew;

VisionData_inertial_k = NaN(1,visualN);
for k = 1:visualN
	VisionData_inertial_k(k) = otherMakers(k).inertial_k;
end
    
wh = waitbar(0,'GetINSCompensateFromVNS');
step = 1;
CalEndINSave = 0;
CalStartVNSave = 0;
for k=2:step:visualN
    
    CalStartVN = CalStartVNSave+1;
    CalEndVN = k ;    
    if CalEndVN > visualN-step
       CalEndVN = visualN ;
    end
    CalStartVNSave = CalEndVN;
    
    CalStartIN = CalEndINSave+1; % 计算起点 惯性  
    CalEndIN = VisionData_inertial_k(CalEndVN);  % 计算终点 惯性
    CalEndINSave = CalEndIN;    % 记录CalEndIN  使惯性序号保持连续
    
    CalculateOrder.CalStartVN = CalStartVN;
    CalculateOrder.CalEndVN = CalEndVN;
    CalculateOrder.CalStartIN = CalStartIN;
    CalculateOrder.CalEndIN = CalEndIN;
    
    if CalculateOrder.CalEndIN <= inertialN
        %% CalculateOrder 的设置规则
        %   CalStartVN 和 CalStartIN 从1开始，且与上一时刻保持连续： CalStartIN = CalEndINSave+1; CalStartVN = CalStartVNSave+1;
        %   CalEndIN 大于或等于 CalStartVN ，  CalEndVN 大于或等于CalStartVN

%            dbstop in GetINSCompensateFromVNS
         [ AccumulateCompensate_k,otherMakersNew ] = GetINSCompensateFromVNS...
        ( InertialData,otherMakersNew,compensateRate,CalculateOrder ) ;
    end
    if mod(k,fix(visualN/10))==0
        waitbar(k/visualN);
    end
end
close(wh)

ContinuesFlag = zeros(1,visualN);
for k=1:visualN
    ContinuesFlag(k)=otherMakersNew(k).ContinuesFlag ;
end


%% 

return;
% [ InertialPositionCompensate,HipDisplacementNew ] = GetINSCompensateFromVNS_mex...
%     ( InertialData,otherMakersNew,compensateRate,CalStartVN,CalEndVN ) ;

% 更新 BVHStruct
HipDisplacementNew_NUE = Hip_NED2NUE( HipDisplacementNew ) ;

ROOT_Hips = BVHStruct.JointData.ROOT_Hips;
ROOT_Hips(:,1:3) = HipDisplacementNew_NUE;
BVHStruct.JointData.ROOT_Hips = ROOT_Hips ;
BVHStruct = UpdateBVHStruct( BVHStruct ) ;
WriteBVH( BVHStruct,dataFolder,['CompensatedBVH-',num2str(compensateRate)],1 ) ;

% HipAttitude = GetHipAttitude( InertialData.HipQuaternion )*180/pi;
% 
% figure
% subplot(3,1,1)
% plot( HipAttitude(1,:) )
% subplot(3,1,2)
% plot( HipAttitude(2,:) )
% subplot(3,1,3)
% plot( HipAttitude(3,:) )


%% 北天东 -> 北东地
% BVH  hip 和 reference 的坐标系： 北天东
% 中间数据坐标系：北东地
function HipDisplacement = Hip_NUE2NED( HipDisplacement )

C_NUE2NED = RotateX(pi/2);  % 绕 X 转90°，从NUE 到 NED

HipDisplacement = HipDisplacement';
HipDisplacement = C_NUE2NED*HipDisplacement/100 ; % 北天东 -> 北东地  转为米

%%  北东地  ->  北天东
% BVH  hip 和 reference 的坐标系： 北天东
% 中间数据坐标系：北东地
function HipDisplacement = Hip_NED2NUE( HipDisplacement )
C_NED2NUE = RotateX(-pi/2);  

HipDisplacement = C_NED2NUE*HipDisplacement*100 ; % 北天东 -> 北东地  转为cm
HipDisplacement = HipDisplacement';

%% 计算同步映射（离线）  otherMakers(k).inertial_k  和  InertialData.visuak_k 
function [ otherMakers,InertialData ] = CalInertialVisualSyc( otherMakers,InertialData )
VN = length(otherMakers);
IN = size(InertialData.HipPosition,2);

inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;

visionFre = otherMakers(1).frequency ;

otherMakersTime = zeros(1,VN);
for k = 1:VN
    otherMakersTime(k) = otherMakers(k).time;
end

for k=1:IN
    InertialData.visual_k(k) = NaN ;
end

for vision_k=1:VN
    vision_T = otherMakersTime(vision_k);
    inerital_k = fix( vision_T*inertialFre(1)+1 );
    
%     inerital_k = VisionK_to_InertialK( vision_k,visionFre,inertialFre(1),otherMakersTime,inertialTime );
    otherMakers(vision_k).inertial_k = inerital_k;
    InertialData.visual_k(inerital_k) = vision_k ;
end

last_visual_k = 0;
for k=1:IN
    if isnan(InertialData.visual_k(k))
        InertialData.visual_k(k) = last_visual_k+1 ; % 从当前往后都写 visualN
    else
        last_visual_k = InertialData.visual_k(k);
    end    
end
InertialData.CalculatedTime = zeros(1,IN);

function [ otherMakers,InertialData ] = MakeSize( otherMakers,InertialData,INbuf,VNbuf )
IN = size(InertialData.time);
for k=IN:INbuf
    InertialData.time(k) = NaN;
    InertialData.visual_k(k) = int32(0);
    InertialData.HipQuaternion(:,k) = NaN(4,1);
    InertialData.HipPosition(:,k) = NaN(3,1);
    InertialData.HeadQuaternion(:,k) = NaN(4,1);
    InertialData.HeadPosition(:,k) = NaN(3,1);
end
VN = length(otherMakers);

for k=VN:VNbuf
    otherMakers(k) = GetOneEmpty_otherMakers();    
end

function otherMakers_Empty_k = GetOneEmpty_otherMakers()
otherMakers_Empty_k.frequency = NaN;
otherMakers_Empty_k.Position = NaN(3,1);
otherMakers_Empty_k.otherMakersN = NaN;
otherMakers_Empty_k.time = NaN;
otherMakers_Empty_k.inertial_k = int32(0);
otherMakers_Empty_k.MarkerSet = NaN;
otherMakers_Empty_k.ContinuesFlag = NaN;
otherMakers_Empty_k.ContinuesLastPosition = NaN(3,1);
otherMakers_Empty_k.ContinuesLastTime = NaN;
otherMakers_Empty_k.ContinuesLastK = NaN;
otherMakers_Empty_k.CalculatedTime = 0;

