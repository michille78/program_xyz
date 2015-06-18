%% xyz 2015 5.25

%% otherMakers
% otherMakers(k).frequency [1]
% otherMakers(k).Position  [3*M]
% otherMakers(k).otherMakersN [1]
%　otherMakers(k).time [1]
%　otherMakers(k).inertial_k [1]
%　otherMakers(k).MarkerSet ""

        % 记录每个马克点的连续特性
% otherMakers(k).trackedMakerPosition  = NaN(3,1);   

% otherMakers(k).ContinuesFlag = zeros(1,M) ; % 不连续
% otherMakers(k).ContinuesLastPosition = NaN(3,M)  ;
% otherMakers(k).ContinuesLastTime = NaN[1*M] ; 
% otherMakers(k).ContinuesLastK = NaN[1*M];

%% InertialData
% InertialData.frequency (k)
% InertialData.time (k)
% InertialData.visuak_k  (k)
% InertialData.HipQuaternion(k)  [4*N]
% InertialData.HipPosition (k)  [3*N]
% InertialData.HeadQuaternion (k)  [4*N]
% InertialData.HeadPosition (k)  [3*N]
% InertialData.BodyDirection(k)  [3*1]

%% CalculateOrder 计算序号
% CalStartVN = CalculateOrder.CalStartVN ;  视觉计算起点 int32[1] 
% CalEndVN = CalculateOrder.CalEndVN ;      视觉计算终点
% CalStartIN = CalculateOrder.CalStartIN;   惯性计算起点   
% CalEndIN = CalculateOrder.CalEndIN;       惯性计算终点
%% CalculateOrder 的设置规则
%   CalStartVN 和 CalStartIN 从1开始，且与上一时刻保持连续： CalStartIN = CalEndINSave+1; CalStartVN = CalStartVNSave+1;
%   CalEndIN 大于或等于 CalStartVN ，  CalEndVN 大于或等于CalStartVN

%% 用 Optitrack 的 OtherMarker 补偿惯性系统


%% 得到惯性 Hip 位置补偿量 InertialPositionCompensate
% InertialPositionCompensate [ 3*N ]  m  NED系
% CalStartVN_in ： 视觉计算起点（）
% CalEndVN_in ：计算终点（视觉）
% IsHandledVisual IsHandledInerital 记录每个数的处理次数 ： 0表示没处理，1表示1次-正常，2次则重复处理
 
function [ InertialPositionCompensate,HipDisplacementNew,otherMakers ] = GetINSCompensateFromVNS...
    ( InertialData,otherMakers,compensateRate,CalculateOrder )

coder.extrinsic('fprintf');
coder.inline('never');
coder.extrinsic('DrawTracedINSVNS');

global    visionFre inertialFre
global InertialData_visual_k  VisionData_inertial_k CalStartIN  CalEndIN
%% 计算序号  实时与离线的切换 实现 
%   CalStartN ： 计算起始点（视觉）
%   CalEndN ： 计算结束点（视觉）
global CalStartVN CalEndVN  
visualN = length(otherMakers);
CalStartVN = CalculateOrder.CalStartVN ;
CalEndVN = CalculateOrder.CalEndVN ;
CalStartIN = CalculateOrder.CalStartIN;
CalEndIN = CalculateOrder.CalEndIN;
   
% 当视觉标定时朝北，而不对人做任何要求时，可通过 BodyDirection 将视觉的朝向调整至于人一致
% inertialTime(CalStartVN:CalEndVN ) = InertialData.time(CalStartVN:CalEndVN ) ;
if isempty(VisionData_inertial_k)
    VisionData_inertial_k = NaN(1,visualN);
end
if CalStartVN<10
   temp = 1;
else
    temp = double(CalStartVN);
end
for k = temp:CalEndVN
    VisionData_inertial_k(k) = otherMakers(k).inertial_k;
end

%% otherMakers 预处理
otherMakers = PreProcess( otherMakers,InertialData.BodyDirection );
% fprintf('PreProcess OK \n');
%       DrawAllINSVNS( otherMakers,InertialData ) ;
%    return;

%% 单马克点跟踪
%       dbstop in GetRightOtherMaker
%% load data
% inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;
InertialData_visual_k = double(InertialData.visual_k) ;
visionFre = otherMakers(1).frequency ;
MarkerSet= otherMakers(1).MarkerSet ;

switch MarkerSet   % 按中间数据骨骼编号
    case 16   % 'Head'
        InertialPosition = InertialData.HeadPosition ;
        
    case 1 %  'Hip'
        InertialPosition = InertialData.HipPosition ;
    otherwise
        InertialPosition = InertialData.HeadPosition ;
end
HipQuaternion = InertialData.HipQuaternion ;
HeadQuaternion = InertialData.HeadQuaternion ;


[ trackedMakerPosition_InertialTime,otherMakers ] =  GetRightOtherMaker( otherMakers,InertialPosition ) ;

%% 位置补偿

[ InertialPositionCompensate,HipDisplacementNew ] = VNSCompensateINS...
    ( compensateRate,trackedMakerPosition_InertialTime,InertialData.HipPosition,InertialPosition ) ;

    

%% 位移补偿
%%% Input
% trackedMakerPosition_InertialTime ： 马克点光学位置，按惯性时序存储
% HipDisplacement ： Hip在北东地下的位置 （由 BVH 得到）
% InertialPosition： 惯性系与马克点安装位置对应关节的位置（安装在头上时，InertialPosition 为惯性头的位置）
%%% Output
% InertialPositionNew ： 补偿后惯性的位置
% HipDisplacementNew： 补偿后Hip的位置
function [ InertialPositionCompensate_out,HipDisplacementNew_out ] = VNSCompensateINS...
( compensateRate,trackedMakerPosition_InertialTime,HipDisplacement,InertialPosition )
coder.inline('never');
coder.extrinsic('fprintf');
coder.extrinsic('DrawCompensate');
global CalStartIN  CalEndIN 

%% BVH 读取的个数 N_BVH 可能会比 N1 多几个
N_BVH = size(HipDisplacement,2);
N1 = size(trackedMakerPosition_InertialTime,2);
for k=N1+1:N_BVH
    trackedMakerPosition_InertialTime(:,k) = trackedMakerPosition_InertialTime(:,N1);
    InertialPosition(:,k) = InertialPosition(:,N1);
end

%% 先补偿 InertialPositionNew
persistent InertialPositionCompensate  InertialPositionNew HipDisplacementNew
if isempty(InertialPositionCompensate)
    InertialPositionCompensate = NaN(3,N_BVH); % 每一步的累积位移补偿量 记录
    InertialPositionCompensate(:,1)=zeros(3,1);
    InertialPositionNew = NaN(3,N_BVH);
    InertialPositionNew(3,:) = InertialPosition(3,:); % 高度不补偿
    InertialPositionNew(1:2,1) = InertialPosition(1:2,1) ;  % 起始点选择惯性
    
    HipDisplacementNew = NaN(3,N_BVH); 
    HipDisplacementNew(3,:) = HipDisplacement(3,:);
    HipDisplacementNew(:,1) = HipDisplacement(:,1);
end
persistent  InertialErr StepCompensate
if isempty(StepCompensate)
    InertialErr = zeros(2,N_BVH);       % 实时记录补偿后的误差
    StepCompensate = zeros(2,N_BVH);  % 单步补偿量
end

for k = max(2,CalStartIN) : CalEndIN 
    % 先用纯惯性递推
    InertialPositionNew(1:2,k) = InertialPositionNew(1:2,k-1)+( InertialPosition(1:2,k)-InertialPosition(1:2,k-1) );
    % 先求纯惯性为误差
    if ~isnan(trackedMakerPosition_InertialTime(1,k))
        InertialErr(:,k) = trackedMakerPosition_InertialTime(1:2,k) - InertialPositionNew(1:2,k) ;
        % 补偿误差
        InertialPositionNew(1:2,k) = InertialPositionNew(1:2,k) + InertialErr(:,k)*compensateRate ; 
        StepCompensate(:,k) = InertialErr(:,k)*compensateRate ;
    end
    InertialPositionCompensate(1:2,k) = InertialPositionNew(1:2,k) - InertialPosition(1:2,k) ;  % 累积位移补偿量
end

%% 通过 InertialPositionNew 计算 HipDisplacementNew
% 将Head位置传到Hip，：保持头和head的相对位移

for k=CalStartIN:CalEndIN    
    HipDisplacementNew( 1:2,k ) = HipDisplacement(1:2,k) + InertialPositionCompensate(1:2,k) ;
    HipDisplacementNew( 1:3,k ) = HipDisplacement( 1:3,k );
end

InertialPositionCompensate_out = InertialPositionCompensate ;
HipDisplacementNew_out = HipDisplacementNew;
if CalEndIN >= size(trackedMakerPosition_InertialTime,2)-10
    DrawCompensate( InertialErr,StepCompensate,trackedMakerPosition_InertialTime,...
        InertialPosition,InertialPositionNew,InertialPositionCompensate,compensateRate,N_BVH ) ;
end

%% 视觉位置预处理
% 1）将视觉数据  从视觉世界坐标系 转到 北东地坐标系
function otherMakers = PreProcess( otherMakers,BodyDirection  )
coder.inline('never');
global CalStartVN CalEndVN 
persistent VisualP_t0  % 0时刻的视觉位置
if isempty(VisualP_t0)
   VisualP_t0 = zeros(3,1); 
end
%% 先转到北东地的同名坐标系
Cv_r1 = [ 0 0 1; -1 0 0; 0 -1 0 ];
% dbstop in BodyDirection2Cr_r1
Cr_r1 = BodyDirection2Cr_r1( BodyDirection );   %   （法1）    要求人朝视觉标定标定的世界坐标系进行对准
Cvr = Cr_r1' * Cv_r1 ;
Cvr = Cv_r1 ;         %   （法2）    要求视觉世界坐标系的Z轴朝向正北

%% 先转到 惯性系 的北东地，并以视觉的初始点为原点（不处理高度）

Position_1 = otherMakers(1).Position(:,1) ;
Position_1(2) = 0 ;      % 不处理高度方向
if CalStartVN==1
    VisualP_t0 = Position_1;
end
for k=CalStartVN:CalEndVN
    Position_k = otherMakers(k).Position ;
    if ~isempty(Position_k)
%         m = size(Position_k,2);
%         position_offest = repmat(Position_1,1,m);     
%         Position_k_new = Cvr*(Position_k-position_offest) ;  % 从视觉世界坐标系 转到 北东地坐标系
    
        

        Position_k_new = Cvr*Position_k;
        otherMakers(k).Position = Position_k_new; 
    else
        otherMakers(k).Position = NaN(3,1); 
    end
    otherMakers(k).CalculatedTime = otherMakers(k).CalculatedTime+1 ;
    if coder.target('MATLAB') && otherMakers(k).CalculatedTime~=1
       fpritnf( 'otherMakers(%0.0f).CalculatedTime = %0.0f\n',k,otherMakers(k).CalculatedTime ); 
    end
end


%% 根据 BodyDirection 求姿态
% 符号有待验证！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
%   BodyDirection  人朝北为 [1 0 0]
function Cr_r1 = BodyDirection2Cr_r1( BodyDirection )
coder.inline('never');

V = [1 0 0];   % 正北
thita = acos( V*BodyDirection / normest(BodyDirection)   );
% 通过叉乘可判断角度方向
temp = cross(V,BodyDirection) ;  % 从 V 到 BodyDirection 逆时针180°以内时，temp朝上，temp(3)<0
if temp(3)>0
        % 从 V 到 BodyDirection 逆时针转动超过180°
      thita = -thita ;
end
    
Cr_r1 = RotateZ( thita ) ;


%% InertialData  和 otherMakers 置 NaN
%% 用于给 C++ 自动生成 初始化 函数

function [InertialData,otherMakers] = Set_InertialData_otherMakers_NaN( InertialData,otherMakers,I_N,V_N )
coder.inline('never');

InertialData.frequency = 96;
InertialData.time  = NaN;
InertialData.HipQuaternion  =  NaN(4,I_N);
InertialData.HipPosition =  NaN(3,I_N);
InertialData.HeadQuaternion  =  NaN(4,I_N);
InertialData.HeadPosition =  NaN(3,I_N);
InertialData.BodyDirection =  NaN(3,1);
% InertialData.DataStyle =  'GlobalBoneQuat';

for k=1:V_N
    otherMakers(k).frequency = 30;
    otherMakers(k).Position = NaN(3,1);
    otherMakers(k).otherMakersN = int32(NaN);
    otherMakers(k).time = NaN;
    otherMakers(k).MarkerSet = 16 ;  % 'head';
    otherMakers(k).trackedMakerPosition  = NaN(3,1);  
    otherMakers(k).ContinuesFlag = NaN;
    otherMakers(k).ContinuesLastPosition = NaN(3,1);
    otherMakers(k).ContinuesLastTime = NaN;
    otherMakers(k).ContinuesLastK = NaN;
end
