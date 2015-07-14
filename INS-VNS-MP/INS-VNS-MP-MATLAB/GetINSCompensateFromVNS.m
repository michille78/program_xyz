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
% otherMakers(k).InitialJointK = NaN(1,MaxotherMakersN_k);  % 对应惯性节点序号

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


%% 得到惯性 Hip 位置补偿量 SingleFrameCompensate
% SingleFrameCompensate [ 3*N ]  m  NED系
% CalStartVN_in ： 视觉计算起点（）
% CalEndVN_in ：计算终点（视觉）
% IsHandledVisual IsHandledInerital 记录每个数的处理次数 ： 0表示没处理，1表示1次-正常，2次则重复处理
 
function [ AccumulateCompensate_k_Out,otherMakers ] = GetINSCompensateFromVNS...
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
ineritalN = size(InertialData.HeadPosition,2);
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
otherMakers = PreProcess( otherMakers );
% fprintf('PreProcess OK \n');
%       DrawAllINSVNS( otherMakers,InertialData ) ;
%    return;

%% 单马克点跟踪
%       dbstop in GetRightOtherMaker
%% load data
% inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;
if isempty(InertialData_visual_k)
    InertialData_visual_k = double(InertialData.visual_k) ;
else
    InertialData_visual_k(CalStartIN:CalEndIN) = double(InertialData.visual_k(CalStartIN:CalEndIN)) ;
end

visionFre = otherMakers(1).frequency ;
MarkerSet= otherMakers(1).MarkerSet ;

persistent InertialPosition
if isempty(InertialPosition)
    InertialPosition = NaN(3,ineritalN);
end
switch MarkerSet   % 按中间数据骨骼编号
    case 16   % 'Head'
        InertialPosition(:,CalStartIN:CalEndIN) = InertialData.HeadPosition(:,CalStartIN:CalEndIN) ;
        
    case 1 %  'Hip'
        InertialPosition(:,CalStartIN:CalEndIN) = InertialData.HipPosition(:,CalStartIN:CalEndIN) ;
    otherwise
        InertialPosition(:,CalStartIN:CalEndIN) = InertialData.HeadPosition(:,CalStartIN:CalEndIN) ;
end
% HipQuaternion = InertialData.HipQuaternion ;
% HeadQuaternion = InertialData.HeadQuaternion ;


[ trackedMakerPosition_InertialTime,otherMakers ] =  GetRightOtherMaker( otherMakers,InertialPosition ) ;

%% 位置补偿

% [ InertialPositionCompensate,HipDisplacementNew ] = VNSCompensateINS...
%     ( compensateRate,trackedMakerPosition_InertialTime,InertialData.HipPosition,InertialPosition ) ;

CalInertialN = CalEndIN - CalStartIN;

persistent InertialPositionNew_k AccumulateCompensate_k
if isempty(AccumulateCompensate_k)
   AccumulateCompensate_k = zeros(3,1); 
   InertialPositionNew_k = NaN(3,1); % 在 k-1 时刻补偿后结果基础上，纯惯性递推一个时刻的 K 时刻结果
end

persistent  InertialPositionNew AccumulateCompensate
if isempty(InertialPositionNew)
   InertialPositionNew = NaN(3,ineritalN); 
   AccumulateCompensate = NaN(3,ineritalN); 
end

for k = CalStartIN : CalEndIN   
    if CalStartIN == 1
        InertialPositionNew_k = InertialPosition(:,1);
        AccumulateCompensate_k = zeros(3,1);
    else
        if isnan(trackedMakerPosition_InertialTime(1,k))   % 上一时刻没有跟踪成功，直接纯惯性递推
            InertialPositionNew_k = InertialPositionNew_k+( InertialPosition(:,k)-InertialPosition(:,k-1) );
            
        else
            [ SingleFrameCompensate_k,AccumulateCompensate_k,InertialPositionNew_k ] = VNSCompensateINS_k...
                ( compensateRate,trackedMakerPosition_InertialTime(:,k),InertialPosition(:,k-1:k),InertialPositionNew_k );
        end
    end
    if coder.target('MATLAB')
        InertialPositionNew(:,k) = InertialPositionNew_k;
        AccumulateCompensate(:,k) = AccumulateCompensate_k;
    end
end

AccumulateCompensate_k_Out = AccumulateCompensate_k;

 

if coder.target('MATLAB')
    if CalEndIN >= ineritalN-2 || CalEndVN >= visualN-2
        DrawCompensated( compensateRate,trackedMakerPosition_InertialTime,InertialPosition,InertialPositionNew,...
            AccumulateCompensate,ineritalN );
    end
end


%% 单帧 位移补偿 k时刻
%%% Input
% trackedMakerPosition_InertialTime ： 马克点光学位置，按惯性时序存储
% HipDisplacement ： Hip在北东地下的位置 （由 BVH 得到）
% InertialPosition_k： [3,2] k-1和k时刻   惯性系与马克点安装位置对应关节的位置（安装在头上时，InertialPosition 为惯性头的位置）
%%% Output
% InertialPositionNew ： 补偿后惯性的位置
% HipDisplacementNew： 补偿后Hip的位置
function [ SingleFrameCompensate_k,AccumulateCompensate_k,InertialPositionNew_k ] = VNSCompensateINS_k...
( compensateRate,trackedMakerPosition_InertialTime_k,InertialPosition_k,InertialPositionNew_k_last )
coder.inline('never');
% coder.extrinsic('fprintf');
% coder.extrinsic('DrawCompensate');
% global CalStartIN  CalEndIN     
% persistent InertialPositionNew_k  % 在k-1时刻补偿后位置的基础上，进行一步纯惯性递推得到的位置
% 先用纯惯性递推：在前一时刻补偿后基础上，惯性递推一步得到的位移
InertialPositionNew_k = InertialPositionNew_k_last+( InertialPosition_k(:,2)-InertialPosition_k(:,1) );
% 先求纯惯性误差
InertialErr_k = trackedMakerPosition_InertialTime_k - InertialPositionNew_k ;  % 当前相对光学的误差
InertialErr_k(3) = 0;   % 高度方向不补偿
SingleFrameCompensate_k = InertialErr_k*compensateRate ;  % 单帧补偿量
InertialPositionNew_k = InertialPositionNew_k + SingleFrameCompensate_k;   % 补偿 ： 向光学靠近
AccumulateCompensate_k = InertialPositionNew_k-InertialPosition_k(:,2) ;  % 累积补偿量

 
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

% %% BVH 读取的个数 N_BVH 可能会比 N1 多几个
N_BVH = size(HipDisplacement,2);
% N1 = size(trackedMakerPosition_InertialTime,2);
% for k=N1+1:N_BVH
%     trackedMakerPosition_InertialTime(:,k) = trackedMakerPosition_InertialTime(:,N1);
%     InertialPosition(:,k) = InertialPosition(:,N1);
% end

%% 先补偿 InertialPositionNew
persistent SingleFrameCompensate  InertialPositionNew HipDisplacementNew
if isempty(SingleFrameCompensate)
    SingleFrameCompensate = NaN(3,N_BVH); % 每一步的累积位移补偿量 记录
    SingleFrameCompensate(:,1)=zeros(3,1);
    InertialPositionNew = NaN(3,N_BVH);
    InertialPositionNew(3,:) = InertialPosition(3,:); % 高度不补偿
    InertialPositionNew(1:2,1) = InertialPosition(1:2,1) ;  % 起始点选择惯性
    
    HipDisplacementNew = NaN(3,N_BVH); 
    HipDisplacementNew(3,:) = HipDisplacement(3,:);
    HipDisplacementNew(:,1) = HipDisplacement(:,1);
end
persistent   InertialErr
if isempty(InertialErr)
    InertialErr = NaN(2,N_BVH);       % 实时记录补偿后的误差
end
CalN = CalEndIN-CalStartIN+1; % 涉及本次计算的数据长度
AccumulateCompensate_k = zeros(3,CalN);

for k = max(2,CalStartIN) : CalEndIN 
    i = k-CalStartIN+1;  % 在本次计算中的序号
    % 先用纯惯性递推：在前一时刻补偿后基础上，惯性递推一步得到的位移
    if coder.target('MATLAB')
        % MATLAB 离线处理时，得到的 InertialPosition
        % 是一直没有补偿处理的。先利用纯惯导递推得到仅最新一个时刻没有补偿的结果
        InertialPositionNew(1:2,k) = InertialPositionNew(1:2,k-1)+( InertialPosition(1:2,k)-InertialPosition(1:2,k-1) );
    else
        % 实时运算时，得到的 InertialPosition 是只有 CalStartIN: CalEndIN  时刻没有补偿处理的。
        InertialPositionNew(1:2,k) = InertialPositionNew(1:2,k-1)+( InertialPosition(1:2,k)-InertialPosition(1:2,k-1) );
    end
    % 先求纯惯性误差
    if ~isnan(trackedMakerPosition_InertialTime(1,k))
        InertialErr(:,k) = trackedMakerPosition_InertialTime(1:2,k) - InertialPositionNew(1:2,k) ;  % 当前相对光学的误差
        % 补偿误差
        SingleFrameCompensate(1:2,k) = InertialErr(:,k)*compensateRate ;  % 单帧补偿量        
    else
        SingleFrameCompensate(1:2,k) = [0;0] ;  % 单帧补偿量
    end
    InertialPositionNew(1:2,k) = InertialPositionNew(1:2,k) + SingleFrameCompensate(1:2,k) ;  % 向光学靠近
    InertialPositionNew(3,k) = InertialPosition(3,k);
    AccumulateCompensate_k(:,i) = InertialPositionNew(:,k)-InertialPosition(:,k) ;  % 累积补偿量
end

%% 通过 InertialPositionNew 计算 HipDisplacementNew
% 将Head位置传到Hip，：保持头和head的相对位移

for k=CalStartIN:CalEndIN    
    HipDisplacementNew( 1:2,k ) = HipDisplacement(1:2,k) + SingleFrameCompensate(1:2,k) ;
    HipDisplacementNew( 3,k ) = HipDisplacement( 3,k );
end

InertialPositionCompensate_out = SingleFrameCompensate ;

HipDisplacementNew_out = HipDisplacementNew;

if coder.target('MATLAB') &&  CalEndIN >= size(trackedMakerPosition_InertialTime,2)-1 
    DrawCompensate( InertialErr,SingleFrameCompensate,trackedMakerPosition_InertialTime,...
        InertialPosition,InertialPositionNew,SingleFrameCompensate,compensateRate,N_BVH ) ;
end



%% 视觉位置预处理
% 1）将视觉数据  从视觉世界坐标系 转到 北东地坐标系
function otherMakers = PreProcess( otherMakers  )
coder.inline('never');
global CalStartVN CalEndVN 
persistent VisualP_t0  % 0时刻的视觉位置
if isempty(VisualP_t0)
   VisualP_t0 = zeros(3,1); 
end
%% 先转到北东地的同名坐标系
Cv_r1 = [ 0 0 1; -1 0 0; 0 -1 0 ];
% dbstop in BodyDirection2Cr_r1
% Cr_r1 = BodyDirection2Cr_r1( BodyDirection );   %   （法1）    要求人朝视觉标定标定的世界坐标系进行对准
% Cvr = Cr_r1' * Cv_r1 ;
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
