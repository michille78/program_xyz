%% xyz 2015 5.25

%% otherMakers
% otherMakers(k).frequency [1]
% otherMakers(k).Position  [3*M]
% otherMakers(k).otherMakersN [1]
%　otherMakers(k).time [1]
%　otherMakers(k).MarkerSet ""

        % 记录每个马克点的连续特性
% otherMakers(k).ContinuesFlag = zeros(1,M) ; % 不连续
% otherMakers(k).ContinuesLastPosition = NaN(3,M)  ;
% otherMakers(k).ContinuesLastTime = NaN[1*M] ; 
% otherMakers(k).ContinuesLastK = NaN[1*M];

%% InertialData
% InertialData.frequency (k)
% InertialData.time (k)
% InertialData.HipQuaternion(k)  [4*N]
% InertialData.HipPosition (k)  [3*N]
% InertialData.HeadQuaternion (k)  [4*N]
% InertialData.HeadPosition (k)  [3*N]
% InertialData.BodyDirection(k)  [3*1]
% InertialData.DataStyle(k)  'GlobalBoneQuat'


%% 用 Optitrack 的 OtherMarker 补偿惯性系统


%% 得到惯性 Hip 位置补偿量 InertialPositionCompensate
% InertialPositionCompensate [ 3*N ]  m  NED系

function [ InertialPositionCompensate,HipDisplacementNew,InertialData1,otherMakers1,compensateRate1,InertialData2,otherMakers2 ] = GetINSCompensateFromVNS( InertialData,otherMakers,compensateRate )

coder.extrinsic('fprintf');
coder.inline('never');
coder.extrinsic('DrawTracedINSVNS');
coder.varsize('otherMakers2','v_Position');

%% 

I_N = size( InertialData.time,2 );
V_N = length(otherMakers);
MaxOtherMarkerBufN = size(otherMakers(1).ContinuesFlag,2);

%% InertialData  和 otherMakers 置 NaN
[InertialData,otherMakers] = Set_InertialData_otherMakers_NaN( InertialData,otherMakers );



%% InertialData 格式数据创建

InertialData2.HeadQuaternion = InertialData.HeadQuaternion;


%% InertialData 格式数据Copy
InertialData1 = InertialData;

%% otherMakers 格式数据创建
otherMakers2 =otherMakers(1);
otherMakers2(15) =otherMakers(5);


%% otherMakers 格式数据Copy
otherMakers1 =otherMakers;

%% compensateRate
compensateRate1 = compensateRate;

%% InertialData 数据赋值
InertialData1.frequency = 96;
InertialData.HipPosition(:,1) = zeros(3,1);
InertialData.HipQuaternion(:,1) = NaN(4,1);


%% otherMakers 数据赋值
otherMakers1(1).frequency = 30;
P1 = [1;2;3];
P2 = [0.1;0.2;0.3];
otherMakers1(1).Position(:,1:2) = [P1 P2];
otherMakers1(1).MarkerSet = int8(16);

% %% 给 otherMakers 赋值函数 1
% frequency = otherMakers(2).frequency;
% time = otherMakers(2).time;
% v_Position = zeros(3,20);
% v_Position(:,32) = [1;3;-2];
% otherMakersN = otherMakers(6).otherMakersN;
% 
% for visual_k=1:100
%     otherMakers1 = Assign_otherMakers_1( otherMakers1,visual_k,frequency,time,v_Position,otherMakersN );
% end


%% 初始化输出为 NaN
function InitialOutput( InertialPositionCompensate,HipDisplacementNew )


%% InertialData  和 otherMakers 置 NaN
%% 用于给 C++ 自动生成 初始化 函数

function [InertialData,otherMakers] = Set_InertialData_otherMakers_NaN( InertialData,otherMakers )
coder.inline('never');

I_N = size( InertialData.time,2 );
V_N = length(otherMakers);
MaxOtherMarkerBufN = size(otherMakers(1).ContinuesFlag,2);

InertialData.frequency = 96;
InertialData.HipQuaternion  =  NaN(4,I_N);
InertialData.time  = NaN(1,I_N);
InertialData.HipPosition =  NaN(3,I_N);
InertialData.HeadQuaternion  =  NaN(4,I_N);
InertialData.HeadPosition =  NaN(3,I_N);
InertialData.BodyDirection =  NaN(3,1);
% InertialData.DataStyle =  'GlobalBoneQuat';

for k=1:V_N
    otherMakers(k).frequency = 30;
    otherMakers(k).Position = NaN(3,MaxOtherMarkerBufN);
    otherMakers(k).otherMakersN = int32(NaN);
    otherMakers(k).time = NaN;
    otherMakers(k).MarkerSet = int8(16); % 'head';
    otherMakers(k).ContinuesFlag = NaN(1,MaxOtherMarkerBufN);
    otherMakers(k).ContinuesLastPosition = NaN(3,MaxOtherMarkerBufN);
    otherMakers(k).ContinuesLastTime = NaN(1,MaxOtherMarkerBufN);
    otherMakers(k).ContinuesLastK = NaN(1,MaxOtherMarkerBufN);
end

%% 给 otherMakers 赋值函数 1
% 给第 visual_k 个值的 frequency,time,Position,otherMakersN 赋值
function otherMakers = Assign_otherMakers_1( otherMakers,visual_k,frequency,time,Position,otherMakersN )
coder.inline('never');

otherMakers(visual_k).frequency = frequency ;
otherMakers(visual_k).time = time ;
otherMakers(visual_k).Position = Position ;
otherMakers(visual_k).otherMakersN = int32(otherMakersN) ;
