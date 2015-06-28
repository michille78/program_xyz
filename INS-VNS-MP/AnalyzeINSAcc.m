%% xyz 2015.6.24

%% 分析惯性加速度

function AnalyzeINSAcc(  )

clc
clear all
% close all
global dataFolder 
 dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\摔头1';
% dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\头-综合';
dataName = 'CalculationData';
% dataName = 'GlobalAcc';

%% load data
CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadA = CalStruct.Head.A ;
LeftHandA = CalStruct.LeftHand.A ;
LeftFoot = CalStruct.LeftFoot.A;

HeadA(3,:) = HeadA(3,:)-1 ;
LeftHandA(3,:) = LeftHandA(3,:)-1 ;
LeftFoot(3,:) = LeftFoot(3,:)-1 ;

inertialFre = 96;
inertialN = size(HeadA,2);

time = (1:inertialN)/inertialFre;

%% 检测头的波峰和波谷
HeadA_V = NaN(5,inertialN);  
HeadA_V_SubRate = 1/5;  % 成比例减小 HeadA_V （绘图时）


adjacentN = 4;  %  波峰波谷判断延迟个数
waveThreshold_Min_dataA  = 30;
MinWaveData = 0.3;      % 波峰波谷处 abs(data) 最小值

HeadA_WaveFlag = NaN(3,inertialN); 
data_Acc_wave = NaN(3,inertialN);
k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）
for j=1:inertialN
    
    %% 计算速度
   [ Velocity_k,k_calV ] = CalVelocity( HeadA,j,inertialFre,0.15,0.1,2 ) ;
   if k_calV>0  && ~isnan(Velocity_k(1))
        HeadA_V(:,k_calV) = Velocity_k ;
   end
   %% 判断 第 k_wave_i = k_calV - adjacentN; 个点的 波峰波谷特征
   [ WaveFlag_k,k_waves,data_Acc_k_wave ] = FindCrestThrough( HeadA,inertialFre,HeadA_V,k_calV,adjacentN,...
       waveThreshold_Min_dataA,MinWaveData );
   % 第 k_waves(i) 个点判断成功后，后续的 adjacentN 个点就不再判断（不然可能会出现误判反而覆盖的现象）
   for i=1:3
       
       if k_waves(i)>0 && k_waves(i)~=k_waves_OKLast(i)
           HeadA_WaveFlag(i,k_waves(i)) = WaveFlag_k(i);
           data_Acc_wave(i,k_waves(i)) = data_Acc_k_wave(i);
       end
       % 记录上一时刻 判断成功的位置
       if ~isnan(WaveFlag_k(i))
           k_waves_OKLast(i) = k_waves(i);
       end
   end
end




%% HeadA 
figure('name',[dataName,' - HeadA'])

subplot( 3,1,1 );
plot(time,HeadA(1,:))
title(get(gcf,'name'))
hold on
plot( time,HeadA_V(1,:)*HeadA_V_SubRate,'g' )
plot( time,HeadA_WaveFlag(1,:),'*r' )
ylabel('x')

subplot( 3,1,2 );
plot(time,HeadA(2,:))
hold on
plot( time,HeadA_V(2,:)*HeadA_V_SubRate,'g' )
plot( time,HeadA_WaveFlag(2,:),'*r' )
ylabel('y')

subplot( 3,1,3 );
plot(time,HeadA(3,:))
hold on
plot( time,HeadA_V(3,:)*HeadA_V_SubRate,'g' )
plot( time,HeadA_WaveFlag(3,:),'*r' )
ylabel('z')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

%%
figure('name',[dataName,' - HeadA-Acc-wave'])
subplot( 3,1,1 );
plot(time,data_Acc_wave(1,:),'.m')
subplot( 3,1,2 );
plot(time,data_Acc_wave(2,:),'.m')
subplot( 3,1,3 );
plot(time,data_Acc_wave(3,:),'.m')

%% LeftHandA 
figure('name',[dataName,' - LeftHandA'])

subplot( 3,1,1 );
plot(time,LeftHandA(1,:))
title(get(gcf,'name'))
ylabel('x')

subplot( 3,1,2 );
plot(time,LeftHandA(2,:))
ylabel('y')

subplot( 3,1,3 );
plot(time,LeftHandA(3,:))
ylabel('z')
saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

return;
%% LeftFoot 
figure('name',[dataName,' - LeftFoot'])

subplot( 3,1,1 );
plot(time,LeftFoot(1,:))
title(get(gcf,'name'))
ylabel('x')

subplot( 3,1,2 );
plot(time,LeftFoot(2,:))
ylabel('y')

subplot( 3,1,3 );
plot(time,LeftFoot(3,:))
ylabel('z')

disp('')

%% 计算加速度的大小和方向
% Acc [3*N]  x、y、z
% AccOut[5*N]  
%       AccOut(4,:) xyz 3D模
%       AccOut(5,:) xy 平面方向
%       AccOut(6,:) xy平面模/地向大小
function GetNormAngle( Acc )