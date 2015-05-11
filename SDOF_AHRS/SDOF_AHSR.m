%% xyz 2015.4.21



function SDOF_AHSR(  )

% clc
clear all
close all

dataFolder = 'E:\data_xyz\AHRS Data\TurntableData_5.4-AllData';
dataName = 'TurntableData_0.5' ;

AHRSData = importdata( [ dataFolder,'\IMU_',dataName,'.mat' ] );
Nframes  = length(AHRSData.accNorm);
AHRSData.Nframes = Nframes ;
AHRSData.frequency = 250 ;
save( [ dataFolder,'\IMU_',dataName,'.mat' ],'AHRSData' )


% spanTime = 0.1;
% AHRSData = IMUDataPreprocess( AHRSData,spanTime );

%% 转台参考数据：第二列是测量值，°，1000HZ
RefRotateAngle = importdata( [ dataFolder,'\Ref_',dataName,'.mat' ] );
%%% 找到正弦的0点作为起点
k = 1 ;
while k<length(RefRotateAngle) 
   if sign( RefRotateAngle(k) )+sign( RefRotateAngle(k+1) ) ==0 || RefRotateAngle(k) == 0
       RefRotateAngle = RefRotateAngle( k:length(RefRotateAngle)  );       
       break;
   end
   k = k+1 ;
end

AHRSRefData.frequency = 1000 ;

NframesNew = fix( length(RefRotateAngle)* AHRSData.frequency / AHRSRefData.frequency ) ;
NframesNew = min( NframesNew,Nframes );
RefRotateAngleNew = zeros(1,NframesNew);
for k=1:NframesNew
    k_raw = fix( k* AHRSRefData.frequency / AHRSData.frequency);
    RefRotateAngleNew(k) = RefRotateAngle(k_raw);
end
AHRSRefData.RefRotateAngle = RefRotateAngleNew ;

%% parameters to set
NavigationFrame = 'NED';
%%% the threshold value to judge .. state
%%% 动态变化过程中，0加速度时刻的判断 指标
AHRSThreshod.GyroNormZeroThreshod = 0.7 *pi/180 ;       % 0.7 °/s  角速度为0判断下限
AHRSThreshod.AccNormZeroThreshod = 3/1000 ;             % 3mg  加速度模为0判断下限
AHRSThreshod.DynamicIsStaticSmoothStepTime = 0.05 ;     % 动态静止平滑步长时间
AHRSThreshod.GyroContinuousZeroTimeThreshod = 0.3 ;     % 角速度变化率=0判断：角速度为0保持时间
AHRSThreshod.GyroContinuousZeroMinRate = 0.7 ;          % 角速度变化率=0判断：角速度为0在邻域内保持的比例
AHRSThreshod.IsContinuousGyroNormZeroSmoothStepTime = 0.2  ; % 角速度变化率为0判断结果平滑步长 （可以稍微长点）
%%% 长时间保持 0加速度 判断指标： 初始零位判断、替代零位判断
AHRSThreshod.IsLongContinuousOnes_SmoothStepTime = 0.3 ;  % 长时间保持 0加速度 判断 前的 平滑步长时间
AHRSThreshod.IsLongContinuousOnes_JudgeStepTime = 0.1 ;   % 长时间保持 0加速度 判断的 判断步长时间

%%% 转轴计算指标
AHRSThreshod.RoateVectorCalMinAngleFirst = 10*pi/180;   % 假设航向保持0时，俯仰和横滚转动四元数的转角大于 RoateVectorCalMinAngleFirst 角度时，用于旋转轴的第一次计算
AHRSThreshod.RoateVectorCalMinAngleSecond = 20*pi/180;  % 根据初次转轴解算结果，选择转动角度大于 RoateVectorCalMinAngleSecond 的进行转轴的详细解算
AHRSThreshod.RoateVectorCalMinAngleScope = 10*pi/180 ;  % 转轴计算数据选择的转角范围，如果第二次旋转的转角小于这个范围，发出警告
AHRSThreshod.RoateVectorCalMinAngleScopeSub = 1*pi/180 ;% 正转角的转角范围 和 负转角的转角范围 的最大值
AHRSThreshod.RoateVectorAccCalTime  = 25 ;              % 从零位静止停止后多长时间的数据用于纯加计转轴计算。之后就要求输出角度。

%%
RotateAngle = SDOF_AHSR_One( AHRSData,AHRSRefData,NavigationFrame,AHRSThreshod,RefRotateAngle ) ;

disp( '     SDOF_AHSR finished ' );
%% 
function RotateAngle = SDOF_AHSR_One( AHRSData,AHRSRefData,NavigationFrame,AHRSThreshod,RefRotateAngle )

%% load data
% quaternion = AHRSData.quaternion ;
gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = Nframes/frequency ;

%% SINS Data Format
imuInputData.wibb = gyro ;
imuInputData.fb_g = acc ;
imuInputData.frequency = frequency ;

InitialData.NavigationFrame = NavigationFrame;
InitialData.Vwb0 = zeros(3,1);
InitialData.rwb0 = zeros(3,1);

%% Calculate time of initial static state 
%   dbstop in Judge0Acceleration
[ AccelerationZeroJudge,initialStaticStart,initialStaticEnd ]= Judge0Acceleration( AHRSData,AHRSThreshod,RefRotateAngle ) ;

IsSDOFAccelerationZero = AccelerationZeroJudge.IsSDOFAccelerationZero  ;
IsSDOFAccelerationToHeartZero = AccelerationZeroJudge.IsSDOFAccelerationToHeartZero  ;
IsAccNormZero = AccelerationZeroJudge.IsAccNormZero  ;
IsLongSDOFAccelerationZero = AccelerationZeroJudge.IsLongSDOFAccelerationZero ;
IsLongSDOFAccelerationToHeartZero = AccelerationZeroJudge.IsLongSDOFAccelerationToHeartZero ;
IsLongAccNormZero = AccelerationZeroJudge.IsLongAccNormZero ;

initialStaticTime = ( initialStaticEnd-initialStaticStart )/frequency ;
if initialStaticTime < 1
   errordlg(sprintf('初始静止状态时长=%0.2f sec， 太短！',initialStaticTime)); 
end
%% calculate the initial static attitude
[ pitch,roll,Qnb ] = Acc2PitchRoll( acc,NavigationFrame ) ;
%%% initial static acc  gyro  : r frame
accStatic = acc( :,initialStaticStart:initialStaticEnd ) ;
acc_r = mean( accStatic,2 );
gyroStatic = gyro( :,initialStaticStart:initialStaticEnd ) ;
gyro_r = mean( gyroStatic,2 );
[ pitch_r,roll_r,Qwr ] = Acc2PitchRoll( acc_r,NavigationFrame ) ;

InitialData.Qwb0 = Qwr ;
%% calculate the rotate vector only by Acc
%%% 截取零位计算时段的数据 Qnb_ZeroCal
RoateVectorAccCalTime = AHRSThreshod.RoateVectorAccCalTime ; 
RoateVectorCalN = RoateVectorAccCalTime*frequency + initialStaticEnd ;
Qnb_RVCal = Qnb( :,1:RoateVectorCalN ) ;
% dbstop in GetRotateVector_Acc
AccelerationZeroJudge_RVCal.IsAccNormZero = IsAccNormZero(1:RoateVectorCalN);
AccelerationZeroJudge_RVCal.IsSDOFAccelerationToHeartZero = IsSDOFAccelerationToHeartZero(1:RoateVectorCalN);
AccelerationZeroJudge_RVCal.IsSDOFAccelerationZero = IsSDOFAccelerationZero(1:RoateVectorCalN);
AccelerationZeroJudge_RVCal.IsLongSDOFAccelerationZero = IsLongSDOFAccelerationZero(1:RoateVectorCalN);
[ Ypr_Acc,RecordStr_Ypr_Acc ] = GetRotateVector_Acc( Qnb_RVCal,Qwr,AHRSThreshod,AccelerationZeroJudge_RVCal ) ;
%% calculate the rotate vector only by Gyro
imuInputData_RVCal.wibb = gyro(:,initialStaticEnd:RoateVectorCalN); 
imuInputData_RVCal.fb_g = acc(:,initialStaticEnd:RoateVectorCalN); 
imuInputData_RVCal.frequency = frequency ;
%  dbstop in GetRotateVector_Gyro
[ Ypr_Gyro,RecordStr_Ypr_Gyro ] = GetRotateVector_Gyro( imuInputData_RVCal,InitialData,AHRSThreshod,AccelerationZeroJudge );

Ypr_Gyro_Acc_difAngle = acos( Ypr_Acc'*Ypr_Gyro )*180/pi ;
Ypr_Gyro_Acc_difAngleStr = sprintf( 'difference angle 0f Ypr_Gyro and Ypr_Acc = %0.2f degree',Ypr_Gyro_Acc_difAngle );
disp(Ypr_Gyro_Acc_difAngleStr)
%% calculate the rotate angle only by Acc
AccCalNum1 = find(IsSDOFAccelerationZero==1) ;
AccCalNum2 = find(IsSDOFAccelerationToHeartZero==1) ;
AccCalNum3 = find(IsAccNormZero==1) ;
RotateAngle_Acc1= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum1 ) ;
RotateAngle_Acc2= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum2) ;
RotateAngle_Acc3= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum3 ) ;
%% calculate the rotate angle only by Gyro
% dbstop in CalculateRotateAngle_Gyro
RotateAngle_Gyro = CalculateRotateAngle_Gyro( imuInputData,InitialData,Ypr_Gyro );
%%
RotateAngle = RotateAngle_Gyro ;

DrawRotateAngleAcc( RotateAngle_Acc1,AccCalNum1,frequency );
DrawRotateAngleAcc( RotateAngle_Acc2,AccCalNum2,frequency );
DrawRotateAngleAcc( RotateAngle_Acc3,AccCalNum3,frequency );
%% error
RotateAngleErrStrAcc = AnalyseRotateAngle( RotateAngle_Acc,AccCalNum,AHRSRefData,frequency,'Acc' ) ;
RotateAngleErrStrGyro = AnalyseRotateAngle( RotateAngle_Gyro,AHRSRefData,frequency,'Gyro' ) ;


function DrawRotateAngleAcc( RotateAngle_Acc,AccCalNum,frequency )
timeData = AccCalNum/frequency ;
figure('name','RotateAngle_Acc')
plot(timeData,RotateAngle_Acc*180/pi,'.')
xlabel('time /s')
ylabel('RotateAngle_Acc ^o')


function RotateAngleErrStr = AnalyseRotateAngle( RotateAngle,AHRSRefData,frequency,dataName,CalNum )
if ~exist( 'var','CalNum' )
    CalNum = 1:length(RotateAngle);
end 
%% 取第一个正弦的0点作为起点，从而与参考数据进行对比
stepN = 20 ;
k=stepN+1;
while k<length(RotateAngle)-stepN
   sum_sign1 =  sum ( sign( RotateAngle(k-stepN:k-1) ) )  ;
   sum_sign2 =  sum ( sign( RotateAngle(k+1:k+stepN) ) )  ;
   if sum_sign1 > stepN-1 && sum_sign2 < -stepN+1
       RotateAngleNew = RotateAngle( k:length(RotateAngle) );
       break;
   end
   k=k+1;
end

RefRotateAngle = AHRSRefData.RefRotateAngle ;
N = min( length(RotateAngleNew),length(RefRotateAngle) );
RotateAngleErr = RotateAngleNew(1:N) - RefRotateAngle(1:N) ;

RotateAngleErr_Mean = mean(RotateAngleErr)*180/pi;
RotateAngleErr_Std = std(RotateAngleErr)*180/pi;

RotateAngleErrStr = sprintf( ' RotateAngleErr_Mean=%0.2f \n RotateAngleErr_Std=%0.2f \n',RotateAngleErr_Mean,RotateAngleErr_Std );
disp(RotateAngleErrStr);
%% draw
timeData = (1:N)/frequency ;
figure('name',[dataName,'-RotateAngleErr'])
plot(timeData,RotateAngleErr*180/pi)
xlabel('time /s')

figure('name',[dataName,'-RotateAngle'])
plot( timeData,RotateAngleNew(1:N)*180/pi,'b' )
hold on
plot( timeData,RefRotateAngle(1:N)*180/pi,'r' )
% plot(RotateAngleErr*180/pi,'k,')

legend('RotateAngleNew','RefRotateAngle')
xlabel('time /s')
