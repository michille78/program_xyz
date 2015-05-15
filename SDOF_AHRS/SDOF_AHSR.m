%% xyz 2015.4.21



function SDOF_AHSR(  )

% clc
clear all
close all

% dataFolder = 'E:\data_xyz\AHRS Data\TurntableData_5.4-AllData';
% dataName = 'TurntableData_1' ;
% AHRSData = importdata( [ dataFolder,'\IMU_',dataName,'.mat' ] );

dataFolder = 'E:\data_xyz\AHRS Data\ahrs_raw_data_4.20\Xu';
dataName = 'ahrs1';
AHRSData = importdata( [ dataFolder,'\',dataName,'.mat' ] );

Nframes  = length(AHRSData.accNorm);
AHRSData.Nframes = Nframes ;

save( [ dataFolder,'\IMU_',dataName,'.mat' ],'AHRSData' )


% spanTime = 0.05;
% AHRSData = IMUDataPreprocess( AHRSData,spanTime );

%% 转台参考数据：第二列是测量值，°，1000HZ
% RefRotateAngle = importdata( [ dataFolder,'\Ref_',dataName,'.mat' ] );
RefRotateAngle = importdata( [ dataFolder,'\ref\',dataName,'.dat' ] );
AHRSRefData.RefRotateAngle = RefRotateAngle(:,2)'*pi/180;
AHRSRefData.frequency = 1000 ;


%% parameters to set
NavigationFrame = 'NED';
%%% the threshold value to judge .. state
%%% 动态变化过程中，0加速度时刻的判断 指标
AHRSThreshod.GyroNormZeroThreshod = 0.7 *pi/180 ;       % 0.7 °/s  角速度为0判断下限    ( 1°/s的向心加速度时0.031 mg -> 可接受 )
AHRSThreshod.AccNormZeroThreshod = 3/1000 ;             % 3mg  加速度模为0判断下限 （这个要求是次要判断指标，不需要给太严格，主要还是通过角速度判断。）
AHRSThreshod.DynamicIsStaticSmoothStepTime = 0.1 ;      % 动态静止平滑步长时间

AHRSThreshod.maxAngularAcc_GyroZero = 2*pi/180 ;             % 角速度变化率=0判断：直线拟合的角加速度 °/s^2
AHRSThreshod.minGyroZeroContinuesT = 0.1 ;                   % 角速度变化率=0判断：角速度为0保持时间最小值
AHRSThreshod.IsContinuousGyroNormZeroSmoothStepTime = 0.3  ; % 角速度变化率为0判断结果平滑步长 （可以稍微长点）

AHRSThreshod.IsDoSmoothIsAccZero = 1;                      % 是否对初始0加速度判断结果做平滑
%%% 长时间保持 0加速度 判断指标： 初始零位判断、替代零位判断
AHRSThreshod.SmoothRate = 0.8;                            % 平滑IsOne数据时采用的判断比例，窗口中大于换这个比例则认为全为1，（判断长时间未1时先进行平滑）
AHRSThreshod.IsLongContinuousOnes_SmoothStepTime = 1 ;     % 长时间保持 0加速度 判断 前的 平滑步长时间
AHRSThreshod.IsLongContinuousOnes_JudgeStepTime = 0.3 ;   % 长时间保持 0加速度 判断的 判断步长时间
%%% 零位的保持时长

%%% 转轴计算指标
AHRSThreshod.RoateVectorCalMinAngleFirst = 10*pi/180;   % 假设航向保持0时，俯仰和横滚转动四元数的转角大于 RoateVectorCalMinAngleFirst 角度时，用于旋转轴的第一次计算
AHRSThreshod.RoateVectorCalMinAngleSecond = 20*pi/180;  % 根据初次转轴解算结果，选择转动角度大于 RoateVectorCalMinAngleSecond 的进行转轴的详细解算
AHRSThreshod.RoateVectorCalMinAngleScope = 10*pi/180 ;  % 转轴计算数据选择的转角范围，如果第二次旋转的转角小于这个范围，发出警告
AHRSThreshod.RoateVectorCalMinAngleScopeSub = 0.5*pi/180 ;% 正转角的转角范围 和 负转角的转角范围 的最小值

AHRSThreshod.RoateVectorAccCalTime  = 25 ;              % 从零位静止停止后多长时间的数据用于纯加计转轴计算。之后就要求输出角度。
AHRSThreshod.RoateVectorGyroCalTime = 6 ;
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
[ Ypr_Acc,RecordStr_Ypr_Acc ] = GetRotateVector_Acc( Qnb_RVCal,Qwr,AHRSThreshod,AccelerationZeroJudge_RVCal,frequency ) ;
%% calculate the rotate vector only by Gyro
RoateVectorGyroCalTime = AHRSThreshod.RoateVectorGyroCalTime;
RoateVectorGyroCalN = RoateVectorGyroCalTime*frequency + initialStaticEnd ;
imuInputData_RVCal.wibb = gyro(:,initialStaticEnd:RoateVectorGyroCalN); 
imuInputData_RVCal.fb_g = acc(:,initialStaticEnd:RoateVectorGyroCalN); 
imuInputData_RVCal.frequency = frequency ;
%  dbstop in GetRotateVector_Gyro
[ Ypr_Gyro,RecordStr_Ypr_Gyro ] = GetRotateVector_Gyro( imuInputData_RVCal,InitialData,AHRSThreshod,AccelerationZeroJudge );

Ypr_Gyro_Acc_difAngle = acos( Ypr_Acc'*Ypr_Gyro )*180/pi ;
Ypr_Gyro_Acc_difAngleStr = sprintf( 'difference angle 0f Ypr_Gyro and Ypr_Acc = %0.2f degree',Ypr_Gyro_Acc_difAngle );
disp(Ypr_Gyro_Acc_difAngleStr)
%% analyze the rotate vector
StaticNum = initialStaticStart:initialStaticEnd ;
YprAnalyzeStr = AnalyzeYpr( Qnb,Qwr,Ypr_Gyro,StaticNum,'Ypr_Gyro' ) 
YprAnalyzeStr = AnalyzeYpr( Qnb,Qwr,Ypr_Acc,StaticNum,'Ypr_Acc' ) 
%% calculate the rotate angle only by Acc
AccCalNum1 = find(IsSDOFAccelerationZero==1) ;
AccCalNum2 = find(IsSDOFAccelerationToHeartZero==1) ;
AccCalNum3 = find(IsAccNormZero==1) ;
AccCalNum0 = 1:Nframes;
RotateAngle_Acc0= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum0 ) ;
RotateAngle_Acc1= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum1 ) ;
RotateAngle_Acc2= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum2) ;
RotateAngle_Acc3= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Acc,AccCalNum3 ) ;

RotateAngle_Acc_GyroV1= CalculateRotateAngle_Acc( Qnb,Qwr,Ypr_Gyro,AccCalNum1 ) ;
%% calculate the rotate angle only by Gyro
% dbstop in CalculateRotateAngle_Gyro
RotateAngle_Gyro = CalculateRotateAngle_Gyro( imuInputData,InitialData,Ypr_Gyro );
%%
RotateAngle = RotateAngle_Gyro ;

DrawRotateAngle( RotateAngle_Gyro,0,frequency,'gyro' );
DrawRotateAngle( RotateAngle_Acc1,AccCalNum1,frequency,'Acc1' );
DrawRotateAngle( RotateAngle_Acc_GyroV1,AccCalNum1,frequency,'Acc_GyroV1' );


DrawRotateAngle( RotateAngle_Acc2,AccCalNum2,frequency,'Acc2'  );
DrawRotateAngle( RotateAngle_Acc3,AccCalNum3,frequency,'Acc3'  );

DrawRotateAngle( RotateAngle_Acc0,AccCalNum0,frequency,'Acc0',''  );


%% error
RotateAngleErrStrAcc = AnalyseRotateAngle( RotateAngle_Acc0,AHRSRefData,frequency,'Acc',AccCalNum0 ) ;
% RotateAngleErrStrGyro = AnalyseRotateAngle( RotateAngle_Gyro,AHRSRefData,frequency,'Gyro' ) ;


function DrawRotateAngle( RotateAngle,DataNum,frequency,DataName,mark )
if ~exist('mark','var')
   mark = '.'; 
end
if DataNum==0
   DataNum  = 1:length(RotateAngle); 
end
timeData = DataNum/frequency ;
figure('name',['RotateAngle-',DataName])
plot(timeData,RotateAngle*180/pi,mark)
xlabel('time /s')
ylabel(['RotateAngle-',DataName])


function RotateAngleErrStr = AnalyseRotateAngle( RotateAngle,AHRSRefData,frequency,dataName,CalNum )
if ~exist('CalNum', 'var')
    CalNum = 1:length(RotateAngle);
end 
%%  处理参考数据
RefRotateAngle = AHRSRefData.RefRotateAngle -30*pi/180 ;
RefFre = AHRSRefData.frequency ;

N_Ref = length(RefRotateAngle);
%%% 找到正弦的0点作为起点
k = 1 ;
while k<length(RefRotateAngle) 
   if sign( RefRotateAngle(k) )+sign( RefRotateAngle(k+1) ) ==0 || RefRotateAngle(k) == 0
       RefRotateAngle = RefRotateAngle( k:length(RefRotateAngle)  );       
       break;
   end
   k = k+1 ;
end


% 降频
N_Ref = length(RefRotateAngle);
N_RefNew = fix( (N_Ref-1)*frequency/RefFre )+1 ;
RefRotateAngleNew = zeros(1,N_RefNew);
for k=1:N_RefNew
    k_old = fix((k-1)*RefFre/frequency)+1;
    RefRotateAngleNew(k) = RefRotateAngle(k_old);
end
RefRotateAngle = RefRotateAngleNew ;
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

% Nframes = size(RotateAngleNew,2);
% SimRefRotateAngle = zeros(1,Nframes);
% for k=1:Nframes
%     SimRefRotateAngle(k) = -sin( 2*pi/frequency*10*k )*30*pi/180 ;
% end




N = min( length(RotateAngleNew),length(RefRotateAngle) );
RotateAngleErr = RefRotateAngle(1:N) - RotateAngleNew(1:N) ;
RotateAngleErr = RotateAngleErr/2;

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

legend('计算值','参考值')
xlabel('time /s')
