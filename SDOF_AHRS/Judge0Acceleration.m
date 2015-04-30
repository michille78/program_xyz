%% xyz 2015.4.23

%% Judge is the IMU being state of acceleration==0
% IsSDOFAccelerationZero:N*1  利用 SDOF运动特点综合判断的
% IsSDOFAccelerationZero(k)==1: 判定为静止
% IsSDOFAccelerationZero(k)==1: 判定为非静止
function [ AccelerationZeroJudge,initialStaticStart,initialStaticEnd ] = Judge0Acceleration( AHRSData,AHRSThreshod )
% load data
% gyro = AHRSData.gyro ;
% acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;

%%% the threshold value to judge is being static
AccNormZeroThreshod = AHRSThreshod.AccNormZeroThreshod ;
GyroNormZeroThreshod = AHRSThreshod.GyroNormZeroThreshod ;
GyroContinuousZeroTimeThreshod = AHRSThreshod.GyroContinuousZeroTimeThreshod ;
GyroContinuousZeroMinRate = AHRSThreshod.GyroContinuousZeroMinRate ;
IsContinuousGyroNormZeroSmoothStepTime = AHRSThreshod.IsContinuousGyroNormZeroSmoothStepTime ;
DynamicIsStaticSmoothStepTime = AHRSThreshod.DynamicIsStaticSmoothStepTime ;
IsLongContinuousOnes_SmoothStepTime = AHRSThreshod.IsLongContinuousOnes_SmoothStepTime ;
IsLongContinuousOnes_JudgeStepTime = AHRSThreshod.IsLongContinuousOnes_JudgeStepTime ;

DynamicIsStaticSmoothStepN = fix( DynamicIsStaticSmoothStepTime*frequency ) ;
%% （1）加速度的模与重力加速度相差小于 AccNormZeroThreshod = 1~3mg
accNormErr = abs( accNorm-1 ) ;
IsAccNormZero = accNormErr < AccNormZeroThreshod ;
IsAccNormZero = SmoothJudgeData( IsAccNormZero,DynamicIsStaticSmoothStepN,0.6 ) ; 
%% (2) 陀螺测量得到的角速度的模小于 GyroNormZeroThreshod = 0.3~0.5 °/s
IsGyroNormZero = gyroNorm < GyroNormZeroThreshod ;
IsGyroNormZero = SmoothJudgeData( IsGyroNormZero,DynamicIsStaticSmoothStepN,0.6 ) ; 
%% (3) 角速度变化率为0：  IsGyroNormZero = 1 的点是否连续在 GyroVelocityZeroTimeThreshod = 0.05 S 内保持为0
frontT = GyroContinuousZeroTimeThreshod*0.8 ;
laterT = GyroContinuousZeroTimeThreshod*0.2 ;
minRateFront = GyroContinuousZeroMinRate ;
minRateLater = GyroContinuousZeroMinRate ;
% dbstop in JudgeContinuousOnes
IsContinuousGyroNormZero = JudgeContinuousOnes( IsGyroNormZero,frontT,laterT,frequency,minRateFront,minRateLater ) ;
stepN = fix( IsContinuousGyroNormZeroSmoothStepTime*frequency ) ;
IsContinuousGyroNormZero = SmoothJudgeData( IsContinuousGyroNormZero,stepN,0.6 ) ;      % 
%% 同时满足以上条件认定为0加速度
IsSDOFAccelerationZero = IsContinuousGyroNormZero.*IsAccNormZero ;  %  切向加速度和法向加速度都为0
IsSDOFAccelerationToHeartZero = IsGyroNormZero.*IsAccNormZero ;  %  仅向心加速度为0

IsSDOFAccelerationZero = SmoothJudgeData( IsSDOFAccelerationZero,DynamicIsStaticSmoothStepN,0.6 ) ;
%% 判断初始零位静止时间长度
%  dbstop in JudgeLongContinuousOnes
[ LongSDOFAccelerationZeroStart,LongSDOFAccelerationZeroEnd,IsLongSDOFAccelerationZero ] = JudgeLongContinuousOnes...
    ( IsSDOFAccelerationZero,IsLongContinuousOnes_SmoothStepTime,IsLongContinuousOnes_JudgeStepTime,frequency ) ;
% 第一次长时间保持0位表示为零位角度
initialStaticStart = LongSDOFAccelerationZeroStart(1);
initialStaticEnd = LongSDOFAccelerationZeroEnd(1);

%% Output All the Acceleration Zero result
[ ~,~,IsLongSDOFAccelerationToHeartZero ] = JudgeLongContinuousOnes...
    ( IsSDOFAccelerationToHeartZero,IsLongContinuousOnes_SmoothStepTime,IsLongContinuousOnes_JudgeStepTime,frequency ) ;
[ ~,~,IsLongAccNormZero ] = JudgeLongContinuousOnes...
    ( IsAccNormZero,IsLongContinuousOnes_SmoothStepTime,IsLongContinuousOnes_JudgeStepTime,frequency ) ;

AccelerationZeroJudge.IsSDOFAccelerationZero = IsSDOFAccelerationZero ;
AccelerationZeroJudge.IsSDOFAccelerationToHeartZero = IsSDOFAccelerationToHeartZero ;
AccelerationZeroJudge.IsAccNormZero = IsAccNormZero ;
AccelerationZeroJudge.IsLongSDOFAccelerationZero = IsLongSDOFAccelerationZero ;
AccelerationZeroJudge.IsLongSDOFAccelerationToHeartZero = IsLongSDOFAccelerationToHeartZero ;
AccelerationZeroJudge.IsLongAccNormZero = IsLongAccNormZero ;

%%%
DrawIsAccelerationZero( AHRSData,IsSDOFAccelerationZero,'IsSDOFAccelerationZero' ) ;

figure('name','Judge0Acceleration')
axes('YLim',[-0.2 1.2])
subplot(2,1,1)
hold on
plot(IsSDOFAccelerationZero,'k.')
plot(IsSDOFAccelerationToHeartZero*0.97,'b.')
plot(IsAccNormZero*0.95,'ro')
legend('IsSDOFAccelerationZero','IsSDOFAccelerationToHeartZero','IsAccNormZero')

subplot(2,1,2)
hold on
plot(IsLongSDOFAccelerationZero,'k.')
plot(IsLongSDOFAccelerationToHeartZero*0.97,'b.')
plot(IsLongAccNormZero*0.95,'ro')
legend('IsLongSDOFAccelerationZero','IsLongSDOFAccelerationToHeartZero','IsLongAccNormZero')



%% 计算初始时刻静止状态的时间长度
% IsSDOFAccelerationZero ： [1*N]
% 动态的是否0加速度判断结果，利用这个判断初始静止状态持续的时间长度。使用前先做一个较长时间（InitialIsStaticSmoothStepTime）的平滑
% 当出现连续 InitialIsStaticJudgeStepTime 长时间0加速度时判断为静止状态开始
% 当出现 非 连续 InitialIsStaticJudgeStepTime 长时间保持静止判断为静止状态结束
function [ initialStaticStart,initialStaticEnd ] = CalInitialStaticN( IsSDOFAccelerationZero,InitialIsStaticSmoothStepTime,InitialIsStaticJudgeStepTime,InitialStaticAbandonTime,frequency ) 
JudgeRate = 0.8 ;
SmoothStepN = fix( InitialIsStaticSmoothStepTime*frequency ) ;
IsSDOFAccelerationZero = SmoothJudgeData( IsSDOFAccelerationZero,SmoothStepN,0.6 ) ;  % 进行较大的步长平滑后再做判断

stepN = fix( InitialIsStaticJudgeStepTime*frequency ) ;
Nframes = length(IsSDOFAccelerationZero) ;
k=1;
%% find initialStaticStart
while k<Nframes
    if IsSDOFAccelerationZero(k)==1
        sum_k = sum( IsSDOFAccelerationZero(k:k+stepN-1) );
        if sum_k >= stepN*JudgeRate
            initialStaticStart = k ;        % 出现连续 InitialIsStaticJudgeStepTime 长时间0加速度时判断为静止状态开始
            break;
        end        
    end
    k = k+1 ;
end
%% find initialStaticStop
k = max( k,stepN );
while k<Nframes
    sum_k = sum( IsSDOFAccelerationZero(k:k+stepN-1) );
    if sum_k < stepN*JudgeRate
        initialStaticEnd = k ;    % 从第k个开始已经不是连续 InitialIsStaticJudgeStepTime 长时间保持0加速度
        break;
    end        
    k = k+1 ;
end

abandonNum = fix(InitialStaticAbandonTime*frequency);
if initialStaticEnd-initialStaticStart > abandonNum*5
    initialStaticStart = initialStaticStart+abandonNum ;
    initialStaticEnd = initialStaticEnd-abandonNum ;
end
disp('');
