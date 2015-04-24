%% xyz 2015.4.23

%% Judge is the IMU being state of acceleration==0
% IsSDOFAccelerationZero:N*1  利用 SDOF运动特点综合判断的
% IsSDOFAccelerationZero(k)==1: 判定为静止
% IsSDOFAccelerationZero(k)==1: 判定为非静止
function [ IsSDOFAccelerationZero,initialStaticStart,initialStaticEnd,IsSDOFAccelerationToHeartZero,IsAccNormZero ] = Judge0Acceleration( AHRSData,AHRSThreshod )
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
InitialIsStaticJudgeStepTime = AHRSThreshod.InitialIsStaticJudgeStepTime ;
InitialStaticAbandonTime = AHRSThreshod.InitialStaticAbandonTime ;
IsContinuousGyroNormZeroSmoothStepTime = AHRSThreshod.IsContinuousGyroNormZeroSmoothStepTime ;
DynamicIsStaticSmoothStepTime = AHRSThreshod.DynamicIsStaticSmoothStepTime ;
InitialIsStaticSmoothStepTime = AHRSThreshod.InitialIsStaticSmoothStepTime ;

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
% dbstop in CalInitialStaticN
[ initialStaticStart,initialStaticEnd ] = CalInitialStaticN( IsSDOFAccelerationZero,InitialIsStaticSmoothStepTime,InitialIsStaticJudgeStepTime,InitialStaticAbandonTime,frequency )  ;

%%%
DrawIsAccelerationZero( AHRSData,IsSDOFAccelerationZero,'IsSDOFAccelerationZero' ) ;

disp('');

 %% 平滑
 % JudgeData： [1*N]
 % stepN :平滑步长
% 1001 -> 1111
% 窗口： 前面 stepN 个 后面 stepN/3 个
% 当窗口中：
% 1）首尾minNum个是1， 2）窗口中的1个数大于一半(SmoothRate)。认为这一段均为1
function JudgeData = SmoothJudgeData( JudgeData,stepN,SmoothRate )
Nframes = length(JudgeData) ;
stepN = max(stepN,4);
minNum = fix(stepN/8) ;  % 首尾 minNum 个要求都是1
minNum = max( minNum,1 );
fillNum = 0;

for k=1:Nframes-stepN+1
    end_k = k+stepN-1 ;
    sumHead = sum( JudgeData( k:k+minNum-1 ) );
    sumEnd = sum( JudgeData( end_k-minNum+1:end_k ) );
    if sumHead == minNum && sumEnd == minNum   % 首尾是1
        staticSum = sum( JudgeData( k:end_k ) );
        if staticSum>=stepN*SmoothRate       % 窗口中的1个数大于一半
            JudgeData( k:end_k ) = ones( stepN,1 );
            fillNum = fillNum + stepN - staticSum ;
        end 
    end
end
display( sprintf('fillNum = %0.0f',fillNum) );

%% 判断某个点k领域内连续为1
% 1）第k-1点为1,第k点为1,第k+1点为1
% 2）第k点前的 frontN 个点有 70% 以上为1
% 3）第k点后的 laterN 个点有 70% 以上为1
%%% INput
% IsOnes: [1*N]或[N*1] 已经判断好的 是否为1的结果
function IsContinuousOnes = JudgeContinuousOnes( IsOnes,frontT,laterT,frequency,minRateFront,minRateLater )
Nframes = length( IsOnes );
IsContinuousOnes = zeros( size(IsOnes) );
frontN = max( fix(frontT * frequency),3) ;
laterN = max( fix(laterT * frequency),1) ;

for k = frontN+1:Nframes-laterN-1
    IsOnes_kSegment = IsOnes( k-frontN:k+laterN ) ;
    IsContinuousOnes(k) = JudgeContinuousOnes_One( IsOnes_kSegment,frontN+1,minRateFront,minRateLater );
end

%% 输入第k时刻邻域的 IsOnes_kSegment（是否为1） 判断 该点是否为连续 =1
% 待判断的点为 IsOnes_kSegment 的第k个点
function IsContinuousOnes_k = JudgeContinuousOnes_One( IsOnes_kSegment,k,minRateFront,minRateLater )

IsContinuousOnes_k = 0;
N = length(IsOnes_kSegment);
if IsOnes_kSegment(k)==1 
    if IsOnes_kSegment(k-1)==1 || IsOnes_kSegment(k+1) == 1  % 前后紧邻至少有一个满足
        IsOnes_Front = IsOnes_kSegment( 1:k-1 );
        IsOnes_Later = IsOnes_kSegment( k+1:N );
        if sum( IsOnes_Front ) >= ceil( minRateFront*(k-1) )       % 前面1的个数满足
           if  sum( IsOnes_Later ) >= fix( minRateLater*(N-k) )   % 后面1的个数满足
               IsContinuousOnes_k = 1 ;               
           end
        end
    end    
end


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
