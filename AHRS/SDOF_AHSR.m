%% xyz 2015.4.21



function SDOF_AHSR(  )

clc
clear all
close all

dataFolder = 'E:\data_xyz_noitom\AHRS Data\ahrs_raw_data_4.20\Xu';
refDataFolder = [dataFolder,'\ref'];
%  dataFolder = 'E:\data_xyz_noitom\AHRS Data\staticData_4.21_250HZ';
% dataFolder = 'E:\data_xyz_noitom\AHRS Data\staticData_4.21';
dataName = 'ahrs1' ;
% dataName = 'static3';

AHRSData = importdata( [ dataFolder,'\',dataName,'.mat' ] );
Nframes  = length(AHRSData.accNorm);
AHRSData.Nframes = Nframes ;

%% 转台参考数据：第二列是测量值，°，1000HZ
AHRSRefData_Raw = importdata( [ refDataFolder,'\',dataName,'.dat' ] );
RefRotateAngle = ( AHRSRefData_Raw(:,2)-30 ) *pi/180 ;
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
%%
RotateAngle = SDOF_AHSR_One( AHRSData,AHRSRefData ) ;

%% 
function RotateAngle = SDOF_AHSR_One( AHRSData,AHRSRefData )

%% load data
% quaternion = AHRSData.quaternion ;
gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = Nframes/frequency ;

%% static time judge
%%% the threshold value to judge is being static
AHRSThreshod.GyroNormZeroThreshod = 0.7 *pi/180 ;       % 0.5 °/s  角速度为0判断下限
AHRSThreshod.AccNormZeroThreshod = 2/1000 ;           % 3mg  加速度模为0判断下限
AHRSThreshod.InitialIsStaticJudgeStepTime = 0.1 ;       % 初始静止判断步长时间
AHRSThreshod.InitialIsStaticSmoothStepTime = 0.5 ;     % 动态静止平滑步长时间
AHRSThreshod.DynamicIsStaticSmoothStepTime = 0.05 ;     % 动态静止平滑步长时间
AHRSThreshod.InitialStaticAbandonTime = 0.05 ;          % 初始静止时段首尾抛弃时长
AHRSThreshod.RoateVectorCalMinAngle = 10*pi/180;        % 旋转轴计算数据选择时，旋转角度下限
AHRSThreshod.RoateVectorCalTime  = 15 ;                 % 从静止开始多次时间的数据用于转轴计算。之后就要求输出角度。
AHRSThreshod.GyroContinuousZeroTimeThreshod = 0.3 ;     % 角速度变化率=0判断：角速度为0保持时间
AHRSThreshod.GyroContinuousZeroMinRate = 0.7 ;          % 角速度变化率=0判断：角速度为0在邻域内保持的比例
AHRSThreshod.IsContinuousGyroNormZeroSmoothStepTime = 0.2  ; % 角速度变化率为0判断结果平滑步长 （可以稍微长点）

% dbstop in Judge0Acceleration
[ IsSDOFAccelerationZero,initialStaticStart,initialStaticEnd,IsSDOFAccelerationToHeartZero,IsAccNormZero ] = Judge0Acceleration( AHRSData,AHRSThreshod ) ;
AHRSStateResult.IsSDOFAccelerationZero = IsSDOFAccelerationZero ;
initialStaticTime = ( initialStaticEnd-initialStaticStart )/frequency ;
if initialStaticTime < 1
   errordlg(sprintf('初始静止状态时长=%0.2f sec， 太短！',initialStaticTime)); 
end
%% calculate the rotate vector
[ pitch,roll,Qnb ] = Acc2PitchRoll( acc ) ;
%%% initial static acc  gyro  : r frame
accStatic = acc( initialStaticStart:initialStaticEnd,: ) ;
acc_r = mean( accStatic,1 );
gyroStatic = gyro( initialStaticStart:initialStaticEnd,: ) ;
gyro_r = mean( gyroStatic,1 );
[ pitch_r,roll_r,Qwr ] = Acc2PitchRoll( acc_r ) ;

pitch_d = pitch*180/pi;
roll_d = roll*180/pi;

pitch_r_d = pitch_r*180/pi;
roll_r_d = roll_r*180/pi;

%%% 选择满足转轴解算的数据
[ Qnb_RCD,Qwr_RCD ] = SelectRotateVectorCalcualteData( Qnb,Qwr,AHRSThreshod,AHRSStateResult,frequency ) ;
%%% 转轴解算
Ypr = CalculateRotateVector_Acc(Qnb_RCD,Qwr_RCD ) ;

%% 纯加计转角解算
RotateAngle = CalculateRotateAngle_Acc( Qnb,Qwr,Ypr ) ;



%%
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

figure
plot(RotateAngleErr*180/pi)

figure
plot( RotateAngleNew(1:N)*180/pi,'b' )
hold on
plot( RefRotateAngle(1:N)*180/pi,'r' )
% plot(RotateAngleErr*180/pi,'k,')

legend('RotateAngleNew','RefRotateAngle')
%%
return
staticTime = 18 ;
staticNum = frequency*staticTime ;   % 3 sec

% DrawAHRSData( AHRSData,'raw' ) ;

meam_gyro = GetStaticStateFeature( AHRSData,staticNum ) ;
if staticTime>15
     gyroNew = gyro - repmat( meam_gyro,Nframes,1 );
     gyroNormNew = gyroNorm ;
 for k=1:Nframes
     gyroNormNew(k,1) = normest( gyroNew(k,:) );
 end
     AHRSDataNew = AHRSData ;
     AHRSDataNew.gyro = gyroNew ;
     AHRSDataNew.gyroNorm = gyroNormNew ;
     meam_gyro_new = GetStaticStateFeature( AHRSDataNew,staticNum ) ;
else
    AHRSDataNew = AHRSData;
end

%% 纯加计解算转动角度
function RotateAngle = CalculateRotateAngle_Acc( Qnb,Qwr,Ypr ) 

Nframes = length( Qnb ) ;
RotateAngle = zeros( 1,Nframes );
for k=1:Nframes
    A = CalculateA_One( Qnb(:,k),Qwr )  ;
    temp = -A(2,2:4)*Ypr / A(2,1) ;
    RotateAngle(k) = acot( temp )*2;
end

figure
plot(RotateAngle*180/pi)
ylabel('RotateAngle');

%% select data be suitable for rotate vector calculating

function [ Qnb_RCD,Qwr_RCD ] = SelectRotateVectorCalcualteData( Qnb,Qwr,AHRSThreshod,AHRSStateResult,frequency )
RoateVectorCalTime = AHRSThreshod.RoateVectorCalTime ; 
RoateVectorCalN = RoateVectorCalTime*frequency ;
Qnb = Qnb( :,1:RoateVectorCalN ) ;

Qrw = Qinv( Qwr );
N = size(Qnb,2);
Qrb_false = QuaternionMultiply( repmat(Qrw,1,N),Qnb );
angle_false = GetQAngle( Qrb_false ) ;
RoateVectorCalMinAngle = AHRSThreshod.RoateVectorCalMinAngle ;
IsAngleBig =  angle_false > RoateVectorCalMinAngle | angle_false < -RoateVectorCalMinAngle ;
IsSDOFAccelerationZero = AHRSStateResult.IsSDOFAccelerationZero ;
IsSDOFAccelerationZero = IsSDOFAccelerationZero(1:RoateVectorCalN);
IsAngleBigStatic = IsAngleBig & IsSDOFAccelerationZero' ;
 IsAngleBigStatic = IsAngleBig ;

Qnb_RCD = Qnb( :,IsAngleBigStatic );
Qwr_RCD = Qwr;

%% check
angle_false_RCD = angle_false(IsAngleBigStatic);

time = 1:N;
time = time(IsAngleBigStatic);

figure
plot( angle_false*180/pi )
hold on
plot( time,angle_false_RCD*180/pi,'r.' )

disp('')

 
%% Calculate Rotate Vector only by Acc when satic

function Ypr = CalculateRotateVector_Acc( Qnb,Qwr )
Nframes = size(Qnb,2);

D = CalculateD( Qnb,Qwr ) ;

DTD = D'*D ;
[ V,D ] = eig( DTD );
eigValue = diag(D);
[ minEigValue,minEig_k ] = min( eigValue );
X = V( :,minEig_k );
Ypr = X( 1:3 );
Ypr = Ypr/normest(Ypr);

%% check
DX = D*X ;
DTDX = DTD*V( :,1 ) ;
return

resolutionNum = 3+Nframes - rank(DTD) ;     % the number of Rotate Vector equation resolutions
if resolutionNum == 1
    
elseif resolutionNum>1
    errordlg('Rotate Vector Equation more than 1 solution! ');
elseif resolutionNum<1
    errordlg('Rotate Vector Equation NO solution! ');
end
Ypr = ones(3,1) ;

function D = CalculateD( Qnb,Qwr )
Nframes = size(Qnb,2);
D = zeros( 2*Nframes,3+Nframes );
for i=1:Nframes
    Ai = CalculateA_One( Qnb(:,i),Qwr ) ;
    As_i = Ai(2:3,1);
    Av_i = Ai(2:3,2:4);
    D( 2*i-1:2*i,1:3 ) = Av_i ;
    D( 2*i-1:2*i,3+i ) = As_i ;
end
disp('')


function A = CalculateA_One( Qnb,Qwr ) 
if length(Qnb)==4
    Qbn = [ Qnb(1);-Qnb(2:4) ] ;
    LQMwr = LeftQMultiplyMatrix( Qwr ) ;
    RQMbn = RightQMultiplyMatrix( Qbn ) ;
    A = RQMbn * LQMwr ;
else
    A = NaN;
end

       