%% xyz 2015 5.25

%% otherMakers
% otherMakers(k).frequency
% otherMakers(k).Position
% otherMakers(k).otherMakersN
%　otherMakers(k).time

%% InertialData
% InertialData.frequency (k)
% InertialData.time (k)
% InertialData.HipQuaternion(k)
% InertialData.HipPosition (k)  [3*N]
% InertialData.HeadQuaternion (k)
% InertialData.HeadPosition (k)
% InertialData.BodyDirection(k)
% InertialData.DataStyle(k)
% InertialData.HeadHipLength(k)


function OptitrackInertialFusion(  )
clc
clear all
close all
global dataFolder otherMakersTime
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head6';

%% load data
CalStruct = ReadCalData ( dataFolder,'CalData_Avatar00' ) ;

BVHStruct = readBVHData ( dataFolder,'BVHData_Avatar00' );
HipDisplacement = GetJointDisplacement( BVHStruct,'ROOT_Hips' );
HipDisplacement = Hip_NUE2NED( HipDisplacement ) ;
HeadDisplacement = GetJointDisplacement( BVHStruct,'Head' );

InertialData = importdata( [dataFolder,'\InertialData.mat'] );
otherMakers = importdata( [dataFolder,'\otherMakers.mat'] );


% 用 CalStruct 替代 InertialData
InertialData.HipPosition = CalStruct.ROOT_Hips.X ; 
InertialData.HeadPosition = CalStruct.Head.X ; 


%% otherMakers 预处理
otherMakers = PreProcess( otherMakers,InertialData.BodyDirection );

[ otherMakersTime,otherMakersN ] = Get_otherMakersData( otherMakers );

%       DrawAllINSVNS( otherMakers,InertialData ) ;
%    return;

%% 单马克点跟踪
%      dbstop in GetRightOtherMaker
  
 [ trackedMakerPosition,trackedMakerPosition_InertialTime,trackedMakerVelocity,INSVNSCalib_VS_k,InertialPosition ] = GetRightOtherMaker( otherMakers,InertialData ) ;
trackedMaker.trackedMakerPosition = trackedMakerPosition ;
trackedMaker.time = otherMakersTime ;
trackedMaker.MarkerSet = otherMakers(1).MarkerSet ;

 
N = length(trackedMakerPosition) ;
for k=1:N
    otherMakers(k).trackedMakerPosition = trackedMakerPosition(k) ;
end
for k=1:N
    i = N-k+1;
    if ~isnan(trackedMakerPosition(i))
        trackedMaker.LastMakerPosition = trackedMakerPosition(:,i);
        trackedMaker.LastMakerTime = otherMakers(i).time ;
        break;
    end
end
save([dataFolder,'\trackedMaker.mat'],'trackedMaker')

%% 位置补偿
compensateRate = 0.1 ;
[ HipDisplacementNew,InertialPositionNew ] = VNSCompensateINS...
    ( compensateRate,trackedMakerPosition_InertialTime,HipDisplacement,InertialPosition ) ;
HipDisplacementNew_NUE = Hip_NED2NUE( HipDisplacementNew ) ;
% 更新 BVHStruct
ROOT_Hips = BVHStruct.JointData.ROOT_Hips;
ROOT_Hips(:,1:3) = HipDisplacementNew_NUE;
BVHStruct.JointData.ROOT_Hips = ROOT_Hips ;
BVHStruct = UpdateBVHStruct( BVHStruct ) ;
WriteBVH( BVHStruct,dataFolder,['CompensatedBVH-',num2str(compensateRate)],1 ) ;

DrawTracedINSVNS( trackedMaker,InertialData,trackedMakerVelocity,INSVNSCalib_VS_k,InertialPositionNew ) ;

disp('OptitrackInertialFusion OK')

function Draw( CalStruct,InertialData )

HipPosition = CalStruct.ROOT_Hips.X ;
HeadPosition = CalStruct.Head.X ;
InertialP =InertialData.HeadPosition;

figure()
plot( HipPosition(1,:) )
hold on
plot( InertialP(1,:),'r' )

figure()
plot( HipPosition(2,:) )
hold on
plot( InertialP(2,:),'r' )

%% 位移补偿
%%% Input
% trackedMakerPosition_InertialTime ： 马克点光学位置，按惯性时序存储
% HipDisplacement ： Hip在北东地下的位置 （由 BVH 得到）
% InertialPosition： 惯性系与马克点安装位置对应关节的位置（安装在头上时，InertialPosition 为惯性头的位置）
%%% Output
% InertialPositionNew ： 补偿后惯性的位置
% HipDisplacementNew： 补偿后Hip的位置
function [ HipDisplacementNew,InertialPositionNew ] = VNSCompensateINS...
( compensateRate,trackedMakerPosition_InertialTime,HipDisplacement,InertialPosition )

%% BVH 读取的个数 N_BVH 可能会比 N1 多几个
N_BVH = size(HipDisplacement,2);
N1 = size(trackedMakerPosition_InertialTime,2);
for k=N1+1:N_BVH
    trackedMakerPosition_InertialTime(:,k) = trackedMakerPosition_InertialTime(:,N1);
    InertialPosition(:,k) = InertialPosition(:,N1);
end

%% 先补偿 InertialPositionNew
InertialPositionNew = NaN(3,N_BVH);
InertialPositionNew(3,:) = InertialPosition(3,:); % 高度不补偿
InertialPositionNew(1:2,1) = InertialPosition(1:2,1) ;  % 起始点选择惯性
InertialPositionCompensate = zeros(3,N_BVH); % 每一步的累积位移补偿量 记录
InertialErr = zeros(2,N_BVH);       % 实时记录补偿后的误差
StepCompensate = zeros(2,N_BVH);  % 单步补偿量
for k=2:N_BVH
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
HipDisplacementNew = HipDisplacement ;
for k=1:N_BVH
    
    HipDisplacementNew( 1:2,k ) = HipDisplacement(1:2,k) + InertialPositionCompensate(1:2,k) ;
end


figure('name','InertialErr - StepCompensate')
subplot( 2,1,1 )
plot( InertialErr(1,:),'.b' )
hold on
plot( StepCompensate(1,:),'.r' )
ylabel('x')
subplot( 2,1,2 )
plot( InertialErr(2,:),'.b' )
hold on
plot( StepCompensate(2,:),'.r' )
ylabel('y')


figure( 'name',[num2str(compensateRate),'-INS VNS Hip x'] )
plot( trackedMakerPosition_InertialTime(1,:),'.b' )
hold on
plot( InertialPosition(1,:),'r' )
plot( InertialPositionNew(1,:),'g' )
line( [0 N_BVH],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )

figure( 'name',[num2str(compensateRate),'-INS VNS Hip y'] )
plot( trackedMakerPosition_InertialTime(2,:),'.b'  )
hold on
plot( InertialPosition(2,:),'r' )
plot( InertialPositionNew(2,:),'g' )
line( [0 N_BVH],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )

figure( 'name',[num2str(compensateRate),'-INS VNS Hip z'] )
plot( trackedMakerPosition_InertialTime(3,:),'.b'  )
hold on
plot( InertialPosition(3,:),'r' )
plot( InertialPositionNew(3,:),'g' )
line( [0 N_BVH],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )

figure('name','InertialPositionCompensate')
subplot(2,1,1)
plot( InertialPositionCompensate(1,:) )
hold on
line( [0 N_BVH],[0 0],'color','k','lineStyle',':' );
subplot(2,1,2)
plot( InertialPositionCompensate(2,:) )
hold on
line( [0 N_BVH],[0 0],'color','k','lineStyle',':' );
return

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

function otherMakers = PreProcess( otherMakers,BodyDirection  )
%% 先转到北东地的同名坐标系
Cv_r1 = [ 0 0 1; -1 0 0; 0 -1 0 ];
Cr_r1 = BodyDirection2Cr_r1( BodyDirection );
Cvr = Cr_r1' * Cv_r1 ;


%% 先转到 惯性系 的北东地，并以视觉的初始点为原点（不处理高度）
N = length( otherMakers );
Position_1 = otherMakers(1).Position(:,1) ;
Position_1(2) = 0 ;      % 不处理高度方向

for k=1:N
    Position_k = otherMakers(k).Position ;
    if ~isempty(Position_k)
        m = size(Position_k,2);
        Position_1_m = repmat(Position_1,1,m);
        
        Position_k_new = Cvr*(Position_k-Position_1_m) ;
        otherMakers(k).Position = Position_k_new; 
    else
        otherMakers(k).Position = []; 
    end
end


%% 根据 BodyDirection 求姿态
% 符号有待验证！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
function Cr_r1 = BodyDirection2Cr_r1( BodyDirection )
thita = acos( [1 0 0]*BodyDirection / normest(BodyDirection)   );
if BodyDirection(2)<0
    thita = -thita ;
end
Cr_r1 = RotateZ( thita ) ;


function DrawTracedINSVNS( trackedMaker,InertialData,trackedMakerVelocity,INSVNSCalib_VS_k,InertialPositionNew )
global dataFolder INSVNSCalibSet inertialFre

trackedMakerPosition = trackedMaker.trackedMakerPosition  ;
visionTime = trackedMaker.time  ;

InertialMarkerPosition = GetInertialMarkerPosition( InertialData,trackedMaker.MarkerSet ) ;
% inertialTime = InertialData.time ;
InertialN = size(InertialMarkerPosition,2);
% InertialN0 = length(inertialTime);
% inertialTimeE= inertialTime ;
% for k=InertialN0+1:InertialN
%     inertialTimeE(k) = inertialTimeE(InertialN0);
% end

inertialTime = (1:InertialN)/inertialFre ;

VSN = length(visionTime) ;
calib_M = size(INSVNSCalib_VS_k,2);

figure('name','Tracked XY')
plot( InertialMarkerPosition(1,:),InertialMarkerPosition(2,:),'ob' )
hold on

plot( trackedMakerPosition(1,:),trackedMakerPosition(2,:),'.r' )
% plot( InertialPositionNew(1,:),InertialPositionNew(2,:),'.g' )
% plot( InertialPositionNew(1,1),InertialPositionNew(2,1),'*k' )
legend( 'inertial','vision','Compensated' );

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( InertialMarkerPosition(1,calib_k_i),InertialMarkerPosition(2,calib_k_i),'.g' );
end

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( trackedMakerPosition(1,calib_k_i),trackedMakerPosition(2,calib_k_i),'.k' );
end

plot( trackedMakerPosition(1,1),trackedMakerPosition(2,1),'*k' )
plot( InertialMarkerPosition(1,1),InertialMarkerPosition(2,1),'*k' )

xlabel('x m')
ylabel('y m')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

%% compensate
figure('name','compensate')
plot( InertialMarkerPosition(1,:),InertialMarkerPosition(2,:),'ob' )
hold on
plot( InertialPositionNew(1,:),InertialPositionNew(2,:),'.g' )
plot( InertialPositionNew(1,1),InertialPositionNew(2,1),'or' )
plot( InertialMarkerPosition(1,1),InertialMarkerPosition(2,1),'or' )

figure('name','Tracked X')
subplot(2,1,1)
plot( inertialTime,InertialMarkerPosition(1,:),'.b' )
hold on
plot( visionTime,trackedMakerPosition(1,:),'.r' ) 
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerPosition(1,calib_k_i),'.k' );
end
ylabel('position x m/s')
legend('inertial','vision','calibData')

subplot(2,1,2)
plot( visionTime,trackedMakerVelocity(1,:),'.r' );
ylabel('velocity x m/s')
hold on 
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(1,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

figure('name','Tracked Y')
subplot(2,1,1)
plot( inertialTime,InertialMarkerPosition(2,:),'.b' )
hold on
plot( visionTime,trackedMakerPosition(2,:),'.r' )
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerPosition(2,calib_k_i),'.k' );
end
ylabel('position y m/s')
legend('inertial','vision','calibData')
subplot(2,1,2)
plot( visionTime,trackedMakerVelocity(2,:),'.r' );
ylabel('velocity y m/s')
hold on 
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(2,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );


figure('name','Tracked Z')
subplot(2,1,1)
 plot( inertialTime,InertialMarkerPosition(3,:),'.b' )
% plot( InertialMarkerPosition(3,:),'.b' )
hold on
 plot( visionTime,trackedMakerPosition(3,:),'.r' )
% plot( trackedMakerPosition(3,:),'.r' )
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerPosition(3,calib_k_i),'.k' );
end
ylabel('position z m/s')
legend('inertial','vision')
subplot(2,1,2)
plot( visionTime,trackedMakerVelocity(3,:),'.r' );
ylabel('velocity z m/s')
hold on 
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(3,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );


figure('name','trackedMakerVelocity xyNorm')
subplot(2,1,1)
plot( visionTime,trackedMakerVelocity(4,:),'.r' );
ylabel( 'xy velocity normest' );

temp = INSVNSCalibSet.MinVXY_Calib ;
line( [visionTime(1) visionTime(VSN)],[temp temp],'color','r' )

subplot(2,1,2)
plot( visionTime,trackedMakerVelocity(5,:)*180/pi,'.b' );
ylabel( 'xy velocity angle' );

xlabel('time sec')


function DrawAllINSVNS( otherMakers,InertialData )
global dataFolder

% delete(gcp)
% parpool(4)

MarkerSet= otherMakers(1).MarkerSet ;
inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;
[ otherMakersTime,otherMakersN ] = Get_otherMakersData( otherMakers );
vsN = length(otherMakersTime);
InertialMarkerPosition = GetInertialMarkerPosition( InertialData,MarkerSet ) ;

MarkerPlot = { '.r','.g','.k','.y','m' };

figure('name','All Marker Inertial XY')
plot( InertialMarkerPosition(1,:),InertialMarkerPosition(2,:),'.b' )
hold on
plot( InertialMarkerPosition(1,1),InertialMarkerPosition(2,1),'*k','MarkerSize',13 );

for  k=1:vsN
    for i=1:otherMakersN(k)
        plot( otherMakers(k).Position(1,i),otherMakers(k).Position(2,i),MarkerPlot{i} );        
    end
    if k==1
        for i=1:otherMakersN(k)
            plot( otherMakers(k).Position(1,i),otherMakers(k).Position(2,i),'*k','MarkerSize',13 );    
        end
    end
end
xlabel('time sec')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

figure('name','All Marker Inertial X')
plot( inertialTime,InertialMarkerPosition(1,:),'.b' )
hold on
for  k=1:vsN
    for i=1:otherMakersN(k)
        plot( otherMakersTime(k),otherMakers(k).Position(1,i),MarkerPlot{i} );
    end
end
xlabel('time sec')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

figure('name','All Marker Inertial Y')
plot( inertialTime,InertialMarkerPosition(2,:),'.b' )
hold on
for  k=1:vsN
    for i=1:otherMakersN(k)
        plot( otherMakersTime(k),otherMakers(k).Position(2,i),MarkerPlot{i});
    end
end
xlabel('time sec')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

figure('name','All Marker Inertial Z')
plot( inertialTime,InertialMarkerPosition(3,:),'.b' )
hold on
for  k=1:vsN
    for i=1:otherMakersN(k)
        plot( otherMakersTime(k),otherMakers(k).Position(3,i),MarkerPlot{i} );
    end
end
xlabel('time sec')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

% delete(gcp)
disp('draw OK')


function InertialMarkerPosition = GetInertialMarkerPosition( InertialData,MarkerSet )
switch MarkerSet
    case 'Head' 
        InertialMarkerPosition = InertialData.HeadPosition ;
    case 'Hip'
        InertialMarkerPosition = InertialData.HipPosition ;
end

