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
% InertialData.HipPosition (k)
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
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head8';
InertialData = importdata( [dataFolder,'\InertialData.mat'] );
otherMakers = importdata( [dataFolder,'\otherMakers.mat'] );

%% otherMakers 预处理
otherMakers = PreProcess( otherMakers,InertialData.BodyDirection );

[ otherMakersTime,otherMakersN ] = Get_otherMakersData( otherMakers );

%       DrawAllINSVNS( otherMakers,InertialData ) ;
%    return;

%% 但马克点跟踪
   dbstop in GetRightOtherMaker
  
 [ trackedMakerPosition,trackedMakerVelocity,INSVNSCalib_VS_k ] = GetRightOtherMaker( otherMakers,InertialData ) ;
trackedMaker.trackedMakerPosition = trackedMakerPosition ;
trackedMaker.time = otherMakersTime ;
trackedMaker.MarkerSet = otherMakers(1).MarkerSet ;

DrawTracedINSVNS( trackedMaker,InertialData,trackedMakerVelocity,INSVNSCalib_VS_k ) ;

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



disp('OptitrackInertialFusion OK')


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


function DrawTracedINSVNS( trackedMaker,InertialData,trackedMakerVelocity,INSVNSCalib_VS_k )
global dataFolder INSVNSCalibSet

trackedMakerPosition = trackedMaker.trackedMakerPosition  ;
visionTime = trackedMaker.time  ;

InertialMarkerPosition = GetInertialMarkerPosition( InertialData,trackedMaker.MarkerSet ) ;
inertialTime = InertialData.time ;

VSN = length(visionTime) ;
calib_M = size(INSVNSCalib_VS_k,2);

figure('name','Tracked XY')

hold on
plot( InertialMarkerPosition(1,:),InertialMarkerPosition(2,:),'.b' )
plot( trackedMakerPosition(1,:),trackedMakerPosition(2,:),'.r' )
legend( 'inertial','vision' );

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( InertialMarkerPosition(1,calib_k_i),InertialMarkerPosition(2,calib_k_i),'.g' );
end

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( trackedMakerPosition(1,calib_k_i),trackedMakerPosition(2,calib_k_i),'.k' );
end

plot( trackedMakerPosition(1,1),trackedMakerPosition(2,1),'or' )
plot( InertialMarkerPosition(1,1),InertialMarkerPosition(2,1),'ob' )

xlabel('time sec')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );


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

