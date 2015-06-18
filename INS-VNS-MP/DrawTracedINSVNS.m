function DrawTracedINSVNS( TrackFlag,trackedMaker,InertialPosition,trackedMakerVelocity,InertialVelocity,INSVNSCalib_VS_k,INSVNSCalibSet )
global dataFolder  inertialFre visionFre

trackedMakerPosition = trackedMaker.trackedMakerPosition  ;
VSN = length(trackedMakerPosition) ;
visionTime = (1:VSN)/visionFre ;

% inertialTime = InertialData.time ;
InertialN = size(InertialPosition,2);
% InertialN0 = length(inertialTime);
% inertialTimeE= inertialTime ;
% for k=InertialN0+1:InertialN
%     inertialTimeE(k) = inertialTimeE(InertialN0);
% end

inertialTime = (1:InertialN)/inertialFre ;

for k=1:size(INSVNSCalib_VS_k,2)
    if isnan(INSVNSCalib_VS_k(1,k))
        calib_M = k-1 ;
        break;
    end
end

figure('name','trackFlag')
plot(TrackFlag,'.')
% plot(otherMakersTime,TrackFlag,'.')
% xlabel('time sec')

figure('name','Tracked XY')
plot( InertialPosition(1,:),InertialPosition(2,:),'ob' )
hold on

plot( trackedMakerPosition(1,:),trackedMakerPosition(2,:),'.r' )
% plot( InertialPositionNew(1,:),InertialPositionNew(2,:),'.g' )
% plot( InertialPositionNew(1,1),InertialPositionNew(2,1),'*k' )
legend( 'inertial','vision','Compensated' );

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( InertialPosition(1,calib_k_i),InertialPosition(2,calib_k_i),'.g' );
end

for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( trackedMakerPosition(1,calib_k_i),trackedMakerPosition(2,calib_k_i),'.k' );
end

plot( trackedMakerPosition(1,1),trackedMakerPosition(2,1),'*k' )
plot( InertialPosition(1,1),InertialPosition(2,1),'*k' )

xlabel('x m')
ylabel('y m')
saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

% %% compensate
% figure('name','compensate')
% plot( InertialPosition(1,:),InertialPosition(2,:),'ob' )
% hold on
% plot( InertialPositionNew(1,:),InertialPositionNew(2,:),'.g' )
% plot( InertialPositionNew(1,1),InertialPositionNew(2,1),'or' )
% plot( InertialPosition(1,1),InertialPosition(2,1),'or' )

figure('name','Tracked X')
subplot(2,1,1)
plot( inertialTime,InertialPosition(1,:),'.b' )
hold on
plot( visionTime,trackedMakerPosition(1,:),'.r' ) 
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerPosition(1,calib_k_i),'.k' );
end
ylabel('position x m/s')
legend('inertial','vision','calibData')

subplot(2,1,2)
hold on 
plot( visionTime,trackedMakerVelocity(1,:),'.r' );
plot( inertialTime,InertialVelocity(1,:),'.b' );
ylabel('velocity x m/s')
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(1,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );

figure('name','Tracked Y')
subplot(2,1,1)
plot( inertialTime,InertialPosition(2,:),'.b' )
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
plot( inertialTime,InertialVelocity(2,:),'.b' );
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(2,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );


figure('name','Tracked Z')
subplot(2,1,1)
plot( inertialTime,InertialPosition(3,:),'.b' )
hold on
plot( visionTime,trackedMakerPosition(3,:),'.r' )
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
plot( inertialTime,InertialVelocity(3,:),'.b' );
for i=1:calib_M
    calib_k_i = INSVNSCalib_VS_k(1,i):INSVNSCalib_VS_k(2,i) ;
    plot( visionTime(calib_k_i),trackedMakerVelocity(3,calib_k_i),'.k' );
end

saveas( gcf,sprintf('%s\\%s.fig',dataFolder,get(gcf,'name')) );
saveas( gcf,sprintf('%s\\%s.jpg',dataFolder,get(gcf,'name')) );


figure('name','trackedMakerVelocity xyNorm')
subplot(2,1,1)
plot( visionTime,trackedMakerVelocity(4,:),'.r' );
hold on
plot( inertialTime,InertialVelocity(4,:),'.b' );
ylabel( 'xy velocity normest' );

temp = INSVNSCalibSet.MinVXY_Calib ;
line( [visionTime(1) visionTime(VSN)],[temp temp],'color','r' )

subplot(2,1,2)
plot( visionTime,trackedMakerVelocity(5,:)*180/pi,'.r' );
hold on
plot( inertialTime,InertialVelocity(5,:)*180/pi,'.b' );
ylabel( 'xy velocity angle' );
xlabel('time sec')
legend('vision','inertial')


function InertialPosition = GetInertialMarkerPosition( InertialData,MarkerSet )
switch MarkerSet
    case 'Head' 
        InertialPosition = InertialData.HeadPosition ;
    case 'Hip'
        InertialPosition = InertialData.HipPosition ;
end