%% xyz 2015.6.23

function DrawCompensated( compensateRate,trackedMakerPosition_InertialTime,InertialPosition,...
    InertialPositionNew,AccumulateCompensate,ineritalN )

figure( 'name',[num2str(compensateRate),'-INS VNS xy'] )
plot( trackedMakerPosition_InertialTime(1,:),trackedMakerPosition_InertialTime(2,:),'.b' )
hold on
plot( InertialPosition(1,:),InertialPosition(2,:),'.r' )
plot( InertialPositionNew(1,:),InertialPositionNew(2,:),'.g' )
legend( 'VNS','INS','INS\_New' )

figure( 'name',[num2str(compensateRate),'-INS VNS x'] )
plot( trackedMakerPosition_InertialTime(1,:),'.b' )
hold on
plot( InertialPosition(1,:),'r' )
plot( InertialPositionNew(1,:),'g' )
line( [0 ineritalN],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )

figure( 'name',[num2str(compensateRate),'-INS VNS y'] )
plot( trackedMakerPosition_InertialTime(2,:),'.b'  )
hold on
plot( InertialPosition(2,:),'r' )
plot( InertialPositionNew(2,:),'g' )
line( [0 ineritalN],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )

figure( 'name',[num2str(compensateRate),'-INS VNS z'] )
plot( trackedMakerPosition_InertialTime(3,:),'.b'  )
hold on
plot( InertialPosition(3,:),'r' )
plot( InertialPositionNew(3,:),'g' )
line( [0 ineritalN],[0 0],'color','k','lineStyle',':' );
legend( 'VNS','INS','INS\_New' )


figure('name','AccumulateCompensate')
subplot(2,1,1)
plot( AccumulateCompensate(1,:) )
hold on
line( [0 ineritalN],[0 0],'color','k','lineStyle',':' );
subplot(2,1,2)
plot( AccumulateCompensate(2,:) )
hold on
line( [0 ineritalN],[0 0],'color','k','lineStyle',':' );
