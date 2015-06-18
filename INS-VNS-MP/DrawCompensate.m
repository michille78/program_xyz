function DrawCompensate( InertialErr,StepCompensate,trackedMakerPosition_InertialTime,...
    InertialPosition,InertialPositionNew,InertialPositionCompensate,compensateRate,N_BVH )

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