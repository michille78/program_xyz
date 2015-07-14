
VSN = size(trackedMakerPosition,2) ;
vsN1 = fix(VSN/2) ;
vsN2 = fix(VSN/4*3) ;
inN1 = VisionK_to_InertialK (vsN1);
inN2 = VisionK_to_InertialK (vsN2);
trackedMakerPosition1 = trackedMakerPosition( :,vsN1:vsN2 );

figure('name','Tracked XY')
plot( InertialMarkerPosition(1,inN1:inN2),InertialMarkerPosition(2,inN1:inN2),'ob' )
hold on
plot( trackedMakerPosition(1,vsN1:vsN2),trackedMakerPosition(2,vsN1:vsN2),'.r' )

figure('name','Z')
subplot(2,1,1)
 plot( inertialTime( inN1:inN2 ),InertialMarkerPosition( 3,inN1:inN2 ),'.b' )
hold on
 plot( visionTime( vsN1:vsN2 ),trackedMakerPosition( 3,vsN1:vsN2 ),'.r' )

ylabel('position z m/s')
legend('inertial','vision')
subplot(2,1,2)
plot( visionTime( vsN1:vsN2 ),trackedMakerVelocity( 3,vsN1:vsN2 ),'.r' );
ylabel('velocity z m/s')

figure('name','X')
subplot(2,1,1)
 plot( inertialTime( inN1:inN2 ),InertialMarkerPosition( 1,inN1:inN2 ),'.b' )
hold on
 plot( visionTime( vsN1:vsN2 ),trackedMakerPosition( 1,vsN1:vsN2 ),'.r' )

ylabel('position x m/s')
legend('inertial','vision')
subplot(2,1,2)
plot( visionTime( vsN1:vsN2 ),trackedMakerVelocity( 1,vsN1:vsN2 ),'.r' );
ylabel('velocity x m/s')

figure('name','y')
subplot(2,1,1)
 plot( inertialTime( inN1:inN2 ),InertialMarkerPosition( 2,inN1:inN2 ),'.b' )
hold on
 plot( visionTime( vsN1:vsN2 ),trackedMakerPosition( 2,vsN1:vsN2 ),'.r' )

ylabel('position z m/s')
legend('inertial','vision')
subplot(2,1,2)
plot( visionTime( vsN1:vsN2 ),trackedMakerVelocity( 2,vsN1:vsN2 ),'.r' );
ylabel('velocity y m/s')

return

e = 1e-3 ;
for k=1:VSN
    x = trackedMakerPosition(1,k) ;
    y = trackedMakerPosition(2,k) ;
   if    x <-0.0677 + e  && x > -0.0677 - e
       if  y <1.9 + e  && y > 1.9 - e
            k
       end
   end

end
