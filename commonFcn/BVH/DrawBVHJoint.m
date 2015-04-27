%% xyz 2015.4.9

function fh = DrawBVHJoint( BVHStruct,JointName )
data = BVHStruct.data ;
Frame_Time = BVHStruct.Frame_Time ;
RotationOrder = GetJointRotationOrder( BVHStruct,JointName );
if isempty(RotationOrder)
   fh = [];
   return;
end
JointData = eval( sprintf( 'data.%s ;',JointName ) );
Nframes = size( JointData,1 );

Time  = ( 0:(Nframes-1) )*Frame_Time ;

fh = figure( 'name',JointName );

subplot( 3,1,1 );
plot( Time,JointData(:,1),'r' );
ylabel( RotationOrder(1) );
title( [ JointName '  /degree' ] )
subplot( 3,1,2 );
plot( Time,JointData(:,2),'g' );
ylabel( RotationOrder(2) );
subplot( 3,1,3 );
plot( Time,JointData(:,3),'b' );
ylabel( RotationOrder(3) );

xlabel( 'time/s' );

