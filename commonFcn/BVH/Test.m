

 dataName = 'BVHData';
 dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head1';


BVHStruct = readBVHData ( dataFolder,dataName ) ;

DrawAllBVHJoint( BVHStruct );

IsWriteDisp = 0;
WriteBVH( BVHStruct,dataFolder,'BVHDataWritten',IsWriteDisp );




disp('');

