function Finger(  )

clc
clear all
close all
%% parameters to set
dataName = 'Finger1';  % ÎÞºó×º .bvh
% dataFolder = 'E:\data_xyz_noitom\BVHTransformTestData\BVHtransform';
dataFolder = 'E:\data_xyz_noitom\Finger\Finger1';
rotateOrder_BVH = 'YXZ' ;

global resPath isCheck
isCheck = 0 ;
resPath = [dataFolder,'\',dataName,'_result'];
if ~isdir(resPath)
    mkdir(resPath)
else
    delete([resPath,'\*'])
end
%% load data
BVHFilePath = [dataFolder,'\',dataName,'.bvh'] ;
newBVHFilePath = [dataFolder,'\new_',dataName,'.bvh'] ;
%    dbstop in readBVHData
BVHStruct = readBVHData ( dataFolder,dataName ) ;
BVHFormat = BVHStruct.BVHFormat ;
data = BVHStruct.data ;
RightHandMiddle1 = data.RightHandMiddle1 ;

% fh_RightHandMiddle = DrawBVHJoint( BVHStruct,'RightInHandIndex' );
% fh_RightHandMiddle1 = DrawBVHJoint( BVHStruct,'RightHandIndex1' );
% fh_RightHandMiddle2 = DrawBVHJoint( BVHStruct,'RightHandIndex2' );
% fh_RightHandMiddle3 = DrawBVHJoint( BVHStruct,'RightHandIndex3' );    
    
% fh_RightHandMiddle = DrawBVHJoint( BVHStruct,'RightInHandMiddle' );
fh_RightHandMiddle1 = DrawBVHJoint( BVHStruct,'RightHandMiddle1' );
% fh_RightHandMiddle2 = DrawBVHJoint( BVHStruct,'RightHandMiddle2' );
% fh_RightHandMiddle3 = DrawBVHJoint( BVHStruct,'RightHandMiddle3' );


RightHandMiddle1_ZXY = ChangeEulerRotate( RightHandMiddle1 *pi/180,'YXZ','ZXY',[1,1,1],[1,1,1] ) *180/pi ;
Draw( RightHandMiddle1_ZXY, 'RightHandMiddle1_ZXY','ZXY' ) ;

RightHandMiddle1_ZYX = ChangeEulerRotate( RightHandMiddle1 *pi/180,'YXZ','ZYX',[1,1,1],[1,1,1] ) *180/pi ;
Draw( RightHandMiddle1_ZYX, 'RightHandMiddle1_ZYX','ZYX' ) ;

C_RightHandMiddle1  = Euler2C( RightHandMiddle1 *pi/180,'YXZ' );
RightHandMiddle1_ZYX_New = C2Euler( C_RightHandMiddle1,'ZYX',[1,1,1],1 ) *180/pi;
Draw( RightHandMiddle1_ZYX_New, 'RightHandMiddle1_ZYX_New','ZYX' ) ;

RightHandMiddle1_ZXY_New = C2Euler( C_RightHandMiddle1,'ZXY',[1,1,1],-1 ) *180/pi;
Draw( RightHandMiddle1_ZXY_New, 'RightHandMiddle1_ZXY_New','ZXY' ) ;

RightHandMiddle1_new = Handle_RightHandMiddle1( RightHandMiddle1,'YXZ' ) ;
Draw( RightHandMiddle1_new, 'RightHandMiddle1_YXZ_2','YXZ' ) ;

disp('OK');



function fh = Draw( JointData,JointName,RotationOrder )
fh = figure( 'name',JointName );

Time = 1:size(JointData,1);

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