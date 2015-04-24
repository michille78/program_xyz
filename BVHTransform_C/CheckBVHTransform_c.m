function  CheckBVHTransform_c(  )
clc
clear all
close all
%% parameters to set
dataName = 'NaoTestBVH3';  % ÎÞºó×º .bvh
% dataFolder = 'E:\data_xyz_noitom\BVHTransformTestData\BVHtransform';
dataFolder = 'E:\data_xyz_noitom\NaoTestBVH\NaoTestBVH3';
rotateOrder_BVH = 'YXZ' ;

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

% dbstop in CutBVHStruct
BVHStruct = CutBVHStruct( BVHStruct,13 ) ;

WriteBVH( BVHStruct,dataFolder,[dataName,'_NoDisplacement_CTest'] );


BVHData = BVHStruct.BVHData ;
BVHFormat = BVHStruct.BVHFormat ;
BVHHeadStr = BVHStruct.BVHHeadStr ;
% dbstop in BVHTransform_c
% NaoData = BVHTransform_c( BVHData ) ;
Nframes = size(BVHData,1);
NaoData = zeros(25,Nframes);
ROOT_Hips_RotationNew = zeros(Nframes,3);
 [ NaoData,ROOT_Hips_RotationNew ] = BVHTransform_c( BVHData ) ; % degree
for k=1:Nframes
    [ NaoData(:,k),ROOT_Hips_RotationNew(k,:) ] = BVHTransform_c( BVHData(k,:) ) ; % degree
% NaoData_mex = BVHTransform_c_mex( BVHData ) ;

% NaoData_err = NaoData-NaoData_mex ;
end
NaoData = NaoData*180/pi ;
ROOT_Hips_RotationNew = ROOT_Hips_RotationNew*180/pi ;
% save NaoData NaoData

ROOT_HipsOld = BVHStruct.data.ROOT_Hips ;
ROOT_HipsNew = [ ROOT_HipsOld(:,1:3) ROOT_Hips_RotationNew*pi/180];
BVH_Nao = NaoData_To_NaoBVH_2( NaoData*pi/180, ROOT_HipsNew, BVHFormat,BVHHeadStr ) ;  % all rad
BVH_Nao = radBVH_To_degree( BVH_Nao ) ;  
WriteBVH( BVH_Nao,dataFolder,[dataName,'_NaoRotationBVH_CTest'] );

save BVH_Nao BVH_Nao

%%       
%%%  reduce the frequency
reduceFre = 1 ;
NaoDataNew = zeros(25,10);
n = 0;
for k=1:reduceFre:size(NaoData,2)
    n = n+1;
    NaoDataNew(:,n) = NaoData(:,k);
end
NaoData = NaoDataNew(:,1:n);
%%
Frame_Time_ms = BVHStruct.Frame_Time*100 ;
Frame_Time_ms = Frame_Time_ms*reduceFre ;
Frame_Time_ms_Str = sprintf( 'FrameNumber(%0.1fms)',Frame_Time_ms );
%%%
Nframes = size(NaoData,2);
numdata = 0:(Nframes-1);
NaoData = [ numdata;NaoData ];
for k=1:26
    NaoDataStr( k,: ) = sprintf( '%9.3f ',NaoData( k,: ) );
end

NaoName = { Frame_Time_ms_Str,'HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw'...
            'LElbowRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll'...
            'HipYawPitch','LHipRoll','LHipPitch','LKneePitch','RHipRoll','RHipPitch'...
            'RKneePitch','LAnklePitch','LAnkleRoll','RAnklePitch','RAnkleRoll'...
            'LWristYaw','RWristYaw','LHand','RHand'};
N_name = length(NaoName) ;
for k=1:N_name
    NaoNameStr(k,:) = sprintf( '%20s',NaoName{k} );
end

NaoStr = [ NaoNameStr  NaoDataStr ];
%% Output
% DrawNaoData( NaoData,NaoName );

fID = fopen( [ dataFolder,'\',dataName,'_NaoData_C.txt' ],'w' );

for k=1:size(NaoStr,1)
    fprintf( fID,'%s\n',NaoStr(k,:) );
end
fprintf( fID,'%s','JOINTANGLES_END' );
fclose(fID);
% uiopen([ resPath,'\',dataName,'_NaoData.txt' ]);

disp('Write NaoData OK')

return

function C =  EulerToC_Multi( euler,rotateOrder,rotateDirection )

format long
if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end
if ~exist('rotateDirection','var')
    rotateDirection = [1,1,1];
end

if size(euler,2)~=3
    C = NaN ;
   errordlg('EulerToC_Multi: size of euler Wrong'); 
   return;
end
Nframes = size( euler,1 );
C = zeros( Nframes,3,3 );
for k=1:Nframes
   C( k,:,: )  = EulerToC( euler(k,:),rotateOrder,rotateDirection );
end
