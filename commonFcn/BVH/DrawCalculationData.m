%% xyz  2015.6.3

%% 根据中间数据画出人形
% 中间数据
% CalData.ROOT_Hips.X   [3*N]  m
%                  .V   [3*N]  m/s
%                  .Q   [4*N]
%                  .A   [3*N]
%                  .W   [3*N]
%  21个关节名见 GetCal21Bone() 

function  DrawCaliculationData( CalStruct )

clc
close all
clear all

CalStruct = importdata( 'CalStruct.mat' );

JointName_Cal = GetCal21Bone();
for i=1:21
    % Z反向
    eval( sprintf( 'CalStruct.%s.X(3,:) = -CalStruct.%s.X(3,:) ;',JointName_Cal{i},JointName_Cal{i} ) ) ;
    eval( sprintf( '%s = CalStruct.%s.X ;',JointName_Cal{i},JointName_Cal{i} ) ) ;
    eval( sprintf( 'X%d = CalStruct.%s.X ;',i,JointName_Cal{i} ) ) ;
    eval( sprintf( 'X(%d,:,:) = CalStruct.%s.X ;',i,JointName_Cal{i} ) ) ;
end

%     ROOT_Hips
%     RightUpLeg
%     RightLeg
%     RightFoot
%     LeftUpLeg
%     LeftLeg
%     LeftFoot
%     RightShoulder
%     RightArm
%     RightForeArm
%     RightHand
%     LeftShoulder
%     LeftArm
%     LeftForeArm
%     LeftHand
%     Head
%     Neck
%     Spine3
%     Spine2
%     Spine1
%     Spine
    
%% 

N = size( X,3 );







for k=1:10:N
    figure('name','CalculationData')
    hold on
    for i=1:21
       joint = X(i,:,k);
       plot3( joint(1),joint(2),joint(3),'*','MarkerSize',8);
    end
    
    PlotOneBone( X(1,:,k),X(2,:,k) ) ;
    
    PlotOneBone( X(2,:,k),X(3,:,k) ) ;
    PlotOneBone( X(3,:,k),X(4,:,k) ) ;
    PlotOneBone( X(1,:,k),X(5,:,k) ) ;
    PlotOneBone( X(5,:,k),X(6,:,k) ) ;
    PlotOneBone( X(6,:,k),X(7,:,k) ) ;
    
    PlotOneBone( X(1,:,k),X(21,:,k) ) ;
    PlotOneBone( X(21,:,k),X(20,:,k) ) ;
    PlotOneBone( X(20,:,k),X(19,:,k) ) ;
    PlotOneBone( X(19,:,k),X(18,:,k) ) ;
    PlotOneBone( X(18,:,k),X(17,:,k) ) ;
    PlotOneBone( X(17,:,k),X(16,:,k) ) ;
    
    PlotOneBone( X(18,:,k),X(8,:,k) ) ;
    PlotOneBone( X(8,:,k),X(9,:,k) ) ;
    PlotOneBone( X(9,:,k),X(10,:,k) ) ;
    PlotOneBone( X(10,:,k),X(11,:,k) ) ;
    PlotOneBone( X(18,:,k),X(12,:,k) ) ;
    PlotOneBone( X(12,:,k),X(13,:,k) ) ;
    PlotOneBone( X(13,:,k),X(14,:,k) ) ;
    PlotOneBone( X(14,:,k),X(15,:,k) ) ;
    axis equal
    
    close gcf
    hold off
end



function PlotOneBone( Xa,Xb )
plot3( [ Xa(1) Xb(1) ],[ Xa(2) Xb(2) ],[ Xa(3) Xb(3) ] ) ;
