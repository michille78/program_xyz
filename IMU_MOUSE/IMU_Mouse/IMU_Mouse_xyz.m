%% xyz 2015.2.15
%% buaaxyz@yeah.net


function IMU_Mouse_xyz(  )

clc
close all
clear all
%% parameters to set
fileName = 'slip one time';
%% make and empty dir
addpath( 'coordinate');

dataPath = [ 'data\',fileName, ];
resultPath = [ dataPath,'\result' ];
if isdir( resultPath )
    delete([resultPath,'\*'])
else
    mkdir(resultPath);
end
%% load data
% frameId	modelId	qs	qx	qy	qz	gx	gy	gz	ax	ay	az
QuaGyroAccData_txt = importdata( [dataPath,'\QuaGyroAccData.txt'] );
QuaGyroAccData = QuaGyroAccData_txt.data ;
frameId = QuaGyroAccData(:,1) ;
attQua = QuaGyroAccData(:,3:6) ;
gyro = QuaGyroAccData(:,7:9) ;
acc = QuaGyroAccData(:,10:12)/2048 ;    % unit:g
acc = addAccWhole( acc ) ;

att_Euler_xyz =  CalAttitude_Acc( acc ) ;

N = size(attQua,1) ;
attEuler = zeros(N,3);
for k=1:N
   Cnb = FQtoCnb( attQua(k,:) ) ; 
   attEuler(k,:) = GetAttitude( Cnb,'degree' ) ;
end

%% calculate
% reject th acc to water plane
acc_waterPlane = rejectAccToWaterPlane( acc,attQua ) ;


%% draw parameters
drawParameters.lineWidth = 2 ;
drawParameters.labelFontSize  = 13 ;
drawParameters.legendFontSize = 15 ;
%% draw 
acc_fh1 = drawAcc( acc,drawParameters,'accRaw' ) ;
acc_fh2 = drawAcc( acc_waterPlane,drawParameters,'accWP' ) ;
acc_fh = [ acc_fh1 acc_fh2 ] ;

euler_fh = drawEuler( attEuler,drawParameters );
gyro_fh = drawGyro( gyro,drawParameters );

euler_fh_B = drawEuler( att_Euler_xyz,drawParameters );


saveas( acc_fh(1),[resultPath,'\',get(acc_fh(1),'name'),'.fig'] )
saveas( acc_fh(2),[resultPath,'\',get(acc_fh(2),'name'),'.fig'] )
saveas( euler_fh(1),[resultPath,'\',get(euler_fh(1),'name'),'.fig'] )
saveas( gyro_fh(1),[resultPath,'\',get(gyro_fh(1),'name'),'.fig'] )

saveas( acc_fh(1),[resultPath,'\',get(acc_fh(1),'name'),'.emf'] )
saveas( acc_fh(2),[resultPath,'\',get(acc_fh(2),'name'),'.emf'] )
saveas( euler_fh(1),[resultPath,'\',get(euler_fh(1),'name'),'.emf'] )
saveas( gyro_fh(1),[resultPath,'\',get(gyro_fh(1),'name'),'.emf'] )

disp('OK')

%% accNew = acc - Cnb*G
% G = [ 0 0 1];
function accNew = compensateG(acc,attQua)


%% get the acc rejected to the water plane
% 
function acc_waterPlane = rejectAccToWaterPlane( acc,attQua )
N = size(acc,1) ;
acc_waterPlane = zeros(N,3) ;
for k=1:N
    Cnb = FQtoCnb(attQua(k,:)) ;
    acc_waterPlane(k,:) = ( Cnb*acc(k,1:3)' )';    
end


function acc_fh = drawAcc( acc,drawParameters,dataName )
time = (1:size(acc,1))/120;
acc_fh1 = figure('name',dataName);

subplot(3,1,1);
plot(time,acc(:,1),'linewidth',drawParameters.lineWidth)
ylabel('acc\_x','fontSize',drawParameters.labelFontSize)

title(dataName,'fontsize',drawParameters.labelFontSize)

subplot(3,1,2);
plot(time,acc(:,2),'linewidth',drawParameters.lineWidth)
ylabel('acc\_y','fontSize',drawParameters.labelFontSize)

subplot(3,1,3);
plot(time,acc(:,3),'linewidth',drawParameters.lineWidth)
ylabel('acc\_z','fontSize',drawParameters.labelFontSize)

xlabel('time  s','fontSize',drawParameters.labelFontSize)

if size(acc,2)==4
    acc_fh2 = figure('name','acc_whole');
    plot(time,acc(:,4),'linewidth',drawParameters.lineWidth)
    title([dataName,'_whole'],'fontsize',drawParameters.labelFontSize)
    xlabel('time  s','fontSize',drawParameters.labelFontSize)
    ylabel('acc\_whole  g','fontSize',drawParameters.labelFontSize)

    acc_fh = [ acc_fh1,acc_fh2 ] ;
else
    acc_fh = acc_fh1 ;
end

function euler_fh = drawEuler( euler,drawParameters )
time = (1:size(euler,1))/120;
euler_fh = figure('name','attitude_pitch_roll_yaw');
plot(time,euler,'linewidth',drawParameters.lineWidth)
title('attitude\_pitch\_roll\_yaw','fontsize',drawParameters.labelFontSize)
legend({'pitch','roll','yaw'},'fontsize',drawParameters.labelFontSize)
xlabel('time  s','fontSize',drawParameters.labelFontSize)
ylabel('attitude  бу','fontSize',drawParameters.labelFontSize)

function gyro_fh = drawGyro( gyro,drawParameters )
time = (1:size(gyro,1))/120;
gyro_fh = figure('name','angular velocity');

subplot(3,1,1)
plot(time,gyro(:,1),'linewidth',drawParameters.lineWidth)
ylabel('pitch velocity','fontSize',drawParameters.labelFontSize)

title('angular velocity  ','fontsize',drawParameters.labelFontSize)

subplot(3,1,2)
plot(time,gyro(:,2),'linewidth',drawParameters.lineWidth)
ylabel('roll velocity','fontSize',drawParameters.labelFontSize)

subplot(3,1,3)
plot(time,gyro(:,3),'linewidth',drawParameters.lineWidth)
ylabel('yaw velocity','fontSize',drawParameters.labelFontSize)

xlabel('time  s','fontSize',drawParameters.labelFontSize)

function acc = addAccWhole( acc )
N = size(acc,1) ;
acc_new = zeros( N,4 );
acc_new(:,1:3) = acc;
for k=1:N
    acc_new(k,4) = normest(acc(k,:)) ;
end
acc = acc_new ;
