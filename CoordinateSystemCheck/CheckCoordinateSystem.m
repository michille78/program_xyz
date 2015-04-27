%% xyz  2015 3.9
% check the coordinate system

function CheckCoordinateSystem(  )
clc
clear all
close all
%% load data of Hib Root
global resultPath
%  dataFolder = 'E:\data_xyz_noitom\data_CoordinateSystemCheck\YXZ_AttitudeTest';
dataFolder = 'E:\data_xyz_noitom\data_CoordinateSystemCheck\3.11\ZYX1';
% dataFolder = 'E:\data_xyz_noitom\data_CoordinateSystemCheck\ZYX_AttitudeTest';
dataFolder = 'E:\data_xyz_noitom\data_CoordinateSystemCheck\3.11Second\YXZ_RollOnly';

BVHRotateOrder = 'YXZ';

resultPath = [dataFolder,'\result'];
if ~isdir(resultPath)
    mkdir(resultPath);
else
   delete([resultPath,'\*']); 
end

BVHPath = [ dataFolder,'\BVHData.bvh' ];
CalculationQPath = [ dataFolder,'\CalculationData.txt' ];


BVH = readBVH( BVHPath,353 ) ;
BVH = BVH*pi/180 ;
Q = readQ( CalculationQPath,10 );
N_Q = size(Q,1) ;
Qtc = zeros(size(Q)) ;  % Q of t to c
Qbc = [ 1/2;1/2;-1/2;1/2 ];
%% NED attitude
attitudeNED = zeros( N_Q,3 );  % attitudeNED: n*3  [ yaw,pitch,roll ] unit:rad
for k=1:N_Q
    Qtc(k,:) = QuaternionMultiply( Q(k,:),Qbc ) ;
    attitudeNED(k,:) = QtoAttitudeNED( Qtc(k,:) ) ;
end

%% Q-> YXZ euler
eulerYXZ_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerYXZ_Q(k,:) = QtoEuler(Q(k,:),'YXZ') ;
end

%% Q-> XYZ euler
eulerXYZ_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerXYZ_Q(k,:) = QtoEuler(Q(k,:),'XYZ') ;
end

%% Q-> ZYX euler
eulerZYX_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerZYX_Q(k,:) = QtoEuler(Q(k,:),'ZYX') ;
end

%% Q-> ZXY euler
eulerZXY_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerZXY_Q(k,:) = QtoEuler(Q(k,:),'ZXY') ;
end

%% Q-> XZY euler
eulerXZY_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerXZY_Q(k,:) = QtoEuler(Q(k,:),'XZY') ;
end

%% Q-> YZX euler
eulerYZX_Q = zeros( N_Q,3 ) ;
for k=1:N_Q
    eulerYZX_Q(k,:) = QtoEuler(Q(k,:),'YZX') ;
end

%% draw parameters
drawParameters.lineWidth = 2 ;
drawParameters.labelFontSize  = 13 ;
drawParameters.legendFontSize = 15 ;

attNED_fh = DrawAttitudeNED( attitudeNED,drawParameters ) ;
YXZBVH_fh = DrawEuler( BVH,drawParameters,[BVHRotateOrder,'-BVH'] ) ;
YXZEulerQ_fh = DrawEuler( eulerYXZ_Q,drawParameters,'YXZ-Euler-Q' ) ;
XYZEulerQ_fh = DrawEuler( eulerXYZ_Q,drawParameters,'XYZ-Euler-Q' ) ;
YZXEulerQ_fh = DrawEuler( eulerYZX_Q,drawParameters,'YZX-Euler-Q' ) ;
ZYXEulerQ_fh = DrawEuler( eulerZYX_Q,drawParameters,'ZYX-Euler-Q' ) ;
ZXYEulerQ_fh = DrawEuler( eulerZXY_Q,drawParameters,'ZXY-Euler-Q' ) ;
XZYEulerQ_fh = DrawEuler( eulerXZY_Q,drawParameters,'XZY-Euler-Q' ) ;

disp('')


%%
function BVH = readBVH( filePath,numberStartLine )
fid = fopen(filePath,'r' ) ;
BVH = zeros(10,3) ;  % N*3
n = 1 ;
k = 0 ;
while ~feof(fid)
   tline = fgetl(fid) ; 
   if n>=numberStartLine
       lineData = textscan( tline,'%f' ) ;
       k = k+1 ;
       BVH(k,:) = lineData{1}(4:6)' ;
   end
   n = n+1 ;
   
end
fcolse(fid) ;

%%
function Q = readQ( filePath,numberStartLine )
fid = fopen(filePath,'r' ) ;
Q = zeros(10,4) ;  % N*4
n = 1 ;
k = 0 ;
while ~feof(fid)
   tline = fgetl(fid) ; 
   if n>=numberStartLine
       lineData = textscan( tline,'%f' ) ;
       k = k+1 ;
       Q(k,:) = lineData{1}(7:10)' ;
   end
   n = n+1 ;
end


% attitudeNED: n*3  [ yaw,pitch,roll ] unit:rad
function Euler_fh = DrawEuler( Euler,drawParameters,name )
global  resultPath
Euler = Euler*180/pi ;  %  rad to degree

time = (1:size(Euler,1))/120;
Euler_fh = figure('name',name) ;
subplot(3,1,1)
plot( time,Euler(:,1),'linewidth',drawParameters.lineWidth ) ;
ylabel('afa бу','fontSize',drawParameters.labelFontSize)
subplot(3,1,2)
plot( time,Euler(:,2),'linewidth',drawParameters.lineWidth ) ;
ylabel('beita бу','fontSize',drawParameters.labelFontSize)
subplot(3,1,3)
plot( time,Euler(:,3),'linewidth',drawParameters.lineWidth ) ;
ylabel('gama бу','fontSize',drawParameters.labelFontSize)

xlabel('time  s','fontSize',drawParameters.labelFontSize)

saveas( Euler_fh,[resultPath,'\',name,'.fig'] )

% attitudeNED: n*3  [ yaw,pitch,roll ] unit:rad
function attNED_fh = DrawAttitudeNED( attitudeNED,drawParameters )
global resultPath
attitudeNED = attitudeNED*180/pi ;  %  rad to degree

time = (1:size(attitudeNED,1))/120;
attNED_fh = figure('name','attitudeNED') ;
subplot(3,1,1)
plot( time,attitudeNED(:,1),'linewidth',drawParameters.lineWidth ) ;
ylabel('yaw бу','fontSize',drawParameters.labelFontSize)
subplot(3,1,2)
plot( time,attitudeNED(:,2),'linewidth',drawParameters.lineWidth ) ;
ylabel('pitch бу','fontSize',drawParameters.labelFontSize)
subplot(3,1,3)
plot( time,attitudeNED(:,3),'linewidth',drawParameters.lineWidth ) ;
ylabel('roll бу','fontSize',drawParameters.labelFontSize)

xlabel('time  s','fontSize',drawParameters.labelFontSize)

saveas( attNED_fh,[resultPath,'\attitudeNED.fig'] ) 
