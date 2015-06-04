%%  xyz  2015.3.13
%  Magnetic Compass Calibration

function ellipseResultStr = MagneticCompassCalibration(  )

% clc
close all
clear all
run('E:\document and program\program_xyz\commonFcn\add_CommonFcn_ToPath.m')

dataName = { '2_StandardPlane','2_InclinedPlane', '2_InclinedPlane2LargeData','2_InclinedPlane3LargeData' };
dataName = {'8clean2_raw_data_calibrate','8_raw_data_calibrate','8Clean_raw_data_calibrate','3_raw_data_calibrate','5Cleared_raw_data_calibrate','new_raw_data_calibrate'};
dataName = {'1clean_raw_data_calibrate','2clean_raw_data_calibrate'};
dataName = dataName(2);
% dataName = {'2_InclinedPlane2LargeData'};
%dataFolder = 'E:\data_xyz_noitom\magneticData\Calibrated_4.9B_Stand\8';
dataFolder = 'E:\data_xyz\magneticData\magnetic_raw_6.4_B';
% dataFolder = 'E:\xyz\research\program_xyz_noitom\magnetic compass calibration\magn_sphere' ;

N = length( dataName );
ellipseResultStr = cell(1,N);
for k=1:N
    ellipseResultStr{k} = DoMagneticCompassCalibration( dataName{k},dataFolder ) ;
end
ellipseResultStr_All = ellipseResultStr{1};
for k=2:N
   ellipseResultStr_All = sprintf( '%s \n%s',ellipseResultStr_All,ellipseResultStr{k} ); 
end

display( ellipseResultStr_All )
disp('MagneticCompassCalibration end ');

function ellipseResultStr = DoMagneticCompassCalibration( dataName,dataFolder )
format long
%% cosnt data
global H_earth_Norm H_earth_xy_Norm H_earth_z
H_earth_Norm = 54552 / 1 ;  % nT  % 地球磁场
H_earth_xy_Norm = sqrt(27949^2+3279^2) / 1 ;%  28140.7 nT
H_earth_z = 46733 ;

% H_earth_Norm = 1 ;
% H_earth_xy_Norm = sqrt(27949^2+3279^2)/54552 ;
% H_earth_z = 46733/54552 ;
% % 
% H_earth_Norm = 54552/sqrt(27949^2+3279^2) ;
% H_earth_xy_Norm = 1 ;
% H_earth_z = 46733 /sqrt(27949^2+3279^2) ;
%% load data
H_b=importdata([dataFolder,'\',dataName,'.mat']) ;

global resPath
resPath = [dataFolder,'\',dataName,'_result'];
if ~isdir(resPath)
    mkdir(resPath)
else
    delete([resPath,'\*'])
end
%% 将单位转为 nT
%%% （1）（桂哥富士通工装校准工具）桂哥的下位机将数据将磁罗盘数据归一化至模=10000   m_new = （m/360）*10000
% H_b = H_b*H_earth_Norm/10000 ;  % nT
%%% （2）（陈聪磁罗盘标定工具）直接得到原始数据，16位模式，每个LSB为 0.15 uT
H_b = H_b * 0.15 *1000 ;
DrawH_b( H_b );
%% reject magnet_b to H_t ( plane fitting )
% dbstop in RotatePlaneFitting
[ H_n,Cnb,ZYX_Euler,Hz_n,planeFittingError1,planeFittingErrorFh ] = RotatePlaneFitting( H_b,'First',H_earth_xy_Norm,H_earth_z ) ;
if planeFittingError1.re_mean_err > 0.03
    warndlg( sprintf(' re_mean_err=%0.2f (first) ',planeFittingError1.re_mean_err),'Rotate Plane Fitting' );
end
if planeFittingError1.re_std_err > 0.08
    warndlg( sprintf(' re_std_err=%0.2f (first) ',planeFittingError1.re_std_err),'Rotate Plane Fitting' );
end

saveas(planeFittingErrorFh,[resPath,'\',get(planeFittingErrorFh,'name')],'fig')
saveas(planeFittingErrorFh,[resPath,'\',get(planeFittingErrorFh,'name')],'emf')
[ H_b,remainNum,H_b_PerfectedStr ] = PerfectPointsOnPlane( planeFittingError1.planeFittingErrorData,H_b,0.5 ) ;

[ H_n,Cnb,ZYX_Euler,Hz_n,planeFittingError2,planeFittingErrorFh ] = RotatePlaneFitting( H_b,'Second',H_earth_xy_Norm,H_earth_z ) ;
saveas(planeFittingErrorFh,[resPath,'\',get(planeFittingErrorFh,'name')],'fig')
saveas(planeFittingErrorFh,[resPath,'\',get(planeFittingErrorFh,'name')],'emf')

% [ H_b,remainNum,H_b_PerfectedStr ] = PerfectPointsOnPlane( planeFittingError2,H_b,0.8 ) ;
% [ H_n,Cnb,ZYX_Euler,Hz_n,planeFittingError2,planeFittingErrorStr2,planeFittingErrorFh ] = RotatePlaneFitting( H_b,'Second',H_earth_xy_Norm,H_earth_z ) ;

%% b_n zero offset 
% 1) 假设 H_n_z 只有零漂，无不正交误差，无倍率误差    2)假设已知 水平 强度与铅垂方向强度比值：已知 H_earth_z
b_n_z = Hz_n-H_earth_z ;

%% ellipse fitting
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
% [ cx;cy;thita;a;b;e;f ]
pointN = size(H_n,1);
H_n_xy = H_n(:,[1 2]) ;
% dbstop in EllipseFitfing
[ ePoly,eStand_Fitting,ellipseFitfingError,ellipseFitfingErrorFh,ThitaFlag ] = EllipseFitfing( H_n_xy,'First',H_earth_xy_Norm ) ;
if ellipseFitfingError.re_mean_err>0.05
    warndlg( sprintf('re_mean_err = %0.2f (first)',ellipseFitfingError.re_mean_err),'Ellipse Fitfing(first)' );
end
if ellipseFitfingError.re_std_err>0.4
    warndlg( sprintf('re_std_err = %0.2f (first)',ellipseFitfingError.re_std_err),'Ellipse Fitfing(first)' );
end

saveas(ellipseFitfingErrorFh,[resPath,'\',get(ellipseFitfingErrorFh,'name')],'fig')
saveas(ellipseFitfingErrorFh,[resPath,'\',get(ellipseFitfingErrorFh,'name')],'emf')
save( [resPath,'\ePoly.mat'],'ePoly' )
save( [resPath,'\eStand_Fitting.mat'],'eStand_Fitting' )

fittingEllipse_n = GenerateEllipseData( ePoly,-H_earth_xy_Norm*1.5,H_earth_xy_Norm*1.5,300 ) ; 
% fittingEllipse_n = GenerateEllipseData( ePoly,-30000,30000,300 ) ;
fittingEllipse_n = [ fittingEllipse_n ones(size(fittingEllipse_n,1),1)*mean(H_n(1:100,3)) ];
fittingN = size(fittingEllipse_n,1);
fittingEllipse_b = zeros(fittingN,3);
for k=1:fittingN
    fittingEllipse_b(k,:) = (Cnb*fittingEllipse_n(k,:)')';
end

cx_f = eStand_Fitting.cx;
cy_f = eStand_Fitting.cy;
a_f = eStand_Fitting.a;
b_f = eStand_Fitting.b;
e_f = eStand_Fitting.e;
f_f = eStand_Fitting.f;
thita_f = eStand_Fitting.thita;

center = [cx_f(1),cy_f(1)];
reCenter = [cx_f(1),cy_f(1)]/H_earth_xy_Norm ;
ellipseResultStr = sprintf( 'e=%0.3f \t c=( %0.2f,%0.2f )=( %0.2f,%0.2f )*Hnxy norm \n thita=%0.2f /degree (%s)',e_f(1),cx_f(1),cy_f(1),reCenter(1),reCenter(2),thita_f*180/pi,ThitaFlag );
%% calculate the magnetic error model
[ ePolyNorm_ErrorModel,rate ] = EPolyNorm_ErrorModel( ePoly,H_earth_xy_Norm ) ;
[G_n,b_n,H_b_Norm] =  EllipsePolyToMagnErrorModel_2D( ePolyNorm_ErrorModel ) ;
%%%% 2D ->  3D 
b_n = [ b_n; b_n_z ];
G_n_z = ( G_n(1,1)+G_n(2,2) )/2;
G_n = [ G_n [0;0]; 0 0 G_n_z ]; 
%%%  magnetic calibration parameters of n frame to b frame
Cbn = Cnb' ;
G_b = Cnb * G_n * Cbn ;
b_b = Cnb * b_n ;
n_MagneticErrorModelStr = sprintf( 'n frame Magnetic Error Model:\n\t G_n = [ %0.2f  %0.2f  %0.2f;  %0.2f  %0.2f %0.2f ;  %0.2f  %0.2f %0.2f ] \n\t b_n = [ %0.2f ; %0.2f ; %0.2f ];',G_n(1,1),G_n(1,2),G_n(1,3),G_n(2,1),G_n(2,2),G_n(2,3),G_n(3,1),G_n(3,2),G_n(3,3),b_n(1),b_n(2),b_n(3) );
b_MagneticErrorModelStr = sprintf( 'b frame Magnetic Error Model:\n\t G_b = [ %0.2f  %0.2f  %0.2f;  %0.2f  %0.2f %0.2f ;  %0.2f  %0.2f %0.2f ] \n\t b_b = [ %0.2f ; %0.2f ; %0.2f ];',G_b(1,1),G_b(1,2),G_b(1,3),G_b(2,1),G_b(2,2),G_b(2,3),G_b(3,1),G_b(3,2),G_b(3,3),b_b(1),b_b(2),b_b(3) );
MagneticErrorModelStr = sprintf( '%s \n  %s',n_MagneticErrorModelStr,b_MagneticErrorModelStr );
%% compansate H_n_xy
H_n_xy_Rectified = CompensateMagnetic_2D( H_n_xy,G_n,b_n ) ;
H_n_Rectified = [ H_n_xy_Rectified repmat( Hz_n,pointN,1 ) ];
%%% compansate H_n_xy
H_b_Rectified = zeros( size(H_n_Rectified) );
for k=1:pointN
    H_b_Rectified(k,:) = ( Cnb * H_n_Rectified(k,:)' )' ;
end
%% fitting the compansated result: H_n_xy_Rectified
% to check the compansation performance
[ ePoly2,eStand_Fitting2,ellipseFitfingError2,ellipseFitfingErrorFh2,ThitaFlag ] = EllipseFitfing( H_n_xy_Rectified,'Second',H_earth_xy_Norm ) ;
cx_f2 = eStand_Fitting2.cx;
cy_f2 = eStand_Fitting2.cy;
a_f2 = eStand_Fitting2.a;
b_f2 = eStand_Fitting2.b;
e_f2 = eStand_Fitting2.e;
f_f2 = eStand_Fitting2.f;
thita_f2 = eStand_Fitting2.thita;
if ellipseFitfingError2.re_mean_err>0.01
    warndlg( sprintf('re_mean_err = %0.2f (second)',ellipseFitfingError2.re_mean_err),'Ellipse Fitfing (Second)' );
end
if ellipseFitfingError2.re_std_err>0.2
    warndlg( sprintf('re_std_err = %0.2f (second)',ellipseFitfingError2.re_std_err),'Ellipse Fitfing (Second)' );
end
if abs (cx_f2/H_earth_xy_Norm )>1e-4 || abs (cy_f2/H_earth_xy_Norm )>1e-4 || e_f2>1e-5
    warndlg( sprintf(' c = (%0.2f,0.2f)   e = %0.2f  too big! ',cx_f2,cy_f2,e_f2 ),'Ellipse Fitfing (Second)' );
end
saveas(ellipseFitfingErrorFh,[resPath,'\',get(ellipseFitfingErrorFh2,'name')],'fig')
saveas(ellipseFitfingErrorFh,[resPath,'\',get(ellipseFitfingErrorFh2,'name')],'emf')
save( [resPath,'\ePoly2.mat'],'ePoly' )
save( [resPath,'\eStand2.mat'],'eStand_Fitting2' )

center2 = [cx_f2(1),cy_f2(1)];
reCenter2 = [cx_f2(1),cy_f2(1)]/H_earth_xy_Norm ;
ellipseResultStr2 = sprintf( 'e=%0.3f \t c=( %0.2f,%0.2f )=( %0.2f,%0.2f )*Hnxy norm \n thita=%0.2f /degree (%s)',e_f2(1),cx_f2(1),cy_f2(1),reCenter2(1),reCenter2(2) ,thita_f2*180/pi,ThitaFlag );
if e_f2>0.05 || abs( reCenter2(1) )>0.05 || abs( reCenter2(2) )>0.05
   errordlg( 'EllipseFitfing Error!' ); 
end

% fittingEllipse_n2 = GenerateEllipseData( ePoly2,-30000,30000,300 ) ;
fittingEllipse_n2 = GenerateEllipseData( ePoly2,-H_earth_xy_Norm*1.5,H_earth_xy_Norm*1.5,300 ) ;
fittingEllipse_n2 = [ fittingEllipse_n2 ones(size(fittingEllipse_n2,1),1)*mean(H_n(1:100,3)) ];
fittingN2 = size(fittingEllipse_n2,2);
fittingEllipse_b2 = zeros(fittingN2,3);
for k=1:fittingN2
    fittingEllipse_b2(k,:) = (Cnb*fittingEllipse_n2(k,:)')';
end
%% Output
ResultStr2 = DrawH_n( H_b,H_n,Cnb,H_n_Rectified,H_b_Rectified,fittingEllipse_n,...
    fittingEllipse_b,center,fittingEllipse_n2,fittingEllipse_b2,center2 ) ;

ellipseResultStr = sprintf( '%s\n\n %s \n  %s \n',dataName,ellipseResultStr,MagneticErrorModelStr );
ResultStr = sprintf( '%s \n\n %s %s %s %s %s \n %s',ellipseResultStr,planeFittingError1.planeFittingErrorStr,...
   planeFittingError1.planeFittingErrorStr,ellipseFitfingError.Str, H_b_PerfectedStr, ellipseFitfingError2.Str,  ResultStr2 );
fID = fopen( [resPath,'\',dataName,'_result.txt'],'w' ) ;
fprintf( fID,ResultStr );
display(ResultStr)
fclose(fID);
disp('OK')



function DrawH_b( H_b )
global resPath

%% 1D x  y  z
figure('name','H_b_x_y_z')
subplot(3,1,1)
plot(H_b(:,1));
ylabel('x')
subplot(3,1,2)
plot(H_b(:,2));
ylabel('y')
subplot(3,1,3)
plot(H_b(:,3));
ylabel('z')

%% norm
N = size( H_b,1 );
H_b_Norm = zeros( N,1 );
for k=1:N
   H_b_Norm(k)  = normest( H_b(k,:) );
end
figure( 'name','H_b_norm' )
plot( H_b_Norm )

%% 3D point in b frame
figure( 'name','H_b_3D_Point_bFrame' )
plot3( H_b(:,1),H_b(:,2),H_b(:,3),'.r' )
xlabel('x\_b')
ylabel('y\_b')
zlabel('z\_b')
[ xmin, xmax, ymin, ymax, zmin, zmax ] =  AxisSameSpace( H_b ) ;
XYZ_Axis = [ xmin, xmax, ymin, ymax, zmin, zmax ] ;
axis(XYZ_Axis) ;

saveas(gcf,[resPath,'\','H_b_3DPoint.fig'])
saveas(gcf,[resPath,'\','H_b_3DPoint.emf'])



%% change unit of H to T （特斯拉）
function Hnew = HUnitToT( H )
Hnew = H/10000*he
N = size(H,1);
normH = zeros(N,1) ;
for k=1:N
    normH(k) = normest(H(k,:));
end
normHmean = mean(normH) ;
Hnew = H/10000*HearthNorm/normHmean ;
