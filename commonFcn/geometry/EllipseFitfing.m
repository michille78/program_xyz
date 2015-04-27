%% xyz  2015.3.13

%% ellipse fitting
%%% Input 
%  H_n_xy (N*2)
%%% output
% ePoly_Fitting(:,k) = [ A; B; C; D; E; F ]
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
% eStand_Fitting.cx  .cy  .thita  .a  .b  .e  .f
% literature  [ ] 孙宏伟, 房建成, 李艳. 椭圆拟合方法在磁罗盘罗差校准中的应用 [J]. 光学 精密工程, 2009, 17(12): 3034.
%             [ ] FITZGIBBON A, PILU M, FISHER R B. Direct least square fitting of ellipses [J]. Pattern Analysis and Machine Intelligence, IEEE Transactions on, 1999, 21(5): 476-80.

% for i=1:size(ePoly_Fitting,2)
%     [ A_f(1,i), B_f(1,i), C_f(1,i), D_f(1,i), E_f(1,i), F_f(1,i) ] = deal( ePoly_Fitting(1,i),ePoly_Fitting(2,i),ePoly_Fitting(3,i),ePoly_Fitting(4,i),ePoly_Fitting(5,i),ePoly_Fitting(6,i) ) ;
%     H_Fiting{i} = GenerateEllipseData( ePoly_Fitting ) ;  % Fitfing Ellipse point
% end

function [ ePoly_Fitting,eStand_Fitting,ellipseFitfingError,ellipseFitfingErrorFh,ThitaFlag ] = EllipseFitfing( H_n_xy,dataName,H_earth_xy_Norm )
format long
if ~exist('dataName','var')
   dataName = ''; 
end

N = size(H_n_xy,1);
Hx = H_n_xy(:,1) ;
Hy = H_n_xy(:,2) ;

% X(k,:)  = [ x^2, x*y, y^2, x, y, 1 ];
X = zeros( N,6 ) ;
for k=1:N
    x = H_n_xy(k,1) ;
    y = H_n_xy(k,2) ;
    X(k,:)  = [ x^2, x*y, y^2, x, y, 1 ];    
end
% get D1 D2 S1 S2
D1 = X( :,1:3 ) ;
D2 = X( :,4:6 ) ;
S1 = D1'*D1 ;
S2 = D1'*D2 ;
S3 = D2'*D2 ;
C1 = [  0    0   2
        0   -1  0
        2   0   0   ];
    
if rank(S3) == length(S3)
    K = C1\(S1-S2/S3*S2') ;
    [V,D] = eig(K) ;
    eigValue = diag(D) ;
    eigNumPos = sum( eigValue>0 ) ;
    fprintf( 'ellipse solution num = (positive eigenvalue num ) = %d \n',eigNumPos );
    ePoly = zeros( 6,eigNumPos );
    j=0;
    constraintEllipse = zeros(1,3) ;
    for i=1:3
       constraintEllipse(i)  = V(:,i)' * C1 * V(:,i) ;
       if  constraintEllipse(i)>0
           % got a solution
           j = j+1 ;
           a1 = V(:,1) * sqrt( 1/( constraintEllipse(i) ) ) ;  
           a2 = -S3\S2'*a1(:,j) ;
           ePoly(:,j) = [ a1;a2 ];
       end
    end
else
    errdlg('无畸变')
    ePoly = zeros(6,1);
end
if ePoly(1)<0
    ePoly=-ePoly ;
end
% berr = IsStandEllipsePoly( ePoly ) ;

%% Normalize the ePoly 
 dbstop in EPolyNorm_Geometry
[ ePoly_Fitting,eStand_Fitting ] = EPolyNorm_Geometry( ePoly ) ;

[ eStand_Fitting,ThitaFlag ] = CalculateThita_WhenZero( eStand_Fitting,H_n_xy );

%% check
% ePoly_Fitting_Check = EllipseStandToPoly( eStand_Fitting ) ; 
% [ ePoly_Fitting_Check,eStand_Fitting_Check ] = EPolyNorm_Geometry( ePoly_Fitting_Check ) ;
% ePoly_Fitting_err = ePoly_Fitting - ePoly_Fitting_Check ;
% ePoly_Fitting_err_re = ePoly_Fitting_err./ePoly_Fitting ;
% eStand_Fitting_err  = eStand_Fitting-eStand_Fitting_Check ;
%% record
%  Ellipse Fitfing result and error
ellipseFitfingErrorData = zeros( N,1 ) ;
for k=1:N
    temp = X(k,:)*ePoly_Fitting ;
    ellipseFitfingErrorData(k) = sqrt( sqrt(abs(temp)) )*sign(temp) ;     % change to stand unit (nT)
end
mean_err = mean(ellipseFitfingErrorData) ;
re_mean_err = mean_err / H_earth_xy_Norm ;
std_err = std(ellipseFitfingErrorData) ;
re_std_err = std_err / H_earth_xy_Norm ;
ellipseFitfingErrorStr = sprintf( 'The ellipse fitting error( %s ): mean= %0.2f(%0.2f) \t std= %0.2f(%0.2f) \n',...
        dataName,mean_err,re_mean_err,std_err,re_std_err );

ellipseFitfingError.Str = ellipseFitfingErrorStr ;
ellipseFitfingError.re_mean_err = re_mean_err ;
ellipseFitfingError.re_std_err = re_std_err ;
ellipseFitfingError.ellipseFitfingErrorData = ellipseFitfingErrorData ;
%% draw
ellipseFitfingErrorFh = figure( 'name',['ellipseFitfingError',dataName] ) ;
plot( ellipseFitfingErrorData,'r.' )
xlabel('number')
ylabel('ellipse error')
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text( 0,( YTick(1) + YTick(2) )/2,ellipseFitfingErrorStr,'color','r' ) ;


