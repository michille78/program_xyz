%% H_b got by rotate on a unknown plane (frame n)
% Calculate the plane by fitting: Hz_n = a*Hx_b + b*Hy_b + c*Hz_b 
% by A = [ a b c], calculate Cnb
% Note: the n frame of every frame is different.
function [ H_n,Cnb,ZYX_Euler,Hz_n,planeFittingError,planeFittingErrorFh ] = RotatePlaneFitting( H_b,dataName,H_earth_xy_Norm,H_earth_z )
format long

if ~exist('dataName','var')
   dataName = ''; 
end

N = size(H_b,1);
if rank(H_b)<3
    errordlg('罗盘数据不足，无法进行校正');
    Cnb = eye(3) ;
    return ;
end
%% 计算导航系n到本体系b的选装矩阵 Cnb
% 导航系n: 航向与b相同的水平坐标系（每个时刻的n系不同）
% dbstop in linearEquation
B = linearEquation( H_b,ones(N,1) ) ;   % must be format long
d = 1/normest(B) ;
A = B*d ;
a = A(1);
b = A(2);
c = A(3);
% [a b c] -> ZYX Euler angle -> Cnb
beita = -asin(a) ;
gama = atan2( b,c ) ;
Cnb = Euler2C( [0 beita gama],'ZYX' ) ;

N = size(H_b,1);
H_n = zeros(N,3);
for k=1:N
    H_n(k,:) = ( Cnb' * H_b(k,:)' )' ;
end

ZYX_Euler = [ 0,beita,gama ]*180/pi ;
Hz_n = d ; % When n frame is  horizontal frame, Hz_n： 天向磁场大小

%%% the plane fitting error
planeFittingErrorData = zeros(N,1);
for k=1:N
    planeFittingErrorData(k) = H_b(k,:)*[ a;b;c ] - d ;
end
mean_err = mean(planeFittingErrorData) ;
re_mean_err = abs( mean_err/H_earth_xy_Norm ) ;
std_err = std(planeFittingErrorData) ;
re_std_err = std_err/H_earth_xy_Norm ;
H_n_z_err = Hz_n-H_earth_z  ;
H_n_z_re_err = H_n_z_err/H_earth_z ;
planeFittingErrorStr = sprintf( 'The plane fitting error:\t mean= %0.0f(%0.2f) \t std= %0.0f(%0.2f) \n\t Hnz= %0.3f  Hnz_err = %0.2f(%0.2f) \n',...
    mean_err,re_mean_err,std_err,re_std_err,Hz_n,H_n_z_err,H_n_z_re_err );
planeFittingError.planeFittingErrorStr = planeFittingErrorStr ;
planeFittingError.re_mean_err = re_mean_err ;
planeFittingError.re_std_err = re_std_err ;
planeFittingError.planeFittingErrorData = planeFittingErrorData ;
%% draw
planeFittingErrorFh = figure( 'name',['planeFittingError_',dataName] ) ;
plot( planeFittingErrorData,'r.' )
xlabel('number')
ylabel('Hn\_z error')
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text( 0,( YTick(1) + YTick(2) )/2,planeFittingErrorStr,'color','r' ) ;
