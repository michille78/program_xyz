%% xyz 2015.3.13

%% plot H_n_x H_n_y
function ResultStr = DrawH_n( H_b,H_n,Cnb,H_n_Rec,H_b_Rec,fittingEllipse_n,...
    fittingEllipse_b,center,fittingEllipse_n2,fittingEllipse_b2,center2 )
global resPath  H_earth_Norm H_earth_xy_Norm H_earth_z
ResultStr = '';
N = size(H_b,1) ;
%% 1D x  y  z
figure('name','H_n_x_y_z')
subplot(3,1,1)
plot(H_n(:,1),'b');
hold on 
plot(H_n_Rec(:,1),'r');
ylabel('x')
legend('original','rejected')
subplot(3,1,2)
plot(H_n(:,2),'b');
hold on 
plot(H_n_Rec(:,2),'r');
ylabel('y')
subplot(3,1,3)
plot(H_n(:,3),'b');
hold on 
plot(H_n_Rec(:,3),'r');
ylabel('z')
[ mean_H_nz,std_H_nz,maxErr1_H_nz,maxErr2_H_nz ] = ConstDataError( H_n(:,3) );
H_nz_str = sprintf( 'Hnz: std = %0.2f(%0.2f); mean = %0.2f(%0.2f); maxErr1 = %0.2f; maxErr2 = %0.2f',std_H_nz,std_H_nz/H_earth_z,mean_H_nz,mean_H_nz/H_earth_z-1,maxErr1_H_nz,maxErr2_H_nz ) ;
ResultStr = sprintf( '%s \n%s',ResultStr,H_nz_str );
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text( 0,( YTick(1)*3 + YTick(2) )/4,H_nz_str,'color','r' ) ;
line( [ 0  XTick(length(XTick))],[mean_H_nz mean_H_nz] ,'color','r') ;

saveas(gcf,[resPath,'\','H_n_x_y_z.fig'])
saveas(gcf,[resPath,'\','H_n_x_y_z.emf'])

%% 1D xyNorm  xyzNorm
Hxy = zeros(N,1);
Hxyz = zeros(N,1);
Hxy_Rec = zeros(N,1);
Hxyz_Rec = zeros(N,1);
for k=1:N
   Hxy(k)  = normest(H_n(k,1:2)) ;
   Hxyz(k)  = normest(H_n(k,:)) ;
   Hxy_Rec(k)  = normest(H_n_Rec(k,1:2)) ;
   Hxyz_Rec(k)  = normest(H_n_Rec(k,:)) ;
end

%% H_n_xy normest   and    H_n_xyz normest 
figure('name','H_n_Normest')
%% H_n_xy normest
subplot(2,1,1)
plot(Hxy,'b')
hold on 
plot(Hxy_Rec,'r');
legend('original','rejected')

[ meanD,stdD,maxErr1,maxErr2 ] = ConstDataError( Hxy );
Hxystr = sprintf( 'Hxy: std = %0.2f(%0.2f); mean = %0.2f(%0.2f); maxErr1 = %0.2f; maxErr2 = %0.2f',stdD,stdD/H_earth_xy_Norm,meanD,meanD/H_earth_xy_Norm-1,maxErr1,maxErr2 ) ;
ResultStr = sprintf( '%s \n%s',ResultStr,Hxystr );
[ meanD,stdD,maxErr1,maxErr2 ] = ConstDataError( Hxy_Rec );
HxyRecstr = sprintf( 'Hx Rec: std = %0.2f(%0.2f); mean = %0.2f(%0.2f); maxErr1 = %0.2f; maxErr2 = %0.2f',stdD,stdD/H_earth_xy_Norm,meanD,meanD/H_earth_xy_Norm-1,maxErr1,maxErr2 ) ;
ResultStr = sprintf( '%s \n%s',ResultStr,HxyRecstr );
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text( 0,( YTick(1) + YTick(2) )/2,Hxystr,'color','r' ) ;
text( 0,( YTick(length(YTick)) + YTick(length(YTick)-1) )/2,HxyRecstr,'color','r' ) ;
line( [ 0  XTick(length(XTick))],[meanD meanD] ,'color','r') ;

ylabel('H\_n\_xy')
%% H_n_xyz normest 
subplot(2,1,2)
plot(Hxyz)
hold on 
plot(Hxyz_Rec,'r');
legend('original','rejected')

[ meanD,stdD,maxErr1,maxErr2 ] = ConstDataError( Hxyz );
Hxyz_str = sprintf( 'Hxyz : std = %0.2f(%0.2f); mean = %0.2f(%0.2f); maxErr1 = %0.2f; maxErr2 = %0.2f',stdD,stdD/H_earth_Norm,meanD,meanD/H_earth_Norm-1,maxErr1,maxErr2 ) ;

[ meanD,stdD,maxErr1,maxErr2 ] = ConstDataError( Hxyz_Rec );
HxyzRec_str = sprintf( 'Hxyz Rec : std = %0.2f(%0.2f); mean = %0.2f(%0.2f); maxErr1 = %0.2f; maxErr2 = %0.2f',stdD,stdD/H_earth_Norm,meanD,meanD/H_earth_Norm-1,maxErr1,maxErr2 ) ;
ResultStr = sprintf( '%s \n%s',ResultStr,HxyzRec_str );
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text( 0,( YTick(1) + YTick(2) )/2,Hxyz_str,'color','r' ) ;
text( 0,( YTick(length(YTick)) + YTick(length(YTick)-1) )/2,HxyzRec_str,'color','r' ) ;

line( [ 0  XTick(length(XTick))],[meanD meanD] ,'color','r') ;

ylabel('H\_n\_xyz')
saveas(gcf,[resPath,'\','H_n_Normest.fig'])
saveas(gcf,[resPath,'\','H_n_Normest.emf'])

%% n_xy point 2D
figure('name','H_n_xy_Point')
plot(H_n(:,1),H_n(:,2),'.r');
hold on
plot(H_n_Rec(:,1),H_n_Rec(:,2),'.g');
plot(fittingEllipse_n(:,1),fittingEllipse_n(:,2),'.k','markersize',4);
plot(fittingEllipse_n2(:,1),fittingEllipse_n2(:,2),'.b','markersize',4);

legend('original','rejected','fitting ellipse','compansated fitting ellipse');
plot(0,0,'+r','markersize',6)
plot(center(1),center(2),'+k','markersize',6)
plot(center2(1),center2(2),'+b','markersize',6)

xlabel('x')
ylabel('y')
[ xmin, xmax, ymin, ymax ] =  AxisSameSpace( H_n ) ;
[ xmin_Rec, xmax_Rec, ymin_Rec, ymax_Rec ] =  AxisSameSpace( H_n_Rec ) ;
axis([ min(xmin,xmin_Rec), max(xmax,xmax_Rec), min(ymin,ymin_Rec), max(ymax,ymax_Rec)]) ;
axis equal
str = sprintf( 'points num=%d',length(H_n) ) ;
YTick = get( gca,'YTick' ) ;
XTick = get( gca,'XTick' ) ;
text(XTick(1),YTick(1),str) ;
saveas(gcf,[resPath,'\','H_n_xy.fig'])
saveas(gcf,[resPath,'\','H_n_xy.emf'])

%% 2D point in n_xz frame
figure( 'name','H_n_xz_Point' )
plot( H_n(:,1),H_n(:,3),'.r' )
hold on
plot(H_n_Rec(:,1),H_n_Rec(:,3),'.g');
legend('original','rejected')

xlabel('x\_n')
ylabel('z\_n')
[ xmin, xmax, zmin, zmax ] =  AxisSameSpace( H_n(:,[1,3]) ) ;
axis([ xmin, xmax, zmin, zmax]) ;
axis equal

hold on
XLim = get(gca,'XLim') ; 
line( XLim,[mean_H_nz,mean_H_nz],'color','b' ) ;
% 上下点个数
Hz_high = H_n(:,3)>mean_H_nz ;
N_Hz_high = sum(Hz_high) ;
Hz_low = H_n(:,3)<mean_H_nz ;
N_Hz_low = sum(Hz_low) ;

str = sprintf( ' Origina \n Number of points = %d \n Numbers of high points = %d \n Numbers of low points = %d \n',length(H_n),N_Hz_high,N_Hz_low ) ;
YLim = get( gca,'YLim' ) ;
text(XLim(1),(YLim(1)+YLim(2)*3)/4,str) ;

saveas(gcf,[resPath,'\','H_n_xz_Point.fig'])
saveas(gcf,[resPath,'\','H_n_xz_Point.emf'])

%% 3D point in n frame
figure( 'name','H_n_3D_Point' )
plot3( H_n(:,1),H_n(:,2),H_n(:,3),'.r' )
hold on
plot3( H_n_Rec(:,1),H_n_Rec(:,2),H_n_Rec(:,3),'.g','markersize',5 )
plot3(fittingEllipse_n(:,1),fittingEllipse_n(:,2),fittingEllipse_n(:,3),'.k','markersize',4);
plot3(fittingEllipse_n2(:,1),fittingEllipse_n2(:,2),fittingEllipse_n2(:,3),'.b','markersize',4);
legend('original','rejected','fitting ellipse','compansated f e');

xlabel('x\_n')
ylabel('y\_n')
zlabel('z\_n')
[ xmin, xmax, ymin, ymax, zmin, zmax ] =  AxisSameSpace( H_n ) ;
axis([ xmin, xmax, ymin, ymax, zmin, zmax ]) ;
axis_scope = xmax-xmin ;

bAxis_n = Cnb'*eye(3)*axis_scope/4;      % b系坐标轴在n系下的表达 bAxis_n(:,i)
origin_n = mean(H_n,1)' ;         % 简单的以均值为原点
bAxisPoint_n = bAxis_n+repmat(origin_n,1,3) ;   % b系坐标轴端点的n系坐标

bX_n = [ origin_n, bAxisPoint_n(:,1) ] ;   % b系x轴上 两个端点的坐标 bX_n(:,1) 和 bX_n(:,2)
bY_n = [ origin_n, bAxisPoint_n(:,2) ] ; 
bZ_n = [ origin_n, bAxisPoint_n(:,3) ] ; 
hold on
plot3( bX_n(1,:),bX_n(2,:),bX_n(3,:),'color','b','linewidth',2 )
plot3( bY_n(1,:),bY_n(2,:),bY_n(3,:),'color','b','linewidth',2 )
plot3( bZ_n(1,:),bZ_n(2,:),bZ_n(3,:),'color','b','linewidth',2 )
text( origin_n(1),origin_n(2),origin_n(3),'O\_n','fontsize',13,'color','g' )
text( bX_n(1,2),bX_n(2,2),bX_n(3,2),'X\_b','fontsize',13,'color','b' )
text( bY_n(1,2),bY_n(2,2),bY_n(3,2),'Y\_b','fontsize',13,'color','b' )
text( bZ_n(1,2),bZ_n(2,2),bZ_n(3,2),'Z\_b','fontsize',13,'color','b' )

step = ( xmax-xmin )/10 ;
[ meshX,meshY ] = meshgrid( xmin:step:xmax,ymin:step:ymax ) ;
meshZ = ones( size(meshX) )*mean_H_nz ;
sh = surf(meshX,meshY,meshZ) ;
set(sh,'FaceColor',[1 0.968627452850342 0.921568632125854],'EdgeColor',[0 0 1])

saveas(gcf,[resPath,'\','H_n_3DPoint.fig'])
saveas(gcf,[resPath,'\','H_n_3DPoint.emf'])
%% 3D point in b frame
figure( 'name','Show_nFrame_In_bFrame' )
plot3( H_b(:,1),H_b(:,2),H_b(:,3),'.r' )
hold on
plot3( H_b_Rec(:,1),H_b_Rec(:,2),H_b_Rec(:,3),'.g','markersize',5);
plot3(fittingEllipse_b(:,1),fittingEllipse_b(:,2),fittingEllipse_b(:,3),'.k','markersize',4);
plot3(fittingEllipse_b2(:,1),fittingEllipse_b2(:,2),fittingEllipse_b2(:,3),'.b','markersize',4);
legend('original','rejected','fitting ellipse','compansated f e');

xlabel('x\_b')
ylabel('y\_b')
zlabel('z\_b')
[ xmin, xmax, ymin, ymax, zmin, zmax ] =  AxisSameSpace( H_b ) ;
XYZ_Axis = [ xmin, xmax, ymin, ymax, zmin, zmax ] ;
axis(XYZ_Axis) ;

nAxis_b = Cnb*eye(3)*axis_scope/4;      % n系坐标轴在b系下的表达 nAxis_b(:,i)
origin_b = mean(H_b,1)' ;         % 简单的以均值为原点
nAxisPoint_b = nAxis_b+repmat(origin_b,1,3) ;   % n系坐标轴端点的b系坐标

nX_b = [ origin_b, nAxisPoint_b(:,1) ] ;   
nY_b = [ origin_b, nAxisPoint_b(:,2) ] ; 
nZ_b = [ origin_b, nAxisPoint_b(:,3) ] ; 
hold on
plot3( nX_b(1,:),nX_b(2,:),nX_b(3,:),'color','b','linewidth',2 )
plot3( nY_b(1,:),nY_b(2,:),nY_b(3,:),'color','b','linewidth',2 )
plot3( nZ_b(1,:),nZ_b(2,:),nZ_b(3,:),'color','b','linewidth',2 )
text( origin_b(1),origin_b(2),origin_b(3),'O\_b','fontsize',13,'color','g' )
text( nX_b(1,2),nX_b(2,2),nX_b(3,2),'X\_n','fontsize',13,'color','b' )
text( nY_b(1,2),nY_b(2,2),nY_b(3,2),'Y\_n','fontsize',13,'color','b' )
text( nZ_b(1,2),nZ_b(2,2),nZ_b(3,2),'Z\_n','fontsize',13,'color','b' )

saveas(gcf,[resPath,'\','Show_nFrame_In_bFrame.fig'])
saveas(gcf,[resPath,'\','Show_nFrame_In_bFrame.emf'])

%% get a same scope of x y z axis
function [ xmin, xmax, ymin, ymax, zmin, zmax ] =  AxisSameSpace( H )
if size(H,2)==2
    H = [H ones(size(H,1),1)];
end

xmin = min( H(:,1) ) ;
xmax = max( H(:,1) ) ;
xsope = xmax-xmin ;
ymin = min( H(:,2) ) ;
ymax = max( H(:,2) ) ;
ysope = ymax-ymin ;

zmin = min( H(:,3) ) ;
zmax = max( H(:,3) ) ;
zsope = zmax-zmin ;

maxScope = max( [ xsope,ysope,zsope ] ) ;
maxScope = maxScope*1.1 ;

xmin = xmin - (maxScope-xsope)/2 ;
xmax = xmax + (maxScope-xsope)/2 ;

ymin = ymin - (maxScope-ysope)/2 ;
ymax = ymax + (maxScope-ysope)/2 ;

zmin = zmin - (maxScope-zsope)/2 ;
zmax = zmax + (maxScope-zsope)/2 ;

XYZ_Axis = [ xmin, xmax, ymin, ymax, zmin, zmax ] ;