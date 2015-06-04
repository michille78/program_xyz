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