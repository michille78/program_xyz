%% xyz 2015 3.19
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
% ePoly = [ A, B, C, D, E, F ]

function H = GenerateEllipseData( ePoly,minX,maxX,pointN )
disp('start Generate EllipseData')

if ~exist('minX','var')
    minX = -100 ;
    maxX = 100 ;
    pointN = 50 ;
end

[ AA, BB, CC, DD, EE, FF ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) ) ;

syms xx y
fStr  = sym('AA*xx^2+BB*xx*y+CC*y^2+DD*xx+EE*y+FF=0') ;
fStr = subs(fStr);

YSym = feval( symengine,'solve',fStr,'y' ) ;  % 用xx表达y
XSym = feval( symengine,'solve',fStr,'xx' ) ; % 用y表达xx

%% scan x at minX:step:maxX
% [Hx_ScanX,Hy_ScanX]
step = (maxX-minX)/pointN*2 ;
k=0;
Hx_k=minX:step:maxX ;
Nx = length(Hx_k)-1;
Hx_ScanX = zeros(Nx,1);
Hy_ScanX = zeros(Nx,1);
for Hx_k=minX:step:maxX
    xx = Hx_k;
    Hy_k = double(subs(YSym));  % solve the equation YSym
    for i=1:length(Hy_k)
        if isreal(Hy_k(i))
            k = k+1 ;
            Hx_ScanX(k,1) = Hx_k ;
            Hy_ScanX(k,1) = Hy_k(i) ;
        end
    end
end
H_ScanX = [ Hx_ScanX Hy_ScanX ] ;
H_ScanX = H_ScanX(1:k,:) ;
%% scan y at minY:step:maxY
% [Hx_ScanY,Hy_ScanY]
maxY = maxX ;
minY = minX ;
step = (maxY-minY)/pointN*2 ;
k=0;
Hy_k=minY:step:maxY ;
Ny = length(Hy_k)-1;
Hx_ScanY = zeros(Ny,1);
Hy_ScanY = zeros(Ny,1);
for Hy_k=minY:step:maxY
    y = Hy_k;
    Hx_k = double(subs(XSym));  % solve the equation XSym
    for i=1:length(Hx_k)
        if isreal(Hx_k(i))
            k = k+1 ;
            Hx_ScanY(k,1) = Hx_k(i) ;
            Hy_ScanY(k,1) = Hy_k ;
        end
    end
end
H_ScanY = [ Hx_ScanY Hy_ScanY ] ;
H_ScanY = H_ScanY(1:k,:) ;

H = [ H_ScanX ; H_ScanY ] ;
disp('Generate EllipseData OK')

%% 按照椭圆的曲线取排序
function SortedH = SortH( H )
N = size(H,1);
Hx = H(:,1) ;
Hy = H(:,2) ;


SortedHx = zeros(N,1);
SortedHy = zeros(N,1);

minHx = min(Hx) ;
maxHx = max(Hx);

HA = zeros(10,1);
HB = zeros(10,1);

