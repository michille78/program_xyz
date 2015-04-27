%% xyz 2015 3.18
% ellipse polynomial parameters to stand parameters
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
%%% input ePoly = [ A, B, C, D, E, F ]
%%% output eStand = [ cx,cy,a,b,thita,e,f ]
% [cx,cy] : circle of ellipse
% a : x semi-axes length
% b : y semi-axes length
% thita : rotation angle from (x,y) to (x'',y''). There is no rotation in the ellipse in (x'',y'').
%  unit : (rad)
% e: eccentricity 
% f: The distance from the center C to either focus
%% When two solutions
% first one is: a>b   solutionFlag = 'a>b'
% second one is a<b   solutionFlag = 'a<b'

function  [ cx,cy,thita,a,b,e,f,solutionFlag ] = ellipsePolyToStand( ePoly )

if ePoly(1)<0
   ePoly = -ePoly ; 
end
[ A, B, C, D, E, F ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) ) ;

K = [ -2*A,  -B
      -B,   -2*C ] ;
center = K \ [D;E];
cx = center(1) ;
cy = center(2) ;
%% 
G = A*cx^2+B*cx*cy+C*cy^2-F ;

abMin_s = ( A+C-sqrt( (A+C)^2-4*G ) )/2 ;
abMax_s = G/abMin_s ;
abMin = sqrt(abMin_s);
abMax = sqrt(abMax_s);

%%% a<b
a1 = abMin ;
b1 = abMax ;
thita1 = asin( B/(b1^2-a1^2) )/2 ;
%%% a>b
a2 = abMax ;
b2 = abMin ;
thita2 = asin( B/(b2^2-a2^2) )/2 ;





if abs((A-C)/A)<1e-5 && abs(B/A)
    % 认为a=b
    % 正圆情况下的 罗差计算，需要利用 圆心和原始数据重新计算（缸体二阶最小惯性矩法）
    % 暂时将 thita 置零，之后再独立计算
    
    
    thita = 0 ;   % 有问题！
    a = sqrt( (A+C)/2 ) ;
    b = a ;
    solutionFlag = {'a=b'};
else
    a = zeros(1,2);
    b = zeros(1,2);
    thita = zeros(1,2);
    solutionFlag = {'a>b','a<b'};
    %%% a>b解
    thita(1) = 0.5*atan2(-B,C-A) ;
    %%% a<b解
    thita(2) = 0.5*atan2(B,A-C) ;

    for i=1:2
        if abs(thita(i))<1e-4
            a(i) = sqrt(C);
            b(i) = sqrt(A);
        else
            a(i) = sqrt( 0.5*(A+C-B/sin(2*thita(i))) ) ;
            b(i) = sqrt( 0.5*(A+C+B/sin(2*thita(i))) ) ;
        end
    end
    cx = [cx cx];
    cy = [cy cy];
end
for k=1:length(cx)   
   f(1,k) = sqrt( abs(a(1,k)^2-b(1,k)^2) ) ;
   e(1,k) = f(1,k)/max(a(1,k),b(1,k)) ;
end

soluN = length(a);
eStand = struct;
for k=1:soluN
   eStand(k).cx = cx(1) ;
   eStand(k).cy = cy(1) ;
   eStand(k).a = a(1) ;
   eStand(k).b = b(1) ;
   eStand(k).thita = thita(1) ;
   eStand(k).e = e(1) ;
   eStand(k).f = f(1) ;
end

%% 
% for check : D,E,cx,cy
function [ a,b,thita ] = aBig( A,B,C,D,E,cx,cy )

thita = 0.5*atan2(-B,C-A) ;

if thita==0
    a = sqrt(C);
    b = sqrt(A);
else
    a = sqrt( 0.5*(A+C-B/sin(2*thita)) ) ;
    b = sqrt( 0.5*(A+C+B/sin(2*thita)) ) ;
end
    
AplusC_check = A-C-(b^2-a^2)*( cos(thita)^2-sin(thita)^2)  ;
AplusC_check = abs( AplusC_check/(A-C) ) ;
D_check = abs( ( -2*A*cx-B*cy-D )/D ) ;
E_check = abs( (-B*cx-2*C*cy-E)/E ) ;
if AplusC_check > 1e-5 || D_check>1e-5 || E_check>1e-5
   % 无效
   a = []; 
   b = [];
   thita = [];
end

function [ a,b,thita ] = bBig( A,B,C,D,E,cx,cy )
thita = 0.5*atan2(B,A-C) ;
if thita==0
    a = sqrt(C);
    b = sqrt(A);
else
    a = sqrt( 0.5*(A+C-B/sin(2*thita)) ) ;
    b = sqrt( 0.5*(A+C+B/sin(2*thita)) ) ;
end

AplusC_check = A-C-(b^2-a^2)*( cos(thita)^2-sin(thita)^2)  ;
AplusC_check = abs( AplusC_check/(A-C) ) ;
D_check = abs( ( -2*A*cx-B*cy-D )/D ) ;
E_check = abs( (-B*cx-2*C*cy-E)/E ) ;
if AplusC_check > 1e-5 || D_check>1e-5 || E_check>1e-5
   % 无效
   a = []; 
   b = [];
   thita = [];
end