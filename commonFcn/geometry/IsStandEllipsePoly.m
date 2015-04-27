

function berr = IsStandEllipsePoly( ePoly )


[ A,B,C,D,E,F ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) ) ;

K = [ -2*A,  -B
      -B,   -2*C ] ;
center = K \ [D;E];
cx = center(1) ;
cy = center(2) ;

%% a>b
%%% A method
t = A*cx^2+B*cx*cy+C*cy^2-F ;
b1s(1) =  ( A+C-sqrt( (A+C)^2-4*t ) )/2 ;

%%% B method
thita = 0.5*atan2(-B,C-A) ;
b2s(1) = ( A+C+B/sin(2*thita) )/2 ;

%% a<b
%%% A method
t = A*cx^2+B*cx*cy+C*cy^2-F ;
b1s(2) =  ( A+C+sqrt( (A+C)^2-4*t ) )/2 ;

%%% B method
thita = 0.5*atan2(B,A-C) ;
b2s(2) = ( A+C+B/sin(2*thita) )/2 ;


berr = (b1s-b2s)./b1s ;
