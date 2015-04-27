%% xyz 2015 3.18
% ellipse polynomial parameters to stand parameters

%%% input : cx,cy,a,b,thita
% [cx,cy] : circle of ellipse
% a : x semi-axes length
% b : y semi-axes length
% thita : rotation angle from (x,y) to (x'',y''). There is no rotation in the ellipse in (x'',y'').
%  unit : (rad)
%%% output ePolyolyoly = [ A, B, C, D, E, F ]
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0

function  [ A, B, C, D, E, F ] = ellipseStandToPoly(  cx,cy,thita,a,b  )
S = sin(thita);
C = cos(thita);
A = (a*S)^2+(b*C)^2 ;
B = 2*(b^2-a^2)*S*C ;
C = (a*C)^2+(b*S)^2 ;
D = -2*A*cx-B*cy ;
E = -B*cx-2*C*cy ;
F = A*cx^2+B*cx*cy+C*cy^2-a^2*b^2 ;
