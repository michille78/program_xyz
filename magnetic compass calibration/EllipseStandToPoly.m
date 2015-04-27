%% xyz 2015.4.13

% ePoly_Fitting(:,k) = [ A; B; C; D; E; F ]
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
% eStand_Fitting(:,k) = [ cx;cy;thita;a;b;e;f ]
function ePoly = EllipseStandToPoly( eStand )
[ cx,cy,thita,a,b,e,f ] = deal( eStand(1),eStand(2),eStand(3),eStand(4),eStand(5),eStand(6),eStand(7) );
A = a^2*sin(thita)^2 + b^2*cos(thita)^2 ;
B = 2*(b^2-a^2)*sin(thita)*cos(thita) ;
C = a^2*cos(thita)^2+b^2*sin(thita)^2 ;
D = -2*A*cx-B*cy ;
E = -B*cx-2*C*cy ;
F = A*cx^2+B*cx*cy+C*cy^2-a^2*b^2 ;
ePoly = [ A;B;C;D;E;F ];

