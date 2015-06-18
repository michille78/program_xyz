

Q1 =[0.2102;-0.73679;0.127618;0.6298];
Q2 =[ 0.1857;-0.69898;0.18415;0.6656 ];

E1 = Q2Euler( Q1,'YXZ',1,1 )*180/pi
E2 = Q2Euler( Q2,'YXZ',1,1 )*180/pi


% zd = [1 0 0]';
% rotateAngle = [ 40 30 40 ];
% bone_w = GenThumbDirection( rotateAngle,zd,'Left','A' ) ;

