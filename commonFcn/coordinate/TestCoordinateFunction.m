%% xyz 2015.4.8
% Test coordinate common functions



euler = [20 -30 150]'*pi/180 ;
Cab = Euler2C(euler,'YXZ',[1,1,1]);
Qab = C2Q(Cab);
Va = [1 -2 3]';


Vb1 = QuaternionRotate( Va,Qab ) 
Vb2 = Cab*Va





E = [ 30 -50 120; 40 20 -120 ;20 -20 122 ]*pi/180  ;
% E = [ 30 -50 120; 40 20 -120  ]*pi/180  ;
rotateOrder = 'XYZ';
rotateDirection = [1,1,1];
CosBeita = -1 ;

rotateOrder = [];
rotateDirection = [];

C = Euler2C( E,rotateOrder,rotateDirection);
E2 = C2Euler( C,rotateOrder,rotateDirection,CosBeita );
C2 = Euler2C(E2,rotateOrder,rotateDirection);
E3 = C2Euler( C2,rotateOrder,rotateDirection,CosBeita );

Eerr2 = E-E2 ;
Eerr3 = E-E3 ;
Eerr23 = E3-E2 ;

Q = C2Q(C);
E_Q = Q2Euler(Q,rotateOrder,rotateDirection,CosBeita);
C_Q = Q2C(Q);
E_C_Q = C2Euler(C_Q,rotateOrder,rotateDirection,CosBeita);

E_err = E_Q- ChangeEulerBeitaScope (E);
Eerr_CQ = E_C_Q- ChangeEulerBeitaScope( E )   ;
Eerr_Q = E_Q-E_C_Q ;



disp('ok')