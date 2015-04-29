%% xyz  2015.4.29

%% rotate by quaternion
% Vb = Qab' o Va o Qab

function Vb = QuaternionRotate( Va,Qab )

Q_Vb = RightQMultiplyMatrix( Qab ) * LeftQMultiplyMatrix( Qinv(Qab) )  * [0;Va];

Vb = Q_Vb(2:4)  ;
