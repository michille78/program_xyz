%% xyz 2015.4.23

%% left quaternion multiply matrix

% QoP = LQM*P

function LQM = LeftQMultiplyMatrix( Q )

Q = Q/normest(Q) ;
qs = Q(1) ;
qx = Q(2) ;
qy = Q(3);
qz = Q(4);
qv = [ qx ; qy ; qz ];

LQM = [ qs  -qv' ;  qv qs*eye(3)+getCrossMatrix(qv) ];



