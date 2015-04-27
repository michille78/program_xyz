%% xyz 2015.4.23

%% right quaternion multiply matrix

% PoQ = RQM*P

function RQM = RightQMultiplyMatrix( Q )

Q = Q/normest(Q) ;
qs = Q(1) ;
qx = Q(2) ;
qy = Q(3);
qz = Q(4);
qv = [ qx ; qy ; qz ];

RQM = [ qs  -qv' ;  qv qs*eye(3)-getCrossMatrix(qv) ];


