%% From ellipse polynomial parameters to magnetometer error parameters
%%% input ePoly = [ A, B, C, D, E, F ]
% A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
%%% Output: [M,b ]
% Hm_b: b frame H with errors
% H_b : b frame H without errors
% Hm_b = M*H_b+b   H_b = G*(Hm_b-b)   G=inv(M)
% K=G'*G    

% [ref] JIANCHENG F, HONGWEI S, JUANJUAN C, et al. A Novel Calibration Method of Magnetic Compass Based on Ellipsoid Fitting [J].
% Instrumentation and Measurement, IEEE Transactions on, 2011, 60(6): 2053-61.


function  [G,b,H_b_Norm] =  EllipsePolyToMagnErrorModel_2D( ePoly )

[ A, B, C, D, E, F ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) ) ;
K = [ A  B/2; B/2  C ];
G = chol(K);
b  = ((-2*K)')\[D;E]; 
H_b_Norm = sqrt( b'*K*b-F ) ;
