%% xyz 2015.4.9
% There are two kinds of Euler Angles from one attitude to another. One C
% can get two Euler Angles. The root difference of the two Euler Angles is that
% the beita is different,
%   euler1         1) cos(beita)>0, beita [ -pi/2,pi/2 ]
%   euler2         2) cos(beita)<0, beita [ -pi,-pi/2 ] or [ pi/2,pi ]
%   relationship: 

%   euler2(1) = pi+euler1(1)
%   euler2(2) = pi-euler1(2)
%   euler2(3) = pi+euler1(3)

%   euler1(1) = pi+euler2(1)
%   euler1(2) = pi-euler2(2)
%   euler1(3) = pi+euler2(3)

function euler_new = ChangeEulerBeitaScope( euler )

euler = Make_N_Const( euler,3 ) ; 
Nframes = size( euler,1 );
euler_new = zeros( Nframes,3 );
for k=1:Nframes
   euler_new(k,:) = ChangeEulerBeitaScope_One( euler(k,:) ) ;
end
euler_new = MakeEuler_In2Pi( euler_new ) ;

function euler_new = ChangeEulerBeitaScope_One( euler )

euler_new = zeros( size(euler) );
euler_new(1) = pi+euler(1) ;
euler_new(2) = pi-euler(2) ;
euler_new(3) = pi+euler(3) ;