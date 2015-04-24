%% xyz 2015.4.22 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% accData: Nframes*3 
% NED navigation frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch: Nframes*3  
% roll: Nframes*3 

%%
function [ pitch,roll,Q_nb ] = Acc2PitchRoll( accData )

accData = Make_N_Const(accData,3);
Nframes = size(accData,1);
pitch = zeros( 1,Nframes );
roll = zeros( 1,Nframes );
for k=1:Nframes
    [ pitch(k),roll(k) ] = Acc2PitchRoll_One( accData(k,:) ) ;
end

%%% Q_nb
euler_nb_ZYX  = [ zeros(1,Nframes); pitch; roll ];
Q_nb = Euler2Q( euler_nb_ZYX,'ZYX',[1,1,1] );


function [ pitch,roll ] = Acc2PitchRoll_One( accData_k )
[ gx_b,gy_b,gz_b ] = deal( accData_k(1),accData_k(2),accData_k(3) );

pitch = asin( -gx_b );
roll = atan2( gy_b,gz_b );
