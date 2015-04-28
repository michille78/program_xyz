%% xyz 2015.4.22 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% accData: Nframes*3 
% NED navigation frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch: Nframes*3  
% roll: Nframes*3 

%%
function [ pitch,roll,Q_nb ] = Acc2PitchRoll( accData,NavigationFrame )

accData = Make_Const_N(accData,3);
Nframes = size(accData,2);
pitch = zeros( 1,Nframes );
roll = zeros( 1,Nframes );
for k=1:Nframes
    [ pitch(k),roll(k) ] = Acc2PitchRoll_One( accData(:,k) ) ;
end

%%% Q_nb
attitude.yaw = zeros(1,Nframes) ;
attitude.pitch  = pitch ;
attitude.roll = roll ;

Q_nb = Attitude2Q(attitude,NavigationFrame);


function [ pitch,roll ] = Acc2PitchRoll_One( accData_k )
[ gx_b,gy_b,gz_b ] = deal( accData_k(1),accData_k(2),accData_k(3) );

pitch = asin( -gx_b );
roll = atan2( gy_b,gz_b );
