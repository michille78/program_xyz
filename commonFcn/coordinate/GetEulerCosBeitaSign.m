%% xyz 2015.4.9

% get which kind of euler angles

function CosBeitaSign = GetEulerCosBeitaSign( euler )

euler = Make_N_Const( euler,3 ) ; 
Nframes = size( euler,1 );
CosBeitaSign = zeros( Nframes,1 );
for k=1:Nframes
   CosBeitaSign( k )  = sign( cos( euler(k,2) ) ) ;
end

