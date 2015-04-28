%% xyz 2015.4.28

% Qwb: [4*N]
% NavigationFrame : NED(North-East-Down) / ENT(East-North-Tian) geography frame

% Attitude  rad
    % Attitude.yaw   [1*N] rad
    % Attitude.pitch  
    % Attitude.roll
    
function Qnb = Attitude2Q( Attitude,NavigationFrame )
yaw = Attitude.yaw  ;
pitch = Attitude.pitch   ;
roll = Attitude.roll ;

euler = [ yaw;pitch;roll ];
        
switch NavigationFrame
    case 'NED'
        Qnb = Euler2Q( euler,'ZYX',[1,1,1] );
    case 'ENT'
        Qnb = Euler2Q( euler,'ZXY',[1,1,1] );        
end



