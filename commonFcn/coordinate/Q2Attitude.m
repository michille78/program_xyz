%% xyz 2015.4.28

% Qwb: [4*N]
% NavigationFrame : NED(North-East-Down) / ENT(East-North-Tian) geography frame

% Attitude  rad
    % Attitude.yaw   [1*N] rad
    % Attitude.pitch  
    % Attitude.roll

function Attitude = Q2Attitude( Qwb,NavigationFrame )


switch NavigationFrame
    case 'NED'
        euler = Q2Euler( Qwb,'ZYX',[1,1,1],1 );
        
    case 'ENT'
        euler = Q2Euler( Qwb,'ZXY',[1,1,1],1 );
        
end

Attitude.yaw = euler(1,:);
Attitude.pitch = euler(2,:);
Attitude.roll = euler(3,:);
        