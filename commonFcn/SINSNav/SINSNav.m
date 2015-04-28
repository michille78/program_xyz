%% xyz 2015.4.27

%% SINS navigation


function [ Qwb,Vwb,rwb,Attitude ] = SINSNav( imuInputData,InitialData )

%% Load data
wibb = imuInputData.wibb;        % [ 3*N ] rad/s
fb_g = imuInputData.fb_g;            % [ 3*N ] g
imuFre = imuInputData.frequency;   % Hz

Qwb0 = InitialData.Qwb0 ;   % initial attitude quaternion
Vwb0 = InitialData.Vwb0 ;   % initial velocity
rwb0 = InitialData.rwb0 ;   % initial position
NavigationFrame = InitialData.NavigationFrame ;

%% const data
earthConst = getEarthConst;
gNorm = earthConst.gNorm ;

%% Calcualtion
[ Qwb,Vwb,rwb ] = SINSNav_World_Simple( wibb,fb_g,Qwb0,Vwb0,rwb0,imuFre,gNorm,NavigationFrame );
Attitude = Q2Attitude( Qwb,NavigationFrame );

%% Simplied SINS navigation
% 1) ignore rotation of earth
% 2) navigation frame: the world frame / the initial geography frame
% 3) ignore moving of geography frame: geography frame == the world frame
%%% Input
% NavigationFrame : NED(North-East-Down) / ENT(East-North-Tian) geography frame

function [ Qwb,Vwb,rwb ] = SINSNav_World_Simple( wibb,fb_g,Qwb0,Vwb0,rwb0,imuFre,gNorm,NavigationFrame )

imuT = 1/imuFre ;
Wwbb = wibb ;   % ignore rotation of earth
fb = fb_g*gNorm ;
Nframes = size(wibb,2);
%% assigin initial value
Qwb = zeros(4,Nframes);
Vwb = zeros(3,Nframes);
rwb = zeros(3,Nframes);
Qwb(:,1) = Qwb0 ;
Vwb(:,1) = Vwb0 ;
rwb(:,1) = rwb0 ;

switch NavigationFrame
    case 'NED'
        gw = [ 0 0 1 ]'*gNorm ;      % NED
    case 'ENT'
        gw = [ 0 0 -1 ]'*gNorm ;      % ENT
end
%% SINS calculation

for k=1:Nframes-1
    %%% attitude update
    Qwb(:,k+1)  = QuaternionDifferential( Qwb(:,k),Wwbb(:,k),imuT ) ;
    %%% velocity update  
    Cwb = Q2C(Qwb(:,k)) ;
    Cbw = Cwb' ;
    a_wbw = Cbw * fb(:,k) + gw;  
    Vwb(:,k+1) = Vwb(:,k)+a_wbw*imuT ;
    %%% position update 
    rwb(:,k+1) = rwb(:,k)+Vwb(:,k)*imuT ;
end


