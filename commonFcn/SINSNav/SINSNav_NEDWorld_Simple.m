%% xyz 2015.4.27

%% SINS navigation


function SINSNav_NEDWorld_Simple( imuInputData,InitialData )

%% Load data
wibb = imuInputData.wib;        % [ 3*N ] rad/s
fb = imuInputData.f;            % [ 3*N ] m/s^2
imuFre = imuInputData.frequency;   % Hz

Qwb0 = InitialData.Qwb0 ;   % initial attitude quaternion
Vwb0 = InitialData.Vwb0 ;   % initial velocity
rwb0 = InitialData.rwb0 ;   % initial position

%% Simplied SINS navigation
% 1) ignore rotation of earth
% 2) navigation frame: the world frame / the North-East-Down geography frame
% 3) ignore moving of geography frame: geography frame == the world frame

function [ Qwb,Vwb,rwb ] = SINSNav_NEDWorld_Simple( wibb,fb,Qwb0,Vwb0,rwb0,imuFre, )

imuT = 1/imuFre ;
Wwbb = Wibb ;   % ignore rotation of earth
Nframes = size(wibb,2);
%% assigin initial value
Qwb = zeros(4,Nframes);
Vwb = zeros(3,Nframes);
rwb = zeros(3,Nframes);
Qwb(:,1) = Qwb0 ;
Vwb(:,1) = Vwb0 ;
rwb(:,1) = rwb0 ;

Cwb0 = Q2C( Qwb0 );
gw = Cwb0'*gb ;
%% SINS calculation

for k=1:Nframes-1
    %%% attitude update
    Qwb_Next  = QuaternionDifferential( Qwb(:,k),Wwbb(:,k),imuT ) ;
    Qwb(:,k+1) = Qwb_Next ;
    %%% velocity update  
    Cwb = Q2C(Qwb(:,k)) ;
    Cbw = Cwb' ;
    a_wbw = Cbw * fb(:,k) + gw;  
    Vwb(:,k+1) = Vwb(:,k)+a_wbw*imuT ;
    %%% position update 
    rwb(:,k+1) = rwb(:,k)+Vwb(:,k)*imuT ;
end


