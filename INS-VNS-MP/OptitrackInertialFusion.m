%% xyz 2015 5.25

%% otherMakers
% otherMakers.frequency
% otherMakers.Position
% otherMakers.otherMakersN

%% InertialData
% InertialData.frequency 
% InertialData.time 
% InertialData.HipQuaternion
% InertialData.HipPosition 
% InertialData.HeadQuaternion 
% InertialData.HeadPosition 
% InertialData.BodyDirection
% InertialData.DataStyle


function OptitrackInertialFusion(  )

dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\Head\Head1';
InertialData = importdata( [dataFolder,'\InertialData.mat'] );
otherMakers = importdata( [dataFolder,'\otherMakers.mat'] );
otherMakers = PreProcess( otherMakers,InertialData.BodyDirection );

%% otherMakers 预处理

% dbstop in GetRightOtherMaker
trackedMakerPosition = GetRightOtherMaker( otherMakers,InertialData ) ;



disp('')


function otherMakers = PreProcess( otherMakers,BodyDirection )
%% 先转到北东地的同名坐标系
Cv_r1 = [ 0 0 1; -1 0 0; 0 -1 0 ];
Cr_r1 = BodyDirection2Cr_r1( BodyDirection );
N = length( otherMakers );
Position_1 = otherMakers(1).Position(:,1) ;
Position_1(2) = 0;      % 高度方向不补偿
for k=1:N
    Position_k = otherMakers(k).Position ;
    m = size(Position_k,2);
    Position_1_m = repmat(Position_1,1,m);
    Cvr = Cr_r1' * Cv_r1 ;
    Position_k_new = Cvr*(Position_k-Position_1_m) ;
    otherMakers(k).Position = Position_k_new; 
end

%% 根据 BodyDirection 求姿态
% 符号有待验证！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
function Cr_r1 = BodyDirection2Cr_r1( BodyDirection )
thita = acos( [1 0 0]*BodyDirection / normest(BodyDirection)   );
if BodyDirection(2)<0
    thita = -thita ;
end
Cr_r1 = RotateZ( thita ) ;

