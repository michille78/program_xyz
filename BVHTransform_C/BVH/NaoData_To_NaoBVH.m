%% Input: rad   Output: rad      
% All have to be rad
% 按照 Nao 关节旋转顺序的BVH数据
function BVH_Nao = NaoData_To_NaoBVH(  HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw,  LElbowRoll, ...
           RShoulderPitch, RShoulderRoll, RElbowYaw , RElbowRoll, ...
           HipYawPitch, LHipRoll , LHipPitch , LKneePitch , RHipRoll , RHipPitch, ...
           RKneePitch , LAnklePitch , LAnkleRoll , RAnklePitch , RAnkleRoll, ...
           LWristYaw , RWristYaw , LHand , RHand, ROOT_Hips, BVHFormat,BVHHeadStr   )
       
Nframes = size( HeadYaw,2 );  

LShoulderPitch = -LShoulderPitch ;
RElbowYaw = -RElbowYaw ;
RWristYaw = -RWristYaw ;

% shoulder zero reference conpansate
E_RShoulderRoll_Zero_ZYX = [ 0,pi/2,0 ];
E_RShoulderRoll_Zero_ZYX = repmat( E_RShoulderRoll_Zero_ZYX,Nframes,1 );
C_RShoulderRoll_Zero_ZYX  =EulerToC_Multi( E_RShoulderRoll_Zero_ZYX,'ZYX' );
E_RShoulder = [ RShoulderPitch',RShoulderRoll',zeros(Nframes,1) ];
C_RShoulder = EulerToC_Multi( E_RShoulder,'ZYX'  );
C_RShoulder_New = CalculateC_Multi(  C_RShoulderRoll_Zero_ZYX,C_RShoulder );
E_RShoulder_New = CToEuler_Multi( C_RShoulder_New,'ZYX' );

RightArm  = E_RShoulder_New ;
% RShoulderPitch = E_RShoulder_New(:,1)' ;
% RShoulderRoll = E_RShoulder_New(:,2)' ;

%%%
E_LShoulderRoll_Zero_ZYX = [ 0,-pi/2,0 ];
E_LShoulderRoll_Zero_ZYX = repmat( E_LShoulderRoll_Zero_ZYX,Nframes,1 );
C_LShoulderRoll_Zero_ZYX  =EulerToC_Multi( E_LShoulderRoll_Zero_ZYX,'ZYX' );
E_LShoulder = [ LShoulderPitch',LShoulderRoll',zeros(Nframes,1) ];
C_LShoulder = EulerToC_Multi( E_LShoulder,'ZYX'  );
C_LShoulder_New = CalculateC_Multi( C_LShoulderRoll_Zero_ZYX,C_LShoulder );
E_LShoulder_New = CToEuler_Multi( C_LShoulder_New,'ZYX' );

% LShoulderPitch = E_LShoulder_New(:,1)' ;
% LShoulderRoll = E_LShoulder_New(:,2)' ;

LeftArm = E_LShoulder_New ;

EZeros = zeros( Nframes,1 );
Head = [ HeadYaw' HeadPitch' EZeros ];
% LeftArm = [ LShoulderPitch'  LShoulderRoll'  EZeros  ]; % -ZY
LeftForeArm = [ LElbowYaw' LElbowRoll'  EZeros ];
% RightArm = [ RShoulderPitch', RShoulderRoll',EZeros ];
RightForeArm = [ RElbowYaw' , RElbowRoll',EZeros ]; % -XY
LeftUpLeg = [ LHipRoll' , LHipPitch' ,EZeros  ];
LeftLeg = [ LKneePitch' ,EZeros,EZeros ];
RightUpLeg = [ RHipRoll' , RHipPitch' ,EZeros ];
RightLeg = [ RKneePitch',EZeros,EZeros ];
LeftFoot = [ LAnklePitch' , LAnkleRoll' ,EZeros ];
RightFoot = [ RAnklePitch' , RAnkleRoll' ,EZeros ];
LeftHand = [ LWristYaw', EZeros,EZeros];
RightHand = [ RWristYaw',EZeros,EZeros ];  % -X

%% 所有 59个BVH关节 欧拉角数据
% 无数据的给0
BVHdataN = length( BVHFormat );
for k=1:BVHdataN
   if exist( BVHFormat{k},'var' ) 
       eval( sprintf( 'data.%s=%s ;',BVHFormat{k},BVHFormat{k} ) );
   else
       eval( sprintf( 'data.%s=zeros(%d,3);',BVHFormat{k},Nframes ) );
   end
end

%% 关节旋转顺序 数据头文件
NaoRotateOrder.Head = 'YXZ';
NaoRotateOrder.LeftArm = 'ZYX';
NaoRotateOrder.LeftForeArm = 'XYZ';
NaoRotateOrder.RightArm = 'ZYX';
NaoRotateOrder.RightForeArm = 'XYZ';
NaoRotateOrder.LeftUpLeg = 'ZXY';
NaoRotateOrder.LeftLeg = 'XYZ';
NaoRotateOrder.RightUpLeg = 'ZXY';
NaoRotateOrder.RightLeg = 'XYZ';
NaoRotateOrder.LeftFoot = 'XZY';
NaoRotateOrder.RightFoot = 'XZY';
NaoRotateOrder.LeftHand = 'XZY';
NaoRotateOrder.RightHand = 'XZY';

field_NaoData = fieldnames( NaoRotateOrder );
N = length(field_NaoData);
for k=1:N
   order_k = eval( sprintf( 'NaoRotateOrder.%s ;',field_NaoData{k} ) );
   order_k_new = sprintf( 'CHANNELS 3 %srotation %srotation %srotation',order_k(1),order_k(2),order_k(3) );
   eval( sprintf( 'NaoRotateOrder.%s = order_k_new ;',field_NaoData{k} ) );
end

NaoJOINTS_OrderNum = zeros(1,N);  % Nao 关节点在 BVHHeadStr 的序号（不包括Hip）
for k=1:N
    str = sprintf('JOINT %s',field_NaoData{k} ) ;
    temp = strfind( BVHHeadStr,str );
    NaoJOINTS_OrderNum(k)= temp(1);
end
JOINTS_OrderNum = strfind( BVHHeadStr,'JOINT' );
CHANNELS_OrderNum = strfind( BVHHeadStr,'CHANNELS 3' );

[ NaoJOINTS_OrderNum_Sorted, NaoJointSort] = sort( NaoJOINTS_OrderNum ); % NaoJointSort:Nao 关节点在BVH中58个关节点的顺序（不包括Hip）


for k=1:N
    CHANNELS_k =  find(JOINTS_OrderNum==NaoJOINTS_OrderNum(k))  ;
    start_k = CHANNELS_OrderNum( CHANNELS_k );
    L = eval( sprintf('length( NaoRotateOrder.%s );',field_NaoData{k}) );
    BVHHeadStr( start_k:start_k+L-1 ) = eval(sprintf(' NaoRotateOrder.%s ',field_NaoData{k}))   ;
end
%% Update  Frames  Frame_Time
MOTION_Str_Start_k = strfind( BVHHeadStr,'MOTION' );
Frames_Str_Start_k = strfind( BVHHeadStr,'Frames: ' );
Frame_Time_Str_Start_k = strfind( BVHHeadStr,'Frame Time: ' );

FramesNum_Str = BVHHeadStr( Frames_Str_Start_k+7:Frame_Time_Str_Start_k-1 ) ;
FramesNum_old = str2double(FramesNum_Str);
Frame_Time_Str = BVHHeadStr( Frame_Time_Str_Start_k+11:length(BVHHeadStr)-1 ) ;
Frame_Time = str2double(Frame_Time_Str);

Frame_Time_New = Frame_Time*FramesNum_old/Nframes ;

BVHHeadStr = sprintf( '%sMOTION\n  \nFrames: %0.0f\nFrame Time: %0.2f\n',BVHHeadStr(1:MOTION_Str_Start_k-1),Nframes,Frame_Time_New );


BVH_Nao.data = data ;
BVH_Nao.BVHHeadStr = BVHHeadStr ;
BVH_Nao.BVHFormat = BVHFormat ;
BVH_Nao = UpdateBVHStruct( BVH_Nao ) ;

