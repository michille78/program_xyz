%% xyz 2015.4.27

%% 纯陀螺积分计算转轴
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Ypr_Gyro,RecordStr ] = GetRotateVector_Gyro( imuInputData,InitialData,AHRSThreshod,SDOFStaticFlag )

RoateVectorCalMinAngleSecond = AHRSThreshod.RoateVectorCalMinAngleSecond ;

Qwr = InitialData.Qwb0 ;
[ Qwb,Vwb,rwb,Attitude ] = SINSNav( imuInputData,InitialData ) ;
Qrb = LeftQMultiplyMatrix(  Qinv(Qwr) )*Qwb ;
Qrb_NormVector = GetNormVectorQ( Qrb ) ;

%% when rotate angle is too small , calcualtion error of Qrb_NormVector and Ypr_Gyro is too big
Index_BigAngle = abs(Qrb_NormVector(1,:)) < abs(cot( RoateVectorCalMinAngleSecond/2 ));
Qrb_NormVector_BigAngle = Qrb_NormVector(:,Index_BigAngle);   % get big rotate angle data
Qrb_NormVector_BigAngle_V = Qrb_NormVector_BigAngle( 2:4,: );
% make Qrb_NormVector_BigAngle_V direction same
Qrb_NormVector_BigAngle_V = MakeVectorDirectionSame( Qrb_NormVector_BigAngle_V ) ;

Ypr_Gyro = mean( Qrb_NormVector_BigAngle_V,2) ;     % get the mean roate vector

Ypr_GyroStr = sprintf( '%0.5f  ',Ypr_Gyro );
RecordStr = sprintf( ' Ypr_Gyro = %s ;',Ypr_GyroStr );
disp(RecordStr);