%% xyz 2015.4.27

%% �����ݻ��ּ���ת��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Ypr_Gyro,RecordStr ] = GetRotateVector_Gyro( imuInputData,InitialData,AHRSThreshod,SDOFStaticFlag )

RoateVectorCalMinAngleSecond = AHRSThreshod.RoateVectorCalMinAngleSecond ;

Qwr = InitialData.Qwb0 ;
[ Qwb,Vwb,rwb,Attitude ] = SINSNav( imuInputData,InitialData ) ;
Qrb = LeftQMultiplyMatrix(  Qinv(Qwr) )*Qwb ;
[ QAngle,QVectorNormed] = GetQAngle( Qrb ) ;

%% when rotate angle is too small , calcualtion error of Qrb_NormVector and Ypr_Gyro is too big
Index_BigAngle = abs(QAngle) < abs(RoateVectorCalMinAngleSecond );
Qrb_NormVector_BigAngle_V = QVectorNormed( :,Index_BigAngle );   % get big rotate angle data

Ypr_Gyro = mean( Qrb_NormVector_BigAngle_V,2) ;     % get the mean roate vector

Ypr_GyroStr = sprintf( '%0.5f  ',Ypr_Gyro );
RecordStr = sprintf( ' Ypr_Gyro = %s ;',Ypr_GyroStr );
disp(RecordStr);