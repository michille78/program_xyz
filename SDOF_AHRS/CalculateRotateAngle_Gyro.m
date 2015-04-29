%% xyz 2015.4.29

%% 纯陀螺积分解转角


function RotateAngle_Gyro = CalculateRotateAngle_Gyro( imuInputData,InitialData,Ypr )

Qwr = InitialData.Qwb0 ;
[ Qwb,Vwb,rwb,Attitude ] = SINSNav( imuInputData,InitialData ) ;
Qrb = LeftQMultiplyMatrix(  Qinv(Qwr) )*Qwb ;
Qrb_NormVector = GetNormVectorQ( Qrb ) ;

N = size(Qrb_NormVector,2);
RotateAngle_Gyro = zeros(1,N);
for k=1:N
    % 使 Qrb_NormVector 的矢量方向始终与 Ypr 一致
    if sign(Qrb_NormVector(2,k)) ~= sign(Ypr(1))
        Qrb_NormVector(:,k) = -Qrb_NormVector(:,k);
    end
   RotateAngle_Gyro(k)  = acot( Qrb_NormVector(1,k) )*2;
end

%% 平行因子
wibb = imuInputData.wibb; 

wibbNormed = zeros(3,N);
wibbNorm = zeros(1,N);
for k=1:N
    norm_k = normest(wibb(:,k)) ;
    wibbNormed(:,k) = wibb(:,k)/norm_k;
    wibbNorm(k) = norm_k ;
end

f_parallel = Ypr'*wibbNormed ;
f_parallel = abs(f_parallel);

%% 漂移因子
f_drift = zeros(1,N);
for k=1:N
    f_drift(k) = (f_parallel(k)-1)*wibbNorm(k);
end

%% 取角速度大数据
IsWibbBig = find( wibbNorm > 5*pi/180);
f_drift_BigWibb = f_drift(IsWibbBig);

f_drift_BigWibbMean = mean( f_drift_BigWibb )*180/pi;

timeData = 1:N ;
timeData_BigWibb = timeData(IsWibbBig);
%%

figure('name','f_parallel')
subplot(2,1,1)
plot(f_parallel)
ylabel('f_parallel')
subplot(2,1,2)
plot(wibbNorm*180/pi)
ylabel('角速度 /°')

figure('name','f_drift')
subplot(2,1,1)
plot(f_drift*180/pi)
hold on
plot(timeData_BigWibb,f_drift_BigWibb*180/pi,'r.')
ylabel('f_drift  °/s')
subplot(2,1,2)
plot(wibbNorm*180/pi)
ylabel('角速度 /°')

figure('name','f_drift_BigWibb')
plot(f_drift_BigWibb*180/pi)
ylabel('f\_drift  °/s')

disp('')
