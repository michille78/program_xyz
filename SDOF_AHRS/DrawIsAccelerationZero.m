%% xyz 2015.4.23
% 


function DrawIsAccelerationZero( AHRSData,IsAccelerationZero,dataName,RefRotateAngle_1000HZ ) 
gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = (Nframes-1)/frequency ;
timeSample = 0:1/frequency:time;

staticOrder = find(IsAccelerationZero==1);

timeSampleStatic = timeSample(staticOrder);
gyroStatic = gyro(:,staticOrder);
accStatic = acc(:,staticOrder);
gyroNormStatic = gyroNorm(:,staticOrder);
accNormStatic = accNorm(:,staticOrder);

RefN = fix ( length(RefRotateAngle_1000HZ) * frequency /1000 ) ;
RefRotateAngle = zeros(1,RefN);
for k=1:RefN
    RefRotateAngle(k) = RefRotateAngle_1000HZ( fix( (k-1)*1000/frequency )+1 );
end
RefTime = 1:RefN ;

%% gyro
figure('name',[dataName,'-gyro'])
subplot(4,1,1)
plot(  timeSample,gyro(1,:)*180/pi,'.b' )
hold on
plot(  timeSampleStatic,gyroStatic(1,:)*180/pi,'.r' )
ylabel('wx ^o/s')
title(get(gcf,'name'))
subplot(4,1,2)
plot( timeSample,gyro(2,:)*180/pi,'.b' )
hold on
plot(  timeSampleStatic,gyroStatic(2,:)*180/pi,'.r' )
ylabel('wy ^o/s')
subplot(4,1,3)
plot( timeSample,gyro(3,:)*180/pi,'.b' )
hold on
plot(  timeSampleStatic,gyroStatic(3,:)*180/pi,'.r' )
ylabel('wz ^o/s')

subplot(4,1,4)
plot( RefTime,RefRotateAngle*180/pi,'.b' )

ylabel('ref ^o')

xlabel('sec')
%% gyro norm
figure('name',[dataName,'-gyroNorm'])
plot( timeSample,gyroNorm(1,:)*180/pi,'.b' )
hold on
plot(  timeSampleStatic,gyroNormStatic(1,:)*180/pi,'.r' )
ylabel('gyro norm ^o/s')
title(get(gcf,'name'))
xlabel('sec')
%% acc
figure('name',[dataName,'-acc'])
subplot(4,1,1)
plot( timeSample,acc(1,:)*1000,'.b' )
hold on
plot( timeSampleStatic,accStatic(1,:)*1000,'.r' )
ylabel('ax mg')
title(get(gcf,'name'))
subplot(4,1,2)
plot( timeSample,acc(2,:)*1000,'.b' )
hold on
plot( timeSampleStatic,accStatic(2,:)*1000,'.r' )
ylabel('ay mg')
subplot(4,1,3)
plot( timeSample,acc(3,:)*1000,'.b' )
hold on
plot( timeSampleStatic,accStatic(3,:)*1000,'.r' )
ylabel('az mg')
subplot(4,1,4)
plot( RefTime,RefRotateAngle*180/pi,'.b' )

ylabel('ref ^o')

xlabel('sec')
%% acc norm
figure('name',[dataName,'-accNorm'])
plot( timeSample,accNorm(1,:)*1000,'.b' )
hold on
plot( timeSampleStatic,accNormStatic(1,:)*1000,'.r' )
ylabel('acc norm mg')

title(get(gcf,'name'))
xlabel('sec')