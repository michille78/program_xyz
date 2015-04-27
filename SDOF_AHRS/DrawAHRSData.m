%% xyz 2015.4.23

function DrawAHRSData( AHRSData,dataName )

gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = (Nframes-1)/frequency ;
timeSample = 0:1/frequency:time;

%% gyro
figure('name',[dataName,'-gyro'])
subplot(3,1,1)
plot(  timeSample,gyro(:,1)*180/pi )
ylabel('wx ^o/s')
title(get(gcf,'name'))
subplot(3,1,2)
plot( timeSample,gyro(:,2)*180/pi )
ylabel('wy ^o/s')
subplot(3,1,3)
plot( timeSample,gyro(:,3)*180/pi )
ylabel('wz ^o/s')
xlabel('sec')
%% gyro norm
figure('name',[dataName,'-gyroNorm'])
plot( timeSample,gyroNorm(:,1)*180/pi )
ylabel('gyro norm ^o/s')
title(get(gcf,'name'))
xlabel('sec')
%% acc
figure('name',[dataName,'-acc'])
subplot(3,1,1)
plot( timeSample,acc(:,1)*1000 )
ylabel('ax mg')
title(get(gcf,'name'))
subplot(3,1,2)
plot( timeSample,acc(:,2)*1000 )
ylabel('ay mg')
subplot(3,1,3)
plot( timeSample,acc(:,3)*1000 )
ylabel('az mg')
xlabel('sec')
%% acc norm
figure('name',[dataName,'-accNorm'])
plot( timeSample,accNorm(:,1)*1000 )
ylabel('acc norm mg')
title(get(gcf,'name'))
xlabel('sec')
