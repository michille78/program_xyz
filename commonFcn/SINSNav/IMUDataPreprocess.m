%% xyz 2015.5.6

%% IMU data preprocess


function newAHRSData = IMUDataPreprocess( AHRSData,spanTime )

gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;


span = spanTime*frequency ;

gyro(1,:) = SmoothDataOne( gyro(1,:),span );
gyro(2,:) = SmoothDataOne( gyro(2,:),span );
gyro(3,:) = SmoothDataOne( gyro(3,:),span );

acc(1,:) = SmoothDataOne( acc(1,:),span );
acc(2,:) = SmoothDataOne( acc(2,:),span );
acc(3,:) = SmoothDataOne( acc(3,:),span );

for k=1:Nframes
    gyroNorm(k) = normest( gyro(:,k) );
    accNorm(k) = normest( acc(:,k) );
end

newAHRSData.gyro = gyro ;
newAHRSData.acc = acc ;
newAHRSData.gyroNorm = gyroNorm ;
newAHRSData.accNorm = accNorm ;
newAHRSData.frequency = frequency ;
newAHRSData.Nframes = Nframes ;


% DrawAHRSDataCompare( AHRSData,newAHRSData,'cmp' ) ;


function dataNew = SmoothDataOne( data,span )

method = 'sgolay';
dataNew = smooth( data,span,method,3 );




function DrawAHRSDataCompare( AHRSData,newAHRSData,dataName )

gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = (Nframes-1)/frequency ;
timeSample = 0:1/frequency:time;


gyroNew = newAHRSData.gyro ;
accNew = newAHRSData.acc ;
gyroNormNew = newAHRSData.gyroNorm ;
accNormNew = newAHRSData.accNorm ;


%% gyro
figure('name',[dataName,'-gyro'])
subplot(3,1,1)
plot(  timeSample,gyro(1,:)*180/pi )
hold on
plot(  timeSample,gyroNew(1,:)*180/pi,'r' )

ylabel('wx ^o/s')
title(get(gcf,'name'))
subplot(3,1,2)
plot( timeSample,gyro(2,:)*180/pi )
hold on
plot(  timeSample,gyroNew(2,:)*180/pi,'r' )

ylabel('wy ^o/s')
subplot(3,1,3)
plot( timeSample,gyro(3,:)*180/pi )
hold on
plot(  timeSample,gyroNew(3,:)*180/pi,'r' )
ylabel('wz ^o/s')
xlabel('sec')
%% gyro norm
figure('name',[dataName,'-gyroNorm'])
plot( timeSample,gyroNorm(1,:)*180/pi )
hold on
plot(  timeSample,gyroNormNew(1,:)*180/pi,'r' )
ylabel('gyro norm ^o/s')
title(get(gcf,'name'))
xlabel('sec')
%% acc
figure('name',[dataName,'-acc'])
subplot(3,1,1)
plot( timeSample,acc(1,:)*1000 )
hold on
plot(  timeSample,accNew(1,:)*1000,'r' )

ylabel('ax mg')
title(get(gcf,'name'))
subplot(3,1,2)
plot( timeSample,acc(2,:)*1000 )
hold on
plot(  timeSample,accNew(2,:)*1000,'r' )
ylabel('ay mg')
subplot(3,1,3)
plot( timeSample,acc(3,:)*1000 )
hold on
plot(  timeSample,accNew(3,:)*1000,'r' )
ylabel('az mg')
xlabel('sec')
%% acc norm
figure('name',[dataName,'-accNorm'])
plot( timeSample,accNorm(1,:)*1000 )
hold on
plot(  timeSample,accNormNew(1,:)*1000,'r' )
ylabel('acc norm mg')
title(get(gcf,'name'))
xlabel('sec')

