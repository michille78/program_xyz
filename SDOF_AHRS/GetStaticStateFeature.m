%% xyz 2015.4.23

%% analysis the static feature
% Input data of static state
function meam_gyro_rad = GetStaticStateFeature( AHRSData,staticNum )
gyro = AHRSData.gyro ;
acc = AHRSData.acc ;
gyroNorm = AHRSData.gyroNorm ;
accNorm = AHRSData.accNorm ;
frequency = AHRSData.frequency ;
Nframes = AHRSData.Nframes ;
time = (Nframes-1)/frequency ;
timeSample = 0:1/frequency:time;
timeSample = timeSample(1:staticNum) ;

gyro = gyro( 1:staticNum,: ) ;
acc = acc( 1:staticNum,: ) ;
gyroNorm = gyroNorm( 1:staticNum,: ) ;
accNorm = accNorm( 1:staticNum,: ) ;

meam_gyro_rad = mean( gyro,1 );
meam_gyro = meam_gyro_rad*180/pi;
meam_gyro_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',meam_gyro(1),meam_gyro(2),meam_gyro(3) );
std_gyro = std( gyro,1 )*180/pi;
std_gyro_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',std_gyro(1),std_gyro(2),std_gyro(3) );

max_gyro = max(gyro)*180/pi;
max_gyro_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',max_gyro(1),max_gyro(2),max_gyro(3) );
min_gyro = min( gyro )*180/pi;
min_gyro_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',min_gyro(1),min_gyro(2),min_gyro(3) );
maxPlusMin_gyro = max_gyro-min_gyro ;
maxPlusMin_gyro_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',maxPlusMin_gyro(1),maxPlusMin_gyro(2),maxPlusMin_gyro(3) );

mean_gyroNorm = mean(gyroNorm,1)*180/pi;
mean_gyroNorm_Str = sprintf( '%0.2f',mean_gyroNorm );

meam_acc = mean( acc,1 )*1000;
meam_acc_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',meam_acc(1),meam_acc(2),meam_acc(3) );
accErr = acc-repmat(meam_acc/1000,size(acc,1),1);
std_acc = std( accErr,1 )*1000;
std_acc_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',std_acc(1),std_acc(2),std_acc(3) );

max_acc = max( ( acc ) )*1000;
max_acc_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',max_acc(1),max_acc(2),max_acc(3) );
min_acc = min( ( acc ) )*1000;
min_acc_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',min_acc(1),min_acc(2),min_acc(3) );
maxAccErr = max(abs(accErr))*1000 ;
maxAccErr_Str = sprintf( '( %0.2f, %0.2f, %0.2f )',maxAccErr(1),maxAccErr(2),maxAccErr(3) );


staticStr1 = sprintf( ' meam_gyro = %s бу/s \n std_gyro = %s бу/s \n mean_gyroNorm = %s бу/s \n meam_acc = %s mg \n std_acc = %s mg \n',meam_gyro_Str,std_gyro_Str,mean_gyroNorm_Str,meam_acc_Str,std_acc_Str );
staticStr2 = sprintf( ' max_gyro = %s бу/s \n min_gyro = %s бу/s \n maxPlusMin_gyro = %s бу/s \n max_acc = %s mg \n min_acc = %s mg \n maxAccErr = %s mg \n',...
    max_gyro_Str,min_gyro_Str,maxPlusMin_gyro_Str,max_acc_Str,min_acc_Str,maxAccErr_Str );
display(staticStr1)
display(staticStr2)

DrawAHRSData( AHRSData,'static' ) ;

figure('name',['static','-accErr'])
subplot(3,1,1)
plot( timeSample,accErr(:,1)*1000 )
ylabel('axErr mg')
title(get(gcf,'name'))
subplot(3,1,2)
plot( timeSample,accErr(:,2)*1000 )
ylabel('ayErr mg')
subplot(3,1,3)
plot( timeSample,accErr(:,3)*1000 )
ylabel('azErr mg')

disp('')

