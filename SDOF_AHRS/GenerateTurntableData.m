%% xyz 2015.4.24

%% 生成转台角度驱动数据

% 实验流程：
% （1）零位时保持静止1S以上
% （2）以20°/s角速度速度运动到30°，以-20°/s角速度速度运动到-30°，以20°/s角速度速度运动到20°，静止0.5S。
% （3）从20°以2°/s角速度速度运动到30°保持静止0.5S。以-20°/s角速度速度运动到0°，保持静止0.5S。
% （4）以0.1HZ  30°幅值进行正弦运动，每运动3个周期保持静止1S。运动2分钟后停止。

function GenerateTurntableData(  )

%%
dataFre = 1000 ;            % 输出数据频率

ZeroStaticTime = 1 ;        % 零位静止时长
FastRotateW = 20 ;          % 快速转动角速度
MaxRotateAmplitude = 30 ;  % 快速转动幅值
FastToSlowTime = 0.5 ;      % 快速慢速转动间隔
SlowRotateW = 2 ;           % 慢速转动角速度
SlowRotateMin = 20 ;        % 慢速转动最小值   ： SlowRotateMin 到 MaxRotateAmplitude
BeforeSinStaticTime = 0.5 ;       % 启动正弦前的静止时间
SinAmplitude = 30;          % 正弦转动幅值
SinFrequency = 0.1;         % 正弦转动频率
SinStaticTime = 0.5 ;         % 每3个周期正弦转动保持静止的时长

ThreeSinNum = 4 ;           % 正弦转动周期数 = ThreeSinNum*3


%% (1)
N1 = ZeroStaticTime*dataFre ;
data1_Static = zeros( 1,N1 );

%% (2)
FastRotateStep = FastRotateW / dataFre ;
data2_1 = 0:FastRotateStep:MaxRotateAmplitude ;
data2_2 = MaxRotateAmplitude:-FastRotateStep:-MaxRotateAmplitude ;
data2_3 = -MaxRotateAmplitude:FastRotateStep:SlowRotateMin ;
data2_4 = ones( 1,FastToSlowTime*dataFre )*SlowRotateMin ;

data2 = [ data2_1 data2_2 data2_3 data2_4 ];

%% (3)
SlowRotateStep = SlowRotateW / dataFre ;
data3_1 = SlowRotateMin:SlowRotateStep:MaxRotateAmplitude ;
data3_2 = MaxRotateAmplitude:-FastRotateStep:0 ;
data3_3 = zeros( 1,BeforeSinStaticTime*dataFre ) ;

data3 = [ data3_1 data3_2 data3_3  ];

%% (4)
sinStep = 2*pi/ ( dataFre/SinFrequency ) ;
sinTime = (0:sinStep:6*pi) ;
data4_sin = sin( sinTime )*SinAmplitude;

data4_Static = zeros( 1,SinStaticTime*dataFre );
data4 = [ data4_sin data4_Static ];
data4 = repmat( data4,1,ThreeSinNum );
%%
data = [ data1_Static data2  data3 data4 ];

dataNumber  = length(data) ;
sprintf('%0.0f',dataNumber)
sprintf('%0.2f',dataNumber/60/1000)

dataFolder = [ pwd,'\TurntableData1' ];
if ~isdir(dataFolder)
    mkdir(dataFolder)
else
   delete([dataFolder,'\*']) 
end
fid = fopen( [dataFolder,'\TurntableData.txt'],'w' );

fprintf( fid,'%0.3f\n',data );
fclose(fid);


save data data


%%
time = (1:length(data))/dataFre ;
figure 
plot(time,data)

disp('')
