%% xyz 2015.6.24

%% otherMakers
% otherMakers(k).Position
% otherMakers(k).otherMakersN
%　otherMakers(k).time

% otherMakers(k).frequency
% otherMakers(k).MarkerSet

%     otherMakers(k).ContinuesFlag = NaN;
%     otherMakers(k).ContinuesLastPosition = NaN(3,1);
%     otherMakers(k).ContinuesLastTime = NaN;
%     otherMakers(k).ContinuesLastK = NaN;
%                     ContinuesLasti

%     otherMakers(k).CalculatedTime  计算次数 初始=0

%% otherMakersContinues  存储当前最新的连续线段，排序与当前的　otherMakers(k).Position　保持一致
% otherMakersContinues.otherMakersN  马克点个数
% otherMakersContinues.dataN [5,20]  有效数据长度  dataN [1,iMarker]
    % 是第iMarker个点的位置有效长度； dataN [2,iMarker]是速度
% otherMakersContinues.ConP1 2 3...

%% 计算光学的加速度

function AnalyzeVisualAcc(  )
global dataFolder
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\摔头1';
%　dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head6';
otherMakers = ReadOptitrack( dataFolder,'\Opt.txt' );

otherMakers = FullotherMakersField( otherMakers );

visualN = size(otherMakers,2);
global visionFre   

%% 参数与输出预设
trackedMakerPosition = NaN(3,visualN);  % 无跟踪情况下的连续性
visionFre = otherMakers(1).frequency;
otherMakersContinues = Initial_otherMakersContinues( visualN );

[ makerTrackThreshold,INSVNSCalibSet ] = SetConstParameters( visionFre );
waveThreshold_VNSAcc = SetWaveThresholdParameters( 'VNSAcc' );

parametersSet.waveThreshold_VNSAcc = waveThreshold_VNSAcc;
parametersSet.INSVNSCalibSet = INSVNSCalibSet;
%%  开始
otherMakersN1 = otherMakers(1).otherMakersN ;
otherMakers(1).ContinuesFlag = zeros(1,otherMakersN1);

otherMakers(1) = PreProcess_otherMakers( otherMakers(1)  );

for k_vision=2:visualN

    otherMakers(k_vision) = PreProcess_otherMakers( otherMakers(k_vision)  );
    otherMakers_k_last = otherMakers(k_vision-1);
    [ otherMakers(k_vision),dPi_ConJudge ] = ContinuesJudge( otherMakers(k_vision),otherMakers_k_last,trackedMakerPosition,...
        k_vision,makerTrackThreshold );
    
        % 以上更新了 k_vision 时刻的马克点连续性信息，
        %% 利用 otherMakers(k_vision) 更新 当前的连续曲线 
        % 每个时刻将 otherMakersContinues 按照新的 otherMakers 序号进行排序，并更新顺序更新方法
    ContinuesLasti_All = otherMakers(k_vision).ContinuesLasti ;  % 当前时刻 所有马克点 对应的 上时刻马克点序号
    % 先将 otherMakersContinues 按照新的马克点序号排序
    otherMakersContinues = ReOrderContinues( ContinuesLasti_All,otherMakersContinues ) ;
    otherMakersN = otherMakers(k_vision).otherMakersN ;
    for i_marker=1:otherMakersN
        
        [ otherMakersContinues ] = AnalyzeContinues( otherMakers,k_vision,i_marker,otherMakersContinues,parametersSet ) ;
        if k_vision==658
            Draw_otherMakersContinues( otherMakersContinues,i_marker,visionFre,k_vision );
        end
    end

   
end

[ k_vision_L,maker_i_L,maxConN ] = FindLongestLine( otherMakers );
% GetContinues( otherMakers,k_vision_L,maker_i_L,INSVNSCalibSet );

% Draw_otherMakersContinues( otherMakersContinues,maker_i_L,visionFre,k_vision_L );

disp('')

%% 实时 分析马克点的 连续 位置曲线、 速度曲线、 加速度曲线、 加速度曲线的波形
% 只计算一个时刻

function [ otherMakersContinues ] = AnalyzeContinues( otherMakers,k_vision,i_marker,otherMakersContinues,parametersSet )

global visionFre  
persistent A_k_waves_OKLast
if isempty(A_k_waves_OKLast)
    A_k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）
end

waveThreshold_VNSAcc = parametersSet.waveThreshold_VNSAcc ;
INSVNSCalibSet = parametersSet.INSVNSCalibSet;
%% 更新连续线段 位置

[~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker ); % 更新前的连续线段
dataN_i_PValid = otherMakersContinues.dataN( 1,i_marker );  % 第 i_marker 马克点 位置有效长度

[ ConPosition_i,dataN_i_P_new] = GetContinuesPosition_2...
    ( otherMakers,k_vision,i_marker,ConPosition_i,dataN_i_PValid );
% 将新增加的点 NewPosition_i 更新到  otherMakersContinues
otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,ConPosition_i,1,dataN_i_P_new );

if dataN_i_P_new>0
    %% 计算马克点速度
    [ Velocity_k,k_calV ] = CalVelocity( ConPosition_i,dataN_i_P_new,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle ) ;
    if k_calV>0  && ~isnan(Velocity_k(1))
        ConVelocity_i(:,k_calV) = Velocity_k ;
        
        otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,ConVelocity_i,2,k_calV );
        
         %% 计算马克点加速度
        [ acc_k,k_calA ] = CalVelocity( ConVelocity_i(1:3,:),k_calV,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle ) ;
       if k_calA>0  && ~isnan(acc_k(1))
 %% ？？？？？？？？ 不知为何反号了 ？？？？？？？？？？？？？？？？？？？？？？？？？
            acc_k = -acc_k;  
            ConAcc_i(:,k_calA) = acc_k ;
            otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,ConAcc_i,3,k_calA );
            
           %% ConAcc 波形分析
            ConAcc_i_valid = ConAcc_i(1:3,1:k_calA) ;
            A_WaveFlag  = AWave( (14:16)-13,:);  
            A_V = AWave((17:21)-13,:); 
            A_Acc_waveFront = AWave((22:24)-13,:); 
            A_Acc_waveBack = AWave((25:27)-13,:);   
              %% 启动搜索
            
             [ A_WaveFlag,A_V,A_Acc_waveFront,A_Acc_waveBack,A_k_waves_OKLast ] = AnalyzeWave...
                ( ConAcc_i_valid,k_calA,visionFre,A_V,A_WaveFlag,A_k_waves_OKLast,A_Acc_waveFront,A_Acc_waveBack,waveThreshold_VNSAcc );
            
            AWave = [  A_WaveFlag;A_V; A_Acc_waveFront; A_Acc_waveBack ];
            otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,AWave,4,max(A_k_waves_OKLast) );
       end
           
    end
      
end




%% 寻找 otherMakers 中第 k_vision 个时刻， 第 i_marker 个马克点 对应的连续线段（在前时刻基础上查找）
% ConPosition_i [3*ConN_i] ConN_i 是该连续线段的长度
function [ ConPosition_i_new,dataN_i_P_new] = GetContinuesPosition_2...
    ( otherMakers,k_vision,i_marker,ConPosition_i,dataN_i_PValid )
ContinuesFlag = otherMakers(k_vision).ContinuesFlag(i_marker) ;
ContinuesLasti_All = otherMakers(k_vision).ContinuesLasti ;  % 当前时刻 所有马克点 对应的 上时刻马克点序号
ContinuesLasti_cur = ContinuesLasti_All(i_marker) ;  % 当前时刻 当期马克点 对应的 上时刻马克点序号
if isnan(ContinuesFlag) || ContinuesFlag==0 || isnan(ContinuesLasti_cur) % 无2个以上连续马克点
    ConPosition_i_new = ConPosition_i;
    dataN_i_P_new = dataN_i_PValid;
    return;
end

% 先将 otherMakersContinues 按照新的马克点序号排序
% otherMakersContinues = ReOrderContinues( ContinuesLasti_All,otherMakersContinues ) ;

ContinuesLastK = otherMakers(k_vision).ContinuesLastK(i_marker) ;  % 连续线段最早点的时刻
ConN_i = k_vision-ContinuesLastK+1 ; % 连续线段长度

NewPosition_i = otherMakers(k_vision).Position(:, i_marker );  % 最新的点
% [~,ConPosition_i] = Read_otherMakersContinues_i( otherMakersContinues,i_marker ); % 更新前的连续线段
% dataN_i_P = otherMakersContinues.dataN( 1,i_marker );  % 第 i_marker 马克点 位置有效长度
if ConN_i == 2
   %% 新的线段开始 2个点
   NewPosition_i_2 = otherMakers(k_vision-1).Position(:, ContinuesLasti_cur );   % 前一个（第一个点）
   NewPosition_i = [ NewPosition_i_2 NewPosition_i ];
   ConPosition_i_new( :,1:2 ) = NewPosition_i;   % 更新后的连续线段
   dataN_i_P_new = 2;
   if dataN_i_PValid ~=0  % 正确的情况下，应该不存在，返回空
      disp('wrong-1 in GetContinuesPosition_2') 
   end
else
    %% 延续前时刻的线段 增加一个点
    
    [ ConPosition_i_new,dataN_i_P_new ] = AddListData( ConPosition_i,dataN_i_PValid,NewPosition_i );
end
% 检查长度
if dataN_i_P_new ~= ConN_i
    disp('wrong-2 in GetContinuesPosition_2') 
end
    
% % 将新增加的点 NewPosition_i 更新到  otherMakersContinues
% otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,ConPosition_i_new,1,dataN_i_P_new );
% ConPosition_i_new_Valid = ConPosition_i_new( :,1:ConN_i );

%%  (非实时)计算 第 k_vision 个时刻 第 i_marker 个马克点 对应连续曲线的 速度曲线 加速度曲线

function [ ConPosition_i,ConVelocity,ConAcc,ConN ] = GetContinues( otherMakers,k_vision,i_marker,INSVNSCalibSet )
global visionFre  
[ ConPosition_i,ConN] = GetContinuesPosition( otherMakers,k_vision,i_marker );

% [ otherMakersContinues,ConPosition_i,ConN] = GetContinuesPosition_2...
%     ( otherMakers,k_vision,i_marker,otherMakersContinues );

ConVelocity = NaN(5,ConN);           
for j=1:ConN
    %% 计算马克点速度
   [ Velocity_k,k_calV ] = CalVelocity( ConPosition_i,j,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle ) ;
   if k_calV>0  && ~isnan(Velocity_k(1))
        ConVelocity(:,k_calV) = Velocity_k ;
   end
end


ConAcc = NaN(5,ConN);
for j=1:ConN
    %% 计算马克点加速度
   [ acc_k,k_calA ] = CalVelocity( ConVelocity(1:3,:),j,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle ) ;
   if k_calA>0  && ~isnan(acc_k(1))
        ConAcc(:,k_calA) = acc_k ;
   end
end
%% ？？？？？？？？ 不知问反号了
ConAcc = -ConAcc;  


    %% ConAcc 波形分析
waveThreshold_VNSAcc = SetWaveThresholdParameters( 'VNSAcc' );

VNSA_V = NaN(5,ConN);  
VNSA_WaveFlag = NaN(3,ConN); 
VNSA_Acc_waveFront = NaN(3,ConN);
VNSA_Acc_waveBack = NaN(3,ConN);
VNSA_k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）

  %% 启动搜索
for j=1:ConN
    [ VNSA_WaveFlag,VNSA_V,VNSA_Acc_waveFront,VNSA_Acc_waveBack,VNSA_k_waves_OKLast ] = AnalyzeWave...
    ( ConAcc(1:3,:),j,visionFre,VNSA_V,VNSA_WaveFlag,VNSA_k_waves_OKLast,VNSA_Acc_waveFront,VNSA_Acc_waveBack,waveThreshold_VNSAcc );
end

if coder.target('MATLAB')
    DrawPVA( ConPosition_i,ConVelocity,ConAcc,visionFre,k_vision );
    DrawWaveSearchResult( ConAcc,visionFre,VNSA_V,VNSA_WaveFlag,VNSA_Acc_waveFront,VNSA_Acc_waveBack,'VNSAcc',k_vision );
end

%% 找到最长的连续线段
function [ k_vision,i_marker,maxConN ] = FindLongestLine( otherMakers )
N = size(otherMakers,2);
k_vision = 0;
i_marker = 0;
maxConN = 0;
for k=1:N
    OtherMarkersN = otherMakers(k).otherMakersN;
    for i=1:OtherMarkersN
        ConN = k-otherMakers(k).ContinuesLastK(i);
        if ConN > maxConN
           maxConN =  ConN;
           k_vision = k;
           i_marker = i;
        end
    end
    
end

%% 寻找 otherMakers 中第 k_vision 个时刻， 第 i_marker 个马克点 对应的连续线段（独立查找）
% ConPosition_i [3*ConN_i] ConN_i 是该连续线段的长度
function [ ConPosition_i,ConN_i] = GetContinuesPosition( otherMakers,k_vision,i_marker )
ContinuesFlag = otherMakers(k_vision).ContinuesFlag(i_marker) ;
ContinuesLasti_cur = otherMakers(k_vision).ContinuesLasti(i_marker) ;
if isnan(ContinuesFlag) || ContinuesFlag==0 || isnan(ContinuesLasti_cur) % 无2个以上连续马克点
    ConPosition_i = NaN;
    ConN_i = 0;
    return;
end

ContinuesLastK = otherMakers(k_vision).ContinuesLastK(i_marker) ;  % 连续线段最早点的时刻
ConN_i = k_vision-ContinuesLastK+1 ; % 连续线段长度

ConOrder_i = zeros(1,ConN_i);        % 连续线段每个点对应的
ConPosition_i = zeros(3,ConN_i);

ConOrder_i(ConN_i) = i_marker;  % 最后一个是自己
ConPosition_i(:,ConN_i) = otherMakers(k_vision).Position(:,i_marker);
for j=2:ConN_i
    k = k_vision-j+1;
    i = ConN_i-j+1;
    ConOrder_i(i) = otherMakers(k+1).ContinuesLasti(ConOrder_i(i+1));  % k 时刻记录的
    ConPosition_i(:,i) = otherMakers(k).Position(:,ConOrder_i(i));   
    %  last_k = otherMakers(k_vision-j+1).
end

%% 利用 otherMakers(k).ContinuesLasti 将上时刻的连续线段进行重新排序
%% 对象： 速度 加速度 波形参数 等
% ContinuesLasti 当前时刻 所有马克点 对应的 上时刻马克点序号
function otherMakersContinues_New = ReOrderContinues( ContinuesLasti,otherMakersContinues ) 
%% 不需要排序
otherMakersN_Last = otherMakersContinues.otherMakersN;
otherMakersN_new = length(ContinuesLasti);

if   otherMakersN_Last==otherMakersN_new
    if otherMakersN_Last == 0 
        otherMakersContinues_New = otherMakersContinues; % 不需要排序
        return;
    end

    err = sum(ContinuesLasti - (1:otherMakersN_Last));
    if err == 0
        otherMakersContinues_New = otherMakersContinues; % 不需要排序
        return;
    end
end

%% 需要排序
visualN = size(otherMakersContinues.data1,2);
otherMakersContinues_New = Initial_otherMakersContinues( visualN );

otherMakersContinues_New.otherMakersN = otherMakersN_new;

for i=1:otherMakersN_new
    if ~isnan(ContinuesLasti(i)) && ContinuesLasti(i)>0    % otherMakersContinues 有值     
        otherMakersContinues_New.dataN(:,i) = otherMakersContinues.dataN(:,ContinuesLasti(i));
        
        data_i = Read_otherMakersContinues_i( otherMakersContinues,ContinuesLasti(i) );
        otherMakersContinues_New = Write_otherMakersContinues_i( otherMakersContinues_New,i,data_i,0,Inf ); % 最后一个参数无效
    else
        otherMakersContinues_New.dataN(i) = 0;
    end
end

%% 将 otherMakers 的成员补偿完整
function otherMakers = FullotherMakersField( otherMakers )

visualN = size(otherMakers,2);
if ~isfield(otherMakers(1),'frequency')
   frequency =  visualN/(otherMakers(visualN).time - otherMakers(1).time) ;
   otherMakers(1).frequency = frequency;
end
if ~isfield(otherMakers(1),'MarkerSet')
    otherMakers(1).MarkerSet = 6;
end
for k=1:visualN
    otherMakers(k).frequency = otherMakers(1).frequency;
    otherMakers(k).MarkerSet = otherMakers(1).MarkerSet;
    
    otherMakers(k).ContinuesFlag = NaN;
    otherMakers(k).ContinuesLastPosition = NaN(3,1);
    otherMakers(k).ContinuesLastTime = NaN;
    otherMakers(k).ContinuesLastK = NaN;    
    otherMakers(k).CalculatedTime = 0;
    otherMakers(k).ContinuesLasti = NaN;
    
    otherMakers(k).MarkerSet = 16 ; % head
    
end

function Draw_otherMakersContinues( otherMakersContinues,i_marker,visionFre,k_vision_End )
[~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
dataN_i_P = otherMakersContinues.dataN( 1,i_marker );
DrawPVA( ConPosition_i(:,1:dataN_i_P),ConVelocity_i(:,1:dataN_i_P),ConAcc_i(:,1:dataN_i_P),visionFre,k_vision_End );

A_WaveFlag  = AWave( (14:16)-13,1:dataN_i_P);  
A_V = AWave((17:21)-13,1:dataN_i_P); 
A_Acc_waveFront = AWave((22:24)-13,1:dataN_i_P); 
A_Acc_waveBack = AWave((25:27)-13,1:dataN_i_P);   
DrawWaveSearchResult( ConAcc_i(:,1:dataN_i_P),visionFre,A_V,A_WaveFlag,A_Acc_waveFront,A_Acc_waveBack,'VNSAcc',k_vision_End );

function DrawPVA( ConPosition_i,ConVelocity,ConAcc,visionFre,k_vision_End )
global dataFolder

visionN = size(ConPosition_i,2);
time = ((1:visionN)+k_vision_End-visionN) /visionFre;

%% x_PVA
figure('name','x-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(1,:) )
title(get(gcf,'name'))
ylabel('x')

subplot(3,1,2)
plot( time,ConVelocity(1,:) )
ylabel('y')

subplot(3,1,3)
plot( time,ConAcc(1,:) )
ylabel('z')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])


%% y_PVA
figure('name','y-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(2,:) )
title(get(gcf,'name'))
ylabel('x')

subplot(3,1,2)
plot( time,ConVelocity(2,:) )
ylabel('y')

subplot(3,1,3)
plot( time,ConAcc(2,:) )
ylabel('z')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

%% z_PVA
figure('name','z-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(3,:) )
title(get(gcf,'name'))
ylabel('x')

subplot(3,1,2)
plot( time,ConVelocity(3,:) )
ylabel('y')

subplot(3,1,3)
plot( time,ConAcc(3,:) )
ylabel('z')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

%% 初始化 otherMakersContinues
%  otherMakersContinues ：存储当前最新的连续线段，排序与当前的　otherMakers(k).Position　保持一致
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    % (14:16,:)是加速度波形参数
function otherMakersContinues = Initial_otherMakersContinues( visualN )

otherMakersContinues = struct;      % 最多10条，最长10sec，连续曲线
otherMakersContinues.otherMakersN = 0;
otherMakersContinues.dataN = zeros(5,20);

M = 27;
otherMakersContinues.data1 = NaN( M,visualN );
otherMakersContinues.data2 = NaN( M,visualN );
otherMakersContinues.data3 = NaN( M,visualN );
otherMakersContinues.data4 = NaN( M,visualN );
otherMakersContinues.data5 = NaN( M,visualN );
otherMakersContinues.data6 = NaN( M,visualN );
otherMakersContinues.data7 = NaN( M,visualN );
otherMakersContinues.data8 = NaN( M,visualN );
otherMakersContinues.data9 = NaN( M,visualN );
otherMakersContinues.data10 = NaN( M,visualN );
otherMakersContinues.data11 = NaN( M,visualN );
otherMakersContinues.data12 = NaN( M,visualN );
otherMakersContinues.data13 = NaN( M,visualN );
otherMakersContinues.data14 = NaN( M,visualN );
otherMakersContinues.data15 = NaN( M,visualN );
otherMakersContinues.data16 = NaN( M,visualN );
otherMakersContinues.data17 = NaN( M,visualN );
otherMakersContinues.data18 = NaN( M,visualN );
otherMakersContinues.data19 = NaN( M,visualN );
otherMakersContinues.data20 = NaN( M,visualN );

%% 将 第 i 个马克点的连续线段写到 otherMakersContinues
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
        
% dataWrite： 待写入的数据
% dataN_i_j: 第 i 个马克点 的dataWrite 数据的有效长度。dataN(1,i)是当前第i马克点的位置
% dataFlag： dataWrite 的数据类型
function otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i,dataWrite,dataFlag,dataN_i_j )

switch dataFlag
    case 0  % dataWrite  为 data_i
        data_i  = dataWrite;
    case {1,2,3,4} % dataWrite 为 ConPosition_i   
        %  更新 dataN
        otherMakersContinues.dataN(dataFlag,i) = dataN_i_j;
        data_i = Read_otherMakersContinues_i( otherMakersContinues,i );        
        % 更新数据
        switch dataFlag
            case 1
                M1 = 1;        % ConPosition_i
                M2 = 3; 
            case 2
                M1 = 4;        % ConVelocity_i
                M2 = 8; 
            case 3
                M1 = 9;        % ConAcc_i
                M2 = 13; 
            case 4
                M1 = 14;        % ConAccWaveFlag_i
                M2 = 27; 
           	otherwise
                disp('error-1 in Write_otherMakersContinues_i');
                otherMakersContinues = NaN;
                return;
        end
        data_i( M1:M2,1:size(dataWrite,2) ) = dataWrite;  
    otherwise
        disp('error-2 in Write_otherMakersContinues_i');
        otherMakersContinues = NaN;
        return;
end

switch i
    case 1
        otherMakersContinues.data1 = data_i;
    case 2
        otherMakersContinues.data2 = data_i;
    case 3
        otherMakersContinues.data3 = data_i;
	case 4
        otherMakersContinues.data4 = data_i;
    case 5
        otherMakersContinues.data5 = data_i;
    case 6
        otherMakersContinues.data6 = data_i;
	case 7
        otherMakersContinues.data7 = data_i;
    case 8
        otherMakersContinues.data8 = data_i;
    case 9
        otherMakersContinues.data9 = data_i;
	case 10
        otherMakersContinues.data10 = data_i;
    case 11
        otherMakersContinues.data11 = data_i;
    case 12
        otherMakersContinues.data12 = data_i;
    case 13
        otherMakersContinues.data13 = data_i;
	case 14
        otherMakersContinues.data14 = data_i;
    case 15
        otherMakersContinues.data15 = data_i;
    case 16
        otherMakersContinues.data16 = data_i;
	case 17
        otherMakersContinues.data17 = data_i;
    case 18
        otherMakersContinues.data18 = data_i;
    case 19
        otherMakersContinues.data19 = data_i;
	case 20
        otherMakersContinues.data20 = data_i;
    otherwise
        otherMakersContinues.data1 = data_i;
end

%% 读 第i个马克点的 otherMakersContinues 数据
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack

function [data_i,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i )

% if dataN_i==0
%     data_i = [];
% else
    switch i
        case 1
                data_i = otherMakersContinues.data1 ;
        case 2
                data_i = otherMakersContinues.data2 ;
        case 3
                data_i = otherMakersContinues.data3 ;
        case 4
                data_i = otherMakersContinues.data4 ;
        case 5
                data_i = otherMakersContinues.data5 ;
        case 6
                data_i = otherMakersContinues.data6 ;
        case 7
                data_i = otherMakersContinues.data7 ;
        case 8
                data_i = otherMakersContinues.data8 ;
        case 9
                data_i = otherMakersContinues.data9 ;
        case 10
                data_i = otherMakersContinues.data10 ;
        case 11
                data_i = otherMakersContinues.data11 ;
        case 12
                data_i = otherMakersContinues.data12 ;
        case 13
                data_i = otherMakersContinues.data1 ;
        case 14
                data_i = otherMakersContinues.data13 ;
        case 15
                data_i = otherMakersContinues.data14 ;
        case 16
                data_i = otherMakersContinues.data15 ;
        case 17
                data_i = otherMakersContinues.data16 ;
        case 18
                data_i = otherMakersContinues.data17 ;
        case 19
                data_i = otherMakersContinues.data18 ;
        case 20
                data_i = otherMakersContinues.data19 ;
        otherwise 
            disp('error in Write_otherMakersContinues_i');
            data_i = NaN;ConPosition_i = NaN;ConVelocity_i = NaN;ConAcc_i = NaN;
            return;
    end
% end

% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
M = size(data_i,1);    
if isempty(data_i)
    ConPosition_i = [];
    ConVelocity_i = [];
    ConAcc_i = [];
else
    ConPosition_i = data_i(1:3,:);
    if M>=8
        ConVelocity_i = data_i(4:8,:);
    else
        ConVelocity_i=[];
    end
    if M>=13
        ConAcc_i = data_i(9:13,:);
    else
        ConAcc_i=[];
    end
    if M>=16
        AWave = data_i( 14:27,: );
        
%        AWave.A_WaveFlag = data_i(14:16,:); 
%        AWave.A_V = data_i(17:21,:); 
%        AWave.A_Acc_waveFront = data_i(22:24,:); 
%        AWave.A_Acc_waveBack = data_i(25:27,:); 
    else
        AWave = [];
        
%        AWave.A_WaveFlag = []; 
%        AWave.A_V = [];
%        AWave.A_Acc_waveFront = [];
%        AWave.A_Acc_waveBack = [];
    end
end