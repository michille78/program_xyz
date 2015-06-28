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

%% 计算光学的加速度

function AnalyzeVisualAcc(  )
global dataFolder
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\摔头1';
%　dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head6';
otherMakers = ReadOptitrack( dataFolder,'\Opt.txt' );

otherMakers = FullotherMakersField( otherMakers );

visualN = size(otherMakers,2);
global visionFre  INSVNSCalibSet


trackedMakerPosition = NaN(3,visualN);  % 无跟踪情况下的连续性
visionFre = otherMakers(1).frequency;

[ makerTrackThreshold,INSVNSCalibSet ] = SetConstParameters( visionFre );

otherMakersN1 = otherMakers(1).otherMakersN ;
otherMakers(1).ContinuesFlag = zeros(1,otherMakersN1);

otherMakers(1) = PreProcess_otherMakers( otherMakers(1)  );

for k_vision=2:visualN
    if k_vision==646
       disp('') 
    end
    otherMakers(k_vision) = PreProcess_otherMakers( otherMakers(k_vision)  );
    otherMakers_k_last = otherMakers(k_vision-1);
    [ otherMakers(k_vision),dPi_ConJudge ] = ContinuesJudge( otherMakers(k_vision),otherMakers_k_last,trackedMakerPosition,...
        k_vision,makerTrackThreshold );
%     if k_vision==982
% %         [ k_vision_L,maker_i_L,maxConN ] = FindLongestLine( otherMakers(1:k_vision) );
%         DrawContinues( otherMakers,k_vision,1 );
%     end
   
end

[ k_vision_L,maker_i_L,maxConN ] = FindLongestLine( otherMakers );
DrawContinues( otherMakers,k_vision_L,maker_i_L );



     % 找到最新的连续线段 ConPosition
        
        
        
        
     %   [ Velocity_k,k_calV ] = CalVelocity( ConPosition_i,data_k,fre,dT_CalV,MinXYVNorm_CalAngle ) ;

disp('')

function DrawContinues( otherMakers,k_vision,maker_i )
global visionFre  INSVNSCalibSet
[ ConPosition_i,ConN] = GetContinuesPosition( otherMakers,k_vision,maker_i );
        
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
ConAcc = -ConAcc;
   DrawPVA( ConPosition_i,ConVelocity,ConAcc,visionFre,k_vision );

%% 找到最长的连续线段
function [ k_vision,maker_i,maxConN ] = FindLongestLine( otherMakers )
N = size(otherMakers,2);
k_vision = 0;
maker_i = 0;
maxConN = 0;
for k=1:N
    OtherMarkersN = otherMakers(k).otherMakersN;
    for i=1:OtherMarkersN
        ConN = k-otherMakers(k).ContinuesLastK(i);
        if ConN > maxConN
           maxConN =  ConN;
           k_vision = k;
           maker_i = i;
        end
    end
    
end


%% 寻找 otherMakers 中第 k_vision 个时刻， 第 i_marker 个马克点 对应的连续线段
function [ ConPosition_i,N] = GetContinuesPosition( otherMakers,k_vision,i_marker )
if otherMakers(k_vision).ContinuesFlag(i_marker)==0 || isnan(otherMakers(k_vision).ContinuesFlag(i_marker))
    ConPosition_i = NaN;
    N = 0;
    return;
end
ContinuesLastK = otherMakers(k_vision).ContinuesLastK(i_marker) ;  % 连续线段最早点的时刻
N = k_vision-ContinuesLastK+1 ; % 连续线段长度
ConOrder_i = zeros(1,N);        % 连续线段每个点对应的
ConPosition_i = zeros(3,N);

ConOrder_i(N) = i_marker;  % 最后一个是自己
ConPosition_i(:,N) = otherMakers(k_vision).Position(:,i_marker);
for j=2:N
    k = k_vision-j+1;
    i = N-j+1;
    ConOrder_i(i) = otherMakers(k+1).ContinuesLasti(ConOrder_i(i+1));  % k 时刻记录的
    ConPosition_i(:,i) = otherMakers(k).Position(:,ConOrder_i(i));   
    %  last_k = otherMakers(k_vision-j+1).
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

