%% xyz  2015.7.7

%% 绘制某个时刻 惯性和视觉的所有位置对比图

function CompareINSVNSP(  )

VNST = 1297/120;

dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\7.2 dataB\T2';
% dataFolder =  'E:\data_xyz\Hybrid Motion Capture Data\7.6data\1Person3Points_2';

% dataName = 'GlobalAcc';
dataName0 = 'CalData0';
dataName1 = 'CalData1';
DrawAllPosition( dataFolder,dataName0,dataName1,0,VNST+4,120 );

%% VNS
otherMakers = ReadOptitrack( dataFolder,'\Opt.txt' );

otherMakers = FullotherMakersField( otherMakers );

startK = 1;
otherMakers = CutData( otherMakers,startK );

visualN = size(otherMakers,2);
global visionFre   

%% 参数与输出预设
trackedMakerPosition = NaN(3,visualN);  % 无跟踪情况下的连续性
visionFre = otherMakers(1).frequency;
otherMakersContinues = Initial_otherMakersContinues( visualN );
A_k_waves_OKLast_All = zeros(3,20);

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
    [otherMakersContinues,A_k_waves_OKLast_All] = ReOrderContinues( ContinuesLasti_All,otherMakersContinues,A_k_waves_OKLast_All ) ;
    otherMakersN = otherMakers(k_vision).otherMakersN ;
    for i_marker=1:otherMakersN
        [ otherMakersContinues,A_k_waves_OKLast_All(:,i_marker) ] = AnalyzeVNSAWave...
            ( otherMakers,k_vision,i_marker,otherMakersContinues,parametersSet,visionFre,A_k_waves_OKLast_All(:,i_marker) ) ;
        if k_vision==1297 && i_marker==1
            DrawAllVNSPosition( otherMakersContinues,visionFre,otherMakersN,k_vision );
      %     Draw_otherMakersContinues( otherMakersContinues,i_marker,visionFre,k_vision );
        end
    end

   
end

%% 绘制所有的INS位置

function DrawAllPosition( dataFolder,dataName1,dataName2,StartT,StopT,fre )

[ HeadP1,LeftHandP1,RightHandP1 ] = ReadINSPData( dataFolder,dataName1,1 ) ;
[M,N] = size(HeadP1);
if isempty(dataName2)
    HeadP2 = NaN(M,N);
    LeftHandP2 = NaN(M,N);
    RightHandP2 = NaN(M,N);
else
    [ HeadP2,LeftHandP2,RightHandP2 ] = ReadINSPData( dataFolder,dataName2,1 ) ;    
end

INSPosition = [ HeadP1;LeftHandP1;RightHandP1;HeadP2;LeftHandP2;RightHandP2 ];



 StartK = StartT*fre ;
 if StartK<1     
     StartK=1;
 end
 StopK = StopT*fre ;
 time = (StartK:StopK)/fre;
 INSPosition = INSPosition( :,StartK:StopK );
 
 figure('name','all INS xyP')
 subplot(3,2,1)
 plot( INSPosition(1,:),INSPosition(2,:) )
  subplot(3,2,2)
 plot( INSPosition(4,:),INSPosition(5,:) )
  subplot(3,2,3)
 plot( INSPosition(7,:),INSPosition(8,:) )
  subplot(3,2,4)
 plot( INSPosition(10,:),INSPosition(11,:) )
  subplot(3,2,5)
 plot( INSPosition(13,:),INSPosition(14,:) )
  subplot(3,2,6)
 plot( INSPosition(16,:),INSPosition(17,:) )
 
   figure('name','all INS xP')
 subplot(3,2,1)
 plot( time,INSPosition(1,:) )
  subplot(3,2,2)
 plot( time,INSPosition(4,:) )
  subplot(3,2,3)
 plot( time,INSPosition(7,:) )
  subplot(3,2,4)
 plot( time,INSPosition(10,:) )
  subplot(3,2,5)
 plot( time,INSPosition(13,:) )
  subplot(3,2,6)
 plot( time,INSPosition(15,:) )
 
   figure('name','all INS yP')
 subplot(3,2,1)
 plot( time,INSPosition(2,:) )
  subplot(3,2,2)
 plot( time,INSPosition(5,:) )
  subplot(3,2,3)
 plot( time,INSPosition(8,:) )
  subplot(3,2,4)
 plot( time,INSPosition(11,:) )
  subplot(3,2,5)
 plot( time,INSPosition(14,:) )
  subplot(3,2,6)
 plot( time,INSPosition(17,:) )
 
  figure('name','all INS zP')
 subplot(3,2,1)
 plot( time,INSPosition(3,:) )
  subplot(3,2,2)
 plot( time,INSPosition(6,:) )
  subplot(3,2,3)
 plot( time,INSPosition(9,:) )
  subplot(3,2,4)
 plot( time,INSPosition(12,:) )
  subplot(3,2,5)
 plot( time,INSPosition(15,:) )
  subplot(3,2,6)
 plot( time,INSPosition(18,:) )
 
 %% 绘制所有的INS位置

function DrawAllVNSPosition( otherMakersContinues,visionFre,otherMarkersN,k_vision_End )
ConPosition_All = cell(1,otherMarkersN);
for i_marker=1:otherMarkersN
    [~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
    dataN_i_P = otherMakersContinues.dataN( 1,i_marker );
    ConPosition_All{i_marker} =  ConPosition_i(:,1:dataN_i_P) ;
end
 
 figure('name','all VNS xyP')
 for i_marker=1:min(otherMarkersN,6)
    subplot(3,2,i_marker)
    plot( ConPosition_All{i_marker}(1,:),ConPosition_All{i_marker}(2,:) )
 end
 
   figure('name','all VNS xP')
 for i_marker=1:min(otherMarkersN,6)
    subplot(3,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(1,:) )
 end
 
   figure('name','all VNS yP')
 for i_marker=1:min(otherMarkersN,6)
    subplot(3,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(2,:) )
 end
 
   figure('name','all VNS zP')
 for i_marker=1:min(otherMarkersN,6)
    subplot(3,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(3,:) )
 end