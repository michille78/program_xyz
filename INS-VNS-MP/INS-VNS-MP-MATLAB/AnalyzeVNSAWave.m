
%% 实时 分析马克点的 连续 位置曲线、 速度曲线、 加速度曲线、 加速度曲线的波形
% 只计算一个时刻

function [ otherMakersContinues,A_k_waves_OKLast ] = AnalyzeVNSAWave( otherMakers,k_vision,i_marker,otherMakersContinues,...
    parametersSet,visionFre,A_k_waves_OKLast )

% persistent A_k_waves_OKLast
% if isempty(A_k_waves_OKLast)
%     A_k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）
% end

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
    [ Velocity_k,k_calV ] = CalVelocity( ConPosition_i,dataN_i_P_new,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle,1 ) ;
    if k_calV>0  && ~isnan(Velocity_k(1))
        ConVelocity_i(:,k_calV) = Velocity_k ;
        
        otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,ConVelocity_i,2,k_calV );
        
         %% 计算马克点加速度
        [ acc_k,k_calA ] = CalVelocity( ConVelocity_i(1:3,:),k_calV,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle,1 ) ;
                
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
            otherMakersContinues = Write_otherMakersContinues_i( otherMakersContinues,i_marker,AWave,4,A_k_waves_OKLast );
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
if dataN_i_P_new ~= ConN_i && dataN_i_P_new < size(ConPosition_i,2)
    disp('wrong-2 in GetContinuesPosition_2') 
end

