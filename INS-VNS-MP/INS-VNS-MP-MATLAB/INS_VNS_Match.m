%% xyz 2015.6.30

%% 马克点识别函数
% 1) 往函数内一帧帧输入惯性加速度 和视觉的马克点位置（存在otherMakers）
%  对惯性和视觉独立进行加速度波形分析
    % 每输入一帧惯性加速度，对惯性加速度进行一次波形分析
    % 没输入一帧视觉马克点位置，对其求加速度后进行加速度波形分析
% 每当惯性和视觉加速度波形都更新至少一帧时，进行一帧马克点分类

%% Input （一个时刻的值）
% INS_Joint_N： 惯性节点个数
% INSA_All_k [ INS_Joint_N , 3 ]  : 某一个时刻 INS_Joint_N 个点 的 加速度 
% inertialFre 惯性频率
% ins_k_g  惯性时刻

% otherMakersPosition_k  [3*20] : 最多存 20  个马克点的位置
% otherMakersN_k ： 有效马克点个数
% vns_k_g : 视觉的 序号
% visionFre ： 视觉的频率

% MinMatchDegree = 0.6; % 最小匹配度

%% 数据输入规则
% 1） INSA_All_k [ INS_Joint_N , 3 ] INS_Joint_N 为惯性节点个数，要求始终保持 个数 和 顺序 不变

%% otherMakers
% otherMakers(k).frequency [1]
% otherMakers(k).Position  [3*M]
% otherMakers(k).otherMakersN [1]
%　otherMakers(k).time [1]
%　otherMakers(k).inertial_k [1]
%　otherMakers(k).MarkerSet ""

        % 记录每个马克点的连续特性
% otherMakers(k).trackedMakerPosition  = NaN(3,1);   

% otherMakers(k).ContinuesFlag = zeros(1,M) ; % 不连续
% otherMakers(k).ContinuesLastPosition = NaN(3,M)  ;
% otherMakers(k).ContinuesLastTime = NaN[1*M] ; 
% otherMakers(k).ContinuesLastK = NaN[1*M];


function [ MatchedVNSP,INSA_TestOut,otherMakersContinuesTestOut,vns_kTestOut,VNSWaveResultOut,INSWaveResultOut,otherMakers_kNew ]  = INS_VNS_Match...
    ( INSA_All_k,inertialFre,otherMakersPosition_k,otherMakersN_k,visionFre,MinMatchDegreeIn )
global  MinMatchDegree     
%% 参数设置
MinMatchDegree = MinMatchDegreeIn;
BufferTime = 5; % sec  数据缓存时间长度
IBN  = fix(BufferTime*120);
VBN = fix(BufferTime*120);
Max_otherMakersN = 20;  % 最多存 20  个马克点的位置
IsOnlyLost = 0;     % 1会出问题

INS_Joint_N = 6;

%% 输入预处理
IsINSReceived = ~ isempty( INSA_All_k ) ;
IsVNSReceived = ~ isempty( otherMakersN_k );

%% OUT
INSA_TestOut = [];
vns_kTestOut=[];
otherMakersContinuesTestOut=[];
VNSWaveResultOut=[];
INSWaveResultOut=[];
otherMakers_kNew=[];
%% 时间同步参数
persistent ins_k_g  vns_k_g    % 全局的序号，从第一帧起一直往前增加
if IsINSReceived
    if isempty( ins_k_g )
        ins_k_g = 0;  % 第一帧到来前
    end
    ins_k_g = ins_k_g+1;
end
if IsVNSReceived
    if isempty( vns_k_g )
        vns_k_g = 0;  % 第一帧到来前
    end
    vns_k_g = vns_k_g+1;
end

persistent ins_k  vns_k  %  在缓存区内部的序号

if IsINSReceived
    if isempty( ins_k )
        ins_k = 0;  % 第一帧到来前
    end
end
if IsVNSReceived
    if isempty( vns_k )
        vns_k = 0;  % 第一帧到来前
    end
end

%% 存储惯性加速度
persistent INSA_All   INSWaveResult INSA_WaveFlag_All
% INSA_All [INS_Joint_N * 3 * IBN]  惯性加速度缓存
if IsINSReceived    
    if isempty(INSA_All)
%         INS_Joint_N = size(INSA_All_k,1)/3;  % 惯性节点个数，要求始终保持 个数 和 顺序 不变
%         if mod(INS_Joint_N,1)~=0
%            disp('error INS_Joint_N'); 
%         end
        INSA_All = NaN( INS_Joint_N*3,IBN );  
        INSA_WaveFlag_All = NaN( INS_Joint_N*3,IBN ); 
    end
    [ INSA_All,ins_k,removeN ] = AddListData( INSA_All,ins_k,INSA_All_k(1:INS_Joint_N*3,:) );
    %% 惯性加速度 波形分析
    [INSA_WaveFlag_All,INSA_TestOut] = INSA_Wave_Analyze...
        ( INSA_All, INS_Joint_N,ins_k,IBN,inertialFre,removeN ) ;
    INSA_ValidN = ones( size(INSA_WaveFlag_All,1),1 )*ins_k;
    INSWaveResult = GetWaveResult( INSA_WaveFlag_All,INSA_ValidN,ins_k_g,inertialFre,INSA_All );
    
    
    if coder.target('MATLAB')
        INSWaveResultOut = INSWaveResult;
        INSA_TestOut.validN = ins_k;
        INSA_TestOut.timeN = ins_k_g;
    end
    
    IsLostMark=0;  % 惯性受到数据，视觉没有的情况下，不匹配
end

%% 存储视觉马克点
persistent otherMakers  otherMakers_k_new VNSWaveResult
if IsVNSReceived
    if isempty(otherMakers)
        otherMakers = otherMakersInitial( VBN,visionFre,Max_otherMakersN );  % 频率只读这一次
        otherMakers_k_new = otherMakers(1);
    end
    
    otherMakers_k_new.otherMakersN = double(otherMakersN_k);
    otherMakers_k_new.Position = double(otherMakersPosition_k);
    otherMakers_k_new.time = double(vns_k_g/visionFre) ;       % 当算法对时间同步要求低时，近似通过频率计算时间

    [ otherMakers,vns_k ] = AddListotherMakers( otherMakers,vns_k,otherMakers_k_new );
    
    %% 连续性分析
    if vns_k>1
        [ otherMakers,IsLostMark ] = ContinuesAnalyze( otherMakers,vns_k,INS_Joint_N,visionFre,VBN );
    end
    %% 视觉马克点波形分析
    if vns_k>1 && IsLostMark
        [ otherMakers,VNSA_WaveFlag_All,VNSP_ValidN_All,VNSA_All,otherMakersContinuesTestOut ] = VNSP_AWave_Analyze...
            ( otherMakers,vns_k,vns_k_g,VBN,IsOnlyLost );
        
        VNSWaveResult = GetWaveResult( VNSA_WaveFlag_All,VNSP_ValidN_All,vns_k_g,visionFre,VNSA_All );
        if coder.target('MATLAB')
            VNSWaveResultOut = VNSWaveResult;
            
        end
    end
    vns_kTestOut = vns_k_g;   
    
 %   DrawotherMakersContinuesRealTime( otherMakersContinuesTestOut,otherMakers,vns_k );
end

if isempty(vns_k) || isempty(ins_k) || vns_k<2 || ins_k<2
    MatchedVNSP = NaN(INS_Joint_N*3,1);
    return;
end
%% 所有惯性点 与 所有视觉点 进行匹配

if  IsLostMark
    [ MarkerMatchingINSk,matchedDegreeSyn ] = WaveMatch( INSWaveResult,VNSWaveResult,IsOnlyLost,otherMakers(vns_k).InitialJointK,vns_k_g );
    
else
    MarkerMatchingINSk=[];
end

%%  用 MarkerMatchingINSk 更新  otherMakers(vns_k).InitialJointK
if  IsLostMark
    
    otherMakersN_k = otherMakers(vns_k).otherMakersN;
    for i=1:otherMakersN_k
        MarkerMatchingINSk_i  = MarkerMatchingINSk(i,1);
        if ~isnan(MarkerMatchingINSk_i)   &&  MarkerMatchingINSk(i,2) > MinMatchDegree
            % 该点有匹配结果
            InitialJointK_old = otherMakers(vns_k).InitialJointK(i) ;  % 通过连续性判断的结果
            if isnan( InitialJointK_old )
                % 之前丢失
                otherMakers(vns_k).InitialJointK(i) = MarkerMatchingINSk_i ;
                fprintf('首次识别: mark(%d)-INS(%d)[%0.4f] , ins_k_g=%d t=%0.3f sec \n',i,MarkerMatchingINSk_i,MarkerMatchingINSk(i,2),ins_k_g,ins_k_g/inertialFre );
            else
          %      fprintf('     再识别: mark(%d)-INS(%d)[%0.4f] , ins_k_g=%d t=%0.3f sec \n',i,MarkerMatchingINSk_i,MarkerMatchingINSk(i,2),ins_k_g,ins_k_g/inertialFre );
                % 通过连续性已经得到结果
                if InitialJointK_old~=MarkerMatchingINSk_i
                   fprintf('连续性记录的前时刻判断和当前判断不一致！ \n'); 
                   fprintf( '   mark(%d)，之前对应 INS(%d)，现在对应 INS(%d), vns_k_g=%d \n',i,InitialJointK_old,MarkerMatchingINSk_i,vns_k_g );
                end
            end
        end        
    end     
    
    InitialJointK_k = otherMakers(vns_k).InitialJointK(1:INS_Joint_N) ;
    IsMatchOK =  ~sum(isnan(InitialJointK_k)) ;
    if IsMatchOK 
        disp('全部匹配OK')
    end  
    
    otherMakers_kNew = otherMakers(vns_k);
end
% 绘制匹配成功点的情况
%  dbstop in DrawMatchedResult at 568
%     DrawMatchedResult( otherMakers,otherMakersContinuesTestOut,vns_k,vns_k_g,visionFre,INSA_All,ins_k,ins_k_g,INSA_WaveFlag_All,inertialFre );  
    
%% MatchedVNSP ： 按惯性顺序存储的马克点位置
%  [3*20] 与惯性节点匹配的马克点位置，顺序与 INSA_All_k 一致
MatchedVNSP = NaN(INS_Joint_N*3,1);
if ~isempty(vns_k) && ~isempty(ins_k) && vns_k>1 && ins_k>1
    otherMakersN_k = otherMakers(vns_k).otherMakersN;
    Position_k = otherMakers(vns_k).Position;
    for i=1:otherMakersN_k
        InitialJointK_i = otherMakers(vns_k).InitialJointK(i); % 第 i 个马克点对应的惯性节点序号
        if InitialJointK_i > 0
            MatchedVNSP( InitialJointK_i*3-2:InitialJointK_i*3 ,: ) = Position_k(:,i) ;
        end
    end
end


%% 给加速度矢量 增加角度信息
% Acc [M*3,1]  M 个点，一个时刻
function AccAngle = GetAccAngle( Acc )
[M1,N] = size(Acc);
M = M1/3;
AccAngle = NaN(M*2,N);

for k=1:N
    for i=1:M
        Acc_k = Acc(i*3-2:i*3,k);
        Acc_xyzNorm_k = normest( Acc_k(1:3) );
        if Acc_xyzNorm_k > 0.1
            Acc_xyNorm_k = normest( Acc_k(1:2) );
            % 水平加速度 跟 东向之间的夹角
            temp1 = atan2( Acc_k(1),Acc_k(2) );
            AccAngle(i*2-1,k) = temp1;
            % 3D 加速度与地向的夹角
            temp = atan2( Acc_xyNorm_k,Acc_k(3) );
            AccAngle(i*2,k)  = temp;
        end
    end
end


%%  所有惯性点 与 所有视觉点 进行匹配
% WaveResult 
% M = 3*PointN  点个数的3倍  
    % WaveResult.wave  [ M*100 ] 直接保存波特征点 （最多100个）   每一行存储一个马克点的一维的波形特征值
    % WaveResult.time  [M*100]  每个波特征点对应的时间
    % WaveResult.waveN  [M*1]  波的个数
    % WaveResult.waveReadedN  [M*1]  已度波个数
%% MarkerMatchINSk
% [ otherMakersN,2 ] 
%   MarkerMatchINSk(i_marker,1)为马克点匹配的惯性关节序号，
%   MarkerMatchINSk(i_marker,2)为对应的匹配度

function [ MarkerMatchingINSk,matchedDegreeSyn ] = WaveMatch( INSWaveResult,VNSWaveResult,IsOnlyLost,InitialJointK,vns_k_g )

VNS_M = size( VNSWaveResult.wave,1 );
otherMakersN = VNS_M/3; % 视觉马克点个数

INS_M = size( INSWaveResult.wave,1 );
INS_Joint_N = INS_M/3;  % 惯性关节点个数

matchedDegreeSyn = NaN( otherMakersN,INS_Joint_N );  % 综合匹配度
matchedDegree = NaN( otherMakersN,INS_Joint_N,4 );

%% 计算所有组合的匹配度
for i=1:otherMakersN
    if IsOnlyLost && ~isnan( InitialJointK(i) ) 
       continue;  % 如果这个点没有丢就不分析 
    end
    
    VNS_WaveFetureN = VNSWaveResult.waveN( i*3-2:i*3,: );
    if sum(VNS_WaveFetureN) < 2
       continue; 
    end
    VNS_WaveFeture = VNSWaveResult.wave( i*3-2:i*3,: );    
    VNS_WaveFetureT = VNSWaveResult.time( i*3-2:i*3,: );
    VNS_Acc5D = VNSWaveResult.Acc5D( i*3-2:i*3,:,: );
    
   for j=1:INS_Joint_N        
        INS_WaveFetureN = INSWaveResult.waveN( j*3-2:j*3,: );
        if sum(INS_WaveFetureN) < 2
           continue; 
        end
        INS_WaveFeture = INSWaveResult.wave( j*3-2:j*3,: );
        INS_WaveFetureT = INSWaveResult.time( j*3-2:j*3,: );
        INS_Acc5D = INSWaveResult.Acc5D( j*3-2:j*3,:,: );    
        
       [ matchedDegree_i_j ] =  WaveMatch_OnePoint( INS_WaveFeture,INS_WaveFetureN,INS_WaveFetureT,INS_Acc5D, ...
           VNS_WaveFeture,VNS_WaveFetureN,VNS_WaveFetureT,VNS_Acc5D );
       matchedDegreeSyn(i,j) = matchedDegree_i_j(4);    % 综合匹配度
       matchedDegree(i,j,:) = matchedDegree_i_j;
       
   end
end

%% 由 matchedDegreeSyn 计算每个马克点匹配的惯性关节点 MarkerMatchINSk
MarkerMatchingINSk = ExtractMatchDegree( matchedDegreeSyn,otherMakersN,INS_Joint_N,InitialJointK ) ;

% MarkerMatchingINSk = NaN( otherMakersN,2 ); % MarkerMatchINSk(i_marker,1)为马克点匹配的惯性关节序号，MarkerMatchINSk(i_marker,2)为对应的匹配度
% for i=1:otherMakersN
%     if IsOnlyLost && ~isnan( InitialJointK(i) ) 
%        continue;  % 如果这个点没有丢就不分析 
%     end
%     
%     matchedDegreeSyn_i = matchedDegreeSyn(i,:);
%     [ minMatchedDegree,matchINSk ] = max( matchedDegreeSyn_i,[],2 );
%     if minMatchedDegree>0
%         MarkerMatchingINSk(i,1)  = matchINSk;
%     else
%         MarkerMatchingINSk(i,1)  = NaN;
%     end
%     MarkerMatchingINSk(i,2) = minMatchedDegree;
% end


%% 从匹配度组合中 提取 马克点 和 关节点的匹配关系
%% MarkerMatchINSk
% [ otherMakersN,2 ] 
%   MarkerMatchINSk(i_marker,1)为马克点匹配的惯性关节序号，
%   MarkerMatchINSk(i_marker,2)为对应的匹配度
function MarkerMatchingINSk = ExtractMatchDegree( matchedDegreeSyn,otherMakersN,INS_Joint_N,InitialJointK )
MinContrast = 0.6;  % 有效匹配度 比 第二匹配度 的最小对比度

for i=1:numel(matchedDegreeSyn)
   if isnan( matchedDegreeSyn(i) )
       matchedDegreeSyn(i) = 0;
   end
end

MarkerMatchingINSk = NaN( otherMakersN,2 );
INSJoint_M_VK = NaN(1,INS_Joint_N);  % INSJoint_M_VK(k) 为第k个关节对应的马克点序号
% 首先提取无歧义的点
for i=1:otherMakersN
    matchedDegreeSyn_i = matchedDegreeSyn(i,:); % mark i 对应的所有匹配度
    [ matchedDegreeSyn_i_sorted,I ] = sort(matchedDegreeSyn_i,'descend');
    if matchedDegreeSyn_i_sorted(2) / matchedDegreeSyn_i_sorted(1)  < MinContrast
       % 最大的匹配度 比 第二大的匹配度 大 1 倍以上, 有效
       MarkerMatchingINSk( i,1 ) = I(1);  % 马克i匹配 惯性关节序号 I(1)
       MarkerMatchingINSk( i,2 ) = matchedDegreeSyn_i_sorted(1); % 匹配度
       INSJoint_M_VK(I(1)) = i;
    end
end
% 在提取模糊点
for i=1:otherMakersN
    matchedDegreeSyn_i = matchedDegreeSyn(i,:); % mark i 对应的所有匹配度
    [ matchedDegreeSyn_i_sorted,I ] = sort(matchedDegreeSyn_i,'descend');
    if matchedDegreeSyn_i_sorted(2) / matchedDegreeSyn_i_sorted(1)  > MinContrast
       % 最大的匹配度 与 第二大的匹配度 比较近
       % 排除掉已经被识别的惯性关节后，如果不存在模糊问题，依然有效
       if ~isnan(INSJoint_M_VK(I(1))) || ~isnan(InitialJointK(I(1)))  % 第1个点已被排除（ 此次加速度识别 || 之前连续性排除 ）
           if matchedDegreeSyn_i_sorted(3) / matchedDegreeSyn_i_sorted(2)  < MinContrast
               
                fprintf('三级有效（*0.7）： [%0.2f，%0.2f，%0.2f]，第1个点已被排除 \n',matchedDegreeSyn_i_sorted(1),matchedDegreeSyn_i_sorted(2),matchedDegreeSyn_i_sorted(3));
                MarkerMatchingINSk( i,1 ) = I(2);  % 马克i匹配 惯性关节序号 I(1)
             	MarkerMatchingINSk( i,2 ) = matchedDegreeSyn_i_sorted(2)*0.7; % 匹配度
                INSJoint_M_VK(I(2)) = i;
           end
       end
       
       if ~isnan(INSJoint_M_VK(I(2))) || ~isnan(InitialJointK(I(2))) % 第2个点已被排除
           if matchedDegreeSyn_i_sorted(3) / matchedDegreeSyn_i_sorted(1)  < MinContrast
                MarkerMatchingINSk( i,1 ) = I(1);  % 马克i匹配 惯性关节序号 I(1)
                MarkerMatchingINSk( i,2 ) = matchedDegreeSyn_i_sorted(1)*0.9; % 匹配度
                INSJoint_M_VK(I(1)) = i;
                fprintf('二级有效（*0.9）： [%0.2f，%0.2f，%0.2f]，第2个点已被排除 \n',matchedDegreeSyn_i_sorted(1),matchedDegreeSyn_i_sorted(2),matchedDegreeSyn_i_sorted(3));
           end
       end
       
    end
end

%% 单点 波形特征匹配度计算
% INS_WaveFeture [3*N]  波特征参数
% INS_WaveFetureN　［3*1］ 波个数
% INS_WaveFetureT  [3*1] 波时间

% VNS_WaveFeture [3*N]  波特征参数
% VNS_WaveFetureN　［3*1］ 波个数
% VNS_WaveFetureT  [3*1] 波时间

% matchedDegree = zeros(4,1) ;    % 前三维是 x、y、z 的匹配度，第4维是综合匹配度

function  [ matchedDegree ] =  WaveMatch_OnePoint( INS_WaveFeture,INS_WaveFetureN,INS_WaveFetureT,INS_Acc5D,...
           VNS_WaveFeture,VNS_WaveFetureN,VNS_WaveFetureT,VNS_Acc5D )

matchedDegree = zeros(4,1) ;    % 前三维是 x、y、z 的匹配度，第4维是综合匹配度
waveTimeErr = NaN(3,max(VNS_WaveFetureN));
INS_WaveV = InitialWaveV(3,1);
VNS_WaveV = InitialWaveV(3,1);

for i=1:3    
      [ matchedDegree(i),waveTimeErr(i,1:VNS_WaveFetureN(i,1)),INS_WaveV(i),VNS_WaveV(i) ] = WaveMatch_OneDim...
          ( INS_WaveFeture(i,:),INS_WaveFetureN(i,:),INS_WaveFetureT(i,:),INS_Acc5D(i,:,:),...
              VNS_WaveFeture(i,:),VNS_WaveFetureN(i,1),VNS_WaveFetureT(i,:),VNS_Acc5D(i,:,:) );      
end
matchedDegree(4) = sum( matchedDegree(1:3) );

return;
global  MinMatchDegree 
if coder.target('MATLAB') && matchedDegree(4)>MinMatchDegree
    Name='XYZ';
    for i=1:3 
        DrawMatchedWave( Name(i),INS_WaveFeture(i,:),INS_WaveFetureT(i,:),INS_WaveFetureN(i,:),...
            VNS_WaveFeture(i,:),VNS_WaveFetureT(i,:),VNS_WaveFetureN(i,:),INS_WaveV(i),VNS_WaveV(i),waveTimeErr(i,:) );
    end
end

function WaveV = InitialWaveV( Dim1N,Dim2N )
WaveV(Dim1N,Dim2N).N = 0;
WaveV(Dim1N,Dim2N).Sign = NaN(1,10);   %　第一个点的符号
WaveV(Dim1N,Dim2N).T1 = NaN(1,10);     % 第一段的时间
WaveV(Dim1N,Dim2N).T2 = NaN(1,10);     % 第二段的时间
WaveV(Dim1N,Dim2N).PointT = NaN(3,10); % 4个点的时间（记录整个波）
WaveV(Dim1N,Dim2N).PointK = NaN(3,10);
WaveV(Dim1N,Dim2N).T1Err = NaN(1,10);
WaveV(Dim1N,Dim2N).T2Err = NaN(1,10);

%% 一维 惯性 视觉数据匹配
% INS_WaveFeture [1*N]  波特征参数
% INS_WaveFetureN　［1*1］ 波个数
% INS_WaveFetureT  [1*1] 波时间

% VNS_WaveFeture [1*N]  波特征参数
% VNS_WaveFetureN　［1*1］ 波个数
% VNS_WaveFetureT  [1*1] 波时间

% 匹配特征
% 1）在相近的时间有相同方向的波峰/波谷  （依赖绝对时间）
    %  很相近 matchedDegree = matchedDegree + 0.21;
    %  比较相近 matchedDegree = matchedDegree + 0.101;

% 2) 出现  V 或 倒V 形波 很相似的 （不依赖绝对时间）
    % 很相似（波峰与波谷的时间差很小） matchedDegree = matchedDegree + 0.6002;
    % 比较相似（波峰与波谷的时间差略大） matchedDegree = matchedDegree + 0.30002;


function [ matchedDegree,waveTimeErr,INS_WaveV,VNS_WaveV ] = WaveMatch_OneDim...
    ( INS_WaveFeture,INS_WaveFetureN,INS_WaveFetureT,INS_Acc5D,...
           VNS_WaveFeture,VNS_WaveFetureN,VNS_WaveFetureT,VNS_Acc5D )
 IsUseWaveV = 0;
       
matchedDegree = 0;       

%% 依赖绝对时间的方法
% 两个相邻的波峰出现的时间差一半为 0.4 sec 左右
absTimeErr_Small = 0.1; % sec  很接近的 惯性视觉 视觉 绝对时间差 
absTimeErr_Big = 0.15 ;   % sec  最大忍受的 惯性视觉 视觉 绝对时间差 
MaxAngleErr = 30;

waveTimeErr = NaN(1,VNS_WaveFetureN);

INS_Angle = INS_Acc5D( 1,:,4:5 );
VNS_Angle = VNS_Acc5D( 1,:,4:5 );
         
for i=1:VNS_WaveFetureN
   for j=1:INS_WaveFetureN
      if sign( VNS_WaveFeture(i) ) == sign( INS_WaveFeture(j) )
          if sign( VNS_WaveFeture(i) )==0
                continue;   % 暂时不考虑波腰
          end
         timeErr_i_j =  VNS_WaveFetureT(i) - INS_WaveFetureT(j);
         
         AngleErr1 = abs(INS_Angle(1,j,1)-VNS_Angle(1,i,1))*180/pi;
         AngleErr2 = abs(INS_Angle(1,j,2)-VNS_Angle(1,i,2))*180/pi;
         if AngleErr1>180
            AngleErr1 = AngleErr1-180; 
         end
         if AngleErr2>180
            AngleErr2 = AngleErr2-180; 
         end
         
         if ~isnan(waveTimeErr(i)) && abs(timeErr_i_j) < abs(waveTimeErr(i))             
 %            disp('error 1 WaveMatch_OneDim 同一个视觉点匹配成功了2个惯性点，absTimeErr_Big 设置太大！')
             waveTimeErr(i) = timeErr_i_j;
             waveTimeErr(i) = NaN;
             continue;
         end
         
         if abs(timeErr_i_j) < absTimeErr_Big % 找到一个波形相同，时间比较近的点
             % 判断加速度方向
            if AngleErr1 > MaxAngleErr || AngleErr2 > MaxAngleErr   
               break;
            end
     %       fprintf( '单点 AngleErr = %0.2f \n',AngleErr );
            
             if abs(timeErr_i_j) < absTimeErr_Small % 找到一个波形相同，时间很近的点
                 matchedDegree = matchedDegree + 0.21;   %　波峰／波谷
             else
                 matchedDegree = matchedDegree + 0.101;    %　波峰／波谷
             end
             waveTimeErr(i) = timeErr_i_j;
         end
         
      end
   end
end

%% 不依赖绝对时间的方法
% 一般相邻波峰和波谷出现的时间差为 0.2sec  
reTimeErr_Small = 0.08; % sec  很接近的 相邻波峰和波谷出现的相对时间差 惯性视觉误差
reTimeErr_Big = 0.15;    % sec  最大忍受的 相邻波峰和波谷出现的相对时间差 惯性视觉误差
absTimeErr_Large = 0.7; % 允许较大的绝对时间误差

% 搜索  V 或 倒V 形波    N 或 倒N 形波
[ INS_WaveV,INS_WaveN ] = SearchWave_V_N( INS_WaveFeture,INS_WaveFetureN,INS_WaveFetureT );
[ VNS_WaveV,VNS_WaveN ] = SearchWave_V_N( VNS_WaveFeture,VNS_WaveFetureN,VNS_WaveFetureT );
if IsUseWaveV==0
   return; 
end
for i=1:INS_WaveV.N
   for j=1:VNS_WaveV.N
       if INS_WaveV.Sign(i) == VNS_WaveV.Sign(j)  % 1：都为V  -1：都为倒V
           T1Err_i_j = INS_WaveV.T1(i) - VNS_WaveV.T1(j) ;
           T2Err_i_j = INS_WaveV.T2(i) - VNS_WaveV.T2(j) ;
           EndTErr_i_j = INS_WaveV.PointT(3,i) - VNS_WaveV.PointT(3,j) ;
           
            AngleErr = abs(INS_Angle(1,INS_WaveV.PointK(3,i),:)-VNS_Angle(1,VNS_WaveV.PointK(3,j),:))*180/pi;
           if AngleErr>180
                AngleErr = AngleErr-180; 
            end
           
           if  abs(EndTErr_i_j)<absTimeErr_Large &&  ~isnan(VNS_WaveV.T1Err(j)) && abs(T1Err_i_j) < reTimeErr_Big && abs(T2Err_i_j) < reTimeErr_Big              
  %           disp('error 2 WaveMatch_OneDim 同一个视觉点匹配成功了2个惯性点，reTimeErr_Big 设置太大！')
             VNS_WaveV.T1Err(j) = T1Err_i_j;
             VNS_WaveV.T2Err(j) = T2Err_i_j; 
             continue;
           end
           
           
           if abs(EndTErr_i_j)<absTimeErr_Large &&  abs(T1Err_i_j) < reTimeErr_Big && abs(T2Err_i_j) < reTimeErr_Big 
               % 得到一个比较相近的 V 波形
               if AngleErr(1) > MaxAngleErr || AngleErr(2) > MaxAngleErr                
                   
                   break;
               end
    %           fprintf( 'V Wave AngleErr = %0.2f \n',AngleErr );
               
               if abs(EndTErr_i_j)<absTimeErr_Large && abs(T1Err_i_j) < reTimeErr_Small && abs(T2Err_i_j) < reTimeErr_Small 
                    % 得到一个很相近的 V 波形
                    matchedDegree = matchedDegree + 0.6002;
                    VNS_WaveV.T1Err(j) = T1Err_i_j;
                    VNS_WaveV.T2Err(j) = T2Err_i_j; 
               else
                   matchedDegree = matchedDegree + 0.30002;
                   VNS_WaveV.T1Err(j) = T1Err_i_j;
                   VNS_WaveV.T2Err(j) = T2Err_i_j;  
               end
           end
                               
       end
   end
end

%% 
% matchedDegree = NaN( otherMakersN,INS_Joint_N,4 );
% MarkerMatchingINSk = NaN( otherMakersN,2 ); % MarkerMatchINSk(i_marker,1)为马克点匹配的惯性关节序号，MarkerMatchINSk(i_marker,2)为对应的匹配度
function DrawMatchedResult( otherMakers,otherMakersContinues,vns_k,vns_k_g,visionFre,INSA_All,...
    ins_k,ins_k_g,INSA_WaveFlag_All,inertialFre )
% global INS_WAVE_MATCH

if isempty(otherMakersContinues)
   return; 
end
otherMakersN = otherMakers.otherMakersN;
InitialJointK = otherMakers(vns_k).InitialJointK;
InitialJointK_last = otherMakers(vns_k-1).InitialJointK;
for i=1:otherMakersN
    if ~isnan(InitialJointK(i)) && isnan(InitialJointK_last(i))
        % 刚刚识别出一个点，绘制位置和加速度曲线
        [~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i );
        dataN_P =  otherMakersContinues.dataN( 1,i ) ;
        ConPosition_i = ConPosition_i( 1:3,1:dataN_P );
        ConAcc_i = ConAcc_i( 1:3,1:dataN_P );
        ConVelocity_i  = ConVelocity_i( 1:3,1:dataN_P );
        AWave = AWave( 1:3,1:dataN_P );
        
        vN = size(ConPosition_i,2);
        timeVNS = (vns_k_g-vN+1:vns_k_g)/visionFre ;
        timeINS = (ins_k_g-vN+1 :ins_k_g)/inertialFre ;
        
        dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\7.2 dataB\T2';
        INSPosition = importdata([dataFolder,'\INSPosition.mat']);
        INSPosition_i  =INSPosition( 3*InitialJointK(i)-2:3*InitialJointK(i),ins_k_g-vN+1:ins_k_g );
        figure('name','识别成功点位置-xy')
        plot( ConPosition_i(1,:),ConPosition_i(2,:),'r' )   % 视觉点的位置
        hold on
        plot( INSPosition_i(1,:),INSPosition_i(2,:),'b' )   % 惯性点的位置
        legend('VNS','INS');
        
        figure('name','识别成功点位置-z')
        
        subplot( 3,1,1 )
        plot( timeVNS, ConPosition_i(1,:)+1.1,'r' )   % 视觉点的位置
        hold on
        plot( timeINS, INSPosition_i(1,:),'b' )   % 惯性点的位置
        subplot( 3,1,2 )
        plot( timeVNS, ConPosition_i(2,:)+1.1,'r' )   % 视觉点的位置
        hold on
        plot( timeINS, INSPosition_i(2,:),'b' )   % 惯性点的位置
        subplot( 3,1,3 )
        plot( timeVNS, ConPosition_i(3,:)+1.1,'r' )   % 视觉点的位置
        hold on
        plot( timeINS, INSPosition_i(3,:),'b' )   % 惯性点的位置
        legend('VNS','INS');
        
        figure('name','识别成功点的加速度-VNS')        
        subplot(3,1,1)
        plot( timeVNS, ConAcc_i(1,:)' )
        hold on
        plot( timeVNS, AWave(1,:)','*r' )
        subplot(3,1,2)
        plot( timeVNS, ConAcc_i(2,:)' )
        hold on
        plot( timeVNS, AWave(2,:)','*r' )
        subplot(3,1,3)
        plot( timeVNS, ConAcc_i(3,:)' )
        hold on
        plot( timeVNS, AWave(3,:)','*r' )
        
        figure('name','识别成功点的速度-VNS')        
        subplot(3,1,1)
        plot( timeVNS, ConVelocity_i(1,:)' )
        subplot(3,1,2)
        plot( timeVNS, ConVelocity_i(2,:)' )
        subplot(3,1,3)
        plot( timeVNS, ConVelocity_i(3,:)' )
        
%         INS_WaveFetureT = INS_WAVE_MATCH.INS_WaveFetureT;
%         INS_WaveFetureN = INS_WAVE_MATCH.INS_WaveFetureN;
%         INS_WaveFeture = INS_WAVE_MATCH.INS_WaveFeture;
        
        figure('name','识别成功点的加速度-INS')
        INSA_All_i = INSA_All( InitialJointK(i)*3-2:InitialJointK(i)*3,ins_k-vN+1:ins_k );
        INSA_WaveFlag_All_i = INSA_WaveFlag_All( InitialJointK(i)*3-2:InitialJointK(i)*3,ins_k-vN+1:ins_k );
        
        subplot(3,1,1)
        plot( timeINS, INSA_All_i(1,:)' )
        hold on
        plot( timeINS, INSA_WaveFlag_All_i(1,:)','*r' )   
        subplot(3,1,2)
        plot( timeINS, INSA_All_i(2,:)' )        
        hold on
        plot( timeINS, INSA_WaveFlag_All_i(2,:)','*r' )
        subplot(3,1,3)
        plot( timeINS, INSA_All_i(3,:)' )
        hold on
        plot( timeINS,INSA_WaveFlag_All_i(3,:)','*r' )
    end
end


%% 实时绘制 视觉 位置曲线
function DrawotherMakersContinuesRealTime( otherMakersContinues,otherMakers,vns_k )
IsDrawNon = 0;
if isempty(otherMakersContinues)
   return; 
end
persistent vnsPh vnsAOK_h

if coder.target('MATLAB') 
    if isempty(vnsAOK_h)
        if IsDrawNon
            vnsPh = figure('name','未识别马克点位置');
        end
        vnsAOK_h = figure('name','已识别马克点位置');
    end
    if IsDrawNon
        for k=1:8
            figure(vnsPh)
            subplot( 4,2,k );
            delete(cla)
        end
    end
    
    InitialJointK = otherMakers(vns_k).InitialJointK;
    otherMakersN = otherMakers(vns_k).otherMakersN ;
    InitialJoint_IsOK = zeros(1,6);
    for i_marker=1:otherMakersN
        InitialJointK_i = InitialJointK(i_marker);
        [~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
        ConPosition_i = ConPosition_i( 1:3,1:otherMakersContinues.dataN( 1,i_marker ) );
        ConAcc_i = ConAcc_i( 1:3,1:otherMakersContinues.dataN( 9,i_marker ) );
        AWave = AWave( 1:3,1:otherMakersContinues.dataN( 9,i_marker ) );
        
        if ~isnan(InitialJointK_i)
            %% 识别成功的点
            InitialJoint_IsOK( InitialJointK_i ) = 1;
            figure(vnsAOK_h)
            subplot( 3,2,InitialJointK_i );              
                plot( ConPosition_i(1,:),ConPosition_i(2,:) )
                
        elseif IsDrawNon
            %% 未识别成功的点
            if i_marker>8
               continue; 
            end
            figure(vnsPh)
            subplot( 4,2,i_marker );
                plot( ConPosition_i(1,:),ConPosition_i(2,:) )
        end
    end
    for k=1:6
       if  InitialJoint_IsOK(k)==0  % 这个点丢了
           figure(vnsAOK_h)
           subplot( 3,2,k );   
           delete(cla)
       end
    end
end

%% 一维 波特征绘制
function DrawMatchedWave( Name,INS_WaveFeture,INS_WaveFetureT,INS_WaveFetureN,VNS_WaveFeture,...
    VNS_WaveFetureT,VNS_WaveFetureN,INS_WaveV,VNS_WaveV,waveTimeErr )

% INS_WaveFeture = INS_WAVE.INS_WaveFeture;
% INS_WaveFetureT = INS_WAVE.INS_WaveFetureT;
% INS_WaveFetureN = INS_WAVE.INS_WaveFetureN;
% INS_WaveV = INS_WAVE.INS_WaveV;
% 
% VNS_WaveFeture = VNS_WAVE.VNS_WaveFeture;
% VNS_WaveFetureT = VNS_WAVE.VNS_WaveFetureT;
% VNS_WaveFetureN = VNS_WAVE.VNS_WaveFetureN;
% VNS_WaveV = VNS_WAVE.VNS_WaveV;
% waveTimeErr = VNS_WAVE.waveTimeErr;
 

    T1Err  = VNS_WaveV.T1Err ;
    T2Err = VNS_WaveV.T2Err ;

    figure( 'name',[Name,'-单波形'] )
    plot( INS_WaveFetureT(1:INS_WaveFetureN),INS_WaveFeture(1:INS_WaveFetureN),'*r' )
    hold on
    plot( VNS_WaveFetureT(1:VNS_WaveFetureN),VNS_WaveFeture(1:VNS_WaveFetureN),'*b' )
    for i=1:VNS_WaveFetureN
        if ~isnan(waveTimeErr(i))
            text( VNS_WaveFetureT(i),VNS_WaveFeture(i),num2str(waveTimeErr(i)) ); 
        end
    end
    legend('INS','VNS')
    
    figure( 'name',[Name,'-波形组合'] )
    plot( INS_WaveFetureT(1:INS_WaveFetureN),INS_WaveFeture(1:INS_WaveFetureN),'*r' )
    hold on
    plot( VNS_WaveFetureT(1:VNS_WaveFetureN),VNS_WaveFeture(1:VNS_WaveFetureN),'*b' )
    legend('INS','VNS')
    
    for i=1:INS_WaveV.N
        plot( INS_WaveFetureT(INS_WaveV.PointK(:,i)),INS_WaveFeture(INS_WaveV.PointK(:,i)),'--r' )
    end
    for i=1:VNS_WaveV.N
        plot( VNS_WaveFetureT(VNS_WaveV.PointK(:,i)),VNS_WaveFeture(VNS_WaveV.PointK(:,i)),'--b' )
        if ~isnan( T1Err(i) )
            text( VNS_WaveFetureT(VNS_WaveV.PointK(1,i)),VNS_WaveFeture(VNS_WaveV.PointK(1,i)),num2str(T1Err(i)) );
            text( VNS_WaveFetureT(VNS_WaveV.PointK(3,i)),VNS_WaveFeture(VNS_WaveV.PointK(3,i)),num2str(T2Err(i)) );
      	end
    end



%% 搜索  V 或 倒V 形波    N 或 倒N 形波
function [ WaveV,WaveN ] = SearchWave_V_N( WaveFeture,WaveFetureN,WaveFetureT )


WaveN.N = 0;            %  N 行波的个数
WaveN.Sign = NaN(1,10);   %　第一个点的符号
WaveN.T1 = NaN(1,10);     % 第一段的时间
WaveN.T2 = NaN(1,10);     % 第一段的时间
WaveN.T3 = NaN(1,10);     % 第一段的时间
WaveN.PointT = NaN(4,10); % 3个点的时间（记录整个波）
WaveN.PointK = NaN(4,10);
WaveN.T1Err = NaN(1,10);
WaveN.T2Err = NaN(1,10);

WaveV.N = 0;
WaveV.Sign = NaN(1,10);   %　第一个点的符号
WaveV.T1 = NaN(1,10);     % 第一段的时间
WaveV.T2 = NaN(1,10);     % 第二段的时间
WaveV.PointT = NaN(3,10); % 4个点的时间（记录整个波）
WaveV.PointK = NaN(3,10);
WaveV.T1Err = NaN(1,10);
WaveV.T2Err = NaN(1,10);

k1 = 0; 
for k=1:WaveFetureN-2
   % V 波
   if k<=k1
      continue; % 防止重复 
   end
   k1 = k;
   if WaveFeture(k1)==0  % 跳过波腰
        k1 = k1+1;
   end
   if k1 > WaveFetureN
      break; 
   end
   
   k2 = k1+1;
   if WaveFeture(k2)==0  % 跳过波腰
        k2 = k2+1;
   end
   if k2 > WaveFetureN
      break; 
   end
   
   k3 = k2+1;
   if WaveFeture(k3)==0  % 跳过波腰
        k3 = k3+1;
   end
   if k3 > WaveFetureN
      break; 
   end
   
   k4 = k3+1;
   if k4 <= WaveFetureN &&  WaveFeture(k4)==0  % 跳过波腰
        k4 = k4+1;
   end
   %% 先搜索 N 波
   if k4 <= WaveFetureN 
       if sign( WaveFeture(k1) ) == -sign( WaveFeture(k2) ) == sign( WaveFeture(k3) )  == -sign( WaveFeture(k4) )
        % 判定为  N 或 倒N 形波
        WaveN.N = WaveN.N+1;
        WaveN.Sign(WaveN.N) = sign( WaveFeture(k1) );
        WaveN.T1(WaveN.N) = WaveFetureT(k2)-WaveFetureT(k1) ;     % 第一段的时间
        WaveN.T2(WaveN.N) = WaveFetureT(k3)-WaveFetureT(k2) ;     % 第二段的时间
        WaveN.T3(WaveN.N) = WaveFetureT(k4)-WaveFetureT(k3) ;     % 第三段的时间
        WaveN.PointT(:,WaveN.N) = [ WaveFetureT(k1); WaveFetureT(k2); WaveFetureT(k3); WaveFetureT(k4) ];            
        WaveN.PointK(:,WaveN.N) = [ k1;k2;k3;k4 ];
       end
   end
   %% 再搜索 V 波
   if sign( WaveFeture(k1) ) == -sign( WaveFeture(k2) ) == sign( WaveFeture(k3) )
       % 判定为  V 或 倒V 形波
       WaveV.N = WaveV.N+1;
       WaveV.Sign(WaveV.N) = sign( WaveFeture(k1) );
       WaveV.T1(WaveV.N) = WaveFetureT(k2)-WaveFetureT(k1) ;     % 第一段的时间
       WaveV.T2(WaveV.N) = WaveFetureT(k3)-WaveFetureT(k2) ;     % 第二段的时间
       WaveV.PointT(:,WaveV.N) = [ WaveFetureT(k1); WaveFetureT(k2); WaveFetureT(k3) ];  
       WaveV.PointK(:,WaveV.N) = [ k1;k2;k3 ];
   end
   
end

%% WaveFlag 转为 WaveResult
%　　WaveFlag　［(M)*BN］  M是马克点个数*3 ，BN是数据缓存长度
%  validN ： WaveFlag的有效长度
% k_global ：validN 对应的 从程序运行的第一帧到当前的帧数（不受缓存大小影响）
% WaveResult 
    % WaveResult.wave  [ M*100 ] 直接保存波特征点 （最多100个）   每一行存储一个马克点的一维的波形特征值
    % WaveResult.Angle [ M/3*2,100 ]  (m*2-1,:) 第m个马克点加速度水平面内与东向的夹角，
        % (m*2,:) 第m个马克点3D加速度矢量与地向的夹角
    % WaveResult.time  [M*100]  每个波特征点对应的时间
    % WaveResult.waveN  [M*1]  波的个数
    % WaveResult.waveReadedN  [M*1]  已度波个数
    
function WaveResult = GetWaveResult( WaveFlag,validN,k_global,fre,Acc )

M = size(WaveFlag,1);  % 数据维数： 点的个数*3

WaveResult = struct;
MaxWaveN = 20;   % 缓存的最大波特征个数
wave = NaN(M,MaxWaveN);
time = NaN(M,MaxWaveN);
waveN = zeros(M,1);
waveReadedN = zeros(M,1);
waveFlag_k = NaN(M,MaxWaveN);  % waveN 在 WaveFlag 中的列数
Acc5D = NaN( M,MaxWaveN,5 );  % 波特征点处的 3维加速度和方向

for i=1:M  % 第 i 个数据维度（M/3个马克点，共有M个数据维度）
    for j=1:validN(i)  % 第 j 个时刻
        if ~isnan( WaveFlag(i,j) )
            waveN(i) = waveN(i)+1;
            wave(i,waveN(i)) = WaveFlag(i,j);  
            time(i,waveN(i)) = (k_global+j-validN(i))/fre;  % 从程序运行的第一帧到当前的时间
            waveFlag_k(i,waveN(i)) = j;
            
            m = ceil(i/3);  % 第 m个点
            Acc_k = Acc(m*3-2:m*3,j);            
            AccAngle_k = GetAccAngle( Acc_k );
            Acc5D( i,waveN(i),: ) = [ Acc_k;AccAngle_k ];
        end
    end
end
WaveResult.wave = wave;
WaveResult.time = time;
WaveResult.waveFlag_k = waveFlag_k;
WaveResult.waveN = waveN;
WaveResult.Acc5D = Acc5D;
WaveResult.waveReadedN = waveReadedN;

%% 一个时刻的连续性分析

function [ otherMakers,IsLostMark ] = ContinuesAnalyze( otherMakers,k_vision,INS_Joint_N,visionFre,VBN )

persistent makerTrackThreshold
if isempty(makerTrackThreshold)
    [ makerTrackThreshold,~ ] = SetConstParameters( visionFre ); 
end

trackedMakerPosition = NaN(3,VBN);  % 无跟踪情况下的连续性

otherMakers(k_vision) = PreProcess_otherMakers( otherMakers(k_vision)  );
otherMakers_k_last = otherMakers(k_vision-1);
[ otherMakers(k_vision),dPi_ConJudge ] = ContinuesJudge( otherMakers(k_vision),otherMakers_k_last,...
    trackedMakerPosition,k_vision,makerTrackThreshold );

%% 判断是否有马克点丢失情况
InitialJointK_k = otherMakers(k_vision).InitialJointK(1:INS_Joint_N) ;
if sum(isnan(InitialJointK_k)) == 0
    % 没有马克点丢失，不需要识别
    IsLostMark = 0;
else
    IsLostMark = 1;
end

%% （一个时刻） 输入马克点位置， 输出加速度波形参数
%% WaveResult
%   WaveResult.WaveFlag
function  [ otherMakers,VNSA_WaveFlag_All,VNSP_ValidN_All,VNSA_All,otherMakersContinuesTestOut ] = VNSP_AWave_Analyze...
    ( otherMakers,vns_k,vns_k_g,VBN,IsOnlyLost )

persistent parametersSet  otherMakersContinues  A_k_waves_OKLast_All
visionFre  = otherMakers(vns_k).frequency;
if isempty(parametersSet)
    [ ~,INSVNSCalibSet ] = SetConstParameters( visionFre );    
    waveThreshold_VNSAcc = SetWaveThresholdParameters( 'VNSAcc' );
    parametersSet.waveThreshold_VNSAcc = waveThreshold_VNSAcc;
    parametersSet.INSVNSCalibSet = INSVNSCalibSet;
    otherMakersContinues = Initial_otherMakersContinues( VBN );
    A_k_waves_OKLast_All = zeros( 3*20 ); %  所有马克点（最多20个 ）  % A_k_waves_OKLast_All(:,i_marker)记录上一时刻判断成功的点（防止判断成功的点被覆盖）
end

    % 以上更新了 k_vision 时刻的马克点连续性信息，
    %% 利用 otherMakers(k_vision) 更新 当前的连续曲线 
    % 每个时刻将 otherMakersContinues 按照新的 otherMakers 序号进行排序，并更新顺序更新方法
ContinuesLasti_All = otherMakers(vns_k).ContinuesLasti ;  % 当前时刻 所有马克点 对应的 上时刻马克点序号
%% 先将 otherMakersContinues 按照新的马克点序号 排序
%% 以 otherMakers(k_vision).Position 中的马克点序号为准
[ otherMakersContinues,A_k_waves_OKLast_All ] = ReOrderContinues( ContinuesLasti_All,otherMakersContinues,A_k_waves_OKLast_All ) ;
otherMakersN = otherMakers(vns_k).otherMakersN ;

VNSA_All = NaN(3*otherMakersN,VBN) ;
VNSA_WaveFlag_All = NaN(3*otherMakersN,VBN) ;
VNSP_ValidN_All = zeros(3*otherMakersN,1) ;  

% WaveResult [ MaxotherMakersN*3,1 ]

otherMakersContinuesTestOut=[];

for i_marker=1:otherMakersN
    if IsOnlyLost && ~isnan( otherMakers(vns_k).InitialJointK(i_marker) ) 
       continue;  % 如果这个点没有丢就不分析 
    end

    [ otherMakersContinues,A_k_waves_OKLast_All( :,i_marker ) ] = AnalyzeVNSAWave( otherMakers,vns_k,i_marker,...
        otherMakersContinues,parametersSet,visionFre,A_k_waves_OKLast_All( :,i_marker ) ) ;
    otherMakersContinues.otherMakersN = otherMakers(vns_k).otherMakersN;
    otherMakersContinues.InitialJointK = otherMakers(vns_k).InitialJointK;
    % 根据 otherMakersContinues 更新 WaveResult
    
    [~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
    VNSA_WaveFlag_All( i_marker*3-2:i_marker*3,: )  = AWave( (14:16)-13,:);  
    VNSP_ValidN_All( i_marker*3-2:i_marker*3,1 ) = otherMakersContinues.dataN( 1:3,i_marker );  % 位置的有效长度（与vns_k_g 插值才能得到绝对时间）
    VNSA_All( i_marker*3-2:i_marker*3,: ) = ConAcc_i( 1:3,: );
    
    if coder.target('MATLAB') 
        otherMakersContinuesTestOut = otherMakersContinues;
    end
end



%% 惯性加速度波形分析 （一个时刻）

function [INSA_WaveFlag_All_Out,INSA_TestOut] = INSA_Wave_Analyze...
    ( INSA_All, INS_Joint_N,ins_k,IBN,inertialFre,removeN )
persistent waveThreshold_INSAcc
if isempty(waveThreshold_INSAcc)
    waveThreshold_INSAcc = SetWaveThresholdParameters( 'INSAcc' );
end
%% 所有惯性静态数据
persistent  INSA_WaveFlag_All  INSA_V_All   INSA_waveFront_All  INSA_waveBack_All  INSA_k_waves_OKLast_All
if isempty( INSA_WaveFlag_All )
    INSA_WaveFlag_All = NaN( 3*INS_Joint_N,IBN );
    INSA_V_All = NaN( 5*INS_Joint_N,IBN );
    INSA_waveFront_All = NaN( 3*INS_Joint_N,IBN );
    INSA_waveBack_All = NaN( 3*INS_Joint_N,IBN );
    INSA_k_waves_OKLast_All = zeros(3*INS_Joint_N,1);
end
if removeN>0
    INSA_WaveFlag_All = RemoveLastN( INSA_WaveFlag_All,removeN );
    INSA_V_All = RemoveLastN( INSA_V_All,removeN );
    INSA_waveFront_All = RemoveLastN( INSA_waveFront_All,removeN );
    INSA_waveBack_All = RemoveLastN( INSA_waveBack_All,removeN );
    INSA_k_waves_OKLast_All = RemoveLastN( INSA_k_waves_OKLast_All,removeN );
end
%% 对 INS_Joint_N 个点进行波形分析
for n = 1:INS_Joint_N
    data_WaveFlag = INSA_WaveFlag_All( n*3-2:n*3,: );
    dataV  = INSA_V_All( n*5-4:n*5,: );
    dataA_waveFront = INSA_waveFront_All( n*3-2:n*3,: );
    dataA_waveBack = INSA_waveBack_All( n*3-2:n*3,: );
    k_waves_OKLast = INSA_k_waves_OKLast_All( n*3-2:n*3,: );  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）
    
    [ data_WaveFlag,dataV,dataA_waveFront,dataA_waveBack,k_waves_OKLast ] = AnalyzeWave...
    ( INSA_All( n*3-2:n*3,: ),ins_k,inertialFre,dataV,data_WaveFlag,k_waves_OKLast,dataA_waveFront,dataA_waveBack,waveThreshold_INSAcc );

    INSA_WaveFlag_All( n*3-2:n*3,: ) = data_WaveFlag;
    
    INSA_V_All( n*5-4:n*5,: ) = dataV;
    INSA_waveFront_All( n*3-2:n*3,: ) = dataA_waveFront;
    INSA_waveBack_All( n*3-2:n*3,: ) = dataA_waveBack;
    INSA_k_waves_OKLast_All( n*3-2:n*3,: ) = k_waves_OKLast;
end

if coder.target('MATLAB')
    INSA_TestOut.INSA = INSA_All;
    INSA_TestOut.INSA_WaveFlag_All = INSA_WaveFlag_All;
    INSA_TestOut.INSA_V_All = INSA_V_All ;
    INSA_TestOut.INSA_waveFront_All = INSA_waveFront_All ;
    INSA_TestOut.INSA_waveBack_All = INSA_waveBack_All ;
else
    INSA_TestOut = [];
end
INSA_WaveFlag_All_Out = INSA_WaveFlag_All;

%% 去除最前 removeN 个时刻，后续补NaN
function data = RemoveLastN( data,removeN )
if removeN<1
   return; 
end
dataN = size( data,2 );
valid_N = dataN-removeN;
for i=1:valid_N
    data( :,i ) = data(:,removeN+i);
end

% dataBack = data(:,removeN+1:dataN);
% data( :,1:dataN-removeN )  =dataBack ;

for i=1:removeN
    data( :,valid_N+i ) = NaN ; 
end
% data( :,dataN-removeN+1:dataN ) = NaN ; 




%% 初始化 otherMakersContinues
%  otherMakersContinues ：存储当前最新的连续线段，排序与当前的　otherMakers(k).Position　保持一致
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
% visualN ： 连续曲线的最大长度        
function otherMakersContinues = Initial_otherMakersContinues( visualN )

otherMakersContinues = struct;      % 最多10条，最长10sec，连续曲线
otherMakersContinues.otherMakersN = 0;
otherMakersContinues.dataN = zeros(5,20);

M = 27;
otherMakersContinues.data1 = NaN( M,visualN );  % 第 1 个马克点
otherMakersContinues.data2 = NaN( M,visualN );  % 第 2 个马克点
otherMakersContinues.data3 = NaN( M,visualN );  % 第 3 个马克点
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
