%% xyz 2015.4.23

%% 判断某个点k领域内连续为1
% 1）第k-1点为1,第k点为1,第k+1点为1
% 2）第k点前的 frontN 个点有 70% 以上为1
% 3）第k点后的 laterN 个点有 70% 以上为1
%%% INput
% IsOnes: [1*N]或[N*1] 已经判断好的 是否为1的结果
function IsContinuousOnes = JudgeContinuousOnes( IsOnes,frontT,laterT,frequency,minRateFront,minRateLater )
Nframes = length( IsOnes );
IsContinuousOnes = zeros( size(IsOnes) );
frontN = max( fix(frontT * frequency),3) ;
laterN = max( fix(laterT * frequency),1) ;

for k = frontN+1:Nframes-laterN-1
    IsOnes_kSegment = IsOnes( k-frontN:k+laterN ) ;
    IsContinuousOnes(k) = JudgeContinuousOnes_One( IsOnes_kSegment,frontN+1,minRateFront,minRateLater );
end

%% 输入第k时刻邻域的 IsOnes_kSegment（是否为1） 判断 该点是否为连续 =1
% 待判断的点为 IsOnes_kSegment 的第k个点
function IsContinuousOnes_k = JudgeContinuousOnes_One( IsOnes_kSegment,k,minRateFront,minRateLater )

IsContinuousOnes_k = 0;
N = length(IsOnes_kSegment);
if IsOnes_kSegment(k)==1 
    if IsOnes_kSegment(k-1)==1 || IsOnes_kSegment(k+1) == 1  % 前后紧邻至少有一个满足
        IsOnes_Front = IsOnes_kSegment( 1:k-1 );
        IsOnes_Later = IsOnes_kSegment( k+1:N );
        if sum( IsOnes_Front ) >= ceil( minRateFront*(k-1) )       % 前面1的个数满足
           if  sum( IsOnes_Later ) >= fix( minRateLater*(N-k) )   % 后面1的个数满足
               IsContinuousOnes_k = 1 ;               
           end
        end
    end    
end
