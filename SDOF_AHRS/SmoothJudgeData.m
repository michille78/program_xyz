%% 2015.4.23


 %% 平滑
 % JudgeData： [1*N]
 % stepN :平滑步长
% 1001 -> 1111
% 窗口： 前面 stepN 个 后面 stepN/3 个
% 当窗口中：
% 1）首尾minNum个是1， 2）窗口中的1个数大于一半(SmoothRate)。认为这一段均为1
function JudgeData = SmoothJudgeData( JudgeData,stepN,SmoothRate )
Nframes = length(JudgeData) ;
stepN = max(stepN,4);
minNum = fix(stepN/8) ;  % 首尾 minNum 个要求都是1
minNum = max( minNum,1 );
fillNum = 0;

for k=1:Nframes-stepN+1
    end_k = k+stepN-1 ;
    sumHead = sum( JudgeData( k:k+minNum-1 ) );
    sumEnd = sum( JudgeData( end_k-minNum+1:end_k ) );
    if sumHead == minNum && sumEnd == minNum   % 首尾是1
        staticSum = sum( JudgeData( k:end_k ) );
        if staticSum>=stepN*SmoothRate       % 窗口中的1个数大于一半
            JudgeData( k:end_k ) = ones( stepN,1 );
            fillNum = fillNum + stepN - staticSum ;
        end 
    end
end
% display( sprintf('fillNum = %0.0f',fillNum) );



