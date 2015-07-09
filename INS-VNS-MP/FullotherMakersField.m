%% 将 otherMakers 的成员补偿完整
function otherMakers = FullotherMakersField( otherMakers )

MaxotherMakersN_k = 10;

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
    otherMakers(k).InitialJointK = NaN(1,MaxotherMakersN_k);  % 对应惯性节点序号
end