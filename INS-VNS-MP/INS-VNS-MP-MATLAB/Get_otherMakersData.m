%% xyz 2015.5.26

function   otherMakersTime = Get_otherMakersData( otherMakers )
coder.inline('never');
global CalStartVN CalEndVN  otherMakersTime

N = length(otherMakers);
otherMakersTime = zeros(1,N);
for k = CalStartVN:CalEndVN
    otherMakersTime(k) = otherMakers(k).time;
end