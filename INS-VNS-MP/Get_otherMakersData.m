%% xyz 2015.5.26

function  [ otherMakersTime,otherMakersN ] = Get_otherMakersData( otherMakers )

N = length(otherMakers);
otherMakersTime = zeros(1,N);
otherMakersN = zeros(1,N);
for k=1:N
    otherMakersTime(k) = otherMakers(k).time;
    otherMakersN(k) = otherMakers(k).otherMakersN;
end