%% xyz 2015 4.8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intput
% data: [N*const] or [const*N]
% const
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% data: [const*N]

function data_new = Make_Const_N( data,const )

if size(data,1) == const
    data_new = data ;
    return;
end

if size(data,2) == const
    data_new = data';
else
    data_new = NaN;
end
