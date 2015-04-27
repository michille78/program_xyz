%% xyz 2015.4.9

% make euler angle in pi*2

function euler = MakeEuler_In2Pi( euler )
for k=1:numel(euler)
    euler(k) = MakeEuler_In2Pi_One( euler(k) ) ;
end

function euler = MakeEuler_In2Pi_One( euler )


if euler <= 2*pi && euler>=-2*pi
    return;
end
if euler > 2*pi
    euler = mod( euler,2*pi );
end
if euler < -2*pi
    euler = mod( euler,-2*pi );
end