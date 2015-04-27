%% xyz 2015.3.26

% unit:rad

function euler = EularScopeLimit( euler,eulerScope )

N = numel(euler);
for k=1:N
    euler(k) = EularScopeLimit_One( euler(k),eulerScope ) ;
end



function euler = EularScopeLimit_One( euler,eulerScope )

switch eulerScope
    case '-pi to pi' 
        if euler-pi > 0 || euler+pi < 0
            if euler-pi > 0
                euler = euler-pi*2 ;
            end
            if euler+pi < 0
                euler = euler+pi*2 ;
            end
        end
end

