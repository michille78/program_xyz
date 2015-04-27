%% xyz 2015.2.19
% calculate the attitude from the accelebrate

function   att_Euler =  CalAttitude_Acc( acc )
N = size(acc,1);
att_Euler = zeros(N,3) ;

for k=1:N
   acc_k =  acc(k,:);
   pitch = acos(-acc_k(2))*180/pi-90 ;
   roll = acos( acc_k(1) )*180/pi-90 ;
   
   att_Euler(k,:) = [ pitch roll 0] ;
end
