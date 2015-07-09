
function DrawPVA( ConPosition_i,ConVelocity,ConAcc,visionFre,k_vision_End )
global dataFolder

visionN = size(ConPosition_i,2);
time = ((1:visionN)+k_vision_End-visionN) /visionFre;

%%
figure( 'name','xy-P' )
plot( ConPosition_i(1,:),ConPosition_i(2,:) )
xlabel('x')
ylabel('y')

%% x_PVA
figure('name','x-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(1,:) )
title(get(gcf,'name'))
ylabel('P')

subplot(3,1,2)
plot( time,ConVelocity(1,:) )
ylabel('V')

subplot(3,1,3)
plot( time,ConAcc(1,:) )
ylabel('A')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])


%% y_PVA
figure('name','y-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(2,:) )
title(get(gcf,'name'))
ylabel('P')

subplot(3,1,2)
plot( time,ConVelocity(2,:) )
ylabel('V')

subplot(3,1,3)
plot( time,ConAcc(2,:) )
ylabel('A')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

%% z_PVA
figure('name','z-PVA')
subplot(3,1,1)
plot( time,ConPosition_i(3,:) )
title(get(gcf,'name'))
ylabel('P')

subplot(3,1,2)
plot( time,ConVelocity(3,:) )
ylabel('V')

subplot(3,1,3)
plot( time,ConAcc(3,:) )
ylabel('A')

saveas(gcf,[dataFolder,'\',get(gcf,'name'),'.fig'])

