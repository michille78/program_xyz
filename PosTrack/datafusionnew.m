%FusionData = fopen('d:\pos.txt');
close all;
[time, inerX, inerY, inerZ, optiX, optiZ, optiY] = textread('pos.txt',...
    '%s %f %f %f %f %f %f');

a =50;b = length(inerX)-50;
optical=[optiX,optiY,optiZ];
inertia=[inerX,inerY,inerZ];
%fclose(FusionData);
%先做出原始图
figure
axis equal
plot(optical(a:1:b,1),optical(a:1:b,2),'-b',optical(a,1),optical(a,2),'ob',optical(b,1),optical(b,2),'*b');
hold on
plot(inertia(a:1:b,1),inertia(a:1:b,2),'-r',inertia(a,1),inertia(a,2),'or',inertia(b,1),inertia(b,2),'*r');
hold off
title('origin x-y')
grid on
axis([-5,5,-5,5])

%做高度
figure
plot(optical(a:1:b,3),'-b')%optical(a,3),'ob',optical(b,3),'*b');
hold on
plot(inertia(a:1:b,3),'-r')%inertia(a,3),'or',inertia(b,3),'*r');
hold off
title('origin z')

%将惯性的整体平移（有待商榷）,强行把起始点拉到坐标原点
Translate1=inertia(a,:)
for i=1:1:max(size(inertia))
    inertia(i,:)=inertia(i,:)-Translate1;
end

%将光学的整体平移
Translate=inertia(a,:)-optical(a,:)
for i=1:1:max(size(optical))
    optical(i,:)=optical(i,:)+Translate;
end
%看平移转换后的图像
figure
plot3(optical(a:b,1),optical(a:b,2),optical(a:b,3),'-b',optical(a,1),optical(a,2),optical(a,3),'ob',optical(b,1),optical(b,2),optical(b,3),'*b');
hold on
plot3(inertia(a:b,1),inertia(a:b,2),inertia(a:b,3),'-r',inertia(a,1),inertia(a,2),inertia(a,3),'or',inertia(b,1),inertia(b,2),inertia(b,3),'*r');
hold off 
axis equal
title('Translate x-y-z')
axis([-5,5,-5,5 -5 5])
grid on

%下面进行平面坐标下的转化——转角问题，取最初下蹲到后5s的数
%取前120个点来做校正
for i=1:1:120
    datai(i,:)=inertia(a+(i-1),:);
end
for i=1:1:120
    datao(i,:)=optical(a+(i-1),:);
end
%上面得到想要优化的原始样本，构造Wahba problem的SVD解法
B=zeros(2,2);
for i =1:1:120
    B=B+datai(i,1:2)'*datao(i,1:2);
end
[U,S,V] = svd (B);
M=[1,0;0,det(U)*det(V)];
R=U*M*V'
%得到旋转矩阵R，下面把optical的所有都按照R做变换
for i=1:1:max(size(optical))
    opticalnew(i,1:2)=(R*(optical(i,1:2)'))';
    opticalnew(i,3)=optical(i,3);
end
%验证
figure
axis equal
plot(opticalnew(a:1:b,1),opticalnew(a:1:b,2),'-b',opticalnew(a,1),opticalnew(a,2),'ob',opticalnew(b,1),opticalnew(b,2),'*b');
hold on
plot(inertia(a:1:b,1),inertia(a:b,2),'-r',inertia(a,1),inertia(a,2),'or',inertia(b,1),inertia(b,2),'*r');
hold off
title('sample x-y')
grid on
axis([-5,5,-5,5])
