
%function iterative_least_square()

% clear all;
% clc;
% close all;
format long

% magn = load('rawdata.txt');
%magn = load('D:\\Work\\SensorsCalibrater\\Reference\\ellipsoid_fitting\\raw_data_2.txt');
%magn = load('D:\\Work\\MCP_PROTOCOL\\TestCase\\Neuron1\\Neuron\\36_raw_data_calibrate.txt');
%magn = load('D:\\Work\\MCP_PROTOCOL\\TestCase\\Neuron1\\Neuron\\2_raw_data_calibrate.txt');
%magn=load('D:\\Work\\MCP_PROTOCOL\\TestCase\\Calibration\\magn_rawdata1.txt');
%magn=load('D:\\Work\\MCP_PROTOCOL\\Reference\\校准计算\\gsl_linalg_cholesky_decomp精度测试.txt');
%magn=load('D:\\WangBin\\QQ\\705271307\\FileRecv\\36_raw_data_calibrate_new.txt')
magn=load('E:\data_xyz_noitom\magneticData\magnetic_ellipsoid\magn_rawdata1.txt') ;

row = size(magn,1); %列数

x = magn(:,1);
y = magn(:,2);
z = magn(:,3);

D = [ x.^2, y.^2, z.^2, 2*x.*y, 2*x.*z, 2*y.*z, 2*x, 2*y, 2*z, ones(size(x))];  %   size 获取 x的行数与列数；ones(a,b)产生a行b列全1数组 ones(a)产生a行a列全1数组

S = D'*D;   %   D'为D的 转置

C = diag([-1 -1 -1 zeros(1, 7)]);   %   斜对角阵 10*10  zeros用法同ones，只是产生的是全0数组
I1 = [  2, 3, 11,  13,  21, 22];
I2 = [  34, 45, 56];

Fitting_flag = 0;
k_max = 1e5;
% k_max = 4;

C(I1) = k_max/2 - 1;
C(I2) = -k_max;
% [U1,S1,D1]=svd(S);
S
C
[vectors,values]=eig(S, C);%特征根和特征向量
vectors
values
values = diag(values);

% I = find(real(values) > 0 & ~isinf(values));
I = find(isreal(values) & (values > 0) & ~isinf(values));

if length(I) >= 1
    V = real(vectors(:,I));
    V
    V(1)
    V(2)
    V(3)
    V(4)
    V(5)
    V(6)
    V(7)
    V(8)
    V(9)
    V(10)
    [a, b, c, d, e, f, p, q, r ,g] = deal(V(1), V(2), V(3), V(4), V(5), V(6), V(7) ,V(8) ,V(9) ,V(10)); %deal对应赋值
    I1 = a + b + c;
    I2 = a*b + b*c + a*c -d^2 - e^2 - f^2;
    I3 = det([a d e; d b f; e f c]);    %det(A)求特征值
    I4 = det([a d e p; d b f q; e f c r; p q r g]);
    if (I1 ~= 0) && (I2 > 0 ) && (I1*I3 > 0) && (I4 < 0) % ellipsoid?
        Fitting_flag = 1;
    end
end

if Fitting_flag == 0
    % iterative kmax
    k_scale_range_min = 3;
    k_scale_range_max = k_max;
    
    Flag = 0;
    
    while Flag == 0 && abs(k_scale_range_max-k_scale_range_min) > 1e-3
        k_max = (k_scale_range_max + k_scale_range_min)/2;
        C(I1) = k_max/2 - 1;
        C(I2) = -k_max;
        [vectors,values]=eig(S, C);
        values = diag(values);
%         I = find(real(values) > 0 & ~isinf(values));
        I = find(isreal(values) & (values > 0) & ~isinf(values));
        
        % positive eig found？
        if length(I) >= 1
            if length(I) == 1
                V = real(vectors(:,I));
            else
                
            end
            [a, b, c, d, e, f, p, q, r ,g] = deal(V(1), V(2), V(3), V(4), V(5), V(6), V(7) ,V(8) ,V(9) ,V(10));
            I1 = a + b + c;
            I2 = a*b + b*c + a*c -d^2 - e^2 - f^2;
            I3 = det([a d e; d b f; e f c]);
            I4 = det([a d e p; d b f q; e f c r; p q r g]);    
            if (I1 == 0) || (I2 <= 0 ) || (I1*I3 <= 0) || (I4 >= 0) 
                %fitting failed.
                k_scale_range_max = k_max;
                Flag = 1;
            else
                %fitting successfully.
                k_scale_range_min = k_max; 
                Fitting_flag = 1;
                Flag = 1;
            end
        else
            k_scale_range_max = k_max;
        end
    end
end

if Fitting_flag == 0
    errordlg('Fitting failed.');  
    return;
end

A = [V(1) V(4) V(5);V(4) V(2) V(6); V(5) V(6) V(3)];
b = [V(7); V(8); V(9)];

X0 = -inv(A)*b; %inv是求逆矩阵。x‘是矩阵转置（对复数矩阵而言是共轭转置）。
Ae = A / (X0'*A*X0 - V(10));

RRRR=Cholesky(Ae,'upper');
RRRR

AA = [V(1) V(4) V(5) V(7);V(4) V(2) V(6) V(8); V(5) V(6) V(3) V(9);V(7) V(8) V(9) V(10)];
% form the corresponding translation matrix
%   Y = eye(n)：返回n*n单位矩阵；
%   Y = eye(m,n)：返回m*n单位矩阵；
%   Y = eye([m n])：返回m*n单位矩阵；
%   Y = eye(size(A))：返回与A一样大小的单位矩阵；
%   Y = eye：标量1
T = eye( 4 );   %该函数返回单位矩阵
T( 4, 1:3 ) = X0';
% translate to the center
R = T * AA * T';
% solve the eigen problem
[~, evals] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
radii = sqrt( 1 ./ diag( evals ) );

figure;
% draw data
plot3( magn(:,1), magn(:,2), magn(:,3), '*' );
hold on;
grid on;
points = 30;
color = zeros(points+1,points+1);
[xx, yy, zz] = ellipsoid(X0(1),X0(2),X0(3),radii(1),radii(2),radii(3),30);  % x0 是椭球圆心， radii时椭球半径
surf(xx, yy, zz,color);
shading interp
colormap(cool)
max_radii = max(abs(radii(:)));
axis([X0(1)-max_radii, X0(1)+max_radii,X0(2)-max_radii, X0(2)+max_radii,X0(3)-max_radii, X0(3)+max_radii]);    
axis equal;
xlabel('mx');ylabel('my');zlabel('mz');
title('Before calibration');
hold off;

% (X-X0)'*Ae*(X-X0) = 1
% Cholesky decomposition Ae = R'*R;
% ellipsoid fitting error calculation
R = chol(Ae);   %特征值分解，然后特征值对角阵开方，再重新组成矩阵。不过可能会出现复数矩阵，而且不正定的话不保证有解
magn_crc = zeros(row,3);
fitting_error = zeros(row,1);
magn_modulus = zeros(row,1);
magn_crc_modulus = zeros(row,1);

[a, b, c, d, e, f, p, q, r ,g] = deal(V(1), V(2), V(3), V(4), V(5), V(6), V(7) ,V(8) ,V(9) ,V(10));     % 拟合系数
for i=1:row
    magn_crc(i,:)=R*(magn(i,1:3)'-X0);  % 拟合
    mx = magn(i,1);
    my = magn(i,2);
    mz = magn(i,3);    
    fitting_error(i) = a * mx^2 + b * my^2 + c * mz^2 + 2 * d * mx * my + 2 * e * mx * mz + 2 * f * my * mz + 2 * p * mx + 2 * q * my + 2 * r * mz + g;   % 拟合误差
    magn_modulus(i) = sqrt(mx^2+my^2+mz^2);
    magn_crc_modulus(i) = magn_modulus(i) * sqrt(magn_crc(i,1)^2+magn_crc(i,2)^2+magn_crc(i,3)^2);
    
end

figure
plot3( magn_crc(:,1), magn_crc(:,2), magn_crc(:,3), '.r' );
axis equal
grid on    
hold on
points = 30;
color = zeros(points+1,points+1);
[x_sphere, y_sphere, z_sphere] = sphere(points);
surf(x_sphere, y_sphere, z_sphere,color);
%axis([-1.1, 1.1, -1.1, 1.1, -1.1, 1.1]);
axis([-2, 2, -2, 2, -2, 2]);
xlabel('mx');ylabel('my');zlabel('mz');
title('After calibration');
shading interp
colormap(cool)

figure;
plot(fitting_error,'-r');
title('Fitting error');
xlim([0,row]);
ylim([-1,1]);
hold on;

grid on;  

fitting_error_std = std(fitting_error);
fitting_error_mean = mean(fitting_error);

fitting_error_mean_p = zeros(row,1);
fitting_error_std_p = zeros(row,1);
for i=1:row
fitting_error_mean_p(i) = fitting_error_mean; 
fitting_error_std_p(i) = fitting_error_std; 
end

hold on;

mn = plot(fitting_error_mean_p,'-b');
st = plot(fitting_error_std_p,'-g');

set(mn,'LineWidth',2);
set(st,'LineWidth',2);

legend('err','mean','std');

figure;
plot(magn_modulus,'-r');
hold on

plot(magn_crc_modulus,'-b');
title('magn modulus');
grid on; 

k_max;

X0  % 椭球球心

R

radii  % 椭球半径

fitting_error_std

fitting_error_mean




