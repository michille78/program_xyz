
format long


addpath('sphereFit')

 test_data = load('magn1.txt');
% test_data = load('D:\\WangBin\\QQ\\705271307\\FileRecv\\1_raw_data_verify.txt');

row = size(test_data,1);

mx = zeros(row,1);
my = zeros(row,1);
mz = zeros(row,1);

fitting_err = zeros(row,1);

X = zeros(row,3);

for i=1:row
    
% mx(i) = test_data(i,2) * 256 + test_data(i,1);
% my(i) = test_data(i,4) * 256 + test_data(i,3);
% mz(i) = test_data(i,6) * 256 + test_data(i,5);
% 
% if mx(i) >= 32768
%     mx(i) = mx(i) - 65536;
% end
% 
% if my(i) >= 32768
%     my(i) = my(i) - 65536;
% end
% 
% if mz(i) >= 32768
%     mz(i) = mz(i) - 65536;
% end

mx(i) = test_data(i,1);
my(i) = test_data(i,2);
mz(i) = test_data(i,3);

X(i,1) = mx(i) * 0.0001;
X(i,2) = my(i) * 0.0001;
X(i,3) = mz(i) * 0.0001;

end

[Center_LSE,Radius_LSE] = sphereFit(X);

for i=1:row
    fitting_err(i) = (X(i,1) - Center_LSE(1))^2 + (X(i,2) - Center_LSE(2))^2 + (X(i,3) - Center_LSE(3))^2 - Radius_LSE ^2;
end

avg = mean(fitting_err)

sd  = std(fitting_err)

figure(1);
plot3(X(:,1),X(:,2),X(:,3),'r.')
grid on; 
% axis([-1 1 -1 1 -1 1]);
axis equal;

figure(2);
plot(fitting_err(:,1),'r-')
grid on;

Center_LSE

Radius_LSE
