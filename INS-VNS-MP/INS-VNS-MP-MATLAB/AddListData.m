%% xyz 2015.6.29

%% 先进先出形式 增加数据

% 不允许扩容，超出则将前面的数据挤出去
% data [M*N]
% validN ： data有效数据个数 data(:,1:validN) 有效
% dataAdd ： 新增数据

function [ data,validN,removeN ] = AddListData( data,validN,dataAdd )

[M,dataN] = size(data);
dataAddN = size(dataAdd,2);
if validN > dataN
   fprintf('error (AddListData):validN > dataAddN '); 
end
if size(dataAdd,1) ~= M
   fprintf('error (AddListData):validN > size(dataAdd,1) ~= size(data,1) ');  
end

removeN = (validN+dataAddN)-dataN ;  % 需要从前面挤出去数据个数
if removeN > 0
   % 需要挤掉前面 removeN 个数据
   
   data_BackValid_N = dataN-dataAddN ;
   for i=1:data_BackValid_N
       data(:,i) = data( :, removeN+i );  % 保留后面有效部分
   end
%    data(:,1:data_BackValid_N) = data( :, removeN+1 : validN );  % 保留后面有效部分
   data(:,data_BackValid_N+1:dataN) = dataAdd;
   
   
   validN = dataN;
else
    % 空闲足够
    data( :,validN+1:validN+dataAddN ) = dataAdd;
    validN = validN+dataAddN;
end

