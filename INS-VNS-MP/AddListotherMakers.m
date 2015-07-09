%% xyz 2015.6.29

%% 先进先出形式 增加数据

% 不允许扩容，超出则将前面的数据挤出去
% otherMakers [M*N]
% validN ： data有效数据个数 otherMakers(:,1:validN) 有效
% dataAdd ： 新增数据

function [ otherMakers,validN,removeN ] = AddListotherMakers( otherMakers,validN,dataAdd )

[M,dataN] = size(otherMakers);
dataAddN = size(dataAdd,2);
if validN > dataN
   fprintf('error (AddListData):validN > dataAddN '); 
end
if size(dataAdd,1) ~= M
   fprintf('error (AddListData):validN > size(dataAdd,1) ~= size(otherMakers,1) ');  
end

removeN = (validN+dataAddN)-dataN ;  % 需要从前面挤出去数据个数
if removeN > 0
   % 需要挤掉前面 removeN 个数据
   data_BackValid = otherMakers( :, removeN+1 : validN );  % 保留后面有效部分
   for i = 1:size(data_BackValid,2)
        ContinuesLastK_New = data_BackValid(i).ContinuesLastK-removeN;  % 这个参数变了
        for j=1:size(ContinuesLastK_New,2)
            if ContinuesLastK_New(j) <1
               ContinuesLastK_New(j) = 1; % 挤掉了连续的部分（连续长度超过了缓存） 
            end
        end        
        data_BackValid(i).ContinuesLastK = ContinuesLastK_New;
   end
   
   data_BackValid_N = dataN-dataAddN ;
   otherMakers(:,1:data_BackValid_N) = data_BackValid ;
   otherMakers(:,data_BackValid_N+1:dataN) = dataAdd;
   
   validN = dataN;
else
    % 空闲足够
    otherMakers( :,validN+1:validN+dataAddN ) = dataAdd;
    validN = validN+dataAddN;
end

