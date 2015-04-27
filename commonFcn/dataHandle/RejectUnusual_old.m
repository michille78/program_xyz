% buaa xyz 2013.12.31

% ��̬��������쳣

function new_data = RejectUnusual( old_data,relativeErrorNeed,absErrorNeed )
%% ���ܣ� ��ʸ������(��ά����)ȥ���쳣����
% relativeErrorNeed��������,absErrorNeed���������  �����Ժ����ֻ������һ�����ɣ�
% �ж��д洢�����д洢������Ϊʱ����
ErrorNum = 0 ;  %��ȥ����ʱ����
data_size = size(old_data) ;
%���Ϊ�д洢��תΪ�д洢��һ��һ��ʱ�̣�
if data_size(1)>data_size(2)
    old_data = old_data' ;
    data_size = size(old_data) ;
end
new_data = zeros(data_size) ;
for k = 1:data_size(1)    %�д洢��һ��һ��ʱ�̣�
    data_k = old_data(k,:) ;
    new_data_k = RejectUnusual_Scalar( data_k,relativeErrorNeed,absErrorNeed(k) ) ;
    new_data(k,:) = new_data_k ;
end
% ȥ���쳣���ݣ�����ʱ��ͬ���ԣ�ÿ��ֻҪ��һ���쳣������ȥ��
i = 1;
while i<=length(new_data)
   %һ���м��
   temp = isnan( new_data(:,i) ) ;
   if sum( temp ) ~= 0      %��i�к��� nan
       new_data(:,i) = [] ;     %ȥ����i��
       ErrorNum =ErrorNum +1 ;
   else
       i = i+1 ;
   end
end
disp('��ȥ����ʱ����')
display(ErrorNum)

function  new_data  = RejectUnusual_Scalar( old_data,relativeErrorNeed,absErrorNeed )
%% ���ܣ� �Ա������� ��ע �쳣���ݣ�һά���飨�쳣���עΪ nan��
% �����
    % new_data���µ�����
    % ErrorNum�������쳣���ݵĸ���
ErrorNum = 0 ;
data_size = size(old_data) ;
if min(data_size)>1
   errordlg('��������ݲ���һά(RejectUnusual_Scalar)') 
   return 
end
N_data = length(old_data) ;
new1_data = zeros(1,N_data) ;
new2_data = zeros(1,N_data) ;
data_temp = old_data ;
%% ��һ���쳣ֵ�޳�: ������ 3/100 ����
ErrorRate = 1 ;
MaxErrorRate = min(relativeErrorNeed*5,1) ;     %�������������
cal_n = 1 ;
data_ave = mean(old_data) ;
while ErrorRate>10/100
    %MaxError = MaxErrorRate*( abs(data_ave) ) ;   %����������
    MaxError = max( MaxErrorRate*( abs(data_ave)),absErrorNeed*5 );    %����������
    for i=1:length(old_data)
         error_i = abs(old_data(i)-data_ave) ;   %i�����
        if error_i>MaxError
            new1_data(i) = nan ;
            data_temp(i) = 0 ;
            ErrorNum = ErrorNum+1 ;
        else
            new1_data(i) = old_data(i);
        end   
    end
    ErrorRate = ErrorNum/N_data ;
    
disp('��һ���쳣ֵ����')
display(ErrorNum)
display(ErrorRate)
display(MaxError)
display(cal_n)

    ErrorNum = 0 ;
    MaxErrorRate =MaxErrorRate*(1+0.5) ; %ÿ���޳�����̫��ͽ��������� 50%,��100%��ʼ
    cal_n = cal_n+1 ;
    if cal_n>10
        errordlg('����̫�ӣ��޳��쳣ʧ��')
        return
    end
end

ErrorNum = 0 ;
%% �ڶ����쳣ֵ�޳�: ������ 5% ����
ErrorRate = 1 ;
MaxErrorRate = relativeErrorNeed ;     %�������������
cal_n = 1 ;
data_ave = mean(data_temp) ;     %������ new1_data ���� nan
while ErrorRate>5/100
    %MaxError = MaxErrorRate*( abs(data_ave)+2 )+0.1 ;   %����������
    MaxError = max( MaxErrorRate*( abs(data_ave)),absErrorNeed );    %����������
    for i=1:length(new1_data)
        if ~isnan(new1_data(i))
           	error_i = abs(new1_data(i)-data_ave) ;   %i�����
            if error_i>MaxError
                new2_data(i) = nan ;
                ErrorNum = ErrorNum+1 ;
            else
                 new2_data(i) = new1_data(i) ;
            end   
        else
            new2_data(i) = new1_data(i) ;
        end
    end
    ErrorRate = ErrorNum/N_data ;
    
disp('�ڶ����쳣ֵ����')
display(ErrorNum)
display(ErrorRate)
display(MaxError)
display(cal_n)

    ErrorNum = 0 ;
    MaxErrorRate =MaxErrorRate+5/100 ; %ÿ���޳�����̫��ͽ��������� 5%,��5%��ʼ
	cal_n = cal_n+1 ;
    if cal_n>10
        errordlg('����̫�ӣ��޳��쳣ʧ��')
        return
    end
end
new_data = new2_data ;