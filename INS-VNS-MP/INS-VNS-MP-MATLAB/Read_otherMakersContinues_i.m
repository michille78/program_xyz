
%% 读 第i个马克点的 otherMakersContinues 数据
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack

function [data_i,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i...
    ( otherMakersContinues,i )
data_i=[];
ConPosition_i = [];
ConVelocity_i = [];
ConAcc_i = [];
AWave=[];
if isempty(otherMakersContinues)
   return; 
end

% if dataN_i==0
%     data_i = [];
% else
    switch i
        case 1
                data_i = otherMakersContinues.data1 ;
        case 2
                data_i = otherMakersContinues.data2 ;
        case 3
                data_i = otherMakersContinues.data3 ;
        case 4
                data_i = otherMakersContinues.data4 ;
        case 5
                data_i = otherMakersContinues.data5 ;
        case 6
                data_i = otherMakersContinues.data6 ;
        case 7
                data_i = otherMakersContinues.data7 ;
        case 8
                data_i = otherMakersContinues.data8 ;
        case 9
                data_i = otherMakersContinues.data9 ;
        case 10
                data_i = otherMakersContinues.data10 ;
        case 11
                data_i = otherMakersContinues.data11 ;
        case 12
                data_i = otherMakersContinues.data12 ;
        case 13
                data_i = otherMakersContinues.data1 ;
        case 14
                data_i = otherMakersContinues.data13 ;
        case 15
                data_i = otherMakersContinues.data14 ;
        case 16
                data_i = otherMakersContinues.data15 ;
        case 17
                data_i = otherMakersContinues.data16 ;
        case 18
                data_i = otherMakersContinues.data17 ;
        case 19
                data_i = otherMakersContinues.data18 ;
        case 20
                data_i = otherMakersContinues.data19 ;
        otherwise 
            disp('error in Write_otherMakersContinues_i');
            data_i = NaN;ConPosition_i = NaN;ConVelocity_i = NaN;ConAcc_i = NaN;
            return;
    end
% end

% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
M = size(data_i,1);    
if isempty(data_i)
    ConPosition_i = [];
    ConVelocity_i = [];
    ConAcc_i = [];
else
    ConPosition_i = data_i(1:3,:);
    if M>=8
        ConVelocity_i = data_i(4:8,:);
    else
        ConVelocity_i=[];
    end
    if M>=13
        ConAcc_i = data_i(9:13,:);
    else
        ConAcc_i=[];
    end
    if M>=16
        AWave = data_i( 14:27,: );
        
%             A_WaveFlag  = AWave( (14:16)-13,:);  
%             A_V = AWave((17:21)-13,:); 
%             A_Acc_waveFront = AWave((22:24)-13,:); 
%             A_Acc_waveBack = AWave((25:27)-13,:);   
    else
        AWave = [];
        

    end
end