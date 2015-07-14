
%% otherMakersContinues  见 Initial_otherMakersContinues
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
%% 将 第 i_marker 个马克点的连续线段写到 otherMakersContinues
        
% dataWrite： 待写入的数据
% dataN_i_j: [1,1] 或 [3,1]  第 i_marker 个马克点 的dataWrite 数据的有效长度。dataN(1,i_marker)是当前第i马克点的位置
% dataFlag： dataWrite 的数据类型
function otherMakersContinues = Write_otherMakersContinues_i...
    ( otherMakersContinues,i_marker,dataWrite,dataFlag,dataN_i_j )

switch dataFlag
    case 0  % dataWrite  为 data_i
        data_i  = dataWrite;
    case {1,2,3,4} % dataWrite 为 ConPosition_i   
        
        data_i = Read_otherMakersContinues_i( otherMakersContinues,i_marker );        
        % 更新数据
        switch dataFlag
            case 1
                M1 = 1;        % ConPosition_i
                M2 = 3; 
                %  更新 dataN
                otherMakersContinues.dataN(M1:M2,i_marker) = repmat(dataN_i_j,3,1) ;
            case 2
                M1 = 4;        % ConVelocity_i
                M2 = 8; 
                %  更新 dataN
                otherMakersContinues.dataN(M1:M2,i_marker) = repmat(dataN_i_j,5,1) ;
            case 3
                M1 = 9;        % ConAcc_i
                M2 = 13; 
                %  更新 dataN
                otherMakersContinues.dataN(M1:M2,i_marker) = repmat(dataN_i_j,5,1) ;
            case 4
                M1 = 14;        % ConAccWaveFlag_i
                M2 = 27; 
                %  更新 dataN
                otherMakersContinues.dataN(14:16,i_marker) = dataN_i_j ;
                otherMakersContinues.dataN(17:27,i_marker) = repmat(max(dataN_i_j),11,1)  ;
                
           	otherwise
                disp('error-1 in Write_otherMakersContinues_i');
                otherMakersContinues = NaN;
                return;
        end
        
        %  更新 data_i
        data_i( M1:M2,1:size(dataWrite,2) ) = dataWrite;  
    otherwise
        disp('error-2 in Write_otherMakersContinues_i');
        otherMakersContinues = NaN;
        return;
end

switch i_marker
    case 1
        otherMakersContinues.data1 = data_i;
    case 2
        otherMakersContinues.data2 = data_i;
    case 3
        otherMakersContinues.data3 = data_i;
	case 4
        otherMakersContinues.data4 = data_i;
    case 5
        otherMakersContinues.data5 = data_i;
    case 6
        otherMakersContinues.data6 = data_i;
	case 7
        otherMakersContinues.data7 = data_i;
    case 8
        otherMakersContinues.data8 = data_i;
    case 9
        otherMakersContinues.data9 = data_i;
	case 10
        otherMakersContinues.data10 = data_i;
    case 11
        otherMakersContinues.data11 = data_i;
    case 12
        otherMakersContinues.data12 = data_i;
    case 13
        otherMakersContinues.data13 = data_i;
	case 14
        otherMakersContinues.data14 = data_i;
    case 15
        otherMakersContinues.data15 = data_i;
    case 16
        otherMakersContinues.data16 = data_i;
	case 17
        otherMakersContinues.data17 = data_i;
    case 18
        otherMakersContinues.data18 = data_i;
    case 19
        otherMakersContinues.data19 = data_i;
	case 20
        otherMakersContinues.data20 = data_i;
    otherwise
        otherMakersContinues.data1 = data_i;
end