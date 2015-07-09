%% 初始化 otherMakersContinues
%  otherMakersContinues ：存储当前最新的连续线段，排序与当前的　otherMakers(k).Position　保持一致
% otherMakersContinues.data_i [*N]  (1:3,:)是位置，(4:8,:)是速度，(9:13,:)是加速度，
    %  AWave = data_i( 14:27,: ); 
        %  (14:16,:)是加速度波形参数 VNSA_WaveFlag。 (17:21,:) 是VNSA_V，
        % (22:24,:)是VNSA_Acc_waveFront，(25:27,:) 是VNSA_Acc_waveBack
function otherMakersContinues = Initial_otherMakersContinues( visualN )

otherMakersContinues = struct;      % 最多10条，最长10sec，连续曲线
otherMakersContinues.otherMakersN = 0;
M = 27;
otherMakersContinues.dataN = zeros(M,20);  % dataN( m,i_marker ) 表示第 i_marker 个马克点的第 m 行数据 的有效长度


otherMakersContinues.data1 = NaN( M,visualN );
otherMakersContinues.data2 = NaN( M,visualN );
otherMakersContinues.data3 = NaN( M,visualN );
otherMakersContinues.data4 = NaN( M,visualN );
otherMakersContinues.data5 = NaN( M,visualN );
otherMakersContinues.data6 = NaN( M,visualN );
otherMakersContinues.data7 = NaN( M,visualN );
otherMakersContinues.data8 = NaN( M,visualN );
otherMakersContinues.data9 = NaN( M,visualN );
otherMakersContinues.data10 = NaN( M,visualN );
otherMakersContinues.data11 = NaN( M,visualN );
otherMakersContinues.data12 = NaN( M,visualN );
otherMakersContinues.data13 = NaN( M,visualN );
otherMakersContinues.data14 = NaN( M,visualN );
otherMakersContinues.data15 = NaN( M,visualN );
otherMakersContinues.data16 = NaN( M,visualN );
otherMakersContinues.data17 = NaN( M,visualN );
otherMakersContinues.data18 = NaN( M,visualN );
otherMakersContinues.data19 = NaN( M,visualN );
otherMakersContinues.data20 = NaN( M,visualN );