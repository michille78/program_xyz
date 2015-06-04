%% xyz 2015.6.2

%% 读中间数据
% CalData.ROOT_Hips.X   [3*N]  m
%                  .V   [3*N]  m/s
%                  .Q   [4*N]
%                  .A   [3*N]
%                  .W   [3*N]
%  21个关节名见 GetCal21Bone() 

function CalStruct = ReadCalData ( dataFolder,dataName )


%  dataName = 'CalData_Avatar00';
%  dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head2';
CalFilePath = [dataFolder,'\',dataName,'.calc'] ;

if ~exist([dataFolder,'\',dataName,'.mat'],'file')
% if 1
    CalStruct = NaN;
    if ~exist(CalFilePath,'file')
       errordlg(sprintf('不存在 calc 文件 %s',CalFilePath));
       return;
    end
    disp('read calData')
    % dbstop in GetNumberStartLine
    [ CalDataMatrix,HeadLine ] = ReadCal( CalFilePath );
    dataN = length(HeadLine);
    boneN = (dataN-2)/16 ;  %  =21
    if boneN~=21
        errordlg('boneN~=21');
        CalDataMatrix = NaN;
       return; 
    end
    CalDataMatrix = CalDataMatrix' ;
    JointName_Cal = GetCal21Bone();
    for k=1:boneN
        
        X_k1 = 1+(k-1)*16;
        X_k2 = 3+(k-1)*16;
        eval(sprintf('CalStruct.%s.X = CalDataMatrix(%d :%d,: );',JointName_Cal{k},X_k1,X_k2 ));
        
        V_k1 = 4+(k-1)*16;
        V_k2 = 6+(k-1)*16;
        eval(sprintf('CalStruct.%s.V = CalDataMatrix(%d :%d,: );',JointName_Cal{k},V_k1,V_k2 ));
        
        Q_k1 = 7+(k-1)*16;
        Q_k2 = 10+(k-1)*16;
        eval(sprintf('CalStruct.%s.Q = CalDataMatrix(%d :%d,: );',JointName_Cal{k},Q_k1,Q_k2 ));
        
        A_k1 = 11+(k-1)*16;
        A_k2 = 13+(k-1)*16;
        eval(sprintf('CalStruct.%s.A = CalDataMatrix(%d :%d,: );',JointName_Cal{k},A_k1,A_k2 ));
        
        W_k1 = 14+(k-1)*16;
        W_k2 = 16+(k-1)*16;
        eval(sprintf('CalStruct.%s.W = CalDataMatrix(%d :%d,: );',JointName_Cal{k},W_k1,W_k2 ));
        
    end
    save( [dataFolder,'\',dataName,'.mat'],'CalStruct' ) ;
    
else
    CalStruct = importdata( [dataFolder,'\',dataName,'.mat'] );
    disp('没读 calc，直接导入mat结果。 import CalStruct')    
end



%% 第一行数据的行序号
% BVHHeadStr： BVH 头字符串
% numberStartLine： 第一行数字的行序号
function [ CalData,HeadLine ] = ReadCal( CalFilePath )
fid = fopen(CalFilePath,'r' ) ;
line_n = 0 ;
numberStartLine = 0;
CalData = [];
while ~feof(fid) 
    tline = fgetl(fid) ; 
    line_n = line_n+1 ;  
    if ~isempty(tline)
        if numberStartLine==0
            lineData = textscan( tline,'%s' ) ;     
            if strcmp( lineData{1}{1},'01-X-x' )
                numberStartLine = line_n ;
                HeadLine = lineData{1}' ;
            end
        else
            lineData = textscan( tline,'%f' ) ; 
            if numberStartLine>0 && line_n>numberStartLine
               CalData_k = lineData{1}' ;
               CalData = [ CalData ; CalData_k  ];
            end
        end
               
    end
    
end





