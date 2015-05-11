%% xyz 2015.4.21

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 1*N

%% 
function ReadAHRSData(  )

%   path = 'E:\data_xyz_noitom\AHRS Data\staticData_4.21_250HZ';
path = 'E:\data_xyz\AHRS Data\TurntableData_5.4-AllData';

listing = dir( [path,'\IMU_TurntableData*.txt'] ) ;
dataN = length(listing) ;

for list_k = 1:dataN
    pureName=  getPureName( listing(list_k).name ) ;
    
    if ~exist( [path,'\',pureName,'.mat'],'file' )
    
        [ quaternion,gyro,acc ] =  readHexRaw( [path,'\',listing(list_k).name] ) ;
        euler = Q2Euler( quaternion,'ZYX',[1,1,1],1 );
        AHRSData.quaternion = quaternion ;
        AHRSData.gyro = gyro ;
        AHRSData.acc = acc ;
        AHRSData.euler = euler ;
        AHRSData.frequency = 250 ;
                
        %%% get the normest
        Nframes = length(gyro);
        gyroNorm = zeros(1,Nframes);
        accNorm = zeros(1,Nframes);
        for k=1:Nframes
            gyroNorm(k) = normest( gyro(:,k) );
            accNorm(k) = normest(acc(:,k));
        end
        AHRSData.gyroNorm = gyroNorm ;
        AHRSData.accNorm = accNorm ;
        Nframes  = length(AHRSData.accNorm);
        AHRSData.Nframes = Nframes ;
                        
        save([path,'\',pureName,'.mat'],'AHRSData')

        resPath = [path,'\',pureName,'Fig'];
        if isdir( resPath )
            delete([resPath,'\*'])
        else
            mkdir(resPath) ;
        end
        figure('name',[pureName,'-euler'])
        subplot(3,1,1)
        plot( euler(1,:)*180/pi )
        ylabel('yaw ^o')
        title(get(gcf,'name'))
        subplot(3,1,2)
        plot( euler(2,:)*180/pi )
        ylabel('pitch ^o')
        subplot(3,1,3)
        plot( euler(3,:)*180/pi )
        ylabel('roll ^o')
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
        
        figure('name',[pureName,'-gyro'])
        subplot(3,1,1)
        plot( gyro(1,:)*180/pi )
        ylabel('wx ^o/s')
        title(get(gcf,'name'))
        subplot(3,1,2)
        plot( gyro(2,:)*180/pi )
        ylabel('wy ^o/s')
        subplot(3,1,3)
        plot( gyro(3,:)*180/pi )
        ylabel('wz ^o/s')
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
        
        figure('name',[pureName,'-gyroNorm'])
        plot( gyroNorm*180/pi )
        ylabel('gyro normest ^o/s')
        title(get(gcf,'name'))        
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
        
        figure('name',[pureName,'-acc'])
        subplot(3,1,1)
        plot( acc(1,:)*1000 )
        ylabel('ax mg')
        title(get(gcf,'name'))
        subplot(3,1,2)
        plot( acc(2,:)*1000 )
        ylabel('ay mg')
        subplot(3,1,3)
        plot( acc(3,:)*1000 )
        ylabel('az mg')
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
        
        figure('name',[pureName,'-accNorm'])
        plot( accNorm*1000 )
        ylabel('acc normest mg')
        title(get(gcf,'name'))        
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
    end
end


disp('OK')


% 3*N
% 4*N

function [ quaternion,gyro,acc ] = readHexRaw( filePath )
fid = fopen(filePath,'r' ) ;
BytesNum = 29;      % 一帧字节数
quaternion = zeros(4,1000) ;  % 
gyro = zeros(3,1000) ;
acc = zeros(3,1000) ;

sprintf('in process of reading Hex Raw data')
k = 0 ;
while ~feof(fid)
    data_str = textscan( fid,'%s',BytesNum ) ;
    if isempty(data_str{1})
       break; 
    end
    if strcmp( [data_str{1}{1},data_str{1}{2}],'FD05') && strcmp( data_str{1}{BytesNum},'FE')
        
        i=8;
        qs_low = data_str{1}{i} ;   i = i+1 ;
        qs_high = data_str{1}{i} ;  i = i+1 ;
        qs_hex = [ qs_high qs_low ];
        qs = SignHexToDec( qs_hex ) ;
        
        qx_low = data_str{1}{i} ;   i = i+1 ;
        qx_high = data_str{1}{i} ;  i = i+1 ;
        qx_hex = [ qx_high qx_low ];
        qx = SignHexToDec( qx_hex ) ;
        
        qy_low = data_str{1}{i} ;   i = i+1 ;
        qy_high = data_str{1}{i} ;  i = i+1 ;
        qy_hex = [ qy_high qy_low ];
        qy = SignHexToDec( qy_hex ) ;
        
        qz_low = data_str{1}{i} ;   i = i+1 ;
        qz_high = data_str{1}{i} ;  i = i+1 ;
        qz_hex = [ qz_high qz_low ];
        qz = SignHexToDec( qz_hex ) ;
        
        wx_low = data_str{1}{i} ;   i = i+1 ;
        wx_high = data_str{1}{i} ;  i = i+1 ;
        wx_hex = [ wx_high wx_low ];
        wx = SignHexToDec( wx_hex ) ;
        
        wy_low = data_str{1}{i} ;   i = i+1 ;
        wy_high = data_str{1}{i} ;  i = i+1 ;
        wy_hex = [ wy_high wy_low ];
        wy = SignHexToDec( wy_hex ) ;
        
        wz_low = data_str{1}{i} ;   i = i+1 ;
        wz_high = data_str{1}{i} ;  i = i+1 ;
        wz_hex = [ wz_high wz_low ];
        wz = SignHexToDec( wz_hex ) ;
        
        
        ax_low = data_str{1}{i} ;   i = i+1 ;
        ax_high = data_str{1}{i} ;  i = i+1 ;
        ax_hex = [ ax_high ax_low ];
        ax = SignHexToDec( ax_hex ) ;
        
        ay_low = data_str{1}{i} ;   i = i+1 ;
        ay_high = data_str{1}{i} ;  i = i+1 ;
        ay_hex = [ ay_high ay_low ];
        ay = SignHexToDec( ay_hex ) ;
        
        az_low = data_str{1}{i} ;   i = i+1 ;
        az_high = data_str{1}{i} ;  i = i+1 ;
        az_hex = [ az_high az_low ];
        az = SignHexToDec( az_hex ) ;
        

        k = k+1 ;
        quaternion(:,k) = [ qs qx qy qz ]';
        gyro(:,k) = [ wx wy wz ]';
        acc(:,k) = [ ax ay az ]';
    else
        
        findFD05 = 0;
        findFD_k = 0 ;
        while findFD05==0 && findFD_k<100
            magn_str = textscan( fid,'%s',1 ) ;
            if isempty(magn_str{1})
               break; 
            end
            if strcmp( magn_str{1}{1},'FD' ) 
                magn_str = textscan( fid,'%s',1 ) ;
                if strcmp( magn_str{1}{1},'05' ) 
                   findFD05 = 1 ;      
                end                            
            end
            findFD_k = findFD_k+1 ;
            display( sprintf('finding FD05: %0.0f',findFD_k) )
        end
        if findFD05==0
            errordlg(sprintf('找不到FD:%0.0f',k))
            break;
        end        
        sprintf('finding FD05 OK: %0.0f',findFD_k)
        magn_str = textscan( fid,'%s',BytesNum-2 ) ;
    end
    if mod(k,500)==0
         display( sprintf('in process of reading Hex Raw data: k = %0.0f',k) )
    end
end
quaternion = quaternion( :,1:k );
gyro = gyro( :,1:k );
acc = acc( :,1:k );

%% transform to stand unit
gyro = gyro*0.1*pi/180 ;    % rad/s
acc = acc*0.333/1000 ;      % g

fclose(fid)

%% name.txt  -> name
function pureName = getPureName( name )
pureName = name ;
k=length(name);
while k<=length(name)
   if ~strcmp(name(k),'.')
      pureName(k) =[];
      k = k-1 ;
   else
       pureName(k) =[];
       break;
   end
end

%% 十六进制原码->十进制 （带符号）
function decSign = SignHexToDec( SignHex )
decUnsign = hex2dec(SignHex) ;
bin = dec2bin(decUnsign,16) ;
N = length(bin) ;
if strcmp(bin(1),'1')    
    bin = bin( 2:N );
    for i=1:length(bin)
       if strcmp(bin(i),'1') 
           bin(i)='0';
       else
           bin(i)='1';
       end
    end
    dec = bin2dec(bin) ;
    dec = dec+1 ;
    decSign = -dec ;
else
    decSign = decUnsign ;
end

