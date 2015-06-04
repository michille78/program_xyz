%% xyz 2015 3.14

% read magnetic data of all the magnetic hex data  to dec data
%% 读取十六进制磁力计数据：通过富士康用校准软件 和 串口助手 采集
% 数据已被规范化至模=1000    m_new = （m/360）*10000

function ReadHEXMagneticData(  )

path = 'E:\data_xyz_noitom\magneticData\magnetic data 4.13';

listing = dir( [path,'\*.txt'] ) ;
dataN = length(listing) ;

for list_k = 1:dataN
    pureName=  getPureName( listing(list_k).name ) ;
    
    if ~exist( [path,'\',pureName,'.mat'],'file' )
    
        magnetic_k =  readMagneticHex( [path,'\',listing(list_k).name] ) ;
        save([path,'\',pureName,'.mat'],'magnetic_k')

        resPath = [path,'\',pureName,'Fig'];
        if isdir( resPath )
            delete([resPath,'\*'])
        else
            mkdir(resPath) ;
        end
        figure('name',pureName)
        subplot(3,1,1)
        plot( magnetic_k(:,1) )
        ylabel('mx')
        subplot(3,1,2)
        plot( magnetic_k(:,2) )
        ylabel('my')
        subplot(3,1,3)
        plot( magnetic_k(:,3) )
        ylabel('mz')

        saveas(gcf,[resPath,'\',pureName,'.fig'])
    end
end


disp('OK')




function magnetic = readMagneticHex( filePath )
fid = fopen(filePath,'r' ) ;
magnetic = zeros(10,3) ;  % N*3

k = 0 ;
while ~feof(fid)
    magn_str = textscan( fid,'%s',27 ) ;
    if isempty(magn_str{1})
       break; 
    end
    if strcmp( [magn_str{1}{1},magn_str{1}{2}],'FD04') && strcmp( magn_str{1}{27},'FE')
        mx_low = magn_str{1}{20} ;
        mx_high = magn_str{1}{21} ;
        mx_hex = [mx_high mx_low];
        my_low = magn_str{1}{22} ;
        my_high = magn_str{1}{23} ;
        my_hex = [my_high my_low];    
        mz_low = magn_str{1}{24} ;
        mz_high = magn_str{1}{25} ;
        mz_hex = [mz_high mz_low];  

        mx = hex2dec(mx_hex) ;
        my = hex2dec(my_hex) ;
        mz = hex2dec(mz_hex) ;
        
        mx = SignHexToDec( mx_hex ) ;
        my = SignHexToDec( my_hex ) ;
        mz = SignHexToDec( mz_hex ) ;

        k = k+1 ;
        magnetic(k,:) = [ mx my mz ];
    else
        
        findFD04 = 0;
        findFD_k = 0 ;
        while findFD04==0 && findFD_k<100
            magn_str = textscan( fid,'%s',1 ) ;
            if isempty(magn_str{1})
               break; 
            end
            if strcmp( magn_str{1}{1},'FD' ) 
                magn_str = textscan( fid,'%s',1 ) ;
                if strcmp( magn_str{1}{1},'04' ) 
                   findFD04 = 1 ;      
                end                            
            end
            findFD_k = findFD_k+1 ;
            sprintf('finding FD04: %0.0f',findFD_k)
        end
        if findFD04==0
            errordlg(sprintf('找不到FD:%0.0f',k))
            break;
        end        
        sprintf('finding FD04 OK: %0.0f',findFD_k)
        magn_str = textscan( fid,'%s',27-2 ) ;
    end
    
end
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



