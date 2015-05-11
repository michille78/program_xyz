%% xyz  2015.5.6

function ReadRefData(  )

path = 'E:\data_xyz\AHRS Data\TurntableData_5.4-AllData';

listing = dir( [path,'\Ref_TurntableData*.dat'] ) ;
dataN = length(listing) ;

for list_k = 1:dataN
    pureName=  getPureName( listing(list_k).name ) ;
    
    if ~exist( [path,'\',pureName,'.mat'],'file' )
    
        AHRSRefData_Raw = importdata( [ path,'\',pureName,'.dat' ] );
        RefRotateAngle = ( AHRSRefData_Raw(:,2)-30 ) *pi/180 ;
        RefRotateAngle_Cmd = ( AHRSRefData_Raw(:,1)-30 ) *pi/180 ;
    
    
        save([path,'\',pureName,'.mat'],'RefRotateAngle')

        resPath = [path,'\',pureName,'Fig'];
        if isdir( resPath )
            delete([resPath,'\*'])
        else
            mkdir(resPath) ;
        end
        figure('name',pureName)
        hold on
        plot(RefRotateAngle*180/pi,'r')
        plot(RefRotateAngle_Cmd*180/pi,'b')
        legend( 'refMeasure','refCmd' )
        saveas(gcf,[resPath,'\',get(gcf,'name'),'.fig'])
    end
end




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

