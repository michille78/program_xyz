function Euler = FQtoEuler_NT( Q,rotateOrder )

PIdiv = 180/pi ;

w = Q(1) ;
x0 = Q(3) ;
y0 = Q(4) ;
z0 = Q(2) ;

switch rotateOrder
    case 'XZY' 
        x = x0 ;
        y = y0 ;
        z = z0 ;
    case 'YXZ'
        x = y0 ;
        y = z0 ;
        z = x0 ;
	case 'XYZ'
        x = x0 ;
        y = -z0 ;
        z = y0 ;  
        case 'YZX'
        x = y0 ;
        y = -y0 ;
        z = x0 ; 
        case 'ZXY'
        x = z0 ;
        y = x0 ;
        z = y0 ; 
        case 'ZYX'
        x = z0 ;
        y = x0 ;
        z = y0 ; 
end

asinValue = 2*(w*y-z*x);
if asinValue>1  
    asinValue = 1;
end
if asinValue<-1
    asinValue = -1;
end

 angX = atan2(2*(w*z+x*y),(1-2*(y*y+z*z)))*PIdiv;		
 angZ = asin(asinValue)*PIdiv;						
 angY = atan2(2*(w*x+y*z),(1-2*(x*x+y*y)))*PIdiv;	

 Euler = [ angX,angZ,angY  ];
 