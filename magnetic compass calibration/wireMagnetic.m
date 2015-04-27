%% 导线磁场估计



function wireMagnetic(  )

clc

earthH = 5*1e-5 ;



r = 15*1e-3 ;
I = 40*1e-3 ;
B = CalMagnetic( I,r ) ;
b = B/earthH 

r = 10*1e-3 ;
I = 600*1e-3 ;
B = CalMagnetic( I,r ) ;
b = B/earthH 




function B = CalMagnetic( I,r )

u0 = 4*pi*1e-7 ;

B = u0 * I/(2*pi)/r ;
