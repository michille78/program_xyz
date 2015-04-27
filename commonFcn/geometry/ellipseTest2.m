

function ellipseTest2(  )


% cx = 0;
% cy = 0;
% a = 60 ;
% b = 30 ;
% thita = -30*pi/180 ;  % rad
% f = sqrt( abs(a^2-b^2) ) ;
% e = f/max(a,b) ;
% 
% eStandIn = [cx;cy;thita;a;b;e;f];
% eStandIn = [eStandIn eStandIn];
% 
% [ A, B, C, D, E, F ] = ellipseStandToPoly(  cx,cy,thita,a,b  ) ;
% ePoly = [ A, B, C, D, E, F ] ;

load ePoly

ePoly2 = ePoly/10;
ePoly3 = ePoly/100;
ePoly4 = ePoly/10000;

[ ePolyNormalized,eStand ] = EPolyNorm_Geometry( ePoly ) ;
berr = IsStandEllipsePoly( ePolyNormalized ) ;
[ ePolyNormalized2,eStand2 ] = EPolyNorm_Geometry( ePoly2 ) ;
berr2 = IsStandEllipsePoly( ePolyNormalized2 ) ;
[ ePolyNormalized3,eStand3 ] = EPolyNorm_Geometry( ePoly3 ) ;
berr3 = IsStandEllipsePoly( ePolyNormalized3 ) ;
[ ePolyNormalized4,eStand4 ] = EPolyNorm_Geometry( ePoly4 ) ;
berr4 = IsStandEllipsePoly( ePolyNormalized4 ) ;

ePolyIn = ePolyNormalized;
ePolyNormalized_err = ( ePolyNormalized-ePolyIn )./ePolyIn ;
ePolyNormalized_err2 = ( ePolyNormalized2-ePolyIn )./ePolyIn ;
ePolyNormalized_err3 = ( ePolyNormalized3-ePolyIn )./ePolyIn ;
ePolyNormalized_err4 = ( ePolyNormalized4-ePolyIn )./ePolyIn ;

eStandIn = eStand ;
eStand_err = ( eStand - eStandIn )./eStandIn ;
eStand_err2 = ( eStand2 - eStandIn )./eStandIn ;
eStand_err3 = ( eStand3 - eStandIn )./eStandIn ;
eStand_err4 = ( eStand4 - eStandIn )./eStandIn ;

H = GenerateEllipseData( ePoly,-80,80,100 ) ;

H2 = GenerateEllipseData( ePoly2,-80,80,100 ) ;

figure('name','ellipse_Poly_fitting')
plot(H(:,1),H(:,2),'*g')
hold on
plot(H2(:,1),H2(:,2),'.r')

legend('1','2')

figure
plot(H(:,1),'*g')

hold on
plot(H2(:,1),'.r')

disp('OK')

